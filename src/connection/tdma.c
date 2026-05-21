/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include "tdma.h"
#include "esb.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tdma, LOG_LEVEL_INF);

/*
 * Reference-point sync: receiver time projected from the last PONG.
 * tdma_get_time() = rx_ref + (local_now - local_ref).
 * 32-bit wrap in the subtraction is handled correctly by unsigned modular
 * arithmetic; elapsed is bounded by staleness checks (< TIME_SYNC_TIMEOUT_MS).
 */
static volatile uint32_t tdma_rx_ref;
static volatile uint32_t tdma_local_ref;
static volatile bool tdma_time_synced = false;

uint32_t tdma_get_time(void)
{
	if (!tdma_time_synced) {
		return 0;
	}
	uint32_t elapsed = sys_clock_tick_get_32() - tdma_local_ref;
	return tdma_rx_ref + elapsed;
}

void tdma_set_sync_ref(uint32_t rx_ticks, uint32_t local_ticks)
{
	tdma_rx_ref = rx_ticks;
	tdma_local_ref = local_ticks;
	tdma_time_synced = true;
}

void tdma_clear_sync(void)
{
	tdma_rx_ref = 0;
	tdma_local_ref = 0;
	tdma_time_synced = false;
}

void tdma_update_timer_offset(int32_t diff)
{
	(void)diff;
}

/*
 * Dynamic TDMA parameters packed into a single uint64_t and read
 * with a seqlock-style retry loop.  On 32-bit Cortex-M a uint64_t load
 * is two 32-bit operations — an interrupt between them tears the value.
 * tdma_update_config() runs from the ESB event handler ISR, while
 * tdma_wait_for_slot() reads from the connection thread.
 * Layout: [47:40]=slot_ticks, [39:32]=total_slots, [31:16]=frame_ticks,
 *         [15:0]=slot_index<<8 | epoch
 */
static volatile uint64_t tdma_dyn_packed = 0;

static inline void tdma_dyn_write(uint8_t slot_ticks, uint8_t total_slots,
				  uint16_t frame_ticks, uint8_t slot_index, uint8_t epoch)
{
	tdma_dyn_packed = ((uint64_t)slot_ticks << 40)
			| ((uint64_t)total_slots << 32)
			| ((uint64_t)frame_ticks << 16)
			| ((uint64_t)slot_index << 8)
			| (uint64_t)epoch;
}

/* Diagnostic counters for TDMA slot hit/miss tracking */
static uint32_t tdma_slot_hits = 0;
static uint32_t tdma_slot_overshoots = 0;
static uint32_t tdma_grace_hits = 0;
static int64_t tdma_stats_last_ts = 0;

/*
 * Slot targeting offset: aim for a point TDMA_SLOT_TARGET_OFFSET ticks INTO
 * the slot rather than at the boundary.  This provides margin on both sides
 * so that natural jitter (k_sleep() rounding, scheduler latency, EVENT_IRQ
 * delay) doesn't push packets across the slot boundary.
 *
 * With TDMA_SLOT_TICKS=18 and TARGET_OFFSET=4:
 *   - k_sleep wakes at target_offset (4 ticks into slot)
 *   - scheduler latency adds +1-2 ticks → ~5-6 ticks into slot
 *   - Packet reaches receiver at ~6-10 ticks into slot (center of 18)
 *   - 8+ ticks margin before slot end, 4+ ticks after slot start
 */
#define TDMA_SLOT_TARGET_OFFSET 4
#define TDMA_SYNC_STALE_MS (PING_INTERVAL_MS * 10)

static bool tdma_runtime_enabled = false; /* disabled until receiver sends config */

void tdma_init(uint8_t tracker_id)
{
	uint8_t init_slot = tracker_id % TDMA_NUM_TRACKERS;
	tdma_dyn_write(TDMA_SLOT_TICKS, TDMA_NUM_TRACKERS, TDMA_FRAME_TICKS, init_slot, 0);
#if CONFIG_CONNECTION_TDMA
	LOG_INF("TDMA init: slot=%u/%u, frame=%u ticks (~%u.%01u ms), ~%u TPS/tracker",
		init_slot, TDMA_NUM_TRACKERS,
		TDMA_FRAME_TICKS,
		(TDMA_FRAME_TICKS * 1000) / 32768,
		((TDMA_FRAME_TICKS * 10000) / 32768) % 10,
		32768 / TDMA_FRAME_TICKS);
#else
	LOG_INF("TDMA compiled out, all transmissions immediate");
#endif
}

void tdma_wait_for_slot(void)
{
#if !CONFIG_CONNECTION_TDMA
	return;
#else
	if (!tdma_runtime_enabled) {
		return;
	}

	/* Read dynamic parameters with seqlock retry (64-bit is non-atomic on 32-bit) */
	uint64_t packed;
	do {
		packed = tdma_dyn_packed;
	} while (packed != tdma_dyn_packed);
	uint8_t slot_ticks = (uint8_t)(packed >> 40);
	uint16_t frame_ticks = (uint16_t)(packed >> 16);
	uint8_t slot_index = (uint8_t)(packed >> 8);

	if (frame_ticks == 0 || slot_ticks == 0) {
		return; /* invalid config — transmit immediately */
	}

	/*
	 * Skip TDMA gating when time sync is stale.
	 */
	int64_t sync_age = esb_get_sync_age_ms();
	if (sync_age < 0 || sync_age > TDMA_SYNC_STALE_MS) {
		return; /* sync lost or too stale — transmit immediately */
	}

	uint32_t now = tdma_get_time();
	if (now == 0) {
		return; /* not synced — fallback to immediate TX */
	}

	uint32_t slot_start = (uint32_t)slot_index * slot_ticks;
	uint32_t frame_phase = now % frame_ticks;

	/* Check if we're already inside our slot (between start and end) */
	int32_t pos_in_slot = (int32_t)frame_phase - (int32_t)slot_start;
	/* Normalize to [-FRAME/2, FRAME/2] for wrap-around */
	if (pos_in_slot > (int32_t)(frame_ticks / 2)) {
		pos_in_slot -= frame_ticks;
	} else if (pos_in_slot < -(int32_t)(frame_ticks / 2)) {
		pos_in_slot += frame_ticks;
	}

	/*
	 * If we're anywhere inside [0, slot_ticks + grace), TX now.
	 */
	if (pos_in_slot >= 0 &&
	    pos_in_slot < (int32_t)(slot_ticks + TDMA_OVERSHOOT_GRACE)) {
		if (pos_in_slot >= (int32_t)slot_ticks) {
			tdma_grace_hits++;
			LOG_DBG("TDMA grace hit: pos_in_slot=%d", pos_in_slot);
		} else {
			tdma_slot_hits++;
		}
		return; /* in slot (or minor overshoot) — start TX now */
	}

	/*
	 * Compute ticks to sleep until slot_start + TARGET_OFFSET.
	 */
	uint32_t target_phase = slot_start + TDMA_SLOT_TARGET_OFFSET;
	if (target_phase >= frame_ticks) {
		target_phase -= frame_ticks;
	}
	uint32_t ticks_to_target;
	if (target_phase >= frame_phase) {
		ticks_to_target = target_phase - frame_phase;
	} else {
		ticks_to_target = frame_ticks - frame_phase + target_phase;
	}

	if (ticks_to_target > 0 && ticks_to_target <= frame_ticks) {
		k_sleep(K_TICKS(ticks_to_target));
	}

	/* Re-read time after sleep to classify landing accuracy.
	 * Regardless of where we land, always proceed to TX. */
	now = tdma_get_time();
	if (now == 0) {
		return;
	}
	frame_phase = now % frame_ticks;
	slot_start = (uint32_t)slot_index * slot_ticks;
	pos_in_slot = (int32_t)frame_phase - (int32_t)slot_start;
	if (pos_in_slot > (int32_t)(frame_ticks / 2)) {
		pos_in_slot -= frame_ticks;
	} else if (pos_in_slot < -(int32_t)(frame_ticks / 2)) {
		pos_in_slot += frame_ticks;
	}

	if (pos_in_slot >= 0 && pos_in_slot < (int32_t)(slot_ticks + TDMA_OVERSHOOT_GRACE)) {
		/* Landed in slot or within grace window — ideal */
		if (pos_in_slot >= (int32_t)slot_ticks) {
			tdma_grace_hits++;
		} else {
			tdma_slot_hits++;
		}
	} else {
		/* Overshot significantly — TX anyway to prevent cascade */
		tdma_slot_overshoots++;
		LOG_WRN("TDMA overshoot: pos=%d target=%u slept=%u (slot 0-%d)",
			pos_in_slot, ticks_to_target,
			(uint32_t)(frame_phase >= target_phase ?
				frame_phase - target_phase :
				frame_phase + frame_ticks - target_phase),
			(int)(slot_ticks - 1));
	}

	/* Periodic TDMA statistics */
	int64_t stat_now = k_uptime_get();
	if (tdma_stats_last_ts == 0) {
		tdma_stats_last_ts = stat_now;
	}
	if (stat_now - tdma_stats_last_ts >= 10000) {
		uint32_t total = tdma_slot_hits + tdma_grace_hits + tdma_slot_overshoots;
		if (total > 0) {
			LOG_INF("TDMA stats (10s): hits=%u grace=%u overshoot=%u total=%u",
				tdma_slot_hits, tdma_grace_hits,
				tdma_slot_overshoots, total);
		}
		tdma_slot_hits = 0;
		tdma_grace_hits = 0;
		tdma_slot_overshoots = 0;
		tdma_stats_last_ts = stat_now;
	}
#endif
}

void tdma_set_enabled(bool enabled)
{
#if CONFIG_CONNECTION_TDMA
	if (tdma_runtime_enabled != enabled) {
		tdma_runtime_enabled = enabled;
		LOG_INF("TDMA %s at runtime", enabled ? "enabled" : "disabled");
	}
#else
	ARG_UNUSED(enabled);
	LOG_WRN("TDMA not compiled in, cannot change at runtime");
#endif
}

bool tdma_is_enabled(void)
{
#if CONFIG_CONNECTION_TDMA
	return tdma_runtime_enabled;
#else
	return false;
#endif
}

void tdma_update_config(uint8_t slot_index, uint8_t total_slots, uint8_t slot_ticks, uint8_t epoch)
{
#if CONFIG_CONNECTION_TDMA
	if (total_slots == 0 || slot_ticks == 0) {
		return;
	}

	if (slot_index >= total_slots) {
		LOG_WRN("TDMA: Invalid slot_index=%u (>= total_slots=%u), ignoring config update",
			slot_index, total_slots);
		return;
	}

	uint16_t frame_ticks = (uint16_t)slot_ticks * total_slots;
	tdma_dyn_write(slot_ticks, total_slots, frame_ticks, slot_index, epoch);

	if (!tdma_runtime_enabled) {
		tdma_runtime_enabled = true;
		LOG_INF("TDMA auto-enabled by receiver config");
	}

	LOG_INF("TDMA config: slot=%u/%u, slot_ticks=%u, frame=%u ticks, ~%u TPS (epoch=%u)",
		slot_index, total_slots, slot_ticks,
		frame_ticks,
		frame_ticks > 0 ? 32768 / frame_ticks : 0,
		epoch);
#else
	ARG_UNUSED(slot_index);
	ARG_UNUSED(total_slots);
	ARG_UNUSED(slot_ticks);
	ARG_UNUSED(epoch);
#endif
}

uint8_t tdma_get_config_epoch(void)
{
#if CONFIG_CONNECTION_TDMA
	/* Read epoch from packed value with seqlock for consistency */
	uint64_t packed;
	do {
		packed = tdma_dyn_packed;
	} while (packed != tdma_dyn_packed);
	return (uint8_t)packed;
#else
	return 0;
#endif
}
