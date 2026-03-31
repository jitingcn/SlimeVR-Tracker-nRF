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

static uint8_t tdma_slot_index = 0;
static bool tdma_runtime_enabled = true;

/* Diagnostic counters for TDMA slot hit/miss tracking */
static uint32_t tdma_slot_hits = 0;
static uint32_t tdma_slot_overshoots = 0;
static uint32_t tdma_grace_hits = 0;
static int64_t tdma_stats_last_ts = 0;

void tdma_init(uint8_t tracker_id)
{
	tdma_slot_index = tracker_id % TDMA_NUM_TRACKERS;
#if CONFIG_CONNECTION_TDMA
	LOG_INF("TDMA init: slot=%u/%u, frame=%u ticks (~%u.%01u ms), ~%u TPS/tracker",
		tdma_slot_index, TDMA_NUM_TRACKERS,
		TDMA_FRAME_TICKS,
		(TDMA_FRAME_TICKS * 1000) / 32768,
		((TDMA_FRAME_TICKS * 10000) / 32768) % 10,
		32768 / TDMA_FRAME_TICKS);
#else
	LOG_INF("TDMA compiled out, all transmissions immediate");
#endif
}

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

void tdma_wait_for_slot(void)
{
#if !CONFIG_CONNECTION_TDMA
	return;
#else
	if (!tdma_runtime_enabled) {
		return;
	}

	/*
	 * Skip TDMA gating when time sync is stale.
	 *
	 * Between PONGs, the server time estimate drifts by
	 * (true_skew - estimated_skew) * elapsed.  With a typical nRF52
	 * LFCLK error of 20-250 PPM and the EMA skew filter lagging
	 * behind, the cumulative phase error can exceed a full TDMA slot
	 * (20 ticks ≈ 610 µs) within a few seconds of lost sync.
	 *
	 * If we keep scheduling with a stale estimate, packets land in
	 * the wrong slot and collide with other trackers, progressively
	 * degrading TPS.  Falling back to immediate TX (no scheduling)
	 * is far better than systematic wrong-slot transmission.
	 *
	 */
	int64_t sync_age = esb_get_sync_age_ms();
	if (sync_age < 0 || sync_age > TDMA_SYNC_STALE_MS) {
		return; /* sync lost or too stale — transmit immediately */
	}

	uint64_t server_ticks = esb_get_server_time_ticks_64();
	if (server_ticks == 0) {
		return; /* not synced — fallback to immediate TX */
	}

	uint32_t frame_phase = (uint32_t)(server_ticks % TDMA_FRAME_TICKS);
	uint32_t slot_start = (uint32_t)tdma_slot_index * TDMA_SLOT_TICKS;

	/* Check if we're already inside our slot (between start and end) */
	int32_t pos_in_slot = (int32_t)frame_phase - (int32_t)slot_start;
	/* Normalize to [-FRAME/2, FRAME/2] for wrap-around */
	if (pos_in_slot > (int32_t)(TDMA_FRAME_TICKS / 2)) {
		pos_in_slot -= TDMA_FRAME_TICKS;
	} else if (pos_in_slot < -(int32_t)(TDMA_FRAME_TICKS / 2)) {
		pos_in_slot += TDMA_FRAME_TICKS;
	}

	/*
	 * If we're anywhere inside [0, TDMA_SLOT_TICKS + grace), TX now.
	 *
	 * First half  (pos < SLOT/2): plenty of margin — NoACK packet
	 * (~7-8 ticks at 2 Mbps) finishes well within our slot.
	 * Second half (pos < SLOT):   packet may finish near/at slot boundary
	 * but still within our allocation.
	 * Grace window (pos < SLOT + grace): minor overshoot from scheduler
	 * jitter, still safe before neighbour's target offset.
	 */
	if (pos_in_slot >= 0 &&
	    pos_in_slot < (int32_t)(TDMA_SLOT_TICKS + TDMA_OVERSHOOT_GRACE)) {
		if (pos_in_slot >= (int32_t)TDMA_SLOT_TICKS) {
			tdma_grace_hits++;
			LOG_DBG("TDMA grace hit: pos_in_slot=%d", pos_in_slot);
		} else {
			tdma_slot_hits++;
		}
		return; /* in slot (or minor overshoot) — start TX now */
	}

	/*
	 * Compute ticks to sleep until slot_start + TARGET_OFFSET.
	 *
	 * target_phase = slot_start + TDMA_SLOT_TARGET_OFFSET
	 * ticks_to_target = (target_phase - frame_phase) mod FRAME_TICKS
	 *
	 * This gives a bounded [1, FRAME_TICKS] sleep that always targets
	 * a few ticks into the slot, providing margin on both sides.
	 */
	uint32_t target_phase = slot_start + TDMA_SLOT_TARGET_OFFSET;
	if (target_phase >= TDMA_FRAME_TICKS) {
		target_phase -= TDMA_FRAME_TICKS;
	}
	uint32_t ticks_to_target;
	if (target_phase >= frame_phase) {
		ticks_to_target = target_phase - frame_phase;
	} else {
		ticks_to_target = TDMA_FRAME_TICKS - frame_phase + target_phase;
	}

	if (ticks_to_target > 0 && ticks_to_target <= TDMA_FRAME_TICKS) {
		k_sleep(K_TICKS(ticks_to_target));
	}

	/* Re-read server time after sleep to classify landing accuracy.
	 * Regardless of where we land, always proceed to TX.
	 *
	 * Rationale: the initial sleep targeted our slot correctly.
	 * Overshoots are caused by the sensor thread (priority 7) preempting
	 * the connection thread (priority 8) for 2-7 ms during FIFO processing.
	 * Sleeping another full frame (~6.1 ms) on overshoot creates a cascade:
	 * each penalty shifts our phase, increasing the chance of the next
	 * overshoot, progressively degrading TPS.
	 * A single out-of-slot NoACK TX (probabilistic collision) is far less
	 * harmful than guaranteed cascading throughput loss.
	 */
	server_ticks = esb_get_server_time_ticks_64();
	if (server_ticks == 0) {
		return;
	}
	frame_phase = (uint32_t)(server_ticks % TDMA_FRAME_TICKS);
	pos_in_slot = (int32_t)frame_phase - (int32_t)slot_start;
	if (pos_in_slot > (int32_t)(TDMA_FRAME_TICKS / 2)) {
		pos_in_slot -= TDMA_FRAME_TICKS;
	} else if (pos_in_slot < -(int32_t)(TDMA_FRAME_TICKS / 2)) {
		pos_in_slot += TDMA_FRAME_TICKS;
	}

	if (pos_in_slot >= 0 && pos_in_slot < (int32_t)(TDMA_SLOT_TICKS + TDMA_OVERSHOOT_GRACE)) {
		/* Landed in slot or within grace window — ideal */
		if (pos_in_slot >= (int32_t)TDMA_SLOT_TICKS) {
			tdma_grace_hits++;
		} else {
			tdma_slot_hits++;
		}
	} else {
		/* Overshot significantly — TX anyway to prevent cascade */
		tdma_slot_overshoots++;
		LOG_DBG("TDMA overshoot: pos=%d (slot 0-%d)",
			pos_in_slot, (int)(TDMA_SLOT_TICKS - 1));
	}

	/* Periodic TDMA statistics */
	int64_t now = k_uptime_get();
	if (tdma_stats_last_ts == 0) {
		tdma_stats_last_ts = now;
	}
	if (now - tdma_stats_last_ts >= 10000) {
		uint32_t total = tdma_slot_hits + tdma_grace_hits + tdma_slot_overshoots;
		if (total > 0) {
			LOG_INF("TDMA stats (10s): hits=%u grace=%u overshoot=%u total=%u",
				tdma_slot_hits, tdma_grace_hits,
				tdma_slot_overshoots, total);
		}
		tdma_slot_hits = 0;
		tdma_grace_hits = 0;
		tdma_slot_overshoots = 0;
		tdma_stats_last_ts = now;
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
