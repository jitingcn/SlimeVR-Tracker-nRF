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
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(tdma, LOG_LEVEL_INF);

/*
 * Pack slot_index[7:0] | slot_ticks[15:8] | frame_ticks[31:16] into one atomic
 * so ESB RX can publish and TX thread can snapshot without irq_lock.
 */
static atomic_t tdma_cfg_pack;
static atomic_t tdma_epoch_at;
static atomic_t tdma_runtime_enabled;

static uint8_t tdma_slot_index_init; /* used only before first dyn config */

#define TDMA_PACK(slot, sticks, frame) \
	(((uint32_t)(uint8_t)(slot)) | ((uint32_t)(uint8_t)(sticks) << 8) | ((uint32_t)(uint16_t)(frame) << 16))
#define TDMA_UNPACK_SLOT(p) ((uint8_t)((p) & 0xFFu))
#define TDMA_UNPACK_STICKS(p) ((uint8_t)(((p) >> 8) & 0xFFu))
#define TDMA_UNPACK_FRAME(p) ((uint16_t)(((p) >> 16) & 0xFFFFu))

/* Cumulative canary counters; printed only on explicit console request. */
static uint32_t tdma_admitted;
static uint32_t tdma_deferred_late;
static uint32_t tdma_deferred_frame_used;
static uint32_t tdma_guard_own_ping;
static uint32_t tdma_guard_previous_ping;
static uint32_t tdma_radio_busy;
static uint32_t tdma_ping_slotted;
static uint32_t tdma_ping_startup;
static uint32_t tdma_ping_deferred_frame;
static uint32_t tdma_ping_missed_window;
static uint32_t tdma_deferred_for_ping;
static uint64_t tdma_last_admitted_frame = UINT64_MAX;
static uint32_t tdma_last_admitted_pack;
static uint64_t tdma_last_ping_frame = UINT64_MAX;
static uint32_t tdma_last_ping_pack;

void tdma_init(uint8_t tracker_id)
{
	tdma_slot_index_init = tracker_id % TDMA_NUM_TRACKERS;
	atomic_set(&tdma_cfg_pack,
		   (atomic_val_t)TDMA_PACK(tdma_slot_index_init, TDMA_SLOT_TICKS, TDMA_FRAME_TICKS));
	atomic_set(&tdma_epoch_at, 0);
	atomic_set(&tdma_runtime_enabled, 0);
	tdma_last_admitted_frame = UINT64_MAX;
	tdma_last_admitted_pack = 0;
	tdma_last_ping_frame = UINT64_MAX;
	tdma_last_ping_pack = 0;
#if CONFIG_CONNECTION_TDMA
	LOG_INF("TDMA init: slot=%u/%u, frame=%u ticks (~%u.%01u ms), ~%u TPS/tracker",
		tdma_slot_index_init, TDMA_NUM_TRACKERS,
		TDMA_FRAME_TICKS,
		(TDMA_FRAME_TICKS * 1000) / 32768,
		((TDMA_FRAME_TICKS * 10000) / 32768) % 10,
		32768 / TDMA_FRAME_TICKS);
#else
	LOG_INF("TDMA compiled out, all transmissions immediate");
#endif
}

/* Guarded-PING TDMA parameters. Reserves are call-to-END budgets measured on
 * 2 Mbps hardware (call->READY plus READY->END per payload class), taken from
 * the configured slot target offset; each admission window must stay >= 3
 * ticks wide so wake jitter cannot starve it. The guarded PING transaction
 * (two attempts) must fit its two-slot window; its late tolerance trades a
 * rare guard overrun against the risk of PING starvation, which would cost
 * clock sync for every tracker. */
#define TDMA_LEN17_RESERVE_TICKS 8
#define TDMA_LEN18_32_RESERVE_TICKS 10
#define TDMA_LEN33_PLUS_RESERVE_TICKS 12
#define TDMA_SLOT_TARGET_OFFSET 2
#define TDMA_PING_LATE_TOLERANCE_TICKS 6
#define TDMA_MIN_SAFE_SLOT_TICKS 16
#define TDMA_SYNC_STALE_MS (PING_INTERVAL_MS * 10)
#define TDMA_PING_WINDOW_SLOTS 2

static uint32_t tdma_ping_period_frames(uint16_t frame_ticks)
{
	/* Floor keeps every deterministic interval at or below one second. */
	return frame_ticks > 0 ? MAX(1U, 32768U / frame_ticks) : 0;
}

static uint32_t tdma_ping_phase_frame(uint8_t slot_index, uint8_t total_slots, uint32_t period_frames)
{
	return ((uint64_t)slot_index * period_frames) / total_slots;
}

static bool tdma_slot_ping_frame(
	uint64_t frame_number,
	uint8_t slot_index,
	uint8_t total_slots,
	uint32_t period_frames
)
{
	return frame_number % period_frames
		== tdma_ping_phase_frame(slot_index, total_slots, period_frames);
}
enum tdma_admission_kind {
	TDMA_ADMISSION_DATA,
	TDMA_ADMISSION_PING,
};

static uint8_t tdma_reserve_ticks(uint8_t payload_length)
{
	if (payload_length <= 17) {
		return TDMA_LEN17_RESERVE_TICKS;
	}
	if (payload_length <= 32) {
		return TDMA_LEN18_32_RESERVE_TICKS;
	}
	return TDMA_LEN33_PLUS_RESERVE_TICKS;
}

static void tdma_sleep_network_ticks(uint64_t ticks)
{
	uint64_t us = (ticks * 1000000U + 32767U) / 32768U;
	if (us > 0) {
		k_sleep(K_USEC(us));
	}
}

#define TDMA_COARSE_WAKE_LEAD_TICKS 4

static void tdma_wait_until_network_tick(uint64_t target_ticks)
{
	for (;;) {
		uint64_t now_ticks = esb_get_server_time_ticks_64();
		if (now_ticks == 0 || now_ticks >= target_ticks) {
			return;
		}
		uint64_t remaining = target_ticks - now_ticks;
		if (remaining > TDMA_COARSE_WAKE_LEAD_TICKS) {
			tdma_sleep_network_ticks(remaining - TDMA_COARSE_WAKE_LEAD_TICKS);
		} else {
			/* Tight slots leave only a couple of ticks of call-time margin for
			 * the longest payload. Finish the coarse sleep with a bounded spin
			 * so wake jitter cannot skip the admission window. */
			k_busy_wait(10);
		}
	}
}
static bool tdma_config_snapshot(
	uint32_t *pack,
	uint8_t *slot_index,
	uint8_t *total_slots,
	uint8_t *slot_ticks,
	uint16_t *frame_ticks
)
{
	if (!atomic_get(&tdma_runtime_enabled)) {
		return false;
	}
	*pack = (uint32_t)atomic_get(&tdma_cfg_pack);
	*slot_index = TDMA_UNPACK_SLOT(*pack);
	*slot_ticks = TDMA_UNPACK_STICKS(*pack);
	*frame_ticks = TDMA_UNPACK_FRAME(*pack);
	*total_slots = *slot_ticks > 0 ? *frame_ticks / *slot_ticks : 0;
	return *frame_ticks > 0 && *slot_ticks >= TDMA_MIN_SAFE_SLOT_TICKS
		&& *total_slots > 0 && *slot_index < *total_slots;
}

static bool tdma_data_frame_guarded(
	uint64_t frame_number,
	uint8_t slot_index,
	uint8_t total_slots,
	uint32_t period_frames,
	bool *own_ping
)
{
	*own_ping = tdma_slot_ping_frame(frame_number, slot_index, total_slots, period_frames);
	uint8_t previous_slot = slot_index == 0 ? total_slots - 1U : slot_index - 1U;
	if (slot_index == 0 && frame_number == 0) {
		return *own_ping;
	}
	uint64_t previous_frame = slot_index == 0 ? frame_number - 1U : frame_number;
	return *own_ping || tdma_slot_ping_frame(previous_frame, previous_slot, total_slots, period_frames);
}

static bool tdma_wait_for_data_admission(uint8_t reserve_ticks)
{
	uint32_t pack;
	uint8_t slot_index;
	uint8_t total_slots;
	uint8_t slot_ticks;
	uint16_t frame_ticks;
	if (!tdma_config_snapshot(&pack, &slot_index, &total_slots, &slot_ticks, &frame_ticks)) {
		return false;
	}
	int64_t sync_age = esb_get_sync_age_ms();
	if (sync_age < 0 || sync_age > TDMA_SYNC_STALE_MS || slot_ticks <= reserve_ticks) {
		return false;
	}

	uint8_t latest_start = slot_ticks - reserve_ticks;
	bool counted_late = false;
	bool counted_frame_used = false;
	bool counted_guard = false;
	for (;;) {
		if (esb_get_sync_age_ms() < 0 || esb_get_sync_age_ms() > TDMA_SYNC_STALE_MS) {
			return false;
		}
		uint64_t server_ticks = esb_get_server_time_ticks_64();
		if (server_ticks == 0) {
			return false;
		}
		uint32_t current_pack = (uint32_t)atomic_get(&tdma_cfg_pack);
		if (current_pack != pack) {
			if (!tdma_config_snapshot(&pack, &slot_index, &total_slots, &slot_ticks, &frame_ticks)
				|| slot_ticks <= reserve_ticks) {
				return false;
			}
			latest_start = slot_ticks - reserve_ticks;
			tdma_last_admitted_frame = UINT64_MAX;
			tdma_last_admitted_pack = 0;
		}

		uint64_t frame_number = server_ticks / frame_ticks;
		uint32_t frame_phase = server_ticks % frame_ticks;
		uint32_t slot_start = (uint32_t)slot_index * slot_ticks;
		int32_t pos_in_slot = (int32_t)frame_phase - (int32_t)slot_start;
		bool frame_unused = pack != tdma_last_admitted_pack || frame_number != tdma_last_admitted_frame;
		uint32_t period_frames = tdma_ping_period_frames(frame_ticks);
		bool own_ping;
		bool guarded = tdma_data_frame_guarded(
			frame_number, slot_index, total_slots, period_frames, &own_ping
		);

		if (!guarded && frame_unused
			&& pos_in_slot >= TDMA_SLOT_TARGET_OFFSET && pos_in_slot <= latest_start) {
			tdma_last_admitted_pack = pack;
			tdma_last_admitted_frame = frame_number;
			tdma_admitted++;
			return true;
		}
		if (guarded && !counted_guard) {
			if (own_ping) {
				tdma_guard_own_ping++;
			} else {
				tdma_guard_previous_ping++;
			}
			counted_guard = true;
		}
		if (!frame_unused && !counted_frame_used) {
			tdma_deferred_frame_used++;
			counted_frame_used = true;
		}
		if (pos_in_slot > latest_start && !counted_late) {
			tdma_deferred_late++;
			counted_late = true;
		}
		uint64_t current_target = frame_number * frame_ticks
			+ slot_start + TDMA_SLOT_TARGET_OFFSET;
		uint64_t target_frame = !guarded && frame_unused && server_ticks < current_target
			? frame_number : frame_number + 1U;
		while (tdma_data_frame_guarded(
			target_frame, slot_index, total_slots, period_frames, &own_ping
		)) {
			target_frame++;
		}
		uint64_t target = target_frame * frame_ticks + slot_start + TDMA_SLOT_TARGET_OFFSET;
		uint64_t own_ping_frame = frame_number - frame_number % period_frames
			+ tdma_ping_phase_frame(slot_index, total_slots, period_frames);
		if (own_ping_frame <= frame_number) {
			own_ping_frame += period_frames;
		}
		uint64_t own_ping_target = own_ping_frame * frame_ticks
			+ slot_start + TDMA_SLOT_TARGET_OFFSET;
		if (own_ping_target < target) {
			/* Return to the connection loop before a due PING instead of sleeping
			 * through it while waiting for a later data opportunity. */
			tdma_deferred_for_ping++;
			return false;
		}
		tdma_wait_until_network_tick(target);
	}
}

bool tdma_wait_for_slot(uint8_t payload_length)
{
	return tdma_wait_for_data_admission(tdma_reserve_ticks(payload_length));
}

static void tdma_ping_target(
	uint64_t server_ticks,
	uint8_t slot_index,
	uint8_t total_slots,
	uint8_t slot_ticks,
	uint16_t frame_ticks,
	uint64_t *target_frame,
	uint64_t *target_ticks,
	bool *in_current_window
)
{
	uint32_t period_frames = tdma_ping_period_frames(frame_ticks);
	uint32_t phase_frame = tdma_ping_phase_frame(slot_index, total_slots, period_frames);
	uint64_t frame_number = server_ticks / frame_ticks;
	uint64_t cycle_base = frame_number - frame_number % period_frames;
	uint64_t frame = cycle_base + phase_frame;
	uint64_t start = frame * frame_ticks
		+ (uint32_t)slot_index * slot_ticks + TDMA_SLOT_TARGET_OFFSET;
	uint64_t end = start + (uint32_t)slot_ticks * TDMA_PING_WINDOW_SLOTS;
	*in_current_window = server_ticks >= start && server_ticks < end;
	if (!*in_current_window && server_ticks >= end) {
		frame += period_frames;
		start += (uint64_t)period_frames * frame_ticks;
	}
	*target_frame = frame;
	*target_ticks = start;
}

bool tdma_ping_wake_delay_ms(uint32_t *delay_ms)
{
	uint32_t pack;
	uint8_t slot_index;
	uint8_t total_slots;
	uint8_t slot_ticks;
	uint16_t frame_ticks;
	if (delay_ms == NULL || !tdma_config_snapshot(
		&pack, &slot_index, &total_slots, &slot_ticks, &frame_ticks
	) || esb_get_sync_age_ms() < 0 || esb_get_sync_age_ms() > TDMA_SYNC_STALE_MS) {
		return false;
	}
	uint64_t server_ticks = esb_get_server_time_ticks_64();
	if (server_ticks == 0) {
		return false;
	}
	uint64_t target_frame;
	uint64_t target;
	bool in_current_window;
	tdma_ping_target(
		server_ticks, slot_index, total_slots, slot_ticks, frame_ticks,
		&target_frame, &target, &in_current_window
	);
	if (in_current_window && (pack != tdma_last_ping_pack || target_frame != tdma_last_ping_frame)) {
		*delay_ms = 0;
		return true;
	}
	if (in_current_window) {
		uint32_t period_frames = tdma_ping_period_frames(frame_ticks);
		target += (uint64_t)period_frames * frame_ticks;
	}
	uint64_t ticks = target - server_ticks;
	/* Wake one whole frame early. The connection loop then keeps serving data
	 * until data admission sees that the PING target precedes its next usable
	 * slot and returns directly to this scheduler. Sleeping to the exact PING
	 * millisecond cannot meet a one-tick (~30 us) software admission window. */
	uint64_t wake_ticks = ticks > frame_ticks ? ticks - frame_ticks : 0;
	*delay_ms = (uint32_t)((wake_ticks * 1000U) / 32768U);
	return true;
}

enum tdma_ping_admission tdma_wait_for_ping_window(void)
{
	uint32_t pack;
	uint8_t slot_index;
	uint8_t total_slots;
	uint8_t slot_ticks;
	uint16_t frame_ticks;
	if (!tdma_config_snapshot(&pack, &slot_index, &total_slots, &slot_ticks, &frame_ticks)
		|| esb_get_sync_age_ms() < 0 || esb_get_sync_age_ms() > TDMA_SYNC_STALE_MS) {
		tdma_ping_startup++;
		return TDMA_PING_UNAVAILABLE;
	}

	uint64_t server_ticks = esb_get_server_time_ticks_64();
	uint64_t target_frame;
	uint64_t target;
	bool in_current_window;
	tdma_ping_target(
		server_ticks, slot_index, total_slots, slot_ticks, frame_ticks,
		&target_frame, &target, &in_current_window
	);
	if (!in_current_window) {
		if (target - server_ticks > frame_ticks) {
			tdma_ping_deferred_frame++;
			return TDMA_PING_DEFERRED;
		}
		tdma_wait_until_network_tick(target);
		server_ticks = esb_get_server_time_ticks_64();
		tdma_ping_target(
			server_ticks, slot_index, total_slots, slot_ticks, frame_ticks,
			&target_frame, &target, &in_current_window
		);
	}
	uint64_t window_pos = server_ticks - target;
	bool duplicate = pack == tdma_last_ping_pack && target_frame == tdma_last_ping_frame;
	if (!in_current_window || window_pos > TDMA_PING_LATE_TOLERANCE_TICKS || duplicate) {
		if (in_current_window && !duplicate) {
			tdma_last_ping_pack = pack;
			tdma_last_ping_frame = target_frame;
		}
		tdma_ping_missed_window++;
		return TDMA_PING_DEFERRED;
	}
	tdma_last_ping_pack = pack;
	tdma_last_ping_frame = target_frame;
	tdma_last_admitted_pack = pack;
	tdma_last_admitted_frame = target_frame;
	tdma_ping_slotted++;
	return TDMA_PING_ADMITTED;
}

void tdma_note_radio_busy(void)
{
	tdma_radio_busy++;
}

void tdma_print_stats(void)
{
	printk(
		"TDMA ADMIT mode=ping_guard_v1 ok=%u defer_late=%u defer_frame=%u defer_ping=%u guard_own=%u guard_prev=%u radio_busy=%u ping_window=%u ping_startup=%u ping_defer=%u ping_missed=%u\n",
		tdma_admitted,
		tdma_deferred_late,
		tdma_deferred_frame_used,
		tdma_deferred_for_ping,
		tdma_guard_own_ping,
		tdma_guard_previous_ping,
		tdma_radio_busy,
		tdma_ping_slotted,
		tdma_ping_startup,
		tdma_ping_deferred_frame,
		tdma_ping_missed_window
	);
}

void tdma_set_enabled(bool enabled)
{
#if CONFIG_CONNECTION_TDMA
	atomic_val_t prev = atomic_set(&tdma_runtime_enabled, enabled ? 1 : 0);
	if (prev == 0 && enabled) {
		tdma_last_admitted_frame = UINT64_MAX;
		tdma_last_admitted_pack = 0;
	}
	if ((prev != 0) != enabled) {
		tdma_last_ping_frame = UINT64_MAX;
		tdma_last_ping_pack = 0;
		LOG_INF("TDMA %s at runtime", enabled ? "enabled" : "disabled");
	}
#else
	ARG_UNUSED(enabled);
	LOG_WRN("TDMA not compiled in, cannot change at runtime");
#endif
}

uint16_t tdma_frame_ticks_get(void)
{
	uint32_t pack = (uint32_t)atomic_get(&tdma_cfg_pack);
	return TDMA_UNPACK_FRAME(pack);
}

bool tdma_config_get(uint8_t *slot_index, uint8_t *slot_ticks, uint16_t *frame_ticks)
{
#if CONFIG_CONNECTION_TDMA
	if (!atomic_get(&tdma_runtime_enabled) || slot_index == NULL || slot_ticks == NULL || frame_ticks == NULL) {
		return false;
	}
	uint32_t pack = (uint32_t)atomic_get(&tdma_cfg_pack);
	*slot_index = TDMA_UNPACK_SLOT(pack);
	*slot_ticks = TDMA_UNPACK_STICKS(pack);
	*frame_ticks = TDMA_UNPACK_FRAME(pack);
	return *slot_ticks > 0 && *frame_ticks > 0;
#else
	ARG_UNUSED(slot_index);
	ARG_UNUSED(slot_ticks);
	ARG_UNUSED(frame_ticks);
	return false;
#endif
}

bool tdma_is_enabled(void)
{
#if CONFIG_CONNECTION_TDMA
	return atomic_get(&tdma_runtime_enabled) != 0;
#else
	return false;
#endif
}

void tdma_update_config(uint8_t slot_index, uint8_t total_slots, uint8_t slot_ticks, uint8_t epoch)
{
#if CONFIG_CONNECTION_TDMA
	if (total_slots == 0 || slot_ticks < TDMA_MIN_SAFE_SLOT_TICKS) {
		LOG_WRN("TDMA: Invalid config total=%u slot_ticks=%u", total_slots, slot_ticks);
		return;
	}

	if (slot_index >= total_slots) {
		LOG_WRN("TDMA: Invalid slot_index=%u (>= total_slots=%u), ignoring config update",
			slot_index, total_slots);
		return;
	}

	uint16_t frame_ticks = (uint16_t)slot_ticks * total_slots;
	atomic_set(&tdma_cfg_pack, (atomic_val_t)TDMA_PACK(slot_index, slot_ticks, frame_ticks));
	atomic_set(&tdma_epoch_at, epoch);
	bool newly_enabled = (atomic_set(&tdma_runtime_enabled, 1) == 0);

	tdma_last_admitted_frame = UINT64_MAX;
	tdma_last_admitted_pack = 0;
	tdma_last_ping_frame = UINT64_MAX;
	tdma_last_ping_pack = 0;
	if (newly_enabled) {
		LOG_INF("TDMA auto-enabled by receiver config");
	}

	LOG_INF("TDMA config: mode=ping_guard_v1 slot=%u/%u base_slot=%u frame=%u opportunity=%u TPS epoch=%u",
		slot_index, total_slots, slot_ticks, frame_ticks,
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
	return (uint8_t)atomic_get(&tdma_epoch_at);
#else
	return 0;
#endif
}
