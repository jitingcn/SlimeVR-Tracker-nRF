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
#include "system/led.h"
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(tdma, LOG_LEVEL_INF);

/* Slot[3:0], enabled[7], width[15:8], total[23:16], receiver epoch[31:24]. */
static atomic_t tdma_cfg_pack;

static uint8_t tdma_slot_index_init; /* used only before first dyn config */

#define TDMA_ENABLED_BIT 0x80U
#define TDMA_PACK(slot, sticks, total, epoch) \
	((uint32_t)(uint8_t)(slot) | ((uint32_t)(uint8_t)(sticks) << 8) \
	 | ((uint32_t)(uint8_t)(total) << 16) | ((uint32_t)(uint8_t)(epoch) << 24))
#define TDMA_UNPACK_SLOT(p) ((uint8_t)((p) & 0x0Fu))
#define TDMA_UNPACK_STICKS(p) ((uint8_t)(((p) >> 8) & 0xFFu))
#define TDMA_UNPACK_TOTAL(p) ((uint8_t)(((p) >> 16) & 0xFFu))
#define TDMA_UNPACK_EPOCH(p) ((uint8_t)((p) >> 24))
#define TDMA_UNPACK_FRAME(p) ((uint16_t)(TDMA_UNPACK_STICKS(p) * TDMA_UNPACK_TOTAL(p)))

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
static uint32_t tdma_last_admitted_frame = UINT32_MAX;
static uint32_t tdma_last_admitted_pack;
static uint32_t tdma_last_ping_frame = UINT32_MAX;
static uint32_t tdma_last_ping_pack;

void tdma_init(uint8_t tracker_id)
{
	tdma_slot_index_init = tracker_id % TDMA_NUM_TRACKERS;
	unsigned key = irq_lock();
	atomic_set(&tdma_cfg_pack,
		   (atomic_val_t)TDMA_PACK(tdma_slot_index_init, TDMA_SLOT_TICKS, TDMA_NUM_TRACKERS, 0));
	tdma_last_admitted_frame = UINT32_MAX;
	tdma_last_admitted_pack = 0;
	tdma_last_ping_frame = UINT32_MAX;
	tdma_last_ping_pack = 0;
	irq_unlock(key);
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

struct tdma_config {
	uint32_t pack;
	uint8_t epoch;
	uint8_t slot_index;
	uint8_t total_slots;
	uint8_t slot_ticks;
	uint16_t frame_ticks;
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

static bool tdma_config_snapshot(struct tdma_config *config)
{
	config->pack = (uint32_t)atomic_get(&tdma_cfg_pack);
	config->epoch = TDMA_UNPACK_EPOCH(config->pack);
	config->slot_index = TDMA_UNPACK_SLOT(config->pack);
	config->slot_ticks = TDMA_UNPACK_STICKS(config->pack);
	config->frame_ticks = TDMA_UNPACK_FRAME(config->pack);
	config->total_slots = TDMA_UNPACK_TOTAL(config->pack);
	return (config->pack & TDMA_ENABLED_BIT) && config->total_slots != 0;
}

static bool tdma_config_matches(const struct tdma_config *config)
{
	return (config->pack & TDMA_ENABLED_BIT)
		&& (uint32_t)atomic_get(&tdma_cfg_pack) == config->pack;
}

static bool tdma_clock_snapshot(const struct tdma_config *config, struct esb_network_clock *clock)
{
	return esb_get_network_clock(clock) && clock->sync_age_ms <= TDMA_SYNC_STALE_MS
		&& tdma_config_matches(config);
}

int64_t tdma_status_clock_max_age_ms(void)
{
#if CONFIG_CONNECTION_TDMA
	struct tdma_config config;
	if (tdma_config_snapshot(&config)) {
		return TDMA_SYNC_STALE_MS;
	}
#endif
	return -1;
}

static uint32_t tdma_last_frame(const struct tdma_config *config)
{
	return UINT32_MAX / config->frame_ticks;
}

static bool tdma_frame_eligible(uint32_t frame, const struct tdma_config *config)
{
	return frame != 0 && frame < tdma_last_frame(config);
}

static bool tdma_ping_frame_eligible(uint32_t frame, uint8_t slot, const struct tdma_config *config)
{
	uint32_t period = tdma_ping_period_frames(config->frame_ticks);
	if (!tdma_frame_eligible(frame, config)
		|| frame % period != tdma_ping_phase_frame(slot, config->total_slots, period)) {
		return false;
	}
	uint64_t start = (uint64_t)frame * config->frame_ticks
		+ (uint32_t)slot * config->slot_ticks + TDMA_SLOT_TARGET_OFFSET;
	return start + TDMA_PING_LATE_TOLERANCE_TICKS
		+ (uint32_t)config->slot_ticks * TDMA_PING_WINDOW_SLOTS <= 0x100000000ULL;
}

static bool tdma_data_frame_guarded(uint32_t frame, const struct tdma_config *config, bool *own_ping)
{
	*own_ping = tdma_ping_frame_eligible(frame, config->slot_index, config);
	uint8_t previous_slot = config->slot_index ? config->slot_index - 1U : config->total_slots - 1U;
	uint32_t previous_frame = config->slot_index ? frame : (frame ? frame - 1U : tdma_last_frame(config));
	return *own_ping || tdma_ping_frame_eligible(previous_frame, previous_slot, config);
}

static uint32_t tdma_next_frame(uint32_t frame, const struct tdma_config *config)
{
	return frame >= tdma_last_frame(config) ? 0 : frame + 1U;
}

/* Five frames cover the final frame, frame zero, two PING guards and a usable
 * frame. Valid layouts have at most 16 slots of 255 ticks. A PING period is at
 * most 32768 ticks; 53684 ticks bounds either search across rollover. */
#define TDMA_SEARCH_FRAMES 5
#define TDMA_MAX_RELATIVE_TICKS (32768U + TDMA_SEARCH_FRAMES * TDMA_MAX_TRACKERS * UINT8_MAX \
	+ TDMA_PING_WINDOW_SLOTS * UINT8_MAX + TDMA_PING_LATE_TOLERANCE_TICKS)
#define TDMA_COARSE_WAKE_LEAD_TICKS 4
#define TDMA_COARSE_SLEEPS 8
#define TDMA_FINE_SPINS 32 /* At most 320 us of 10 us busy waits. */

static void tdma_sleep_network_ticks(uint32_t ticks)
{
	uint64_t us = ((uint64_t)ticks * 1000000U + 32767U) / 32768U;
	if (us > 0) {
		k_sleep(K_USEC(us));
	}
}

static bool tdma_wait_until_network_tick(
	const struct tdma_config *config, struct esb_network_clock clock, uint32_t target
)
{
	uint32_t distance = target - clock.receiver_ticks;
	if (distance > TDMA_MAX_RELATIVE_TICKS) {
		return false;
	}
	uint64_t start_local = clock.local_kernel_ticks;
	uint64_t budget_us = ((uint64_t)(distance + config->slot_ticks + TDMA_COARSE_WAKE_LEAD_TICKS)
		* 1000000U + 32767U) / 32768U;
	unsigned spins = 0, sleeps = 0;
	for (;;) {
		if (!tdma_clock_snapshot(config, &clock) || clock.local_kernel_ticks < start_local
			|| k_ticks_to_us_floor64(clock.local_kernel_ticks - start_local) > budget_us) {
			return false;
		}
		uint32_t remaining = target - clock.receiver_ticks;
		if (remaining == 0 || remaining > 0x80000000U) {
			return true;
		}
		if (remaining == 0x80000000U || remaining > TDMA_MAX_RELATIVE_TICKS) {
			return false;
		}
		if (remaining > TDMA_COARSE_WAKE_LEAD_TICKS) {
			if (sleeps++ >= TDMA_COARSE_SLEEPS) {
				return false;
			}
			tdma_sleep_network_ticks(remaining - TDMA_COARSE_WAKE_LEAD_TICKS);
		} else {
			if (spins++ >= TDMA_FINE_SPINS) {
				return false;
			}
			k_busy_wait(10);
		}
	}
}

static bool tdma_ping_target(
	uint32_t now, const struct tdma_config *config,
	uint32_t *target_frame, uint32_t *target, bool *in_current_window
)
{
	uint32_t period = tdma_ping_period_frames(config->frame_ticks);
	uint32_t phase = tdma_ping_phase_frame(config->slot_index, config->total_slots, period);
	uint32_t frame = now / config->frame_ticks;
	uint32_t remainder = (frame % period + period - phase) % period;
	if (frame >= remainder) {
		uint32_t previous = frame - remainder;
		uint64_t start = (uint64_t)previous * config->frame_ticks
			+ (uint32_t)config->slot_index * config->slot_ticks + TDMA_SLOT_TARGET_OFFSET;
		if (tdma_ping_frame_eligible(previous, config->slot_index, config)
			&& now >= start && now < start + (uint32_t)config->slot_ticks * TDMA_PING_WINDOW_SLOTS) {
			*target_frame = previous;
			*target = (uint32_t)start;
			*in_current_window = true;
			return true;
		}
	}

	uint32_t candidate = frame + (phase + period - frame % period) % period;
	bool after_wrap = false;
	for (unsigned i = 0; i < 4; i++) {
		if (candidate >= tdma_last_frame(config)) {
			candidate = phase ? phase : period;
			after_wrap = true;
		}
		uint64_t start = (uint64_t)candidate * config->frame_ticks
			+ (uint32_t)config->slot_index * config->slot_ticks + TDMA_SLOT_TARGET_OFFSET;
		if (tdma_ping_frame_eligible(candidate, config->slot_index, config)
			&& (after_wrap || start > now)) {
			*target_frame = candidate;
			*target = (uint32_t)start;
			*in_current_window = false;
			return (uint32_t)(*target - now) <= TDMA_MAX_RELATIVE_TICKS;
		}
		candidate += period;
	}
	return false;
}

static bool tdma_wait_for_data_admission(uint8_t reserve_ticks)
{
	struct tdma_config config;
	if (!tdma_config_snapshot(&config) || config.slot_ticks <= reserve_ticks) {
		return false;
	}
	bool counted_late = false;
	bool counted_frame_used = false;
	bool counted_guard = false;
	for (unsigned attempt = 0; attempt < TDMA_SEARCH_FRAMES; attempt++) {
		struct esb_network_clock clock;
		unsigned key = irq_lock();
		if (!tdma_clock_snapshot(&config, &clock)) {
			irq_unlock(key);
			return false;
		}
		uint32_t frame = clock.receiver_ticks / config.frame_ticks;
		uint32_t phase = clock.receiver_ticks % config.frame_ticks;
		uint32_t slot_start = (uint32_t)config.slot_index * config.slot_ticks;
		int32_t pos = (int32_t)phase - (int32_t)slot_start;
		bool unused = config.pack != tdma_last_admitted_pack || frame != tdma_last_admitted_frame;
		bool own_ping;
		bool guarded = tdma_data_frame_guarded(frame, &config, &own_ping);
		if (tdma_frame_eligible(frame, &config) && !guarded && unused
			&& pos >= TDMA_SLOT_TARGET_OFFSET && pos <= config.slot_ticks - reserve_ticks) {
			tdma_last_admitted_pack = config.pack;
			tdma_last_admitted_frame = frame;
			tdma_admitted++;
			irq_unlock(key);
			return true;
		}
		uint32_t admitted_pack = tdma_last_admitted_pack;
		uint32_t admitted_frame = tdma_last_admitted_frame;
		irq_unlock(key);

		if (guarded && !counted_guard) {
			if (own_ping) {
				tdma_guard_own_ping++;
			} else {
				tdma_guard_previous_ping++;
			}
			counted_guard = true;
		}
		if (!unused && !counted_frame_used) {
			tdma_deferred_frame_used++;
			counted_frame_used = true;
		}
		if (pos > config.slot_ticks - reserve_ticks && !counted_late) {
			tdma_deferred_late++;
			counted_late = true;
		}
		uint32_t candidate = tdma_frame_eligible(frame, &config) && !guarded && unused
			&& pos < TDMA_SLOT_TARGET_OFFSET ? frame : tdma_next_frame(frame, &config);
		bool found = false;
		uint32_t target = 0;
		for (unsigned i = 0; i < TDMA_SEARCH_FRAMES; i++) {
			bool ignored;
			if (tdma_frame_eligible(candidate, &config)
				&& !tdma_data_frame_guarded(candidate, &config, &ignored)
				&& (config.pack != admitted_pack || candidate != admitted_frame)) {
				target = candidate * config.frame_ticks + slot_start + TDMA_SLOT_TARGET_OFFSET;
				found = true;
				break;
			}
			candidate = tdma_next_frame(candidate, &config);
		}
		if (!found || (uint32_t)(target - clock.receiver_ticks) > TDMA_MAX_RELATIVE_TICKS) {
			return false;
		}
		uint32_t ping_frame, ping_target;
		bool in_ping;
		if (!tdma_ping_target(clock.receiver_ticks, &config, &ping_frame, &ping_target, &in_ping)) {
			return false;
		}
		uint32_t ping_delay = in_ping ? 0 : ping_target - clock.receiver_ticks;
		if (ping_delay <= (uint32_t)(target - clock.receiver_ticks)) {
			tdma_deferred_for_ping++;
			return false;
		}
		if (!tdma_wait_until_network_tick(&config, clock, target)) {
			return false;
		}
	}
	return false;
}

bool tdma_wait_for_slot(uint8_t payload_length)
{
	return tdma_wait_for_data_admission(tdma_reserve_ticks(payload_length));
}

bool tdma_ping_wake_delay_ms(uint32_t *delay_ms)
{
	struct tdma_config config;
	struct esb_network_clock clock;
	if (!delay_ms) {
		return false;
	}
	unsigned key = irq_lock();
	bool configured = tdma_config_snapshot(&config);
	uint32_t ping_pack = tdma_last_ping_pack;
	uint32_t ping_frame = tdma_last_ping_frame;
	irq_unlock(key);
	if (!configured || !tdma_clock_snapshot(&config, &clock)) {
		return false;
	}
	uint32_t current_frame = clock.receiver_ticks / config.frame_ticks;
	if (!tdma_frame_eligible(current_frame, &config)) {
		uint32_t ticks = current_frame == 0 ? config.frame_ticks - clock.receiver_ticks
			: 0U - clock.receiver_ticks + config.frame_ticks;
		*delay_ms = (uint32_t)(((uint64_t)ticks * 1000U + 32767U) / 32768U);
		return true;
	}
	uint32_t frame, target;
	bool in_window;
	if (!tdma_ping_target(clock.receiver_ticks, &config, &frame, &target, &in_window)) {
		*delay_ms = 1;
		return true;
	}
	if (in_window && (config.pack != ping_pack || frame != ping_frame)) {
		*delay_ms = 0;
		return true;
	}
	if (in_window && !tdma_ping_target(target + config.slot_ticks * TDMA_PING_WINDOW_SLOTS,
		&config, &frame, &target, &in_window)) {
		*delay_ms = 1;
		return true;
	}
	uint32_t ticks = target - clock.receiver_ticks;
	uint32_t wake_ticks = ticks > config.frame_ticks ? ticks - config.frame_ticks : 0;
	*delay_ms = (uint32_t)(((uint64_t)wake_ticks * 1000U) / 32768U);
	return true;
}

enum tdma_ping_admission tdma_wait_for_ping_window(void)
{
	struct tdma_config config;
	struct esb_network_clock clock;
	if (!tdma_config_snapshot(&config) || !tdma_clock_snapshot(&config, &clock)) {
		tdma_ping_startup++;
		return TDMA_PING_UNAVAILABLE;
	}
	if (!tdma_frame_eligible(clock.receiver_ticks / config.frame_ticks, &config)) {
		return TDMA_PING_DEFERRED;
	}
	uint32_t frame, target;
	bool in_window;
	if (!tdma_ping_target(clock.receiver_ticks, &config, &frame, &target, &in_window)) {
		return TDMA_PING_DEFERRED;
	}
	if (!in_window) {
		uint32_t distance = target - clock.receiver_ticks;
		if (distance > config.frame_ticks) {
			tdma_ping_deferred_frame++;
			uint32_t wake_ticks = distance - config.frame_ticks;
			if (wake_ticks <= 32768U / 1000U) {
				tdma_sleep_network_ticks(wake_ticks);
			}
			return TDMA_PING_DEFERRED;
		}
		if (!tdma_wait_until_network_tick(&config, clock, target)) {
			return TDMA_PING_DEFERRED;
		}
	}

	unsigned key = irq_lock();
	if (!tdma_clock_snapshot(&config, &clock)
		|| !tdma_ping_target(clock.receiver_ticks, &config, &frame, &target, &in_window)) {
		irq_unlock(key);
		return TDMA_PING_DEFERRED;
	}
	bool duplicate = config.pack == tdma_last_ping_pack && frame == tdma_last_ping_frame;
	if (!in_window || clock.receiver_ticks - target > TDMA_PING_LATE_TOLERANCE_TICKS || duplicate) {
		if (in_window && !duplicate) {
			tdma_last_ping_pack = config.pack;
			tdma_last_ping_frame = frame;
		}
		tdma_ping_missed_window++;
		irq_unlock(key);
		return TDMA_PING_DEFERRED;
	}
	tdma_last_ping_pack = config.pack;
	tdma_last_ping_frame = frame;
	tdma_last_admitted_pack = config.pack;
	tdma_last_admitted_frame = frame;
	tdma_ping_slotted++;
	irq_unlock(key);
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
	unsigned key = irq_lock();
	uint32_t pack = (uint32_t)atomic_get(&tdma_cfg_pack);
	bool prev = (pack & TDMA_ENABLED_BIT) != 0;
	atomic_set(&tdma_cfg_pack, (atomic_val_t)(enabled
		? pack | TDMA_ENABLED_BIT : pack & ~TDMA_ENABLED_BIT));
	if (!prev && enabled) {
		tdma_last_admitted_frame = UINT32_MAX;
		tdma_last_admitted_pack = 0;
	}
	if ((prev != 0) != enabled) {
		tdma_last_ping_frame = UINT32_MAX;
		tdma_last_ping_pack = 0;
	}
	irq_unlock(key);
	if (prev != enabled) {
		LOG_INF("TDMA %s at runtime", enabled ? "enabled" : "disabled");
	}
#else
	ARG_UNUSED(enabled);
	LOG_WRN("TDMA not compiled in, cannot change at runtime");
#endif
}

int tdma_user_set_enabled(bool enabled)
{
#if CONFIG_CONNECTION_TDMA
	tdma_set_enabled(enabled);
	led_request_event(LED_OWNER_RADIO, led_request_id(), led_event_id(), LED_SUCCESS);
	return 0;
#else
	ARG_UNUSED(enabled);
	led_request_event(LED_OWNER_RADIO, led_request_id(), led_event_id(), LED_REJECTED);
	return -ENOTSUP;
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
	if (slot_index == NULL || slot_ticks == NULL || frame_ticks == NULL) {
		return false;
	}
	uint32_t pack = (uint32_t)atomic_get(&tdma_cfg_pack);
	if (!(pack & TDMA_ENABLED_BIT)) {
		return false;
	}
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
	return (atomic_get(&tdma_cfg_pack) & TDMA_ENABLED_BIT) != 0;
#else
	return false;
#endif
}

bool tdma_admission_stalled(void)
{
	/* Every TDMA config is validated before it can enable admission, so a
	 * missing runtime config is "not stalled" rather than a permanent stall. */
	if (!(atomic_get(&tdma_cfg_pack) & TDMA_ENABLED_BIT)) {
		return false;
	}
	/* Mirrors the refusal in tdma_wait_for_data_admission(): without receiver
	 * time there is no slot to wait for, so admission fails immediately. */
	int64_t sync_age = esb_get_sync_age_ms();
	return sync_age < 0 || sync_age > TDMA_SYNC_STALE_MS;
}

bool tdma_update_config(uint8_t slot_index, uint8_t total_slots, uint8_t slot_ticks, uint8_t epoch)
{
	if (total_slots == 0 || total_slots > TDMA_MAX_TRACKERS
		|| slot_ticks < TDMA_MIN_SAFE_SLOT_TICKS) {
		LOG_WRN("TDMA: Invalid config total=%u slot_ticks=%u", total_slots, slot_ticks);
		return false;
	}

	if (slot_index >= total_slots) {
		LOG_WRN("TDMA: Invalid slot_index=%u (>= total_slots=%u), ignoring config update",
			slot_index, total_slots);
		return false;
	}

#if CONFIG_CONNECTION_TDMA
	uint16_t frame_ticks = (uint16_t)slot_ticks * total_slots;
	unsigned key = irq_lock();
	bool newly_enabled = !(atomic_get(&tdma_cfg_pack) & TDMA_ENABLED_BIT);
	atomic_set(&tdma_cfg_pack, (atomic_val_t)(TDMA_PACK(slot_index, slot_ticks, total_slots, epoch)
		| TDMA_ENABLED_BIT));

	tdma_last_admitted_frame = UINT32_MAX;
	tdma_last_admitted_pack = 0;
	tdma_last_ping_frame = UINT32_MAX;
	tdma_last_ping_pack = 0;
	irq_unlock(key);
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
	return true;
}

uint8_t tdma_get_config_epoch(void)
{
#if CONFIG_CONNECTION_TDMA
	return TDMA_UNPACK_EPOCH((uint32_t)atomic_get(&tdma_cfg_pack));
#else
	return 0;
#endif
}
