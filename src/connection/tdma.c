#include "../frame_ledger.h"
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
#include "relay_protocol.h"
#include "network_ticks.h"
#include "esb.h"
#include "system/led.h"
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(tdma, LOG_LEVEL_INF);

/* Slot[4:0], enabled[7], width[15:8], total[23:16], receiver epoch[31:24]. */
static atomic_t tdma_cfg_pack;

static uint8_t tdma_slot_index_init; /* used only before first dyn config */

#define TDMA_ENABLED_BIT 0x80U
#define TDMA_PROFILE_BIT 0x00800000U
#define TDMA_PACK(slot, sticks, total, epoch) \
	((uint32_t)(uint8_t)(slot) | ((uint32_t)(uint8_t)(sticks) << 8) \
	 | ((uint32_t)(uint8_t)(total) << 16) | ((uint32_t)(uint8_t)(epoch) << 24))
#define TDMA_UNPACK_SLOT(p) ((uint8_t)((p) & 0x1Fu))
#define TDMA_UNPACK_STICKS(p) ((uint8_t)(((p) >> 8) & 0xFFu))
#define TDMA_UNPACK_TOTAL(p) ((uint8_t)(((p) >> 16) & 0x7Fu))
#define TDMA_UNPACK_EPOCH(p) ((uint8_t)((p) >> 24))
#define TDMA_UNPACK_FRAME(p) ((uint16_t)(TDMA_UNPACK_STICKS(p) * TDMA_UNPACK_TOTAL(p)))

/* Staged layout (relay_protocol.h, "Staged layout activation"): announced
 * ahead, promoted by the first configuration snapshot at or after its
 * activation tick. Slot indices use five bits; bits 5-6 hold staging flags. */
#define TDMA_STAGED_BIT 0x40U
#define TDMA_STAGED_UNASSIGNED 0x20U
static atomic_t tdma_staged_pack;
static struct relay_geometry tdma_active_geometry = {16, 1, 16, 120, 288, 1, 0};
static struct relay_geometry tdma_staged_geometry;
const struct relay_geometry *relay_geometry_active = &tdma_active_geometry;
struct relay_timeline relay_timeline_active;
struct relay_runtime_anchor relay_direct_anchor;
static uint32_t tdma_profile_origin;
static uint32_t tdma_anchor_frame, tdma_anchor_start;
static atomic_t tdma_staged_activation; /* receiver ticks */
/* Kernel tick before which the activation cannot be due. Snapshots run on
 * the stream's admission path, where reading the network clock each time
 * made large frames miss their 2-tick admission window. */
static uint64_t tdma_staged_local;
/* Promoted to "no slot" (left the layout): data stays off the air until the
 * receiver grants a slot again. */
static atomic_t tdma_withdrawn;
/* No data transmission may straddle an activation tick: the two layouts'
 * slot boundaries differ, and trackers' receiver clocks disagree by a tick
 * or two. Admission keeps frames inside [slot start + 2, slot end). */
#define TDMA_ACTIVATION_QUIET_TICKS 4U

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
/* Forwarding-slot admission keeps its own one-packet-per-frame record. */
static uint32_t tdma_forward_admitted_frame = UINT32_MAX;
static uint32_t tdma_forward_admitted_pack;
static uint32_t tdma_last_admitted_pack;
static uint32_t tdma_stream_slots;
static uint32_t tdma_last_ping_frame = UINT32_MAX;
static uint32_t tdma_last_ping_pack;

void tdma_init(uint8_t tracker_id)
{
	tdma_slot_index_init = tracker_id % TDMA_NUM_TRACKERS;
	unsigned key = irq_lock();
	atomic_set(&tdma_cfg_pack,
		   (atomic_val_t)TDMA_PACK(tdma_slot_index_init, TDMA_SLOT_TICKS, TDMA_NUM_TRACKERS, 0));
	tdma_last_admitted_frame = UINT32_MAX;
	tdma_forward_admitted_frame = UINT32_MAX;
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
#define TDMA_FRESH_SOURCE_HANDOFF_TICKS 4
_Static_assert(TDMA_SLOT_TARGET_OFFSET == RELAY_V3_TX_OFFSET, "v3 TX target");
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

static void tdma_reset_admission_locked(void)
{
	tdma_last_admitted_frame = UINT32_MAX;
	tdma_forward_admitted_frame = UINT32_MAX;
	tdma_last_admitted_pack = 0;
	tdma_last_ping_frame = UINT32_MAX;
	tdma_last_ping_pack = 0;
}

static void tdma_promote_staged(void)
{
	uint32_t staged = (uint32_t)atomic_get(&tdma_staged_pack);
	if (!(staged & TDMA_STAGED_BIT)) {
		return;
	}
	/* Without receiver time the activation cannot be placed; the layout
	 * grant (shorter than any sync holdover) keeps data off the air. */
	struct esb_network_clock clock;
	if (k_uptime_ticks() < tdma_staged_local || !esb_get_network_clock(&clock)
	    || clock.sync_age_ms > TDMA_SYNC_STALE_MS
	    || (int32_t)(clock.receiver_ticks - (uint32_t)atomic_get(&tdma_staged_activation)) < 0) {
		return;
	}
	unsigned key = irq_lock();
	if ((uint32_t)atomic_get(&tdma_staged_pack) == staged) {
		/* Every activation begins a new geometry timeline. B supplies the
		 * authoritative frame number and two announced future starts. */
		tdma_profile_origin = (uint32_t)atomic_get(&tdma_staged_activation);
		tdma_anchor_frame = 0;
		tdma_anchor_start = tdma_profile_origin;
		relay_timeline_clear(&relay_timeline_active);
		relay_direct_anchor.valid = false;
		atomic_set(&tdma_staged_pack, 0);
		if (staged & TDMA_STAGED_UNASSIGNED) {
			atomic_set(&tdma_cfg_pack, (atomic_val_t)((uint32_t)atomic_get(&tdma_cfg_pack)
				& ~TDMA_ENABLED_BIT));
			atomic_set(&tdma_withdrawn, 1);
		} else {
			if (staged & TDMA_PROFILE_BIT) {
				tdma_active_geometry = tdma_staged_geometry;
			}
			atomic_set(&tdma_cfg_pack, (atomic_val_t)((staged & ~(TDMA_STAGED_BIT | TDMA_STAGED_UNASSIGNED))
				| TDMA_ENABLED_BIT));
			atomic_set(&tdma_withdrawn, 0);
		}
		tdma_reset_admission_locked();
	}
	irq_unlock(key);
}

#if defined(__GNUC__) && !defined(__clang__)
/* Attribute provenance: Round 1 scheduling and round 10 final-validation cohorts measured this group, not this attribute alone. Retain pending isolated A/B. */
__attribute__((optimize("O3")))
#endif
static bool tdma_config_snapshot(struct tdma_config *config)
{
	tdma_promote_staged();
	config->pack = (uint32_t)atomic_get(&tdma_cfg_pack);
	config->epoch = TDMA_UNPACK_EPOCH(config->pack);
	config->slot_index = TDMA_UNPACK_SLOT(config->pack);
	config->slot_ticks = TDMA_UNPACK_STICKS(config->pack);
	config->frame_ticks = TDMA_UNPACK_FRAME(config->pack);
	config->total_slots = TDMA_UNPACK_TOTAL(config->pack);
	if (config->pack & TDMA_PROFILE_BIT) {
		config->total_slots = RELAY_V3_TOTAL_SLOTS;
		config->frame_ticks = RELAY_V3_FRAME_TICKS_MAX;
	}
	return (config->pack & TDMA_ENABLED_BIT) && config->total_slots != 0;
}

static bool tdma_config_matches(const struct tdma_config *config)
{
	/* A wait that crosses a staged activation sees the layout change. */
	tdma_promote_staged();
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

/* Guard rule for any slot: its owner sends no data in its own PING frame or in
 * the frame where the previous slot's owner PINGs across into it. */
static bool tdma_slot_frame_guarded(uint32_t frame, uint8_t slot, const struct tdma_config *config)
{
	uint8_t previous_slot = slot ? slot - 1U : config->total_slots - 1U;
	uint32_t previous_frame = slot ? frame : (frame ? frame - 1U : tdma_last_frame(config));
	return tdma_ping_frame_eligible(frame, slot, config)
		|| tdma_ping_frame_eligible(previous_frame, previous_slot, config);
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
	uint64_t us = net_ticks_to_us_ceil64(ticks);
	if (us > 0) {
		k_sleep(K_USEC(us));
	}
}

#if defined(__GNUC__) && !defined(__clang__)
/* Attribute provenance: Round 1 scheduling and round 10 final-validation cohorts measured this group, not this attribute alone. Retain pending isolated A/B. */
__attribute__((optimize("O3")))
#endif
static bool tdma_wait_until_network_tick(
	const struct tdma_config *config, struct esb_network_clock clock, uint32_t target
)
{
	uint32_t distance = target - clock.receiver_ticks;
	if (distance > TDMA_MAX_RELATIVE_TICKS) {
		return false;
	}
	uint64_t start_local = clock.local_kernel_ticks;
	uint64_t budget_us = net_ticks_to_us_ceil64(
		distance + config->slot_ticks + TDMA_COARSE_WAKE_LEAD_TICKS);
	unsigned spins = 0, sleeps = 0;
	for (;;) {
		if (!tdma_clock_snapshot(config, &clock) || clock.local_kernel_ticks < start_local
			|| k_ticks_to_us_floor64(clock.local_kernel_ticks - start_local) > budget_us) {
			return false;
		}
		uint32_t remaining = target - clock.receiver_ticks;
		if (remaining == 0 || remaining > 0x80000000U) {
#ifdef CONFIG_RELAY_FRAME_LEDGER
			ledger_wait_lateness = MIN(clock.receiver_ticks - target, 32767U);
#endif
			return true;
		}
		if (remaining == 0x80000000U || remaining > TDMA_MAX_RELATIVE_TICKS) {
			return false;
		}
		if (remaining > TDMA_COARSE_WAKE_LEAD_TICKS) {
			if (sleeps++ >= TDMA_COARSE_SLEEPS) {
				return false;
			}
			/* Anchor the existing wake lead to the sampled kernel time.
			 * Relative sleeps add their entry work and an alignment tick. */
			uint64_t ticks = ((uint64_t)(remaining - TDMA_COARSE_WAKE_LEAD_TICKS)
				* CONFIG_SYS_CLOCK_TICKS_PER_SEC + 32767U) / 32768U;
			k_sleep(K_TIMEOUT_ABS_TICKS(clock.local_kernel_ticks + ticks));
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

/* The window [start, start + width) (receiver ticks) comes within the quiet
 * zone of a staged activation (see TDMA_ACTIVATION_QUIET_TICKS). */
static bool tdma_window_quiet(uint32_t start, uint32_t width)
{
	if (!((uint32_t)atomic_get(&tdma_staged_pack) & TDMA_STAGED_BIT)) {
		return false;
	}
	uint32_t from = (uint32_t)atomic_get(&tdma_staged_activation) - TDMA_ACTIVATION_QUIET_TICKS;
	/* Overlap of [start, start + width) with [from, from + 2 * quiet). */
	return (uint32_t)(start - from) < 2U * TDMA_ACTIVATION_QUIET_TICKS
		|| (uint32_t)(from - start) < width;
}

static bool tdma_activation_quiet(uint32_t frame, uint8_t slot, const struct tdma_config *config)
{
	return tdma_window_quiet(frame * config->frame_ticks + (uint32_t)slot * config->slot_ticks,
		config->slot_ticks);
}

static bool tdma_admission_guarded(uint32_t frame, uint8_t slot, const struct tdma_config *config, bool *own_ping)
{
	if (tdma_activation_quiet(frame, slot, config)) {
		*own_ping = false;
		return true;
	}
	if (slot == config->slot_index) {
		return tdma_data_frame_guarded(frame, config, own_ping);
	}
	*own_ping = false;
	return tdma_slot_frame_guarded(frame, slot, config);
}

/* A second forwarding attempt must not wait across the next unused own
 * opportunity. The connection owner must prepare and send its own pose first. */
static bool tdma_own_opportunity_before(const struct tdma_config *config,
	uint32_t now, uint32_t target)
{
	uint32_t frame = now / config->frame_ticks;
	for (unsigned i = 0; i < TDMA_SEARCH_FRAMES; i++, frame = tdma_next_frame(frame, config)) {
		bool own_ping;
		if (!tdma_frame_eligible(frame, config)
		    || tdma_data_frame_guarded(frame, config, &own_ping)
		    || (config->pack == tdma_last_admitted_pack && frame == tdma_last_admitted_frame)) {
			continue;
		}
		uint32_t start = frame * config->frame_ticks
			+ (uint32_t)config->slot_index * config->slot_ticks;
		int32_t pos = (int32_t)(now - start);
		if (pos > (int32_t)(config->slot_ticks - TDMA_LEN17_RESERVE_TICKS)) {
			continue;
		}
		uint32_t own = start + TDMA_SLOT_TARGET_OFFSET;
		if ((int32_t)(own - now) < 0) {
			own = now;
		}
		return (uint32_t)(own - now) <= (uint32_t)(target - now);
	}
	return false;
}

uint8_t tdma_profile_get(void)
{
	tdma_promote_staged();
	return ((uint32_t)atomic_get(&tdma_cfg_pack) & TDMA_PROFILE_BIT)
		? RELAY_PROFILE_120HZ : 0;
}

void tdma_stream_anchor(uint32_t preamble, uint32_t frame)
{
	tdma_anchor_frame = frame;
	tdma_anchor_start = relay_control_start(preamble);
	tdma_profile_origin = relay_control_origin(preamble, frame);
}

void tdma_stream_control(const struct relay_control *control)
{
	(void)relay_geometry_make_b(
		&tdma_active_geometry,
		control->direct_count,
		control->repair_count,
		control->slot_ticks,
		control->tps_max,
		control->second_b
	);
	relay_runtime_anchor_set(
		&relay_direct_anchor,
		control->frame,
		control->preamble_ticks,
		control->direct_count,
		control->repair_count,
		relay_control_next_repairs(control),
		relay_control_next2_repairs(control),
		control->slot_ticks,
		control->tps_max
	);
	relay_runtime_anchor_second_b(
		&relay_direct_anchor,
		control->repair_count,
		relay_control_next_repairs(control),
		relay_control_next2_repairs(control),
		control->tps_max,
		control->second_b
	);
	relay_timeline_announce(&relay_timeline_active, &relay_direct_anchor);
	tdma_stream_anchor(control->preamble_ticks, control->frame);
}

/* Next tick at `offset` into a frame, at or after now. */
static uint32_t tdma_stream_at(uint32_t now, uint32_t offset)
{
	uint32_t frame = relay_frame_near(tdma_anchor_frame, tdma_anchor_start, now);
	uint32_t at = relay_frame_start_near(tdma_anchor_frame, tdma_anchor_start, frame) + offset;
	return (int32_t)(at - now) < 0 ? relay_frame_start(tdma_profile_origin, frame + 1U) + offset : at;
}

bool tdma_stream_next(uint32_t *target, bool *broadcast)
{
	if (!tdma_profile_get()) {
		return false;
	}
	struct esb_network_clock clock;
	if (!esb_get_network_clock(&clock)) {
		return false;
	}
	/* An exhausted announcement fences TX, but acquisition keeps listening.
	 * It must not spend a PING interval silent after an ordinary B pair loss. */
	if (!relay_direct_anchor.valid || !relay_runtime_direct_granted(&relay_direct_anchor, clock.receiver_ticks)) {
		*broadcast = true;
		*target = clock.receiver_ticks;
		return true;
	}
	uint8_t slot = TDMA_UNPACK_SLOT((uint32_t)atomic_get(&tdma_cfg_pack));
	uint32_t own = tdma_stream_at(clock.receiver_ticks,
		slot * RELAY_V3_SLOT_TICKS + TDMA_SLOT_TARGET_OFFSET);
	/* Payload preparation can pass the target while admission is still
	 * legal. Preserve this frame until its existing reserve expires. */
	uint32_t pack = (uint32_t)atomic_get(&tdma_cfg_pack);
	uint32_t frame = relay_frame_near(tdma_anchor_frame, tdma_anchor_start, clock.receiver_ticks);
	uint32_t start = relay_frame_start(tdma_profile_origin, frame) + slot * RELAY_V3_SLOT_TICKS;
	int32_t pos = clock.receiver_ticks - start;
	bool consumed = tdma_last_admitted_pack == pack && tdma_last_admitted_frame == frame
		&& (tdma_stream_slots & (1U << slot));
	if (!consumed && pos >= TDMA_SLOT_TARGET_OFFSET &&
	    pos <= (int32_t)(RELAY_V3_SLOT_TICKS - tdma_reserve_ticks(relay_direct_payload_max(RELAY_PROFILE_120HZ, slot)))) {
		own = clock.receiver_ticks;
	}
	uint32_t beacon = tdma_stream_at(clock.receiver_ticks, RELAY_V3_B_START - RELAY_RX_LEAD_TICKS);
	/* A joiner has no direct grant until its first B. Retrying an
	 * ungranted own slot can repeatedly sleep across that B window. */
	*broadcast = !esb_layout_granted() || (int32_t)(beacon - own) < 0;
	*target = *broadcast ? beacon : own;
	return true;
}

/* Stream refusals: no grant, quiet window, early, late, slot already sent. */
uint32_t tdma_stream_refused[5];

#if defined(__GNUC__) && !defined(__clang__)
/* Attribute provenance: Round 1 scheduling and round 10 final-validation cohorts measured this group, not this attribute alone. Retain pending isolated A/B. */
__attribute__((optimize("O3")))
#endif
static bool tdma_stream_admit(uint8_t slot, uint8_t reserve, bool consume)
{
	struct esb_network_clock clock;
	for (;;) {
		if (!esb_get_network_clock(&clock) || !esb_relay_granted_at(clock.receiver_ticks)) {
			tdma_stream_refused[0] += consume;
			return false;
		}
		uint32_t pack = (uint32_t)atomic_get(&tdma_cfg_pack);
		uint32_t frame = relay_frame_near(tdma_anchor_frame, tdma_anchor_start, clock.receiver_ticks);
		uint32_t start = relay_frame_start(tdma_profile_origin, frame) + slot * RELAY_V3_SLOT_TICKS;
		int32_t pos = clock.receiver_ticks - start;
		/* D0's existing target-minus-four handoff starts in the previous
		 * frame. Treat those two ticks as early for the upcoming D0, not
		 * late for the old one. Final validation never takes this path. */
		if (consume && slot == 0) {
			uint32_t next = relay_frame_start(tdma_profile_origin, frame + 1U);
			int32_t early = clock.receiver_ticks - next;
			if (early >= TDMA_SLOT_TARGET_OFFSET - TDMA_FRESH_SOURCE_HANDOFF_TICKS && early < 0) {
				frame++;
				start = next;
				pos = early;
			}
		}
		/* The connection loop arrives at the slot start with its frame built;
		 * hold for the target offset instead of refusing. */
		if (consume && pos >= TDMA_SLOT_TARGET_OFFSET - TDMA_FRESH_SOURCE_HANDOFF_TICKS && pos < TDMA_SLOT_TARGET_OFFSET) {
			k_busy_wait(5);
			continue;
		}
		int reason = tdma_window_quiet(start, RELAY_V3_SLOT_TICKS) ? 1
			: pos < TDMA_SLOT_TARGET_OFFSET ? 2
			: pos > (int32_t)(RELAY_V3_SLOT_TICKS - reserve) ? 3 : -1;
		bool same = tdma_last_admitted_pack == pack && tdma_last_admitted_frame == frame;
		if (!consume) {
#ifdef CONFIG_RELAY_FRAME_LEDGER
			if (reason >= 0 || !same || !(tdma_stream_slots & (1U << slot)))
				ledger_window_detail = reason == 3 ? MIN(pos - (RELAY_V3_SLOT_TICKS - reserve), 32767)
					: -100 - (reason < 0 ? 5 : reason);
#endif
			return reason < 0 && same && (tdma_stream_slots & (1U << slot));
		}
		if (reason < 0 && !same) {
			tdma_stream_slots = 0;
		}
		if (reason < 0 && (tdma_stream_slots & (1U << slot))) {
			reason = 4;
		}
		if (reason >= 0) {
			tdma_stream_refused[reason]++;
#ifdef CONFIG_RELAY_FRAME_LEDGER
			ledger_window_detail = reason == 3 ? MIN(pos - (RELAY_V3_SLOT_TICKS - reserve), 32767)
				: -100 - reason;
#endif
			return false;
		}
		/* A helper may own several slots in one frame (own + repairs). */
		tdma_stream_slots |= 1U << slot;
		tdma_last_admitted_pack = pack;
		tdma_last_admitted_frame = frame;
		return true;
	}
}

/* The caller already sampled and promoted the slot configuration. */
static bool tdma_wait_for_admission(const struct tdma_config *config, uint8_t slot, uint8_t reserve_ticks,
	uint32_t *last_pack, uint32_t *last_frame)
{
	if (config->slot_ticks <= reserve_ticks
	    || slot >= config->total_slots) {
		return false;
	}
	if (tdma_profile_get()) {
		bool admitted = tdma_stream_admit(slot, reserve_ticks, true);
		if (admitted) {
			*last_pack = config->pack;
			*last_frame = tdma_last_admitted_frame;
		}
		return admitted;
	}
	bool own_slot = slot == config->slot_index;
	bool counted_late = false;
	bool counted_frame_used = false;
	bool counted_guard = false;
	for (unsigned attempt = 0; attempt < TDMA_SEARCH_FRAMES; attempt++) {
		struct esb_network_clock clock;
		unsigned key = irq_lock();
		if (!tdma_clock_snapshot(config, &clock)) {
			irq_unlock(key);
			return false;
		}
		uint32_t frame = clock.receiver_ticks / config->frame_ticks;
		uint32_t phase = clock.receiver_ticks % config->frame_ticks;
		uint32_t slot_start = (uint32_t)slot * config->slot_ticks;
		int32_t pos = (int32_t)phase - (int32_t)slot_start;
		bool unused = config->pack != *last_pack || frame != *last_frame;
		bool own_ping;
		bool guarded = tdma_admission_guarded(frame, slot, config, &own_ping);
		if (tdma_frame_eligible(frame, config) && !guarded && unused
			&& pos >= TDMA_SLOT_TARGET_OFFSET && pos <= config->slot_ticks - reserve_ticks) {
			*last_pack = config->pack;
			*last_frame = frame;
			if (own_slot) {
				tdma_admitted++;
			}
			irq_unlock(key);
			return true;
		}
		uint32_t admitted_pack = *last_pack;
		uint32_t admitted_frame = *last_frame;
		irq_unlock(key);

		if (own_slot && guarded && !counted_guard) {
			if (own_ping) {
				tdma_guard_own_ping++;
			} else {
				tdma_guard_previous_ping++;
			}
			counted_guard = true;
		}
		if (own_slot && !unused && !counted_frame_used) {
			tdma_deferred_frame_used++;
			counted_frame_used = true;
		}
		if (own_slot && pos > config->slot_ticks - reserve_ticks && !counted_late) {
			tdma_deferred_late++;
			counted_late = true;
		}
		uint32_t candidate = tdma_frame_eligible(frame, config) && !guarded && unused
			&& pos < TDMA_SLOT_TARGET_OFFSET ? frame : tdma_next_frame(frame, config);
		bool found = false;
		uint32_t target = 0;
		for (unsigned i = 0; i < TDMA_SEARCH_FRAMES; i++) {
			bool ignored;
			if (tdma_frame_eligible(candidate, config)
				&& !tdma_admission_guarded(candidate, slot, config, &ignored)
				&& (config->pack != admitted_pack || candidate != admitted_frame)) {
				target = candidate * config->frame_ticks + slot_start + TDMA_SLOT_TARGET_OFFSET;
				found = true;
				break;
			}
			candidate = tdma_next_frame(candidate, config);
		}
		if (!found || (uint32_t)(target - clock.receiver_ticks) > TDMA_MAX_RELATIVE_TICKS) {
			return false;
		}
		if (!own_slot && tdma_own_opportunity_before(config, clock.receiver_ticks, target)) {
			return false;
		}
		uint32_t ping_frame, ping_target;
		bool in_ping;
		if (!tdma_ping_target(clock.receiver_ticks, config, &ping_frame, &ping_target, &in_ping)) {
			return false;
		}
		uint32_t ping_delay = in_ping ? 0 : ping_target - clock.receiver_ticks;
		if (ping_delay <= (uint32_t)(target - clock.receiver_ticks)) {
			if (own_slot) {
				tdma_deferred_for_ping++;
			}
			return false;
		}
		if (!tdma_wait_until_network_tick(config, clock, target)) {
			return false;
		}
	}
	return false;
}

static bool tdma_wait_for_data_admission(uint8_t reserve_ticks)
{
	struct tdma_config config;
	if (!tdma_config_snapshot(&config)) {
		return false;
	}
	return tdma_wait_for_admission(&config, config.slot_index, reserve_ticks,
		&tdma_last_admitted_pack, &tdma_last_admitted_frame);
}

bool tdma_wait_for_slot(uint8_t payload_length)
{
	return tdma_wait_for_data_admission(tdma_reserve_ticks(payload_length));
}

/* IRQ-guarded source admission must never fall back to a sleeping legacy
 * wait if a staged layout changes profile. All stream deadlines are shared. */
bool tdma_admit_stream_source(uint8_t payload_length)
{
	struct tdma_config config;
	uint8_t reserve = tdma_reserve_ticks(payload_length);
	return tdma_config_snapshot(&config) && (config.pack & TDMA_PROFILE_BIT) &&
		config.slot_index < config.total_slots && config.slot_ticks > reserve &&
		tdma_stream_admit(config.slot_index, reserve, true);
}

bool tdma_wait_for_forward_slot(uint8_t slot, uint8_t payload_length)
{
	struct tdma_config config;
	if (!tdma_config_snapshot(&config) || slot == config.slot_index) {
		return false;
	}
	return tdma_wait_for_admission(&config, slot, tdma_reserve_ticks(payload_length),
		&tdma_forward_admitted_pack, &tdma_forward_admitted_frame);
}

/* Callers already need the slot configuration. Reuse that snapshot so the
 * final deadline check does not decode and promote the same config twice. */
#if defined(__GNUC__) && !defined(__clang__)
/* Attribute provenance: Round 1 scheduling and round 10 final-validation cohorts measured this group, not this attribute alone. Retain pending isolated A/B. */
__attribute__((optimize("O3")))
#endif
static bool tdma_tx_valid(const struct tdma_config *config, uint8_t slot, uint8_t length,
			  uint32_t admitted_pack, uint32_t admitted_frame)
{
	struct esb_network_clock clock;
	if (config->pack != admitted_pack
	    || slot >= config->total_slots) {
		return false;
	}
	if (tdma_profile_get()) {
		return length <= (slot >= RELAY_V3_REPAIR_FIRST_SLOT
				      ? RELAY_FRAME_MAX
				      : relay_direct_payload_max(RELAY_PROFILE_120HZ, slot)) &&
		       tdma_stream_admit(slot, tdma_reserve_ticks(length), false);
	}
	if (!tdma_clock_snapshot(config, &clock)) {
		return false;
	}
	uint32_t frame = clock.receiver_ticks / config->frame_ticks;
	uint32_t phase = clock.receiver_ticks % config->frame_ticks;
	int32_t pos = (int32_t)phase - (int32_t)((uint32_t)slot * config->slot_ticks);
	uint8_t reserve = tdma_reserve_ticks(length);
	bool own_ping;
	return frame == admitted_frame && tdma_frame_eligible(frame, config)
		&& !tdma_admission_guarded(frame, slot, config, &own_ping)
		&& config->slot_ticks > reserve && pos >= TDMA_SLOT_TARGET_OFFSET
		&& pos <= (int32_t)(config->slot_ticks - reserve);
}

#if defined(__GNUC__) && !defined(__clang__)
/* Attribute provenance: Round 1 scheduling and round 10 final-validation cohorts measured this group, not this attribute alone. Retain pending isolated A/B. */
__attribute__((optimize("O3")))
#endif
bool tdma_slot_tx_valid(uint8_t payload_length)
{
	struct tdma_config config;
	return tdma_config_snapshot(&config) && tdma_tx_valid(&config, config.slot_index, payload_length,
		tdma_last_admitted_pack, tdma_last_admitted_frame);
}

#if defined(__GNUC__) && !defined(__clang__)
/* Attribute provenance: Round 1 scheduling and round 10 final-validation cohorts measured this group, not this attribute alone. Retain pending isolated A/B. */
__attribute__((optimize("O3")))
#endif
bool tdma_forward_tx_valid(uint8_t slot, uint8_t payload_length)
{
	struct tdma_config config;
	return tdma_config_snapshot(&config) && tdma_tx_valid(&config, slot, payload_length,
		tdma_forward_admitted_pack, tdma_forward_admitted_frame);
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
		*delay_ms = net_ticks_to_ms_ceil32(ticks);
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
	*delay_ms = net_ticks_to_ms_floor32(wake_ticks);
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
	/* A PING window straddling a staged activation is skipped like a late one. */
	bool quiet = tdma_window_quiet(target, (uint32_t)config.slot_ticks * TDMA_PING_WINDOW_SLOTS);
	if (!in_window || clock.receiver_ticks - target > TDMA_PING_LATE_TOLERANCE_TICKS || duplicate
	    || quiet) {
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
		"TDMA ADMIT mode=ping_guard_v2 ok=%u defer_late=%u defer_frame=%u defer_ping=%u guard_own=%u guard_prev=%u radio_busy=%u ping_window=%u ping_startup=%u ping_defer=%u ping_missed=%u\n",
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
		tdma_forward_admitted_frame = UINT32_MAX;
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

void tdma_profile_drop(void)
{
	tdma_set_enabled(false);
	relay_timeline_clear(&relay_timeline_active);
	relay_direct_anchor.valid = false;
	unsigned key = irq_lock();
	atomic_set(&tdma_cfg_pack, (atomic_val_t)((uint32_t)atomic_get(&tdma_cfg_pack) & ~TDMA_PROFILE_BIT));
	irq_unlock(key);
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
	atomic_set(&tdma_withdrawn, 0);
	/* The legacy tuple cannot supersede a profile-1 join announcement,
	 * even when both describe the same epoch. */
	uint32_t staged = (uint32_t)atomic_get(&tdma_staged_pack);
	if ((staged & TDMA_STAGED_BIT) && !(staged & TDMA_PROFILE_BIT)
	    && TDMA_UNPACK_EPOCH(staged) == epoch) {
		atomic_set(&tdma_staged_pack, 0);
	}

	tdma_last_admitted_frame = UINT32_MAX;
	tdma_forward_admitted_frame = UINT32_MAX;
	tdma_last_admitted_pack = 0;
	tdma_last_ping_frame = UINT32_MAX;
	tdma_last_ping_pack = 0;
	irq_unlock(key);
	if (newly_enabled) {
		LOG_INF("TDMA auto-enabled by receiver config");
	}

	LOG_INF("TDMA config: mode=ping_guard_v2 slot=%u/%u base_slot=%u frame=%u opportunity=%u TPS epoch=%u",
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

bool tdma_stage_config(uint8_t slot_index, uint8_t total_slots, uint8_t slot_ticks, uint8_t epoch,
	uint32_t activation_ticks)
{
	struct relay_geometry decoded;
	bool profile = (total_slots & 0x80U) != 0;
	const struct relay_geometry *geometry = &decoded;
	if (profile && !relay_geometry_from_bootstrap(total_slots, slot_ticks, &decoded)) {
		return false;
	}
	bool unassigned = slot_index == TDMA_SLOT_UNASSIGNED;
	if (total_slots == 0 || (!profile && total_slots > TDMA_MAX_TRACKERS)
		|| (!profile && slot_ticks < TDMA_MIN_SAFE_SLOT_TICKS)
		|| (profile && !unassigned && slot_index >= geometry->direct)
		|| (!unassigned && !profile && slot_index >= total_slots)) {
		return false;
	}
#if CONFIG_CONNECTION_TDMA
	struct esb_network_clock clock;
	int32_t lead = esb_get_network_clock(&clock) ? (int32_t)(activation_ticks - clock.receiver_ticks) - 2 : 0;
	uint64_t local = lead > 0 ? clock.local_kernel_ticks + k_us_to_ticks_floor64(net_ticks_to_us_ceil64(lead)) : 0;
	uint32_t pack = TDMA_PACK(
						unassigned ? 0 : slot_index,
						slot_ticks,
						profile ? geometry->direct + 1U + geometry->repairs : total_slots,
						epoch
					)
				  | (profile ? TDMA_PROFILE_BIT : 0U) | TDMA_STAGED_BIT | (unassigned ? TDMA_STAGED_UNASSIGNED : 0U);
	unsigned key = irq_lock();
	/* The receiver keeps announcing a running profile for joiners. */
	if ((uint32_t)atomic_get(&tdma_cfg_pack)
	    == ((pack & ~(TDMA_STAGED_BIT | TDMA_STAGED_UNASSIGNED)) | TDMA_ENABLED_BIT)) {
		irq_unlock(key);
		return true;
	}
	if ((uint32_t)atomic_get(&tdma_staged_pack) != pack
	    || (uint32_t)atomic_get(&tdma_staged_activation) != activation_ticks) {
		if (profile) {
			tdma_staged_geometry = decoded;
		}
		atomic_set(&tdma_staged_activation, (atomic_val_t)activation_ticks);
		tdma_staged_local = local;
		atomic_set(&tdma_staged_pack, (atomic_val_t)pack);
	}
	irq_unlock(key);
#else
	ARG_UNUSED(epoch);
	ARG_UNUSED(activation_ticks);
#endif
	return true;
}

void tdma_stage_clear(void)
{
	atomic_set(&tdma_staged_pack, 0);
}

bool tdma_layout_withdrawn(void)
{
	return atomic_get(&tdma_withdrawn) != 0;
}

uint8_t tdma_get_config_epoch(void)
{
#if CONFIG_CONNECTION_TDMA
	return TDMA_UNPACK_EPOCH((uint32_t)atomic_get(&tdma_cfg_pack));
#else
	return 0;
#endif
}

/* ---------------------------------------------------------------------------
 * Peer relay schedule queries. Every tracker derives every slot's data and
 * PING windows from receiver time and the shared layout, so a helper can
 * listen to another slot and a tracker that lost its uplink can listen to
 * other trackers' PING/PONG exchanges without any extra signalling.
 */
bool tdma_layout_get(struct tdma_layout *layout)
{
	struct tdma_config config;
	if (layout == NULL || !tdma_config_snapshot(&config)) {
		return false;
	}
	layout->slot_index = config.slot_index;
	layout->total_slots = tdma_profile_get() ? RELAY_V3_TOTAL_SLOTS : config.total_slots;
	layout->profile = tdma_profile_get();
	layout->slot_ticks = config.slot_ticks;
	layout->frame_ticks = config.frame_ticks;
	layout->epoch = config.epoch;
	return true;
}

bool tdma_network_now(uint32_t *ticks)
{
	struct tdma_config config;
	struct esb_network_clock clock;
	if (ticks == NULL || !tdma_config_snapshot(&config) || !tdma_clock_snapshot(&config, &clock)) {
		return false;
	}
	*ticks = clock.receiver_ticks;
	return true;
}

bool tdma_slot_data_window(uint8_t slot, uint32_t after, uint32_t *start)
{
	struct tdma_config config;
	if (start == NULL || !tdma_config_snapshot(&config) || slot >= config.total_slots) {
		return false;
	}
	if (tdma_profile_get()) {
		uint32_t frame = relay_frame_near(tdma_anchor_frame, tdma_anchor_start, after + 1U);
		uint32_t known;
		if (!relay_runtime_start(&relay_direct_anchor, frame, &known)) {
			return false;
		}
		uint32_t candidate = known + slot * RELAY_V3_SLOT_TICKS;
		if ((int32_t)(candidate - after) <= 0) {
			if (!relay_runtime_start(&relay_direct_anchor, frame + 1U, &known)) {
				return false;
			}
			candidate = known + slot * RELAY_V3_SLOT_TICKS;
		}
		/* A helper cannot infer f+3 from a stale fixed grid. Leave the
		 * radio to B acquisition when both announced starts are spent. */
		*start = candidate;
		return !tdma_window_quiet(*start, RELAY_V3_SLOT_TICKS);
	}
	uint32_t frame = after / config.frame_ticks;
	for (unsigned i = 0; i < TDMA_SEARCH_FRAMES + 1U; i++, frame = tdma_next_frame(frame, &config)) {
		if (!tdma_frame_eligible(frame, &config) || tdma_slot_frame_guarded(frame, slot, &config)) {
			continue;
		}
		uint32_t candidate = frame * config.frame_ticks + (uint32_t)slot * config.slot_ticks;
		if ((int32_t)(candidate - after) > 0 && (uint32_t)(candidate - after) <= TDMA_MAX_RELATIVE_TICKS) {
			*start = candidate;
			return true;
		}
	}
	return false;
}

bool tdma_slot_ping_window(uint8_t slot, uint32_t after, uint32_t *start, uint16_t *length)
{
	struct tdma_config config;
	if (start == NULL || length == NULL || !tdma_config_snapshot(&config)
	    || slot >= config.total_slots) {
		return false;
	}
	uint32_t period = tdma_ping_period_frames(config.frame_ticks);
	uint32_t phase = tdma_ping_phase_frame(slot, config.total_slots, period);
	uint32_t frame = after / config.frame_ticks;
	uint32_t candidate = frame - frame % period + phase;
	for (unsigned i = 0; i < 3; i++, candidate += period) {
		if (candidate >= tdma_last_frame(&config) || !tdma_ping_frame_eligible(candidate, slot, &config)) {
			continue;
		}
		uint32_t window = candidate * config.frame_ticks + (uint32_t)slot * config.slot_ticks;
		if ((int32_t)(window - after) > 0 && (uint32_t)(window - after) <= TDMA_MAX_RELATIVE_TICKS) {
			*start = window;
			*length = (uint16_t)(config.slot_ticks * TDMA_PING_WINDOW_SLOTS
				+ TDMA_SLOT_TARGET_OFFSET + TDMA_PING_LATE_TOLERANCE_TICKS);
			return true;
		}
	}
	return false;
}

bool tdma_wait_network_tick(uint32_t target)
{
#ifdef CONFIG_RELAY_FRAME_LEDGER
	ledger_wait_lateness = -32768;
#endif
	struct tdma_config config;
	struct esb_network_clock clock;
	if (!tdma_config_snapshot(&config) || !tdma_clock_snapshot(&config, &clock)) {
		return false;
	}
	/* Reached or just passed: callers judge lateness against their own
	 * window. The stream's second wait sits 2 ticks after the first, with
	 * the frame build in between, and dropped ~5 % of frames (all of them
	 * while a staged layout slowed each clock snapshot) when it refused. */
	if ((int32_t)(target - clock.receiver_ticks) <= 0) {
#ifdef CONFIG_RELAY_FRAME_LEDGER
		ledger_wait_lateness = MIN(clock.receiver_ticks - target, 32767U);
#endif
		return true;
	}
	return tdma_wait_until_network_tick(&config, clock, target);
}
