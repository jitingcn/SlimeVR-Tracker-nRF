#include "frame_ledger.h"
/*
 * Tracker side of receiver-coordinated peer relay. See relay.h.
 *
 * Profile 1 (v3) uses B watch assignments, a three-capture-frame cache and
 * granted repair slots. The relay thread owns peer windows; the connection
 * thread owns B windows and submits repairs within the unchanged deadline.
 *
 * Profile-0 compatibility timing budget of one relayed frame (receiver ticks, 1 tick = 30.5 us,
 * slot width S >= 16, frame F = total_slots * S):
 *   child  TX admitted at slot pos [2, S - reserve], on air ~4 ticks (17 B)
 *   head   RX from slot start - RELAY_RX_LEAD_TICKS (40 us ramp), radio
 *          disables itself at the child's END; thread collects at slot end
 *          (+ RELAY_RX_TAIL_US if a frame is still arriving)
 *   record valid until the child's next slot start (capture window + F), and
 *          is forwarded only while a full bundle sent now ends before then
 *          (RELAY_FORWARD_GUARD_TICKS), so a relayed copy always reaches the
 *          receiver before the child's next frame and can never be mistaken
 *          for newer data
 *   head   forwards in its own slot (bundled, <= 1 frame later) or in its
 *          forwarding slot at the end of the same frame
 * Added latency is therefore below one frame (5.4 ms at 11 trackers, S = 16).
 */
#include "relay.h"

#include "relay_radio.h"
#include "link_supervision.h"
#include "esb.h"
#include "tdma.h"
#include "connection.h"
#include "radio_capture.h"
#include "globals.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <string.h>

LOG_MODULE_REGISTER(relay, LOG_LEVEL_INF);

struct relay_record {
	uint8_t data[RELAY_RECORD_MAX_LEN];
	uint8_t length;
	uint8_t rssi;
	uint8_t child;
	uint8_t epoch;
	uint16_t capture_frame;
	uint32_t expiry; /* Profile 0: next child slot; v3 uses capture_frame age. */
	bool valid;
};

/* Assignment from the own PONG trailer; written in ISR, read under irq_lock. */
static struct {
	struct relay_pong_trailer trailer;
	int64_t received_ms;
	bool valid;
} assignment;

#define RELAY_CACHE_RECORDS (RELAY_MAX_TRACKERS * RELAY_V3_CACHE_FRAMES)
static struct relay_record records[RELAY_MAX_TRACKERS * RELAY_CACHE_MAX_FRAMES];
static struct relay_control control_plan;
static struct {
	uint8_t expected, heard;
	uint16_t rssi_x8;
} peer_counts[RELAY_MAX_TRACKERS];
static struct {
	uint32_t windows, captured, stored, grants, selected, queued, refused;
} repair_diag[RELAY_MAX_TRACKERS];
static uint8_t repair_pending[RELAY_MAX_TRACKERS];
static struct relay_peer_report pending_report;
static bool control_valid;
static bool control_account_pending;
static uint32_t relay_generation;
static bool maintenance_attempted;
uint32_t relay_maintenance_granted, relay_maintenance_late, relay_maintenance_lag_max;
static uint8_t control_helper;
static uint32_t repair_attempted;
static uint64_t repair_selected, repair_pending_mask;
static uint64_t repair_selected_high, repair_pending_high;

static bool overflow_pending;
static int64_t forward_needed_until_ms;

static struct {
	uint32_t head_windows;
	uint32_t head_captures;
	uint32_t head_foreign;
	uint32_t head_busy;
	uint32_t forwarded;
	uint32_t expired;
	uint32_t overflow;
	uint32_t replaced;
	uint32_t presence_windows;
	uint32_t presence_busy;
	uint32_t presence_rx;
	uint32_t lease_expired;
	uint32_t head_no_window;
	uint32_t head_rx_any;
	uint32_t head_crc_fail;
	uint32_t head_len_fail;
	uint32_t head_no_end;
	uint32_t head_address_timeout;
	uint8_t fail_rssi;
	uint8_t capture_rssi;
	uint8_t fail_length;
	uint8_t capture_length;
	uint16_t start_late_max;
} stats;

/* Last own trailer that carried children (diagnostics). */
static int64_t children_seen_ms;
static bool children_seen;

static K_SEM_DEFINE(relay_wake_sem, 0, 1);
static K_SEM_DEFINE(relay_control_done, 0, 1);

static void relay_control_account(void);

void relay_control_serviced(void)
{
	/* The connection owner has finished the unchanged repair deadlines. */
	relay_control_account();
	k_sem_give(&relay_control_done);
}

void relay_on_own_trailer(const struct relay_pong_trailer *trailer)
{
	unsigned key = irq_lock();
	bool was_head = assignment.valid && assignment.trailer.children_mask;
	assignment.trailer = *trailer;
	assignment.received_ms = k_uptime_get();
	assignment.valid = true;
	if (trailer->children_mask) {
		children_seen_ms = assignment.received_ms;
		children_seen = true;
	}
	irq_unlock(key);
	if (!was_head && trailer->children_mask) {
		k_sem_give(&relay_wake_sem);
	}
}

/* Caller holds IRQ lock. Invalid records never expose their old payload. */
static void relay_cache_invalidate(void)
{
	for (unsigned i = 0; i < RELAY_MAX_TRACKERS * RELAY_CACHE_MAX_FRAMES; i++) {
		records[i].valid = false;
	}
}

void relay_reset(void)
{
	if (control_account_pending) {
		relay_control_account();
	}
	unsigned key = irq_lock();
	relay_generation++;
	assignment.valid = false;
	control_valid = false;
	control_account_pending = false;
	relay_cache_invalidate();
	pending_report.count = 0;
	memset(peer_counts, 0, sizeof(peer_counts));
	overflow_pending = false;
	forward_needed_until_ms = 0;
	irq_unlock(key);
}

/* Snapshot a usable assignment: unexpired lease, same layout epoch as our
 * TDMA configuration, healthy own link, and a layout we can schedule in. */
#if defined(__GNUC__) && !defined(__clang__)
/* Attribute provenance: Capture handoff (0084d3a6) and repair-speed (dc676076) cohorts retained O3 as a group. No isolated attribute-only saving; retain pending A/B. */
__attribute__((optimize("O3")))
#endif
static bool relay_assignment_locked(struct relay_pong_trailer *out, struct tdma_layout *layout)
{
	if (!tdma_layout_get(layout)) {
		return false;
	}
	if (layout->profile) {
		uint32_t now;
		if (!control_valid || control_plan.epoch != layout->epoch || !tdma_network_now(&now) ||
		    (uint32_t)(now - control_plan.preamble_ticks) >
			RELAY_V3_CACHE_FRAMES * RELAY_V3_FRAME_TICKS_MAX) {
			return false;
		}
		uint16_t children = 0;
		for (uint8_t child = 0; child < RELAY_MAX_TRACKERS; child++) {
			if (((control_plan.primary_valid & (1U << child)) &&
			     (control_plan.helpers[child] & 15U) == control_helper) ||
			    ((control_plan.backup_valid & (1U << child)) &&
			     (control_plan.helpers[child] >> 4) == control_helper)) {
				children |= 1U << child;
			}
			if ((control_plan.survey & (1U << child)) &&
			    relay_can_overhear(control_plan.members, control_helper, child)) {
				children |= 1U << child;
			}
		}
		*out = (struct relay_pong_trailer){.epoch = layout->epoch,
						   .children_mask = children,
						   .member_mask = control_plan.members,
						   .forward_slot = RELAY_NO_SLOT};
		return children != 0;
	}
	unsigned key = irq_lock();
	bool valid = assignment.valid;
	struct relay_pong_trailer trailer = assignment.trailer;
	int64_t age = k_uptime_get() - assignment.received_ms;
	if (valid && age > RELAY_HEAD_LEASE_MS) {
		assignment.valid = false;
		valid = false;
		if (trailer.children_mask) {
			stats.lease_expired++;
		}
	}
	irq_unlock(key);
	if (!valid || trailer.epoch != layout->epoch || !trailer.children_mask
	    || esb_link_state() != LINK_HEALTHY) {
		return false;
	}
	*out = trailer;
	return true;
}

static bool relay_assignment_get(struct relay_pong_trailer *out, struct tdma_layout *layout, uint32_t *generation)
{
	unsigned key = irq_lock();
	bool valid = relay_assignment_locked(out, layout);
	if (valid && generation) *generation = relay_generation;
	irq_unlock(key);
	return valid;
}

bool relay_head_active(void)
{
	struct relay_pong_trailer trailer;
	struct tdma_layout layout;
	return relay_assignment_get(&trailer, &layout, NULL);
}

int relay_forward_slot(void)
{
	struct relay_pong_trailer trailer;
	struct tdma_layout layout;
	if (!relay_assignment_get(&trailer, &layout, NULL) || trailer.forward_slot == RELAY_NO_SLOT) {
		return -1;
	}
	/* A forwarding slot lies after every direct member's slot. Anything else
	 * would collide with a tracker and is ignored (bundle in own slot). */
	uint8_t direct = relay_mask_count(trailer.member_mask);
	if (trailer.forward_slot < direct || trailer.forward_slot >= layout.total_slots
	    || trailer.forward_slot == layout.slot_index) {
		return -1;
	}
	return trailer.forward_slot;
}

#if defined(__GNUC__) && !defined(__clang__)
/* Attribute provenance: Capture handoff (0084d3a6) and repair-speed (dc676076) cohorts retained O3 as a group. No isolated attribute-only saving; retain pending A/B. */
__attribute__((optimize("O3")))
#endif
static void relay_store(uint8_t child, const struct relay_rx_frame *frame, uint8_t epoch, uint32_t expiry,
	const struct tdma_layout *snapshot, uint16_t captured, uint32_t generation)
{
	unsigned key = irq_lock();
	if (generation != relay_generation || (snapshot->profile &&
	    (!control_valid || control_plan.epoch != epoch))) {
		irq_unlock(key);
		return;
	}
	const struct tdma_layout layout = *snapshot;
	struct relay_record *slot = NULL;
	for (unsigned i = 0;
		 i < (layout.profile ? RELAY_MAX_TRACKERS * relay_control_cache_frames(&control_plan) : RELAY_MAX_CHILDREN);
		 i++) {
		if (records[i].valid && records[i].child == child &&
		    (!layout.profile || records[i].capture_frame == captured)) {
			slot = &records[i];
			stats.replaced++; /* not forwarded within a frame: superseded */
			break;
		}
	}
	for (unsigned i = 0;
		 slot == NULL
		 && i < (layout.profile ? RELAY_MAX_TRACKERS * relay_control_cache_frames(&control_plan) : RELAY_MAX_CHILDREN);
		 i++) {
		if (!records[i].valid) {
			slot = &records[i];
		}
	}
	if (slot != NULL) {
		memcpy(slot->data, frame->data, frame->length);
		slot->length = frame->length;
		slot->rssi = frame->rssi;
		slot->child = child;
		slot->epoch = epoch;
		slot->capture_frame = captured;
		slot->expiry = expiry;
		slot->valid = true;
		repair_diag[child].stored++;
	}
	irq_unlock(key);
}

/* Drop records that can no longer reach the receiver before their child's
 * next frame (RELAY_FORWARD_GUARD_TICKS), or that belong to another layout.
 * Caller holds irq_lock. */
static void relay_expire_locked(uint32_t now, const struct tdma_layout *layout)
{
	uint16_t frame = layout->profile ? (uint16_t)relay_frame_at_tick(
		relay_control_origin(control_plan.preamble_ticks, control_plan.frame), now) : 0;
	for (unsigned i = 0;
		 i < (layout->profile ? RELAY_MAX_TRACKERS * relay_control_cache_frames(&control_plan) : RELAY_MAX_CHILDREN);
		 i++) {
		if (!records[i].valid) {
			continue;
		}
		int32_t remaining = (int32_t)(records[i].expiry - now);
		if (records[i].epoch != layout->epoch || (!layout->profile && remaining <= (int32_t)RELAY_FORWARD_GUARD_TICKS)
			|| (layout->profile
				&& (uint16_t)(frame - records[i].capture_frame) >= relay_control_cache_frames(&control_plan))) {
			records[i].valid = false;
			stats.expired++;
		}
	}
}

bool relay_records_pending(void)
{
	struct tdma_layout layout;
	uint32_t now;
	if (!tdma_layout_get(&layout) || !tdma_network_now(&now)) {
		return false;
	}
	bool pending = false;
	if (layout.profile) {
		uint32_t target;
		uint8_t slot;
		if (relay_next_repair(&target, &slot) &&
		    !control_plan.grants[slot - RELAY_V3_REPAIR_FIRST_SLOT].children) {
			for (unsigned c = 0; c < RELAY_MAX_TRACKERS; c++) {
				pending |= peer_counts[c].expected != 0;
			}
		}
	}
	unsigned key = irq_lock();
	relay_expire_locked(now, &layout);
	for (unsigned i = 0;
		 i < (layout.profile ? RELAY_MAX_TRACKERS * relay_control_cache_frames(&control_plan) : RELAY_MAX_CHILDREN);
		 i++) {
		pending |= records[i].valid;
	}
	irq_unlock(key);
	return pending;
}

uint8_t relay_info_caps(void)
{
	unsigned key = irq_lock();
	bool needed = k_uptime_get() < forward_needed_until_ms;
	irq_unlock(key);
	return RELAY_INFO_CAPS | (needed ? RELAY_CAP_FORWARD_REQUEST : 0U);
}

size_t relay_build_bundle(uint8_t *buf, size_t own_len, size_t capacity, uint8_t head_id)
{
	struct tdma_layout layout;
	uint32_t now;
	if (!tdma_layout_get(&layout) || !tdma_network_now(&now)) {
		return 0;
	}
	uint8_t own[RELAY_FRAME_MAX];
	if (own_len > sizeof(own)) {
		return 0;
	}
	memcpy(own, buf, own_len);
	uint8_t out[RELAY_FRAME_MAX];
	struct relay_bundle_builder builder;
	size_t length = 0;
	unsigned key = irq_lock();
	relay_expire_locked(now, &layout);
	if (relay_bundle_begin(&builder, out, capacity, head_id, own, (uint8_t)own_len, overflow_pending)) {
		bool dropped = false;
		for (unsigned i = 0;
			 i < (layout.profile ? RELAY_MAX_TRACKERS * relay_control_cache_frames(&control_plan) : RELAY_MAX_CHILDREN);
			 i++) {
			if (!records[i].valid) {
				continue;
			}
			if (relay_bundle_add(&builder, records[i].data, records[i].length, records[i].rssi)) {
				records[i].valid = false;
			} else {
				dropped = true;
			}
		}
		length = relay_bundle_finish(&builder);
		if (dropped) {
			/* A zero-record bundle cannot carry its overflow flag. Advertise
			 * the request in ordinary info traffic, which needs no spare room. */
			overflow_pending = true;
			forward_needed_until_ms = k_uptime_get() + 2000;
			stats.overflow++;
		}
		if (length) {
			stats.forwarded += builder.records;
			/* Report a drop on the next bundle the receiver can see. */
			overflow_pending = dropped;
		}
	} else {
		for (unsigned i = 0;
			 i < (layout.profile ? RELAY_MAX_TRACKERS * relay_control_cache_frames(&control_plan) : RELAY_MAX_CHILDREN);
			 i++) {
			if (records[i].valid) {
				overflow_pending = true;
				forward_needed_until_ms = k_uptime_get() + 2000;
				stats.overflow++;
				break;
			}
		}
	}
	irq_unlock(key);
	if (length) {
		memcpy(buf, out, length);
	}
	return length;
}

/* ------------------------------------------------------------------------ */

static bool relay_clock_held;

static void relay_clock(bool hold)
{
	if (hold != relay_clock_held && esb_relay_clock(hold) == 0) {
		relay_clock_held = hold;
	}
}

static uint32_t net_ticks_of_us(uint32_t us)
{
	return (uint32_t)(((uint64_t)us * 32768U + 999999U) / 1000000U);
}

/* One RX window in a child's slot. Returns true if a window was attempted. */
/* Survey evidence for the next peer report: windows opened, frames heard. */
static void peer_count(uint8_t child, bool heard, uint8_t rssi)
{
	unsigned key = irq_lock();
	if (heard) {
		peer_counts[child].heard++;
		repair_diag[child].captured++;
		peer_counts[child].rssi_x8 = relay_rssi_avg_x8(peer_counts[child].rssi_x8, rssi);
	} else {
		if (peer_counts[child].expected == RELAY_PEER_OBSERVATIONS) {
			for (unsigned i = 0; i < pending_report.count; i++) {
				if (pending_report.entries[i].child == child) {
					pending_report.entries[i].expected /= 2U;
					pending_report.entries[i].heard /= 2U;
				}
			}
			peer_counts[child].expected /= 2U;
			peer_counts[child].heard /= 2U;
		}
		peer_counts[child].expected++;
		repair_diag[child].windows++;
	}
	irq_unlock(key);
}

#if defined(__GNUC__) && !defined(__clang__)
/* Attribute provenance: Capture handoff (0084d3a6) and repair-speed (dc676076) cohorts retained O3 as a group. No isolated attribute-only saving; retain pending A/B. */
__attribute__((optimize("O3")))
#endif
static bool relay_head_window(const struct relay_pong_trailer *trailer, const struct tdma_layout *layout, uint32_t generation)
{
	unsigned snapshot_key = irq_lock();
	if (generation != relay_generation) { irq_unlock(snapshot_key); return false; }
	uint32_t session = control_plan.session;
	uint32_t control_frame = control_plan.frame;
	uint32_t control_start = relay_control_start(control_plan.preamble_ticks);
	bool valid = !layout->profile || (control_valid && control_plan.epoch == layout->epoch);
	irq_unlock(snapshot_key);
	if (!valid) return false;
	uint32_t now;
	if (!tdma_network_now(&now)) {
		return false;
	}
	/* Earliest upcoming data window among the children. A window that began
	 * at most RELAY_RX_LEAD_TICKS ago is still catchable (child pos >= 2). */
	uint32_t after = now - RELAY_RX_LEAD_TICKS - 1U;
	uint32_t best_start = 0;
	int best_child = -1;
	for (uint8_t child = 0; child < RELAY_MAX_TRACKERS; child++) {
		if (!(trailer->children_mask & (1U << child))) {
			continue;
		}
		int slot = relay_member_slot(trailer->member_mask, child);
		uint32_t start;
		if (slot < 0 || slot >= layout->total_slots || slot == layout->slot_index
		    || !tdma_slot_data_window((uint8_t)slot, after, &start)) {
			continue;
		}
		if (best_child < 0 || (int32_t)(start - best_start) < 0) {
			best_start = start;
			best_child = child;
		}
	}
	if (best_child < 0) {
		return false;
	}
	if (layout->profile) {
		uint32_t f = relay_frame_near(control_frame, control_start, best_start);
		if (f <= control_frame || f - control_frame > RELAY_V3_CACHE_FRAMES) {
			return false;
		}
	}
	uint32_t rx_at = best_start - RELAY_RX_LEAD_TICKS;
	uint8_t pipe = 1U + (uint8_t)best_child % 7U;
	uint32_t capture_end = best_start + layout->slot_ticks;
	uint32_t tail_end = capture_end + RELAY_V3_CAPTURE_TAIL_TICKS;
	if (layout->profile &&
	    relay_member_slot(trailer->member_mask, best_child) == RELAY_V3_DIRECT_SLOTS - 1U) {
		/* B owns the radio from its existing RX lead, including when
		 * D10 cannot acquire the radio or never receives an END. */
		capture_end -= RELAY_RX_LEAD_TICKS;
		tail_end = capture_end;
	}
	/* Prepare bounds before the unchanged wake; leave only acquisition and
	 * radio setup between that wake and RXEN. */
	if ((int32_t)(rx_at - now) > 0 && !tdma_wait_network_tick(rx_at)) {
		return false;
	}
	stats.head_windows++;
	/* A long own bundle can finish just after the next slot's RX lead.
	 * Retry briefly: the child's earliest preamble is offset 2 + TX ramp,
	 * so RXEN by offset 1 still leaves one tick of ramp/clock margin. */
	bool acquired = false, have_now;
	unsigned rx_key;
#ifdef CONFIG_RELAY_FRAME_LEDGER
	bool attempted = false;
#endif
	for (;;) {
		/* Keep the acquisition clock sample, ownership and RXEN in one
		 * guarded interval. Release between busy retries so TX can finish. */
		rx_key = irq_lock();
		have_now = tdma_network_now(&now);
		if (!have_now || (int32_t)(now - (best_start + 1U)) > 0) {
			irq_unlock(rx_key);
			break;
		}
#ifdef CONFIG_RELAY_FRAME_LEDGER
		attempted = true;
#endif
		if (esb_raw_rx_begin()) {
			acquired = true;
			break;
		}
		irq_unlock(rx_key);
		k_busy_wait(5);
	}
	if (!acquired) {
		if (have_now && (int32_t)(now - rx_at) > (int32_t)stats.start_late_max)
			stats.start_late_max = (uint16_t)MIN(now - rx_at, UINT16_MAX);
		if (layout->profile) peer_count(best_child, false, 0);
		stats.head_busy++;
		relay_ledger_tick(best_start, best_child, LEDGER_MISS, !have_now ? 6 : attempted ? 1 : 5);
		(void)tdma_wait_network_tick(capture_end);
		return true;
	}
	if (layout->profile) {
		if (radio_capture_init() != 0) {
			irq_unlock(rx_key);
			peer_count(best_child, false, 0);
			esb_raw_rx_end();
			return true;
		}
		uint32_t previous_end;
		(void)radio_capture_rx_end(&previous_end);
		radio_capture_set_enabled(true);
	}
	if (!relay_radio_start(esb_get_radio_channel(), (uint8_t)(1U << pipe), true)) {
		irq_unlock(rx_key);
		relay_clock(false);
		return true;
	}
	irq_unlock(rx_key);
	/* Accounting follows RXEN; the sample is the actual acquisition check. */
	if (have_now && (int32_t)(now - rx_at) > (int32_t)stats.start_late_max)
		stats.start_late_max = (uint16_t)MIN(now - rx_at, UINT16_MAX);
#ifdef CONFIG_RELAY_LEDGER_TIMING
	/* Sample after RXEN so instrumentation cannot delay radio startup. */
	uint32_t rx_started;
	int16_t rx_start = tdma_network_now(&rx_started) ? (int32_t)(rx_started - best_start) : -32768;
#endif
	if (layout->profile) {
		peer_count(best_child, false, 0);
		while (!relay_radio_ended() && tdma_network_now(&now) && (int32_t)(capture_end - now) > 0) {
			k_busy_wait(5);
		}
	} else {
		(void)tdma_wait_network_tick(best_start + layout->slot_ticks);
	}
	for (uint32_t waited = 0;
	     relay_radio_busy() &&
	     waited < (layout->profile ? RELAY_V3_CAPTURE_TAIL_TICKS * 1000000U / 32768U : RELAY_RX_TAIL_US);
	     waited += 10) {
		if (layout->profile && (!tdma_network_now(&now) || (int32_t)(tail_end - now) <= 0)) {
			break;
		}
		k_busy_wait(10);
	}
	struct relay_rx_frame frame;
	bool addressed = relay_radio_addressed();
	bool ended = relay_radio_ended();
	uint8_t raw_length = 0, raw_rssi = 0;
	bool crc_ok = false;
	if (ended) {
		relay_radio_peek(&raw_length, &crc_ok, &raw_rssi);
	}
#ifdef CONFIG_RELAY_LEDGER_TIMING
	uint16_t radio_observation = relay_radio_observation();
#endif
	bool captured = relay_radio_take(&frame);
	if (captured && layout->profile) {
		captured = radio_capture_rx_end(&frame.end_kernel_ticks);
	}
	if (!relay_radio_stop()) { relay_clock(false); return true; }
	esb_raw_rx_end();
#ifdef CONFIG_RELAY_LEDGER_TIMING
	relay_ledger_tick(best_start, best_child, LEDGER_CAPTURE_START, rx_start);
	relay_ledger_tick(best_start, best_child, LEDGER_CAPTURE_RADIO_STATE, radio_observation);
#endif
	stats.head_rx_any += addressed;
	stats.head_no_end += !ended;
	stats.head_address_timeout += addressed && !ended;
	if (!captured) {
		relay_ledger_tick(best_start, best_child, LEDGER_MISS, ended ? (crc_ok ? 4 : 3) : 2);
		if (ended) {
			if (crc_ok) {
				stats.head_len_fail++;
			} else {
				stats.head_crc_fail++;
			}
			stats.fail_rssi = raw_rssi;
			stats.fail_length = raw_length;
		}
		return true;
	}
	stats.capture_rssi = frame.rssi;
	stats.capture_length = frame.length;
	if (!relay_frame_is_pose(frame.data, frame.length, (uint8_t)best_child) ||
	    frame.length >
		(layout->profile
		     ? relay_direct_payload_max(layout->profile,
						(uint8_t)relay_member_slot(trailer->member_mask, best_child))
		     : RELAY_RECORD_MAX_LEN)) {
		stats.head_foreign++;
		return true;
	}
	if (frame.pipe != pipe) {
		stats.head_foreign++;
		return true;
	}
	if (layout->profile && (!control_valid || session != control_plan.session)) {
		return true;
	}
	if (layout->profile) {
		peer_count(best_child, true, frame.rssi);
	}
	relay_ledger_tick(best_start, best_child, LEDGER_CAPTURE, -(int16_t)frame.rssi);
	stats.head_captures++;
	relay_store((uint8_t)best_child, &frame, layout->epoch, best_start + layout->frame_ticks, layout,
		(uint16_t)relay_frame_near(control_frame, control_start, best_start), generation);
	connection_relay_wake();
	return true;
}

/* One continuous RX window over another tracker's PING exchange. */
static void relay_presence_window(const struct tdma_layout *layout)
{
	relay_clock(false);
	uint32_t now;
	if (!tdma_network_now(&now)) {
		return;
	}
	uint32_t best_start = 0;
	uint16_t best_length = 0;
	bool found = false;
	for (uint8_t slot = 0; slot < layout->total_slots; slot++) {
		uint32_t start;
		uint16_t length;
		if (slot == layout->slot_index || !tdma_slot_ping_window(slot, now + 2U, &start, &length)) {
			continue;
		}
		if (!found || (int32_t)(start - best_start) < 0) {
			best_start = start;
			best_length = length;
			found = true;
		}
	}
	if (!found || (uint32_t)(best_start - now) > 40000U) {
		return;
	}
	uint32_t lead = net_ticks_of_us(RELAY_PRESENCE_CLOCK_LEAD_US);
	if ((uint32_t)(best_start - now) > lead && !tdma_wait_network_tick(best_start - lead)) {
		return;
	}
	relay_clock(true);
	if (!relay_clock_held || !tdma_network_now(&now)
	    || (int32_t)(now - (best_start - 1U)) > 0
	    || ((int32_t)(best_start - 1U - now) > 0 && !tdma_wait_network_tick(best_start - 1U))) {
		relay_clock(false);
		return;
	}
	stats.presence_windows++;
	if (!esb_raw_rx_begin()) {
		stats.presence_busy++;
		relay_clock(false);
		return;
	}
	if (!relay_radio_start(esb_get_radio_channel(), 0xFEU, false)) {
		relay_clock(false);
		return;
	}
	uint32_t end = best_start + best_length + net_ticks_of_us(RELAY_RX_TAIL_US);
	struct relay_rx_frame frame;
	bool pong = false;
	while (!pong && tdma_network_now(&now) && (int32_t)(end - now) > 0) {
		if (relay_radio_poll(&frame)) {
			pong = esb_presence_frame(&frame);
			stats.presence_rx += pong;
		} else {
			k_busy_wait(5);
		}
	}
	relay_radio_stop();
	esb_raw_rx_end();
	relay_clock(false);
}

static void relay_thread(void)
{
	int64_t next_presence = 0;
	for (;;) {
		struct relay_pong_trailer trailer;
		struct tdma_layout layout;
		uint32_t generation;
		if (relay_assignment_get(&trailer, &layout, &generation)) {
			relay_clock(true);
			if (!relay_head_window(&trailer, &layout, generation)) {
				stats.head_no_window++;
				k_msleep(1);
			} else if (layout.profile) {
				/* After D10, let the lower-priority connection thread open
				 * B before searching the next frame's helper assignments. */
				uint32_t now;
				unsigned key = irq_lock();
				uint32_t anchor_frame = control_plan.frame;
				uint32_t anchor_preamble = control_plan.preamble_ticks;
				bool valid = control_valid && control_plan.epoch == layout.epoch;
				irq_unlock(key);
				uint32_t origin = relay_control_origin(anchor_preamble, anchor_frame);
				if (valid && tdma_network_now(&now)) {
					uint32_t frame = relay_frame_near(anchor_frame, relay_control_start(anchor_preamble), now);
					uint32_t phase = now - relay_frame_start(origin, frame);
					if (phase >= (RELAY_V3_DIRECT_SLOTS - 1U) * RELAY_V3_SLOT_TICKS &&
					    phase < RELAY_V3_REPAIR_START) {
						/* Do not wake into the connection owner's R0 preparation.
						 * It releases us after servicing its grants. A missing B
						 * still bounds this wait before the next direct frame. */
						uint32_t ticks = relay_frame_start(origin, frame + 1U) - now -
							RELAY_RX_LEAD_TICKS;
						k_sem_reset(&relay_control_done);
						(void)k_sem_take(&relay_control_done,
							K_USEC((ticks * 1000000U + 32767U) / 32768U));
					}
				}
			}
			continue;
		}
		if (tdma_profile_get()) {
			relay_clock(false);
			k_msleep(1);
			continue;
		}
		if (esb_link_wants_presence() && tdma_layout_get(&layout)) {
			int64_t now = k_uptime_get();
			if (now >= next_presence) {
				next_presence = now + RELAY_PRESENCE_INTERVAL_MS;
				relay_presence_window(&layout);
				continue;
			}
			relay_clock(false);
			(void)k_sem_take(&relay_wake_sem, K_MSEC(next_presence - now));
			continue;
		}
		relay_clock(false);
		(void)k_sem_take(&relay_wake_sem, K_MSEC(100));
	}
}

K_THREAD_DEFINE(relay_thread_id, 1024, relay_thread, NULL, NULL, NULL, RELAY_THREAD_PRIORITY, 0, 0);

static void relay_put16(uint8_t *out, uint32_t value)
{
	out[0] = (uint8_t)(value >> 8);
	out[1] = (uint8_t)value;
}

size_t relay_diag_build(uint8_t *buf, size_t capacity, uint8_t tracker_id)
{
	static int64_t last_ms;
	static uint32_t last_presence;
	if (capacity < RELAY_DIAG_LEN || !esb_receiver_relay_aware()) {
		return 0;
	}
	int64_t now = k_uptime_get();
	if (now - last_ms < RELAY_DIAG_INTERVAL_MS) {
		return 0;
	}
	unsigned key = irq_lock();
	struct relay_pong_trailer trailer = assignment.trailer;
	bool lease = assignment.valid && now - assignment.received_ms <= RELAY_HEAD_LEASE_MS;
	bool assigned = children_seen && now - children_seen_ms < 5000;
	irq_unlock(key);
	bool presence = stats.presence_windows != last_presence;
	if (!assigned && !presence) {
		return 0;
	}
	last_ms = now;
	last_presence = stats.presence_windows;
	struct tdma_layout layout = {0};
	bool have_layout = tdma_layout_get(&layout);
	struct relay_pong_trailer active;
	struct tdma_layout unused;
	bool head = relay_assignment_get(&active, &unused, NULL);
	enum link_state link = esb_link_state();
	memset(buf, 0, RELAY_DIAG_LEN);
	buf[0] = RELAY_DIAG_TYPE;
	buf[1] = tracker_id;
	buf[2] = (assigned ? RELAY_DIAG_F_ASSIGNED : 0) | (lease ? RELAY_DIAG_F_LEASE : 0)
		| (have_layout && trailer.epoch == layout.epoch ? RELAY_DIAG_F_EPOCH : 0)
		| (head ? RELAY_DIAG_F_HEAD : 0) | (have_layout ? RELAY_DIAG_F_LAYOUT : 0)
		| (uint8_t)((unsigned)link << RELAY_DIAG_LINK_SHIFT);
	buf[3] = trailer.epoch;
	buf[4] = layout.epoch;
	buf[5] = layout.slot_index;
	buf[6] = layout.total_slots;
	buf[7] = layout.slot_ticks;
	relay_put16(&buf[8], trailer.children_mask);
	relay_put16(&buf[10], trailer.member_mask);
	unsigned peak_key = irq_lock();
	uint16_t start_late = stats.start_late_max;
	stats.start_late_max = 0;
	irq_unlock(peak_key);
	const uint32_t counters[RELAY_DIAG_COUNTERS] = {
		[RELAY_DIAG_WINDOWS] = stats.head_windows,
		[RELAY_DIAG_NO_WINDOW] = stats.head_no_window,
		[RELAY_DIAG_BUSY] = stats.head_busy,
		[RELAY_DIAG_RX_ANY] = stats.head_rx_any,
		[RELAY_DIAG_CRC_FAIL] = stats.head_crc_fail,
		[RELAY_DIAG_CAPTURES] = stats.head_captures,
		[RELAY_DIAG_FOREIGN] = stats.head_foreign,
		[RELAY_DIAG_FORWARDED] = stats.forwarded,
		[RELAY_DIAG_EXPIRED] = stats.expired,
		[RELAY_DIAG_PRESENCE] = stats.presence_windows,
		[RELAY_DIAG_PRESENCE_RX] = stats.presence_rx,
		[RELAY_DIAG_LEASE_EXPIRED] = stats.lease_expired,
		[RELAY_DIAG_LEN_FAIL] = stats.head_len_fail,
		[RELAY_DIAG_RSSI] = ((uint32_t)stats.fail_rssi << 8) | stats.capture_rssi,
		[RELAY_DIAG_START_LATE] = start_late,
		[RELAY_DIAG_LENGTHS] = ((uint32_t)stats.fail_length << 8) | stats.capture_length,
	};
	for (unsigned i = 0; i < RELAY_DIAG_COUNTERS; i++) {
		relay_put16(&buf[RELAY_DIAG_HEADER_LEN + 2U * i], counters[i]);
	}
	return RELAY_DIAG_LEN;
}

void relay_print_status(void)
{
	unsigned key = irq_lock();
	uint32_t frame = control_plan.frame;
	uint8_t direct = control_plan.direct_count, repairs = control_plan.repair_count;
	uint8_t next_repairs = relay_control_next_repairs(&control_plan), width = control_plan.slot_ticks;
	uint8_t next2_repairs = relay_control_next2_repairs(&control_plan);
	uint16_t cap = control_plan.tps_max;
	bool valid = control_valid;
	irq_unlock(key);
	printk(
		"relay B received: valid=%u frame=%u direct=%u R=%u next_R=%u next2_R=%u slot_ticks=%u tps_max=%u\n",
		valid,
		frame,
		direct,
		repairs,
		next_repairs,
		next2_repairs,
		width,
		cap
	);
	struct relay_pong_trailer trailer;
	struct tdma_layout layout;
	bool head = relay_assignment_get(&trailer, &layout, NULL);
	static const char *const states[] = {"healthy", "degraded", "lost"};
	enum link_state link = esb_link_state();
	printk("relay: link=%s head=%s children=0x%04x fwd_slot=%d\n",
		states[link], head ? "yes" : "no", head ? trailer.children_mask : 0,
		relay_forward_slot());
	printk("relay: no_end=%u address_timeout=%u\n", stats.head_no_end, stats.head_address_timeout);
	printk("relay: windows=%u no_window=%u busy=%u rx_any=%u crc_fail=%u len_fail=%u captures=%u foreign=%u rssi fail=-%u capture=-%u\n",
		stats.head_windows, stats.head_no_window, stats.head_busy, stats.head_rx_any,
		stats.head_crc_fail, stats.head_len_fail, stats.head_captures, stats.head_foreign,
		stats.fail_rssi, stats.capture_rssi);
	printk("relay: forwarded=%u expired=%u replaced=%u overflow=%u lease_expired=%u\n",
		stats.forwarded, stats.expired, stats.replaced, stats.overflow, stats.lease_expired);
	printk("relay: presence_windows=%u presence_busy=%u presence_rx=%u\n",
		stats.presence_windows, stats.presence_busy, stats.presence_rx);
	for (unsigned child = 0; child < RELAY_MAX_TRACKERS; child++) {
		if (!repair_diag[child].windows && !repair_diag[child].grants) continue;
		printk("repair: child=%u windows=%u captured=%u stored=%u grants=%u selected=%u queued=%u refused=%u\n",
			child, repair_diag[child].windows, repair_diag[child].captured,
			repair_diag[child].stored, repair_diag[child].grants, repair_diag[child].selected,
			repair_diag[child].queued, repair_diag[child].refused);
	}
	esb_print_link_status();
}

/* Counters and ledger rows do not participate in grant or cache decisions.
 * Keep them out of the B-to-R0 path. Only the connection owner publishes B
 * and services it; the snapshot also keeps reset/console readers harmless. */
static void relay_control_account(void)
{
	unsigned key = irq_lock();
	if (!control_account_pending) {
		irq_unlock(key);
		return;
	}
	struct relay_control snapshot = control_plan;
	uint8_t helper = control_helper;
	control_account_pending = false;
	irq_unlock(key);
	const struct relay_control *control = &snapshot;
#ifdef CONFIG_RELAY_FRAME_LEDGER
	for (unsigned child = 0; child < RELAY_MAX_TRACKERS; child++) {
		if (!ledger_child_enabled(child)) {
			continue;
		}
		uint8_t grants = 0;
		for (unsigned slot = 0; slot < control->repair_count; slot++) {
			if (control->grants[slot].owner == helper && (control->grants[slot].children & (1U << child))) {
				grants |= 1U << slot;
			}
		}
		for (unsigned age = 0; age < relay_control_cache_frames(control); age++) {
			ledger_note(
				control->session,
				control->frame - age,
				child,
				LEDGER_HELPER_B,
				(grants << 1) | !!(relay_control_missing(control, age) & (1U << child)),
				helper,
				age
			);
		}
	}
#endif
	for (unsigned slot = 0; slot < control->repair_count; slot++) {
		if (control->grants[slot].owner != helper) {
			continue;
		}
		for (unsigned child = 0; child < RELAY_MAX_TRACKERS; child++) {
			if (control->grants[slot].children & (1U << child)) {
				repair_diag[child].grants++;
			}
		}
	}
}

#if defined(__GNUC__) && !defined(__clang__)
/* Attribute provenance: Capture handoff (0084d3a6) and repair-speed (dc676076) cohorts retained O3 as a group. No isolated attribute-only saving; retain pending A/B. */
__attribute__((optimize("O3")))
#endif
void relay_on_control(const struct relay_control *control, uint8_t helper)
{
	/* Preserve accounting if a caller publishes another B without service. */
	if (control_account_pending) {
		relay_control_account();
	}
	unsigned key = irq_lock();
	if (!control_valid || control_plan.session != control->session ||
	    control_plan.epoch != control->epoch) {
		relay_generation++;
		/* The payload bytes are irrelevant once a cache entry is invalid. */
		relay_cache_invalidate();
		pending_report.count = 0;
		memset(peer_counts, 0, sizeof(peer_counts));
	}
	control_plan = *control;
	control_helper = helper;
	control_valid = true;
	control_account_pending = true;

	maintenance_attempted = false;
	if ((control->flags & RELAY_CONTROL_MAINTENANCE) && control->maintenance_owner == helper) {
		relay_maintenance_granted++;
	}
	repair_attempted = 0;
	repair_selected = repair_selected_high = 0;
	repair_pending_mask = repair_pending_high = 0;

	for (unsigned i = 0; i < RELAY_MAX_TRACKERS * RELAY_CACHE_MAX_FRAMES; i++) {
		struct relay_record *r = &records[i];
		uint16_t age = (uint16_t)(control->frame - r->capture_frame);
		if (r->valid
			&& (age >= relay_control_cache_frames(control)
				|| !(relay_control_missing(control, age) & (1U << r->child)))) {
			if (age >= relay_control_cache_frames(control)) {
				ledger_note(control_plan.session, control->frame - age, r->child, LEDGER_EXPIRE, 0, helper, age);
			}
			r->valid = false;
		}
	}
	irq_unlock(key);
	/* This does not change the head assignment. Waking the higher-priority
	 * RX worker here delays the connection owner's R0 preparation. Own
	 * trailers still wake that worker when its assignment changes. */
}

bool relay_next_maintenance(uint32_t *target)
{
	uint32_t now;
	if (!tdma_profile_get() || !control_valid || maintenance_attempted ||
	    !(control_plan.flags & RELAY_CONTROL_MAINTENANCE) ||
	    control_plan.maintenance_owner != control_helper || !tdma_network_now(&now)) {
		return false;
	}
	uint32_t start = relay_repair_start(control_plan.preamble_ticks, 0);
	if ((int32_t)(now - start) > (int32_t)relay_maintenance_lag_max) {
		relay_maintenance_lag_max = now - start;
	}
	if ((int32_t)(now - (start + 3U)) > 0) {
		maintenance_attempted = true;
		relay_maintenance_late++;
		return false;
	}
	uint32_t at = start + RELAY_V3_TX_OFFSET;
	*target = (int32_t)(at - now) < 0 ? now : at;
	return true;
}

bool relay_take_maintenance(void)
{
	uint32_t target;
	if (!relay_next_maintenance(&target)) {
		return false;
	}
	maintenance_attempted = true;
	return true;
}

#if defined(__GNUC__) && !defined(__clang__)
/* Attribute provenance: Capture handoff (0084d3a6) and repair-speed (dc676076) cohorts retained O3 as a group. No isolated attribute-only saving; retain pending A/B. */
__attribute__((optimize("O3")))
#endif
bool relay_next_repair(uint32_t *target, uint8_t *slot)
{
	uint32_t now;
	if (!tdma_profile_get() || !control_valid || !tdma_network_now(&now)) {
		return false;
	}
	for (uint8_t i = 0; i < control_plan.repair_count; i++) {
		uint32_t at = relay_repair_start(control_plan.preamble_ticks, i) + RELAY_V3_TX_OFFSET;
		if ((int32_t)(at + RELAY_V3_TX_OFFSET - now) >= 0 && !(repair_attempted & (1U << i)) &&
		    control_plan.grants[i].owner == control_helper) {
			*target = (int32_t)(at - now) < 0 ? now : at;
			*slot = RELAY_V3_REPAIR_FIRST_SLOT + i;
			return true;
		}
	}
	return false;
}

/* Preserve the narrow pool's last-chance rule as its physical R changes. */
static inline bool relay_final_narrow_grant(unsigned grant)
{
	return control_plan.repair_count <= 3U && grant + 1U == control_plan.repair_count;
}

#ifdef CONFIG_RELAY_FRAME_LEDGER
static struct { uint32_t frame; uint8_t child, age; } ledger_pending[RELAY_MAX_CHILDREN];
static unsigned ledger_pending_count;
#endif
#if defined(__GNUC__) && !defined(__clang__)
/* Attribute provenance: Capture handoff (0084d3a6) and repair-speed (dc676076) cohorts retained O3 as a group. No isolated attribute-only saving; retain pending A/B. */
__attribute__((optimize("O3")))
#endif
size_t relay_build_repair(uint8_t *buf, uint8_t slot)
{
	struct relay_repair repair = {
	    .helper = control_helper, .epoch = control_plan.epoch, .session = control_plan.session};
	unsigned key = irq_lock();
	unsigned grant = slot - RELAY_V3_REPAIR_FIRST_SLOT;
	repair_pending_mask = repair_pending_high = 0;
	pending_report.count = 0;
	#ifdef CONFIG_RELAY_FRAME_LEDGER
	ledger_pending_count = 0;
#endif
	memset(repair_pending, 0, sizeof(repair_pending));
	repair_attempted |= 1U << grant;
	if (!control_plan.grants[grant].children) {
		struct relay_peer_report report = {
			.helper = control_helper, .epoch = control_plan.epoch, .session = control_plan.session};
		uint16_t selected = 0;
		for (unsigned n = 0; n < RELAY_PEER_REPORT_ENTRIES; n++) {
			int best = -1;
			for (unsigned c = 0; c < RELAY_MAX_TRACKERS; c++) {
				if (!(selected & (1U << c)) && peer_counts[c].expected && (best < 0 || peer_counts[c].expected > peer_counts[best].expected)) {
					best = c;
				}
			}
			if (best < 0) break;
			report.entries[report.count++] = (struct relay_peer_observation){
				.child = best, .window_end = control_plan.frame,
				.expected = peer_counts[best].expected, .heard = peer_counts[best].heard,
				.rssi = peer_counts[best].rssi_x8 / 8U};
			selected |= 1U << best;
		}
		pending_report = report;
		size_t n = report.count ? relay_peer_report_encode(buf, &report) : 0;
		irq_unlock(key);
		return n;
	}
	size_t length = RELAY_REPAIR_HEADER_LEN;
	/* A newer pose must not obsolete an older repair held by another helper.
	 * Fill consecutive missing ages in one packet; a gap waits for the next
	 * B confirmation or the existing cache expiry, never a longer deadline. */
	/* Classify once instead of scanning all 48 records independently for
	 * each cache age. Ascending record indices preserve packet ordering. */
	uint64_t by_age[RELAY_CACHE_MAX_FRAMES] = {0}, high[RELAY_CACHE_MAX_FRAMES] = {0};
	uint16_t children = control_plan.grants[grant].children;
	for (unsigned i = 0; i < RELAY_MAX_TRACKERS * relay_control_cache_frames(&control_plan); i++) {
		const struct relay_record *r = &records[i];
		/* A narrow grant usually names one child. Reject unrelated records
		 * before computing their age or a 64-bit selection mask. */
		if (!r->valid || !(children & (1U << r->child))) {
			continue;
		}
		uint16_t age = (uint16_t)(control_plan.frame - r->capture_frame);
		if (age < relay_control_cache_frames(&control_plan)
			&& !((i < 64 ? repair_selected : repair_selected_high) & (1ULL << (i % 64U)))) {
			if (i < 64) {
				by_age[age] |= 1ULL << i;
			} else {
				high[age] |= 1ULL << (i - 64U);
			}
		}
	}
	uint16_t blocked = 0;
	for (int age = relay_control_cache_frames(&control_plan) - 1; age >= 0; age--) {
		uint16_t included = 0;
		uint64_t candidates = by_age[age];
		while ((candidates || high[age]) && repair.count < RELAY_MAX_CHILDREN) {
			unsigned i;
			if (candidates) {
				i = __builtin_ctzll(candidates);
				candidates &= candidates - 1U;
			} else {
				i = 64U + __builtin_ctzll(high[age]);
				high[age] &= high[age] - 1U;
			}
			struct relay_record *r = &records[i];
			/* Classification and selection share this IRQ lock. Only gap
			 * blocking and remaining packet space can reject a candidate. */
			if ((blocked & (1U << r->child)) || length + RELAY_REPAIR_RECORD_META_LEN + r->length > RELAY_FRAME_MAX) {
				continue;
			}
			repair.records[repair.count++] = (struct relay_repair_record){
			    .capture_frame = r->capture_frame,
			    .source = {.frame = r->data, .length = r->length, .rssi = r->rssi}};
			length += RELAY_REPAIR_RECORD_META_LEN + r->length;
			included |= 1U << r->child;
			if (i < 64) {
				repair_selected |= 1ULL << i;
			} else {
				repair_selected_high |= 1ULL << (i - 64U);
			}
			if (i < 64) {
				repair_pending_mask |= 1ULL << i;
			} else {
				repair_pending_high |= 1ULL << (i - 64U);
			}
			repair_diag[r->child].selected++;
			ledger_note(control_plan.session, control_plan.frame - age, r->child, LEDGER_SELECTED, -(int16_t)r->rssi, control_helper, age);
			repair_pending[r->child]++;
#ifdef CONFIG_RELAY_FRAME_LEDGER
            ledger_pending[ledger_pending_count].frame = control_plan.frame - age;
            ledger_pending[ledger_pending_count].child = r->child;
            ledger_pending[ledger_pending_count++].age = age;
#endif
		}
		/* An uncaptured age-two record cannot arrive after the final
		 * narrow-pool grant within the cache deadline. It must not suppress
		 * a younger copy held by that owner. Earlier grants and age-one
		 * gaps retain chronological blocking, including an R1 frame. */
		bool final_expiring_gap
			= age == (int)relay_control_cache_frames(&control_plan) - 1 && relay_final_narrow_grant(grant);
		if (!final_expiring_gap) {
			blocked |= relay_control_missing(&control_plan, age) & ~included;
		}
	}
	size_t n = repair.count ? relay_repair_encode(buf, &repair) : 0;
	irq_unlock(key);
	return n;
}

/* Submission counts records, not packets; no-ACK repair TX has no delivery ACK. */
void relay_repair_result(int result)
{
#ifdef CONFIG_RELAY_FRAME_LEDGER
 for (unsigned i=0; i<ledger_pending_count; i++) {
  ledger_note(control_plan.session, ledger_pending[i].frame, ledger_pending[i].child,
   LEDGER_REPAIR_WAIT_LATE, ledger_repair_wait_lateness, control_helper, ledger_pending[i].age);
#ifdef CONFIG_RELAY_LEDGER_TIMING
  ledger_note(control_plan.session, ledger_pending[i].frame, ledger_pending[i].child,
   LEDGER_REPAIR_BUILD_CYCLES, MIN(ledger_repair_build_cycles, 32767U), control_helper, ledger_pending[i].age);
  ledger_note(control_plan.session, ledger_pending[i].frame, ledger_pending[i].child,
   LEDGER_REPAIR_FINAL_CYCLES, MIN(ledger_repair_final_cycles, 32767U), control_helper, ledger_pending[i].age);
#endif
  if (result == -EAGAIN) {
   ledger_note(control_plan.session, ledger_pending[i].frame, ledger_pending[i].child,
    LEDGER_REPAIR_WINDOW, ledger_window_detail, control_helper, ledger_pending[i].age);
   ledger_note(control_plan.session, ledger_pending[i].frame, ledger_pending[i].child,
    LEDGER_REPAIR_REFUSAL, ledger_repair_refusal, control_helper, ledger_pending[i].age);
  }
  ledger_note(control_plan.session, ledger_pending[i].frame, ledger_pending[i].child,
   LEDGER_REPAIR, result, control_helper, ledger_pending[i].age);
 }
 ledger_pending_count = 0;
#endif
	unsigned key = irq_lock();
	if (result == 0) {
		for (unsigned i = 0; i < pending_report.count; i++) {
			const struct relay_peer_observation *p = &pending_report.entries[i];
			peer_counts[p->child].expected -= MIN(peer_counts[p->child].expected, p->expected);
			peer_counts[p->child].heard -= MIN(peer_counts[p->child].heard, p->heard);
			/* Retain the EMA when newer samples remain; it is not additive. */
			if (!peer_counts[p->child].expected && !peer_counts[p->child].heard)
				peer_counts[p->child].rssi_x8 = 0;
		}
	}
	pending_report.count = 0;
	/* A refused packet was never queued. Retain its records for another
	 * granted slot; the failed slot itself remains consumed. */
	if (result != 0) {
		repair_selected &= ~repair_pending_mask;
		repair_selected_high &= ~repair_pending_high;
	}
	repair_pending_mask = repair_pending_high = 0;
	for (unsigned child = 0; child < RELAY_MAX_TRACKERS; child++) {
		if (result == 0) {
			repair_diag[child].queued += repair_pending[child];
			stats.forwarded += repair_pending[child];
		}
		else repair_diag[child].refused += repair_pending[child];
	}
	memset(repair_pending, 0, sizeof(repair_pending));
	irq_unlock(key);
}

#ifdef CONFIG_RELAY_FRAME_LEDGER
void relay_ledger_tick(uint32_t tick, uint8_t child, uint8_t event, int16_t detail)
{
 if (!control_valid) return;
 ledger_note(control_plan.session, relay_frame_near(control_plan.frame, relay_control_start(control_plan.preamble_ticks), tick), child, event, detail, control_helper, 0);
}
#endif
