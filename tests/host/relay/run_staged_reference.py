#!/usr/bin/env python3
"""Staged 0->1 switch and 1->0 exit: the production PONG time-sync update,
esb_stage_from_trailer and TDMA admission for profile-0 neighbours, checked in
receiver time.

A running member keeps its profile-0 layout until activation. Its own PONG
announcing the switch must not move its clock off the reference its peers still
use, or adjacent slots overlap on air. Activation must still fall on the
receiver's tick, and promotion must leave the clock on receiver time for the control frame.

On exit, every member must return to that shared reference at the activation
tick. A member whose filter is still in warm-up (it joined by acquisition, and control frame
never advances update_count) must not be moved most of the way by its first own
PONG while a neighbour that has not PONGed yet stays put: its retried PING would
then reach the next unguarded owner's slot."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

import budget
from run_tdma import ADOPT, PREFIX, extract_block, function, root

tdma = (root / "src/connection/tdma.c").read_text()
esb = (root / "src/connection/esb.c").read_text()
esb_header = (root / "src/connection/esb.h").read_text()
tdma_header = (root / "src/connection/tdma.h").read_text()
network_ticks = (root / "src/connection/network_ticks.h").read_text()
pong = function(esb, "esb_receive_pong")

CASES = r'''
#define K_TRUE 5000000U  /* receiver ticks minus local ticks */
#define ACK_LAG 7U       /* PING TX END to ACK RX: the legacy reference's bias */
#define EPOCH 41U
#define SLOT_TICKS 16U
#define TOTAL 11U
#define FRAME (SLOT_TICKS * TOTAL)
#define AIR_US 109.50    /* longer of the two airtimes in the review */

static uint32_t true_ticks(void) { return now_ticks() + K_TRUE; }
static void set_true(uint32_t ticks) { set_ticks(ticks - K_TRUE); }

static void reset_tracker(uint8_t slot)
{
	tdma_cfg_pack = 0;
	tdma_staged_pack = 0;
	memset(&sync_filter, 0, sizeof(sync_filter));
	server_time_synced = false;
	channel_wait_normal = false;
	own_trailer_valid = false;
	tracker_id = slot;
	relay_timeline_clear(&relay_timeline_active);
	relay_direct_anchor.valid = false;
	tdma_last_admitted_frame = tdma_forward_admitted_frame = UINT32_MAX;
	set_ticks(1000000);
}

/* One own PING/PONG exchange at the current time, as the ESB library and the
 * receiver stamp it: PING RX equals PING TX END on the receiver's clock. */
static uint32_t own_pong(bool pending, uint32_t activation)
{
	uint32_t tx_end = now_ticks();
	uint32_t ping_rx = tx_end + K_TRUE;
	esb_last_ack_tx_end_ticks = (uint32_t)virtual_us;
	set_ticks(tx_end + ACK_LAG);
	own_trailer = (struct relay_pong_trailer){.epoch = EPOCH, .forward_slot = 255,
		.member_mask = 0x7ff, .pending = pending, .pending_epoch = EPOCH + 1,
		.pending_total = relay_geometry_code(11, 5), .pending_slot_ticks = 16,
		.pending_member_mask = 0x7ff,
		.activation_units = pending ? (uint16_t)((activation - ping_rx) / RELAY_ACTIVATION_UNIT_TICKS) : 0};
	own_trailer_valid = true;
	pong_sync(tx_end - 3, ping_rx, tx_end + ACK_LAG);
	/* The NORMAL block stages before it judges the tuple. */
	esb_stage_from_trailer(&own_trailer, ping_rx);
	/* The receiver's activation tick, in whole activation units. */
	return ping_rx + (uint32_t)own_trailer.activation_units * RELAY_ACTIVATION_UNIT_TICKS;
}

static void running_member(uint8_t slot)
{
	reset_tracker(slot);
	for (unsigned i = 0; i < 4; i++) {
		own_pong(false, 0);
		set_ticks(now_ticks() + 32768);
	}
	assert(server_time_synced && sync_filter.initialized);
	assert(tdma_update_config(slot, TOTAL, SLOT_TICKS, EPOCH));
}

/* Slot 5, told of a 0->1 switch activating about 3 s later, in slot 5 of a
 * frame on the receiver's grid. */
static uint32_t staged_member(void)
{
	running_member(5);
	return own_pong(true, (true_ticks() / FRAME + 3 * 32768 / FRAME) * FRAME + 5 * SLOT_TICKS + 9);
}

struct span { int32_t min, max; unsigned count; };

/* Every own-slot admission (production tdma_wait_for_slot and its final TX
 * check) reachable from each start tick in [from, from + frames * FRAME):
 * phase of the TX start against the slot on the receiver's grid. */
static struct span admissions(uint8_t slot, uint32_t from, unsigned frames, uint32_t quiet_end)
{
	struct span span = {INT32_MAX, INT32_MIN, 0};
	for (uint32_t t = from; t < from + frames * FRAME; t++) {
		set_true(t);
		tdma_last_admitted_frame = UINT32_MAX;
		if (!tdma_wait_for_slot(17)) {
			continue;
		}
		assert(tdma_slot_tx_valid(17));
		uint32_t start = true_ticks();
		if (quiet_end) {
			/* Nothing on air within the receiver-time quiet zone. */
			assert((int32_t)(start - (quiet_end - 2 * TDMA_ACTIVATION_QUIET_TICKS)) < 0
			       ? (int32_t)(start + 4 - (quiet_end - 2 * TDMA_ACTIVATION_QUIET_TICKS)) <= 0
			       : (int32_t)(start - quiet_end) >= 0);
		}
		int32_t phase = (int32_t)((start - slot * SLOT_TICKS) % FRAME);
		if (phase > (int32_t)FRAME / 2) {
			phase -= FRAME;
		}
		span.min = MIN(span.min, phase);
		span.max = MAX(span.max, phase);
		span.count++;
	}
	assert(span.count > 0);
	return span;
}

static void staged_switch_keeps_one_reference(void)
{
	/* Slot 4 has not had a PONG since the announcement; slot 5 has. */
	running_member(4);
	uint32_t window = (true_ticks() / FRAME + 4) * FRAME;
	struct span before = admissions(4, window, 3, 0);

	uint32_t activation = staged_member();
	struct span staged = admissions(5, window + 8 * FRAME, 3, 0);
	printf("slot 4 phase %d..%d, staged slot 5 phase %d..%d (receiver ticks)\n",
		before.min, before.max, staged.min, staged.max);
	double gap_us = (double)(SLOT_TICKS + staged.min - before.max) * 1e6 / 32768.0;
	printf("adjacent-slot TX start gap %.2f us, airtime %.2f us\n", gap_us, AIR_US);
	/* One reference: the same phase in receiver time, so slot 4's latest
	 * packet ends before slot 5's earliest begins. */
	assert(staged.min == before.min && staged.max == before.max);
	assert(gap_us >= AIR_US);

	/* Activation still falls on the receiver's tick: the quiet zone holds in
	 * receiver time, promotion waits for it, and the clock is then on
	 * receiver time, where control frame's +/-4-tick acceptance needs it. */
	uint32_t quiet_end = activation + TDMA_ACTIVATION_QUIET_TICKS;
	(void)admissions(5, activation - 3 * FRAME - 5 * SLOT_TICKS, 3, quiet_end);
	/* Those waits may cross the activation; stage again to see promotion. */
	assert(staged_member() == activation);
	set_true(activation - 1);
	assert(tdma_profile_get() == 0);
	set_true(activation);
	assert(tdma_profile_get() == RELAY_PROFILE_120HZ);
	struct esb_network_clock clock;
	assert(esb_get_network_clock(&clock));
	int32_t error = (int32_t)(clock.receiver_ticks - true_ticks());
	printf("receiver-time error after promotion %d ticks\n", error);
	assert(error >= -1 && error <= 1);
	puts("PASS staged 0->1 switch keeps one profile-0 reference and activates on receiver time");
}

/* Acquisition (no profile-0 schedule) still takes receiver time at once. */
static void acquisition_takes_receiver_time(void)
{
	reset_tracker(6);
	own_pong(false, 0);
	set_ticks(now_ticks() + 32768);
	channel_wait_normal = true;
	own_pong(true, true_ticks() + 32768);
	struct esb_network_clock clock;
	assert(esb_get_network_clock(&clock));
	int32_t error = (int32_t)(clock.receiver_ticks - true_ticks());
	assert(error >= -1 && error <= 1);
	puts("PASS acquisition adopts receiver time from the announcing PONG");
}

/* Profile-1 control frame: receiver time taken as is (esb_relay_control_receive). */
static void control_accept(void)
{
	sync_filter.offset = K_TRUE;
	sync_filter.last_local_ticks = now_ticks();
	sync_filter.receiver_lag = 0;
}

static int32_t clock_error(void)
{
	struct esb_network_clock clock;
	assert(esb_get_network_clock(&clock));
	return (int32_t)(clock.receiver_ticks - true_ticks());
}

/* A member that joined profile 1 by acquisition (the announcing PONG restarts
 * its filter on receiver time, and control frame never advances update_count, so the
 * filter stays in warm-up) and followed control frame up to a staged 1->0 exit. Returns
 * the exit's activation tick (receiver time). */
static uint32_t profile1_member(uint8_t slot, uint32_t join)
{
	reset_tracker(slot);
	set_true(join - 32768);
	own_pong(false, 0);
	set_true(join - 32768 / 2);
	channel_wait_normal = true;
	uint32_t activation = own_pong(true, join);
	assert(sync_filter.update_count == 0);
	channel_wait_normal = false;
	set_true(activation);
	assert(tdma_profile_get() == RELAY_PROFILE_120HZ);
	control_accept();
	/* A NORMAL PONG trailer announces the exit about 3 s ahead. */
	set_true(activation + 32768);
	control_accept();
	struct relay_pong_trailer exit = {.epoch = EPOCH + 1, .forward_slot = 255, .member_mask = 0x7ff,
		.pending = true, .pending_epoch = EPOCH + 2, .pending_total = TOTAL,
		.pending_slot_ticks = SLOT_TICKS, .pending_member_mask = 0x7ff,
		.activation_units = 3 * 32768 / RELAY_ACTIVATION_UNIT_TICKS};
	uint32_t t2 = true_ticks();
	esb_stage_from_trailer(&exit, t2);
	return t2 + (uint32_t)exit.activation_units * RELAY_ACTIVATION_UNIT_TICKS;
}

/* Promotion at the exit tick with control frame followed to the end. */
static void promote_exit(uint32_t exit_tick)
{
	set_true(exit_tick - 1);
	control_accept();
	assert(tdma_profile_get() == RELAY_PROFILE_120HZ);
	set_true(exit_tick);
	assert(tdma_profile_get() == 0 && tdma_is_enabled());
}

/* Admitted guarded PING starts (production tdma_wait_for_ping_window) in the
 * first PING window at or after `from`: phase against the slot on the
 * receiver's grid, and the frame on the tracker's own clock. */
static struct span ping_admissions(uint8_t slot, uint32_t from, uint32_t *frame)
{
	struct span span = {INT32_MAX, INT32_MIN, 0};
	for (uint32_t t = from; t < from + 2 * 32768; t++) {
		set_true(t);
		tdma_last_ping_frame = UINT32_MAX;
		tdma_last_ping_pack = 0;
		if (tdma_wait_for_ping_window() != TDMA_PING_ADMITTED) {
			if (span.count && (int32_t)(true_ticks() - from) > 0 && t > from + span.max + 4 * FRAME) {
				break;
			}
			continue;
		}
		uint32_t start = true_ticks();
		struct esb_network_clock clock;
		assert(esb_get_network_clock(&clock));
		if (span.count && clock.receiver_ticks / FRAME != *frame) {
			break;
		}
		*frame = clock.receiver_ticks / FRAME;
		int32_t phase = (int32_t)((start - slot * SLOT_TICKS) % FRAME);
		if (phase > (int32_t)FRAME / 2) {
			phase -= FRAME;
		}
		span.min = MIN(span.min, phase);
		span.max = MAX(span.max, phase);
		span.count++;
	}
	assert(span.count > 0);
	return span;
}

/* Own-slot data admissions (tdma_wait_for_slot) inside one frame of the
 * tracker's own clock. */
static struct span frame_admissions(uint8_t slot, uint32_t frame)
{
	struct span span = {INT32_MAX, INT32_MIN, 0};
	for (uint32_t t = frame * FRAME - FRAME; t < frame * FRAME + 2 * FRAME; t++) {
		set_true(t);
		tdma_last_admitted_frame = UINT32_MAX;
		if (!tdma_wait_for_slot(17)) {
			continue;
		}
		struct esb_network_clock clock;
		assert(esb_get_network_clock(&clock));
		if (clock.receiver_ticks / FRAME != frame) {
			continue;
		}
		int32_t phase = (int32_t)((true_ticks() - slot * SLOT_TICKS) % FRAME);
		if (phase > (int32_t)FRAME / 2) {
			phase -= FRAME;
		}
		span.min = MIN(span.min, phase);
		span.max = MAX(span.max, phase);
		span.count++;
	}
	assert(span.count > 0);
	return span;
}

/* 1->0 exit: a warm-up member PINGs (two attempts, 23-byte PONG) after its
 * first own PONG; the owner two slots on, unguarded in that frame, has not
 * PONGed since the exit. */
static void exit_returns_to_one_reference(void)
{
	uint32_t join = 2000000;
	uint32_t exit_tick = profile1_member(3, join);
	promote_exit(exit_tick);
	int32_t promoted = clock_error();
	set_true(exit_tick + 32768 / 2);
	own_pong(false, 0);
	assert(sync_filter.update_count <= SYNC_WARM_UP_COUNT);
	int32_t pinger = clock_error();
	int32_t skew = sync_filter.skew_ppb;
	uint32_t frame = 0;
	struct span ping = ping_admissions(3, exit_tick + 32768 / 2 + 2 * FRAME, &frame);

	assert(profile1_member(5, join) == exit_tick);
	promote_exit(exit_tick);
	int32_t owner = clock_error();
	struct span data = frame_admissions(5, frame);
	printf("exit: clock error after promotion %d, pinger after its first PONG %d (skew %d ppb), "
		"unPONGed owner %d ticks\n", promoted, pinger, skew, owner);
	double ready = ping.max * TICK_US + PING_END_AFTER_START_US + RAMP_US;
	double preamble = (2 * SLOT_TICKS + data.min) * TICK_US + RAMP_US;
	printf("exit: slot 3 PING phase %d..%d, slot 5 data phase %d..%d in frame %u; "
		"receiver ready %.0f us, next owner's earliest preamble %.0f us\n",
		ping.min, ping.max, data.min, data.max, frame, ready, preamble);
	/* The latest retried PING exchange is over before the next unguarded
	 * owner's earliest preamble, as budget.py requires with no clock error. */
	assert(ready <= preamble);
	assert(pinger - owner >= -1 && pinger - owner <= 1);
	/* Both on the ACK RX reference, the profile-0 clock every member's own
	 * PONGs keep, from the activation tick on. */
	assert(promoted >= -(int32_t)ACK_LAG - 1 && promoted <= -(int32_t)ACK_LAG + 1);
	assert(pinger >= -(int32_t)ACK_LAG - 1 && pinger <= -(int32_t)ACK_LAG + 1);
	assert(owner >= -(int32_t)ACK_LAG - 1 && owner <= -(int32_t)ACK_LAG + 1);
	/* The skew reference moved with the offset: the model's crystals agree,
	 * so the PONG must not read the 7-tick return as drift. */
	assert(skew > -2000 && skew < 2000);
	puts("PASS 1->0 exit returns every member to the ACK RX reference at the activation tick");

	/* The control frame lost in profile 1: the drop returns to the same reference, so a
	 * warm-up filter's first PONG does not leave it part-way. */
	(void)profile1_member(6, join);
	tdma_profile_drop();
	set_true(exit_tick - 32768);
	own_pong(false, 0);
	int32_t dropped = clock_error();
	printf("profile drop: clock error after the first PONG %d ticks\n", dropped);
	assert(dropped >= -(int32_t)ACK_LAG - 1 && dropped <= -(int32_t)ACK_LAG + 1);
	puts("PASS a profile-1 drop returns to the ACK RX reference");
}

int main(void)
{
	staged_switch_keeps_one_reference();
	acquisition_takes_receiver_time();
	exit_returns_to_one_reference();
	return 0;
}
'''


def main():
    # budget.py's guarded-PING model: call to the END of a second attempt
    # carrying a trailer-bearing PONG (no pending layout after the exit).
    pong_len = 13 + budget.consts(root / "src/connection/relay_protocol.h", ["RELAY_TRAILER_LEN"])["RELAY_TRAILER_LEN"]
    attempt = budget.RAMP_US + budget.airtime_us(13) + budget.TURNAROUND_US + budget.airtime_us(pong_len)
    retransmit = budget.kconfig_default(root / "prj.conf", "RADIO_RETRANSMIT_DELAY")
    ping_end = budget.CALL_LATENCY_US + budget.airtime_us(13) + retransmit + attempt
    timing = (f"#define TICK_US {budget.TICK_US!r}\n#define RAMP_US {budget.RAMP_US!r}\n"
              f"#define PING_END_AFTER_START_US {ping_end!r}")
    sync_start = r"^\t\t\tif \(!tdma_profile_get\(\) && rtt_us < rtt_threshold_us\) \{"
    sync = extract_block(pong, sync_start)
    parts = ["#define LOG_DBG(...) ((void)0)", PREFIX, "#include \"frame_ledger.h\"\nint16_t ledger_window_detail, ledger_wait_lateness;",
             "#define CONFIG_CONNECTION_RELAY 1\n#define CLAMP(v, lo, hi) ((v) < (lo) ? (lo) : (v) > (hi) ? (hi) : (v))",
             (root / "src/connection/relay_protocol.h").read_text(),
             "\n".join(re.findall(r"^#define (?:SKEW_REF_REFRESH_TICKS|SYNC_WARM_UP_COUNT)[^\n]*", esb, re.MULTILINE)),
             "static bool channel_wait_normal, own_trailer_valid; static uint8_t tracker_id;\n"
             "static struct relay_pong_trailer own_trailer; static volatile uint32_t esb_last_ack_tx_end_ticks;\n"
             "static uint32_t g_last_rx_raw_ticks, g_min_rtt_ticks = 10;\n"
             "static int64_t k_uptime_get(void) { return (int64_t)(virtual_us / 1000); }",
             "static bool granted = true; static bool esb_layout_granted(void) { return granted; } bool esb_relay_granted_at(uint32_t tick);",
             function(network_ticks, "net_ticks_to_us_ceil64"),
             function(network_ticks, "net_ticks_to_ms_ceil32"),
             function(network_ticks, "net_ticks_to_ms_floor32"),
             extract_block(esb_header, r"^struct esb_network_clock \{", True),
             extract_block(tdma_header, r"^struct tdma_layout \{", True),
             function(esb, "net_tick_difference"),
             function(esb, "kernel_ticks_extend32"),
             extract_block(esb, r"^struct esb_sync_sample \{", True),
             function(esb, "esb_sync_filter_update"),
             ADOPT,
             function(esb, "esb_project_network_ticks"),
             function(esb, "esb_get_network_clock"),
             "uint8_t tdma_profile_get(void); bool tdma_is_enabled(void);",
             tdma[tdma.index("#define TDMA_ENABLED_BIT"):tdma.index("\nvoid tdma_init(")],
             tdma[tdma.index("#define TDMA_LEN17_RESERVE_TICKS"):tdma.index("\nvoid tdma_note_radio_busy")],
             re.search(r"^#define TDMA_SLOT_UNASSIGNED[^\n]*", tdma_header, re.MULTILINE)[0],
             function(tdma, "tdma_is_enabled"),
             function(tdma, "tdma_update_config"),
             function(tdma, "tdma_set_enabled"),
             function(tdma, "tdma_profile_drop"),
             tdma[tdma.index("bool tdma_stage_config("):tdma.index("bool tdma_layout_get(")],
             tdma[tdma.index("bool tdma_layout_get("):],
             function(esb, "esb_stage_from_trailer"),
             "bool esb_relay_granted_at(uint32_t tick) { return granted && relay_runtime_direct_granted(&relay_direct_anchor, tick); }",
             # The production sync update of esb_receive_pong, with the locals
             # the PONG path computed before it.
             "static void pong_sync(uint32_t ping_ticks_for_this_ctr, uint32_t ping_rx_ticks, uint32_t t4_net_ticks)\n{\n"
             "\tuint32_t rtt_us = 300, rtt_threshold_us = 1000, rtt_ticks = 10;\n"
             + sync + "\n\t(void)rtt_ticks;\n}",
             timing,
             CASES]
    with tempfile.TemporaryDirectory() as tmp:
        src = Path(tmp) / "staged_reference.c"
        src.write_text("\n".join(parts))
        exe = Path(tmp) / "staged_reference"
        cc = shlex.split(os.environ.get("CC", "cc"))
        subprocess.run(cc + ["-I", str(root / "src"), "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function",
                             "-Wno-unused-variable", "-Wno-unused-parameter", "-Wno-unused-but-set-variable",
                             "-g", "-O1", "-fsanitize=address,undefined", str(src), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
