#!/usr/bin/env python3
"""Compare runtime schedules against accepted R5 and FULL16/R3 trace digests.

Golden traces cover every slot/tick, RTC wrap, repeated PONGs and activation
seeds. They were verified against the accepted historical TDMA implementations.
The hashes retain the comparison without depending on unpublished Git objects.
Clock, radio and IRQ leaves are modeled. This does not measure RF or latency.
"""

from pathlib import Path
import argparse
import ast
import hashlib
import importlib.util
import json
import os
import shlex
import struct
import subprocess
import tempfile

here = Path(__file__).resolve().parent
root = Path(os.environ.get("SOURCE_ROOT", here.parents[2]))
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--evidence-dir", type=Path)
args = parser.parse_args()


def evidence(name, data):
    if args.evidence_dir:
        args.evidence_dir.mkdir(parents=True, exist_ok=True)
        (args.evidence_dir / name).write_bytes(data)


spec = importlib.util.spec_from_file_location("tdma_harness", here / "run_tdma.py")
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)



def prefix_of(text):
    for n in ast.parse(text).body:
        if isinstance(n, ast.Assign) and any(
            isinstance(t, ast.Name) and t.id == "PREFIX" for t in n.targets
        ):
            return ast.literal_eval(n.value)
    raise ValueError("PREFIX absent")


def build_source(ref, new):
    read = lambda path: (root / path).read_text()
    tdma = read("src/connection/tdma.c")
    esb = read("src/connection/esb.c")
    net = read("src/connection/network_ticks.h")
    eh = read("src/connection/esb.h")
    th = read("src/connection/tdma.h")
    prefix = m.PREFIX if new else prefix_of(read("tests/host/relay/run_tdma.py"))
    prefix = prefix.replace(
        "g_last_sync_timestamp = (int64_t)(virtual_us / 1000); ", ""
    )
    parts = [
        f"#define NEW_VERSION {int(new)}",
        prefix,
        '#include "frame_ledger.h"\nint16_t ledger_window_detail,ledger_wait_lateness;',
        read("src/connection/relay_protocol.h"),
        "static bool granted=true; static bool esb_layout_granted(void){return granted;} bool esb_relay_granted_at(uint32_t tick);",
    ]
    parts += [
        m.function(net, x)
        for x in (
            "net_ticks_to_us_ceil64",
            "net_ticks_to_ms_ceil32",
            "net_ticks_to_ms_floor32",
        )
    ]
    parts += [
        m.extract_block(eh, r"^struct esb_network_clock \{", True),
        m.extract_block(th, r"^struct tdma_layout \{", True),
        m.function(esb, "esb_project_network_ticks"),
        m.function(esb, "esb_get_network_clock"),
        tdma[tdma.index("#define TDMA_ENABLED_BIT") : tdma.index("\nvoid tdma_init(")],
        tdma[
            tdma.index("#define TDMA_LEN17_RESERVE_TICKS") : tdma.index(
                "\nvoid tdma_note_radio_busy"
            )
        ],
        m.re.search(r"^#define TDMA_SLOT_UNASSIGNED[^\n]*", th, m.re.MULTILINE)[0],
        m.function(tdma, "tdma_update_config"),
        tdma[
            tdma.index("bool tdma_stage_config(") : tdma.index("bool tdma_layout_get(")
        ],
        tdma[tdma.index("bool tdma_layout_get(") :],
    ]
    parts.append(
        "bool esb_relay_granted_at(uint32_t tick){"
        + (
            "return granted && relay_runtime_direct_granted(&relay_direct_anchor,tick);"
            if new
            else "(void)tick;return granted;"
        )
        + "}"
    )
    relay = read("src/connection/relay.c")
    parts.append(
        "static bool control_valid=true; static struct relay_control control_plan; static uint8_t control_helper,repair_attempted;"
    )
    parts.append(m.function(relay, "relay_next_repair"))
    parts.append(WINDOW_LEAVES)
    if new:
        parts.append(
            m.re.search(
                r"^#define RELAY_CONTROL_RX_SETUP_TICKS[^\n]*", esb, m.re.MULTILINE
            )[0]
        )
    parts.append(m.function(esb, "esb_relay_control_window"))
    parts += [
        m.function(tdma, x)
        for x in ("tdma_set_enabled", "tdma_profile_drop", "tdma_is_enabled")
    ]
    parts.append(RECEIVE_LEAVES)
    parts += [
        m.function(esb, x)
        for x in (
            "net_tick_difference",
            "kernel_ticks_extend32",
            "esb_stage_from_trailer",
        )
    ]
    link = read("src/connection/link_supervision.h")
    parts.append(
        "#define LINK_PASSIVE_MAX_INNOVATION_TICKS 4\n"
        + m.function(link, "link_passive_sample_acceptable")
    )
    parts.append(
        m.function(esb, "esb_relay_control_receive").replace(
            "esb_relay_control_receive(", "equiv_receive_control("
        )
    )
    pong = m.extract_block(esb, r"^static void esb_receive_pong\([^;{]*\)\s*\{")
    block = m.extract_block(
        pong, r"^\t\tif \(pong_flags == ESB_PONG_FLAG_NORMAL\)\s*\{"
    )
    parts.append(
        "#define ESB_PONG_FLAG_NORMAL 0\nstatic void receive_schedule(const struct esb_payload *payload,uint8_t pong_flags){"
        + block
        + "}"
    )
    helper = m.function(relay, "relay_head_window")
    cut = helper.index("\n\tif (layout->profile) {\n\t\tif (radio_capture_init() != 0)")
    helper = (
        helper[:cut]
        + r"""
 helper_open=true;helper_child=best_child;helper_rx_at=rx_at;helper_capture_end=capture_end;helper_tail_end=tail_end;helper_open_tick=now_ticks();
 irq_unlock(rx_key);return true;
}
"""
    )
    parts.append(helper)
    return "\n".join(parts), read("src/frame_ledger.h")


WINDOW_LEAVES = r"""

#define BIT(n) (1U << (n))
static unsigned control_window_fail[6], control_open_late, control_open_lag_max, control_crc_fail, control_missed,
	control_open_hist[16][2], control_no_address, control_address_timeout, radio_channel;
static uint32_t radio_open_tick, radio_close_tick, radio_end_at;
static bool inject_B;
struct relay_rx_frame {
	uint32_t end_kernel_ticks;
	uint8_t pipe, length, data[72];
};
static int esb_relay_clock(bool on)
{
	(void)on;
	return 0;
}
static int radio_capture_init(void)
{
	return 0;
}
static bool radio_capture_rx_end(uint32_t *t)
{
	*t = now_ticks();
	return true;
}
static void radio_capture_set_enabled(bool on)
{
	(void)on;
}
static bool esb_raw_rx_begin(void)
{
	return true;
}
static void esb_raw_rx_end(void)
{
}
static bool relay_radio_start(unsigned ch, unsigned pipes, bool rx)
{
	(void)ch;
	(void)pipes;
	(void)rx;
	radio_open_tick = now_ticks();
	return true;
}
static bool relay_radio_prepare(unsigned ch,unsigned pipes,bool rx){(void)ch;(void)pipes;(void)rx;return true;}
static bool relay_radio_receive(void){radio_open_tick=now_ticks();return true;}
static bool relay_radio_ended(void)
{
	return inject_B && (int32_t)(now_ticks() - radio_end_at) >= 0;
}
static bool relay_radio_take(struct relay_rx_frame *f)
{
	f->end_kernel_ticks = radio_end_at;
	return inject_B;
}
static void relay_radio_peek(uint8_t *l, bool *c, uint8_t *r)
{
	*l = 0;
	*c = false;
	*r = 0;
}
static bool relay_radio_stop(void)
{
	radio_close_tick = now_ticks();
	return true;
}
static bool relay_radio_addressed(void)
{
	return false;
}
static bool esb_relay_control_receive(struct relay_rx_frame *f)
{
	(void)f;
	return inject_B;
}
static uint32_t relay_generation;
static struct {
	uint32_t head_windows, head_busy, start_late_max;
} stats;
static void peer_count(uint8_t c, bool h, uint8_t r)
{
	(void)c;
	(void)h;
	(void)r;
}
#define relay_ledger_tick(...) ((void)0)
static bool helper_open;
static uint32_t helper_child, helper_rx_at, helper_capture_end, helper_tail_end, helper_open_tick;
"""

RECEIVE_LEAVES = r"""

static bool control_seen, layout_confirm_seen, receiver_heard_seen, relayed_confirm_seen, layout_stale,
	channel_wait_normal;
static uint32_t control_first_received, control_repeat_received, control_repeat_seen, control_repeat_reject[6];
static uint32_t control_session, control_frame, control_tick, control_received, control_reject[6], layout_confirm_time,
	receiver_heard_time, relayed_confirm_time, g_last_rx_raw_ticks;
static int32_t control_err_last, control_err_min, control_err_max;
static atomic_t channel_found;
static uint8_t tracker_id;
static struct relay_pong_trailer own_trailer;
static bool own_trailer_valid = true;
struct esb_payload {
	uint8_t data[23];
};
static uint64_t k_uptime_get(void)
{
	return virtual_us / 1000;
}
static uint32_t k_uptime_get_32(void)
{
	return k_uptime_get();
}
static uint32_t sys_get_be32(const uint8_t *p)
{
	return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}
static void relay_reset(void)
{
}
static void connection_request_ping_resync(void)
{
	assert(false);
}
static void relay_on_control(const struct relay_control *c, uint8_t own)
{
	control_plan = *c;
	control_helper = own;
}
"""

CASES = r"""

static unsigned direct_count, repairs;
static uint32_t origin, seed;
static void at(uint64_t ticks)
{
	virtual_us = (ticks * 1000000U + 32767U) / 32768U;
}
static uint32_t expected_start(uint32_t f)
{
	return origin
		 + (uint32_t)(direct_count == 11 ? (uint64_t)f * 4096 / 15 - (uint64_t)seed * 4096 / 15
										 : (uint64_t)(f - seed) * 320);
}
static void pong(unsigned own)
{
#if NEW_VERSION
	uint8_t code = relay_geometry_code(direct_count, repairs);
#else
	uint8_t code = direct_count == 11 ? 0x81 : 0x82;
#endif
	struct esb_payload p = {0};
	uint32_t t2 = now_ticks();
	p.data[3] = t2 >> 24;
	p.data[4] = t2 >> 16;
	p.data[5] = t2 >> 8;
	p.data[6] = t2;
	p.data[8] = own;
	p.data[9] = direct_count;
	p.data[10] = 16;
	p.data[11] = 7;
	own_trailer = (struct relay_pong_trailer){
		.pending = true,
		.pending_member_mask = (uint16_t)((1U << direct_count) - 1U),
		.pending_total = code,
		.pending_slot_ticks = 16,
		.pending_epoch = 7,
		.activation_units = (int32_t)(origin - t2) > 0 ? (origin - t2) / 4 : 0
	};
	receive_schedule(&p, ESB_PONG_FLAG_NORMAL);
}
static void control(uint32_t f, unsigned own)
{
	struct relay_control c
		= {.profile = 1,
		   .members = (uint16_t)((1U << direct_count) - 1U),
		   .epoch = 7,
		   .session = 42,
		   .frame = f,
		   .preamble_ticks = expected_start(f) + direct_count * 16 + 2,
		   .maintenance_owner = RELAY_OWNER_IDLE,
		   .repair_count = repairs};
	c.expected = c.members;
	c.survey = c.members;
	for (unsigned i = 0; i < 5; i++) {
		c.grants[i].owner = i < repairs ? own : RELAY_OWNER_IDLE;
	}
#if NEW_VERSION
	c.direct_count = direct_count;
	c.slot_ticks = 16;
	c.tps_max = 120;
	relay_control_set_next_repairs(&c, repairs);
	relay_control_set_next2_repairs(&c, repairs);
#endif
	struct relay_rx_frame rx = {.pipe = 1, .length = 72};
	assert(relay_control_encode(rx.data, &c) == 72);
	uint64_t end
		= (uint64_t)origin
		+ (direct_count == 11 ? (uint64_t)f * 4096 / 15 - (uint64_t)seed * 4096 / 15 : (uint64_t)(f - seed) * 320)
		+ direct_count * 16 + 13;
	rx.end_kernel_ticks = (uint32_t)((end * 1000000U + 32767U) / 32768U);
	assert(equiv_receive_control(&rx));
#if NEW_VERSION
 assert(relay_control_next2_repairs(&control_plan)==repairs);
 uint32_t future;
 assert(relay_runtime_start(&relay_direct_anchor,f+3U,&future));
 assert(future==expected_start(f+3U));
#endif
}
static void run(unsigned own)
{
	tdma_cfg_pack = tdma_staged_pack = 0;
	tdma_staged_local = 0;
#if NEW_VERSION
	relay_timeline_clear(&relay_timeline_active);
	relay_direct_anchor.valid = false;
#endif
	tracker_id = own;
	control_seen = false;
	sync_filter = (struct esb_sync_filter){.initialized = true};
	at((uint64_t)origin - 100);
	g_last_sync_timestamp = k_uptime_get();
	server_time_synced = true;
	assert(tdma_update_config(own, direct_count, 16, 6));
	pong(own);
	at((uint64_t)origin + direct_count * 16 + 13);
	assert(tdma_profile_get() == 1);
	granted = true;
	control(seed, own);
	for (unsigned step = 1; step <= 15; step++) {
		unsigned f = seed + step;
		uint32_t start = expected_start(f), length = expected_start(f + 1) - start;
		for (unsigned offset = 0; offset < length; offset++) {
			at((uint64_t)origin
			   + (direct_count == 11 ? (uint64_t)f * 4096 / 15 - (uint64_t)seed * 4096 / 15
									 : (uint64_t)(f - seed) * 320)
			   + offset);
			if (offset == direct_count * 16 + 13) {
				control(f, own);
			}
			if (offset == 1 || offset == direct_count * 16 + 14) {
				pong(own);
			}
			if (seed && offset != 0 && offset != own * 16 && offset != own * 16 + 2 && offset != own * 16 + 4
				&& offset != own * 16 + 6 && offset != own * 16 + 8 && offset != direct_count * 16 - 5
				&& offset != direct_count * 16 - 4 && offset != direct_count * 16 + 13
				&& offset != direct_count * 16 + 14 && offset != length - 2) {
				continue;
			}
			tdma_last_admitted_pack = 0;
			tdma_last_admitted_frame = UINT32_MAX;
			tdma_stream_slots = 0;
			uint32_t row[96];
			unsigned n = 0;
			row[n++] = origin;
			row[n++] = own;
			row[n++] = f;
			row[n++] = offset;
			row[n++] = relay_frame_start(tdma_profile_origin, f);
			row[n++] = relay_frame_start(tdma_profile_origin, f + 1);
			row[n++] = relay_frame_near(tdma_anchor_frame, tdma_anchor_start, now_ticks());
			row[n++] = tdma_stream_at(now_ticks(), own * 16 + 2);
			uint32_t beacon = tdma_stream_at(now_ticks(), direct_count * 16 - 4);
			row[n++] = beacon;
			row[n++] = beacon + 4 + 16;
			bool broadcast = false;
			uint32_t target = 0;
			row[n++] = tdma_stream_next(&target, &broadcast);
			row[n++] = target;
			row[n++] = broadcast;
			uint64_t saved_us = virtual_us;
			if (offset == 0 || offset == direct_count * 16 - 5 || offset == direct_count * 16 + 14) {
				inject_B = false;
				radio_open_tick = radio_close_tick = 0;
				assert(!esb_relay_control_window(beacon));
				row[n++] = radio_open_tick;
				row[n++] = radio_close_tick;
				virtual_us = saved_us;
				inject_B = true;
				radio_end_at = beacon + 4 + 13;
				radio_open_tick = radio_close_tick = 0;
				assert(esb_relay_control_window(beacon));
				row[n++] = radio_open_tick;
				row[n++] = radio_close_tick;
				virtual_us = saved_us;
				inject_B = false;
			} else {
				row[n++] = 0;
				row[n++] = 0;
				row[n++] = 0;
				row[n++] = 0;
			}
			struct tdma_layout layout;
			assert(tdma_layout_get(&layout));
			struct relay_pong_trailer tr = {.member_mask = (uint16_t)((1U << direct_count) - 1U)};
			for (unsigned child = 0; child < direct_count; child++) {
				if (relay_can_overhear(tr.member_mask, own, child)) {
					tr.children_mask |= 1U << child;
				}
			}
			helper_open = false;
			helper_child = helper_rx_at = helper_capture_end = helper_tail_end = helper_open_tick = 0;
			row[n++] = relay_head_window(&tr, &layout, relay_generation);
			row[n++] = helper_open;
			row[n++] = helper_child;
			row[n++] = helper_rx_at;
			row[n++] = helper_capture_end;
			row[n++] = helper_tail_end;
			row[n++] = helper_open_tick;
			virtual_us = saved_us;
			for (unsigned child = 0; child < direct_count; child++) {
				uint32_t at = 0;
				bool ok = tdma_slot_data_window(child, now_ticks(), &at);
				row[n++] = ok ? at : UINT32_MAX;
			}
			for (unsigned r = 0; r < repairs; r++) {
				repair_attempted = (1U << r) - 1U;
				uint8_t slot = 0;
				uint32_t at = 0;
				bool ok = relay_next_repair(&at, &slot);
				row[n++] = ok ? at : UINT32_MAX;
				row[n++] = ok ? slot : UINT32_MAX;
			}
			for (unsigned pc = 0; pc < 3; pc++) {
				unsigned payload = pc == 0 ? 17 : pc == 1 ? 32 : 60;
				virtual_us = saved_us;
				tdma_last_admitted_pack = 0;
				tdma_last_admitted_frame = UINT32_MAX;
				tdma_stream_slots = 0;
				bool admitted = tdma_wait_for_slot(payload);
				row[n++] = admitted;
				row[n++] = now_ticks();
				row[n++] = admitted && tdma_slot_tx_valid(payload);
				bool b = false;
				uint32_t next = 0;
				assert(tdma_stream_next(&next, &b));
				row[n++] = next;
				row[n++] = b;
			}
			for (unsigned r = 0; r < repairs; r++) {
				virtual_us = saved_us;
				tdma_last_admitted_pack = tdma_forward_admitted_pack = 0;
				tdma_last_admitted_frame = tdma_forward_admitted_frame = UINT32_MAX;
				tdma_stream_slots = 0;
				bool admitted = tdma_wait_for_forward_slot(direct_count + 1 + r, 72);
				row[n++] = admitted;
				row[n++] = now_ticks();
				row[n++] = admitted && tdma_forward_tx_valid(direct_count + 1 + r, 72);
				row[n++] = tdma_wait_for_forward_slot(direct_count + 1 + r, 72);
			}
			virtual_us = saved_us;
			assert(n < 96);
			for (; n < 96; n++) {
				row[n] = 0;
			}
			assert(fwrite(row, sizeof(row), 1, stdout) == 1);
		}
	}
}
int main(int argc, char **argv)
{
	assert(argc == 2);
	direct_count = atoi(argv[1]);
	repairs = direct_count == 11 ? 5 : 3;
	for (unsigned wrap = 0; wrap < 2; wrap++) {
		origin = wrap ? 0xfffff000U : 10000U;
		const unsigned seeds[] = {0, 14, 65535};
		for (unsigned si = 0; si < 3; si++) {
			seed = seeds[si];
			for (unsigned own = 0; own < direct_count; own++) {
				run(own);
			}
		}
	}
	return 0;
}
"""
# strtoul/atoi is a leaf of the model, not device scheduling code.
CASES = "#include <stdlib.h>\n" + CASES


def trace_fields(direct, repairs):
    fields = [
        "origin",
        "own",
        "frame",
        "phase",
        "frame_start",
        "next_frame_start",
        "frame_near",
        "direct_target",
        "B_target_open",
        "B_target_close",
        "next_ready",
        "next_target",
        "next_is_B",
        "B_timeout_open",
        "B_timeout_close",
        "B_END_open",
        "B_END_close",
        "helper_attempt",
        "helper_open",
        "helper_child",
        "helper_rx_at",
        "helper_capture_end",
        "helper_tail_end",
        "helper_open_tick",
    ]
    fields += [f"child_{c}_RX" for c in range(direct)]
    fields += [
        f"repair_{r}_{field}" for r in range(repairs) for field in ("target", "slot")
    ]
    fields += [
        f"direct_{payload}_{field}"
        for payload in (17, 32, 60)
        for field in ("admit", "tick", "final", "next_target", "next_is_B")
    ]
    fields += [
        f"repair_{r}_{field}"
        for r in range(repairs)
        for field in ("admit", "tick", "final", "duplicate")
    ]
    return fields + ["padding"] * (96 - len(fields))


GOLDEN = {11: (97312, 'a486e1735a4eeb5544a8257d7dcc9c5a5e8d152e828782164ca375e5e1fff868'), 16: (164100, 'fcd662008d6def92689ce8534db91d2e9af1d7c63fb99d9eb6991afbf903c28c')}

results = []
for direct, ref in ((11, "e9e9a439"), (16, "730bd8a3")):
    with tempfile.TemporaryDirectory(prefix="relay-equiv-") as tmp:
        tmp = Path(tmp)
        outputs = []
        for new in (True,):
            code, ledger = build_source(ref, new)
            (tmp / "frame_ledger.h").write_text(ledger)
            src = tmp / ("new.c" if new else "old.c")
            src.write_text(code + "\n" + CASES)
            exe = src.with_suffix("")
            subprocess.run(
                shlex.split(os.environ.get("CC", "cc"))
                + [
                    "-I",
                    str(tmp),
                    "-std=gnu11",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-Wno-unused-function",
                    "-Wno-unused-variable",
                    "-Wno-unused-parameter",
                    "-Wno-unused-but-set-variable",
                    "-O1",
                    "-g",
                    "-fsanitize=address,undefined",
                    str(src),
                    "-o",
                    str(exe),
                ],
                check=True,
            )
            result = subprocess.run(
                [str(exe), str(direct)], stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            evidence(f"equivalence-{direct}-{int(new)}-stderr.txt", result.stderr)
            assert result.returncode == 0, (src, result.stderr.decode())
            outputs.append(result.stdout)
        new = outputs[0]
        rowbytes = 96 * 4
        assert len(new) % rowbytes == 0
        expected = GOLDEN[direct]
        row = {"direct": direct, "rows": len(new) // rowbytes,
               "trace_sha256": hashlib.sha256(new).hexdigest()}
        row["passed"] = row["rows"] == expected[0] and row["trace_sha256"] == expected[1]
        results.append(row)
        print(json.dumps(row), flush=True)
evidence(
    "schedule-equivalence-results.json", (json.dumps(results, indent=2) + "\n").encode()
)
assert all(r["passed"] for r in results), (
    "Scheduling divergence; retained first differing row"
)
