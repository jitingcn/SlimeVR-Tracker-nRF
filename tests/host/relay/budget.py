#!/usr/bin/env python3
"""End-to-end timing budget of the child -> head -> receiver relay path.

Reads the production constants from both firmware trees and checks that every
hop fits its window at the narrowest slot the receiver can assign, then prints
the budget table used in RELAY.md. Airtime model: nRF ESB DPL at 2 Mbps,
16-bit preamble, 5-byte address, 8-bit length + 3-bit S1, 16-bit CRC, fast
(40 us) ramp-up. CALL_LATENCY_US is an assumed software allowance from
admission to TX enable. These allowances are not hardware measurements.
"""
from pathlib import Path
import re
import sys
import tempfile, subprocess, os, shlex

HERE = Path(__file__).resolve().parent
TRACKER = HERE.parents[2]
RECEIVER = TRACKER.parent / "receiver"
TICK_US = 1e6 / 32768
RAMP_US = 40.0
TURNAROUND_US = 45.0  # receiver END -> ACK TX start (DISABLED ISR + fast ramp)
CALL_LATENCY_US = 20.0
SYNC_ERROR_TICKS = 2   # head vs child receiver-time estimates, combined
RX_SETTLE_US = 40.0    # receiver settling after READY before a preamble is reliably caught


def consts(path, names):
    text = path.read_text()
    out = {}
    for name in names:
        m = re.search(rf"^#define {name}\s+\(?([0-9]+)U?\)?", text, re.MULTILINE)
        if not m:
            raise SystemExit(f"{name} not found in {path}")
        out[name] = int(m.group(1))
    return out


def airtime_us(payload):
    bits = 16 + 40 + 11 + payload * 8 + 16
    return bits / 2.0


def kconfig_default(path, name):
    m = re.search(rf"^CONFIG_{name}=(\d+)", path.read_text(), re.MULTILINE)
    return int(m.group(1))


failures = []


def check(cond, message):
    print(("ok   " if cond else "FAIL ") + message)
    if not cond:
        failures.append(message)


def main():
    t = consts(TRACKER / "src/connection/tdma.c", [
        "TDMA_LEN17_RESERVE_TICKS", "TDMA_LEN18_32_RESERVE_TICKS", "TDMA_LEN33_PLUS_RESERVE_TICKS",
        "TDMA_SLOT_TARGET_OFFSET", "TDMA_PING_LATE_TOLERANCE_TICKS", "TDMA_MIN_SAFE_SLOT_TICKS",
        "TDMA_PING_WINDOW_SLOTS"])
    r = consts(TRACKER / "src/connection/relay.h", [
        "RELAY_HEAD_LEASE_MS", "RELAY_PRESENCE_INTERVAL_MS", "RELAY_RX_TAIL_US",
        "RELAY_FORWARD_GUARD_TICKS", "RELAY_PRESENCE_CLOCK_LEAD_US"])
    p = consts(TRACKER / "src/connection/relay_protocol.h", [
        "RELAY_FRAME_MAX", "RELAY_TRAILER_LEN", "RELAY_TRAILER_PENDING_LEN", "RELAY_ACTIVATION_UNIT_TICKS",
        "RELAY_LAYOUT_GRANT_MS", "RELAY_LAYOUT_ACTIVATION_MS", "RELAY_RX_LEAD_TICKS"])
    r["RELAY_RX_LEAD_TICKS"] = p["RELAY_RX_LEAD_TICKS"]
    link = consts(TRACKER / "src/connection/link_supervision.h", [
        "LINK_UPLINK_HOLDOVER_MS", "LINK_PRESENCE_TIMEOUT_MS", "LINK_DEGRADED_AFTER_MS"])
    ping_interval = consts(TRACKER / "src/connection/esb.h", ["PING_INTERVAL_MS"])["PING_INTERVAL_MS"]
    tracker_max = consts(TRACKER / "src/connection/tdma.h", ["TDMA_MAX_TRACKERS"])["TDMA_MAX_TRACKERS"]
    retransmit_us = kconfig_default(TRACKER / "prj.conf", "RADIO_RETRANSMIT_DELAY")
    rc = None
    if (RECEIVER / "src/connection/relay_coord.h").is_file():
        rc = consts(RECEIVER / "src/connection/relay_coord.h", [
            "RC_FWD_REVOKE_WAIT_MS", "RC_MAX_TOTAL_SLOTS", "RC_WINDOW_MS"])
        recv = consts(RECEIVER / "src/connection/esb.c", [
            "TDMA_BASE_SLOT_TICKS", "TDMA_SLOT_TICKS_MAX_LEVEL0", "TDMA_SHADOW_LEAVE_GRACE_MS"])
    # Worst case: a PONG announcing a pending layout.
    pong_ext = 13 + max(p["RELAY_TRAILER_LEN"], p["RELAY_TRAILER_PENDING_LEN"])
    S = t["TDMA_MIN_SAFE_SLOT_TICKS"]
    slot_us = S * TICK_US

    print(f"Narrowest slot S={S} ticks = {slot_us:.0f} us; 72-byte frame {airtime_us(72):.0f} us on air\n")

    # 1. Guarded PING exchange with the longest PONG, two attempts.
    attempt = RAMP_US + airtime_us(13) + TURNAROUND_US + airtime_us(pong_ext)
    latest_start = (t["TDMA_SLOT_TARGET_OFFSET"] + t["TDMA_PING_LATE_TOLERANCE_TICKS"]) * TICK_US
    # The retry timer begins after the first TX ends, not at its start.
    end = latest_start + CALL_LATENCY_US + airtime_us(13) + retransmit_us + attempt
    window = t["TDMA_PING_WINDOW_SLOTS"] * slot_us
    receiver_ready = end + RAMP_US
    next_preamble = window + t["TDMA_SLOT_TARGET_OFFSET"] * TICK_US + RAMP_US
    check(receiver_ready <= next_preamble,
          f"PING+{pong_ext}B PONG, 2 attempts: END {end:.0f} us, receiver RX ready {receiver_ready:.0f} us"
          f" <= next owner's earliest preamble {next_preamble:.0f} us")
    print(f"     remaining software/turnaround allowance: {next_preamble - receiver_ready:.0f} us;"
          " measure this on both MCUs, including IRQ load and the receiver FEM")

    # 2. Child -> head capture.
    child_end = (S - t["TDMA_LEN17_RESERVE_TICKS"]) * TICK_US + CALL_LATENCY_US + RAMP_US + airtime_us(17)
    check(child_end <= slot_us, f"child 17B frame admitted last ends {child_end:.0f} us <= slot end {slot_us:.0f} us")
    comp_end = (S - t["TDMA_LEN33_PLUS_RESERVE_TICKS"]) * TICK_US + CALL_LATENCY_US + RAMP_US + airtime_us(68)
    check(comp_end - slot_us <= r["RELAY_RX_TAIL_US"],
          f"largest child composite overrun {max(0, comp_end - slot_us):.0f} us <= head RX tail {r['RELAY_RX_TAIL_US']} us")
    rx_ready = -r["RELAY_RX_LEAD_TICKS"] * TICK_US + RAMP_US
    child_first_bit = (t["TDMA_SLOT_TARGET_OFFSET"] - SYNC_ERROR_TICKS) * TICK_US + RAMP_US
    check(rx_ready + 10 + RX_SETTLE_US <= child_first_bit,
          f"head RX settled at {rx_ready + 10 + RX_SETTLE_US:+.0f} us (10 us wake jitter + {RX_SETTLE_US:.0f} us settle) <= "
          f"child's earliest preamble {child_first_bit:+.0f} us with {SYNC_ERROR_TICKS} ticks of relative sync error")

    # 3. Head -> receiver: full-size bundle in the head's own or forwarding slot.
    bundle_end = (S - t["TDMA_LEN33_PLUS_RESERVE_TICKS"]) * TICK_US + CALL_LATENCY_US + RAMP_US + airtime_us(p["RELAY_FRAME_MAX"])
    next_first_bit = slot_us + t["TDMA_SLOT_TARGET_OFFSET"] * TICK_US + RAMP_US
    check(bundle_end <= next_first_bit,
          f"72B bundle admitted last ends {bundle_end:.0f} us <= next owner's earliest preamble {next_first_bit:.0f} us")
    window_ticks = S - t["TDMA_LEN33_PLUS_RESERVE_TICKS"] - t["TDMA_SLOT_TARGET_OFFSET"] + 1
    check(window_ticks >= 3, f"72B admission window {window_ticks} ticks >= 3 (wake-jitter rule)")

    # 4. Ordering: a record is forwarded only while more than
    # RELAY_FORWARD_GUARD_TICKS remain before the child's next slot, and a
    # full bundle ends within that guard of being built (built right before
    # TX), so the relayed copy always precedes the child's next frame.
    bundle_duration = CALL_LATENCY_US + RAMP_US + airtime_us(p["RELAY_FRAME_MAX"])
    guard_us = (r["RELAY_FORWARD_GUARD_TICKS"] + t["TDMA_SLOT_TARGET_OFFSET"]) * TICK_US + RAMP_US
    check(bundle_duration + TICK_US <= guard_us,
          f"72B bundle ends {bundle_duration:.0f} us after it is built (+1 tick clock read) <= guard "
          f"{r['RELAY_FORWARD_GUARD_TICKS']} ticks + child's earliest preamble offset = {guard_us:.0f} us, "
          "so a relayed copy always precedes the child's next frame")
    # The guard must not exceed what the slot right before the child's leaves:
    # a head whose own slot directly precedes the child's (every two-tracker
    # layout) must still be able to forward. Earliest own-slot admission is at
    # TDMA_SLOT_TARGET_OFFSET; the child's next slot starts one slot later.
    latest_72 = S - t["TDMA_LEN33_PLUS_RESERVE_TICKS"]
    adjacent_remaining = S - latest_72
    check(adjacent_remaining > r["RELAY_FORWARD_GUARD_TICKS"],
          f"head slot adjacent to the child's: {adjacent_remaining} ticks remain even at the latest 72B admission > "
          f"guard {r['RELAY_FORWARD_GUARD_TICKS']} (forwardable)")

    # 5. Recovery timing.
    check(r["RELAY_HEAD_LEASE_MS"] >= 2 * ping_interval, f"head lease {r['RELAY_HEAD_LEASE_MS']} ms survives one missed PONG")
    if rc:
        check(rc["RC_FWD_REVOKE_WAIT_MS"] > r["RELAY_HEAD_LEASE_MS"] + ping_interval,
              f"forwarding slot withdrawn {rc['RC_FWD_REVOKE_WAIT_MS']} ms after revoke > lease + PONG period")
        check(rc["RC_MAX_TOTAL_SLOTS"] == tracker_max, "members + forwarding slots stay within what trackers accept")
        check(link["LINK_UPLINK_HOLDOVER_MS"] < recv["TDMA_SHADOW_LEAVE_GRACE_MS"],
              "uplink holdover ends before receiver membership eviction")
    check(link["LINK_UPLINK_HOLDOVER_MS"] < 10 * ping_interval, "uplink holdover ends before TDMA sync goes stale")
    check(link["LINK_PRESENCE_TIMEOUT_MS"] > 3 * r["RELAY_PRESENCE_INTERVAL_MS"],
          "several presence windows fit in the presence timeout")

    # 6. Staged layout activation. The last PONG that renewed a tracker's grant
    # without announcing the change was sent before the receiver staged it,
    # so the grant ends no later than GRANT after staging; activation follows
    # by a margin covering clock disagreement and a full frame on air.
    margin_ms = p["RELAY_LAYOUT_ACTIVATION_MS"] - p["RELAY_LAYOUT_GRANT_MS"]
    worst_frame_ms = (255 * tracker_max * TICK_US) / 1000
    check(margin_ms * 1000 > SYNC_ERROR_TICKS * TICK_US + CALL_LATENCY_US + RAMP_US + airtime_us(p["RELAY_FRAME_MAX"])
          + worst_frame_ms * 1000,
          f"activation {p['RELAY_LAYOUT_ACTIVATION_MS']} ms - grant {p['RELAY_LAYOUT_GRANT_MS']} ms = {margin_ms} ms"
          f" covers sync error, one 72B frame and an admission up to one widest frame ({worst_frame_ms:.0f} ms) old")
    check(p["RELAY_LAYOUT_GRANT_MS"] >= 2 * ping_interval + ping_interval // 2,
          f"grant {p['RELAY_LAYOUT_GRANT_MS']} ms survives two missed own PONGs at {ping_interval} ms")
    units = (p["RELAY_LAYOUT_ACTIVATION_MS"] * 32768 + 999) // 1000 // p["RELAY_ACTIVATION_UNIT_TICKS"] + 1
    check(units <= 0xFFFF, f"activation lead {units} units fits the 16-bit field")
    check(p["RELAY_LAYOUT_ACTIVATION_MS"] >= 3 * ping_interval,
          f"each tracker gets {p['RELAY_LAYOUT_ACTIVATION_MS'] // ping_interval} own PONGs to learn a pending layout")

    # Evaluate the shared table rather than duplicating its layout arithmetic.
    names = ["RELAY_V3_SLOT_TICKS", "RELAY_V3_DIRECT_SLOTS", "RELAY_V3_REPAIR_SLOTS",
             "RELAY_V3_DIRECT_MAX", "RELAY_V3_LAST_DIRECT_MAX", "RELAY_V3_GRID_TICKS", "RELAY_V3_GRID_FRAMES"]
    with tempfile.TemporaryDirectory() as temporary:
        source = Path(temporary) / "geometry.c"
        binary = Path(temporary) / "geometry"
        source.write_text('#include <stdio.h>\n#include "relay_protocol.h"\nint main(void) {\n' +
            ''.join(f'printf("%u\\n", (unsigned){name});\n' for name in names) + '}\n')
        subprocess.run(shlex.split(os.environ.get('CC', 'cc')) + ['-I', str(TRACKER / 'src/connection'), str(source), '-o', str(binary)], check=True)
        v3 = dict(zip(names, map(int, subprocess.check_output([str(binary)], text=True).split())))
    s3 = v3["RELAY_V3_SLOT_TICKS"]
    latest32 = (s3 - t["TDMA_LEN18_32_RESERVE_TICKS"]) * TICK_US
    latest32 += CALL_LATENCY_US + RAMP_US + airtime_us(v3["RELAY_V3_LAST_DIRECT_MAX"])
    earliest_b = (s3 + t["TDMA_SLOT_TARGET_OFFSET"] - SYNC_ERROR_TICKS) * TICK_US + RAMP_US
    for mcu, disable in (("nRF52840", 4), ("nRF54L15", 2)):
        check(latest32 + disable + RAMP_US + RX_SETTLE_US <= earliest_b,
              f"v3 last direct->B {mcu}: {earliest_b - latest32 - disable - RAMP_US - RX_SETTLE_US:.3f} us slack")
    b_end = t["TDMA_SLOT_TARGET_OFFSET"] * TICK_US + RAMP_US + airtime_us(72)
    r0_preamble = (s3 + t["TDMA_SLOT_TARGET_OFFSET"]) * TICK_US + RAMP_US
    check(r0_preamble - b_end - SYNC_ERROR_TICKS * TICK_US >= TURNAROUND_US,
          "v3 B->R0 leaves the receiver turnaround allowance after relative clock error")
    r0_admission = (s3 + t["TDMA_SLOT_TARGET_OFFSET"]) * TICK_US
    check(r0_admission - b_end >= 60, "v3 B decode and ownership transfer have 60 us")
    worst = 0
    for phase in range(v3["RELAY_V3_GRID_FRAMES"]):
        start = lambda frame: (frame * v3["RELAY_V3_GRID_TICKS"]) // v3["RELAY_V3_GRID_FRAMES"]
        for source in range(v3["RELAY_V3_DIRECT_SLOTS"]):
            original = start(phase) + source * s3 + t["TDMA_SLOT_TARGET_OFFSET"]
            for repair in range(v3["RELAY_V3_REPAIR_SLOTS"]):
                for age in range(3):
                    finish = start(phase + age) + (v3["RELAY_V3_DIRECT_SLOTS"] + 1 + repair) * s3
                    finish += s3 - t["TDMA_LEN33_PLUS_RESERVE_TICKS"]
                    delay = (finish - original) * TICK_US + CALL_LATENCY_US + airtime_us(72)
                    delay += SYNC_ERROR_TICKS * TICK_US
                    worst = max(worst, delay)
    check(worst < 25000, f"v3 source/repair/age/phase sweep worst {worst:.3f} us < 25 ms")

    # Table.
    print("\nLatency added by relay (worst case = one frame + one slot) and head RX cost per child:")
    print(" trackers  slot  frame_ms  worst_add_ms  head_rx_duty_typ  head_rx_duty_max")
    for n in (4, 6, 8, 11, 16):
        cap = 220
        ticks = max(S, min(64, (32768 // (n * cap)) & ~1))
        frame = ticks * n
        worst = (frame + ticks) * TICK_US / 1000
        typ = (r["RELAY_RX_LEAD_TICKS"] + t["TDMA_SLOT_TARGET_OFFSET"]) * TICK_US + RAMP_US + airtime_us(17)
        print(f" {n:8d}  {ticks:4d}  {frame * TICK_US / 1000:8.2f}  {worst:12.2f}  "
              f"{typ / (frame * TICK_US) * 100:15.1f}%  {(ticks + r['RELAY_RX_LEAD_TICKS']) / frame * 100:15.1f}%")
    presence = (2 * S + t["TDMA_SLOT_TARGET_OFFSET"] + t["TDMA_PING_LATE_TOLERANCE_TICKS"]) * TICK_US
    presence += r["RELAY_RX_TAIL_US"]
    print(f"\nPresence listening at the minimum slot width, including RX tail: {presence / 1000:.2f} ms/window."
          f" HFCLK startup lead: {r['RELAY_PRESENCE_CLOCK_LEAD_US']} us/window."
          " Wider slots and head HFCLK/CPU costs must also be included in battery measurements.")
    if failures:
        raise SystemExit(f"{len(failures)} budget check(s) failed")


if __name__ == "__main__":
    main()
