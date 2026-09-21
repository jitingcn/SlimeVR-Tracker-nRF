#!/usr/bin/env python3
"""Compile the real event core with deterministic kernel/entropy leaves.

No event state machine is copied. Each case gets a fresh firmware boot. The
optional --wire-output file can be replayed by the receiver harness, which uses
its production decoder/deduplicator and the reference Python client parser.
"""
import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2]))

LEAVES = r'''
#pragma once
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
static uint32_t now_ms;
static unsigned lock_depth, entropy_calls, error_logs, wake_calls;
static bool entropy_error, entropy_zero;
struct k_spinlock { int unused; };
typedef unsigned k_spinlock_key_t;
static k_spinlock_key_t k_spin_lock(struct k_spinlock *lock) {
    (void)lock; assert(lock_depth==0); lock_depth++; return 0;
}
static void k_spin_unlock(struct k_spinlock *lock, k_spinlock_key_t key) {
    (void)lock; (void)key; assert(lock_depth==1); lock_depth--;
}
static uint32_t k_uptime_get_32(void) { return now_ms; }
static int64_t k_uptime_get(void) { return now_ms; }
static int sys_csrand_get(void *dest,size_t len) {
    assert(lock_depth==0 && len==sizeof(uint32_t));
    entropy_calls++;
    uint32_t nonce=entropy_zero?0:0x12345678U+entropy_calls-1;
    memcpy(dest,&nonce,len); return entropy_error?-EIO:0;
}
void connection_tracker_event_wake(void) { assert(lock_depth==0); wake_calls++; }
#define CONFIG_APPLICATION_INIT_PRIORITY 90
#define CONFIG_TRACKER_EVENT_CALIBRATION 1
#define CONFIG_TRACKER_EVENT_TRACKER_REST 1
#define CONFIG_TRACKER_EVENT_FUSION_REST 1
#define CONFIG_TRACKER_EVENT_POWER 1
#define APPLICATION 0
#define SYS_INIT(fn,level,priority) static int (*host_init)(void)=fn
#define LOG_MODULE_REGISTER(...)
#define LOG_ERR(...) do { assert(lock_depth==0); error_logs++; } while(0)
#define LOG_WRN(...) do { assert(lock_depth==0); } while(0)
#define LOG_INF(...) do { assert(lock_depth==0); } while(0)
#define LOG_DBG(...) do { assert(lock_depth==0); } while(0)
'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--case", action="append")
    parser.add_argument("--wire-output", type=Path)
    parser.add_argument("--end-to-end", action="store_true")
    parser.add_argument("--json-output", type=Path)
    args = parser.parse_args()
    cases = args.case or ["copies", "lifecycle", "progress", "online", "states", "receipts", "scheduling", "state-heartbeat", "entropy-error", "entropy-zero",
                          "startup-boot", "startup-wake", "startup-watchdog", "startup-wake-watchdog",
                          "startup-late", "startup-wrap", "startup-session", "power-priority", "wom-cancellation",
                          "terminal-priority", "terminal-before-startup", "terminal-before-schedule",
                          "startup-disconnected", "startup-wrap-sentinel", "next-send-wrap-sentinel"]
    cc = shlex.split(os.environ.get("CC", "cc"))
    with tempfile.TemporaryDirectory(prefix="tracker-events-") as temp:
        temp = Path(temp)
        for name in ("kernel.h", "init.h", "random/random.h", "logging/log.h"):
            header = temp / "zephyr" / name
            header.parent.mkdir(parents=True, exist_ok=True)
            header.write_text('#include "leaves.h"\n')
        (temp / "leaves.h").write_text(LEAVES)
        source = temp / "core.c"
        source.write_text('#include "leaves.h"\n#include "' + str(ROOT / "src/connection/tracker_events.c") + '"\n' + (HERE / "test_core.c").read_text())
        flags = ["-std=c11", "-Wall", "-Wextra", "-Wno-unused-function", "-Wno-unused-variable", "-Wno-misleading-indentation", "-O0", "-g", "-I", str(temp), "-I", str(ROOT / "src")]
        motion = temp / "motion"
        subprocess.run(cc + flags + [str(HERE / "test_motion.c"), str(ROOT / "src/sensor/motion_state.c"), str(ROOT / "src/util.c"), "-lm", "-o", str(motion)], check=True)
        subprocess.run([str(motion)], check=True)
        protocol = temp / "protocol"
        subprocess.run(cc + flags + [str(HERE / "test_protocol.c"), "-o", str(protocol)], check=True)
        subprocess.run([str(protocol)], check=True)
        binary = temp / "core"
        subprocess.run(cc + flags + [str(source), "-o", str(binary)], check=True)
        for case in cases:
            subprocess.run([str(binary), case], check=True)
        if not args.case:
            switches = ("CALIBRATION", "TRACKER_REST", "FUSION_REST", "POWER")
            for mask in range(16):
                disabled = "".join(
                    f"#undef CONFIG_TRACKER_EVENT_{name}\n"
                    for bit, name in enumerate(switches) if not mask & (1 << bit)
                )
                source.write_text(
                    '#include "leaves.h"\n' + disabled
                    + f"#define TEST_EVENT_MASK {mask}\n"
                    + '#include "' + str(ROOT / "src/connection/tracker_events.c") + '"\n'
                    + (HERE / "test_config.c").read_text()
                )
                config_binary = temp / "config"
                subprocess.run(cc + flags + [str(source), "-o", str(config_binary)], check=True)
                subprocess.run([str(config_binary)], check=True)
        if args.wire_output or args.end_to_end:
            wire = subprocess.check_output([str(binary), "wire"], text=True)
            destination = args.wire_output or temp / "wire.txt"
            destination.write_text(wire)
            if args.end_to_end:
                receiver = ROOT.parent / "SlimeVR-Tracker-nRF-Receiver/tests/host/tracker_events/run.py"
                output = subprocess.check_output([sys.executable, str(receiver), "--wire-input", str(destination)], text=True)
                records = [json.loads(line) for line in output.splitlines()]
                assert any(r["event"] == "END" and r["outcome"] == "SUCCESS" for r in records)
                assert any(r["event"] == "END" and r["outcome"] == "FAILED" for r in records)
                assert any(r["source"] == "receiver" and r["outcome"] == "UNKNOWN" and r["event"] == "END" for r in records)
                buttons = [r for r in records if r["kind"] == "BUTTON"]
                assert len(buttons) == 1 and buttons[0]["count"] == 3 and buttons[0]["count_exact"]
                powers = [r for r in records if r["kind"] == "POWER"]
                assert len(powers) == 1 and powers[0]["phase"] == "WILL_WOM" and powers[0]["outcome"] == "NONE"
                assert any(r["kind"] == "IMU_ZRO" and r["phase"] == "COVERAGE" and r["detail"] == 99 for r in records)
                snapshots = [r for r in records if r["snapshot"] and r["kind"] == "TRACKER_REST"]
                assert len(snapshots) >= 2 and snapshots[-1]["phase"] == "REST"
                assert any(r["source"] == "receiver" and r["kind"] == "TRACKER_REST" and r["outcome"] == "UNKNOWN" for r in records)
                if args.json_output:
                    args.json_output.write_text(output)
                print(output, end="")
    if not args.case:
        subprocess.run([sys.executable, str(HERE / "transport.py")], check=True)
        subprocess.run([sys.executable, str(HERE / "sensor.py")], check=True)


if __name__ == "__main__":
    main()
