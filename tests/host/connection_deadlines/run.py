#!/usr/bin/env python3
"""Exercise production deadline/admission boundaries with host kernel leaves."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
root = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2]))
connection = (root / "src/connection/connection.c").read_text()
tdma = (root / "src/connection/tdma.c").read_text()


def function(source, name):
    match = re.search(rf"^(?:static )?(?:enum tdma_ping_admission|void|bool|int64_t|uint32_t) {name}\([^;{{]*\)\s*\{{", source, re.MULTILINE)
    if match is None:
        raise ValueError(name)
    start = source.index("{", match.start())
    tokens = re.compile(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]', re.DOTALL)
    depth = 0
    for token in tokens.finditer(source, start):
        if token.group() == "{":
            depth += 1
        elif token.group() == "}":
            depth -= 1
            if depth == 0:
                return source[match.start():token.end()]
    raise ValueError(f"Unclosed: {name}")

prefix = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define PING_INTERVAL_MS 1000
#define TDMA_COARSE_WAKE_LEAD_TICKS 4
#define K_NO_WAIT 0
#define K_USEC(x) ((uint64_t)(x))
#define K_MSEC(x) ((uint64_t)(x) * 1000)
typedef int64_t atomic_t;
static int64_t atomic_get(const atomic_t *p) { return *p; }
enum tdma_ping_admission { TDMA_PING_UNAVAILABLE, TDMA_PING_DEFERRED, TDMA_PING_ADMITTED };
static atomic_t tdma_cfg_pack, tdma_runtime_enabled;
static uint64_t virtual_us;
static uint32_t kernel_hz, sleep_calls;
static uint64_t first_sleep_us;
static int64_t sync_age;
static bool invalidate_on_sleep;
static uint64_t esb_get_server_time_ticks_64(void) { return virtual_us * 32768 / 1000000; }
static int64_t esb_get_sync_age_ms(void) { return sync_age; }
static void k_sleep(uint64_t us) {
    if (sleep_calls++ == 0) first_sleep_us = us;
    /* Ceil timeout conversion plus one kernel tick models phase uncertainty. */
    uint64_t kticks = (us * kernel_hz + 999999) / 1000000 + 1;
    virtual_us += (kticks * 1000000 + kernel_hz - 1) / kernel_hz;
    if (invalidate_on_sleep) sync_age = -1;
}
static void k_busy_wait(uint32_t us) { virtual_us += us; }
static int connection_wake_sem, sensor_data_snapshot;
static bool sensor_ids_set, m_pending, test_enabled, radio_ready, event_pending;
static int64_t last_mag_time, last_info_time, last_status_time, last_runtime_time;
static atomic_t next_ping_deadline_ms;
static bool test_wake_delay_valid;
static uint64_t test_wake_delay_us, test_delay_us, observed_wait_us;
static bool esb_ready(void) { return radio_ready; }
static bool esb_ota_is_active(void) { return false; }
#define SYS_STATUS_CONNECTION_ERROR 1
static bool get_status(int status) { (void)status; return false; }
static uint32_t tracker_events_deadline(uint32_t now) { return event_pending ? now : UINT32_MAX; }
static bool test_mode_get(void) { return test_enabled; }
static bool sensor_data_snapshot_m_pending(const int *unused) { (void)unused; return m_pending; }
static uint64_t k_uptime_ticks(void) { return virtual_us; }
static uint64_t k_ticks_to_us_near64(uint64_t ticks) { return ticks; }
static uint64_t test_rate_delay_us(uint64_t now) { (void)now; return test_delay_us; }
static int k_sem_take(int *sem, uint64_t us) { (void)sem; observed_wait_us = us; return 0; }
'''
parts = [prefix,
         tdma[tdma.index("#define TDMA_PACK("):tdma.index("\nvoid tdma_init(")],
         tdma[tdma.index("#define TDMA_LEN17_RESERVE_TICKS"):tdma.index("\nstatic uint32_t tdma_ping_period_frames")]]
parts += [function(tdma, name) for name in (
    "tdma_ping_period_frames", "tdma_ping_phase_frame", "tdma_sleep_network_ticks",
    "tdma_wait_until_network_tick", "tdma_config_snapshot", "tdma_ping_target",
    "tdma_ping_wake_delay_ms", "tdma_wait_for_ping_window",
    "tdma_slot_ping_frame", "tdma_data_frame_guarded", "tdma_wait_for_data_admission")]
parts += [function(connection, name) for name in ("connection_next_deadline_ms", "connection_idle_wait")]
start = connection.index("\t\tbool mag_due =")
end = connection.index("\n\n", start)
parts += ["static unsigned due_mask(int64_t now) {\n bool in_test_mode = test_mode_get();\n" + connection[start:end] + "\nreturn mag_due | (info_due << 1) | (status_due << 2) | (runtime_due << 3);\n}"]
parts += [r'''
static void deadlines(void) {
    for (unsigned field = 0; field < 4; ++field) {
        int64_t interval = field < 2 ? 100 : 1000;
        m_pending = field == 0; sensor_ids_set = field == 1;
        test_enabled = false;
        last_mag_time = last_info_time = last_status_time = last_runtime_time = 10000;
        int64_t *times[] = { &last_mag_time, &last_info_time, &last_status_time, &last_runtime_time };
        *times[field] = 1000;
        int64_t deadline = 1000 + interval;
        connection_idle_wait(deadline - 1);
        assert(observed_wait_us > 0 && due_mask(deadline - 1) == 0);
        connection_idle_wait(deadline);
        assert(observed_wait_us == 0);
        assert(due_mask(deadline) == (1U << field));
        assert(due_mask(deadline + 1) == (1U << field));
        /* Consumer commits successful send at the due instant. */
        *times[field] = deadline;
        connection_idle_wait(deadline);
        assert(observed_wait_us > 0 && due_mask(deadline) == 0);
    }
    /* Target-rate mode must not emit low-frequency standalone packets. */
    test_enabled = true; test_delay_us = 125;
    m_pending = sensor_ids_set = true;
    last_mag_time = last_info_time = last_status_time = last_runtime_time = 0;
    assert(due_mask(2000) == 0);
    connection_idle_wait(2000);
    assert(observed_wait_us == 125);
    /* A pending event between target poses must not turn the wait into a
     * zero-time poll. Keep radio/PING active so the production gate is tested. */
    radio_ready = event_pending = true;
    next_ping_deadline_ms = 10000;
    tdma_runtime_enabled = 0;
    test_delay_us = 5000;
    connection_idle_wait(2000);
    assert(observed_wait_us == 5000);
    test_delay_us = 3000;
    connection_idle_wait(2002);
    assert(observed_wait_us == 3000);
    test_delay_us = 0;
    connection_idle_wait(2005);
    assert(observed_wait_us == 0); /* Only the scheduled pose is now due. */
    /* Leaving test mode restores immediate independent event service. */
    test_enabled = false;
    m_pending = sensor_ids_set = false;
    last_status_time = last_runtime_time = 2005;
    connection_idle_wait(2005);
    assert(observed_wait_us == 0);
    event_pending = false;
    connection_idle_wait(2005);
    assert(observed_wait_us > 0);
    puts("PASS low-frequency exact boundaries and target-rate isolation");
}
static uint64_t set_ping_before(uint32_t excess) {
    tdma_runtime_enabled = 1;
    tdma_cfg_pack = TDMA_PACK(0, 16, 128);
    tdma_last_ping_frame = tdma_last_admitted_frame = UINT64_MAX;
    tdma_last_ping_pack = tdma_last_admitted_pack = 0;
    sync_age = 0; sleep_calls = 0; first_sleep_us = 0; invalidate_on_sleep = false;
    uint64_t target = 32768 + TDMA_SLOT_TARGET_OFFSET;
    uint64_t now = target - 128 - excess;
    virtual_us = (now * 1000000 + 32767) / 32768;
    assert(esb_get_server_time_ticks_64() == now);
    return target;
}
static void ping(void) {
    const uint32_t rates[] = { 32768, 31250 };
    for (unsigned rate = 0; rate < 2; ++rate) {
        kernel_hz = rates[rate];
        for (uint32_t excess = 1; excess <= 32; ++excess) {
            uint64_t target = set_ping_before(excess);
            uint32_t delay;
            assert(tdma_ping_wake_delay_ms(&delay) && delay == 0);
            assert(tdma_wait_for_ping_window() == TDMA_PING_DEFERRED);
            assert(sleep_calls == 1);
            assert(first_sleep_us == ((uint64_t)excess * 1000000 + 32767) / 32768);
            uint64_t remaining = target - esb_get_server_time_ticks_64();
            /* Full preparation frame minus at most three modeled kernel/network ticks. */
            assert(remaining <= 128 && remaining >= 125);
            assert(tdma_last_ping_frame == UINT64_MAX);
            assert(tdma_last_admitted_frame == UINT64_MAX);
            assert(tdma_wait_for_ping_window() == TDMA_PING_ADMITTED);
            assert(tdma_last_ping_frame == target / 128);
            /* Retry cannot consume this frame twice. */
            assert(tdma_wait_for_ping_window() == TDMA_PING_DEFERRED);
        }
        /* A changed/farther schedule must not block normal traffic for a period. */
        const uint32_t farther[] = { 33, 16000 };
        for (unsigned i = 0; i < 2; ++i) {
            set_ping_before(farther[i]);
            uint64_t before = virtual_us;
            assert(tdma_wait_for_ping_window() == TDMA_PING_DEFERRED);
            assert(sleep_calls == 0 && virtual_us == before);
            assert(tdma_last_ping_frame == UINT64_MAX);
        }
        /* Config/sync can change while this thread yields; no old reservation survives. */
        set_ping_before(16);
        invalidate_on_sleep = true;
        assert(tdma_wait_for_ping_window() == TDMA_PING_DEFERRED);
        assert(tdma_wait_for_ping_window() == TDMA_PING_UNAVAILABLE);
        assert(tdma_last_ping_frame == UINT64_MAX);
        set_ping_before(16);
        assert(tdma_wait_for_ping_window() == TDMA_PING_DEFERRED);
        tdma_runtime_enabled = 0;
        assert(tdma_wait_for_ping_window() == TDMA_PING_UNAVAILABLE);
        assert(tdma_last_admitted_frame == UINT64_MAX);
        /* Exactly one frame early enters existing precise wait, not early DEFERRED. */
        set_ping_before(0);
        assert(tdma_wait_for_ping_window() == TDMA_PING_ADMITTED);
    }
    puts("PASS PING early yield, slot admission, duplicate and sync/config boundaries at both kernel rates");
}
static void data_ping_boundary(void) {
    const uint32_t rates[] = { 32768, 31250 };
    for (unsigned rate = 0; rate < 2; ++rate) {
        kernel_hz = rates[rate];
        for (uint8_t slots = 1; slots <= 14; ++slots) {
            for (uint8_t width = 16; width <= 64; width += 2) {
                for (uint8_t slot = 0; slot < slots; ++slot) {
                    uint16_t frame_ticks = slots * width;
                    uint32_t period = tdma_ping_period_frames(frame_ticks);
                    uint64_t frame = 20 * period + tdma_ping_phase_frame(slot, slots, period);
                    uint64_t target = frame * frame_ticks + slot * width + TDMA_SLOT_TARGET_OFFSET;
                    /* A data write may reach admission after clock startup,
                     * already inside the frame whose PING is still ahead. */
                    for (uint64_t tick = frame * frame_ticks; tick <= target; ++tick) {
                        tdma_runtime_enabled = 1;
                        tdma_cfg_pack = TDMA_PACK(slot, width, frame_ticks);
                        tdma_last_ping_frame = tdma_last_admitted_frame = UINT64_MAX;
                        tdma_last_ping_pack = tdma_last_admitted_pack = 0;
                        sync_age = 0; invalidate_on_sleep = false;
                        virtual_us = (tick * 1000000 + 32767) / 32768;
                        uint64_t before = virtual_us;
                        assert(!tdma_wait_for_data_admission(8));
                        assert(virtual_us == before); /* Yield, do not sleep through PING. */
                        assert(tdma_wait_for_ping_window() == TDMA_PING_ADMITTED);
                        assert(esb_get_server_time_ticks_64() <= target + TDMA_PING_LATE_TOLERANCE_TICKS);
                    }
                    /* Outside the protected frame, data remains admissible. */
                    virtual_us = ((target + 2 * frame_ticks) * 1000000 + 32767) / 32768;
                    assert(tdma_wait_for_data_admission(8));
                }
            }
        }
    }
    puts("PASS data yields to current-frame PING and resumes after guard at both kernel rates");
}
int main(int argc, char **argv) {
    assert(argc == 2);
    if (strcmp(argv[1], "deadlines") == 0) deadlines();
    else if (strcmp(argv[1], "ping") == 0) ping();
    else if (strcmp(argv[1], "data-ping") == 0) data_ping_boundary();
    else return 2;
    return 0;
}
''']
case = sys.argv[1] if len(sys.argv) > 1 else "all"
with tempfile.TemporaryDirectory(prefix="slimenrf-deadline-smoke-") as directory:
    temp = Path(directory)
    source = temp / "deadline.c"
    source.write_text("\n\n".join(parts))
    binary = temp / "deadline"
    subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
        "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-variable",
        "-g", "-O1", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
        "-fno-pie", "-no-pie", str(source), "-o", str(binary)], check=True)
    for selected in (["deadlines", "ping", "data-ping"] if case == "all" else [case]):
        subprocess.run([str(binary), selected], check=True)
