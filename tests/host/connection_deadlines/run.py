#!/usr/bin/env python3
"""Exercise production deadline/admission boundaries with host kernel leaves."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'harness/python'))
from c_extract import extract_block

HERE = Path(__file__).resolve().parent
root = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2]))
connection = (root / "src/connection/connection.c").read_text()
tdma = (root / "src/connection/tdma.c").read_text()
capture = (root / "src/connection/radio_capture.c").read_text()
esb = (root / "src/connection/esb.c").read_text()
esb_header = (root / "src/connection/esb.h").read_text()


def function(source, name):
    return extract_block(source, rf"^(?:static )?(?:enum tdma_ping_admission|void|bool|int64_t|uint32_t|uint8_t) {re.escape(name)}\([^;{{]*\)\s*\{{")

prefix = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define CLAMP(x,lo,hi) MIN(MAX(x,lo),hi)
#define LOG_WRN(...) ((void)0)
#define LOG_INF(...) ((void)0)
#define ARG_UNUSED(x) ((void)(x))
#define PING_INTERVAL_MS 1000
#define TIME_SYNC_TIMEOUT_MS 15000
#define TDMA_MAX_TRACKERS 16
#define K_NO_WAIT 0
#define K_USEC(x) ((uint64_t)(x))
#define K_MSEC(x) ((uint64_t)(x) * 1000)
typedef int64_t atomic_t;
static int64_t atomic_get(const atomic_t *p) { return *p; }
enum tdma_ping_admission { TDMA_PING_UNAVAILABLE, TDMA_PING_DEFERRED, TDMA_PING_ADMITTED };
static atomic_t tdma_cfg_pack, tdma_runtime_enabled, tdma_epoch_at;
static uint64_t virtual_us;
static uint32_t kernel_hz, sleep_calls, busy_calls, clock_offset;
static uint64_t first_sleep_us;
static int64_t sync_age;
static bool invalidate_on_sleep, change_layout_on_sleep, disable_on_sleep, freeze_clock;
static unsigned irq_lock(void) { return 0; }
static void irq_unlock(unsigned key) { (void)key; }
static bool server_time_synced;
static int64_t g_last_sync_timestamp;
struct esb_sync_filter {
	uint32_t offset;
	uint32_t last_local_ticks;
	uint32_t reference_offset;
	uint32_t reference_local_ticks;
	int32_t skew_ppb;
	uint32_t update_count;
	bool initialized;
};
static struct esb_sync_filter sync_filter;

static uint32_t receiver_tick(void) { return (uint32_t)(virtual_us * 32768 / 1000000) + clock_offset; }
static uint64_t k_uptime_ticks(void) {
    server_time_synced = sync_age >= 0;
    g_last_sync_timestamp = (int64_t)(virtual_us / 1000) - sync_age;
    sync_filter.offset = clock_offset;
    return virtual_us;
}
static uint64_t k_ticks_to_ms_floor64(uint64_t ticks) { return ticks / 1000; }
static uint64_t net_ticks_from_kernel64(uint64_t ticks) { return ticks * 32768 / 1000000; }
static uint64_t k_ticks_to_us_floor64(uint64_t ticks) { return ticks; }
static void k_sleep(uint64_t us) {
    if (sleep_calls++ == 0) first_sleep_us = us;
    /* Ceil timeout conversion plus one kernel tick models phase uncertainty. */
    uint64_t kticks = (us * kernel_hz + 999999) / 1000000 + 1;
    if (!freeze_clock) virtual_us += (kticks * 1000000 + kernel_hz - 1) / kernel_hz;
    if (invalidate_on_sleep) sync_age = -1;
    if (change_layout_on_sleep) ++tdma_epoch_at;
    if (disable_on_sleep) tdma_runtime_enabled = 0;
}
static void k_busy_wait(uint32_t us) { ++busy_calls; if (!freeze_clock) virtual_us += us; }
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
/* The modeled kernel uses microsecond ticks; the network conversion is real. */
static uint64_t k_ticks_to_us_near64(uint64_t ticks) { return ticks; }
static uint64_t test_rate_delay_us(uint64_t now) { (void)now; return test_delay_us; }
static int k_sem_take(int *sem, uint64_t us) { (void)sem; observed_wait_us = us; return 0; }
'''
parts = [prefix, extract_block(esb_header, r"^struct esb_network_clock \{", True),
         function(esb, "esb_project_network_ticks"),
         function(esb, "esb_get_network_clock"), function(esb, "esb_get_sync_age_ms"),
         tdma[tdma.index("#define TDMA_PACK("):tdma.index("\nvoid tdma_init(")],
         tdma[tdma.index("#define TDMA_LEN17_RESERVE_TICKS"):tdma.index("\nvoid tdma_note_radio_busy")],
         function(tdma, "tdma_admission_stalled"), function(tdma, "tdma_update_config")]
parts += [capture[capture.index("#define CAPTURE_METRICS"):capture.index("BUILD_ASSERT")],
          capture[capture.index("struct capture_metric {"):capture.index("static struct capture_record capture_queue")]]
parts += [r'''
#define ESB_SENSOR_DATA_LEN 17
#define __DMB() ((void)0)
#define K_FOREVER 0
static struct capture_record capture_queue[CAPTURE_QUEUE_SIZE];
static uint8_t capture_head, capture_tail;
static struct ping_capture_record ping_queue[CAPTURE_QUEUE_SIZE];
static uint8_t ping_head, ping_tail;
static struct capture_metric metrics[CAPTURE_METRICS];
static struct ping_capture_metric ping_metric;
static int capture_metrics_lock;
static uint64_t capture_platform_now(void) { return virtual_us; }
static void k_mutex_lock(int *lock, int timeout) { (void)lock; (void)timeout; }
static void k_mutex_unlock(int *lock) { (void)lock; }
static bool tdma_config_get(uint8_t *slot, uint8_t *width, uint16_t *frame) {
    struct tdma_config c;
    if (!tdma_config_snapshot(&c)) return false;
    *slot = c.slot_index; *width = c.slot_ticks; *frame = c.frame_ticks;
    return true;
}
''']
parts += [function(capture, name) for name in
          ("metric_index", "duration_bucket", "phase_bucket", "radio_capture_process")]
parts += [function(connection, name) for name in ("connection_next_deadline_ms", "connection_idle_wait")]
start = connection.index("#if CONFIG_SENSOR_USE_MAGNETOMETER\n\t\tbool mag_due =")
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
    sync_age = 0; sleep_calls = 0; first_sleep_us = 0; clock_offset = 0;
    invalidate_on_sleep = change_layout_on_sleep = disable_on_sleep = freeze_clock = false;
    uint64_t target = 32768 + TDMA_SLOT_TARGET_OFFSET;
    uint64_t now = target - 128 - excess;
    virtual_us = (now * 1000000 + 32767) / 32768;
    assert(receiver_tick() == now);
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
            uint64_t remaining = target - receiver_tick();
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
        set_ping_before(0);
        change_layout_on_sleep = true;
        assert(tdma_wait_for_ping_window() == TDMA_PING_DEFERRED);
        assert(tdma_last_ping_frame == UINT64_MAX);
        set_ping_before(0);
        disable_on_sleep = true;
        assert(tdma_wait_for_ping_window() == TDMA_PING_DEFERRED);
        assert(tdma_last_ping_frame == UINT64_MAX);
        set_ping_before(0);
        invalidate_on_sleep = true;
        assert(tdma_wait_for_ping_window() == TDMA_PING_DEFERRED);
        assert(tdma_last_ping_frame == UINT64_MAX);
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
                        sync_age = 0; invalidate_on_sleep = false; clock_offset = 0;
                        virtual_us = (tick * 1000000 + 32767) / 32768;
                        uint64_t before = virtual_us;
                        assert(!tdma_wait_for_data_admission(8));
                        assert(virtual_us == before); /* Yield, do not sleep through PING. */
                        assert(tdma_wait_for_ping_window() == TDMA_PING_ADMITTED);
                        assert(receiver_tick() <= target + TDMA_PING_LATE_TOLERANCE_TICKS);
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
static void admission_stall(void) {
    /* The retry cadence in connection.c backs off exactly while this predicate
     * holds, so it must match an admission that is refused without any radio
     * work: otherwise a disconnect wait spins on hopeless writes. */
    const uint8_t width = 16, slots = 1, reserve = 8;
    kernel_hz = 32768;
    for (int64_t age = -1; age <= PING_INTERVAL_MS * 10 + 1; age += 500) {
        tdma_runtime_enabled = 1;
        tdma_cfg_pack = TDMA_PACK(0, width, slots * width);
        tdma_last_admitted_frame = UINT64_MAX;
        tdma_last_admitted_pack = 0;
        tdma_last_ping_frame = UINT64_MAX;
        tdma_last_ping_pack = 0;
        sync_age = age; invalidate_on_sleep = false;
        virtual_us = 1100000;
        uint64_t before = virtual_us;
        bool admitted = tdma_wait_for_data_admission(reserve);
        if (tdma_admission_stalled()) {
            assert(!admitted && virtual_us == before);
        }
    }
    /* TDMA off is not a stall: admission is not TDMA-gated then. */
    tdma_runtime_enabled = 0;
    sync_age = -1;
    assert(!tdma_admission_stalled());
    tdma_runtime_enabled = 1;
    sync_age = 0;
    assert(!tdma_admission_stalled());
    sync_age = PING_INTERVAL_MS * 10;
    assert(!tdma_admission_stalled());
    sync_age = PING_INTERVAL_MS * 10 + 1;
    assert(tdma_admission_stalled());
    sync_age = -1;
    assert(tdma_admission_stalled());
    puts("PASS admission stall predicate matches refusals that consume no radio time");
}
static void set_receiver_tick(uint32_t tick) {
    clock_offset = 0;
    virtual_us = ((uint64_t)tick * 1000000U + 32767U) / 32768U;
    assert(receiver_tick() == tick);
}
static void configure(uint8_t slots, uint8_t width, uint8_t slot) {
    tdma_runtime_enabled = 1;
    tdma_epoch_at = 1;
    tdma_cfg_pack = TDMA_PACK(slot, width, (uint16_t)slots * width);
    tdma_last_ping_frame = tdma_last_admitted_frame = UINT64_MAX;
    tdma_last_ping_pack = tdma_last_admitted_pack = 0;
    sync_age = 0;
    invalidate_on_sleep = change_layout_on_sleep = disable_on_sleep = freeze_clock = false;
    sleep_calls = busy_calls = 0;
}
static void receiver_clock_rollover(void) {
    kernel_hz = 32768;
    configure(3, 48, 0);
    set_receiver_tick(1122);
    clock_offset = 0x80000000U;
    assert(receiver_tick() == 2147484770U);
    bool admitted = tdma_wait_for_data_admission(8);
    assert(!admitted || receiver_tick() != 2147484770U);
    configure(3, 48, 2);
    set_receiver_tick(2147484770U);
    assert(tdma_wait_for_data_admission(8));
    assert(receiver_tick() == 2147484770U);

    unsigned crossing_pings = 0, eligible_data = 0;
    for (unsigned slots = 1; slots <= 16; ++slots) {
        for (unsigned width = 16; width <= 255; ++width) {
            for (unsigned slot = 0; slot < slots; ++slot) {
                configure(slots, width, slot);
                struct tdma_config config;
                assert(tdma_config_snapshot(&config));
                uint32_t last = UINT32_MAX / config.frame_ticks;
                set_receiver_tick(last * config.frame_ticks);
                uint32_t delay = 0;
                assert(tdma_wait_for_ping_window() == TDMA_PING_DEFERRED);
                assert(tdma_ping_wake_delay_ms(&delay) && delay > 0);
                assert(sleep_calls == 0);
                set_receiver_tick(0);
                assert(tdma_wait_for_ping_window() == TDMA_PING_DEFERRED);
                assert(tdma_ping_wake_delay_ms(&delay) && delay > 0);
                assert(sleep_calls == 0);

                uint32_t previous = last - 1U;
                uint32_t period = tdma_ping_period_frames(config.frame_ticks);
                uint64_t start = (uint64_t)previous * config.frame_ticks
                    + slot * width + TDMA_SLOT_TARGET_OFFSET;
                bool scheduled = previous % period == tdma_ping_phase_frame(slot, slots, period);
                bool fits = start + TDMA_PING_LATE_TOLERANCE_TICKS
                    + width * TDMA_PING_WINDOW_SLOTS <= 0x100000000ULL;
                assert(tdma_ping_frame_eligible(previous, slot, &config) == (scheduled && fits));
                if (scheduled && !fits) {
                    ++crossing_pings;
                    uint32_t next_frame, target;
                    bool in_window;
                    assert(tdma_ping_target((uint32_t)start - 1U, &config,
                        &next_frame, &target, &in_window));
                    assert(!in_window && next_frame != previous);
                    bool own_ping;
                    if (!tdma_data_frame_guarded(previous, &config, &own_ping)) {
                        set_receiver_tick((uint32_t)start);
                        assert(tdma_wait_for_data_admission(8));
                        assert(receiver_tick() == (uint32_t)start);
                        ++eligible_data;
                    }
                }
                bool own_ping;
                bool guarded = tdma_data_frame_guarded(previous, &config, &own_ping);
                uint8_t previous_slot = slot ? slot - 1U : slots - 1U;
                uint32_t previous_guard_frame = slot ? previous : previous - 1U;
                assert(own_ping == tdma_ping_frame_eligible(previous, slot, &config));
                assert(guarded == (own_ping
                    || tdma_ping_frame_eligible(previous_guard_frame, previous_slot, &config)));
            }
        }
    }
    assert(crossing_pings > 0 && eligible_data > 0);
    configure(8, 16, 0);
    set_receiver_tick(10000);
    struct tdma_config config;
    struct esb_network_clock clock;
    assert(tdma_config_snapshot(&config) && tdma_clock_snapshot(&config, &clock));
    freeze_clock = true;
    assert(!tdma_wait_until_network_tick(&config, clock, clock.receiver_ticks + 3));
    assert(busy_calls == TDMA_FINE_SPINS);
    busy_calls = sleep_calls = 0;
    assert(!tdma_wait_until_network_tick(&config, clock, clock.receiver_ticks + 100));
    assert(sleep_calls == TDMA_COARSE_SLEEPS && busy_calls == 0);
    freeze_clock = false;
    puts("PASS receiver32 epochs, rollover exclusions, all layouts, guards and wait bounds");
}
static void capture_phases(void) {
    configure(3, 48, 2);
    ping_queue[0] = (struct ping_capture_record){
        .start_server_ticks = 2147484770U, .duration_us = 305,
        .frame_ticks = 144, .slot_index = 2, .slot_ticks = 48,
        .attempts = 1, .success = true, .have_start = true, .schedule_valid = true
    };
    ping_head = 1;
    radio_capture_process();
    assert(ping_metric.phase_samples == 1);
    assert(ping_metric.start_phase[34] == 1 && ping_metric.end_phase[44] == 1);
    ping_queue[1] = ping_queue[0];
    ping_queue[1].start_server_ticks = UINT32_MAX - 2U;
    ping_head = 2;
    radio_capture_process();
    assert(ping_metric.success[0] == 2 && ping_metric.phase_samples == 1);
    ping_queue[2] = ping_queue[0];
    ping_queue[2].start_server_ticks = 0;
    ping_queue[2].slot_index = 0;
    ping_head = 3;
    radio_capture_process();
    assert(ping_metric.phase_samples == 2 && ping_metric.start_phase[32] == 1);
    assert(ping_metric.start_missing == 0);

    set_receiver_tick(2147484790U);
    capture_queue[0] = (struct capture_record){
        .ready_us = virtual_us - 305, .end_us = virtual_us - 62, .payload_length = 17
    };
    capture_head = 1;
    radio_capture_process();
    assert(metrics[0].phase_samples == 1);
    assert(metrics[0].ready_phase[44] == 1 && metrics[0].end_phase[51] == 1);

    configure(3, 48, 2);
    set_receiver_tick(100U * 144U + 149U);
    capture_queue[1] = (struct capture_record){
        .ready_us = virtual_us - 305, .end_us = virtual_us - 30, .payload_length = 17
    };
    capture_head = 2;
    radio_capture_process();
    assert(metrics[0].phase_samples == 2 && metrics[0].crossed_slot_end == 1);
    set_receiver_tick(1000);
    clock_offset = 0U - 996U;
    capture_queue[2] = (struct capture_record){
        .ready_us = virtual_us - 305, .end_us = virtual_us - 30, .payload_length = 17
    };
    capture_head = 3;
    radio_capture_process();
    assert(metrics[0].samples == 3 && metrics[0].phase_samples == 2);
    puts("PASS capture receiver32 phases, frame crossing and wrap classification");
}
static void validate_layout(void) {
    const uint8_t widths[] = {16, 64, 182, 254, 255};
    for (unsigned i = 0; i < sizeof(widths); ++i) {
        assert(tdma_update_config(15, 16, widths[i], 255));
    }
    atomic_t old_pack = tdma_cfg_pack, old_epoch = tdma_epoch_at;
    assert(!tdma_update_config(0, 0, 16, 1));
    assert(!tdma_update_config(0, 17, 16, 1));
    assert(!tdma_update_config(1, 1, 16, 1));
    assert(!tdma_update_config(0, 1, 15, 1));
    assert(tdma_cfg_pack == old_pack && tdma_epoch_at == old_epoch);
    assert(tdma_update_config(0, 1, 16, 0));
}
int main(int argc, char **argv) {
    validate_layout();
    assert(argc == 2);
    if (strcmp(argv[1], "deadlines") == 0) deadlines();
    else if (strcmp(argv[1], "ping") == 0) ping();
    else if (strcmp(argv[1], "data-ping") == 0) data_ping_boundary();
    else if (strcmp(argv[1], "stall") == 0) admission_stall();
    else if (strcmp(argv[1], "rollover") == 0) receiver_clock_rollover();
    else if (strcmp(argv[1], "capture") == 0) capture_phases();
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
        "-fno-pie", "-no-pie",
        "-DCONFIG_CONNECTION_RAW_COLLECTION=1", "-DCONFIG_SENSOR_USE_MAGNETOMETER=1",
        str(source), "-o", str(binary)], check=True)
    for selected in (["deadlines", "ping", "data-ping", "stall", "rollover", "capture"] if case == "all" else [case]):
        subprocess.run([str(binary), selected], check=True)
