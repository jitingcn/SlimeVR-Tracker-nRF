#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <setjmp.h>
#include <math.h>
#include "../led_feedback_stub.h"
#ifndef CONFIG_SENSOR_TCAL_HEATED
#define CONFIG_SENSOR_TCAL_HEATED 0
#endif
#define CONFIG_SENSOR_TCAL_HEATED_DEFAULT_TARGET_C 44
#define CONFIG_CONNECTION_OVER_HID 0
#define USER_SHUTDOWN_ENABLED 0
#define ESB_ST_PAIRING 0
#define ESB_ST_RECOVERING 2
#define SYS_STATUS_CONNECTION_ERROR 1
#define SYS_STATUS_CALIBRATION_RUNNING 2
#define TX_ERROR_THRESHOLD 10
#define WDT_CHANNEL_ESB 1
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define LOG_DBG(...) ((void)0)
#define LOG_INF(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define LOG_ERR(...) ((void)0)
static uint8_t received_remote_command, acked_remote_command;
static uint16_t received_test_rate_tps, executing_test_rate_tps, acked_test_rate_tps;
static uint8_t received_batch_rate_hz, executing_batch_rate_hz, acked_batch_rate_hz;
static uint8_t received_metadata_mask, received_metadata_chunk, received_sens_auto_axis;
static uint16_t received_metadata_token, received_sens_auto_revolutions;
static bool metadata_echo_pending;
static uint32_t received_channel_value;
static uint32_t executing_channel_value, applied_channel;
static unsigned channel_calls;
static int channel_control_set(int channel) { applied_channel = channel; ++channel_calls; return 0; }
static float received_sens_data[3];
static struct { uint8_t data[13], length; } rx_payload = {.length = 13};
static uint16_t sys_get_be16(const uint8_t *p) { return ((uint16_t)p[0] << 8) | p[1]; }
static uint32_t sys_get_be32(const uint8_t *p)
{
	return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static void connection_request_raw_metadata(uint8_t mask, uint8_t chunk, uint16_t token) {}
static unsigned irq_depth;
static inline unsigned irq_lock(void) { return irq_depth++; }
static inline void irq_unlock(unsigned key) { assert(irq_depth == key + 1); irq_depth = key; }
static void receive_control(uint8_t flag);
static void receive_stop(void);
static bool inject_stop, replaced_shutdown, batch_active = true;
static int64_t remote_command_receive_time = 1, now_ms = 1000;
static uint8_t paired_addr[8];
static struct { uint8_t paired_addr[8]; } retained_state, *retained = &retained_state;
static int esb_conn_state = 1;
static uint32_t ping_failures, ping_success_streak;
static bool ping_pending, ping_failed;
static bool channel_wait_normal, channel_redirect_pending, channel_search;
static bool channel_confirmed = true, channel_legacy_peer;
static unsigned fw_info_requests, abort_requests;
static void esb_ota_handle_query_info(void) { ++fw_info_requests; }
static void esb_ota_request_abort(void) { ++abort_requests; }
static bool radio_user_disabled;
#define K_FOREVER 0
static int esb_radio_lock;
static void k_mutex_lock(int *lock, int timeout) {
    (void)timeout; ++*lock;
}
static void k_mutex_unlock(int *lock) {
    assert(*lock > 0); --*lock;
}
static int64_t ping_send_time, connection_error_start_time;
static unsigned feeds, registrations, shutdown_calls, test_mode_changes;
static unsigned stop_after = 120, reject_count;
static int batch_result;
static jmp_buf thread_stop;
typedef void (*esb_remote_cmd_fn)(void);
static int64_t k_uptime_get(void) { return now_ms; }
static int watchdog_register_thread(int channel, int timeout) { registrations++; return 0; }
#define SYS_REBOOT_COLD 0
static void sys_reboot(int reason) { assert(!"unexpected watchdog reboot"); }
static void watchdog_feed(int channel)
{
    assert(channel == WDT_CHANNEL_ESB);
    feeds++;
    if (shutdown_calls <= reject_count) assert(acked_remote_command != 1);
    if (replaced_shutdown) assert(acked_remote_command != 1);
}
static void k_msleep(int ms)
{
    now_ms += ms;
    if (ms == 1500 && inject_stop && shutdown_calls == 1) {
        inject_stop = false;
        receive_stop();
        replaced_shutdown = true;
    }
    if (feeds >= stop_after) longjmp(thread_stop, 1);
}
static void clocks_request_start(int delay) {}
static void clock_init_external_async(void) {}
static void esb_pair(void) { assert(false); }
static void esb_initialize(bool init) {}
static bool test_mode_get(void) { return false; }
static bool get_status(int status) { return false; }
static void set_status(int status, bool value) {}
static int get_ping_interval_ms(void) { return 997; }
int sys_command_shutdown(void);
static int connection_set_data_collection_batch(bool enabled, uint16_t rate)
{
    if (!enabled) { batch_active = false; return 0; }
    return batch_result;
}
static void test_mode_set(bool active) { test_mode_changes++; }
static void reboot_counter_write(int count) {}
static bool sys_exit_feedback_allowed(bool reboot) { (void)reboot; return false; }
static int sys_request_system_off(void)
{
    shutdown_calls++;
    return shutdown_calls <= reject_count ? -EBUSY : 0;
}
static unsigned heated_calls;
static int heated_result;
#if CONFIG_SENSOR_TCAL_HEATED
static int sensor_tcal_heated_start(float target)
{
    assert(isfinite(target) && target == CONFIG_SENSOR_TCAL_HEATED_DEFAULT_TARGET_C);
    heated_calls++;
    return heated_result;
}
#endif
static uint16_t wom_minutes = 10;
static uint16_t wom_default_minutes = 10, wom_last_set_minutes;
static unsigned wom_writes, wom_queries;
static int wom_set_error;
static bool inject_wom_request, reset_during_wom;
static bool in_wom_rx;
static unsigned wom_enter_calls;
static int wom_enter_error;
static int sys_request_WOM_now(void)
{
    assert(!in_wom_rx && irq_depth == 0);
    ++wom_enter_calls;
    return wom_enter_error;
}
static uint16_t test_mode_get_target_tps(void) { return 250; }
static uint16_t connection_get_data_collection_batch_rate(void) { return 100; }
static void esb_wom_receive(uint16_t minutes, uint16_t token);
static void esb_wom_reset(void);
static void esb_clear_time_sync_state(void) {}
static void tdma_set_enabled(bool enabled) { (void)enabled; }
static void connection_request_ping_resync(void) {}
static uint16_t wom_timeout_get_minutes(void) { ++wom_queries; return wom_minutes; }
static int wom_timeout_set_minutes(uint16_t minutes)
{
    ++wom_writes;
    wom_last_set_minutes = minutes;
    if (inject_wom_request) {
        inject_wom_request = false;
        esb_wom_receive(30, 101);
    }
    if (reset_during_wom) {
        reset_during_wom = false;
        esb_wom_reset();
    }
    if (wom_set_error) return wom_set_error;
    wom_minutes = minutes ? minutes : wom_default_minutes;
    return 0;
}
#include "commands.inc"

static void run_shutdown(unsigned rejected)
{
    feeds = registrations = shutdown_calls = 0;
    reject_count = rejected;
    acked_remote_command = ESB_PONG_FLAG_NORMAL;
    received_remote_command = ESB_PONG_FLAG_SHUTDOWN;
    int64_t start = now_ms;
    if (setjmp(thread_stop) == 0) {
        esb_thread();
        assert(!"ESB thread exited before physical power termination");
    }
    assert(registrations == 1 && feeds == stop_after);
    assert(shutdown_calls == rejected + 1);
    assert(acked_remote_command == ESB_PONG_FLAG_SHUTDOWN);
    assert(now_ms - start > 10000);
}

static void run_heated_request(int result)
{
    receive_control(ESB_PONG_FLAG_NORMAL);
    heated_calls = 0;
    heated_result = result;
    receive_control(ESB_PONG_FLAG_TCAL_HEATED_START);
    now_ms += 100;
    feeds = 0;
    stop_after = 3;
    if (setjmp(thread_stop) == 0) esb_thread();
    assert(acked_remote_command == ESB_PONG_FLAG_TCAL_HEATED_START);
    assert(heated_calls == (CONFIG_SENSOR_TCAL_HEATED ? 1U : 0U));
    /* Busy/OTA refusal must not become a delayed start after conditions clear,
     * even if the receiver repeats its PONG before observing the echo. */
    heated_result = 0;
    receive_control(ESB_PONG_FLAG_TCAL_HEATED_START);
    now_ms += 100;
    feeds = 0;
    if (setjmp(thread_stop) == 0) esb_thread();
    assert(heated_calls == (CONFIG_SENSOR_TCAL_HEATED ? 1U : 0U));
    assert(acked_remote_command == ESB_PONG_FLAG_TCAL_HEATED_START);
}
static void request_wom(uint16_t minutes, uint16_t token)
{
    rx_payload.data[8] = minutes >> 8;
    rx_payload.data[9] = minutes;
    rx_payload.data[10] = token >> 8;
    rx_payload.data[11] = token;
    in_wom_rx = true;
    receive_control(ESB_PONG_FLAG_WOM_CONTROL);
    in_wom_rx = false;
}

static void run_wom_owner(void)
{
    now_ms += 100;
    feeds = 0;
    stop_after = 3;
    if (setjmp(thread_stop) == 0) esb_thread();
}

static void expect_wom(uint8_t flag, uint16_t result, uint16_t token)
{
    uint8_t data[4];
    assert(esb_get_ping_ack_flag() == flag);
    esb_get_ping_request_data(data);
    assert(sys_get_be16(data) == result && sys_get_be16(data + 2) == token);
}

static void test_wom_transactions(void)
{
    channel_confirmed = true;
    channel_legacy_peer = channel_wait_normal = channel_search = false;
    receive_control(ESB_PONG_FLAG_NORMAL);
    esb_wom_reset();
    request_wom(20, 1);
    assert(esb_get_ping_ack_flag() == ESB_PONG_FLAG_NORMAL && !wom_writes);
    run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_RESULT, 20, 1);
    assert(wom_writes == 1);
    request_wom(20, 1); run_wom_owner();
    assert(wom_writes == 1);
    /* An unrelated NORMAL before admission cannot consume the outcome. */
    receive_control(ESB_PONG_FLAG_NORMAL);
    expect_wom(ESB_PING_FLAG_WOM_RESULT, 20, 1);
    wom_command.sent_flag = ESB_PING_FLAG_WOM_RESULT;
    wom_command.sent_token = 1;
    channel_wait_normal = true;
    receive_control(ESB_PONG_FLAG_NORMAL);
    expect_wom(ESB_PING_FLAG_WOM_RESULT, 20, 1);
    channel_wait_normal = false;
    receive_control(ESB_PONG_FLAG_NORMAL);
    assert(!wom_command.response_pending && !wom_command.cached);
    request_wom(20, 2); run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_RESULT, 20, 2);
    assert(wom_writes == 2); /* New token, same value is fresh. */

    request_wom(0, 3); run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_RESULT, 20, 3);
    unsigned queries = wom_queries;
    wom_minutes = 25; /* Local mutation invalidates a stale successful answer. */
    request_wom(0, 3); run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_RESULT, 25, 3);
    assert(wom_queries == queries + 2);
    request_wom(0, 4); run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_RESULT, 25, 4);
    assert(wom_queries == queries + 3);
    wom_set_error = -EIO;
    request_wom(40, 5); run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_ERROR, EIO, 5);
    assert(wom_minutes == 25 && wom_writes == 3);
    wom_set_error = 0;
    request_wom(40, 5); run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_ERROR, EIO, 5);
    assert(wom_writes == 3); /* Failure is terminal for this token too. */
    request_wom(35792, 6); run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_ERROR, EINVAL, 6);
    request_wom(15, 0); run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_ERROR, EINVAL, 0);
    assert(wom_writes == 3);
    wom_minutes = 0;
    request_wom(0, 7); run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_ERROR, ENOTSUP, 7);

    request_wom(35791, 8);
    channel_confirmed = false;
    run_wom_owner();
    assert(wom_writes == 3 && esb_get_ping_ack_flag() == ESB_PING_FLAG_CHANNEL_CONFIRM);
    channel_confirmed = true;
    channel_redirect_pending = true;
    run_wom_owner();
    assert(wom_writes == 3);
    channel_redirect_pending = false;
    run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_RESULT, 35791, 8);
    receive_control(ESB_PONG_FLAG_DATA_COLLECT_METADATA);
    assert(esb_get_ping_ack_flag() == ESB_PONG_FLAG_DATA_COLLECT_METADATA);
    metadata_echo_pending = false;
    expect_wom(ESB_PING_FLAG_WOM_RESULT, 35791, 8);
    receive_control(ESB_PONG_FLAG_OTA_QUERY_INFO);
    run_wom_owner();
    assert(esb_get_ping_ack_flag() == ESB_PONG_FLAG_OTA_QUERY_INFO);
    receive_control(ESB_PONG_FLAG_NORMAL);

    /* RX during blocking persistence must not publish the old token as new. */
    request_wom(22, 100);
    inject_wom_request = true;
    esb_wom_process();
    assert(wom_minutes == 22 && !wom_command.response_pending);
    assert(wom_command.requested && wom_command.token == 101);
    esb_wom_process();
    expect_wom(ESB_PING_FLAG_WOM_RESULT, 30, 101);
    /* Flag/payload split across a newer request is repaired before CRC. */
    uint8_t ping[13] = {[7] = ESB_PING_FLAG_WOM_RESULT};
    request_wom(31, 102);
    esb_wom_prepare_ping(ping);
    assert(ping[7] == ESB_PONG_FLAG_NORMAL && sys_get_be32(&ping[8]) == 0);
    esb_wom_process();
    ping[7] = ESB_PING_FLAG_WOM_ERROR;
    esb_wom_prepare_ping(ping);
    assert(ping[7] == ESB_PING_FLAG_WOM_RESULT);
    assert(sys_get_be16(&ping[8]) == 31 && sys_get_be16(&ping[10]) == 102);
    request_wom(32, 103);
    reset_during_wom = true;
    esb_wom_process();
    assert(!wom_command.cached && !wom_command.requested && !wom_command.response_pending);
    /* Completed token1 may be reused after a fast receiver reboot: NORMAL
     * consumption must already have retired the old transaction cache. */
    wom_minutes = 10;
    request_wom(0, 1); run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_RESULT, 10, 1);
    wom_minutes = 20;
    ping[7] = ESB_PING_FLAG_WOM_RESULT;
    esb_wom_prepare_ping(ping);
    assert(ping[7] == ESB_PING_FLAG_WOM_RESULT);
    assert(sys_get_be16(&ping[8]) == 10); /* Completed snapshot, drained first. */
    assert(!wom_command.requested);
    wom_command.sent_flag = ESB_PING_FLAG_WOM_RESULT;
    wom_command.sent_token = 1;
    receive_control(ESB_PONG_FLAG_NORMAL);
    assert(!wom_command.cached);
    request_wom(0, 1); run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_RESULT, 20, 1);
    /* A detected restart retires even an unconsumed/error response. */
    assert(host_receiver_restart() == 1);
    assert(!wom_command.cached && !wom_command.response_pending);
    wom_minutes = 30;
    request_wom(0, 1); run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_RESULT, 30, 1);
    /* Pending same-token sets cannot report an old success after local change. */
    request_wom(20, 2); run_wom_owner();
    unsigned writes = wom_writes;
    wom_minutes = 40;
    request_wom(20, 2); run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_RESULT, 20, 2);
    assert(wom_minutes == 20 && wom_writes == writes + 1);
    /* CLEAR is an action, not a zero-minute setting or cached readback. */
    writes = wom_writes;
    queries = wom_queries;
    request_wom(ESB_WOM_TIMEOUT_CLEAR, 200);
    assert(wom_writes == writes && !wom_command.response_pending);
    run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_RESULT, 0, 200);
    assert(wom_last_set_minutes == 0 && wom_minutes == wom_default_minutes);
    assert(wom_writes == writes + 1 && wom_queries == queries);
    request_wom(ESB_WOM_TIMEOUT_CLEAR, 200); run_wom_owner();
    assert(wom_writes == writes + 1 && wom_queries == queries);
    wom_minutes = 27; /* Local changes must not cause the same clear to rerun. */
    request_wom(ESB_WOM_TIMEOUT_CLEAR, 200); run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_RESULT, 0, 200);
    assert(wom_minutes == 27 && wom_writes == writes + 1);
    request_wom(ESB_WOM_TIMEOUT_CLEAR, 201); run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_RESULT, 0, 201);
    assert(wom_minutes == wom_default_minutes && wom_writes == writes + 2);
    /* The old zero operation still queries without calling persistence. */
    request_wom(0, 202); run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_RESULT, wom_default_minutes, 202);
    assert(wom_writes == writes + 2);
    wom_minutes = 28;
    wom_set_error = -EIO;
    request_wom(ESB_WOM_TIMEOUT_CLEAR, 203); run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_ERROR, EIO, 203);
    assert(wom_minutes == 28 && wom_writes == writes + 3);
    wom_set_error = 0;
    request_wom(ESB_WOM_TIMEOUT_CLEAR, 203); run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_ERROR, EIO, 203);
    assert(wom_writes == writes + 3);
    /* An exact compiled default need not have an integer-minute readback. */
    wom_default_minutes = 0;
    request_wom(ESB_WOM_TIMEOUT_CLEAR, 204); run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_RESULT, 0, 204);
    assert(wom_minutes == 0 && wom_writes == writes + 4);
    request_wom(0, 205); run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_ERROR, ENOTSUP, 205);
    assert(wom_writes == writes + 4);
    request_wom(ESB_WOM_TIMEOUT_CLEAR, 0); run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_ERROR, EINVAL, 0);
    assert(wom_writes == writes + 4);
    /* Consumed CLEAR may be performed anew even if a fresh epoch reuses it. */
    request_wom(ESB_WOM_TIMEOUT_CLEAR, 206); run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_RESULT, 0, 206);
    wom_command.sent_flag = ESB_PING_FLAG_WOM_RESULT;
    wom_command.sent_token = 206;
    receive_control(ESB_PONG_FLAG_NORMAL);
    assert(!wom_command.cached && !wom_command.response_pending);
    wom_minutes = 29;
    request_wom(ESB_WOM_TIMEOUT_CLEAR, 206); run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_RESULT, 0, 206);
    assert(wom_minutes == 0 && wom_writes == writes + 6);
    puts("commands: WOM tokens, cached failures, query, proof holds and persistence snapshots");
}

static void test_wom_enter(void)
{
    receive_control(ESB_PONG_FLAG_NORMAL);
    esb_wom_reset();
    unsigned writes = wom_writes, queries = wom_queries;
    wom_minutes = 0; /* Unsupported minute readback does not veto entry. */
    request_wom(ESB_WOM_ENTER, 300);
    assert(wom_enter_calls == 0 && !wom_command.response_pending);
    assert(esb_get_ping_ack_flag() == ESB_PONG_FLAG_NORMAL);
    request_wom(ESB_WOM_ENTER, 300); /* Duplicate in RX is still deferred. */
    channel_confirmed = false;
    run_wom_owner();
    assert(wom_enter_calls == 0);
    channel_confirmed = true;
    channel_redirect_pending = true;
    run_wom_owner();
    assert(wom_enter_calls == 0 && !wom_command.response_pending);
    channel_redirect_pending = false;
    run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_RESULT, ESB_WOM_ENTER, 300);
    assert(wom_enter_calls == 1 && wom_minutes == 0);
    request_wom(ESB_WOM_ENTER, 300); run_wom_owner();
    wom_minutes = 37;
    request_wom(ESB_WOM_ENTER, 300); run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_RESULT, ESB_WOM_ENTER, 300);
    assert(wom_enter_calls == 1 && wom_writes == writes && wom_queries == queries);
    receive_control(ESB_PONG_FLAG_NORMAL); /* No sent witness: cache survives. */
    request_wom(ESB_WOM_ENTER, 300); run_wom_owner();
    assert(wom_enter_calls == 1);
    wom_command.sent_flag = ESB_PING_FLAG_WOM_RESULT;
    wom_command.sent_token = 300;
    receive_control(ESB_PONG_FLAG_NORMAL);
    assert(!wom_command.cached);
    request_wom(ESB_WOM_ENTER, 300); run_wom_owner();
    assert(wom_enter_calls == 2); /* New epoch permits fresh core admission. */

    const int errors[] = {-EBUSY, -ENOTSUP, -EIO};
    for (unsigned i = 0; i < sizeof(errors) / sizeof(errors[0]); ++i) {
        uint16_t token = 301 + i;
        unsigned calls = wom_enter_calls;
        wom_enter_error = errors[i];
        request_wom(ESB_WOM_ENTER, token);
        assert(wom_enter_calls == calls && !wom_command.response_pending);
        assert(esb_get_ping_ack_flag() == ESB_PONG_FLAG_NORMAL);
        run_wom_owner();
        expect_wom(ESB_PING_FLAG_WOM_ERROR, -errors[i], token);
        assert(wom_enter_calls == calls + 1);
        wom_enter_error = 0;
        request_wom(ESB_WOM_ENTER, token); run_wom_owner();
        expect_wom(ESB_PING_FLAG_WOM_ERROR, -errors[i], token);
        assert(wom_enter_calls == calls + 1); /* No late retry after veto clears. */
    }
    unsigned calls = wom_enter_calls;
    request_wom(ESB_WOM_ENTER, 0); run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_ERROR, EINVAL, 0);
    assert(wom_enter_calls == calls && wom_writes == writes && wom_queries == queries);
    request_wom(ESB_WOM_ENTER, 304); run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_RESULT, ESB_WOM_ENTER, 304);
    assert(wom_enter_calls == calls + 1 && wom_minutes == 37);
    request_wom(0, 305); run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_RESULT, 37, 305);
    request_wom(ESB_WOM_TIMEOUT_CLEAR, 306); run_wom_owner();
    expect_wom(ESB_PING_FLAG_WOM_RESULT, 0, 306);
    assert(wom_writes == writes + 1 && wom_enter_calls == calls + 1);
    puts("commands: deferred WOM entry admission, terminal errors and action-only duplicate cache");
}

int main(void)
{
    run_shutdown(0);
    run_shutdown(3);
    /* Terminal rate refusal is consumed, but must not prevent an explicit OFF. */
    receive_control(ESB_PONG_FLAG_NORMAL);
    rx_payload.data[8] = 100;
    receive_control(ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON);
    now_ms += 100;
    feeds = 0; stop_after = 3;
    batch_result = -EBUSY;
    if (setjmp(thread_stop) == 0) esb_thread();
    assert(feeds == 3 && test_mode_changes == 0);
    receive_control(ESB_PONG_FLAG_DATA_COLLECT_BATCH_OFF);
    now_ms += 100; feeds = 0;
    if (setjmp(thread_stop) == 0) esb_thread();
    assert(!batch_active && test_mode_changes == 1);

    /* A newer PONG may replace a refused shutdown during its next LED delay. */
    receive_control(ESB_PONG_FLAG_NORMAL);
    receive_control(ESB_PONG_FLAG_SHUTDOWN);
    now_ms += 100; feeds = shutdown_calls = 0; reject_count = 1;
    inject_stop = true; batch_active = true;
    if (setjmp(thread_stop) == 0) esb_thread();
    assert(replaced_shutdown && !batch_active);
    assert(acked_remote_command == ESB_PONG_FLAG_DATA_COLLECT_BATCH_OFF);
    assert(!inject_stop);
    puts("commands: rejected shutdown retries, accepted shutdown keeps feeding; batch rejection propagates");
    run_heated_request(0);
    run_heated_request(-EBUSY);
    run_heated_request(-ENOTSUP);
    puts("commands: heated start consumed once, including refusal and disabled feature");
    receive_control(ESB_PONG_FLAG_NORMAL);
    memset(&rx_payload.data[8], 0, 4);
    rx_payload.data[11] = 51;
    receive_control(ESB_PONG_FLAG_SET_CHANNEL);
    uint32_t generation = remote_command_generation;
    assert(received_channel_value == 51);
    now_ms += 100; feeds = 0; stop_after = 3;
    if (setjmp(thread_stop) == 0) esb_thread();
    assert(channel_calls == 1 && applied_channel == 51);
    assert(acked_remote_command == ESB_PONG_FLAG_SET_CHANNEL);
    receive_control(ESB_PONG_FLAG_SET_CHANNEL);
    assert(remote_command_generation == generation);
    rx_payload.data[11] = 100;
    receive_control(ESB_PONG_FLAG_SET_CHANNEL);
    assert(received_channel_value == 100 && remote_command_generation == generation + 1);
    now_ms += 100; feeds = 0;
    if (setjmp(thread_stop) == 0) esb_thread();
    assert(channel_calls == 2 && applied_channel == 100);
    rx_payload.data[11] = 0;
    receive_control(ESB_PONG_FLAG_SET_CHANNEL);
    assert(received_channel_value == 0 && remote_command_generation == generation + 2);
    now_ms += 100; feeds = 0;
    if (setjmp(thread_stop) == 0) esb_thread();
    assert(channel_calls == 3 && applied_channel == 0);
    /* Executed commands and metadata survive the confirmation NORMAL while
     * scheduling remains gated. Dedicated replies never enter this tail. */
    for (volatile unsigned mode = 0; mode < 2; ++mode) {
        channel_wait_normal = mode == 0;
        channel_search = mode == 1;
        metadata_echo_pending = true;
        receive_control(ESB_PONG_FLAG_NORMAL);
        assert(acked_remote_command == ESB_PONG_FLAG_SET_CHANNEL);
        assert(received_remote_command == ESB_PONG_FLAG_SET_CHANNEL);
        assert(metadata_echo_pending);
        channel_wait_normal = channel_search = false;
        receive_control(ESB_PONG_FLAG_SET_CHANNEL);
        now_ms += 100; feeds = 0;
        if (setjmp(thread_stop) == 0) esb_thread();
        assert(channel_calls == 3); /* Receiver retry is not a new command. */
    }
    receive_control(ESB_PONG_FLAG_NORMAL); /* Ordinary acknowledged NORMAL. */
    assert(acked_remote_command == ESB_PONG_FLAG_NORMAL);
    assert(received_remote_command == ESB_PONG_FLAG_NORMAL && !metadata_echo_pending);
    /* A command not executed before proof remains queued, not discarded. */
    rx_payload.data[11] = 51;
    receive_control(ESB_PONG_FLAG_SET_CHANNEL);
    channel_wait_normal = true;
    channel_confirmed = false;
    now_ms += 100; feeds = 0;
    if (setjmp(thread_stop) == 0) esb_thread();
    assert(channel_calls == 3 && acked_remote_command == ESB_PONG_FLAG_NORMAL);
    receive_control(ESB_PONG_FLAG_NORMAL);
    assert(received_remote_command == ESB_PONG_FLAG_SET_CHANNEL);
    channel_wait_normal = false;
    channel_confirmed = true;
    receive_control(ESB_PONG_FLAG_SET_CHANNEL);
    now_ms += 100; feeds = 0;
    if (setjmp(thread_stop) == 0) esb_thread();
    assert(channel_calls == 4 && applied_channel == 51);
    assert(acked_remote_command == ESB_PONG_FLAG_SET_CHANNEL);
    for (volatile unsigned legacy = 0; legacy < 2; ++legacy) {
        channel_wait_normal = false;
        receive_control(ESB_PONG_FLAG_NORMAL);
        channel_wait_normal = true;
        channel_confirmed = !legacy; channel_legacy_peer = legacy;
        receive_control(ESB_PONG_FLAG_OTA_QUERY_INFO);
        now_ms += 100; feeds = 0;
        if (setjmp(thread_stop) == 0) esb_thread();
        assert(fw_info_requests == legacy + 1 && channel_wait_normal);
        assert(acked_remote_command == ESB_PONG_FLAG_OTA_QUERY_INFO);
        receive_control(ESB_PONG_FLAG_OTA_QUERY_INFO);
        now_ms += 100; feeds = 0;
        if (setjmp(thread_stop) == 0) esb_thread();
        assert(fw_info_requests == legacy + 1);
    }
    channel_wait_normal = false;
    receive_control(ESB_PONG_FLAG_NORMAL);
    receive_control(ESB_PONG_FLAG_OTA_ABORT);
    now_ms += 100; feeds = 0;
    if (setjmp(thread_stop) == 0) esb_thread();
    assert(abort_requests == 1);
    test_wom_transactions();
    test_wom_enter();
    return 0;
}
