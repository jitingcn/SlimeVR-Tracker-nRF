/* Full production connection_thread plus real collection setters; hardware,
 * packet encoding and scheduler leaves are replaced with deterministic doubles. */
#define main collection_existing_main
#include "test_collection.c"
#undef main
#include <setjmp.h>
#define LOG_ERR(...) ((void)0)

#define WDT_CHANNEL_CONNECTION 0
#define PING_INTERVAL_MS 1000
#define PING_QUEUE_RETRY_MS 5
#define OTA_SUPPRESS_TIMEOUT_MS 60000
#define ESB_PING_LEN 12
#define ESB_PING_TYPE 0
#define ESB_PONG_FLAG_TEST_MODE_ON 1
#define ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON 2
#define ESB_PONG_FLAG_DATA_COLLECT_METADATA 3
#define SYS_STATUS_CONNECTION_ERROR 2
#define COMPOSITE_LOOKAHEAD_MS 10
#define SUB_PACKET_QUAT_ACCEL 1
#define SUB_PACKET_QUAT_MAG 2
#define SUB_PACKET_COMPACT_QUAT 3
#define SUB_PACKET_STATUS 4
#define SUB_PACKET_RUNTIME 5
#define SUB_PACKET_INFO 6
static jmp_buf iteration_done;
static unsigned iterations, test_stops, search_calls, ping_calls, ota_calls;
static bool disconnected, searching, radio_available, ota_active, test_enabled;
static bool force_ping;
static int ping_result;
static int64_t dc_conn_error_start, ota_suppress_start_time;
static bool ota_suppressed;
static atomic_t next_ping_deadline_ms, ping_resync_requested, ping_server_phase_aligned;
static uint32_t ping_interval_ms, ping_aligned_interval_ms;
static struct { atomic_t due, attempts, queue_ok, queue_fail, retry_deferred; } ping_sched_stats;
static int sensor_data_snapshot;
static bool sensor_ids_set;
static int64_t last_mag_time, last_info_time, last_status_time, last_runtime_time;
static struct { unsigned tps; } test_rate_schedule;
struct composite_builder { unsigned unused; };
static bool atomic_cas(atomic_t *p, int old, int value) { if (*p != old) return false; *p = value; return true; }
static uint32_t k_uptime_get_32(void) { return now_ms; }
static uint64_t k_uptime_ticks(void) { return (uint64_t)now_ms * 1000; }
static uint64_t k_ticks_to_us_near64(uint64_t ticks) { return ticks; }
static void k_msleep(unsigned ms) {}
static void k_usleep(unsigned us) {}
static int watchdog_register_thread(int channel, int flags) { return 0; }
#define SYS_REBOOT_COLD 0
static void sys_reboot(int reason) { assert(!"unexpected watchdog reboot"); }
static void watchdog_feed(int channel) { if (iterations++) longjmp(iteration_done, 1); }
static uint32_t ping_phase_ms(uint32_t interval) { return 0; }
static bool esb_ready(void) { return radio_available; }
static bool connection_hid_output_ready(void) { return false; }
static int get_status(int mask) { return disconnected ? mask : 0; }
static void esb_process_ota_rx_queue(void) {}
static bool esb_channel_search_poll(bool suppressed) { search_calls++; return searching; }
uint32_t get_ping_interval_ms(void) { return ping_interval_ms; }
static bool tdma_ping_wake_delay_ms(uint32_t *delay) { *delay = force_ping ? 0 : 10; return true; }
static uint32_t ping_server_phase_delay_ms(uint32_t interval) { return 0; }
uint8_t connection_get_id(void) { return tracker_id; }
static uint8_t esb_get_ping_ack_flag(void) { return 0; }
static uint16_t test_mode_get_target_tps(void) { return 30; }
static void esb_get_ping_request_data(uint8_t *data) {}
static int esb_write_ping(uint8_t *data, bool force) { ping_calls++; return ping_result; }
static void ping_stats_attempt(uint32_t now) {}
static uint32_t ping_next_periodic_deadline(uint32_t deadline, uint32_t now, uint32_t interval) { return now + interval; }
static bool esb_ota_is_active(void) { return ota_active; }
static void esb_ota_check_timeout(void) { ota_calls++; }
static void esb_ota_periodic_status(void) {}
void connection_set_ota_suppressed(bool enabled) { ota_suppressed = enabled; }
static void test_mode_set(bool enabled) { test_enabled = enabled; if (!enabled) test_stops++; }
static bool test_mode_get(void) { return test_enabled; }
static bool connection_send_tracker_event(void) { return false; }
bool connection_process_raw_data(void) { return false; }
static bool sensor_data_snapshot_qa_pending(int *snapshot) { return false; }
static bool sensor_data_snapshot_m_pending(int *snapshot) { return false; }
static bool test_rate_due(uint64_t now) { return false; }
static void test_rate_advance(uint64_t now) {}
static void composite_builder_reset(struct composite_builder *builder) {}
static void composite_try_add_due(struct composite_builder *builder, int type, bool due, int64_t *last, int64_t now) {}
static bool connection_sensor_get_precise_quat(void) { return false; }
static void composite_try_add(struct composite_builder *builder, int type) {}
static bool send_composite_or_single(struct composite_builder *builder, int type) { return true; }
static void composite_commit_timestamps(struct composite_builder *builder) {}
static unsigned connection_send_retry_ms(void) { return 1; }
static void connection_idle_wait(int64_t now) {}
#include "lifecycle.inc"

static void tick(int64_t now)
{
    now_ms = now;
    iterations = 0;
    if (setjmp(iteration_done) == 0) connection_thread();
}
static void reset(bool batch)
{
    connection_set_data_collection(false);
    assert(batch_request(false, 0) == 0);
    now_ms = 100;
    if (batch) assert(batch_request(true, 5) == 0);
    else connection_set_data_collection(true);
    dc_conn_error_start = 0;
    disconnected = searching = radio_available = test_enabled = true;
    ota_active = ota_suppressed = force_ping = false;
    ping_result = 0;
    test_stops = search_calls = ping_calls = ota_calls = 0;
}
static void active(bool batch)
{
    assert(connection_raw_collection_active());
    assert(connection_get_data_collection_batch() == batch);
    assert(test_enabled && test_stops == 0);
}
static void stopped(void)
{
    assert(!connection_get_data_collection());
    assert(!connection_get_data_collection_batch());
    assert(!test_enabled && test_stops == 1);
}
int main(void)
{
    for (unsigned batch = 0; batch < 2; batch++) {
        reset(batch);
        tick(100);
        tick(60099); active(batch);
        tick(60100); active(batch); /* Strictly greater than 60 seconds. */
        tick(60101); stopped();
        assert(search_calls == 4);
        tick(120102); stopped(); /* Teardown is not repeated. */

        reset(batch);
        tick(100);
        tick(60099); active(batch);
        disconnected = false; /* Recovery while search still owns the loop. */
        tick(60100); active(batch);
        disconnected = true;
        tick(60101); active(batch);
        tick(120101); active(batch);
        tick(120102); stopped();

        /* The same deadline applies to other early-continue paths. */
        for (unsigned path = 0; path < 5; path++) {
            reset(batch);
            searching = false;
            radio_available = path != 0;
            force_ping = path == 1 || path == 2;
            ping_result = path == 2 ? -EAGAIN : 0;
            ota_active = path == 3;
            ota_suppressed = path == 4;
            tick(100);
            tick(60100); active(batch);
            tick(60101); stopped();
            if (force_ping) assert(ping_calls == 3);
            if (ota_active) assert(ota_calls == 3);
        }
    }
    /* An unrelated explicit test session must survive disconnection when
     * there is no collection session to clean up. */
    reset(false);
    connection_set_data_collection(false);
    tick(100);
    tick(120101);
    assert(test_enabled && test_stops == 0);
    assert(!connection_raw_collection_active());
    puts("lifecycle: full connection loop preserves strict collection timeout, recovery reset, both modes, search/PING/OTA/unready paths and unrelated test sessions");
    return 0;
}
