#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* Hardware leaves only; counters, age gating, warnings, and transitions come
 * from the actual production TX_SUCCESS/TX_FAILED switch cases. */
enum { ESB_ST_PAIRING, ESB_ST_PAIRED, ESB_ST_RECOVERING };
enum { ESB_EVENT_TX_SUCCESS, ESB_EVENT_TX_FAILED };
struct esb_evt { int evt_id; unsigned tx_attempts; };
static int esb_conn_state;
static uint32_t ping_failures, ping_success_streak;
static int consecutive_enomem_errors;
static bool ping_pending, ping_failed, collecting, idle;
static int64_t now_ms, ping_send_time, connection_error_start_time;
static struct {
    uint8_t type, length;
    bool noack;
    int64_t timestamp;
} last_tx;
static unsigned drops, starts, clock_stops;
static unsigned milestone_logs, threshold_logs, pairing_logs, ordinary_logs;
static unsigned logged_success, logged_failed, logged_rate;

static int64_t k_uptime_get(void) { return now_ms; }
static uint32_t k_uptime_get_32(void) { return (uint32_t)now_ms; }
static int get_ping_interval_ms(void) { return 1000; }
static bool connection_get_data_collection(void) { return collecting; }
static bool esb_is_idle(void) { return idle; }
static void drop_failed_tx_payload(void) { ++drops; }
static void esb_start_queued_tx(void) { ++starts; }
static void clocks_stop(void) { ++clock_stops; }
static void debug_log(const char *format, ...) { (void)format; }

/* Classify meaning, not complete message wording or formatting. The focused
 * production cases have two warning classes and one three-counter stats log. */
static void warning_log(const char *format, ...)
{
    if (strstr(format, "threshold")) ++threshold_logs;
    else ++milestone_logs;
}
static void stats_log(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    logged_success = va_arg(args, unsigned);
    logged_failed = va_arg(args, unsigned);
    logged_rate = va_arg(args, unsigned);
    va_end(args);
    if (strstr(format, "pairing")) {
        assert(strstr(format, "waiting") && strstr(format, "receiver"));
        assert(strstr(format, "normal"));
        ++pairing_logs;
    } else {
        ++ordinary_logs;
    }
}
#define LOG_DBG(...) debug_log(__VA_ARGS__)
#define LOG_WRN(...) warning_log(__VA_ARGS__)
#define LOG_INF(...) stats_log(__VA_ARGS__)
#include "tx_failures.inc"

static void reset(void)
{
    esb_conn_state = ESB_ST_PAIRED;
    ping_failures = ping_success_streak = consecutive_enomem_errors = 0;
    ping_pending = ping_failed = collecting = false;
    idle = true;
    now_ms = ping_send_time = connection_error_start_time = 0;
    memset(&last_tx, 0, sizeof(last_tx));
    last_tx.type = ESB_PING_TYPE;
    drops = starts = clock_stops = 0;
    milestone_logs = threshold_logs = pairing_logs = ordinary_logs = 0;
    logged_success = logged_failed = logged_rate = 0;
    tx_success_count = tx_failed_count = last_log_time = 0;
}
static void event(int id)
{
    struct esb_evt e = { .evt_id = id, .tx_attempts = 2 };
    host_tx_event(&e);
}
static void fail(void) { event(ESB_EVENT_TX_FAILED); }
static void overdue(uint32_t count)
{
    ping_failures = count;
    ping_pending = true;
    ping_success_streak = 2;
    ping_send_time = now_ms;
    now_ms += get_ping_interval_ms() - 99;
}
static void callback_accounting(void)
{
    for (uint32_t count = 10; count <= 20; count += 10) {
        reset();
        overdue(count - 1);
        fail();
        assert(ping_failures == count && ping_failed && !ping_pending);
        assert(ping_success_streak == 0 && milestone_logs == 0);
        for (unsigned i = 0; i < 30; ++i) {
            now_ms += 100;
            fail();
        }
        assert(ping_failures == count && milestone_logs == 0);
        assert(threshold_logs == 0 && connection_error_start_time == 0);
        assert(drops == 31 && starts == 31 && tx_failed_count == 31);
    }
}
static void age_boundary(void)
{
    reset();
    ping_failures = 10;
    ping_pending = true;
    ping_success_streak = 2;
    for (now_ms = 0; now_ms <= get_ping_interval_ms() - 100; ++now_ms) fail();
    assert(ping_failures == 10 && ping_pending && !ping_failed);
    assert(ping_success_streak == 2 && milestone_logs == 0);
    fail();
    assert(ping_failures == 11 && !ping_pending && ping_failed);
    assert(ping_success_streak == 0 && milestone_logs == 0);

    reset();
    ping_failures = 9;
    ping_pending = true;
    now_ms = get_ping_interval_ms() - 100;
    fail();
    assert(ping_failures == 9 && ping_pending && milestone_logs == 0);
    ++now_ms;
    fail();
    assert(ping_failures == 10 && !ping_pending && milestone_logs == 0);
}
static void threshold_origin(void)
{
    reset();
    overdue(TX_ERROR_THRESHOLD - 1);
    fail();
    int64_t origin = now_ms;
    unsigned milestones_at_threshold = milestone_logs;
    assert(ping_failures == TX_ERROR_THRESHOLD && threshold_logs == 1);
    assert(esb_conn_state == ESB_ST_RECOVERING);
    assert(connection_error_start_time == origin);
    for (unsigned i = 0; i < 30; ++i) {
        now_ms += 100;
        last_tx.type = i % 2 ? ESB_PING_TYPE : 0x03;
        fail();
    }
    assert(ping_failures == TX_ERROR_THRESHOLD && threshold_logs == 1);
    assert(milestone_logs == milestones_at_threshold);
    assert(connection_error_start_time == origin);
    overdue(TX_ERROR_THRESHOLD);
    fail();
    assert(ping_failures == TX_ERROR_THRESHOLD + 1);
    assert(threshold_logs == 1 && connection_error_start_time == origin);
}
static void non_ping_failure(void)
{
    reset();
    last_tx.type = 0x03;
    overdue(9);
    fail();
    assert(ping_failures == 10 && ping_failed && !ping_pending);
    assert(ping_success_streak == 0 && milestone_logs == 0);
    overdue(TX_ERROR_THRESHOLD - 1);
    fail();
    assert(ping_failures == TX_ERROR_THRESHOLD && threshold_logs == 1);
    assert(esb_conn_state == ESB_ST_RECOVERING);
    assert(connection_error_start_time == now_ms && milestone_logs == 0);
}
static void pairing_stats(void)
{
    reset();
    esb_conn_state = ESB_ST_PAIRING;
    last_tx.type = 0;
    for (unsigned i = 0; i < 99; ++i) fail();
    assert(pairing_logs == 0 && ordinary_logs == 0);
    fail();
    assert(pairing_logs == 1 && ordinary_logs == 0);
    assert(logged_success == 0 && logged_failed == 100 && logged_rate == 100);
    assert(tx_success_count == 0 && tx_failed_count == 100);
    assert(ping_failures == 0 && connection_error_start_time == 0);
    assert(esb_conn_state == ESB_ST_PAIRING && clock_stops == 0);
    assert(drops == 100 && starts == 100);
    now_ms = 5000;
    fail();
    assert(pairing_logs == 1);
    ++now_ms;
    fail();
    assert(pairing_logs == 2 && last_log_time == 5001);
    assert(logged_failed == 102 && logged_rate == 100);

    /* Lifetime hardware counters survive pairing/state transitions. */
    esb_conn_state = ESB_ST_PAIRED;
    consecutive_enomem_errors = 7;
    for (unsigned i = 0; i < 98; ++i) event(ESB_EVENT_TX_SUCCESS);
    assert(consecutive_enomem_errors == 0 && tx_success_count == 98);
    for (unsigned i = 0; i < 98; ++i) fail();
    assert(ordinary_logs == 1 && pairing_logs == 2);
    assert(logged_success == 98 && logged_failed == 200);
    assert(logged_rate == 200 * 100 / 298);
    assert(ping_failures == 0 && connection_error_start_time == 0);
    esb_conn_state = ESB_ST_RECOVERING;
    now_ms += 5001;
    fail();
    assert(ordinary_logs == 2 && pairing_logs == 2);
    assert(logged_success == 98 && logged_failed == 201);
    assert(logged_rate == 201 * 100 / 299);
}
int main(void)
{
    callback_accounting();
    age_boundary();
    threshold_origin();
    non_ping_failure();
    pairing_stats();
    puts("tx_failures: logical loss accounting, age gate, threshold origin, pairing statistics");
    return 0;
}
