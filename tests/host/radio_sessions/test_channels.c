#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdarg.h>
#include <stdlib.h>
#include "system/status.h"
#include "../led_feedback_stub.h"
#define __maybe_unused
#define K_MUTEX_DEFINE(name) static int name
#define K_FOREVER 0
#define ESB_ST_PAIRING 0
static void warning_log(const char *format, ...);
#define LOG_WRN(...) warning_log(__VA_ARGS__)
#define LOG_INF(...) debug_log(__VA_ARGS__)
static void debug_log(const char *format, ...) { (void)format; }
#define LOG_DBG(...) debug_log(__VA_ARGS__)
#define LOG_ERR(...) ((void)0)
#define USER_SHUTDOWN_ENABLED 1
#define CONFIG_CONNECTION_TIMEOUT_DELAY 120000
#define WDT_CHANNEL_ESB 0
#define ESB_ST_RECOVERING 2
#define TX_ERROR_THRESHOLD 300
#define ESB_ST_PAIRED 1
#define PAIRED_ID 32
static struct { uint64_t DEVICEADDR[1]; } ficr = {{0x123456789abc}};
#define NRF_FICR (&ficr)
static uint8_t paired_addr[8], radio_channel, paired_channel_found;
static uint32_t radio_session_generation;
static bool pair_ack_pending, clock_status, ping_failed;
static bool own_pong_seen, pairing_search_active, radio_user_disabled;
static uint32_t pairing_request;
static int persistence_error;
static struct esb_payload { uint8_t data[8]; bool noack; } tx_payload_pair;
static uint8_t pair_target, pair_step;
static void receive_pair(void);
static bool inject_late_pong;
static void late_pong(void);
static unsigned irq_lock(void) { return 0; }
static void irq_unlock(unsigned key) {
    (void)key;
    if (inject_late_pong) { inject_late_pong = false; late_pong(); }
}
static uint32_t get_ping_interval_ms(void) { return 1497; }
static int64_t registered_at = -1;
static void k_mutex_lock(int *lock, int timeout) { (void)timeout; ++*lock; }
static void k_mutex_unlock(int *lock) { assert(*lock > 0); --*lock; }
static bool esb_initialized = true, ota_active, idle = true, server_time_synced;
static int esb_conn_state = 1, status_state;
static uint32_t ping_failures, ping_success_streak;
static unsigned ota_rx_head, ota_rx_tail;
static bool ping_pending, shutdown_requested;
static int64_t connection_error_start_time, now;
static uint8_t tracker_id = 3, ping_counter, epoch;
static uint32_t ping_ctr_sent;
static int64_t ping_send_time;
#define PING_HISTORY_SIZE 8
static struct { uint8_t counter; uint32_t ping_ticks, ping_ticks_kernel; } ping_history[PING_HISTORY_SIZE];
static unsigned ping_history_idx;
static uint64_t k_uptime_ticks(void) { return (uint64_t)now; }
static uint64_t net_ticks_from_kernel64(uint64_t ticks) { return ticks; }
static void record_ping_admission(uint8_t counter);
static int64_t warning_times[256];
static uint32_t warning_failures[256];
static unsigned warning_count;
static bool trace_warnings;
static void warning_log(const char *format, ...) {
    if (!strstr(format, "total")) return;
    assert(warning_count < 256);
    va_list args; va_start(args, format);
    uint32_t failures = va_arg(args, unsigned);
    va_end(args);
    warning_times[warning_count] = now;
    warning_failures[warning_count++] = failures;
    if (trace_warnings) printf("ping-warning t=%lldms failures=%u\n", (long long)now, failures);
}
static unsigned probes, writes, changes, disables;
static struct { uint8_t rf_channel, paired_addr[8]; } storage, *retained = &storage;
static struct { uint8_t data[13], length; } rx_payload;
#define RF_CHANNEL_ID 31
static int64_t k_uptime_get(void) { return now; }
static uint32_t k_uptime_get_32(void) { return (uint32_t)now; }
static uint32_t k_cycle_get_32(void) { return (uint32_t)now; }
static bool esb_ota_is_active(void) { return ota_active; }
static bool esb_is_idle(void) { return idle; }
static void esb_clear_time_sync_state(void) { server_time_synced = false; }
static void tdma_set_enabled(bool enabled) { (void)enabled; }
static void drop_failed_tx_payload_if_pending(void) {}
static void esb_flush_tx(void) { assert(idle); }
static void esb_flush_rx(void) { assert(idle); }
static int esb_set_rf_channel(uint8_t ch) { assert(idle && ch <= 100); ++changes; return 0; }
static uint8_t esb_get_ping_ack_flag(void) { return 0; }
static int esb_write_ping(uint8_t *ping, bool force) {
    assert(force && ping[0] == 0xf0 && ping[1] == tracker_id);
    ++probes; record_ping_admission(ping_counter); return 0;
}
static void clocks_start(void) { clock_status = true; }
static void clocks_stop(void) { clock_status = false; }
static void esb_set_addr_discovery(void) {}
static void esb_set_addr_paired(void) {}
static void tracker_events_session_changed(void) {}
static void connection_set_id(uint8_t id) { tracker_id = id; }
static void set_tracker_id(uint8_t id) { tracker_id = id; }
static uint8_t crc8_ccitt(uint8_t seed, const uint8_t *data, size_t size) {
    for (size_t i = 0; i < size; ++i) {
        seed ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            seed = (seed & 0x80) ? (uint8_t)((seed << 1) ^ 0x07) : (uint8_t)(seed << 1);
    }
    return seed;
}
static void watchdog_feed(int channel) { (void)channel; assert(now < 100000); }
static void k_msleep(int delay) { now += delay; }
static int sys_request_system_off(void) { return 0; }
static int esb_initialize(bool tx) {
    (void)tx; esb_initialized = true;
    ++radio_session_generation;
    radio_channel = retained->rf_channel == 128 ? 0 : retained->rf_channel;
    return 0;
}
static int esb_write_payload(const struct esb_payload *payload) {
    pair_step = payload->data[1]; return 0;
}
static int esb_start_tx(void) {
    if (radio_channel != pair_target) return 0;
    if (pair_step == 0 && registered_at < 0) registered_at = now;
    if (pair_step == 1 && pair_ack_pending && now - registered_at >= 100) {
        rx_payload.length = 8;
        rx_payload.data[0] = tx_payload_pair.data[0];
        rx_payload.data[1] = 3;
        memset(&rx_payload.data[2], 0x77, 6);
        receive_pair();
    }
    return 0;
}
static int sys_write(unsigned id, void *dst, const void *src, size_t len) {
    assert(id == RF_CHANNEL_ID || id == PAIRED_ID);
    memcpy(dst, src, len); if (id == RF_CHANNEL_ID) ++writes; return persistence_error;
}
static void esb_disable(void) { ++disables; }
static uint8_t tdma_get_config_epoch(void) { return epoch; }
static void tdma_update_config(uint8_t slot, uint8_t total, uint8_t ticks, uint8_t new_epoch) {
    assert(slot < total && ticks >= 16); epoch = new_epoch;
}
void set_status(enum sys_status status, bool value) {
    if (value) status_state |= status; else status_state &= ~status;
}
int get_status(enum sys_status status) { return status_state & status; }
#include "channels.inc"
enum { ESB_EVENT_TX_SUCCESS, ESB_EVENT_TX_FAILED };
struct esb_evt { int evt_id; unsigned tx_attempts; };
static int consecutive_enomem_errors;
static struct { uint8_t type, length; bool noack; int64_t timestamp; } last_tx;
static bool connection_get_data_collection(void) { return false; }
static void drop_failed_tx_payload(void) {}
static void esb_start_queued_tx(void) {}
#include "tx_failures.inc"
static void failed_tx(void) {
    struct esb_evt event = {ESB_EVENT_TX_FAILED, 2};
    host_tx_event(&event);
}
static void late_pong(void) {
    rx_payload.length = 13; rx_payload.data[0] = ESB_PONG_TYPE;
    rx_payload.data[1] = tracker_id; rx_payload.data[2] = ping_ctr_sent;
    rx_payload.data[12] = crc8_ccitt(7, rx_payload.data, 12);
    assert(!accept_pong());
}

static void reset(uint8_t home) {
    ++radio_session_generation;
    esb_conn_state = ESB_ST_PAIRED; connection_error_start_time = 0;
    ping_pending = ping_failed = false; ping_send_time = 0;
    ping_success_streak = 0; ping_counter = 0; ping_ctr_sent = 0;
    warning_count = 0; last_tx.type = ESB_PING_TYPE;
    esb_initialized = true; idle = true; ota_active = false;
    ota_rx_head = ota_rx_tail = 0; ping_failures = 3;
    channel_search = channel_wait_normal = channel_found = channel_heard = false;
    radio_channel = home; now = 100; probes = changes = writes = 0;
    own_pong_time = (uint32_t)now;
    storage.rf_channel = esb_rf_channel_encode(home);
    memset(storage.paired_addr, 0x5a, sizeof(storage.paired_addr));
    status_state = SYS_STATUS_CONNECTION_ERROR | SYS_STATUS_USB_CONNECTED;
}

/* One owner tick uses production maintenance, search, admission and callback. */
static void owner_tick(void) {
    maintenance_timeout();
    unsigned before = probes;
    (void)esb_channel_search_poll(false);
    if (probes != before) {
        assert(ping_send_time == now && ping_pending);
        failed_tx();
        assert(ping_pending); /* Fresh probes are not aged losses. */
    }
}
static void initial_losses(void) {
    ping_failures = 0;
    for (unsigned i = 0; i < 3; ++i) {
        record_ping_admission(ping_counter);
        now += get_ping_interval_ms() - 99;
        if (i == 1) maintenance_timeout(); else failed_tx();
        assert(ping_failures == i + 1 && !ping_pending && ping_failed);
        if (i < 2) {
            (void)esb_channel_search_poll(false);
            assert(warning_count == 0);
        }
    }
}
static void cadence(void) {
    reset(2); initial_losses();
    int64_t start = now;
    owner_tick();
    assert(warning_count == 1 && warning_failures[0] == 3);
    unsigned fast_probes = 0;
    for (++now; now <= start + 120000; ++now) {
        unsigned before = probes;
        int64_t previous = ping_send_time;
        owner_tick();
        if (probes != before && now - previous >= 80 && now - previous <= 102)
            ++fast_probes;
    }
    assert(fast_probes > 800 && probes > 1000);
    assert(warning_count == 13);
    int64_t minimum = INT64_MAX, maximum = 0;
    for (unsigned i = 1; i < warning_count; ++i) {
        int64_t spacing = warning_times[i] - warning_times[i - 1];
        if (spacing < minimum) minimum = spacing;
        if (spacing > maximum) maximum = spacing;
        assert(warning_failures[i] > warning_failures[i - 1]);
    }
    assert(minimum >= 10000 && maximum <= 10001);
    assert(ping_failures == (uint32_t)((now - 1 - own_pong_time) / get_ping_interval_ms()));
    /* Accepted own PONG plus NORMAL completes search and resets throttling. */
    rx_payload.length = ESB_PONG_LEN; rx_payload.data[0] = ESB_PONG_TYPE;
    rx_payload.data[1] = tracker_id; rx_payload.data[2] = ping_ctr_sent;
    rx_payload.data[12] = crc8_ccitt(7, rx_payload.data, 12);
    assert(accept_pong() && ping_failures == 0 && !ping_pending);
    server_time_synced = true;
    rx_payload.data[8] = 0; rx_payload.data[9] = 1; rx_payload.data[10] = 16;
    receive_schedule(ESB_PONG_FLAG_NORMAL);
    assert(channel_found && !esb_channel_search_poll(false));
    unsigned recovered_count = warning_count;
    now += 100; owner_tick(); assert(warning_count == recovered_count);
    warning_count = 0;
    initial_losses(); owner_tick();
    assert(warning_count == 1 && warning_failures[0] == 3);
    /* Counter jumps and busy radio cannot starve the warning owner. */
    reset(2); idle = false;
    now += 123456; owner_tick();
    assert(warning_count == 1 && warning_failures[0] > 80 && probes == 0);
    uint32_t jumped = ping_failures;
    now += 10000; owner_tick();
    assert(warning_count == 2 && ping_failures > jumped && changes == 0);
}
static void warning_lifecycle(void) {
    reset(2); owner_tick(); assert(warning_count == 1);
    /* No inactive poll occurs between the radio generations. */
    esb_initialized = false; ++radio_session_generation;
    now += 1; esb_initialized = true; owner_tick();
    assert(warning_count == 2);
    for (unsigned mode = 0; mode < 2; ++mode) {
        if (mode == 0) esb_initialized = false; else esb_conn_state = ESB_ST_PAIRING;
        now += 1; (void)esb_channel_search_poll(false);
        assert(warning_count == 2 + mode);
        esb_initialized = true; esb_conn_state = ESB_ST_PAIRED;
        now += 1; owner_tick(); assert(warning_count == 3 + mode);
    }
    channel_search = false; ping_failures = 0; own_pong_time = now;
    now += 1; owner_tick(); assert(warning_count == 4);
    ping_failures = 3; now += 1; owner_tick(); assert(warning_count == 5);
    for (unsigned mode = 0; mode < 3; ++mode) {
        reset(2); owner_tick();
        int64_t start = now;
        /* Short alternating holds must not reset the deadline and burst. */
        for (now = start + 1; now <= start + 25000; ++now) {
            bool held = (now - start) % 200 < 100;
            ota_active = mode == 1 && held;
            ota_rx_head = mode == 2 && held;
            unsigned before = warning_count;
            (void)esb_channel_search_poll(mode == 0 && held);
            if (held) assert(warning_count == before);
        }
        assert(warning_count == 3);
        for (unsigned i = 1; i < warning_count; ++i) {
            int64_t spacing = warning_times[i] - warning_times[i - 1];
            assert(spacing >= 10000 && spacing <= 10100);
        }
        /* Long suppression: one release warning, never catch-up. */
        ota_active = mode == 1; ota_rx_head = mode == 2;
        now += 30000;
        unsigned before = warning_count, old_probes = probes;
        (void)esb_channel_search_poll(mode == 0);
        assert(warning_count == before && probes == old_probes);
        ota_active = false; ota_rx_head = 0;
        (void)esb_channel_search_poll(false);
        assert(warning_count == before + 1);
        ++now; (void)esb_channel_search_poll(false);
        assert(warning_count == before + 1);
    }
}
static void visit(uint8_t target) {
    assert(esb_channel_search_poll(false));
    for (unsigned i = 0; radio_channel != target && i < 101; ++i) {
        now = search_deadline; assert(esb_channel_search_poll(false));
    }
    assert(radio_channel == target && writes == 0);
}
int main(void) {
    trace_warnings = getenv("RADIO_PING_TRACE") != NULL;
    cadence();
    warning_lifecycle();
    puts("channels: 120s production-path ping cadence, recovery and suppression PASS");
    if (getenv("RADIO_PING_ONLY")) return 0;
    for (unsigned home = 0; home <= 100; ++home) {
        bool seen[101] = {0};
        assert(channel_candidate(home, 0) == home);
        for (unsigned i = 0; i <= 100; ++i) {
            uint8_t ch = channel_candidate(home, i);
            assert(ch <= 100 && !seen[ch]); seen[ch] = true;
        }
    }
    const uint8_t destinations[] = {0, 51, 100};
    for (unsigned i = 0; i < sizeof(destinations); ++i) {
        reset(2); visit(destinations[i]);
        rx_payload.length = 13; rx_payload.data[0] = ESB_PONG_TYPE;
        rx_payload.data[1] = tracker_id + 7;
        rx_payload.data[2] = ping_ctr_sent;
        rx_payload.data[12] = crc8_ccitt(7, rx_payload.data, 12);
        assert(!accept_pong() && channel_search && !channel_heard);
        rx_payload.data[1] = tracker_id;
        rx_payload.data[2] = ping_ctr_sent - 1;
        rx_payload.data[12] = crc8_ccitt(7, rx_payload.data, 12);
        assert(!accept_pong() && !channel_heard);
        rx_payload.data[2] = ping_ctr_sent; ping_pending = false;
        rx_payload.data[12] = crc8_ccitt(7, rx_payload.data, 12);
        assert(!accept_pong()); ping_pending = true;
        rx_payload.data[12] ^= 1; assert(!accept_pong());
        rx_payload.data[12] ^= 1;
        rx_payload.length = 12; assert(!accept_pong()); rx_payload.length = 13;
        assert(accept_pong() && channel_heard);
        server_time_synced = true;
        rx_payload.data[8] = 0xff; rx_payload.data[9] = 1; rx_payload.data[10] = 16;
        receive_schedule(ESB_PONG_FLAG_NORMAL); assert(!channel_found);
        rx_payload.data[8] = 0;
        receive_schedule(ESB_PONG_FLAG_SET_CHANNEL); assert(!channel_found);
        receive_schedule(ESB_PONG_FLAG_NORMAL); assert(channel_found);
        assert(!esb_channel_search_poll(false));
        assert(!channel_search && !channel_wait_normal && writes == 1);
        assert(storage.rf_channel == esb_rf_channel_encode(destinations[i]));
        for (unsigned j = 0; j < sizeof(storage.paired_addr); ++j) assert(storage.paired_addr[j] == 0x5a);
        assert(status_state == SYS_STATUS_USB_CONNECTED);
    }
    reset(2); ping_failures = 0; own_pong_time = (uint32_t)now;
    now += 4499; assert(!esb_channel_search_poll(false));
    now += 1; assert(esb_channel_search_poll(false) && channel_search);
    reset(2); visit(2);
    now = search_deadline; inject_late_pong = true;
    assert(esb_channel_search_poll(false));
    assert(!inject_late_pong && !channel_found && writes == 0 && radio_channel != 2);
    now = own_pong_time + 301 * get_ping_interval_ms();
    assert(esb_channel_search_poll(false) && ping_failures >= 300);
    reset(2); ota_active = true;
    assert(!esb_channel_search_poll(false) && !channel_search && !probes);
    ota_active = false; assert(!esb_channel_search_poll(true) && !channel_search);
    ota_rx_head = 1; assert(!esb_channel_search_poll(false) && !channel_search);
    ota_rx_head = 0; idle = false;
    assert(!esb_channel_search_poll(false) && !channel_search);
    idle = true; visit(51); unsigned old_changes = changes, old_probes = probes;
    ota_active = true; now += 1000;
    assert(!esb_channel_search_poll(false) && changes == old_changes && probes == old_probes);
    ota_active = false; esb_deinitialize();
    assert(!esb_initialized && !channel_search && channel_wait_normal && !channel_found);
    assert(storage.rf_channel == 2 && writes == 0 && disables == 1);
    for (unsigned i = 0; i < sizeof(destinations); ++i) {
        reset(2); memset(paired_addr, 0, sizeof(paired_addr));
        pair_target = destinations[i]; registered_at = -1;
        esb_pair();
        assert(led_test_events[LED_SUCCESS] == i + 1 && led_test_events[LED_PARTIAL] == 0);
        assert(storage.rf_channel == esb_rf_channel_encode(pair_target));
        assert(memcmp(storage.paired_addr, paired_addr, 8) == 0);
        assert(paired_addr[1] == 3 && esb_conn_state == ESB_ST_PAIRED);
        assert(writes == 1 && !esb_initialized);
    }
    reset(2); memset(paired_addr, 0, sizeof(paired_addr)); pair_target = 2;
    registered_at = -1; persistence_error = -EIO;
    unsigned successes = led_test_events[LED_SUCCESS];
    esb_pair();
    assert(led_test_events[LED_PARTIAL] == 1 && led_test_events[LED_SUCCESS] == successes);
    persistence_error = 0;
    struct led_connection_facts facts = {0};
    reset(2); memcpy(paired_addr, storage.paired_addr, sizeof(paired_addr));
    esb_conn_state = ESB_ST_PAIRED; ping_failures = 0; own_pong_seen = false;
    status_state = 0; /* Suppressed status alone is not a health witness. */
    esb_led_connection_facts(&facts); assert(!facts.healthy);
    own_pong_seen = true; own_pong_time = (uint32_t)now;
    esb_led_connection_facts(&facts); assert(facts.healthy);
    ota_active = true;
    esb_led_connection_facts(&facts); assert(!facts.healthy);
    ota_active = false;
    channel_wait_normal = true;
    esb_led_connection_facts(&facts); assert(!facts.healthy);
    channel_wait_normal = false; ping_failures = 3;
    esb_led_connection_facts(&facts); assert(!facts.healthy);
    ping_failures = 0; now += 4500;
    esb_led_connection_facts(&facts); assert(!facts.healthy);
    puts("channels: exhaustive coverage, strict own probes, NORMAL gate, persistence, OTA and interruption PASS");
    return 0;
}
