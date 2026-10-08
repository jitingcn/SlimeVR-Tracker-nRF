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
#define RADIO_RF_CHANNEL 84
static struct { uint64_t DEVICEADDR[1]; } ficr = {{0x123456789abc}};
#define NRF_FICR (&ficr)
static uint8_t paired_addr[8], radio_channel;
static uint32_t radio_session_generation;
static bool pair_ack_pending, clock_status, ping_failed;
static bool own_pong_seen, pairing_search_active, radio_user_disabled;
static uint32_t pairing_request;
static int persistence_error, channel_error;
static unsigned address_writes, schedule_updates;
static uint8_t scheduled_slot, scheduled_total;
static struct esb_payload { uint8_t data[8]; bool noack; } tx_payload_pair;
static uint8_t pair_target, pair_step;
static uint8_t pair_probe_channels[320], last_probe_channel;
static unsigned pair_probe_count, pair_probe_limit;
static void receive_pair(void);
static bool inject_late_pong;
static void late_pong(void);
static unsigned irq_depth, deferred_pongs, delivered_pongs;
static bool pong_after_clock_read, pong_irq_pending;
static void dispatch_pong_irq(void);
static unsigned irq_lock(void) {
    unsigned key = irq_depth;
    ++irq_depth;
    return key;
}
static void irq_unlock(unsigned key) {
    assert(irq_depth > key);
    irq_depth = key;
    if (irq_depth == 0) dispatch_pong_irq();
}
static uint32_t ping_interval_ms = 1497;
static uint32_t get_ping_interval_ms(void) { return ping_interval_ms; }
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
    assert(!irq_depth);
    if (!strstr(format, "total")) return;
    assert(warning_count < 256);
    va_list args; va_start(args, format);
    uint32_t failures = va_arg(args, unsigned);
    va_end(args);
    warning_times[warning_count] = now;
    warning_failures[warning_count++] = failures;
    if (trace_warnings) printf("ping-warning t=%lldms failures=%u\n", (long long)now, failures);
}
static unsigned probes, writes, changes, disables, tdma_resets, tx_flushes, rx_flushes;
static struct { uint8_t rf_channel, paired_addr[8]; } storage, *retained = &storage;
static struct { uint8_t data[13], length; } rx_payload;
#define RF_CHANNEL_ID 31
static int64_t k_uptime_get(void) {
    int64_t sampled = now;
    if (pong_after_clock_read) {
        pong_after_clock_read = false;
        ++now; /* Hardware time advances after the owner's clock sample. */
        pong_irq_pending = true;
        if (irq_depth) ++deferred_pongs;
        else dispatch_pong_irq();
    }
    return sampled;
}
static uint32_t k_uptime_get_32(void) { return (uint32_t)now; }
static uint32_t k_cycle_get_32(void) { return (uint32_t)now; }
static bool esb_ota_is_active(void) { return ota_active; }
static unsigned abort_requests;
static void esb_ota_request_abort(void) { ++abort_requests; }
static bool esb_is_idle(void) { assert(!irq_depth); return idle; }
static void esb_clear_time_sync_state(void) { server_time_synced = false; }
static void tdma_set_enabled(bool enabled) { assert(!irq_depth); if (!enabled) ++tdma_resets; }
static void drop_failed_tx_payload_if_pending(void) {}
static void esb_flush_tx(void) { assert(idle && !irq_depth); ++tx_flushes; }
static void esb_flush_rx(void) { assert(idle && !irq_depth); ++rx_flushes; }
static int esb_set_rf_channel(uint8_t ch) { assert(idle && !irq_depth && ch <= 100); ++changes; return channel_error; }
static uint8_t esb_get_ping_ack_flag(void);
static bool metadata_echo_pending;
static uint8_t acked_remote_command, received_remote_command;
static uint8_t received_metadata_mask, received_metadata_chunk;
static uint16_t received_metadata_token;
static uint16_t test_mode_get_target_tps(void) { return 250; }
static uint16_t connection_get_data_collection_batch_rate(void) { return 100; }
static uint8_t last_probe[13];
static int esb_write_ping(uint8_t *ping, bool force) {
    assert(force && ping[0] == 0xf0 && ping[1] == tracker_id);
    last_probe_channel = radio_channel;
    memcpy(last_probe, ping, sizeof(last_probe));
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
static bool cancel_pair_after_ack;
void esb_deinitialize(void);
static void k_msleep(int delay) {
    now += delay;
    if (cancel_pair_after_ack && delay == 2 && paired_addr[0]) {
        cancel_pair_after_ack = false;
        esb_deinitialize();
    }
}
static int sys_request_system_off(void) { return 0; }
static int esb_initialize(bool tx) {
    (void)tx; esb_initialized = true;
    ++radio_session_generation;
    radio_channel = retained->rf_channel == 128 ? 0 :
        retained->rf_channel == 0xff ? RADIO_RF_CHANNEL : retained->rf_channel;
    return 0;
}
static int esb_write_payload(const struct esb_payload *payload) {
    pair_step = payload->data[1]; return 0;
}
static int esb_start_tx(void) {
    if (pair_probe_limit && pair_step == 0) {
        assert(pair_probe_count < sizeof(pair_probe_channels));
        pair_probe_channels[pair_probe_count++] = radio_channel;
        if (pair_probe_count == pair_probe_limit) ++radio_session_generation;
    }
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
    assert(!irq_depth);
    assert(id == RF_CHANNEL_ID || id == PAIRED_ID);
    if (id == RF_CHANNEL_ID) ++writes; else ++address_writes;
    if (!persistence_error) memcpy(dst, src, len);
    return persistence_error;
}
static void esb_disable(void) { ++disables; }
static uint8_t tdma_get_config_epoch(void) { return epoch; }
static void tdma_update_config(uint8_t slot, uint8_t total, uint8_t ticks, uint8_t new_epoch) {
    assert(slot < total && ticks >= 16); epoch = new_epoch;
    ++schedule_updates; scheduled_slot = slot; scheduled_total = total;
}
void set_status(enum sys_status status, bool value) {
    if (value) status_state |= status; else status_state &= ~status;
}
int get_status(enum sys_status status) { return status_state & status; }
#include "channels.inc"
static void packet_crc(void) {
    rx_payload.data[12] = crc8_ccitt(7, rx_payload.data, 12);
}
static void normal_packet(uint8_t slot, uint8_t total) {
    memset(&rx_payload, 0, sizeof(rx_payload));
    rx_payload.length = ESB_PONG_LEN; rx_payload.data[0] = ESB_PONG_TYPE;
    rx_payload.data[1] = tracker_id; rx_payload.data[2] = ping_ctr_sent;
    rx_payload.data[8] = slot; rx_payload.data[9] = total;
    rx_payload.data[10] = 16; rx_payload.data[11] = epoch + 1;
    packet_crc();
}
static void dedicated_packet(uint8_t channel) {
    normal_packet(0, 1);
    rx_payload.data[7] = ESB_PONG_FLAG_CHANNEL_CONFIRM;
    rx_payload.data[8] = channel;
    rx_payload.data[9] = ESB_CHANNEL_CONFIRM_VERSION;
    rx_payload.data[10] = rx_payload.data[11] = 0;
    packet_crc();
}
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
    normal_packet(0, 1);
    assert(!accept_pong());
}
static void dispatch_pong_irq(void) {
    assert(!irq_depth);
    if (pong_irq_pending) {
        pong_irq_pending = false;
        normal_packet(0, 1);
        assert(accept_pong());
        ++delivered_pongs;
    }
    /* The existing late-probe test targets invalidation, not an earlier
     * unrelated critical section such as the age snapshot. */
    if (inject_late_pong && !ping_pending) {
        inject_late_pong = false;
        late_pong();
    }
}

static void reset(uint8_t home) {
    assert(!irq_depth);
    pong_after_clock_read = pong_irq_pending = inject_late_pong = false;
    deferred_pongs = delivered_pongs = 0;
    tdma_resets = tx_flushes = rx_flushes = 0;
    ping_interval_ms = 1497;
    ++radio_session_generation;
    esb_conn_state = ESB_ST_PAIRED; connection_error_start_time = 0;
    ping_pending = ping_failed = false; ping_send_time = 0;
    ping_success_streak = 0; ping_counter = 0; ping_ctr_sent = 0;
    warning_count = 0; last_tx.type = ESB_PING_TYPE;
    esb_initialized = true; idle = true; ota_active = false;
    ota_rx_head = ota_rx_tail = 0; ping_failures = 3;
    channel_search = channel_wait_normal = channel_found = channel_heard = false;
    channel_redirect_pending = pair_provisional = pairing_search_active = false;
    channel_confirm_capable = channel_legacy_peer = channel_confirmed = false;
    ping_channel_confirm_sent = false; abort_requests = 0;
    cancel_pair_after_ack = false;
    pair_probe_count = pair_probe_limit = 0;
    channel_error = persistence_error = 0;
    radio_user_disabled = own_pong_seen = server_time_synced = false;
    address_writes = schedule_updates = 0;
    metadata_echo_pending = false; acked_remote_command = received_remote_command = 0;
    radio_channel = home; now = 100; probes = changes = writes = disables = 0;
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
    /* Dedicated proof plus a fresh legacy NORMAL completes recovery. */
    dedicated_packet(radio_channel);
    assert(accept_pong() && ping_failures == 0 && !ping_pending);
    assert(channel_confirmed && !channel_found);
    record_ping_admission(ping_counter);
    normal_packet(0, 1);
    assert(accept_pong() && !ping_pending);
    server_time_synced = true;
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
    channel_search = channel_wait_normal = false; ping_failures = 0; own_pong_time = now;
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

static void pong_age(void) {
    const int64_t samples[] = {10800000, UINT32_MAX};
    for (unsigned i = 0; i < sizeof(samples) / sizeof(samples[0]); ++i) {
        reset(2);
        now = samples[i];
        ping_interval_ms = 997;
        ping_failures = 0;
        status_state = SYS_STATUS_USB_CONNECTED;
        own_pong_seen = server_time_synced = true;
        own_pong_time = (uint32_t)(now - 997);
        record_ping_admission(ping_counter);
        pong_after_clock_read = true;
        bool searching = esb_channel_search_poll(false);
        printf("pong-age sample=%lld delivered=%u deferred=%u failures=%u "
               "warnings=%u error=%d search=%d tdma-resets=%u flushes=%u\n",
               (long long)samples[i], delivered_pongs, deferred_pongs, ping_failures,
               warning_count, !!get_status(SYS_STATUS_CONNECTION_ERROR),
               searching, tdma_resets, tx_flushes + rx_flushes);
        fflush(stdout);
        assert(delivered_pongs == 1 && !pong_irq_pending && !pong_after_clock_read);
        assert(own_pong_time == (uint32_t)now);
        assert(!searching && !channel_search && !channel_wait_normal);
        assert(ping_failures == 0 && warning_count == 0);
        assert(!get_status(SYS_STATUS_CONNECTION_ERROR) && connection_error_start_time == 0);
        assert(esb_conn_state == ESB_ST_PAIRED && server_time_synced);
        assert(!tdma_resets && !tx_flushes && !rx_flushes && !probes && !changes);
        assert(deferred_pongs == 1);
    }
    /* Real loss still crosses the exact healthy boundary, including the
     * uint32 uptime rollover; the outage clock uses the full uptime. */
    const int64_t last_pongs[] = {100, (int64_t)UINT32_MAX - 1000};
    for (unsigned i = 0; i < sizeof(last_pongs) / sizeof(last_pongs[0]); ++i) {
        reset(2);
        ping_failures = 0; status_state = SYS_STATUS_USB_CONNECTED;
        own_pong_time = (uint32_t)last_pongs[i];
        now = last_pongs[i] + 4499;
        assert(!esb_channel_search_poll(false));
        assert(!ping_failures && !warning_count && !tdma_resets && !tx_flushes && !rx_flushes);
        ++now;
        assert(esb_channel_search_poll(false));
        assert(channel_search && ping_failures == 4500 / get_ping_interval_ms());
        assert(warning_count == 1 && tdma_resets == 1);
        assert(!get_status(SYS_STATUS_CONNECTION_ERROR) && !connection_error_start_time);
        idle = false;
        now = last_pongs[i] + (TX_ERROR_THRESHOLD - 1) * get_ping_interval_ms();
        (void)esb_channel_search_poll(false);
        assert(ping_failures == TX_ERROR_THRESHOLD - 1 && !connection_error_start_time);
        now = last_pongs[i] + TX_ERROR_THRESHOLD * get_ping_interval_ms();
        (void)esb_channel_search_poll(false);
        assert(ping_failures == TX_ERROR_THRESHOLD);
        assert(connection_error_start_time == now && get_status(SYS_STATUS_CONNECTION_ERROR));
        int64_t outage_start = connection_error_start_time;
        now += 10000;
        (void)esb_channel_search_poll(false);
        assert(connection_error_start_time == outage_start);
    }
    puts("channels: coherent PONG IRQ age, true outage boundary and uint32 wrap PASS");
}
static void visit(uint8_t target) {
    assert(esb_channel_search_poll(false));
    for (unsigned i = 0; radio_channel != target && i < channel_candidate_count(search_home); ++i) {
        now = search_deadline; assert(esb_channel_search_poll(false));
    }
    assert(radio_channel == target && writes == 0);
}
static void confirm_channel(uint8_t target) {
    record_ping_admission(ping_counter);
    dedicated_packet(target);
    assert(accept_pong());
    assert(!channel_found);
    server_time_synced = false;
    record_ping_admission(ping_counter);
    normal_packet(0, 1); assert(accept_pong());
    receive_schedule(ESB_PONG_FLAG_NORMAL);
    assert(!channel_found && !writes && !address_writes);
    record_ping_admission(ping_counter);
    normal_packet(0, 1);
    assert(accept_pong());
    server_time_synced = true;
    receive_schedule(ESB_PONG_FLAG_NORMAL);
    assert(channel_found);
    assert(!esb_channel_search_poll(false));
}

static void advertised_channels(void) {
    /* Adjacent-channel decoding is not proof of the tuned physical channel. */
    const uint8_t targets[] = {84, 85, 0, 100};
    for (unsigned i = 0; i < sizeof(targets); ++i) {
        for (unsigned startup = 0; startup < 2; ++startup) {
            uint8_t from = startup ? 85 : 84;
            if (targets[i] == from) continue;
            reset(startup ? 85 : 84);
            if (!startup) storage.rf_channel = 0xff;
            if (startup) {
                ping_failures = 0; channel_wait_normal = true;
                assert(storage.rf_channel == 85);
            }
            visit(from);
            dedicated_packet(targets[i]);
            assert(!accept_pong());
            assert(channel_redirect_pending && !channel_found);
            assert(!schedule_updates && !writes && !address_writes && radio_channel == from);
            unsigned before = changes;
            ota_active = true;
            assert(!esb_channel_search_poll(false));
            assert(changes == before && !writes && radio_channel == from);
            ota_active = false;
            record_ping_admission(ping_counter); dedicated_packet(targets[i]);
            assert(!accept_pong() && channel_redirect_pending);
            idle = false;
            assert(esb_channel_search_poll(false));
            assert(changes == before && !writes);
            idle = true; channel_error = -EIO;
            assert(esb_channel_search_poll(false));
            assert(channel_redirect_pending && radio_channel == from && !writes);
            channel_error = 0;
            assert(esb_channel_search_poll(false));
            assert(radio_channel == targets[i] && !channel_redirect_pending);
            assert(!channel_found && !server_time_synced && !writes && !schedule_updates);
            dedicated_packet(targets[i]);
            --rx_payload.data[2]; packet_crc();
            assert(!accept_pong() && !writes);
            confirm_channel(targets[i]);
            assert(storage.rf_channel == (targets[i] == 84 ? 0xff :
                targets[i] == 0 ? 128 : targets[i]));
            assert(writes == (unsigned)(startup || targets[i] != 84) && !address_writes);
        }
    }
    /* Version, reserved bytes, physical range, request and live counter matter. */
    for (unsigned invalid = 0; invalid < 37; ++invalid) {
        reset(85); visit(85); dedicated_packet(84);
        if (invalid < 27) rx_payload.data[8] = 101 + invalid;
        else if (invalid == 27) rx_payload.data[9] = 2;
        else if (invalid == 28) ++rx_payload.data[1];
        else if (invalid == 29) --rx_payload.data[2];
        else if (invalid == 30) ping_pending = false;
        else if (invalid == 31) rx_payload.length = 12;
        else if (invalid == 32) rx_payload.data[10] = 1;
        else if (invalid == 33) rx_payload.data[11] = 1;
        else if (invalid == 34) rx_payload.data[9] = 0;
        else if (invalid == 35) ping_channel_confirm_sent = false;
        packet_crc();
        if (invalid == 36) rx_payload.data[12] ^= 1;
        assert(!accept_pong());
        assert(!channel_redirect_pending && !channel_found && !channel_heard);
        assert(!channel_legacy_peer && !channel_confirm_capable && !channel_confirmed);
        assert(radio_channel == 85 && !writes && !address_writes && !schedule_updates);
    }
    unsigned valid = 0;
    for (unsigned total = 1; total <= 16; ++total) {
        for (unsigned slot = 0; slot < total; ++slot) {
            reset(84); visit(84); dedicated_packet(84);
            assert(accept_pong());
            record_ping_admission(ping_counter); normal_packet(slot, total);
            assert(accept_pong()); server_time_synced = true;
            receive_schedule(ESB_PONG_FLAG_NORMAL);
            assert(channel_found && schedule_updates == 1);
            assert(scheduled_slot == slot && scheduled_total == total);
            ++valid;
        }
    }
    assert(valid == 136);
    reset(84); visit(84); dedicated_packet(84); assert(accept_pong());
    record_ping_admission(ping_counter); normal_packet(0xff, 0);
    assert(accept_pong()); server_time_synced = true;
    receive_schedule(ESB_PONG_FLAG_NORMAL);
    assert(!channel_found && !schedule_updates && !writes);
}

static void proof_during_channel_hold(void) {
    for (unsigned mode = 0; mode < 3; ++mode) {
        reset(84); storage.rf_channel = 0xff;
        ping_failures = 0; channel_wait_normal = true;
        ota_active = mode == 1; ota_rx_head = mode == 2;
        record_ping_admission(ping_counter);
        assert(!esb_channel_search_poll(mode == 0));
        assert(ping_pending && channel_wait_normal);
        assert(!changes && !tx_flushes && !rx_flushes && !writes);
        dedicated_packet(84);
        assert(accept_pong() && channel_confirmed);
        record_ping_admission(ping_counter); normal_packet(0, 1);
        assert(accept_pong()); server_time_synced = true;
        receive_schedule(ESB_PONG_FLAG_NORMAL); assert(channel_found);
        assert(!esb_channel_search_poll(mode == 0));
        assert(!channel_wait_normal && !channel_search && !channel_found);
        assert(!changes && !tx_flushes && !rx_flushes && !writes);
        /* Once proof finishes, ordinary control PONGs are admitted despite
         * suppression; the owner can execute UNSUPPRESS or OTA ABORT. */
        record_ping_admission(ping_counter); normal_packet(0, 1);
        rx_payload.data[7] = ESB_PONG_FLAG_OTA_UNSUPPRESS;
        packet_crc();
        assert(accept_pong());

        reset(85); ping_failures = 0; channel_wait_normal = true;
        ota_active = mode == 1; ota_rx_head = mode == 2;
        record_ping_admission(ping_counter); dedicated_packet(84);
        assert(!accept_pong() && channel_redirect_pending);
        assert(!esb_channel_search_poll(mode == 0));
        assert(radio_channel == 85 && channel_wait_normal && !channel_found);
        assert(!changes && !tx_flushes && !rx_flushes && !writes);
    }
    /* User-selected physical84 is not an automatic migration. */
    reset(84); ping_failures = 0; channel_wait_normal = true;
    assert(storage.rf_channel == 84);
    confirm_channel(84);
    assert(storage.rf_channel == 84 && !writes);
    /* Reconnect on that same custom channel keeps the explicit selection. */
    ping_failures = 3; visit(84); confirm_channel(84);
    assert(storage.rf_channel == 84 && !writes);
}

static void provisional_pairing(void) {
    const uint8_t targets[] = {0, 84, 100};
    for (unsigned i = 0; i < sizeof(targets); ++i) {
        reset(85); memset(paired_addr, 0, sizeof(paired_addr));
        pair_target = 85; registered_at = -1;
        unsigned successes = led_test_events[LED_SUCCESS];
        esb_pair();
        assert(pair_provisional && pairing_search_active);
        assert(paired_addr[1] == 3 && esb_conn_state == ESB_ST_PAIRED);
        assert(!writes && !address_writes && led_test_events[LED_SUCCESS] == successes);
        for (unsigned j = 0; j < 8; ++j) assert(storage.paired_addr[j] == 0x5a);
        esb_initialize(true); /* Connection owner starts the paired pipes. */
        visit(85); dedicated_packet(targets[i]);
        assert(!accept_pong() && channel_redirect_pending);
        assert(esb_channel_search_poll(false));
        assert(radio_channel == targets[i] && !writes && !address_writes);
        confirm_channel(targets[i]);
        assert(!pair_provisional && !pairing_search_active);
        assert(address_writes == 1 && writes == 1);
        assert(memcmp(storage.paired_addr, paired_addr, 8) == 0);
        assert(led_test_events[LED_SUCCESS] == successes + 1);
    }
    for (unsigned timeout = 0; timeout < 2; ++timeout) {
        reset(85); memset(paired_addr, 0, sizeof(paired_addr));
        pair_target = 85; registered_at = -1;
        unsigned successes = led_test_events[LED_SUCCESS];
        esb_pair(); assert(pair_provisional);
        if (timeout) {
            now = pair_confirm_deadline;
            (void)esb_channel_search_poll(false);
        } else {
            esb_deinitialize();
        }
        assert(!pair_provisional && !paired_addr[0] && !esb_initialized);
        assert(!writes && !address_writes && storage.rf_channel == 85);
        for (unsigned j = 0; j < 8; ++j) assert(storage.paired_addr[j] == 0x5a);
        assert(led_test_events[LED_SUCCESS] == successes);
    }
    reset(85); memset(paired_addr, 0, sizeof(paired_addr));
    pair_target = 85; registered_at = -1; cancel_pair_after_ack = true;
    unsigned prior_successes = led_test_events[LED_SUCCESS];
    esb_pair();
    assert(!cancel_pair_after_ack && !paired_addr[0] && !pair_provisional);
    assert(!esb_initialized && !writes && !address_writes && storage.rf_channel == 85);
    assert(led_test_events[LED_SUCCESS] == prior_successes);
    for (unsigned j = 0; j < 8; ++j) assert(storage.paired_addr[j] == 0x5a);
    reset(85); memset(paired_addr, 0, sizeof(paired_addr));
    pair_target = 85; registered_at = -1;
    unsigned successes = led_test_events[LED_SUCCESS], partials = led_test_events[LED_PARTIAL];
    esb_pair(); esb_initialize(true); visit(85);
    persistence_error = -EIO; confirm_channel(85);
    assert(led_test_events[LED_PARTIAL] == partials + 1);
    assert(led_test_events[LED_SUCCESS] == successes);
    for (unsigned j = 0; j < 8; ++j) assert(storage.paired_addr[j] == 0x5a);
}

static void legacy_compatibility(void) {
    const uint8_t replies[] = {ESB_PONG_FLAG_NORMAL, ESB_PONG_FLAG_OTA_QUERY_INFO,
        ESB_PONG_FLAG_DATA_COLLECT_METADATA, ESB_PONG_FLAG_TEST_MODE_ON};
    for (unsigned i = 0; i < sizeof(replies); ++i) {
        reset(84); storage.rf_channel = 0xff; visit(86);
        assert(ping_channel_confirm_sent);
        normal_packet(0, 1); rx_payload.data[7] = replies[i]; packet_crc();
        assert(accept_pong() && channel_legacy_peer && !channel_confirmed);
        assert(!channel_found && !writes && !address_writes);
        assert(!(esb_get_ping_ack_flag() & ESB_PING_FLAG_CHANNEL_CONFIRM));
        record_ping_admission(ping_counter); normal_packet(0, 1);
        assert(accept_pong()); server_time_synced = true;
        receive_schedule(ESB_PONG_FLAG_NORMAL);
        assert(channel_found && !esb_channel_search_poll(false));
        assert(radio_channel == 86 && storage.rf_channel == 0xff && !writes);
        /* A later recovery probes again, so a receiver upgrade is discovered. */
        ping_failures = 3; visit(86);
        assert(ping_channel_confirm_sent && !channel_legacy_peer);
        dedicated_packet(84); assert(!accept_pong() && channel_confirm_capable);
        assert(esb_channel_search_poll(false) && radio_channel == 84);
        /* A known new receiver can never be silently downgraded by NORMAL. */
        normal_packet(0, 1);
        assert(!accept_pong() && !channel_legacy_peer && !channel_found);
        confirm_channel(84);
        assert(storage.rf_channel == 0xff && !writes);
    }
    reset(84); visit(86);
    normal_packet(0, 1); ping_channel_confirm_sent = false;
    assert(!accept_pong() && !channel_legacy_peer);
    reset(84); visit(86);
    normal_packet(0, 1); rx_payload.data[7] = 0x7f; packet_crc();
    assert(!accept_pong() && !channel_legacy_peer);
    now += get_ping_interval_ms(); maintenance_timeout();
    assert(!channel_legacy_peer); /* Silence is never capability evidence. */
    reset(84); ping_failures = 0; channel_wait_normal = true;
    record_ping_admission(ping_counter); normal_packet(0, 1);
    assert(accept_pong()); server_time_synced = true;
    receive_schedule(ESB_PONG_FLAG_NORMAL);
    assert(!esb_channel_search_poll(false) && storage.rf_channel == 84 && !writes);
    /* Eight-byte legacy pairing identity commits only after valid NORMAL. */
    reset(84); memset(paired_addr, 0, sizeof(paired_addr));
    pair_target = 86; registered_at = -1;
    esb_pair(); assert(pair_provisional && !address_writes && !writes);
    esb_initialize(true); visit(86);
    normal_packet(0, 1); rx_payload.data[7] = ESB_PONG_FLAG_OTA_QUERY_INFO; packet_crc();
    assert(accept_pong() && channel_legacy_peer);
    assert(!address_writes && !writes);
    record_ping_admission(ping_counter); normal_packet(0, 1);
    assert(accept_pong()); server_time_synced = true;
    receive_schedule(ESB_PONG_FLAG_NORMAL);
    assert(!esb_channel_search_poll(false));
    assert(!pair_provisional && address_writes == 1 && !writes && storage.rf_channel == 84);
    assert(memcmp(storage.paired_addr, paired_addr, 8) == 0);
}

static void ota_status_abort(void) {
    for (unsigned pending = 0; pending < 2; ++pending) {
        for (unsigned invalid = 0; invalid < 5; ++invalid) {
            reset(85); channel_wait_normal = true; ota_active = true;
            ping_pending = pending; ping_ctr_sent = 91; ping_failed = true;
            ping_success_streak = 2; server_time_synced = true;
            normal_packet(0, 1); rx_payload.data[2] = 0;
            rx_payload.data[7] = ESB_PONG_FLAG_OTA_ABORT;
            if (invalid == 1) ++rx_payload.data[1];
            if (invalid == 2) rx_payload.length = 12;
            if (invalid == 3) rx_payload.data[0] = ESB_PING_TYPE;
            packet_crc();
            if (invalid == 4) rx_payload.data[12] ^= 1;
            uint32_t old_time = own_pong_time;
            int old_status = status_state;
            assert(!accept_pong());
            assert(abort_requests == (invalid == 0));
            assert(ping_pending == pending && ping_failed && ping_failures == 3);
            assert(ping_success_streak == 2 && own_pong_time == old_time && !own_pong_seen);
            assert(server_time_synced && status_state == old_status);
            assert(channel_wait_normal && !channel_found && !channel_heard);
            assert(!channel_confirmed && !channel_confirm_capable && !channel_legacy_peer);
            assert(!writes && !address_writes && !schedule_updates);
        }
    }
    reset(84); record_ping_admission(ping_counter); normal_packet(0, 1);
    rx_payload.data[7] = ESB_PONG_FLAG_OTA_ABORT; packet_crc();
    assert(accept_pong() && !abort_requests && !ping_pending);
}

static void owner_command_arguments(void) {
    const uint8_t flags[] = {ESB_PONG_FLAG_TEST_MODE_ON,
        ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON, ESB_PONG_FLAG_DATA_COLLECT_METADATA};
    const uint8_t expected[][4] = {{0, 250, 0, 0}, {100, 0, 0, 0}, {7, 2, 0x12, 0x34}};
    for (unsigned legacy = 0; legacy < 2; ++legacy) {
        for (unsigned command = 0; command < sizeof(flags); ++command) {
            reset(84); visit(84);
            if (legacy) normal_packet(0, 1); else dedicated_packet(84);
            assert(accept_pong() && channel_wait_normal);
            acked_remote_command = received_remote_command = flags[command];
            metadata_echo_pending = command == 2;
            received_metadata_mask = 7; received_metadata_chunk = 2; received_metadata_token = 0x1234;
            unsigned old_probes = probes;
            now = search_probe_at;
            assert(esb_channel_search_poll(false));
            assert(probes == old_probes + 1 && channel_wait_normal && !channel_found);
            assert(last_probe[7] == flags[command] && !ping_channel_confirm_sent);
            assert(memcmp(&last_probe[8], expected[command], 4) == 0);
            assert(acked_remote_command == flags[command]);
            assert(metadata_echo_pending == (command == 2));
        }
    }
    reset(84);
    acked_remote_command = received_remote_command = ESB_PONG_FLAG_TEST_MODE_ON;
    metadata_echo_pending = true;
    visit(84);
    const uint8_t zero[4] = {0};
    assert(last_probe[7] == ESB_PING_FLAG_CHANNEL_CONFIRM);
    assert(memcmp(&last_probe[8], zero, 4) == 0);
    dedicated_packet(84); assert(accept_pong());
    assert(acked_remote_command == ESB_PONG_FLAG_TEST_MODE_ON && metadata_echo_pending);
    assert(received_remote_command == ESB_PONG_FLAG_TEST_MODE_ON && !schedule_updates);
}

static void scan_sequences(void) {
    /* Independent policy oracle: preserve preferred order, then remaining evens. */
    const uint8_t order[] = {
        0, 2, 52, 72, 74, 76, 78, 82, 84, 86, 88, 50, 24, 48,
        70, 68, 46, 44, 20, 54, 56, 28, 30,
        6, 8, 10, 12, 14, 16, 18, 32, 34,
        36, 38, 40, 42, 58, 60, 62, 64, 66,
        4, 22, 26, 80, 90, 92, 94, 96, 98, 100,
    };
    assert(sizeof(order) == 51);
    for (unsigned home = 0; home <= 100; ++home) {
        uint8_t expected[52] = {home};
        unsigned count = 1;
        bool seen[101] = {0};
        for (unsigned i = 0; i < sizeof(order); ++i)
            if (order[i] != home) expected[count++] = order[i];
        assert(count == 51 + (home & 1));
        assert(channel_candidate_count(home) == count);
        for (unsigned i = 0; i < count; ++i) {
            uint8_t ch = channel_candidate(home, i);
            assert(ch == expected[i] && ch <= 100 && !seen[ch]);
            assert(!(ch & 1) || (i == 0 && ch == home));
            seen[ch] = true;
        }
        for (unsigned ch = 0; ch <= 100; ++ch)
            assert(seen[ch] == (!(ch & 1) || ch == home));

        /* Observe actual owner PING submissions through two complete wraps. */
        reset(home);
        for (unsigned i = 0; i <= count * 2; ++i) {
            if (i) now = search_deadline;
            unsigned before = probes;
            assert(esb_channel_search_poll(false));
            assert(probes == before + 1 && last_probe_channel == expected[i % count]);
            assert(search_index == i % count);
        }
        assert(changes == count * 2 && !writes && !address_writes);
        /* OTA freezes the active sweep, including an explicit odd home. */
        ota_active = true; now = search_deadline;
        unsigned old_probes = probes, old_changes = changes;
        assert(!esb_channel_search_poll(false));
        assert(probes == old_probes && changes == old_changes && radio_channel == home);

        /* Actual discovery sends three bursts per candidate; model no peer
         * and interrupt via the existing lifecycle generation after two wraps. */
        reset(home); memset(paired_addr, 0, sizeof(paired_addr));
        pair_target = 0xff; registered_at = -1;
        pair_probe_limit = (count * 2 + 1) * 3;
        esb_pair();
        assert(pair_probe_count == pair_probe_limit && !pairing_search_active);
        for (unsigned i = 0; i < pair_probe_count; ++i)
            assert(pair_probe_channels[i] == expected[(i / 3) % count]);
        assert(changes == count * 2 && !writes && !address_writes && !pair_provisional);
    }
    puts("channels: actual recovery and pairing, all homes, 51 evens, odd home, two wraps PASS");
}

int main(void) {
    if (getenv("RADIO_EVEN_ONLY")) {
        scan_sequences();
        advertised_channels();
        proof_during_channel_hold();
        puts("channels: odd dedicated redirects and OTA holds PASS");
        return 0;
    }
    trace_warnings = getenv("RADIO_PING_TRACE") != NULL;
    pong_age();
    if (getenv("RADIO_PONG_AGE_ONLY")) return 0;
    cadence();
    warning_lifecycle();
    puts("channels: 120s production-path ping cadence, recovery and suppression PASS");
    if (getenv("RADIO_PING_ONLY")) return 0;
    advertised_channels();
    proof_during_channel_hold();
    legacy_compatibility();
    ota_status_abort();
    owner_command_arguments();
    scan_sequences();
    const uint8_t destinations[] = {0, 4, 100};
    for (unsigned i = 0; i < sizeof(destinations); ++i) {
        reset(2); visit(destinations[i]);
        dedicated_packet(destinations[i]);
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
        record_ping_admission(ping_counter); normal_packet(0xff, 0);
        assert(accept_pong()); server_time_synced = true;
        receive_schedule(ESB_PONG_FLAG_NORMAL); assert(!channel_found);
        normal_packet(0, 1);
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
    idle = true; visit(4); unsigned old_changes = changes, old_probes = probes;
    ota_active = true; now += 1000;
    assert(!esb_channel_search_poll(false) && changes == old_changes && probes == old_probes);
    ota_active = false; esb_deinitialize();
    assert(!esb_initialized && !channel_search && channel_wait_normal && !channel_found);
    assert(storage.rf_channel == 2 && writes == 0 && disables == 1);
    provisional_pairing();
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
