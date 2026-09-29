#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "system/status.h"
#define __maybe_unused
#define K_MUTEX_DEFINE(name) static int name
#define K_FOREVER 0
#define ESB_ST_PAIRING 0
#define LOG_WRN(...) ((void)0)
#define LOG_INF(...) ((void)0)
#define LOG_DBG(...) ((void)0)
#define LOG_ERR(...) ((void)0)
#define USER_SHUTDOWN_ENABLED 1
#define CONFIG_CONNECTION_TIMEOUT_DELAY 120000
#define WDT_CHANNEL_ESB 0
#define SYS_LED_PATTERN_SHORT 0
#define SYS_LED_PATTERN_ONESHOT_COMPLETE 1
#define SYS_LED_PRIORITY_CONNECTION 0
#define ESB_ST_RECOVERING 2
#define TX_ERROR_THRESHOLD 300
#define ESB_ST_PAIRED 1
#define PAIRED_ID 32
static struct { uint64_t DEVICEADDR[1]; } ficr = {{0x123456789abc}};
#define NRF_FICR (&ficr)
static uint8_t paired_addr[8], radio_channel, paired_channel_found;
static uint32_t radio_session_generation;
static bool pair_ack_pending, clock_status, ping_failed;
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
static unsigned ping_failures, ping_success_streak, ota_rx_head, ota_rx_tail;
static bool ping_pending, shutdown_requested;
static int64_t connection_error_start_time, now;
static uint8_t tracker_id = 3, ping_ctr_sent, epoch;
static unsigned probes, writes, changes, disables;
static struct { uint8_t rf_channel, paired_addr[8]; } storage, *retained = &storage;
static struct { uint8_t data[13], length; } rx_payload;
#define RF_CHANNEL_ID 31
static int64_t k_uptime_get(void) { return now; }
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
    ++probes; ++ping_ctr_sent; ping_pending = true; return 0;
}
static void clocks_start(void) { clock_status = true; }
static void clocks_stop(void) { clock_status = false; }
static void esb_set_addr_discovery(void) {}
static void esb_set_addr_paired(void) {}
static void set_led(int pattern, int priority) { (void)pattern; (void)priority; }
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
    memcpy(dst, src, len); if (id == RF_CHANNEL_ID) ++writes; return 0;
}
static void esb_disable(void) { ++disables; }
static uint8_t tdma_get_config_epoch(void) { return epoch; }
static void tdma_update_config(uint8_t slot, uint8_t total, uint8_t ticks, uint8_t new_epoch) {
    assert(slot < total && ticks >= 16); epoch = new_epoch;
}
void set_status(enum sys_status status, bool value) {
    if (value) status_state |= status; else status_state &= ~status;
}
#include "channels.inc"
static void late_pong(void) {
    rx_payload.length = 13; rx_payload.data[0] = ESB_PONG_TYPE;
    rx_payload.data[1] = tracker_id; rx_payload.data[2] = ping_ctr_sent;
    rx_payload.data[12] = crc8_ccitt(7, rx_payload.data, 12);
    assert(!accept_pong());
}

static void reset(uint8_t home) {
    esb_initialized = true; idle = true; ota_active = false;
    ota_rx_head = ota_rx_tail = 0; ping_failures = 3;
    channel_search = channel_wait_normal = channel_found = channel_heard = false;
    radio_channel = home; now = 100; probes = changes = writes = 0;
    own_pong_time = (uint32_t)now;
    storage.rf_channel = esb_rf_channel_encode(home);
    memset(storage.paired_addr, 0x5a, sizeof(storage.paired_addr));
    status_state = SYS_STATUS_CONNECTION_ERROR | SYS_STATUS_USB_CONNECTED;
}
static void visit(uint8_t target) {
    assert(esb_channel_search_poll(false));
    for (unsigned i = 0; radio_channel != target && i < 101; ++i) {
        now = search_deadline; assert(esb_channel_search_poll(false));
    }
    assert(radio_channel == target && writes == 0);
}
int main(void) {
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
        assert(storage.rf_channel == esb_rf_channel_encode(pair_target));
        assert(memcmp(storage.paired_addr, paired_addr, 8) == 0);
        assert(paired_addr[1] == 3 && esb_conn_state == ESB_ST_PAIRED);
        assert(writes == 1 && !esb_initialized);
    }
    puts("channels: exhaustive coverage, strict own probes, NORMAL gate, persistence, OTA and interruption PASS");
    return 0;
}
