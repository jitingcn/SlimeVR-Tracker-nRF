#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#define CONFIG_CONNECTION_TDMA 1
#define CONFIG_ESB_MAX_PAYLOAD_LENGTH 64
static void host_log(const char *format, ...) {}
#define LOG_DBG(...) host_log(__VA_ARGS__)
#define LOG_INF(...) host_log(__VA_ARGS__)
#define LOG_WRN(...) host_log(__VA_ARGS__)
#define LOG_ERR(...) host_log(__VA_ARGS__)
#define ESB_CREATE_PAYLOAD(...) {0}
#define ESB_ST_PAIRING 0
#define ENOMEM_ERROR_WINDOW_MS 1000
#define ENOMEM_ERROR_THRESHOLD 10
#define PING_HISTORY_SIZE 8
struct esb_payload { uint8_t pipe, noack, length, data[64]; };
static bool esb_initialized = true, clock_status = true, server_time_synced;
static int esb_conn_state = 1;
static uint8_t tracker_id = 2, ping_counter = 7, ping_ctr_sent;
static bool ping_pending, ping_failed;
static struct { uint8_t type; bool noack; size_t length; int64_t timestamp; } last_tx;
static struct { uint8_t counter; uint32_t ping_ticks, ping_ticks_kernel; } ping_history[PING_HISTORY_SIZE];
static unsigned ping_history_idx, consecutive_enomem_errors;
static unsigned esb_write_dropped, esb_write_dup_queued, esb_write_queued;
static int64_t last_enomem_time, last_tx_time, ping_send_time;
static int64_t now_ms = 1000;
static bool batch, tdma_enabled, idle = true;
static int hook_site, queue_calls, queue_failures;
static struct esb_payload queued[8];
static unsigned queued_count;
int esb_write(uint8_t *data, bool no_ack, size_t length);
static void interleave(int site)
{
    if (hook_site != site) return;
    hook_site = 0;
    uint8_t other[] = {0x70, 0x91, 0x82, 0x73, 0x64};
    assert(esb_write(other, false, sizeof(other)) == 0);
}
static int clocks_start(void) { return 0; }
static void drop_failed_tx_payload_if_pending(void) {}
static bool connection_get_data_collection_batch(void) { return batch; }
static void esb_write_rate_tick(void) {}
static uint64_t esb_get_server_time_ticks_64(void) { return 123; }
static uint8_t crc8_ccitt(uint8_t seed, const uint8_t *data, size_t size) { return seed; }
static int64_t k_uptime_get(void) { return now_ms; }
static uint64_t k_uptime_ticks(void) { return (uint64_t)now_ms * 32; }
static uint64_t net_ticks_from_kernel64(uint64_t ticks) { return ticks; }
static bool tdma_is_enabled(void) { return tdma_enabled; }
static int esb_get_sync_age_ms(void) { return -1; }
static bool esb_is_idle(void) { return idle; }
static void tdma_note_radio_busy(void) {}
static bool tdma_wait_for_slot(uint8_t size) { interleave(1); return true; }
static bool esb_ready(void) { return true; }
static uint32_t k_cycle_get_32(void) { return 500; }
static void k_usleep(uint32_t us) { interleave(2); }
static void k_msleep(int ms) { now_ms += ms; interleave(3); }
static int esb_write_payload(const struct esb_payload *payload)
{
    queue_calls++;
    if (queue_failures) { queue_failures--; return -ENOMEM; }
    assert(queued_count < 8);
    queued[queued_count++] = *payload;
    return 0;
}
static void esb_start_queued_tx(void) {}
static int esb_flush_tx(void) { return 0; }
static int esb_suspend(void) { return 0; }
static void esb_deinitialize(void) {}
static int esb_initialize(bool enabled) { return 0; }
static unsigned irq_lock(void) { return 0; }
static void irq_unlock(unsigned key) {}
#include "payload.inc"

static void reset(void)
{
    queued_count = 0; queue_calls = 0; queue_failures = 0;
    batch = false; tdma_enabled = false; idle = true;
}
static void assert_original(unsigned index, const uint8_t *data, size_t length, bool noack)
{
    assert(queued[index].length == length);
    assert(queued[index].pipe == 3);
    assert(queued[index].noack == noack);
    assert(memcmp(queued[index].data, data, length) == 0);
}
int main(void)
{
    uint8_t ordinary[] = {1, 2, 3, 4, 5, 6};
    for (int site = 1; site <= 2; site++) {
        reset(); hook_site = site; tdma_enabled = site == 1;
        assert(esb_write(ordinary, true, sizeof(ordinary)) == 0);
        assert(queued_count == 2);
        assert_original(1, ordinary, sizeof(ordinary), true);
    }
    /* Reliable raw retry sleeps with FIFO full; its duplicate must retain bytes. */
    uint8_t raw[] = {0x10, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77};
    reset(); hook_site = 3; idle = false; queue_failures = 1;
    assert(esb_write(raw, false, sizeof(raw)) == 0);
    assert(queued_count == 3 && queue_calls == 4);
    assert_original(1, raw, sizeof(raw), false);
    assert_original(2, raw, sizeof(raw), true);
    /* Post-wait PING history and timestamp decisions must use this call's bytes. */
    uint8_t ping[13] = {0xF0};
    reset(); hook_site = 3; idle = false; queue_failures = 1;
    unsigned index = ping_history_idx;
    assert(esb_write(ping, false, sizeof(ping)) == 0);
    assert_original(1, ping, sizeof(ping), false);
    assert(ping_history[index].counter == 7 && ping_ctr_sent == 7);
    assert(ping_history[index].ping_ticks != 0 && ping_history_idx == index + 1);
    puts("payload: TDMA, jitter, FIFO retry, duplicate and PING history remain call-local");
    return 0;
}
