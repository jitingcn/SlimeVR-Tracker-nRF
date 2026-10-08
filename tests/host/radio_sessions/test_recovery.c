#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "system/status.h"

#define LOG_WRN(...) ((void)0)
#define ESB_ST_PAIRED 1
#define ESB_ST_RECOVERING 2
#define atomic_get(value) (*(value))

static int status_state;
static uint8_t tracker_id = 3;
static uint8_t ping_counter = 10, radio_channel = 84, channel_redirect;
static struct { uint8_t data[13]; } rx_payload;
static bool ping_pending, ping_failed, shutdown_requested;
static uint32_t ping_failures, ping_success_streak;
static int esb_conn_state;
static int64_t connection_error_start_time;
static bool channel_wait_normal, channel_heard, channel_found, channel_redirect_pending;
static bool channel_confirm_capable, channel_legacy_peer, channel_confirmed, ping_channel_confirm_sent;
static bool esb_ota_is_active(void) { return false; }
static void esb_ota_request_abort(void) { assert(false); }
static uint8_t ping_ctr_sent;
static uint32_t own_pong_time;
static bool own_pong_seen;
static int64_t k_uptime_get(void) { return 1000; }

/* Status publication is the hardware leaf; the getter and enum are real. */
void set_status(enum sys_status status, bool set)
{
    if (set) status_state |= status;
    else status_state &= ~status;
}

#include "recovery.inc"

static void disconnect(int other_status)
{
    status_state = other_status;
    set_status(SYS_STATUS_CONNECTION_ERROR, true);
    esb_conn_state = ESB_ST_RECOVERING;
    ping_pending = true;
    ping_failed = true;
    ping_failures = 317;
    ping_success_streak = 0;
    connection_error_start_time = 1000;
    shutdown_requested = true;
    ping_ctr_sent = ping_counter - 1;
    channel_confirmed = true;
    rx_payload.data[9] = 1;
}

static void reconnect(uint8_t pong_id, int other_status)
{
    rx_payload.data[1] = pong_id;
    rx_payload.data[2] = ping_counter - 1;
    for (unsigned i = 0; i < PING_RECOVERY_THRESHOLD; ++i) {
        ping_pending = true;
        receive_valid_pong();
    }
    assert(esb_conn_state == ESB_ST_PAIRED);
    assert(!ping_pending && !ping_failed && ping_failures == 0);
    /* This status gate stops all data in connection_thread while set. */
    assert(!get_status(SYS_STATUS_CONNECTION_ERROR));
    assert(status_state == other_status);
    assert(connection_error_start_time == 0 && !shutdown_requested);
    assert(ping_success_streak == 0);
}

int main(void)
{
    /* Only an outstanding own PONG can clear the data gate. */
    const int other_statuses[] = {0, SYS_STATUS_USB_CONNECTED,
        SYS_STATUS_SENSOR_ERROR | SYS_STATUS_PLUGGED};
    for (unsigned i = 0; i < sizeof(other_statuses) / sizeof(other_statuses[0]); ++i) {
        disconnect(other_statuses[i]);
        reconnect(tracker_id, other_statuses[i]);
        receive_valid_pong();
        assert(status_state == other_statuses[i]);
        disconnect(other_statuses[i]);
        rx_payload.data[1] = tracker_id + 1;
        receive_valid_pong();
        assert(get_status(SYS_STATUS_CONNECTION_ERROR));
        assert(ping_pending && ping_failures == 317);
        reconnect(tracker_id, other_statuses[i]);
    }
    puts("recovery: own live PONG clears data gate; shared-pipe PONG rejected; unrelated status preserved");
    return 0;
}
