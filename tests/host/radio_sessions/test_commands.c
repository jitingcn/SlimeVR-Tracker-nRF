#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <setjmp.h>
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
#define SYS_LED_PATTERN_ONESHOT_POWEROFF 1
#define SYS_LED_PRIORITY_HIGHEST 1
static uint8_t received_remote_command, acked_remote_command;
static uint16_t received_test_rate_tps, executing_test_rate_tps, acked_test_rate_tps;
static uint8_t received_batch_rate_hz, executing_batch_rate_hz, acked_batch_rate_hz;
static uint8_t received_metadata_mask, received_metadata_chunk, received_sens_auto_axis;
static uint16_t received_metadata_token, received_sens_auto_revolutions;
static bool metadata_echo_pending;
static uint32_t received_channel_value;
static float received_sens_data[3];
static struct { uint8_t data[13], length; } rx_payload = {.length = 13};
static uint16_t sys_get_be16(const uint8_t *p) { return ((uint16_t)p[0] << 8) | p[1]; }
static void connection_request_raw_metadata(uint8_t mask, uint8_t chunk, uint16_t token) {}
static inline unsigned irq_lock(void) { return 0; }
static inline void irq_unlock(unsigned key) {}
static void receive_control(uint8_t flag);
static void receive_stop(void);
static bool inject_stop, replaced_shutdown, batch_active = true;
static int64_t remote_command_receive_time = 1, now_ms = 1000;
static uint8_t paired_addr[8];
static struct { uint8_t paired_addr[8]; } retained_state, *retained = &retained_state;
static int esb_conn_state = 1;
static unsigned ping_failures, ping_success_streak;
static bool ping_pending, ping_failed;
static int64_t ping_send_time, connection_error_start_time;
static unsigned feeds, registrations, shutdown_calls, test_mode_changes;
static unsigned stop_after = 120, reject_count;
static int batch_result;
static jmp_buf thread_stop;
typedef void (*esb_remote_cmd_fn)(void);
static int64_t k_uptime_get(void) { return now_ms; }
static void watchdog_register_thread(int channel, int timeout) { registrations++; }
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
static void set_led(int pattern, int priority) {}
static int sys_request_system_off(void)
{
    shutdown_calls++;
    return shutdown_calls <= reject_count ? -EBUSY : 0;
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
    return 0;
}
