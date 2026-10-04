#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <setjmp.h>
#include <stdlib.h>
#include <stdarg.h>
#include "retained.h"
#define CONFIG_SENSOR_USE_SENS_CALIBRATION 1
static struct retained_data retained_storage;
struct retained_data *retained = &retained_storage;
static struct { uint32_t DEVICEADDR[2]; } ficr;
#define NRF_FICR (&ficr)
#define ESB_RF_CHANNEL_DEFAULT 255
#define CONFIG_RADIO_RF_CHANNEL 40
static uint8_t esb_rf_channel_decode(uint8_t channel) { return channel; }
static char printed[4096];
static void test_printk(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    size_t used = strlen(printed);
    vsnprintf(printed + used, sizeof(printed) - used, format, args);
    va_end(args);
}
static int64_t now_ms;
static int64_t k_uptime_get(void) { return now_ms; }
static unsigned clear_calls, sensitivity_writes, channel_writes;
static float saved_sensitivity[3];
static int saved_channel;
static int sys_clear(void) { clear_calls++; return 0; }
static int sensor_calibration_set_sensitivity(const float values[3])
{
    sensitivity_writes++;
    memcpy(saved_sensitivity, values, sizeof(saved_sensitivity));
    return 0;
}
static int channel_control_set(int channel)
{
    channel_writes++;
    saved_channel = channel;
    return 0;
}
static void cmd_reset_zro(void) {}
#if CONFIG_SENSOR_USE_ACCEL_CALIBRATION
static void cmd_reset_acc(void) {}
#endif
static void cmd_reset_bat(void) {}
static void cmd_fusion_reset(void) {}
static void cmd_sens_reset(void) {}
static void cmd_sens_auto(const char *axis, const char *revolutions) {}
static int sensor_calibration_clear_mag(void *unused, bool a, bool b) { return 0; }
#define CONFIG_USE_SLIMENRF_CONSOLE 1
#define USB_EXISTS 1
#define UART_CONSOLE_EXISTS 0
#include "console.h"
#include "system/led.h"
static jmp_buf worker_idle;

/* Only Zephyr/UART leaves are modeled; production.inc is extracted verbatim. */
#define CONFIG_CONSOLE_INPUT_MAX_LINE_LEN 128
#define CONFIG_SLIMEVR_USB_DEVICE_MANUFACTURER "test"
#define CONFIG_SLIMEVR_USB_DEVICE_PRODUCT "tracker"
#define FW_STRING "test"
#define FW_GIT_REPO_URL "test"
#define FW_GIT_BRANCH "test"
#define CONSOLE_THREAD_PRIORITY 8
#define BUILD_ASSERT _Static_assert
#define ARG_UNUSED(x) (void)(x)
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define K_NO_WAIT 0
#define K_FOREVER (-1)
#define K_THREAD_STACK_DEFINE(name, size) unsigned char name[size]
#define K_THREAD_STACK_SIZEOF(name) sizeof(name)
#define DEVICE_DT_GET(node) (&uart_device)
#define DT_CHOSEN(node) 0
#define LOG_ERR(...) ((void)0)
#define printk test_printk
struct device { int unused; };
static const struct device uart_device;
struct k_thread { int unused; };
typedef void (*k_thread_entry_t)(void *, void *, void *);
static unsigned thread_creations;
static void k_thread_create(struct k_thread *thread, void *stack, size_t size,
                            k_thread_entry_t entry, void *a, void *b, void *c,
                            int priority, int options, int delay)
{
    thread_creations++;
}
struct k_spinlock { bool held; };
typedef int k_spinlock_key_t;
static k_spinlock_key_t k_spin_lock(struct k_spinlock *lock)
{
    assert(!lock->held);
    lock->held = true;
    return 0;
}
static void k_spin_unlock(struct k_spinlock *lock, k_spinlock_key_t key)
{
    assert(lock->held);
    lock->held = false;
}
struct k_msgq {
    unsigned char *data;
    size_t size, capacity, head, count;
};
#define K_MSGQ_DEFINE(name, size, depth, alignment) \
    static unsigned char name##_storage[(size) * (depth)]; \
    static struct k_msgq name = {name##_storage, size, depth, 0, 0}
static int k_msgq_put(struct k_msgq *queue, const void *message, int timeout)
{
    assert(timeout == K_NO_WAIT);
    if (queue->count == queue->capacity) return -ENOMSG;
    size_t slot = (queue->head + queue->count++) % queue->capacity;
    memcpy(queue->data + slot * queue->size, message, queue->size);
    return 0;
}
static int k_msgq_get(struct k_msgq *queue, void *message, int timeout)
{
    if (!queue->count && timeout == K_FOREVER) longjmp(worker_idle, 1);
    if (!queue->count) return -ENOMSG;
    memcpy(message, queue->data + queue->head * queue->size, queue->size);
    queue->head = (queue->head + 1) % queue->capacity;
    queue->count--;
    return 0;
}
static unsigned char rx[8192];
static size_t rx_head, rx_count;
static bool rx_enabled, tx_enabled;
static size_t echoed, poll_calls;
static void (*irq_callback)(const struct device *, void *);
static bool device_is_ready(const struct device *dev) { return true; }
static void uart_irq_rx_disable(const struct device *dev) { rx_enabled = false; }
static void uart_irq_tx_disable(const struct device *dev) { tx_enabled = false; }
static void uart_irq_rx_enable(const struct device *dev) { rx_enabled = true; }
static void uart_irq_tx_enable(const struct device *dev) { tx_enabled = true; }
static int uart_irq_callback_user_data_set(const struct device *dev,
                                           void (*callback)(const struct device *, void *), void *data)
{
    irq_callback = callback;
    return 0;
}
static int uart_poll_in(const struct device *dev, unsigned char *byte)
{
    poll_calls++;
    if (!rx_count) return -1;
    *byte = rx[rx_head++];
    rx_count--;
    return 0;
}
static int uart_irq_update(const struct device *dev) { return 1; }
static int uart_irq_rx_ready(const struct device *dev) { return rx_enabled && rx_count; }
static int uart_irq_tx_ready(const struct device *dev) { return tx_enabled; }
static int uart_irq_is_pending(const struct device *dev)
{
    return uart_irq_rx_ready(dev) || uart_irq_tx_ready(dev);
}
static int uart_fifo_read(const struct device *dev, uint8_t *bytes, int size)
{
    assert(size == 1);
    if (!rx_count) return 0;
    *bytes = rx[rx_head++];
    rx_count--;
    return 1;
}
static int uart_fifo_fill(const struct device *dev, const uint8_t *bytes, int size)
{
    echoed += (size_t)size;
    return size;
}

#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
static void strtolower(char *text)
{
    for (; *text; text++) *text = (char)tolower((unsigned char)*text);
}
static unsigned completed_handlers;
static bool stop_during_handler;
static void handle_command(size_t argc, char **argv)
{
    if (stop_during_handler) {
        console_serial_close();
        console_serial_stop();
    }
    completed_handlers++;
}
static unsigned zro_requests, accel_requests;
static int sensor_request_calibration(void) { zro_requests++; return 0; }
#if CONFIG_SENSOR_USE_ACCEL_CALIBRATION
static int sensor_request_calibration_accel(void) { accel_requests++; return 0; }
#endif
/* This editor/lifecycle fixture does not drive a physical LED worker. */
uint32_t led_request_id(void) { return 1; }
uint32_t led_event_id(void) { return 1; }
enum led_admission led_request_event(enum led_owner owner, uint32_t request, uint32_t event,
                                    enum led_semantic semantic)
{
    (void)owner; (void)request; (void)event; (void)semantic;
    return LED_ADMITTED;
}
#include "production.inc"

/* Run the actual worker until its next empty blocking queue wait. */
static void run_worker(void)
{
    if (setjmp(worker_idle) == 0) console_thread();
}
static void stage(const char *text)
{
    assert(rx_count == 0);
    rx_head = 0;
    rx_count = strlen(text);
    assert(rx_count <= sizeof(rx));
    memcpy(rx, text, rx_count);
}
static void receive(const char *text)
{
    stage(text);
    irq_callback(console_uart_dev, NULL);
}
static struct console_line_message dequeue(void)
{
    struct console_line_message message;
    assert(k_msgq_get(&console_line_msgq, &message, K_NO_WAIT) == 0);
    return message;
}
static void accepted(const char *expected)
{
    struct console_line_message message = dequeue();
    assert(console_line_is_current(message.epoch));
    assert(strcmp(message.line, expected) == 0);
}
static void empty(void)
{
    struct console_line_message message;
    assert(k_msgq_get(&console_line_msgq, &message, K_NO_WAIT) == -ENOMSG);
}

int main(void)
{
    /* Startup stale UART bytes are still discarded before readiness. */
    stage("stale\n");
    console_serial_start();
    empty();
    assert(rx_enabled && thread_creations == 1);

    receive("info\n");
    console_serial_close();
    accepted("info"); /* Complete queued work survives while DTR remains low. */
    assert(!rx_enabled && !tx_enabled);

    console_serial_start();
    receive("uptime\n");
    struct console_line_message held = dequeue();
    console_serial_close();
    assert(console_line_is_current(held.epoch));
    console_serial_start();
    assert(console_line_is_current(held.epoch)); /* Dequeued before admission. */
    assert(strcmp(held.line, "uptime") == 0);

    receive("help\npartial");
    stage("remainder\n");
    console_serial_close();
    assert(rx_count == 0);
    size_t before = echoed;
    receive("closed\n");
    assert(echoed == before);
    console_serial_start(); /* Drain closed-port UART input, preserve queued help. */
    accepted("help");
    receive("info\n");
    accepted("info"); /* Partial input from the old opening cannot prefix this. */
    empty();

    receive("uptime\nhelp\n");
    held = dequeue();
    console_serial_close();
    console_serial_stop(); /* Hard reset after soft close invalidates both owners. */
    assert(!console_line_is_current(held.epoch));
    empty();
    console_serial_start();
    assert(!console_line_is_current(held.epoch));
    receive("info\n");
    accepted("info");

    for (unsigned i = 0; i < CONSOLE_LINE_QUEUE_DEPTH; i++) receive("help\n");
    receive("overflow\n");
    console_serial_close();
    console_serial_start();
    for (unsigned i = 0; i < CONSOLE_LINE_QUEUE_DEPTH; i++) accepted("help");
    empty(); /* Reject newest overflow; do not overwrite accepted messages. */

    /* Closing is bounded even with more unread UART data than the drain limit. */
    rx_head = 0;
    rx_count = sizeof(rx);
    memset(rx, 'x', sizeof(rx));
    poll_calls = 0;
    console_serial_close();
    assert(poll_calls == CONSOLE_INPUT_DRAIN_MAX);
    assert(rx_count == sizeof(rx) - CONSOLE_INPUT_DRAIN_MAX);
    console_serial_start();
    empty();
    assert(thread_creations == 1);
    receive("info\n");
    console_serial_close();
    run_worker();
    assert(completed_handlers == 1); /* Dispatch really runs while closed. */
    console_serial_start();
    receive("info\nhelp\n");
    stop_during_handler = true;
    run_worker();
    assert(completed_handlers == 2); /* Admitted handler finishes; queued help retires. */
    console_serial_start();
    stop_during_handler = false;
    receive("help\n");
    run_worker();
    assert(completed_handlers == 3);

    /* Actual UART editor, tokenizer, registration and worker: malformed
     * calibration requests must never fall through to the bare ZRO action. */
    const char *invalid[] = {
        "calibrate typo\n", "calibrate acc extra\n", "calibrate gyro\n",
        "calibrate acc a b c d e f g h\n", "6-side extra\n"
    };
    for (size_t i = 0; i < ARRAY_SIZE(invalid); i++) {
        receive(invalid[i]);
        run_worker();
        assert(zro_requests == 0 && accel_requests == 0);
    }
    receive("calibrate\n");
    run_worker();
    assert(zro_requests == 1 && accel_requests == 0);
    receive("calibrate acc\n");
    run_worker();
    assert(zro_requests == 1 && accel_requests == CONFIG_SENSOR_USE_ACCEL_CALIBRATION);
    receive("6-side\n");
    run_worker();
    assert(zro_requests == 1 && accel_requests == 2 * CONFIG_SENSOR_USE_ACCEL_CALIBRATION);
    receive("  CALIBRATE   ACC  \n");
    run_worker();
    assert(zro_requests == 1 && accel_requests == 3 * CONFIG_SENSOR_USE_ACCEL_CALIBRATION);
    receive("calibrate\n");
    run_worker();
    assert(zro_requests == 2 && accel_requests == 3 * CONFIG_SENSOR_USE_ACCEL_CALIBRATION);

    /* Production reset handler/worker, real DTR-close and hard-reset entrypoints. */
    receive("reset all\n"); run_worker();
    assert(clear_calls == 0);
    now_ms = CONSOLE_RESET_CONFIRM_MS;
    receive("reset all\n"); run_worker();
    assert(clear_calls == 0); /* exact deadline expired; re-arm */
    now_ms++;
    receive("RESET ALL\n"); run_worker();
    assert(clear_calls == 1);
    receive("reset all\nhelp\nreset all\n"); run_worker();
    assert(clear_calls == 1); /* unrelated command cancels */
    receive("reset all extra\nreset all\n"); run_worker();
    assert(clear_calls == 1); /* malformed confirmation cancels */
    receive("\nreset all\n"); run_worker();
    assert(clear_calls == 1); /* blank input cancels */
    console_serial_close();
    console_serial_start();
    receive("reset all\n"); run_worker();
    assert(clear_calls == 1); /* reconnect cannot confirm */
    console_serial_stop();
    console_serial_start();
    receive("reset all\n"); run_worker();
    assert(clear_calls == 1);
    /* Queued old-session requests are never allowed to arm or confirm. */
    receive("reset all\nreset all\n");
    console_serial_close();
    console_serial_start();
    run_worker();
    receive("reset all\n"); run_worker();
    assert(clear_calls == 1);
    receive("reset all\n"); run_worker();
    assert(clear_calls == 2);

    const char *bad_sens[] = {
        "sens 1,,2,3\n", "sens 1,2,3,\n", "sens ,1,2\n", "sens 1,2,\n",
        "sens 1,2\n", "sens 1,2,3,4\n", "sens 1,nan,3\n", "sens inf,2,3\n",
        "sens 1,2,1e1000\n", "sens 1,2,3junk\n", "sens 1,2,3 extra\n"
    };
    for (size_t i = 0; i < ARRAY_SIZE(bad_sens); i++) {
        receive(bad_sens[i]); run_worker();
        assert(sensitivity_writes == 0);
    }
    receive("sens 10.5,-2.1,15.0\n"); run_worker();
    assert(sensitivity_writes == 1);
    assert(saved_sensitivity[0] == 10.5f && saved_sensitivity[1] == -2.1f &&
           saved_sensitivity[2] == 15.0f);
    receive("sens +1,2e0,-0\n"); run_worker();
    assert(sensitivity_writes == 2 && saved_sensitivity[0] == 1.0f);

    long parsed = 123;
    const char *bad_numbers[] = {"", " ", "1x", "1 ", "-1", "101", "999999999999999999999999"};
    for (size_t i = 0; i < ARRAY_SIZE(bad_numbers); i++) {
        assert(!parse_long_bounded(bad_numbers[i], 0, 100, &parsed));
        assert(parsed == 123);
    }
    assert(parse_long_bounded("+100", 0, 100, &parsed) && parsed == 100);
    assert(parse_long_bounded("0", 0, 100, &parsed) && parsed == 0);
    receive("channel 999999999999999999999\nchannel 25junk\n"); run_worker();
    assert(channel_writes == 0);
    receive("channel 0\nchannel 100\n"); run_worker();
    assert(channel_writes == 2 && saved_channel == 100);
    float unchanged[3] = {4, 5, 6};
    assert(!parse_float_triplet("1,2,", unchanged));
    assert(unchanged[0] == 4 && unchanged[1] == 5 && unchanged[2] == 6);

    /* Real retained layout: paired_addr is deliberately NOT padded/aligned.
     * printk evaluates the actual production diagnostic arguments under UBSan. */
    assert((uintptr_t)retained->paired_addr % _Alignof(uint64_t) != 0);
    const uint8_t address[8] = {1, 7, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
    memcpy(retained->paired_addr, address, sizeof(address));
    ficr.DEVICEADDR[0] = 0x44332211;
    ficr.DEVICEADDR[1] = 0x6655;
    printed[0] = '\0';
    print_connection();
    assert(strstr(printed, "Receiver address: 665544332211") != NULL);
    assert(strstr(printed, "Device address: 665544332211") != NULL);
    puts("tracker production console lifecycle: PASS");
    return 0;
}
