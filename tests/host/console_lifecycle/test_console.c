#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <setjmp.h>
#define CONFIG_USE_SLIMENRF_CONSOLE 1
#define USB_EXISTS 1
#define UART_CONSOLE_EXISTS 0
#include "console.h"
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
#define printk(...) ((void)0)
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
static size_t parse_args(char *line, char **argv, size_t capacity)
{
    if (!*line) return 0;
    argv[0] = line;
    return 1;
}
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
static const struct {
    const char *name;
    void (*fn)(size_t, char **);
} console_cmds[] = {{"info", handle_command}, {"help", handle_command}};
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
    puts("tracker production console lifecycle: PASS");
    return 0;
}
