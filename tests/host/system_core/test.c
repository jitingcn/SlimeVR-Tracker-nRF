#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <errno.h>
#include <stdio.h>
#include "system/status.h"
#define LOG_ERR(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define LOG_INF(...) ((void)0)
#define K_FOREVER 0
#define BUTTON_EXISTS 1
#define DOCK_EXISTS 1
#define CHG_EXISTS 1
#define STBY_EXISTS 1
#define RBT_CNT_ID 1
#define LED_BUTTON_HOLD_MS 1000
#define STATUS_LED_ERROR_MASK (SYS_STATUS_SENSOR_ERROR | SYS_STATUS_SYSTEM_ERROR)
static int sys_storage_lock, fs;
static bool ram_retention_valid, reboot_counter_ram_only;
static struct { uint8_t reboot_counter; } retained_data, *retained = &retained_data;
static bool mount_ok;
static int read_result, write_result, reads, writes, seals;
static uint8_t flash_counter;
static void k_mutex_lock(int *lock, int timeout) { (void)lock; }
static void k_mutex_unlock(int *lock) { (void)lock; }
static bool sys_nvs_init(void) { return mount_ok; }
static int nvs_read(int *store, int id, void *out, size_t size)
{
    reads++;
    *(uint8_t *)out = flash_counter; /* Even a failing driver may dirty output. */
    return read_result;
}
static int nvs_write(int *store, int id, const void *value, size_t size)
{
    writes++;
    if (write_result >= 0) flash_counter = *(const uint8_t *)value;
    return write_result;
}
static void retained_update(void) { seals++; }
struct device { int unused; };
struct gpio_callback { int unused; };
static int button0, dock, chg, stby, gpio_level;
static bool button_held_from_init;
static int gpio_pin_get_dt(const int *pin) { return gpio_level; }
static int64_t press_time, last_press_duration, last_press_started_at;
static uint32_t press_generation, last_press_generation;
static int64_t now;
static int64_t k_uptime_get(void) { return now; }
static uint32_t led_button_input(void) { return 1; }
static void led_button_hold(uint32_t generation, bool active, uint32_t time) {}
static int status_state, status_lock, status_wake_sem;
static enum sys_sensor_fault status_sensor_fault;
typedef int k_spinlock_key_t;
static bool locked, inject_update;
static int published;
static void status_update(enum sys_status, bool, enum sys_sensor_fault, bool);
static int k_spin_lock(int *lock) { assert(!locked); locked = true; return 0; }
static void k_spin_unlock(int *lock, int key)
{
    assert(locked);
    locked = false;
    /* Deterministically preempt A immediately after its unlock, run B fully. */
    if (inject_update) {
        inject_update = false;
        status_update(SYS_STATUS_USB_CONNECTED, true, SYS_SENSOR_FAULT_OTHER, false);
    }
}
static int atomic_get(int *state) { return *state; }
static void atomic_or(int *state, int value) { *state |= value; }
static void atomic_and(int *state, int value) { *state &= value; }
static void k_sem_give(int *sem) {}
static void connection_update_status(int value) { published = value; }
static unsigned hour, minute, second;
#define BUILD_YEAR 2026
#define BUILD_MONTH 10
#define BUILD_DAY 4
#define BUILD_HOUR hour
#define BUILD_MIN minute
#define BUILD_SEC second
#include "production.inc"

static void reset_nvs(void)
{
    ram_retention_valid = reboot_counter_ram_only = false;
    mount_ok = true;
    reads = writes = seals = 0;
    read_result = write_result = 1;
    flash_counter = 3;
    retained->reboot_counter = 0;
}
int main(void)
{
    unsigned previous = 0;
    for (hour = 0; hour < 24; hour++) {
        for (minute = 0; minute < 60; minute++) {
            for (second = 0; second < 60; second++) {
                unsigned timestamp = BUILD_TIMESTAMP;
                if (hour || minute || second) assert(timestamp == previous + 1);
                previous = timestamp;
            }
        }
    }
    reset_nvs(); mount_ok = false;
    assert(reboot_counter_read() == 0 && reads == 0);
    reboot_counter_write(7);
    mount_ok = true;
    assert(reboot_counter_read() == 7 && reads == 0 && writes == 0);
    for (int error = 0; error < 3; error++) {
        reset_nvs(); read_result = error == 0 ? -EIO : error == 1 ? 0 : 2;
        assert(reboot_counter_read() == 0);
        reboot_counter_write(6);
        assert(reboot_counter_read() == 6 && reads == 1 && writes == 0);
    }
    reset_nvs(); read_result = -ENOENT;
    assert(reboot_counter_read() == 0);
    reboot_counter_write(1);
    assert(writes == 1 && flash_counter == 1);
    reset_nvs(); assert(reboot_counter_read() == 3);
    write_result = -EIO; reboot_counter_write(8);
    assert(reboot_counter_read() == 8 && reads == 1);
    reset_nvs(); write_result = 0; reboot_counter_write(3);
    assert(!reboot_counter_ram_only);
    reset_nvs(); mount_ok = false; reboot_counter_write(9);
    assert(writes == 0 && reboot_counter_read() == 9);
    reset_nvs(); ram_retention_valid = true; retained->reboot_counter = 4;
    assert(reboot_counter_read() == 4 && reads == 0);
    reboot_counter_write(5); assert(writes == 0 && seals == 1);

    for (gpio_level = -10; gpio_level <= 1; gpio_level++) {
        bool active = gpio_level > 0;
        button_held_from_init = false;
        assert(button_read() == active && button_read_filtered() == active);
        assert(dock_read() == active && chg_read() == active && stby_read() == active);
        button_held_from_init = true;
        assert(!button_read_filtered());
    }
    gpio_level = 1; now = 100; button_interrupt_handler(NULL, NULL, 0);
    assert(press_generation != 0);
    gpio_level = -EIO; now = 6000; button_interrupt_handler(NULL, NULL, 0);
    assert(press_generation == 0 && last_press_duration == 0);
    gpio_level = 0; button_interrupt_handler(NULL, NULL, 0);
    assert(last_press_duration == 0); /* Error cannot turn into a hold/release action. */

    inject_update = true;
    status_update(SYS_STATUS_SENSOR_ERROR, true, SYS_SENSOR_FAULT_MISSING, true);
    assert(published == (SYS_STATUS_SENSOR_ERROR | SYS_STATUS_USB_CONNECTED));
    assert(published == status_state && !locked);
    status_update(SYS_STATUS_SENSOR_ERROR, false, SYS_SENSOR_FAULT_NONE, true);
    assert(published == SYS_STATUS_USB_CONNECTED && status_sensor_fault == SYS_SENSOR_FAULT_NONE);
    puts("system core regressions passed");
    return 0;
}
