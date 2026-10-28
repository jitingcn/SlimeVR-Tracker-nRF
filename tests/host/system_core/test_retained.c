#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "retained.h"
#include "test_timestamp.inc"

extern uint64_t test_ticks;
static uint64_t k_uptime_ticks(void) { return test_ticks; }
static void k_mutex_lock(int *lock, int timeout) { (void)lock; (void)timeout; }
static void k_mutex_unlock(int *lock) { (void)lock; }
#define K_FOREVER 0

#ifdef RETAINED_TEST_IMPLEMENTATION
static struct retained_data retained_storage;
struct retained_data *retained = &retained_storage;
static uint64_t init_time;
static int retained_lock;
#define NRF_STATIC_ASSERT(condition, message) _Static_assert(condition, message)
#define RETAINED_CRC_OFFSET offsetof(struct retained_data, crc)
#define RETAINED_CHECKED_SIZE (RETAINED_CRC_OFFSET + sizeof(retained->crc))
#define sys_cpu_to_le32(value) (value)

/* Model Zephyr's CRC-32/ISO-HDLC for the retained-memory host fixture. */
static uint32_t crc32_ieee(const uint8_t *data, size_t size)
{
    uint32_t crc = UINT32_MAX;
    while (size--) {
        crc ^= *data++;
        for (int bit = 0; bit < 8; bit++)
            crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1u));
    }
    return ~crc;
}
#include "production_retained.inc"
void test_retained_init(void) { retained_init(); }
#else
uint64_t test_ticks;
void test_retained_init(void);
static int sys_storage_lock, fs;
static bool nvs_init;
static bool nvs_available = true;
static int clear_result;
static int wom_timeout_delay_ms;
#if CONFIG_SENSOR_USE_TCAL
static int warm_tcal_armed, warm_tcal_receipt_count;
static void tcal_accum_request_reset(void) {}
#endif
static int warm_dirty_count;
#define WOM_TIMEOUT_DEFAULT_MS 600000
#define LED_OWNER_SYSTEM 0
#define LED_FAILED 0
#define LED_SUCCESS 1
#define LOG_ERR(...) ((void)0)
#define LOG_INF(...) ((void)0)
#define printk(...) ((void)0)
#define atomic_set(target, value) (*(target) = (value))
static void sensor_calibration_clear_begin(void) {}
static void sensor_calibration_clear_end(void) {}
static int sys_wom_timeout_change_begin(void) { return 0; }
static void sys_wom_timeout_change_end(bool changed) { (void)changed; }
static bool sys_nvs_init(void) { return nvs_available; }
static int nvs_clear(int *store) { (void)store; return clear_result; }
static int led_request_id(void) { return 0; }
static int led_event_id(void) { return 0; }
static void led_request_event(int owner, int request, int event, int result)
{ (void)owner; (void)request; (void)event; (void)result; }
#include "production_system_clear.inc"

static void assert_defaults(uint64_t now)
{
    assert(retained->max_battery_pptt == -1);
    assert(retained->min_battery_pptt == -1);
    assert(retained->battery_pptt_saved == -1);
    assert(retained->battery_runtime_sum == 0);
    assert(retained->uptime_sum == 0);
    assert(retained->uptime_latest == now);
    assert(retained->battery_uptime_latest == now);
    assert(retained->gyroSensScale[0] == 1.0f);
    assert(retained->gyroSensScale[1] == 1.0f);
    assert(retained->gyroSensScale[2] == 1.0f);
    assert(retained->rf_channel == 0xFF);
    assert(retained->watchdog_state.magic == WATCHDOG_STATE_MAGIC);
    assert(retained->watchdog_state.total_wdt_resets == 0);
#if CONFIG_SENSOR_USE_TCAL
    assert(retained->tcal_enabled);
#ifdef CONFIG_SENSOR_USE_BOOT_CALIBRATION
    assert(retained->bootCalState.enabled);
#else
    assert(!retained->bootCalState.enabled);
#endif
    assert(!retained->bootCalState.completed);
    assert(!retained->bootCalState.doffset_valid);
#endif
}

int main(void)
{
    memset(retained, 0xA5, sizeof(*retained));
    test_ticks = 42;
    test_retained_init();
    assert(!retained_validate());
    assert_defaults(42);
    uint32_t retained_timestamp = retained->build_timestamp;
    assert(retained_timestamp != BUILD_TIMESTAMP);

    retained->gyroBias[1] = 2.5f;
    retained->watchdog_state.total_wdt_resets = 9;
    test_ticks = 52;
    retained_update();
    assert(retained->uptime_sum == 10);
    test_ticks = 3;
    test_retained_init();
    assert(retained_validate());
    assert(retained->gyroBias[1] == 2.5f);
    assert(retained->watchdog_state.total_wdt_resets == 9);
    assert(retained->uptime_latest == 3 && retained->battery_uptime_latest == 3);

    struct retained_data before = *retained;
    clear_result = -EIO;
    assert(sys_clear() == -EIO);
    assert(memcmp(retained, &before, sizeof(before)) == 0);

    clear_result = 0;
    test_ticks = 123456;
    assert(sys_clear() == 0);
    assert(retained->build_timestamp == retained_timestamp);
    assert_defaults(123456);
    assert(wom_timeout_delay_ms == WOM_TIMEOUT_DEFAULT_MS);

    test_ticks = 123493;
    retained_update();
    assert(retained->uptime_sum == 37);
    test_ticks = 7;
    test_retained_init();
    assert(retained_validate());
    assert(retained->uptime_sum == 37);
    retained->build_timestamp ^= 1;
    retained_update();
    assert(!retained_validate());
    assert_defaults(7);
    assert(retained->build_timestamp == retained_timestamp);
    retained_update();
    assert(retained->battery_uptime_latest == 7);
    retained->gyroBias[0] = 8.0f;
    assert(!retained_validate());
    assert_defaults(7);
    assert(retained->gyroBias[0] == 0.0f);
    puts("retained reset and warm validation passed");
    return 0;
}
#endif
