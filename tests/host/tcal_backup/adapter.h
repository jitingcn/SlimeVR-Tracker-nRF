#pragma once
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <errno.h>
#include <ctype.h>
#include <math.h>
#define CONFIG_SENSOR_USE_TCAL 1
#ifndef CONFIG_SENSOR_TCAL_HEATED
#define CONFIG_SENSOR_TCAL_HEATED 1
#endif
#define CONFIG_CMSIS_DSP 0
#define CONFIG_LOG_MODE_DEFERRED 1
#define CONFIG_LOG_PRINTK 1
#define IS_ENABLED(config) (config)
#define BUILD_ASSERT _Static_assert
#define MIN(a,b) ((a)<(b)?(a):(b))
static uint32_t sys_get_le32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
static void sys_put_le32(uint32_t v, uint8_t *p) {
    for (unsigned i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (i * 8));
}
typedef long atomic_t;
static long atomic_get(const atomic_t *value) { return *value; }
static void atomic_set(atomic_t *value, long next) { *value = next; }
static long atomic_inc(atomic_t *value) { return (*value)++; }
#define LED_OWNER_TCAL 1
static unsigned outcome_calls;
static int outcome_result;
static bool outcome_applied;
static int sensor_operation_result(int owner, int result, bool applied) {
    assert(!in_irq);
    (void)owner; outcome_calls++; outcome_result = result; outcome_applied = applied;
    return result;
}
#define CONFIG_SENSOR_POLY_TEMP_MIN 10
#define CONFIG_SENSOR_POLY_TEMP_MAX 45
#define TCAL_BUFFER_SIZE 70
#define snprintk snprintf
#define TCAL_DIR_UNKNOWN 0
static int tcal_current_direction, tcal_compensation_storage_error;
static float tcal_direction_ref_temp;
static bool warm_transaction;
static void (*before_storage_acquire)(void);
static void sys_warm_transaction_begin(void) {
    assert(!warm_transaction);
    if (before_storage_acquire) {
        void (*callback)(void) = before_storage_acquire;
        before_storage_acquire = NULL;
        callback();
    }
    warm_transaction = true;
}
static void sys_warm_transaction_end(bool flush) { (void)flush; assert(warm_transaction); warm_transaction = false; }
#define LOG_MODULE_REGISTER(...)
static void host_log(const char *format, ...) { (void)format; }
#define LOG_INF(...) host_log(__VA_ARGS__)
#define LOG_DBG(...) host_log(__VA_ARGS__)
#define LOG_ERR(...) host_log(__VA_ARGS__)
#define LOG_WRN(...) host_log(__VA_ARGS__)
typedef enum {
    SENSOR_TCAL_APPLY_DISABLED,
    SENSOR_TCAL_APPLY_ZRO_FALLBACK,
    SENSOR_TCAL_APPLY_CURVE,
} sensor_tcal_apply_mode_t;
#define MAIN_GYRO_TEMP_ID 32
#define CONFIG_SENSOR_POLY_STEPS_PER_DEGREE 2
#define TEMP_TO_IDX(temp) (int)(((float)(temp) - CONFIG_SENSOR_POLY_TEMP_MIN) * CONFIG_SENSOR_POLY_STEPS_PER_DEGREE)
#define MAIN_GYRO_TCAL_POINTS_ID 33
#define MAIN_GYRO_TCAL_COEFFS_ID 34
#define MAIN_GYRO_TCAL_STATE_ID 35
#define TCAL_ENABLED_ID 38
struct TempCalPoint { float temp; float bias[3]; };
struct retained_fixture {
    struct { uint16_t count; bool valid; uint8_t degree; } tempCalState;
    struct TempCalPoint tempCalPoints[TCAL_BUFFER_SIZE];
    float tempCalCoeffs[3][4];
    bool tcal_enabled;
    float gyroTemp;
    float gyroBias[3], accelBias[3], magBias[4][3], gyroSensScale;
    unsigned fusion_id;
    struct { bool doffset_valid; float doffset[3]; } bootCalState;
};
static struct retained_fixture retained_data, flash_data;
static struct retained_fixture *retained = &retained_data;
static bool tcal_compensation_enabled, tcal_curve_apply_ready;
static unsigned reset_reference_calls, persist_calls;
static bool maintenance;
#if !ACTUAL_OWNERSHIP
static unsigned accumulator_reset_calls;
static bool busy;
#endif
static int fail_id = -1;
static char output[32768];
static size_t output_size;
static unsigned printk_calls;
static bool background_logs;
static void append_background_log(void) {
    static const char log[] = "[background log]\n";
    assert(output_size + sizeof(log) < sizeof(output));
    memcpy(output + output_size, log, sizeof(log)); output_size += sizeof(log) - 1;
}
static unsigned reference_generation;
static bool measured_bias_reset_pending;
#define LED_CANCELLED 0
static void tcal_feedback_finish_locked(int state) { (void)state; }
static int printk(const char *format, ...) {
    assert(!in_irq && !lock_depth);
    printk_calls++;
    if (background_logs) append_background_log();
    va_list ap;
    va_start(ap, format);
    int n = vsnprintf(output + output_size, sizeof(output) - output_size, format, ap);
    va_end(ap);
    assert(n >= 0 && (size_t)n < sizeof(output) - output_size);
    output_size += n;
    if (background_logs) append_background_log();
    return n;
}
static void retained_update(void) {}
static void sensor_calibration_reset_gyro_reference(void) { reset_reference_calls++; }
#if !ACTUAL_OWNERSHIP
static int sensor_calibration_maintenance_begin(void) {
    if (busy || maintenance) return -EBUSY;
    maintenance = true;
    return 0;
}
static void sensor_calibration_maintenance_end(void) { assert(maintenance); maintenance = false; }
static void tcal_accum_request_reset(void) { accumulator_reset_calls++; }
#endif
static float sensor_get_current_imu_temperature(void) { return 25.0f; }
static int sys_write(uint16_t id, const void *data, void *retained_address, size_t size) {
    assert(!in_irq && id != TCAL_ENABLED_ID);
#if !ACTUAL_OWNERSHIP
    assert(maintenance);
#endif
    persist_calls++;
    if (retained_address != data) memcpy(retained_address, data, size);
    assert(lock_depth == 0 && warm_transaction);
    if ((int)id == fail_id) return -EIO;
    void *destination = NULL;
    size_t expected = 0;
#define RECORD(key, field) case key: destination = &flash_data.field; expected = sizeof(flash_data.field); break
    switch (id) {
    RECORD(MAIN_GYRO_TEMP_ID, gyroTemp);
    RECORD(MAIN_GYRO_TCAL_POINTS_ID, tempCalPoints);
    RECORD(MAIN_GYRO_TCAL_COEFFS_ID, tempCalCoeffs);
    RECORD(MAIN_GYRO_TCAL_STATE_ID, tempCalState);
    default: assert(!"write to unrelated calibration record");
    }
#undef RECORD
    assert(size == expected);
    memcpy(destination, data, size);
    return 0;
}
