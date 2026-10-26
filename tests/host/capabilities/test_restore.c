#define CONFIG_SENSOR_USE_TCAL 0
#define CONFIG_SENSOR_TCAL_HEATED 0
#define CONFIG_SENSOR_MAG_ODR 50
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/sys/atomic.h>
#include "retained.h"
#include "system/led.h"
#include "sensor/calibration/online_mag.h"
#include "util.h"
#define LOG_INF(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define LOG_ERR(...) ((void)0)
#define POWER_RESETREAS_RESETPIN_Msk 1
static struct retained_data storage, flash;
struct retained_data *retained = &storage;
static bool warm_boot, ram_retention_valid;
static unsigned nvs_reads, retained_updates, imu_loads, model_restores, model_configures, online_loads;
static bool configured_online;
static float live_matrix[4][3], magBias[3];
static uint8_t sensor_data[128];
static struct { uint16_t addr; } sensor_imu_dev, sensor_mag_dev;
static uint8_t sensor_imu_dev_reg, sensor_mag_dev_reg;
static bool mag_enabled = true, mag_available = true, mag_calibrated = true;
static float mag_actual_time = 0.02f;
static int sensor_imu_id = 7, sensor_mag_id = 3, published_imu, published_mag;
static uint8_t published_mag_status;
static atomic_t data_collection_active, data_collection_batch_active;
static uint16_t data_collection_batch_rate_hz;
static void wom_timeout_load(void) {}
static uint32_t sys_get_reset_reason(void) { return 0; }
static bool sys_nvs_init(void) { return true; }
static void sys_migrate_battery_curve(void) {}
bool retained_validate(void)
{
    if (!warm_boot) memset(retained, 0, sizeof(*retained));
    return warm_boot;
}
void retained_update(void) { retained_updates++; }
static void sys_read(uint16_t id, void *data, size_t size);
static void sensor_calibration_imu_load(void)
{
    imu_loads++;
    assert(retained->gyroSensScale[0] == 1.0f);
    assert(retained->gyroSensScale[1] == 1.02f);
    assert(retained->gyroSensScale[2] == 1.0f);
}
void magneto_online_replace_BAinv_and_reset(const float matrix[4][3], uint16_t operation, struct led_token feedback)
{
    assert(operation == 0 && feedback.session == 0);
    model_restores++;
    memcpy(live_matrix, matrix, sizeof(live_matrix));
}
void magneto_online_runtime_configure(bool enabled) { configured_online = enabled; model_configures++; }
void magneto_online_runtime_load_retained(void) { online_loads++; }
static bool sensor_calibration_get_online_mag_enabled(void) { return configured_online; }
void magneto_online_snapshot_BAinv(float out[4][3]) { memcpy(out, live_matrix, sizeof(live_matrix)); }
static bool mag_bainv_structurally_ok(const float matrix[4][3], float minimum)
{
    (void)minimum;
    return matrix[1][0] > 0 && matrix[2][1] > 0 && matrix[3][2] > 0;
}
int cal_online_mag_update_count(void) { return 0; }
uint32_t cal_online_mag_norm_count(void) { return 0; }
static void connection_update_sensor_ids(int imu, int mag);
#include "restore.inc"

static void connection_update_sensor_ids(int imu, int mag)
{
    published_imu = imu;
    published_mag = mag;
    published_mag_status = get_server_constant_mag_id(mag);
}

static void sys_read(uint16_t id, void *data, size_t size)
{
    const void *source = NULL;
    size_t expected = 0;
#define RECORD(key, field) case key: source = &flash.field; expected = sizeof(flash.field); break
    switch (id) {
    RECORD(PAIRED_ID, paired_addr);
    RECORD(MAIN_SENSOR_DATA_ID, sensor_data);
    RECORD(MAIN_ACCEL_BIAS_ID, accelBias);
    RECORD(MAIN_GYRO_BIAS_ID, gyroBias);
    RECORD(MAIN_MAG_BIAS_ID, magBAinv);
    RECORD(MAIN_ACC_6_BIAS_ID, accBAinv);
    RECORD(BATT_STATS_CURVE_ID, battery_pptt_curve);
    RECORD(MAIN_GYRO_SENS_ID, gyroSensScale);
    RECORD(RF_CHANNEL_ID, rf_channel);
    RECORD(MAG_ENABLED_ID, mag_enabled);
    RECORD(MAG_ONLINE_CALIBRATION_ID, mag_online_calibration_mode);
    default: assert(!"unexpected retained storage key");
    }
#undef RECORD
    assert(size == expected);
    memcpy(data, source, size);
    nvs_reads++;
}

static void magnetic_equal(const struct retained_data *expected)
{
    assert(memcmp(retained->magBias, expected->magBias, sizeof(retained->magBias)) == 0);
    assert(memcmp(retained->magBAinv, expected->magBAinv, sizeof(retained->magBAinv)) == 0);
    assert(retained->mag_enabled == expected->mag_enabled);
    assert(retained->mag_online_calibration_mode == expected->mag_online_calibration_mode);
    assert(retained->mag_addr == expected->mag_addr && retained->mag_reg == expected->mag_reg);
    assert(memcmp(&retained->onlineMagState, &expected->onlineMagState, sizeof(retained->onlineMagState)) == 0);
}

static void restore_case(bool warm, bool invalid)
{
    memset(&storage, 0, sizeof(storage));
    storage.mag_enabled = true;
    storage.mag_online_calibration_mode = invalid ? 255 : 1;
    storage.mag_addr = 0x1e;
    storage.mag_reg = 0x4f;
    storage.magBAinv[0][0] = 0.25f;
    storage.magBAinv[1][0] = storage.magBAinv[2][1] = storage.magBAinv[3][2] = 1.0f;
    storage.magBias[0] = -0.125f;
    storage.onlineMagState.last_buf_avg_norm = 0.5f;
    storage.onlineMagState.update_count = 17;
    storage.onlineMagState.reserved[0] = 0x31;
    storage.onlineMagState.reserved[1] = 0x42;
    storage.onlineMagState.reserved[2] = 0x53;
    storage.gyroSensScale[0] = 0;
    storage.gyroSensScale[1] = 1.02f;
    storage.gyroSensScale[2] = 2;
    storage.imu_addr = 0x68;
    storage.imu_reg = 0x75;
    storage.sensor_data[7] = 0xab;
    if (invalid) {
        uint32_t nan = 0x7fc12345;
        memcpy(&storage.magBAinv[2][1], &nan, sizeof(nan));
        memcpy(&storage.magBias[0], &nan, sizeof(nan));
    }
    flash = storage;
    struct retained_data expected = storage;
    if (!warm) {
        memset(expected.magBias, 0, sizeof(expected.magBias));
        expected.mag_addr = 0;
        expected.mag_reg = 0;
        memset(&expected.onlineMagState, 0, sizeof(expected.onlineMagState));
    }
    warm_boot = warm;
    nvs_reads = retained_updates = imu_loads = model_restores = model_configures = online_loads = 0;
    configured_online = false;
    memset(&sensor_imu_dev, 0, sizeof(sensor_imu_dev));
    sensor_imu_dev_reg = 0xff;
    memset(&sensor_mag_dev, 0, sizeof(sensor_mag_dev));
    sensor_mag_dev_reg = 0xff;
    assert(sys_retained_init() == 0);
    assert(nvs_reads == (warm ? 0U : 11U));
#if !CONFIG_SENSOR_USE_MAGNETOMETER
    magnetic_equal(&expected);
#endif
    sensor_calibration_read();
    assert(imu_loads == 1 && retained->gyroSensScale[0] == 1 && retained->gyroSensScale[1] == 1.02f && retained->gyroSensScale[2] == 1);
    assert(sensor_data[7] == 0xab);
    sensor_scan_read();
    assert(sensor_imu_dev.addr == (warm ? 0x68 : 0));
    sensor_scan_write();
    sensor_refresh_sensor_ids();
    assert(published_imu == 7);
#if CONFIG_SENSOR_USE_MAGNETOMETER
    assert(sensor_get_mag_enabled() && sensor_get_mag_available() && sensor_get_mag_calibrated());
    assert(sensor_get_mag_odr() == 50 && published_mag == 3 && published_mag_status == 2);
    assert(model_restores == 1 && model_configures == 1 && online_loads == 1 && configured_online);
    assert(retained->mag_online_calibration_mode == (invalid ? 0 : 1));
    if (invalid) {
        assert(live_matrix[2][1] == 1 && retained->magBias[0] == 0);
    } else {
        assert(live_matrix[0][0] == 0.25f && live_matrix[1][0] == 1);
    }
#else
    magnetic_equal(&expected);
    assert(!sensor_get_mag_enabled() && !sensor_get_mag_available() && !sensor_get_mag_calibrated());
    assert(sensor_get_mag_odr() == 0 && published_mag == -1 && published_mag_status == 0);
    assert(model_restores == 0 && model_configures == 0 && online_loads == 0);
    sensor_scan_clear();
    magnetic_equal(&expected);
#endif
    assert(!connection_get_data_collection() && !connection_get_data_collection_batch());
    assert(connection_get_data_collection_batch_rate() == 0);
    if (getenv("CAPABILITY_SMOKE")) printf("restore raw=%d mag=%d warm=%d invalid=%d stored_enabled=%d stored_mode=%u runtime_enabled=%d wire_mag_status=%u online_restores=%u collection=%d batch=%d batch_rate=%u\n",
        CONFIG_CONNECTION_RAW_COLLECTION, CONFIG_SENSOR_USE_MAGNETOMETER, warm, invalid,
        retained->mag_enabled, retained->mag_online_calibration_mode, sensor_get_mag_enabled(), published_mag_status, model_restores,
        connection_get_data_collection(), connection_get_data_collection_batch(), connection_get_data_collection_batch_rate());
}

int main(void)
{
    restore_case(true, false);
    restore_case(true, true);
    restore_case(false, false);
    restore_case(false, true);
    printf("warm/cold retained restore raw=%d mag=%d: PASS\n", CONFIG_CONNECTION_RAW_COLLECTION, CONFIG_SENSOR_USE_MAGNETOMETER);
    return 0;
}
