#define CONFIG_SENSOR_USE_TCAL 0
#define CONFIG_SENSOR_TCAL_HEATED 0
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/sys/atomic.h>
#include "connection/connection.h"
#include "connection/raw_retx.h"
#include "sensor/calibration/calibration.h"
#include "sensor/raw_collection.h"
#include "../led_feedback_stub.h"
#define atomic_clear(value) atomic_set(value, 0)
#define BIT(value) (1U << (value))
#define LOG_INF(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define K_NO_WAIT 0
#define SENSOR_MAGNETOMETER_AXES_ALIGNMENT mx, my, mz
static struct { float gyroSensScale[3]; } saved = {{1.01f, 0.99f, 1.02f}}, *const retained = &saved;
static uint8_t tracker_id = 2;
static int64_t now_ms;
static int64_t k_uptime_get(void) { return now_ms; }
static void k_msleep(int value) { now_ms += value; }
static unsigned irq_lock(void) { return 0; }
static void irq_unlock(unsigned key) { (void)key; }
struct k_spinlock { bool held; };
typedef int k_spinlock_key_t;
static k_spinlock_key_t k_spin_lock(struct k_spinlock *lock) { assert(!lock->held); lock->held = true; return 0; }
static void k_spin_unlock(struct k_spinlock *lock, k_spinlock_key_t key) { (void)key; assert(lock->held); lock->held = false; }
struct k_msgq { char *bytes; size_t size, capacity, count, head; };
#define __noinit
#define __aligned(value) __attribute__((aligned(value)))
#define STRUCT_SECTION_ITERABLE(type, name) struct type name
#define Z_MSGQ_INITIALIZER(name, buffer, size, capacity) {buffer, size, capacity, 0, 0}
static void k_msgq_purge(struct k_msgq *queue) { queue->count = queue->head = 0; }
static int k_msgq_put(struct k_msgq *queue, const void *message, int wait)
{
    (void)wait;
    if (queue->count == queue->capacity) return -ENOMSG;
    size_t index = (queue->head + queue->count++) % queue->capacity;
    memcpy(queue->bytes + index * queue->size, message, queue->size);
    return 0;
}
static int k_msgq_get(struct k_msgq *queue, void *message, int wait)
{
    (void)wait;
    if (!queue->count) return -ENOMSG;
    memcpy(message, queue->bytes + queue->head * queue->size, queue->size);
    queue->head = (queue->head + 1) % queue->capacity;
    queue->count--;
    return 0;
}
static uint16_t sys_get_be16(const uint8_t *data) { return ((uint16_t)data[0] << 8) | data[1]; }
static void sys_put_be16(uint16_t value, uint8_t *data) { data[0] = value >> 8; data[1] = value; }
static bool sensor_diagnostics_maintenance_active(void) { return false; }
static void test_mode_set_target_tps(uint16_t rate) { (void)rate; }
void sensor_set_batch_collect(bool active, float rate) { (void)active; (void)rate; }
void connection_signal_wake(void) {}
static unsigned captures, mag_snapshots, sent_count;
void sensor_calibration_snapshot(sensor_imu_calibration_t *out)
{
    captures++;
    memset(out, 0, sizeof(*out));
    out->accel_matrix[1][0] = out->accel_matrix[2][1] = out->accel_matrix[3][2] = 1;
    out->gyro_bias[0] = 0.125f;
}
static const float magnetic_matrix[4][3] = {{0.25f, -0.5f, 0.75f}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
static void magneto_online_snapshot_BAinv(float out[4][3])
{
    mag_snapshots++;
    memcpy(out, magnetic_matrix, sizeof(magnetic_matrix));
}
static uint8_t sent[52];
static int esb_write(uint8_t *data, bool noack, size_t size)
{
    assert(size == 52 && !noack);
    sent_count++;
    memcpy(sent, data, size);
    return 0;
}
static float accel_actual_time = 0.02f, mag_actual_time = 0.04f;
static float gyro_actual_range = 2000, accel_actual_range = 16;
static uint8_t gyro_oversample_n = 1;
static int sensor_imu_id = 7, sensor_mag_id = 3;
static bool mag_available = true, mag_enabled = true;
#include "raw.inc"

static float packet_float(const uint8_t *packet, size_t offset)
{
    float value;
    memcpy(&value, packet + offset, sizeof(value));
    return value;
}

int main(void)
{
    gyro_actual_time = 0.01f;
    connection_set_data_collection(true);
    sensor_send_raw_metadata();
    assert(captures == 1 && mag_snapshots == CONFIG_SENSOR_USE_MAGNETOMETER);
    assert(raw_metadata_buf[0] == ESB_RAW_META_TYPE && raw_metadata_buf[1] == 2);
    assert(packet_float(raw_metadata_buf, 2) == 2000 && packet_float(raw_metadata_buf, 6) == 16);
    assert(packet_float(raw_metadata_buf, 18) == (CONFIG_SENSOR_USE_MAGNETOMETER ? 25.0f : 0.0f));
    assert(raw_metadata_buf[22] == 7 && raw_metadata_buf[23] == (CONFIG_SENSOR_USE_MAGNETOMETER ? 3 : 255));
    assert(connection_send_calibration_mask(RAW_META_MASK_MAG, 0));
    assert(sent_count == 1 && sent[0] == ESB_RAW_CAL_TYPE && sent[1] == 2 && sent[2] == RAW_CAL_SUB_MAG);
    const uint8_t zeros[48] = {0};
    assert(memcmp(sent + 3, CONFIG_SENSOR_USE_MAGNETOMETER ? (const void *)magnetic_matrix : zeros, 48) == 0);
    if (getenv("CAPABILITY_SMOKE")) printf("raw calibration raw=%d mag=%d packet_type=%u subtype=%u bias=%g,%g,%g\n",
        CONFIG_CONNECTION_RAW_COLLECTION, CONFIG_SENSOR_USE_MAGNETOMETER, sent[0], sent[2],
        packet_float(sent, 3), packet_float(sent, 7), packet_float(sent, 11));
    now_ms = 5000;
    raw_request_mask = raw_cal_pending_mask = 0;
    raw_metadata_pending = raw_cal_pending = false;
    raw_metadata_sent = true;
    struct raw_imu_sample sample = {.gyr_quat = {1, 0, 0, 0}, .accel = {1.25f, -2.5f, 0.5f}, .temp_c = 33.5f};
#if CONFIG_SENSOR_USE_MAGNETOMETER
    const float field[3] = {0.25f, -0.5f, 0.75f};
    connection_queue_raw_mag(field);
#endif
    connection_queue_raw_sample(&sample);
    assert(connection_process_raw_data());
    assert(sent_count == 2 && sent[0] == ESB_RAW_IMU_QUAT_TYPE && sent[1] == 2);
    assert(sys_get_be16(sent + 2) == 0);
    assert(memcmp(sent + 4, sample.gyr_quat, 16) == 0 && memcmp(sent + 20, sample.accel, 12) == 0);
#if CONFIG_SENSOR_USE_MAGNETOMETER
    assert(memcmp(sent + 32, field, 12) == 0 && sent[44] == 1);
#else
    assert(memcmp(sent + 32, zeros, 12) == 0 && sent[44] == 0);
#endif
    assert(packet_float(sent, 45) == 33.5f && sent[49] == 0 && sent[50] == 0 && sent[51] == 0);
    if (getenv("CAPABILITY_SMOKE")) printf("raw sample raw=%d mag=%d packet_type=%u sequence=%u mag_values=%g,%g,%g sample_mag_flags=%u temp=%g\n",
        CONFIG_CONNECTION_RAW_COLLECTION, CONFIG_SENSOR_USE_MAGNETOMETER, sent[0], sys_get_be16(sent + 2),
        packet_float(sent, 32), packet_float(sent, 36), packet_float(sent, 40), sent[44], packet_float(sent, 45));
    connection_queue_raw_sample(&sample);
    assert(connection_process_raw_data());
    assert(sent_count == 3 && sys_get_be16(sent + 2) == 1);
    assert(memcmp(sent + 32, zeros, 12) == 0 && sent[44] == 0);
    if (getenv("CAPABILITY_SMOKE")) printf("raw packet raw=%d mag=%d metadata_mag_id=%u metadata_mag_hz=%g sample_sequence=%u sample_mag_flags=%u temp=%g\n",
        CONFIG_CONNECTION_RAW_COLLECTION, CONFIG_SENSOR_USE_MAGNETOMETER, raw_metadata_buf[23],
        packet_float(raw_metadata_buf, 18), sys_get_be16(sent + 2), sent[44], packet_float(sent, 45));
    printf("raw wire metadata/calibration/samples mag=%d: PASS\n", CONFIG_SENSOR_USE_MAGNETOMETER);
    return 0;
}
