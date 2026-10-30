#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <zephyr/sys/atomic.h>
#include "connection/connection.h"
#include "sensor/raw_collection.h"
#include "../led_feedback_stub.h"
static bool sensor_diagnostics_maintenance_active(void) { return false; }
#define atomic_clear(value) atomic_set(value, 0)
#define LOG_INF(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define DC_BATCH_FUSION_TPS 10U
#define RAW_META_MASK_ALL 0x3f
#define ESB_RAW_META_TYPE 0x12
#define RAW_PACKET_SIZE 52
struct k_spinlock { bool locked; };
typedef int k_spinlock_key_t;
static int k_spin_lock(struct k_spinlock *lock)
{
    while (__atomic_test_and_set(&lock->locked, __ATOMIC_ACQUIRE)) {}
    return 0;
}
static void k_spin_unlock(struct k_spinlock *lock, int key)
{
    (void)key;
    assert(__atomic_load_n(&lock->locked, __ATOMIC_RELAXED));
    __atomic_clear(&lock->locked, __ATOMIC_RELEASE);
}
static unsigned irq_lock(void) { return 0; }
static void irq_unlock(unsigned key) {}
static int64_t now_ms;
static int64_t k_uptime_get(void) { return now_ms; }
static atomic_t data_collection_active, data_collection_batch_active, raw_snapshot_ready, raw_collection_session;
static uint16_t data_collection_batch_rate_hz, raw_sequence;
static uint8_t tracker_id, raw_request_mask, raw_request_chunk, raw_cal_pending_mask;
static bool raw_metadata_sent, raw_metadata_pending, raw_cal_pending, raw_request_token_valid, latest_mag_valid;
static int64_t raw_meta_cal_last_ms, raw_collection_start_ms;
static struct k_spinlock raw_request_lock, latest_mag_lock;
static uint8_t raw_metadata_buf[RAW_PACKET_SIZE], raw_ring_valid_bits[16];
static int raw_retx, raw_imu_msgq;
static struct raw_imu_sample samples[16];
static unsigned sample_count, calibration_captures;
static void raw_retx_reset(int *requests) { *requests = 0; }
static void k_msgq_purge(int *queue) { sample_count = 0; }
static void test_mode_set_target_tps(unsigned tps) {}
struct raw_cal_snapshot { unsigned generation; };
static struct raw_cal_snapshot raw_cal_snapshot;
static void (*capture_hook)(void);
static void connection_capture_calibration_snapshot(struct raw_cal_snapshot *snapshot)
{
    assert(!__atomic_load_n(&raw_request_lock.locked, __ATOMIC_RELAXED));
    snapshot->generation = ++calibration_captures;
    if (capture_hook) capture_hook();
}
void connection_signal_wake(void) {}
void connection_queue_raw_sample(const struct raw_imu_sample *sample)
{
    assert(sample_count < 16);
    samples[sample_count++] = *sample;
}
static void (*batch_publication_hook)(void);
static atomic_val_t publication_atomic_set(atomic_t *target, atomic_val_t value)
{
    atomic_val_t previous = atomic_set(target, value);
    if (target == &data_collection_batch_active && value && batch_publication_hook) {
        void (*hook)(void) = batch_publication_hook;
        batch_publication_hook = NULL;
        hook();
    }
    return previous;
}
#define atomic_set(target, value) publication_atomic_set(target, value)
#include "collection.inc"
#include "sensor/raw_collection.c"

static const struct sensor_raw_collection_config config = {
    .gyro_period = 0.1f, .accel_period = 0.2f, .mag_period = 0.5f,
    .gyro_range = 2000, .accel_range = 16, .gyro_oversample_n = 1,
    .imu_id = 7, .mag_id = 2, .mag_active = true,
};
static void frame_with_config(const struct sensor_raw_collection_config *settings, bool restart_expected)
{
    bool active;
    bool restart = sensor_raw_collection_begin_frame(&active);
    assert(active && restart == restart_expected);
    if (restart) sensor_raw_collection_send_metadata(settings);
}
static void frame(bool restart_expected) { frame_with_config(&config, restart_expected); }
static void sample(void)
{
    const float accel[3] = {1, 2, 3}, gyro[3] = {0, 0, 90};
    sensor_raw_collection_on_sample(accel, gyro, 25, true);
}
static void near(float actual, float expected) { assert(fabsf(actual - expected) < 0.00001f); }
static void restart_during_capture(void)
{
    capture_hook = NULL;
    assert(batch_request(false, 0) == 0);
    assert(batch_request(true, 2) == 0);
}
static void test_single_restart_between_frames(void)
{
    struct sensor_raw_collection_config settings = config;
    settings.gyro_oversample_n = 3;
    connection_set_data_collection(true);
    frame_with_config(&settings, true);
    sample();
    assert(sample_count == 0);
    unsigned captures_before = calibration_captures;
    uint32_t session_before = connection_get_raw_collection_session();
    connection_set_data_collection(false);
    connection_set_data_collection(true);
    assert(connection_get_raw_collection_session() != session_before);
    assert(!atomic_get(&raw_snapshot_ready));
    frame_with_config(&settings, true);
    assert(atomic_get(&raw_snapshot_ready));
    assert(calibration_captures == captures_before + 1);
    connection_set_data_collection(true);
    frame_with_config(&settings, false);
    assert(calibration_captures == captures_before + 1);

    const float zero[3] = {0}, gyro[3] = {0, 0, 90};
    sensor_raw_collection_on_sample(zero, gyro, 25, true);
    assert(sample_count == 0);
    sensor_raw_collection_on_sample(zero, gyro, 25, true);
    assert(sample_count == 1);
    near(samples[0].gyr_quat[0], cosf(0.1570796327f));
    near(samples[0].gyr_quat[3], sinf(0.1570796327f));
    near(samples[0].accel[0], 1);
    near(samples[0].accel[1], 2);
    near(samples[0].accel[2], 3);

    /* Raw session zero is a valid identity after wrap, not inactivity. */
    atomic_set(&raw_collection_session, (atomic_val_t)UINT32_MAX);
    connection_set_data_collection(false);
    connection_set_data_collection(true);
    assert(connection_get_raw_collection_session() == 0);
    frame(true);
    sample();
    assert(sample_count == 1);
    near(samples[0].gyr_quat[0], cosf(0.0785398163f));
    near(samples[0].gyr_quat[3], sinf(0.0785398163f));
}

static uint32_t observed_publication_session;
static void observe_before_batch_settings(void)
{
    bool active;
    assert(sensor_raw_collection_begin_frame(&active));
    assert(active);
    observed_publication_session = connection_get_raw_collection_session();
}
static void test_batch_publication_after_session_observation(void)
{
    /* Observe the session before sensor batch settings are published. The
     * later generation must still request a frame restart for those settings. */
    batch_publication_hook = observe_before_batch_settings;
    assert(batch_request(true, 5) == 0);
    assert(!batch_publication_hook);
    assert(connection_get_raw_collection_session() == observed_publication_session);
    frame(true);
    float rate;
    memcpy(&rate, raw_metadata_buf + 10, sizeof(rate));
    near(rate, 5);
    sample(); sample();
    assert(sample_count == 1);
    near(samples[0].gyr_quat[0], cosf(0.1570796327f));
    near(samples[0].gyr_quat[3], sinf(0.1570796327f));
    assert(batch_request(true, 5) == 0);
    frame(false);

    connection_set_data_collection(true);
    frame(true);
    memcpy(&rate, raw_metadata_buf + 10, sizeof(rate));
    near(rate, 10);
    assert(!connection_get_data_collection_batch());
    sample();
    assert(sample_count == 1);
    near(samples[0].gyr_quat[0], cosf(0.0785398163f));
    near(samples[0].gyr_quat[3], sinf(0.0785398163f));
}

int main(void)
{
    assert(batch_request(true, 5) == 0);
    frame(true);
    float rate;
    memcpy(&rate, raw_metadata_buf + 10, sizeof(rate));
    near(rate, 5);
    sample(); sample(); sample();
    assert(sample_count == 1);
    struct raw_imu_sample first = samples[0];
    uint8_t metadata[RAW_PACKET_SIZE];
    memcpy(metadata, raw_metadata_buf, sizeof(metadata));

    assert(batch_request(true, 5) == 0);
    frame(false);
    assert(batch_request(true, 2) == -EBUSY);
    assert(connection_get_data_collection_batch_rate() == 5);
    frame(false);
    sample();
    assert(sample_count == 2);
    assert(memcmp(&samples[0], &first, sizeof(first)) == 0);
    near(samples[1].gyr_quat[0], cosf(0.3141592654f));
    near(samples[1].gyr_quat[3], sinf(0.3141592654f));
    assert(memcmp(metadata, raw_metadata_buf, sizeof(metadata)) == 0);
    assert(calibration_captures == 1);

    /* Explicit stop/start establishes a fresh snapshot, queue and integrator. */
    assert(batch_request(false, 0) == 0);
    assert(batch_request(true, 2) == 0);
    frame(true);
    memcpy(&rate, raw_metadata_buf + 10, sizeof(rate));
    near(rate, 2);
    assert(sample_count == 0 && calibration_captures == 2);
    for (int i = 0; i < 4; i++) sample();
    assert(sample_count == 0);
    sample();
    assert(sample_count == 1);
    near(samples[0].gyr_quat[0], cosf(0.3926990817f));
    near(samples[0].gyr_quat[3], sinf(0.3926990817f));
    /* The mutex wait may allow stop/start. An old producer must neither
     * mark the new session ready nor overwrite its immutable calibration. */
    assert(batch_request(false, 0) == 0);
    assert(batch_request(true, 2) == 0);
    unsigned published = raw_cal_snapshot.generation;
    capture_hook = restart_during_capture;
    connection_send_raw_metadata(1, 2, 3, 4, 5, 6, 7, 8, 9);
    assert(!atomic_get(&raw_snapshot_ready));
    assert(raw_cal_snapshot.generation == published);
    connection_send_raw_metadata(1, 2, 3, 4, 5, 6, 7, 8, 9);
    assert(atomic_get(&raw_snapshot_ready));
    assert(raw_cal_snapshot.generation == published + 2);
    test_single_restart_between_frames();
    test_batch_publication_after_session_observation();
    puts("collection: rejected/same-rate starts preserve metadata, queued data and partial integration");
    return 0;
}
