/* The included functions and state are extracted from the production sources by
 * run.py. Only kernel, NVS, sensor and fusion leaves are supplied here. */
#define _GNU_SOURCE
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "util.h"

#define K_FOREVER (-1)
#define K_NO_WAIT 0
#define K_MUTEX_DEFINE(name) static pthread_mutex_t name = PTHREAD_MUTEX_INITIALIZER
#define MAIN_GYRO_SENS_ID 1
#define LOG_ERR(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define CONFIG_SENSOR_SENS_REV 1

static pthread_mutex_t sys_storage_lock;
static pthread_mutex_t nvs_gate = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t nvs_changed = PTHREAD_COND_INITIALIZER;
static bool nvs_entered, nvs_release;
static int nvs_error, fs;
static struct {
    float gyroSensScale[3];
} retained_data = {{0.9375f, 1.0625f, 0.96875f}};
static typeof(retained_data) *const retained = &retained_data;
static unsigned imu_loads, retained_updates;

static int k_mutex_lock(pthread_mutex_t *mutex, int timeout)
{
    int error;
    if (timeout == K_NO_WAIT) {
        error = pthread_mutex_trylock(mutex);
        assert(error == 0 || error == EBUSY);
        return error ? -EBUSY : 0;
    }
    assert(timeout == K_FOREVER);
    assert(pthread_mutex_lock(mutex) == 0);
    return 0;
}
static int k_mutex_unlock(pthread_mutex_t *mutex)
{
    assert(pthread_mutex_unlock(mutex) == 0);
    return 0;
}
static bool sys_nvs_init(void) { return true; }
static void retained_update(void) { retained_updates++; }
static void warm_dirty_clear_id_locked(uint16_t id) { (void)id; }
static int nvs_write(void *unused, uint16_t id, const void *data, size_t size)
{
    (void)unused;
    assert(id == MAIN_GYRO_SENS_ID && data && size == 3 * sizeof(float));
    assert(pthread_mutex_lock(&nvs_gate) == 0);
    nvs_entered = true;
    assert(pthread_cond_broadcast(&nvs_changed) == 0);
    while (!nvs_release) assert(pthread_cond_wait(&nvs_changed, &nvs_gate) == 0);
    int result = nvs_error ? nvs_error : (int)size;
    assert(pthread_mutex_unlock(&nvs_gate) == 0);
    return result;
}
static void check_imu_ready(void);
static void sensor_calibration_imu_load(void) { check_imu_ready(); imu_loads++; }

struct led_token { unsigned session, request_id; };
enum {
    CAL_OUTCOME_CANCELLED, CAL_PHASE_APPLIED, CAL_REASON_REPLACED,
    CAL_REASON_RESET, LED_CANCELLED, LED_OWNER_SENS
};
static void cal_event_end(uint16_t op, uint8_t outcome, uint8_t phase, uint8_t reason)
{ (void)op; (void)outcome; (void)phase; (void)reason; }
static void sensor_calibration_result(struct led_token token, int result)
{ (void)token; (void)result; }
static void tracker_events_notify(void) {}
static int sensor_operation_result(int owner, int result, bool applied)
{ (void)owner; (void)applied; return result; }

static unsigned total_gyro_samples, history_discards, fusion_feeds, rebases;
static const float gyro_actual_time = 0.0025f;
static const float raw_sample[3] = {12.0f, -8.0f, 4.0f};
static const float raw_delta[3] = {0.5f, -0.25f, 0.125f};
static float received_g[3], received_delta[3];
static struct { bool frame_invalid; } sensor_motion_state;
static void vqf_discard_motion_history(void) { history_discards++; }
static void sensor_diagnostics_on_raw_gyro(float *g) { (void)g; }
static void sensor_diagnostics_on_cal_gyro(float *g) { (void)g; }
static bool sensor_calibration_process_gyro(float *g, float dt, float *delta)
{
    (void)g;
    assert(dt == gyro_actual_time);
    memcpy(delta, raw_delta, sizeof(raw_delta));
    return false;
}
static void update_gyro(float *g, float dt)
{
    assert(dt == gyro_actual_time);
    memcpy(received_g, g, sizeof(received_g));
    fusion_feeds++;
}
static void rebase_gyro_bias(const float *delta)
{
    memcpy(received_delta, delta, sizeof(received_delta));
    rebases++;
}
static void set_gyro_bias(float *bias) { (void)bias; assert(false); }
static const struct {
    void (*update_gyro)(float *, float);
    void (*rebase_gyro_bias)(const float *);
    void (*set_gyro_bias)(float *);
} backend = {update_gyro, rebase_gyro_bias, set_gyro_bias};
static const typeof(backend) *sensor_fusion = &backend;
static bool sensor_fusion_init = true;

#include "sensitivity_contention_sources.inc"

static void check_imu_ready(void)
{
    if (imu_loads == 0) {
        assert(sensitivity_cache_seeded);
        assert(TEST_CACHE_GENERATION == sensitivity_generation);
        for (int axis = 0; axis < 3; axis++)
            assert(sensitivity_cache.scale[axis] == retained->gyroSensScale[axis]);
    }
}

static const float degrees[3] = {12.0f, -6.0f, 3.0f};
struct writer_args { bool reset; int result; };
static void *write_scales(void *argument)
{
    struct writer_args *args = argument;
    args->result = args->reset ? sensor_calibration_reset_sensitivity() :
                                 sensor_calibration_set_sensitivity(degrees);
    return NULL;
}
static void feed_and_check(const float scale[3], unsigned expected_discards)
{
    int count = 0;
    float raw[3];
    memcpy(raw, raw_sample, sizeof(raw));
    unsigned before_feeds = fusion_feeds, before_rebases = rebases;
    feed_gyro_sample(raw, &count, true);
    assert(count == 1 && fusion_feeds == before_feeds + 1 && rebases == before_rebases + 1);
    assert(history_discards == expected_discards && !sensor_motion_state.frame_invalid);
    for (int axis = 0; axis < 3; axis++) {
        assert(fabsf(received_g[axis] - raw_sample[axis] * scale[axis]) < 1e-6f);
        assert(fabsf(received_delta[axis] - raw_delta[axis] * scale[axis]) < 1e-6f);
    }
}
static void exercise_contention(const float old_scale[3], const float new_scale[3],
                                bool reset, int write_error, bool adopt)
{
    struct writer_args args = {.reset = reset};
    pthread_t writer;
    uint32_t old_generation = sensitivity_generation;
    uint32_t old_cache_generation = TEST_CACHE_GENERATION;
    unsigned old_discards = history_discards;

    assert(pthread_mutex_lock(&nvs_gate) == 0);
    nvs_entered = nvs_release = false;
    nvs_error = write_error;
    assert(pthread_mutex_unlock(&nvs_gate) == 0);
    assert(pthread_create(&writer, NULL, write_scales, &args) == 0);
    assert(pthread_mutex_lock(&nvs_gate) == 0);
    while (!nvs_entered) assert(pthread_cond_wait(&nvs_changed, &nvs_gate) == 0);
    assert(pthread_mutex_unlock(&nvs_gate) == 0);

    /* sys_write already changed RAM, but is still in NVS under both locks.
     * Even the first owner sample must use the complete previous snapshot. */
    for (int axis = 0; axis < 3; axis++)
        assert(fabsf(retained->gyroSensScale[axis] - new_scale[axis]) < 1e-6f);
    for (int i = 0; i < 2; i++) {
        feed_and_check(old_scale, old_discards);
        assert(sensitivity_generation == old_generation);
        assert(TEST_CACHE_GENERATION == old_cache_generation);
    }

    assert(pthread_mutex_lock(&nvs_gate) == 0);
    assert(nvs_entered && !nvs_release);
    nvs_release = true;
    assert(pthread_cond_broadcast(&nvs_changed) == 0);
    assert(pthread_mutex_unlock(&nvs_gate) == 0);
    assert(pthread_join(writer, NULL) == 0);
    assert(args.result == write_error);
    assert(sensitivity_generation == old_generation + 1);
    assert(TEST_CACHE_GENERATION == old_cache_generation);
    assert(history_discards == old_discards);
    if (!adopt) return;
    feed_and_check(new_scale, old_discards + 1);
    assert(TEST_CACHE_GENERATION == sensitivity_generation);
    feed_and_check(new_scale, old_discards + 1);
}
int main(void)
{
    pthread_mutexattr_t storage_attr;
    assert(pthread_mutexattr_init(&storage_attr) == 0);
    assert(pthread_mutexattr_settype(&storage_attr, PTHREAD_MUTEX_RECURSIVE) == 0);
    assert(pthread_mutex_init(&sys_storage_lock, &storage_attr) == 0);
    assert(pthread_mutexattr_destroy(&storage_attr) == 0);

    float persisted[3];
    memcpy(persisted, retained->gyroSensScale, sizeof(persisted));
    sensor_calibration_read();
    assert(imu_loads == 1);
    assert(!memcmp(persisted, retained->gyroSensScale, sizeof(persisted)));
    float replacement[3];
    for (int axis = 0; axis < 3; axis++)
        replacement[axis] = 1.0f / (1.0f - degrees[axis] / 360.0f);
    const float unity[3] = {1.0f, 1.0f, 1.0f};
    exercise_contention(persisted, replacement, false, 0, true);
    exercise_contention(replacement, replacement, false, 0, true); /* Equal scale, new epoch. */
    exercise_contention(replacement, unity, true, -EIO, true); /* RAM survives NVS error. */

    /* A previous commit can be pending while a second writer holds the lock.
     * A failed snapshot must not receipt even that already-committed epoch. */
    exercise_contention(unity, replacement, false, 0, false);
    uint32_t pending_cache_generation = TEST_CACHE_GENERATION;
    assert(pending_cache_generation != sensitivity_generation);
    sensor_calibration_read(); /* A later read must not reseed the live owner. */
    assert(imu_loads == 2 && TEST_CACHE_GENERATION == pending_cache_generation);
    exercise_contention(unity, unity, true, 0, true);
    exercise_contention(unity, replacement, false, 0, true);
    unsigned old_discards = history_discards;

    /* Later validation heals only the bad axis and publishes a new epoch
     * without touching the sensor-owned cache from the load caller. */
    uint32_t before_heal = sensitivity_generation;
    uint32_t before_heal_cache = TEST_CACHE_GENERATION;
    unsigned before_heal_updates = retained_updates;
    retained->gyroSensScale[1] = NAN;
    sensor_calibration_read();
    assert(imu_loads == 3 && retained_updates == before_heal_updates + 1);
    assert(sensitivity_generation == before_heal + 1);
    assert(TEST_CACHE_GENERATION == before_heal_cache);
    float healed[3] = {replacement[0], 1.0f, replacement[2]};
    assert(!memcmp(retained->gyroSensScale, healed, sizeof(healed)));
    feed_and_check(healed, old_discards + 1);
    feed_and_check(healed, old_discards + 1);
    puts("sensitivity contention: persisted first feed, coherent g/delta, deferred epochs, equal replacement and failed reset");
    return 0;
}
