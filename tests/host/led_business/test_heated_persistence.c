#include <errno.h>
#include "../led_sync/test_runtime.h"

#define CONFIG_SENSOR_USE_TCAL 1
#define MAIN_GYRO_TCAL_POINTS_ID 1
#define MAIN_GYRO_TCAL_COEFFS_ID 2
#define MAIN_GYRO_TCAL_STATE_ID 3
#define MAIN_GYRO_TEMP_ID 4
#define LOG_INF(...) ((void)0)
#define HEAT_OFF_FAILED 3
#define TCAL_HEATED_STOP_USER 1

struct k_mutex { unsigned depth; };
static struct k_mutex sys_storage_lock;
static void k_mutex_lock(struct k_mutex *mutex, k_timeout_t timeout)
{
    (void)timeout;
    ++mutex->depth;
}
static void k_mutex_unlock(struct k_mutex *mutex)
{
    assert(mutex->depth);
    --mutex->depth;
}
static int fs;
static bool nvs_ready = true;
static uint16_t failed_id;
static uint32_t durable[32];
static unsigned writes;
static bool sys_nvs_init(void) { return nvs_ready; }
static int nvs_write(int *storage, uint16_t id, const void *data, size_t len)
{
    (void)storage;
    assert(sys_storage_lock.depth && id < 32 && len == sizeof(uint32_t));
    if (id == failed_id) return -EIO;
    memcpy(&durable[id], data, len);
    ++writes;
    return (int)len;
}
static void retained_update(void) { assert(sys_storage_lock.depth); }

static struct {
    struct led_token feedback;
    bool warm_pending;
    uint8_t written_mask;
    uint32_t published_generation;
    int reason, state;
} heat;
static uint32_t model_generation;
static bool heat_locked, model_locked;
static void sensor_tcal_heated_lock(void)
{
    assert(!sys_storage_lock.depth && !heat_locked);
    heat_locked = true;
}
static void sensor_tcal_heated_unlock(void) { assert(heat_locked); heat_locked = false; }
static void sensor_tcal_lock(void) { assert(heat_locked && !model_locked); model_locked = true; }
static void sensor_tcal_unlock(void) { assert(model_locked); model_locked = false; }
static uint32_t sensor_tcal_model_generation(void) { assert(model_locked); return model_generation; }
void sensor_tcal_feedback_persisted(uint32_t identity, uint8_t written_mask, int result);
#include "production.inc"

static uint32_t points, coefficients, state, gyro_temp;
static void reset(void)
{
    host_reset(LED_CAP_RGB_PWM);
    memset(warm_dirty, 0, sizeof(warm_dirty));
    memset(warm_tcal_receipts, 0, sizeof(warm_tcal_receipts));
    memset(durable, 0, sizeof(durable));
    warm_dirty_count = warm_tcal_receipt_count = 0;
    warm_tcal_armed = warm_transaction_depth = 0;
    memset(&heat, 0, sizeof(heat));
    sys_storage_lock.depth = 0;
    nvs_ready = true; failed_id = 0; writes = 0;
    model_generation = 1;
    points = 11; coefficients = 22; state = 33;
    gyro_temp = 45;
    heat.feedback = host_begin(LED_OWNER_TCAL, LED_PROCESSING);
    heat.warm_pending = true;
    heat.published_generation = model_generation;
    host_step(0);
}
static void mark(void)
{
    sys_warm_transaction_begin();
    sys_warm_feedback_arm(heat.feedback.session);
    sys_warm_transaction_mark(MAIN_GYRO_TEMP_ID, &gyro_temp, sizeof(gyro_temp));
    sys_warm_transaction_mark(MAIN_GYRO_TCAL_STATE_ID, &state, sizeof(state));
    sys_warm_transaction_mark(MAIN_GYRO_TCAL_POINTS_ID, &points, sizeof(points));
    sys_warm_transaction_mark(MAIN_GYRO_TCAL_COEFFS_ID, &coefficients, sizeof(coefficients));
    sys_warm_transaction_end(true);
}
static void outcome(enum led_semantic terminal)
{
    assert(!heat.warm_pending && engine.owners[LED_OWNER_TCAL].terminal == terminal);
    host_step(host_now_ms + 300);
    assert(engine.winner.semantic == terminal);
    if (terminal == LED_SUCCESS) assert(host_role == LED_ROLE_POSITIVE);
    else assert(host_role != LED_ROLE_POSITIVE);
}
static void coalesce_temperature_last(void)
{
    /* Existing background model slots survive an eager gyroTemp write.
     * Finalization coalesces those slots and appends gyroTemp last. */
    sys_write_warm(MAIN_GYRO_TCAL_POINTS_ID, &points, &points, sizeof(points));
    sys_write_warm(MAIN_GYRO_TCAL_COEFFS_ID, &coefficients, &coefficients, sizeof(coefficients));
    sys_write_warm(MAIN_GYRO_TCAL_STATE_ID, &state, &state, sizeof(state));
    mark();
    assert(warm_dirty_count == 4);
    assert(warm_dirty[3].id == MAIN_GYRO_TEMP_ID);
}
static void individual_receipts(void)
{
    const uint16_t ids[] = {MAIN_GYRO_TCAL_POINTS_ID, MAIN_GYRO_TCAL_COEFFS_ID,
                           MAIN_GYRO_TCAL_STATE_ID, MAIN_GYRO_TEMP_ID};
    uint32_t *values[] = {&points, &coefficients, &state, &gyro_temp};
    for (unsigned i = 0; i < 4; ++i) {
        sys_warm_transaction_begin();
        sys_warm_feedback_arm(heat.feedback.session);
        sys_warm_transaction_mark(ids[i], values[i], sizeof(*values[i]));
        sys_warm_transaction_end(false);
        assert(sys_flush_warm() == 0);
        assert(writes == i + 1);
        if (i < 3) {
            assert(heat.warm_pending && heat.feedback.session != 0);
            assert(engine.owners[LED_OWNER_TCAL].terminal == LED_NONE);
        }
    }
    outcome(LED_SUCCESS);
}
int main(void)
{
    /* Run the regression first so archived production fails at the false
     * success boundary, not at an incidental queue-size assertion. */
    reset(); coalesce_temperature_last(); failed_id = MAIN_GYRO_TEMP_ID;
    assert(sys_flush_warm() == -EIO && writes == 3);
    assert(durable[1] == points && durable[2] == coefficients && durable[3] == state);
    assert(durable[MAIN_GYRO_TEMP_ID] == 0 && warm_dirty_count == 1);
    outcome(LED_APPLIED_NOT_SAVED);
    failed_id = 0;
    assert(sys_flush_warm() == 0 && durable[MAIN_GYRO_TEMP_ID] == gyro_temp);
    assert(engine.owners[LED_OWNER_TCAL].terminal == LED_APPLIED_NOT_SAVED);

    reset(); individual_receipts();
    reset(); coalesce_temperature_last();
    assert(sys_flush_warm() == 0 && writes == 4);
    outcome(LED_SUCCESS);

    reset(); mark();
    assert(heat.warm_pending && writes == 0 && engine.owners[LED_OWNER_TCAL].terminal == LED_NONE);
    assert(sys_flush_warm() == 0);
    assert(writes == 4 && durable[1] == points && durable[2] == coefficients && durable[3] == state);
    assert(durable[MAIN_GYRO_TEMP_ID] == gyro_temp);
    outcome(LED_SUCCESS);

    reset(); mark(); failed_id = MAIN_GYRO_TCAL_COEFFS_ID;
    assert(sys_flush_warm() == -EIO);
    assert(durable[1] == points && durable[2] == 0 && durable[3] == state && warm_dirty_count == 1);
    assert(durable[MAIN_GYRO_TEMP_ID] == gyro_temp);
    outcome(LED_APPLIED_NOT_SAVED);
    failed_id = 0;
    assert(sys_flush_warm() == 0 && durable[2] == coefficients && durable[3] == state);
    assert(engine.owners[LED_OWNER_TCAL].terminal == LED_APPLIED_NOT_SAVED);

    reset(); mark(); nvs_ready = false;
    assert(sys_flush_warm() == -EIO && writes == 0 && warm_dirty_count == 4);
    outcome(LED_APPLIED_NOT_SAVED);

    reset(); mark(); ++model_generation;
    assert(sys_flush_warm() == 0);
    outcome(LED_PARTIAL);

    reset(); mark();
    struct led_token old = heat.feedback;
    heat.feedback = host_begin(LED_OWNER_TCAL, LED_PROCESSING);
    heat.written_mask = 0;
    ++model_generation; heat.published_generation = model_generation;
    points = 44; coefficients = 55; state = 66;
    gyro_temp = 50;
    mark();
    assert(heat.warm_pending && heat.feedback.session != old.session);
    assert(sys_flush_warm() == 0);
    assert(durable[1] == 44 && durable[2] == 55 && durable[3] == 66);
    assert(durable[MAIN_GYRO_TEMP_ID] == 50 && writes == 4);
    outcome(LED_SUCCESS);

    reset(); mark();
    sys_warm_transaction_begin();
    sys_warm_transaction_begin();
    assert(sys_flush_warm() == 0 && heat.warm_pending && writes == 4);
    sys_warm_transaction_end(false);
    assert(heat.warm_pending && sys_storage_lock.depth == 1);
    sys_warm_transaction_end(false);
    outcome(LED_SUCCESS);

    reset(); mark();
    uint32_t replacement = 99;
    sys_write_warm(MAIN_GYRO_TCAL_COEFFS_ID, &coefficients, &replacement, sizeof(replacement));
    outcome(LED_APPLIED_NOT_SAVED);
    assert(sys_flush_warm() == 0 && durable[2] == 99);
    assert(engine.owners[LED_OWNER_TCAL].terminal == LED_APPLIED_NOT_SAVED);

    reset(); mark();
    sys_write_warm(MAIN_GYRO_TEMP_ID, &gyro_temp, &replacement, sizeof(replacement));
    outcome(LED_APPLIED_NOT_SAVED);
    assert(sys_flush_warm() == 0 && durable[MAIN_GYRO_TEMP_ID] == replacement);
    assert(engine.owners[LED_OWNER_TCAL].terminal == LED_APPLIED_NOT_SAVED);

    reset();
    uint32_t other[8] = {1,2,3,4,5,6,7,8};
    for (unsigned i = 0; i < 8; ++i) sys_write_warm((uint16_t)(5 + i), &other[i], &other[i], sizeof(other[i]));
    failed_id = 5;
    sys_warm_transaction_begin();
    sys_warm_feedback_arm(heat.feedback.session);
    sys_warm_transaction_mark(MAIN_GYRO_TCAL_POINTS_ID, &points, sizeof(points));
    sys_warm_transaction_end(false);
    assert(warm_dirty_count == 8 && writes == 0);
    outcome(LED_APPLIED_NOT_SAVED);
    puts("heated durability: real storage and LED policy observe success only after all writes; failures, replaced epochs, superseded marks, nested flushes and full tables remain truthful");
    return 0;
}
