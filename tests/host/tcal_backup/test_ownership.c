/* Reuse command/export/seed helpers without running mocked-ownership suite. */
#define main backup_suite_main
#include "test_backup.c"
#undef main
static struct retained_fixture committed;
static unsigned committed_writes;
static void import_while_saver_waits(void) {
    assert(!warm_transaction && !lock_depth);
    replay(saved, true);
    assert(strstr(output, "saved successfully"));
    committed = *retained;
    committed_writes = persist_calls;
    /* Simulate the sensor owner consuming the epoch before the old publisher
     * wakes. Clearing a boolean reset flag here used to resurrect stale data. */
    tcal_accum_apply_reset();
}

static int reset_action;
static bool expect_invalidation;
static void clear_while_saver_waits(void) {
    uint32_t before = (uint32_t)atomic_get(&tcal_accum_reset_generation);
    int result;
    assert(tcal_accum.active && tcal_accum.sample_count == 42);
    switch (reset_action) {
    case 0: result = sensor_tcal_clear(); break;
    case 1: result = sensor_tcal_remove_point(0); break;
    case 2: result = sys_clear(); break;
    case 3: result = sensor_tcal_remove_point(1); break;
    default: assert(!"unknown reset action");
    }
    if (fail_id >= 0 || flush_error || nvs_clear_error) assert(result == -EIO);
    else assert(result == 0);
    assert((uint32_t)atomic_get(&tcal_accum_reset_generation) == before + expect_invalidation);
    assert(tcal_accum.active && tcal_accum.sample_count == 42);
    committed = *retained;
    committed_writes = persist_calls;
    tcal_accum_apply_reset();
    assert(tcal_accum.active != expect_invalidation);
    assert(tcal_accum.sample_count == (expect_invalidation ? 0 : 42));
}

static void test_clear_while_saver_waits(int action, bool invalidate, bool storage_error) {
    seed(1, true);
    tcal_accum_apply_reset();
    tcal_accum.active = true;
    tcal_accum.sample_count = 42;
    reset_action = action;
    expect_invalidation = invalidate;
    fail_id = action == 0 && storage_error ? MAIN_GYRO_TCAL_POINTS_ID : -1;
    flush_error = action == 1 && storage_error ? -EIO : 0;
    nvs_clear_error = action == 2 && storage_error ? -EIO : 0;
    const float old_bias[3] = {7, 8, 9};
    uint32_t stale = (uint32_t)atomic_get(&tcal_accum_reset_generation);
    before_storage_acquire = clear_while_saver_waits;
    tcal_save_point(30, old_bias, 25, stale);
    if (invalidate) {
        unchanged(&committed, committed_writes);
        assert(retained->tempCalPoints[30].temp == 0);
    } else {
        assert(retained->tempCalPoints[30].temp == 25);
        assert(!memcmp(retained->tempCalPoints[30].bias, old_bias, sizeof(old_bias)));
    }
    assert(!warm_transaction && !lock_depth);
    fail_id = -1;
    flush_error = nvs_clear_error = 0;
}

/* Feed a genuine qualified ordinary window through the production collector.
 * Storage is the modeled leaf; pending creation, freeze and retry are real. */
static void qualify_contended_point(void) {
    const float bias[3] = {2.0f, 4.0f, 6.0f};
    seed(0, true);
    tcal_accum_request_reset();
    tcal_accum_apply_reset();
    atomic_set(&tcal_auto_calibration_enabled, 1);
    tcal_direction_ref_temp = NAN;
    tcal_current_direction = TCAL_DIR_UNKNOWN;
    storage_contended = true;
    unsigned attempts = storage_try_calls;
    unsigned marks = warm_marks;
    for (int i = 0; i <= 2500; i++) {
        now_ms = (int64_t)i * 10;
        sensor_tcal_feed_continuous_sample(bias, 25.0f);
    }
    assert(storage_try_calls == attempts + 1);
    assert(tcal_pending_point.valid && !tcal_accum.active);
    assert(tcal_pending_point.temperature == 25.0f);
    assert(!memcmp(tcal_pending_point.bias, bias, sizeof(bias)));
    assert(retained->tempCalState.count == 0 && warm_marks == marks);
}

static void test_nonheated_pending(void) {
    const float other_bias[3] = {10.0f, 20.0f, 30.0f};
    for (int motion_retry = 0; motion_retry < 2; motion_retry++) {
        qualify_contended_point();
        struct retained_fixture before = *retained;
        unsigned writes = persist_calls, marks = warm_marks;
        unsigned acquisitions = storage_acquisitions;
        unsigned attempts = storage_try_calls;
        for (int i = 0; i < 3; i++) {
            sensor_tcal_feed_continuous_sample(other_bias, 30.0f);
            sensor_tcal_continuous_motion_detected();
        }
        assert(storage_try_calls == attempts + 6);
        assert(tcal_pending_point.valid && !tcal_accum.active);
        assert(tcal_accum.sample_count == 0 && tcal_accum.observed_count == 0);
        unchanged(&before, writes);
        assert(warm_marks == marks && storage_acquisitions == acquisitions);
        storage_contended = false;
        if (motion_retry) sensor_tcal_continuous_motion_detected();
        else sensor_tcal_feed_continuous_sample(other_bias, 30.0f);
        assert(!tcal_pending_point.valid);
        assert(storage_acquisitions == acquisitions + 1);
        assert(retained->tempCalState.count == 1);
        assert(retained->tempCalPoints[30].temp == 25.0f);
        const float expected[3] = {2.0f, 4.0f, 6.0f};
        assert(!memcmp(retained->tempCalPoints[30].bias, expected, sizeof(expected)));
        assert(retained->tempCalPoints[40].temp == 0.0f);
        assert(warm_marks == marks + 4);
        before = *retained; writes = persist_calls; marks = warm_marks;
        sensor_tcal_continuous_motion_detected();
        sensor_tcal_continuous_motion_detected();
        unchanged(&before, writes);
        assert(warm_marks == marks && storage_acquisitions == acquisitions + 1);
        assert(!warm_transaction && !lock_depth);
    }
    /* Identical qualified result must be consumed without model invalidation,
     * storage marks or a second acquisition on the next motion notification. */
    qualify_contended_point();
    retained->tempCalPoints[30].temp = 25.0f;
    memcpy(retained->tempCalPoints[30].bias, tcal_pending_point.bias,
           sizeof(tcal_pending_point.bias));
    retained->tempCalState.count = 1;
    retained->gyroTemp = 25.0f;
    struct retained_fixture before = *retained;
    unsigned writes = persist_calls, marks = warm_marks;
    unsigned reference = reference_generation, acquisitions = storage_acquisitions;
    storage_contended = false;
    sensor_tcal_continuous_motion_detected();
    assert(!tcal_pending_point.valid);
    unchanged(&before, writes);
    assert(warm_marks == marks && reference_generation == reference);
    assert(storage_acquisitions == acquisitions + 1);
    sensor_tcal_continuous_motion_detected();
    assert(storage_acquisitions == acquisitions + 1);
    assert(!warm_transaction && !lock_depth);
    /* Check the unchanged branch's explicit consumed result, not only the
     * caller's != -EAGAIN decision (which could hide an indeterminate value). */
    assert(tcal_save_point(30, before.tempCalPoints[30].bias, 25.0f,
                          (uint32_t)atomic_get(&tcal_accum_reset_generation)) == 0);

    /* Cancellation works both before and after owner-consumption of reset. */
    for (int consume_reset = 0; consume_reset < 2; consume_reset++) {
        qualify_contended_point();
        before = *retained; writes = persist_calls; marks = warm_marks;
        acquisitions = storage_acquisitions;
        tcal_accum_request_reset();
        if (consume_reset) tcal_accum_apply_reset();
        storage_contended = false;
        sensor_tcal_continuous_motion_detected();
        assert(!tcal_pending_point.valid && !tcal_accum.active);
        unchanged(&before, writes);
        assert(warm_marks == marks && storage_acquisitions == acquisitions);
        assert(!warm_transaction && !lock_depth);
    }
}

int main(void) {
    assert(console_serial_start() == 0);
    seed(4, true); export_model();
    seed(1, false);
    const float old_bias[3] = {7, 8, 9};
    uint32_t stale = (uint32_t)atomic_get(&tcal_accum_reset_generation);
    before_storage_acquire = import_while_saver_waits;
    tcal_save_point(30, old_bias, 25, stale);
    unchanged(&committed, committed_writes);
    assert(!warm_transaction && !lock_depth);
    /* Exact current epoch may publish a subsequent legitimate measurement. */
    unsigned marks = warm_marks;
    tcal_save_point(30, old_bias, 25, (uint32_t)atomic_get(&tcal_accum_reset_generation));
    assert(retained->tempCalPoints[30].temp == 25);
    assert(!memcmp(retained->tempCalPoints[30].bias, old_bias, sizeof(old_bias)));
    assert(warm_marks > marks && !warm_transaction && !lock_depth);
    test_clear_while_saver_waits(0, true, false);
    test_clear_while_saver_waits(0, true, true);
    test_clear_while_saver_waits(1, true, false);
    test_clear_while_saver_waits(1, true, true);
    test_clear_while_saver_waits(2, true, false);
    test_clear_while_saver_waits(2, false, true);
    test_clear_while_saver_waits(3, false, false);
    /* A maintenance owner blocks both automatic saver and normal request clear.
     * Nested begin fails and must not relinquish the outer owner's reservation. */
    assert(sensor_calibration_maintenance_begin() == 0);
    struct retained_fixture before = *retained;
    unsigned writes = persist_calls; marks = warm_marks;
    assert(sensor_calibration_maintenance_begin() == -EBUSY);
    assert(sensor_calibration_request(CAL_REQUEST_QUERY, CAL_REQUEST_AUTO_SILENT) == CAL_REQUEST_MAINTENANCE);
    assert(sensor_calibration_request(CAL_REQUEST_CLEAR, CAL_REQUEST_USER) == -EBUSY);
    assert(sensor_calibration_request(CAL_REQUEST_IMU, CAL_REQUEST_USER) != 0);
    tcal_save_point(31, old_bias, 25.5f, (uint32_t)atomic_get(&tcal_accum_reset_generation));
    unchanged(&before, writes); assert(warm_marks == marks);
    clear_output(); replay(saved, true); unchanged(&before, writes);
    assert(strstr(output, "busy"));
    assert(sensor_calibration_request(CAL_REQUEST_QUERY, CAL_REQUEST_AUTO_SILENT) == CAL_REQUEST_MAINTENANCE);
    sensor_calibration_maintenance_end();
    assert(sensor_calibration_request(CAL_REQUEST_QUERY, CAL_REQUEST_AUTO_SILENT) == 0);
    /* Ordinary pending calibration and magnetometer collection are also busy. */
    assert(sensor_calibration_request(CAL_REQUEST_IMU, CAL_REQUEST_AUTO_SILENT) == 0);
    assert(sensor_calibration_maintenance_begin() == -EBUSY);
    assert(sensor_calibration_request(CAL_REQUEST_CLEAR, CAL_REQUEST_AUTO_SILENT) == 0);
    magneto_progress = 0x80; assert(sensor_calibration_maintenance_begin() == -EBUSY); magneto_progress = 0;
    clear_output(); replay(saved, true); assert(strstr(output, "saved successfully"));
    test_nonheated_pending();
    assert(!lock_depth && !warm_transaction);
    puts("real nonheated ownership/saver: stale epochs, busy admission, qualified pending freeze, sample/motion exactly-once retry, identical consumption and reset cancellation OK");
    return 0;
}
