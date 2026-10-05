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
    assert(!lock_depth && !warm_transaction);
    puts("real nonheated maintenance/request/saver: stale epoch after import, consumed reset, fresh save, nested ownership and busy admission OK");
    return 0;
}
