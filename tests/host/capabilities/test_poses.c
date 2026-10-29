#include <stdlib.h>
#include <stddef.h>
static unsigned allocations, releases;
void *k_malloc(size_t size)
{
    allocations++;
    return malloc(size);
}
void k_free(void *memory)
{
    releases++;
    free(memory);
}
static double expected_ata[100], expected_norm, expected_count;
static unsigned capture_reads, reject_attempt, rejected_pose, checked_fits;
#define magneto_current_calibration checked_calibration
#include "poses.inc"
#undef magneto_current_calibration
int magneto_current_calibration(float matrix[4][3], double *ata, double norm, double count);
int checked_calibration(float matrix[4][3], double *ata, double norm, double count)
{
    checked_fits++;
    assert(count == 9000 && expected_count == 9000);
    assert(norm == expected_norm);
    assert(memcmp(ata, expected_ata, sizeof(expected_ata)) == 0);
    return magneto_current_calibration(matrix, ata, norm, count);
}

int main(void)
{
    pose_index = 0;
    reads = 0;
    now = 0;
    pose_limit = 18;
    ends = 0;
    memset(mag_cal_workspace.ata, 0xa5, sizeof(mag_cal_workspace.ata));
    norm_sum = sample_count = 123;
    magneto_reset();
    assert(norm_sum == 0 && sample_count == 0);
    const unsigned char zero[sizeof(mag_cal_workspace.ata)] = {0};
    assert(memcmp(mag_cal_workspace.ata, zero, sizeof(zero)) == 0);
    float matrix[4][3];
    memset(matrix, 0x5a, sizeof(matrix));
    int captured = -1;
    assert(sensor_calibration_collect_accel_poses(matrix, &captured) == 0);
    assert(captured == 18 && pose_index == 18 && ends == 0);
    assert(allocations == 1 && releases == 1);
    assert(checked_fits == 1 && rejected_pose == 18 && motion_retries == 18);
    for (unsigned axis = 0; axis < 3; axis++) {
        assert(fabsf(matrix[0][axis]) < 0.002f);
        for (unsigned row = 0; row < 3; row++) {
            float expected = row == axis ? 1.0f : 0.0f;
            assert(fabsf(matrix[row + 1][axis] - expected) < 0.002f);
        }
    }
    assert(sample_count == 0 && norm_sum == 0);
    assert(memcmp(mag_cal_workspace.ata, zero, sizeof(zero)) == 0);
    if (getenv("CAPABILITY_SMOKE")) printf("accelerometer raw=%d mag=%d poses=%d bias=%g,%g,%g diagonal=%g,%g,%g allocations=%u frees=%u\n",
        CONFIG_CONNECTION_RAW_COLLECTION, CONFIG_SENSOR_USE_MAGNETOMETER, captured,
        matrix[0][0], matrix[0][1], matrix[0][2], matrix[1][0], matrix[2][1], matrix[3][2], allocations, releases);
    pose_index = reads = ends = capture_reads = reject_attempt = rejected_pose = 0;
    now = 0;
    pose_limit = 6;
    memset(expected_ata, 0, sizeof(expected_ata));
    expected_norm = expected_count = 0;
    assert(sensor_calibration_collect_accel_poses(matrix, &captured) == -3);
    assert(captured == 6 && pose_index == 6 && rejected_pose == 6);
    assert(sample_count == 3000 && expected_count == 3000);
    assert(norm_sum == expected_norm);
    assert(memcmp(mag_cal_workspace.ata, expected_ata, sizeof(expected_ata)) == 0);
    assert(checked_fits == 1 && allocations == 1 && releases == 1);
    printf("shared real accelerometer collector/solver raw=%d mag=%d: PASS\n", CONFIG_CONNECTION_RAW_COLLECTION, CONFIG_SENSOR_USE_MAGNETOMETER);
    return 0;
}
