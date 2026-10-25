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
#include "poses.inc"

int main(void)
{
    pose_index = 0;
    reads = 0;
    now = 0;
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
    for (unsigned axis = 0; axis < 3; axis++) {
        assert(fabsf(matrix[0][axis]) < 0.0001f);
        for (unsigned row = 0; row < 3; row++) {
            float expected = row == axis ? 1.0f : 0.0f;
            assert(fabsf(matrix[row + 1][axis] - expected) < 0.0001f);
        }
    }
    assert(sample_count == 0 && norm_sum == 0);
    assert(memcmp(mag_cal_workspace.ata, zero, sizeof(zero)) == 0);
    if (getenv("CAPABILITY_SMOKE")) printf("accelerometer raw=%d mag=%d poses=%d bias=%g,%g,%g diagonal=%g,%g,%g allocations=%u frees=%u\n",
        CONFIG_CONNECTION_RAW_COLLECTION, CONFIG_SENSOR_USE_MAGNETOMETER, captured,
        matrix[0][0], matrix[0][1], matrix[0][2], matrix[1][0], matrix[2][1], matrix[3][2], allocations, releases);
    printf("shared real accelerometer collector/solver raw=%d mag=%d: PASS\n", CONFIG_CONNECTION_RAW_COLLECTION, CONFIG_SENSOR_USE_MAGNETOMETER);
    return 0;
}
