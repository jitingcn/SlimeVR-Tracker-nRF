#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#define TCAL_BUFFER_SIZE 8
#define MLS_MIN_POINTS_FOR_FIT 4
#define LOG_WRN(...) ((void)0)
#define LOG_INF(...) ((void)0)
struct TempCalPoint { float temp; float bias[3]; };
static struct {
    struct TempCalPoint tempCalPoints[TCAL_BUFFER_SIZE];
    struct { uint8_t count; bool valid; uint8_t degree; } tempCalState;
    float tempCalCoeffs[3][4];
    bool tcal_enabled;
} storage, *retained = &storage;
typedef enum { SENSOR_TCAL_APPLY_DISABLED, SENSOR_TCAL_APPLY_CURVE,
    SENSOR_TCAL_APPLY_ZRO_FALLBACK } sensor_tcal_apply_mode_t;
static bool tcal_compensation_enabled, tcal_curve_apply_ready;
static unsigned locks, lock_depth, updates, resets;
static void (*unlock_hook)(void);
static void sensor_tcal_lock(void) { assert(lock_depth == 0); lock_depth++; locks++; }
static void sensor_tcal_unlock(void) {
    assert(lock_depth == 1); lock_depth--;
    if (unlock_hook) { void (*hook)(void) = unlock_hook; unlock_hook = NULL; hook(); }
}
static bool v_finite(const float *values, size_t n) {
    for (size_t i = 0; i < n; i++) {
        uint32_t bits; memcpy(&bits, values + i, sizeof(bits));
        if ((bits & 0x7f800000u) == 0x7f800000u) return false;
    }
    return true;
}
static void retained_update(void) { updates++; }
static void sensor_calibration_reset_gyro_reference(void) { resets++; }
#include "recovery.inc"
#ifndef RECOVERY_BOOT_ONLY
static void replace_model(void) {
    assert(!lock_depth);
    memset(retained->tempCalPoints, 0, sizeof(retained->tempCalPoints));
    tcal_compensation_enabled = false;
    tcal_curve_apply_ready = false;
}
#endif
int main(void) {
    /* Simulate each POINTS / COEFFS / STATE write boundary for clear and
     * replacement. Independently durable tables must dominate stale state. */
    const unsigned table_sizes[] = {0, 1, 4};
    for (unsigned old = 0; old < 3; old++) {
        for (unsigned next = 0; next < 3; next++) {
            for (unsigned boundary = 0; boundary <= 3; boundary++) {
                memset(retained, 0, sizeof(*retained));
                unsigned count = table_sizes[boundary >= 1 ? next : old];
                unsigned state_count = table_sizes[boundary >= 3 ? next : old];
                for (unsigned i = 0; i < count; i++) {
                    retained->tempCalPoints[i].temp = 20.0f + i;
                    retained->tempCalPoints[i].bias[0] = (float)i;
                }
                retained->tempCalState.count = state_count;
                retained->tempCalState.valid = state_count != 0;
                retained->tempCalState.degree = boundary >= 2 ? 0 : 3;
                retained->tempCalCoeffs[0][0] = boundary >= 2 ? 0 : 99;
                retained->tcal_enabled = true;
                sensor_tcal_runtime_init_from_retained();
                assert(retained->tempCalState.count == count);
                assert(retained->tempCalState.valid == (count != 0));
                assert(retained->tempCalState.degree == 0);
                assert(retained->tempCalCoeffs[0][0] == 0);
                assert(tcal_curve_apply_ready == (count >= MLS_MIN_POINTS_FOR_FIT));
                /* Warm reboot must derive the same state a second time. */
                sensor_tcal_runtime_init_from_retained();
                assert(retained->tempCalState.count == count);
            }
        }
    }
    retained->tempCalPoints[1].temp = NAN;
    retained->tempCalPoints[2].bias[1] = INFINITY;
    sensor_tcal_runtime_init_from_retained();
    assert(retained->tempCalState.count == 2);
    assert(retained->tempCalPoints[1].temp == 0 && retained->tempCalPoints[2].temp == 0);
#ifndef RECOVERY_BOOT_ONLY
    struct TempCalPoint points[TCAL_BUFFER_SIZE] = {0};
    uint16_t count;
    bool enabled;
    locks = 0;
    unlock_hook = replace_model;
    sensor_tcal_apply_mode_t mode = sensor_tcal_snapshot(&enabled, points, &count);
    assert(locks == 1 && lock_depth == 0);
    assert(enabled && mode == SENSOR_TCAL_APPLY_ZRO_FALLBACK && count == 2);
    assert(points[0].temp == 20 && points[1].temp == 23);
    assert(!tcal_compensation_enabled && retained->tempCalPoints[0].temp == 0);
#endif
    assert(updates && resets);
    puts("T-Cal recovery: independent write boundaries, invalid points and one-lock snapshot passed");
}
