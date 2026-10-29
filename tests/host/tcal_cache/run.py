#!/usr/bin/env python3
"""Exercise the production MLS/LUT with host mutex and retained-memory adapters."""
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2])).resolve()
sys.path.insert(0, str(HERE.parent / "harness/python"))
from c_extract import extract_block

GLOBALS = r'''
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#define CONFIG_SENSOR_USE_TCAL 1
#define CONFIG_CMSIS_DSP 0
#define CONFIG_SENSOR_POLY_TEMP_MIN 10
#define CONFIG_SENSOR_POLY_TEMP_MAX 45
#define TCAL_BUFFER_SIZE 16
#define LOG_MODULE_REGISTER(...)
#define LOG_INF(...) ((void)0)
#define LOG_DBG(...) ((void)0)
#define LOG_ERR(...) ((void)0)
#define LOG_WRN(...) ((void)0)
static struct {
    struct { unsigned count; bool valid; unsigned degree; } tempCalState;
    struct { float temp, bias[3]; } tempCalPoints[TCAL_BUFFER_SIZE];
    float tempCalCoeffs[3][4];
    bool tcal_enabled;
} retained_data;
static typeof(retained_data) *retained = &retained_data;
static bool tcal_compensation_enabled, tcal_curve_apply_ready;
static void retained_update(void) {}
static void sensor_calibration_reset_gyro_reference(void) {}
'''
KERNEL = r'''
#pragma once
#include <assert.h>
#include <pthread.h>
#define K_FOREVER (-1)
#define K_MUTEX_DEFINE(name) \
    static pthread_mutex_t name = PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP
static unsigned lock_depth;
static void (*after_unlock)(void);
static int k_mutex_lock(pthread_mutex_t *mutex, int timeout) {
    (void)timeout;
    int result = pthread_mutex_lock(mutex);
    assert(result == 0);
    lock_depth++;
    return result;
}
static int k_mutex_unlock(pthread_mutex_t *mutex) {
    assert(lock_depth);
    lock_depth--;
    int result = pthread_mutex_unlock(mutex);
    assert(result == 0);
    if (!lock_depth && after_unlock) {
        void (*callback)(void) = after_unlock;
        after_unlock = NULL;
        callback();
    }
    return result;
}
static void k_msleep(int ms) { (void)ms; assert(lock_depth == 0); }
'''
UTIL = r'''
#pragma once
#include <stdint.h>
#include <string.h>
static bool v_finite(const float *v, size_t n) {
    for (size_t i = 0; i < n; i++) {
        uint32_t bits;
        memcpy(&bits, &v[i], sizeof(bits));
        if ((bits & 0x7f800000u) == 0x7f800000u) return false;
    }
    return true;
}
'''
MAIN = r'''
#include <stdio.h>
#include <float.h>
#include <stdlib.h>
static void seed(void) {
    sensor_tcal_lock();
    retained->tempCalState.count = 4;
    for (int i = 0; i < 4; i++) {
        float temp = 20.0f + 4.0f * i;
        retained->tempCalPoints[i].temp = temp;
        for (int axis = 0; axis < 3; axis++)
            retained->tempCalPoints[i].bias[axis] = temp * (axis + 1);
    }
    sensor_tcal_model_changed();
    sensor_tcal_unlock();
}
static void finish(void) {
    for (int attempts = 0; attempts < 100; attempts++) {
        if (sensor_tcal_build_lut_continue()) {
            assert(sensor_tcal_lut_get_build_state() == MLS_LUT_BUILD_COMPLETE);
            assert(sensor_tcal_lut_is_valid());
            return;
        }
    }
    assert(!"LUT did not finish");
}
static void expect(float temp, float base, float shift, bool lut) {
    float bias[3];
    int result = lut ? sensor_tcal_curve_lookup(temp, bias) : sensor_tcal_mls_lookup(temp, bias);
    assert(result == 0);
    for (int axis = 0; axis < 3; axis++)
        assert(fabsf(bias[axis] - (base * (axis + 1) + shift)) < 0.002f);
}
static unsigned interleavings;
static void replace_between_entries(void) {
    assert(lock_depth == 0);
    /* The first old-generation entry has been published, but no batch has
       completed: deterministically run a model writer at this unlock boundary. */
    interleavings++;
    sensor_tcal_lock();
    for (int i = 0; i < 4; i++)
        for (int axis = 0; axis < 3; axis++)
            retained->tempCalPoints[i].bias[axis] = 77.0f + axis;
    sensor_tcal_model_changed();
    sensor_tcal_unlock();
}
/* Nonlinear, nonuniform fixture from the confirmed production-path audit.
 * Only the observations are prescribed here; every prediction is production C. */
static void seed_nonlinear(void) {
    static const float temps[TCAL_BUFFER_SIZE] = {
        11.15f, 13.82f, 15.11f, 17.66f, 19.03f, 21.47f, 23.09f, 24.78f,
        26.11f, 27.93f, 29.46f, 31.82f, 34.17f, 37.41f, 40.68f, 43.87f
    };
    static const double base[3] = {.37, -.21, .53};
    static const double linear[3] = {.019, -.013, .008};
    static const double quadratic[3] = {.0024, -.0017, .0011};
    static const double cubic[3] = {.000055, .000080, -.000040};
    sensor_tcal_lock();
    retained->tempCalState.count = TCAL_BUFFER_SIZE;
    for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
        double u = (double)temps[i] - 27.0;
        retained->tempCalPoints[i].temp = temps[i];
        for (int axis = 0; axis < 3; axis++)
            retained->tempCalPoints[i].bias[axis] = (float)(
                base[axis] + u * linear[axis] + u * u * quadratic[axis]
                + u * u * u * cubic[axis]);
    }
    sensor_tcal_model_changed();
    sensor_tcal_unlock();
}
static void expect_exact(const float actual[3], const float expected[3]) {
    assert(memcmp(actual, expected, 3 * sizeof(float)) == 0);
}
static void reference_curve(float temp, float out[3]) {
    if (temp < 10.0f) {
        assert(sensor_tcal_mls_lookup(10.0f, out) == 0);
        return;
    }
    if (temp > 45.0f) {
        assert(sensor_tcal_mls_lookup(45.0f, out) == 0);
        return;
    }
    float index = (temp - 10.0f) * 2.0f;
    int lo = (int)index;
    if (lo >= MLS_LUT_SIZE - 1) lo = MLS_LUT_SIZE - 2;
    float a[3], b[3];
    assert(sensor_tcal_mls_lookup(10.0f + .5f * lo, a) == 0);
    assert(sensor_tcal_mls_lookup(10.0f + .5f * (lo + 1), b) == 0);
    float fraction = index - lo;
    for (int axis = 0; axis < 3; axis++) out[axis] = a[axis] + fraction * (b[axis] - a[axis]);
}
static void literal_curve(void) {
    static const float points[] = {10, 15, 20, 30, 40, 45};
    static const struct { float temp, bias[3]; int bins; } cases[] = {
        {-5,    {.0100000007f, .0009868421f, -.0069999998f}, 1},
        {10.25f,{.0102500003f, .0010552631f, -.0071749995f}, 2},
        {22.25f,{.0222500004f, .0051602963f, -.0155750001f}, 2},
        {44.75f,{.0447500013f, .0200375002f, -.0313250013f}, 2},
        {50,    {.0450000018f, .0202500001f, -.0315000005f}, 1},
    };
    sensor_tcal_lock();
    retained->tempCalState.count = sizeof(points) / sizeof(*points);
    for (unsigned i = 0; i < sizeof(points) / sizeof(*points); i++) {
        float t = points[i];
        retained->tempCalPoints[i].temp = t;
        retained->tempCalPoints[i].bias[0] = t * .001f;
        retained->tempCalPoints[i].bias[1] = t * t * .00001f;
        retained->tempCalPoints[i].bias[2] = t * -.0007f;
    }
    sensor_tcal_model_changed();
    sensor_tcal_unlock();
    uint32_t generation = sensor_tcal_model_generation();
    float actual[3];
    for (unsigned i = 0; i < sizeof(cases) / sizeof(*cases); i++) {
        sensor_tcal_cache_invalidate();
        assert(sensor_tcal_lut_get_build_state() == MLS_LUT_BUILD_IDLE);
        assert(!sensor_tcal_lut_is_valid() && sensor_tcal_lut_get_computed_count() == 0);
        assert(sensor_tcal_curve_lookup(cases[i].temp, actual) == 0);
        for (int axis = 0; axis < 3; axis++)
            assert(fabsf(actual[axis] - cases[i].bias[axis]) < .0000002f);
        assert(sensor_tcal_lut_get_computed_count() == cases[i].bins);
        assert(!sensor_tcal_lut_is_valid());
        assert(sensor_tcal_curve_lookup(cases[i].temp, actual) == 0);
        assert(sensor_tcal_lut_get_computed_count() == cases[i].bins);
    }
    sensor_tcal_build_lut_priority(22.25f);
    finish();
    for (unsigned i = 0; i < sizeof(cases) / sizeof(*cases); i++) {
        assert(sensor_tcal_curve_lookup(cases[i].temp, actual) == 0);
        for (int axis = 0; axis < 3; axis++)
            assert(fabsf(actual[axis] - cases[i].bias[axis]) < .0000002f);
    }
    assert(sensor_tcal_lut_get_computed_count() == MLS_LUT_SIZE);
    assert(sensor_tcal_model_generation() == generation);
}
static void query_between_entries(void) {
    assert(lock_depth == 0);
    if (mls_lut.build_state != MLS_LUT_BUILD_PRIORITY &&
        mls_lut.build_state != MLS_LUT_BUILD_BACKGROUND) return;
    int idx = mls_lut.build_next_idx;
    while (idx < MLS_LUT_SIZE && mls_lut.entries[idx].computed) idx++;
    if (idx < MLS_LUT_SIZE) {
        float bias[3];
        interleavings++;
        /* Reproduce the audit: a nearby foreground fit immediately before the
         * next exact builder temperature must not poison that grid entry. */
        assert(sensor_tcal_mls_lookup(10.0f + .5f * idx + .24f, bias) == 0);
        assert(sensor_tcal_curve_lookup(10.0f + .5f * idx + 1.17f, bias) == 0);
    }
    after_unlock = query_between_entries;
}
static void nonlinear_query_order(void) {
    static const float temps[] = {
        26.11f, 26.27f, 26.48f, 26.59f, 26.72f, 27.24f, 27.49f, 27.66f, 28.11f
    };
    float expected[sizeof(temps) / sizeof(*temps)][3], actual[3];
    seed_nonlinear();
    uint32_t generation = sensor_tcal_model_generation();
    for (unsigned i = 0; i < sizeof(temps) / sizeof(*temps); i++) {
        sensor_tcal_cache_invalidate();
        assert(sensor_tcal_mls_lookup(temps[i], expected[i]) == 0);
    }
    for (int order = 0; order < 2; order++) {
        sensor_tcal_cache_invalidate();
        for (unsigned n = 0; n < sizeof(temps) / sizeof(*temps); n++) {
            unsigned i = order ? sizeof(temps) / sizeof(*temps) - 1 - n : n;
            assert(sensor_tcal_mls_lookup(temps[i] + .24f, actual) == 0);
            assert(sensor_tcal_mls_lookup(temps[i], actual) == 0);
            expect_exact(actual, expected[i]);
        }
    }
    assert(sensor_tcal_model_generation() == generation);
}
static void nonlinear_builder(void) {
    float expected[MLS_LUT_SIZE][3], actual[3];
    seed_nonlinear();
    uint32_t generation = sensor_tcal_model_generation();
    for (int i = 0; i < MLS_LUT_SIZE; i++) {
        sensor_tcal_cache_invalidate();
        assert(sensor_tcal_mls_lookup(10.0f + .5f * i, expected[i]) == 0);
    }
    for (int interleave = 0; interleave < 2; interleave++) {
        sensor_tcal_build_lut_priority(27.24f);
        if (interleave) after_unlock = query_between_entries;
        finish();
        after_unlock = NULL;
        assert(sensor_tcal_lut_get_computed_count() == MLS_LUT_SIZE);
        for (int i = 0; i < MLS_LUT_SIZE; i++) {
            assert(sensor_tcal_curve_lookup(10.0f + .5f * i, actual) == 0);
            expect_exact(actual, expected[i]);
        }
    }
    assert(interleavings > 0);
    assert(sensor_tcal_model_generation() == generation);
}
static void curve_progress(void) {
    static const float temps[] = {
        -FLT_MAX, 10.0f, 11.37f, 17.24f, 24.73f, 27.24f, 31.89f, 43.57f, 45.0f, FLT_MAX
    };
    float expected[sizeof(temps) / sizeof(*temps)][3], actual[3];
    seed_nonlinear();
    uint32_t generation = sensor_tcal_model_generation();
    for (unsigned i = 0; i < sizeof(temps) / sizeof(*temps); i++)
        reference_curve(temps[i], expected[i]);
    sensor_tcal_build_lut_priority(27.24f);
    finish();
    for (unsigned i = 0; i < sizeof(temps) / sizeof(*temps); i++) {
        assert(sensor_tcal_curve_lookup(temps[i], actual) == 0);
        expect_exact(actual, expected[i]);
    }
    for (unsigned i = 0; i < sizeof(temps) / sizeof(*temps); i++) {
        sensor_tcal_cache_invalidate();
        assert(sensor_tcal_lut_get_computed_count() == 0);
        assert(sensor_tcal_curve_lookup(temps[i], actual) == 0);
        expect_exact(actual, expected[i]);
        int count = sensor_tcal_lut_get_computed_count();
        assert(count >= 1 && count <= 2);
        assert(sensor_tcal_curve_lookup(temps[i], actual) == 0);
        assert(sensor_tcal_lut_get_computed_count() == count);
        expect_exact(actual, expected[i]);
    }
    sensor_tcal_cache_invalidate();
    for (int idx = 0; idx < MLS_LUT_SIZE; idx++) {
        int before = sensor_tcal_lut_get_computed_count();
        assert(sensor_tcal_curve_lookup(10.0f + .5f * idx, actual) == 0);
        assert(sensor_tcal_lut_get_computed_count() - before <= 2);
    }
    assert(sensor_tcal_lut_get_computed_count() == MLS_LUT_SIZE);
    assert(sensor_tcal_lut_is_valid());
    sensor_tcal_build_lut_priority(27.24f);
    assert(!sensor_tcal_build_lut_continue());
    assert(sensor_tcal_lut_get_computed_count() < MLS_LUT_SIZE);
    do {
        for (unsigned i = 0; i < sizeof(temps) / sizeof(*temps); i++) {
            int before = sensor_tcal_lut_get_computed_count();
            assert(sensor_tcal_curve_lookup(temps[i], actual) == 0);
            assert(sensor_tcal_lut_get_computed_count() - before <= 2);
            expect_exact(actual, expected[i]);
        }
    } while (!sensor_tcal_build_lut_continue());
    assert(sensor_tcal_lut_get_build_state() == MLS_LUT_BUILD_COMPLETE);
    assert(sensor_tcal_model_generation() == generation);
}
static void curve_generation(void) {
    float before[3], after[3], complete[3], reference[3];
    seed_nonlinear();
    assert(sensor_tcal_mls_lookup(27.24f, before) == 0);
    assert(sensor_tcal_curve_lookup(27.24f, before) == 0);
    uint32_t generation = sensor_tcal_model_generation();
    sensor_tcal_lock();
    for (int i = 0; i < TCAL_BUFFER_SIZE; i++)
        for (int axis = 0; axis < 3; axis++)
            retained->tempCalPoints[i].bias[axis] += 7.0f;
    sensor_tcal_model_changed();
    sensor_tcal_unlock();
    assert(retained->tempCalState.count == TCAL_BUFFER_SIZE);
    assert(sensor_tcal_model_generation() == generation + 1);
    assert(sensor_tcal_lut_get_computed_count() == 0);
    assert(sensor_tcal_curve_lookup(27.24f, after) == 0);
    for (int axis = 0; axis < 3; axis++)
        assert(fabsf(after[axis] - before[axis] - 7.0f) < .00001f);
    reference_curve(27.24f, reference);
    expect_exact(after, reference);
    sensor_tcal_build_lut_priority(27.24f);
    finish();
    assert(sensor_tcal_curve_lookup(27.24f, complete) == 0);
    expect_exact(complete, reference);
}
int main(int argc, char **argv) {
    assert(argc == 2);
    seed();
    uint32_t generation = sensor_tcal_model_generation();
    const char *scenario = argv[1];
    if (!strcmp(scenario, "literal_curve")) {
        literal_curve();
    } else if (!strcmp(scenario, "nonlinear_query_order")) {
        nonlinear_query_order();
    } else if (!strcmp(scenario, "nonlinear_builder")) {
        nonlinear_builder();
    } else if (!strcmp(scenario, "curve_progress")) {
        curve_progress();
    } else if (!strcmp(scenario, "curve_generation")) {
        curve_generation();
    } else if (!strcmp(scenario, "same_count")) {
        sensor_tcal_build_lut_priority(25.0f);
        finish();
        expect(25.0f, 25.0f, 0, false);
        expect(25.0f, 25.0f, 0, true);
        sensor_tcal_lock();
        for (int i = 0; i < 4; i++) {
            retained->tempCalPoints[i].temp += 2.0f;
            for (int axis = 0; axis < 3; axis++)
                retained->tempCalPoints[i].bias[axis] += 7.0f;
        }
        sensor_tcal_model_changed();
        sensor_tcal_unlock();
        assert(sensor_tcal_model_generation() == generation + 1);
        expect(25.0f, 23.0f, 7, false);
        finish();
        expect(25.0f, 23.0f, 7, true);
    } else if (!strcmp(scenario, "rebuild")) {
        sensor_tcal_build_lut_priority(25.0f);
        finish();
        expect(25.0f, 25.0f, 0, true);
        sensor_tcal_cache_invalidate();
        float bias[3];
        assert(sensor_tcal_lut_get_build_state() == MLS_LUT_BUILD_IDLE);
        assert(!sensor_tcal_lut_is_valid() && sensor_tcal_lut_get_computed_count() == 0);
        assert(sensor_tcal_curve_lookup(25.0f, bias) == 0);
        for (int axis = 0; axis < 3; axis++)
            assert(fabsf(bias[axis] - 25.0f * (axis + 1)) < .002f);
        assert(sensor_tcal_lut_get_computed_count() == 2 && !sensor_tcal_lut_is_valid());
        assert(sensor_tcal_model_generation() == generation);
        sensor_tcal_build_lut_priority(25.0f);
        finish();
        assert(sensor_tcal_model_generation() == generation);
        expect(25.0f, 25.0f, 0, true);
    } else if (!strcmp(scenario, "restart")) {
        sensor_tcal_build_lut_priority(25.0f);
        finish();
        sensor_tcal_lock();
        for (int axis = 0; axis < 3; axis++)
            retained->tempCalPoints[0].bias[axis] += 0.01f;
        sensor_tcal_model_changed();
        sensor_tcal_unlock();
        assert(sensor_tcal_lut_get_build_state() == MLS_LUT_BUILD_PRIORITY);
        finish();
        float before[3], after[3];
        assert(sensor_tcal_curve_lookup(20.0f, after) == 0);
        for (int axis = 0; axis < 3; axis++) before[axis] = 20.0f * (axis + 1);
        for (int axis = 0; axis < 3; axis++) assert(after[axis] > before[axis] + 0.005f);
    } else if (!strcmp(scenario, "interleaving")) {
        sensor_tcal_build_lut_priority(25.0f);
        after_unlock = replace_between_entries;
        sensor_tcal_build_lut_continue();
        assert(interleavings == 1);
        assert(sensor_tcal_model_generation() == generation + 1);
        finish();
        for (int idx = 0; idx < MLS_LUT_SIZE; idx++) {
            float bias[3];
            float temp = CONFIG_SENSOR_POLY_TEMP_MIN + 0.5f * idx;
            assert(sensor_tcal_curve_lookup(temp, bias) == 0);
            for (int axis = 0; axis < 3; axis++) assert(fabsf(bias[axis] - (77.0f + axis)) < 0.002f);
        }
    } else if (!strcmp(scenario, "nonfinite")) {
        float bias[3];
        assert(sensor_tcal_mls_lookup(NAN, bias) == -1);
        const float invalid[] = {NAN, INFINITY, -INFINITY};
        for (unsigned i = 0; i < sizeof(invalid) / sizeof(*invalid); i++) {
            const float untouched[3] = {123, 456, 789};
            memcpy(bias, untouched, sizeof(bias));
            assert(sensor_tcal_curve_lookup(invalid[i], bias) == -1);
            expect_exact(bias, untouched);
            assert(sensor_tcal_lut_get_computed_count() == 0);
        }
        sensor_tcal_build_lut_priority(NAN);
        assert(sensor_tcal_lut_get_build_state() == MLS_LUT_BUILD_IDLE);
        sensor_tcal_build_lut_priority(25.0f);
        finish();
        sensor_tcal_build_lut_priority(-INFINITY);
        for (unsigned i = 0; i < sizeof(invalid) / sizeof(*invalid); i++) {
            const float untouched[3] = {123, 456, 789};
            memcpy(bias, untouched, sizeof(bias));
            assert(sensor_tcal_curve_lookup(invalid[i], bias) == -1);
            expect_exact(bias, untouched);
            assert(sensor_tcal_lut_get_computed_count() == MLS_LUT_SIZE);
        }
        expect(25.0f, 25.0f, 0, true);
    } else if (!strcmp(scenario, "boot_recovery")) {
        /* Finite point write survived, STATE did not: count==1 used to turn
         * every lookup into the first-point constant (20 instead of 25). */
        retained->tempCalState.count = 1;
        retained->tempCalState.valid = false;
        retained->tempCalState.degree = 3;
        retained->tempCalCoeffs[0][0] = 999;
        retained->tcal_enabled = true;
        sensor_tcal_runtime_init_from_retained();
        assert(retained->tempCalState.count == 4);
        assert(retained->tempCalState.valid && retained->tempCalState.degree == 0);
        assert(tcal_curve_apply_ready);
        expect(25.0f, 25.0f, 0, false);
        sensor_tcal_build_lut_priority(25.0f);
        finish();
        expect(25.0f, 25.0f, 0, true);
        /* A subsequent boot restores the same table with zero legacy
         * coefficients; volatile MLS/LUT caches start empty again. */
        sensor_tcal_cache_invalidate();
        sensor_tcal_runtime_init_from_retained();
        expect(25.0f, 25.0f, 0, false);
    } else if (!strcmp(scenario, "boot_clear")) {
        /* Clear POINTS persisted, old nonempty STATE/COEFFS survived. */
        memset(retained->tempCalPoints, 0, sizeof(retained->tempCalPoints));
        retained->tempCalState.valid = true;
        retained->tempCalState.degree = 3;
        retained->tempCalCoeffs[0][0] = 999;
        retained->tcal_enabled = true;
        sensor_tcal_runtime_init_from_retained();
        assert(retained->tempCalState.count == 0 && !retained->tempCalState.valid);
        assert(!tcal_curve_apply_ready);
        float bias[3] = {123, 456, 789};
        assert(sensor_tcal_mls_lookup(25.0f, bias) == -1);
        sensor_tcal_build_lut_priority(25.0f);
        assert(sensor_tcal_curve_lookup(25.0f, bias) == -1);
        assert(bias[0] == 123 && bias[1] == 456 && bias[2] == 789);
    } else {
        assert(!"unknown scenario");
    }
    assert(lock_depth == 0);
    puts(scenario);
    return 0;
}
'''

with tempfile.TemporaryDirectory(prefix="tcal-cache-") as directory:
    work = Path(directory)
    for name, content in {"globals.h": GLOBALS, "zephyr/kernel.h": KERNEL,
                          "util.h": UTIL, "sensor/sensor.h": ""}.items():
        destination = work / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(content)
    source = ROOT / "src/sensor/calibration/tcal_mls_lut.c"
    harness = work / "main.c"
    runtime = (ROOT / "src/sensor/calibration/tcal_runtime.c").read_text()
    startup = "\n\n".join(extract_block(
        runtime, rf"^void {name}\([^;{{]*\)\s*\{{") for name in (
        "sensor_tcal_refresh_apply_cache", "sensor_tcal_runtime_init_from_retained"))
    harness.write_text(f'#include "{source.as_posix()}"\n' + startup + "\n" + MAIN)
    executable = work / "tcal-cache"
    compiler = shlex.split(os.environ.get("CC", "cc"))
    subprocess.run(compiler + ["-D_GNU_SOURCE", "-std=gnu11", "-O2", "-pthread",
                              "-I", str(work), str(harness), "-lm", "-o", str(executable)], check=True)
    for scenario in ("same_count", "rebuild", "restart", "interleaving", "nonfinite",
                     "boot_recovery", "boot_clear", "nonlinear_query_order",
                     "nonlinear_builder", "curve_progress", "curve_generation", "literal_curve"):
        subprocess.run([str(executable), scenario], check=True)
