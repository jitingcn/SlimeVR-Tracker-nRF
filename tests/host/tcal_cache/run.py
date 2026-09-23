#!/usr/bin/env python3
"""Exercise the production MLS/LUT with host mutex and retained-memory adapters."""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2])).resolve()

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
static struct {
    struct { unsigned count; } tempCalState;
    struct { float temp, bias[3]; } tempCalPoints[TCAL_BUFFER_SIZE];
} retained_data;
static typeof(retained_data) *retained = &retained_data;
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
    int result = lut ? sensor_tcal_lut_lookup(temp, bias) : sensor_tcal_mls_lookup(temp, bias);
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
int main(int argc, char **argv) {
    assert(argc == 2);
    seed();
    uint32_t generation = sensor_tcal_model_generation();
    const char *scenario = argv[1];
    if (!strcmp(scenario, "same_count")) {
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
        assert(sensor_tcal_lut_lookup(25.0f, bias) == -1);
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
        assert(sensor_tcal_lut_lookup(20.0f, after) == 0);
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
            assert(sensor_tcal_lut_lookup(temp, bias) == 0);
            for (int axis = 0; axis < 3; axis++) assert(fabsf(bias[axis] - (77.0f + axis)) < 0.002f);
        }
    } else if (!strcmp(scenario, "nonfinite")) {
        float bias[3];
        assert(sensor_tcal_mls_lookup(NAN, bias) == -1);
        sensor_tcal_build_lut_priority(NAN);
        assert(sensor_tcal_lut_get_build_state() == MLS_LUT_BUILD_IDLE);
        sensor_tcal_build_lut_priority(25.0f);
        finish();
        assert(sensor_tcal_lut_lookup(NAN, bias) == -1);
        assert(sensor_tcal_lut_lookup(INFINITY, bias) == -1);
        sensor_tcal_build_lut_priority(-INFINITY);
        expect(25.0f, 25.0f, 0, true);
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
    harness.write_text(f'#include "{source.as_posix()}"\n' + MAIN)
    executable = work / "tcal-cache"
    compiler = shlex.split(os.environ.get("CC", "cc"))
    subprocess.run(compiler + ["-D_GNU_SOURCE", "-std=gnu11", "-O2", "-pthread",
                              "-I", str(work), str(harness), "-lm", "-o", str(executable)], check=True)
    for scenario in ("same_count", "rebuild", "restart", "interleaving", "nonfinite"):
        subprocess.run([str(executable), scenario], check=True)
