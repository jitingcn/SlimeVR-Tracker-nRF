#!/usr/bin/env python3
"""Run extracted production gyro feeds and sensitivity lifecycle mutations."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'harness/python'))
from c_extract import extract_block

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get('SOURCE_ROOT', HERE.parents[2]))
source = (ROOT / 'src/sensor/sensor.c').read_text()
sensitivity = (ROOT / 'src/sensor/calibration/cal_sens.c').read_text()
calibration = (ROOT / 'src/sensor/calibration/calibration.c').read_text()

def function(name, text=source):
    return extract_block(text, rf'^(?:static )?(?:void|int|float|bool) {re.escape(name)}\([^;{{]*\)\s*\{{')

preamble = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <errno.h>
#define CONFIG_SENSOR_USE_SENS_CALIBRATION 1
#define CONFIG_SENSOR_SENS_REV 1
#define CONFIG_SENSOR_TCAL_HEATED 0
#define DEBUG 1
#define DEG_TO_RAD (3.14159265358979323846f / 180.0f)
#define RAD_TO_DEG (180.0f / 3.14159265358979323846f)
#include "sensor/motion_state.h"
#include "util.h"
static unsigned total_gyro_samples;
static float gyro_actual_time = 0.0025f;
static unsigned raw_count, cal_count, feed_count;
static bool change_bias_on_feed;
static float fusion_bias[3] = {1.25f, -0.75f, 0.5f};
static float received[64][3], received_dt[64];
static float cal_samples[64][3];
static struct {float gyroSensScale[3];} retained_data = {{1.0625f, 0.9375f, 1.03125f}};
static typeof(retained_data) *retained = &retained_data;
static bool sensor_owner_active;
static struct {unsigned depth;} sensitivity_lock;
static uint32_t sensitivity_generation;
static uint16_t sensitivity_operation;
struct led_token {unsigned session, request_id;};
static struct led_token sensitivity_feedback;
static unsigned storage_depth, storage_writes, history_discards;
#define K_FOREVER (-1)
#define K_NO_WAIT 0
static int k_mutex_lock(typeof(sensitivity_lock) *lock, int wait) {
    assert(lock == &sensitivity_lock);
    if (lock->depth && wait == K_NO_WAIT) return -EBUSY;
    assert(lock->depth == 0);
    lock->depth = 1;
    return 0;
}
static int k_mutex_unlock(typeof(sensitivity_lock) *lock) {
    assert(lock == &sensitivity_lock && lock->depth == 1);
    lock->depth = 0;
    return 0;
}
enum {
    CAL_OUTCOME_CANCELLED, CAL_PHASE_APPLIED, CAL_REASON_REPLACED,
    CAL_REASON_RESET, LED_CANCELLED, LED_OWNER_SENS, MAIN_GYRO_SENS_ID
};
static void cal_event_end(uint16_t operation, uint8_t outcome, uint8_t phase, uint8_t reason) {
    (void)operation; (void)outcome; (void)phase; (void)reason;
}
static void sensor_calibration_result(struct led_token token, int result) {
    (void)token; (void)result;
}
static void tracker_events_notify(void) {}
static int sensor_operation_result(int owner, int result, bool applied) {
    assert(owner == LED_OWNER_SENS && !sensor_owner_active);
    (void)applied;
    return result;
}
static void sys_warm_transaction_begin(void) {
    assert(!sensor_owner_active && storage_depth == 0);
    storage_depth = 1;
}
static void sys_warm_transaction_end(bool commit) {
    assert(!commit && storage_depth == 1);
    storage_depth = 0;
}
static int sys_write(unsigned id, void *destination, const void *value, size_t length) {
    assert(id == MAIN_GYRO_SENS_ID && storage_depth == 1);
    assert(destination == retained->gyroSensScale && length == sizeof(retained->gyroSensScale));
    memmove(destination, value, length);
    storage_writes++;
    return 0;
}
static void retained_update(void) {}
/* Real observer pending/attitude/bias lifecycle is tested by the native
 * observer harness, not inferred from callback counts in this feed fixture. */
static void vqf_discard_motion_history(void) {history_discards++;}
static void sensor_diagnostics_on_raw_gyro(float *g) {(void)g; raw_count++;}
static void sensor_diagnostics_on_cal_gyro(float *g) {
    assert(cal_count < 64);
    memcpy(cal_samples[cal_count++], g, 3*sizeof(float));
}
static float calibration_delta[3];
static bool physical_reset;
static unsigned reset_count;
static bool sensor_calibration_process_gyro(float *g, float dt, float *delta) {
    assert(dt == gyro_actual_time); /* Raw FIFO interval, before any merge. */
    const float offsets[3] = {0.125f, -0.0625f, 0.03125f};
    for (int i=0;i<3;i++) g[i] -= offsets[i];
    memcpy(delta, calibration_delta, sizeof(calibration_delta));
    memset(calibration_delta, 0, sizeof(calibration_delta));
    bool reset = physical_reset;
    physical_reset = false;
    return reset;
}
static void get_bias(float *g) {memcpy(g, fusion_bias, sizeof(fusion_bias));}
static void update_gyro(float *g, float dt) {
    assert(feed_count < 64);
    memcpy(received[feed_count], g, 3*sizeof(float));
    received_dt[feed_count++] = dt;
    if (change_bias_on_feed) fusion_bias[0] = 40.0f;
}
static float filtered_gyro[3] = {2, 3, 4};
static void rebase_bias(const float *delta) {
    for (int i = 0; i < 3; ++i) {
        fusion_bias[i] += delta[i];
        filtered_gyro[i] += delta[i];
    }
}
static void reset_bias(float *bias) {
    memcpy(fusion_bias, bias, sizeof(fusion_bias));
    reset_count++;
}
static const struct {
    void (*update_gyro)(float*,float);
    void (*get_gyro_bias)(float*);
    void (*rebase_gyro_bias)(const float*);
    void (*set_gyro_bias)(float*);
} backend = {update_gyro, get_bias, rebase_bias, reset_bias};
static const typeof(backend) *sensor_fusion = &backend;
static bool sensor_fusion_init = true;
'''
state = source[source.index('static uint8_t gyro_oversample_n'):source.index('\n#if CONFIG_SENSOR_ACCEL_OVERSAMPLING > 1', source.index('static uint8_t gyro_oversample_n'))]
snapshot_start = sensitivity.index('static struct sensitivity_snapshot {')
snapshot_state = sensitivity[snapshot_start:sensitivity.index('\n#endif', snapshot_start)]
parts = [preamble, '#if CONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION',
         snapshot_state, '#endif', function('sensor_calibration_sensitivity_valid', calibration)]
parts += [function(name, sensitivity) for name in (
    'sensitivity_replace_locked', 'sensor_calibration_set_sensitivity',
    'sensor_calibration_reset_sensitivity')]
parts += ['#if CONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION']
parts += [function(name, sensitivity) for name in (
    'sensor_calibration_sensitivity_init', 'sensor_calibration_apply_sensitivity',
    'sensor_calibration_sensitivity_clear_begin',
    'sensor_calibration_sensitivity_clear_end')]
parts += ['#endif', state, 'static struct {bool frame_invalid;} sensor_motion_state;']
parts += [function('feed_calibrated_gyro')]
parts += ['#if CONFIG_SENSOR_GYRO_OVERSAMPLING > 1']
parts += [function(name) for name in ('gyro_dq_mul', 'gyro_dq_accumulate_sample', 'gyro_dq_to_feed_gyro')]
parts += ['#endif', function('sensor_discard_gyro_window'),
          function('sensor_reject_gyro_sample'), function('feed_gyro_sample')]
main = r'''

static void feed_raw(float raw[3], int *count, bool valid_acquisition) {
    assert(!sensor_owner_active);
    sensor_owner_active = true;
    feed_gyro_sample(raw, count, valid_acquisition);
    sensor_owner_active = false;
}

static void feed_desired(const float desired[3], int *count) {
    const float offsets[3] = {0.125f, -0.0625f, 0.03125f};
    float raw[3];
    for (int i = 0; i < 3; ++i) raw[i] = desired[i] / retained->gyroSensScale[i] + offsets[i];
    feed_raw(raw, count, true);
}


static void sensitivity_lifecycle(int n, int *count) {
    const float desired[3] = {fusion_bias[0] + 90.0f, fusion_bias[1], fusion_bias[2]};
    const float degrees[3] = {12.0f, -6.0f, 3.0f};
    const unsigned mutations[] = {0, 0, 1, 1, 0, 2, 2};
    unsigned samples = 1;
    feed_desired(desired, count); /* N4 begins with an existing partial merge. */
    for (unsigned mutation = 0; mutation < sizeof(mutations)/sizeof(*mutations); ++mutation) {
        float previous_scale[3];
        memcpy(previous_scale, retained->gyroSensScale, sizeof(previous_scale));
        uint32_t generation = sensitivity_generation;
        unsigned feeds = feed_count;
        unsigned writes = storage_writes;
        unsigned discards = history_discards;
        if (mutations[mutation] == 0) {
            assert(sensor_calibration_set_sensitivity(degrees) == 0);
        } else if (mutations[mutation] == 1) {
            assert(sensor_calibration_reset_sensitivity() == 0);
        } else {
            /* Same storage -> sensitivity lock order as reset-all. Clear
             * the entire retained fixture, not just the gyro scale field. */
            sys_warm_transaction_begin();
#if CONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION
            sensor_calibration_sensitivity_clear_begin();
#endif
            memset(&retained_data, 0, sizeof(retained_data));
            for (int i = 0; i < 3; ++i) retained->gyroSensScale[i] = 1.0f;
#if CONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION
            sensor_calibration_sensitivity_clear_end();
#endif
            sys_warm_transaction_end(false);
        }
        bool replaced = mutations[mutation] != 2 || CONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION;
        assert(sensitivity_generation == generation + replaced);
        assert(history_discards == discards);
        assert(storage_writes == writes + (mutations[mutation] != 2));
        assert(storage_depth == 0);
        assert(feed_count == feeds && reset_count == 0);
        if (mutation == 1 || mutation == 3 || mutation == 6)
            assert(!memcmp(previous_scale, retained->gyroSensScale, sizeof(previous_scale)));
        else
            assert(memcmp(previous_scale, retained->gyroSensScale, sizeof(previous_scale)));
        for (int sample = 0; sample < n; ++sample) {
            feed_desired(desired, count);
            samples++;
#if CONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION
            assert(history_discards == discards + replaced);
#else
            assert(history_discards == discards);
#endif
            assert(feed_count == samples / (unsigned)n && *count == (int)feed_count);
        }
    }
    for (int sample = 0; sample < n; ++sample) {
        feed_desired(desired, count);
        samples++;
    }
    assert(raw_count == samples && cal_count == samples && total_gyro_samples == samples);
    assert(feed_count == samples / (unsigned)n && reset_count == 0);
    for (unsigned feed = 0; feed < feed_count; ++feed) {
        assert(received_dt[feed] == n * gyro_actual_time);
        for (int axis = 0; axis < 3; ++axis)
            assert(fabsf(received[feed][axis] - desired[axis]) < 0.001f);
    }
    for (int axis = 0; axis < 3; ++axis) {
        assert(fusion_bias[axis] == desired[axis] - (axis == 0 ? 90.0f : 0.0f));
        assert(filtered_gyro[axis] == axis + 2);
    }
    printf("sensitivity lifecycle SIDE=%d N=%d: manual/reset/reset-all same-scale and scale changes, coherent scales and partial merges preserved\n",
        CONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION, n);
}
static void invalid_gyro(int n, int *count) {
    const float values[3]={NAN,INFINITY,-INFINITY};
    const float desired[3]={20,2,-3};
    for(int axis=0;axis<3;axis++) for(int kind=0;kind<3;kind++) {
        for(int sample=0;sample<n-1;sample++) feed_desired(desired,count);
        unsigned feeds=feed_count, calibrated=cal_count, discarded=history_discards;
        float invalid[3]={1,2,3}; invalid[axis]=values[kind];
        sensor_motion_state.frame_invalid=false;
        feed_raw(invalid,count,true);
        assert(sensor_motion_state.frame_invalid && feed_count==feeds && cal_count==calibrated);
#if CONFIG_SENSOR_GYRO_OVERSAMPLING > 1
        assert(gyro_oversample_count==0 && gyro_dq_acc[0]==1);
        assert(gyro_merge_bias_dps[0]==0 && gyro_merge_bias_dps[1]==0 && gyro_merge_bias_dps[2]==0);
#endif
#if CONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION
        assert(history_discards==discarded+1);
#else
        assert(history_discards==discarded);
#endif
        for(int sample=0;sample<n;sample++) feed_desired(desired,count);
        assert(feed_count==feeds+1 && *count==(int)feed_count);
        for(int j=0;j<3;j++) assert(fabsf(received[feeds][j]-desired[j])<.001f);
    }
    puts("nonfinite gyro axes discard partial windows before calibration and fusion");
}
int main(int argc, char **argv) {
    int n = argc > 1 ? atoi(argv[1]) : 1;
    int count = 0;
#if CONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION
    assert(!sensor_calibration_sensitivity_init());
#endif
    gyro_oversample_n = n;
#if CONFIG_SENSOR_GYRO_OVERSAMPLING > 1
    gyro_effective_time = gyro_actual_time * n;
    gyro_dq_acc[0] = 1.0f;
#endif
    if (argc > 2 && !strcmp(argv[2], "invalid")) {
        invalid_gyro(n, &count);
        return 0;
    }
    if (argc > 2 && !strcmp(argv[2], "sensitivity")) {
        sensitivity_lifecycle(n, &count);
        return 0;
    }
    if (argc > 2) {
        const bool reset = !strcmp(argv[2], "physical");
        float old_bias[3]; memcpy(old_bias, fusion_bias, sizeof(old_bias));
        float desired[3] = {old_bias[0] + 90.0f, old_bias[1], old_bias[2]};
        if (n > 1) feed_desired(desired, &count);
        const float shift[3] = {0.5f, -0.25f, 0.125f};
        memcpy(calibration_delta, shift, sizeof(shift));
        if (reset) {
            physical_reset = true;
            desired[0] = 20.0f; desired[1] = desired[2] = 0.0f;
        } else {
            for (int i = 0; i < 3; ++i) desired[i] += calibration_delta[i] * retained->gyroSensScale[i];
        }
        const int remaining = reset ? n : n - (n > 1);
        for (int i = 0; i < remaining; ++i) {
            feed_desired(desired, &count);
            assert(feed_count == (unsigned)(i == remaining - 1));
        }
        assert(count == 1 && reset_count == (unsigned)reset);
        for (int i = 0; i < 3; ++i) {
            assert(fabsf(received[0][i] - desired[i]) < 0.001f);
            assert(fabsf(fusion_bias[i] - (reset ? 0.0f : desired[i] - (i == 0 ? 90.0f : 0.0f))) < 0.00001f);
            assert(fabsf(filtered_gyro[i] - ((i + 2) + shift[i] * retained->gyroSensScale[i])) < 0.00001f);
        }
        assert(received_dt[0] == n * gyro_actual_time);
        puts(reset ? "physical reset discards partial merge" : "scaled rebase preserves merged residual");
        return 0;
    }
    if (n == 1) {
        /* Include near-zero input: exp/log N=1 used to discard tiny rotation. */
        const float inputs[][3] = {
            {0.1250001f, -0.0625001f, 0.0312501f},
            {13.0625f, -9.875f, 31.03125f},
            {1500.125f, -1200.0625f, 500.03125f},
            {0.125f, -0.0625f, 0.03125f},
        };
        const float offsets[3] = {0.125f, -0.0625f, 0.03125f};
        for (unsigned i=0;i<4;i++) {
            float raw[3]; memcpy(raw, inputs[i], sizeof(raw));
            feed_raw(raw, &count, i != 1);
            for (unsigned j=0;j<3;j++) {
                float expected = (inputs[i][j]-offsets[j])*retained->gyroSensScale[j];
                if (received[i][j] != expected) {
                    fprintf(stderr,"runtime N1 changed calibrated gyro sample %u axis %u: %a != %a\n",i,j,received[i][j],expected);
                    return 1;
                }
                assert(cal_samples[i][j] == expected);
            }
            assert(count == (int)i+1);
            assert(received_dt[i] == gyro_actual_time);
        }
        assert(raw_count == 4 && cal_count == 4 && feed_count == 4);
        assert(total_gyro_samples == 3);
        puts("direct N1: calibrated samples, dt, diagnostics and acquisition counts match");
    } else {
        /* Two windows plus partial tail; analytical fixed-axis rotation. */
        for (int i=0;i<10;i++) {
            float desired[3] = {fusion_bias[0]+90.0f, fusion_bias[1], fusion_bias[2]};
            float raw[3] = {desired[0]/retained->gyroSensScale[0]+0.125f,
                desired[1]/retained->gyroSensScale[1]-0.0625f,
                desired[2]/retained->gyroSensScale[2]+0.03125f};
            feed_raw(raw, &count, true);
            assert(count == (i+1)/n);
        }
        assert(raw_count == 10 && cal_count == 10 && total_gyro_samples == 10);
        assert(feed_count == 2);
        for (unsigned i=0;i<feed_count;i++) {
            assert(received_dt[i] == gyro_actual_time*n);
            assert(fabsf(received[i][0]-fusion_bias[0]-90.0f) < 0.001f);
            assert(fabsf(received[i][1]-fusion_bias[1]) < 0.001f);
            assert(fabsf(received[i][2]-fusion_bias[2]) < 0.001f);
        }
#if CONFIG_SENSOR_GYRO_OVERSAMPLING > 1
        assert(gyro_oversample_count == 2);
#endif
        puts("merged N4: full windows, partial tail, bias carrier, dt and sample counts preserved");
    }
    return 0;
}
'''

temperature_selection = source[
    source.index('\tframe->raw_collect_temp_c = NAN;'):
    source.index('\n\t// Debug info', source.index('\tframe->raw_collect_temp_c = NAN;'))
]
frame_end = source.index('} sensor_loop_frame_t;') + len('} sensor_loop_frame_t;')
frame_start = source.rfind('typedef struct {', 0, frame_end)
temperature_fixture = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#define DEBUG 0
static int64_t now_ms;
static int64_t last_temp_time = -1000;
static float temp = 25.0f;
static float sensor_tcal_temp = 25.0f;
static float sensor_tcal_temp_raw = 25.0f;
static bool sensor_tcal_temp_filter_initialized;
static int64_t k_uptime_get(void) { return now_ms; }
static bool bad_only;
static unsigned temperature_calls, gyro_calls, accel_calls, raw_calls;
static unsigned expected_temperature_calls;
static float expected_temperature;
static void assert_temperature(float value) {
    assert(isnan(expected_temperature) ? isnan(value) : value == expected_temperature);
}
#if CONFIG_SENSOR_VQF_ADAPTIVE_BIAS
static void vqf_update_temperature(float value) {
    assert_temperature(value);
    temperature_calls++;
}
#endif
static uint8_t sensor_fifo_raw_buffer[1];
static int fifo_process(uint16_t index, uint8_t *data, float *a, float *g) {
    assert(data == sensor_fifo_raw_buffer && index < 2);
    a[2] = 1.0f;
    g[0] = bad_only ? NAN : 1.0f + index;
    return 0;
}
static const struct {
    int (*fifo_process)(uint16_t, uint8_t *, float *, float *);
} imu = {fifo_process};
static const typeof(imu) *sensor_imu = &imu;
static void sensor_diagnostics_reset_frame(void) {}
static void sensor_diagnostics_on_decoded_gyro(float *g) { assert(bad_only || g[0] != 0); }
static void sensor_raw_collection_on_sample(float *a, float *g, float t, bool active) {
    assert(a[2] == 1 && (bad_only || g[0] != 0) && active);
    assert_temperature(t);
    raw_calls++;
}
static void feed_gyro_sample(float *g, int *count) {
    assert((bad_only || g[0] != 0) && temperature_calls == expected_temperature_calls);
    if (bad_only) { assert(isnan(g[0])); gyro_calls++; return; }
    gyro_calls++;
    (*count)++;
}
static void feed_accel_sample(float *a, float *sum, int *count) {
    assert(a[2] == 1);
    sum[2] += a[2];
    accel_calls++;
    (*count)++;
}
'''
temperature_fixture += source[frame_start:frame_end] + '\n'
temperature_fixture += 'static void select_temperature(sensor_loop_frame_t *frame) {\n'
temperature_fixture += temperature_selection + '\n}\n'
temperature_fixture += function('sensor_loop_process_fifo') + '\n'
temperature_fixture += r'''
static void batch(float expected, uint16_t packets) {
    sensor_loop_frame_t frame = {.packets = packets, .dc_active = true};
    expected_temperature = expected;
    select_temperature(&frame);
    assert_temperature(frame.raw_collect_temp_c);
    gyro_calls = accel_calls = raw_calls = 0;
#if CONFIG_SENSOR_VQF_ADAPTIVE_BIAS
    expected_temperature_calls = temperature_calls + 1;
#else
    expected_temperature_calls = 0;
#endif
    sensor_loop_process_fifo(&frame);
    assert(temperature_calls == expected_temperature_calls);
    assert(gyro_calls == packets && accel_calls == packets && raw_calls == packets);
    assert(frame.g_count == packets && frame.a_count == packets);
    assert(frame.processed_packets == packets && frame.a_sum[2] == packets);
}
int main(void) {
    /* Initialized 25C must not reach fusion before a real measurement. */
    batch(NAN, 2);
    now_ms = last_temp_time = 100;
    temp = sensor_tcal_temp_raw = -10.0f;
    sensor_tcal_temp = -8.25f;
    sensor_tcal_temp_filter_initialized = true;
    float expected = CONFIG_SENSOR_USE_TCAL ? sensor_tcal_temp : temp;
    batch(expected, 2);
    now_ms = 1100; /* Existing recorder freshness includes exactly 1s. */
    batch(expected, 2);
    now_ms = 1101;
    batch(NAN, 2);
    now_ms = last_temp_time = 1200;
    temp = sensor_tcal_temp_raw = sensor_tcal_temp = 25.0f;
    batch(25.0f, 2); /* A real 25C measurement is valid. */
    batch(25.0f, 0);
    bad_only=true; gyro_calls=0;
#if CONFIG_SENSOR_VQF_ADAPTIVE_BIAS
    expected_temperature_calls=temperature_calls+1;
#endif
    sensor_loop_frame_t bad_frame={.packets=1,.dc_active=true,.raw_collect_temp_c=25};
    expected_temperature=25; sensor_loop_process_fifo(&bad_frame);
    assert(gyro_calls==1 && bad_frame.g_count==0 && bad_frame.processed_packets==1);
    printf("batch temperature: adaptive=%d tcal=%d vqf=%d eqf=%d; signed, initial/stale, ordering and sample counts preserved\n",
        CONFIG_SENSOR_VQF_ADAPTIVE_BIAS, CONFIG_SENSOR_USE_TCAL,
        CONFIG_SENSOR_USE_VQF, CONFIG_SENSOR_USE_EQF);
    return 0;
}
'''

with tempfile.TemporaryDirectory(prefix='sensor-fast-') as directory:
    tmp = Path(directory)
    (tmp/'zephyr').mkdir()
    (tmp/'zephyr/kernel.h').write_text('/* util.c needs no kernel services in this host test. */\n')
    unit = tmp/'gyro.c'
    unit.write_text('\n\n'.join(parts)+main)
    # Keep the existing compiled/runtime N1/N4 matrix on both SIDE paths.
    scenarios = [(4, 4), (1, 1), (4, 1)]
    failures = []
    for side in (0, 1):
        for compiled, runtime in scenarios:
            binary = tmp/f'gyro-side{side}-{compiled}'
            subprocess.run(shlex.split(os.environ.get('CC','cc')) + ['-std=gnu11','-Wall','-Wextra','-Werror','-Wno-unused-function','-g','-O1','-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-pie','-no-pie',f'-DCONFIG_SENSOR_GYRO_OVERSAMPLING={compiled}',f'-DCONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION={side}','-DCONFIG_CONNECTION_RAW_COLLECTION=1','-DCONFIG_SENSOR_USE_MAGNETOMETER=1','-I',str(tmp),'-I',str(ROOT/'src'),str(unit),str(ROOT/'src/sensor/motion_state.c'),str(ROOT/'src/util.c'),'-lm','-o',str(binary)],check=True)
            cases = (None, "rebase", "physical", "sensitivity", "invalid")
            for scenario in cases:
                result = subprocess.run([str(binary), str(runtime)] + ([scenario] if scenario else []))
                if result.returncode:
                    failures.append((side, compiled, runtime, scenario, result.returncode))
    if failures:
        raise SystemExit(f'Failed (SIDE, compile N, runtime N, scenario, exit): {failures}')
    # Link the actual startup, writer, snapshot and feed bodies to pthread and
    # gated NVS leaves. Timeout only bounds a regression to a blocking reader.
    system = (ROOT / 'src/system/system.c').read_text()
    globals_start = sensitivity.index('K_MUTEX_DEFINE(sensitivity_lock);')
    globals_end = sensitivity.index('\n#if CONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION', globals_start)
    contention_parts = [
        sensitivity[globals_start:globals_end], snapshot_state,
        function('sensor_calibration_sensitivity_valid', calibration),
        *(function(name, system) for name in ('sys_warm_transaction_begin',
                                              'sys_warm_transaction_end', 'sys_write')),
        *(function(name, sensitivity) for name in (
            'sensor_calibration_sensitivity_init', 'sensitivity_replace_locked',
            'sensor_calibration_apply_sensitivity', 'sensor_calibration_set_sensitivity',
            'sensor_calibration_reset_sensitivity')),
        function('sensor_calibration_read', calibration),
        function('feed_calibrated_gyro'), function('sensor_discard_gyro_window'),
        function('sensor_reject_gyro_sample'), function('feed_gyro_sample'),
        '#define TEST_CACHE_GENERATION (sensitivity_cache.generation)',
    ]
    (tmp/'sensitivity_contention_sources.inc').write_text('\n\n'.join(contention_parts))
    contention_binary = tmp/'sensitivity-contention'
    subprocess.run(shlex.split(os.environ.get('CC', 'cc')) + [
        '-std=gnu11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
        '-g', '-O1', '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
        '-fno-pie', '-no-pie', '-pthread', '-DDEBUG=1',
        '-DCONFIG_SENSOR_USE_SENS_CALIBRATION=1', '-DCONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION=1',
        '-DCONFIG_SENSOR_GYRO_OVERSAMPLING=1', '-DCONFIG_SENSOR_USE_TCAL=0',
        '-DCONFIG_SENSOR_TCAL_HEATED=0', '-DCONFIG_SENSOR_USE_MAGNETOMETER=0',
        '-I', str(tmp), '-I', str(ROOT/'src'), str(HERE/'test_sensitivity_contention.c'),
        str(ROOT/'src/util.c'), '-lm', '-o', str(contention_binary),
    ], check=True)
    subprocess.run([str(contention_binary)], check=True, timeout=10)
    temperature_unit = tmp/'temperature.c'
    temperature_unit.write_text(temperature_fixture)
    for adaptive, tcal, vqf, eqf in (
            (1, 1, 1, 0), (1, 0, 1, 0), (0, 1, 1, 0),
            (0, 0, 1, 0), (0, 1, 0, 1), (0, 0, 0, 0)):
        binary = tmp/f'temperature-{adaptive}-{tcal}-{vqf}-{eqf}'
        subprocess.run(shlex.split(os.environ.get('CC', 'cc')) + [
            '-std=gnu11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-variable',
            '-g', '-O1', '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
            '-fno-pie', '-no-pie',
            '-DCONFIG_CONNECTION_RAW_COLLECTION=1', '-DCONFIG_SENSOR_USE_MAGNETOMETER=1',
            f'-DCONFIG_SENSOR_VQF_ADAPTIVE_BIAS={adaptive}',
            f'-DCONFIG_SENSOR_USE_TCAL={tcal}', f'-DCONFIG_SENSOR_USE_VQF={vqf}',
            f'-DCONFIG_SENSOR_USE_EQF={eqf}', str(temperature_unit), '-lm',
            '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
