#!/usr/bin/env python3
"""Run extracted production gyro feeds. SOURCE_ROOT selects pre/fixed checkout."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get('SOURCE_ROOT', HERE.parents[2]))
source = (ROOT / 'src/sensor/sensor.c').read_text()

def function(name):
    match = re.search(rf'^(?:static )?(?:void|int|float) {name}\([^;{{]*\)\s*\{{', source, re.MULTILINE)
    if match is None:
        raise ValueError(name)
    start = source.index('{', match.start())
    tokens = re.compile(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]', re.DOTALL)
    depth = 0
    for token in tokens.finditer(source, start):
        if token.group() == '{':
            depth += 1
        elif token.group() == '}':
            depth -= 1
            if depth == 0:
                return source[match.start():token.end()]
    raise ValueError(name)

preamble = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#define CONFIG_SENSOR_USE_SENS_CALIBRATION 1
#define DEBUG 1
#define DEG_TO_RAD (3.14159265358979323846f / 180.0f)
#define RAD_TO_DEG (180.0f / 3.14159265358979323846f)
static unsigned total_gyro_samples;
static float gyro_actual_time = 0.0025f;
static unsigned raw_count, cal_count, rest_count, feed_count;
static float fusion_bias[3] = {1.25f, -0.75f, 0.5f};
static float received[32][3], received_dt[32];
static float cal_samples[32][3];
static struct {float gyroSensScale[3];} retained_data = {{1.125f, 0.9375f, 1.0625f}};
static const typeof(retained_data) *retained = &retained_data;
static void sensor_diagnostics_on_raw_gyro(float *g) {(void)g; raw_count++;}
static void sensor_diagnostics_on_cal_gyro(float *g) {memcpy(cal_samples[cal_count++], g, 3*sizeof(float));}
static void sensor_record_rest_gyro_motion(float *g) {(void)g; rest_count++;}
static void sensor_calibration_process_gyro(float *g) {
    const float offsets[3] = {0.125f, -0.0625f, 0.03125f};
    for (int i=0;i<3;i++) g[i] -= offsets[i];
}
static void get_bias(float *g) {memcpy(g, fusion_bias, sizeof(fusion_bias));}
static void update_gyro(float *g, float dt) {
    assert(feed_count < 32);
    memcpy(received[feed_count], g, 3*sizeof(float));
    received_dt[feed_count++] = dt;
}
static const struct {void (*update_gyro)(float*,float); void (*get_gyro_bias)(float*);} backend = {update_gyro, get_bias};
static const typeof(backend) *sensor_fusion = &backend;
'''
state = source[source.index('static uint8_t gyro_oversample_n'):source.index('\n#if CONFIG_SENSOR_ACCEL_OVERSAMPLING > 1', source.index('static uint8_t gyro_oversample_n'))]
parts = [preamble, state, function('feed_calibrated_gyro'), '#if CONFIG_SENSOR_GYRO_OVERSAMPLING > 1']
parts += [function(name) for name in ('gyro_dq_mul', 'gyro_dq_accumulate_sample', 'gyro_dq_to_feed_gyro')]
parts += ['#endif', function('feed_gyro_sample')]
main = r'''
int main(int argc, char **argv) {
    int n = argc > 1 ? atoi(argv[1]) : 1;
    int count = 0;
    gyro_oversample_n = n;
#if CONFIG_SENSOR_GYRO_OVERSAMPLING > 1
    gyro_effective_time = gyro_actual_time * n;
    gyro_dq_acc[0] = 1.0f;
#endif
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
            feed_gyro_sample(raw, &count, i != 1);
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
        assert(raw_count == 4 && cal_count == 4 && rest_count == 4 && feed_count == 4);
        assert(total_gyro_samples == 3);
        puts("direct N1: calibrated samples, dt, diagnostics and acquisition counts match");
    } else {
        /* Two windows plus partial tail; analytical fixed-axis rotation. */
        for (int i=0;i<10;i++) {
            float desired[3] = {fusion_bias[0]+90.0f, fusion_bias[1], fusion_bias[2]};
            float raw[3] = {desired[0]/retained->gyroSensScale[0]+0.125f,
                desired[1]/retained->gyroSensScale[1]-0.0625f,
                desired[2]/retained->gyroSensScale[2]+0.03125f};
            feed_gyro_sample(raw, &count, true);
            assert(count == (i+1)/n);
        }
        assert(raw_count == 10 && cal_count == 10 && rest_count == 10 && total_gyro_samples == 10);
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
with tempfile.TemporaryDirectory(prefix='sensor-fast-') as directory:
    tmp = Path(directory)
    unit = tmp/'gyro.c'
    unit.write_text('\n\n'.join(parts)+main)
    # Always exercise N4, including against the pre-fix source, before N1.
    scenarios = [(4, 4), (1, 1), (4, 1)]
    failures = []
    for compiled, runtime in scenarios:
        binary = tmp/f'gyro-{compiled}'
        subprocess.run(shlex.split(os.environ.get('CC','cc')) + ['-std=gnu11','-Wall','-Wextra','-Werror','-Wno-unused-function','-g','-O1','-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-pie','-no-pie',f'-DCONFIG_SENSOR_GYRO_OVERSAMPLING={compiled}',str(unit),'-lm','-o',str(binary)],check=True)
        result = subprocess.run([str(binary),str(runtime)])
        if result.returncode:
            failures.append((compiled,runtime,result.returncode))
    if failures:
        raise SystemExit(f'Failed (compile N, runtime N, exit): {failures}')
