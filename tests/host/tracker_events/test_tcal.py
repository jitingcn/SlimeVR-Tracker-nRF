#!/usr/bin/env python3
"""Exercise production D_offset lifecycles with hardware and event sinks replaced."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2]))
source = (ROOT / "src/sensor/calibration/tcal_runtime.c").read_text()


def function(name):
    match = re.search(rf"^(?:static )?(?:void|int|bool) {name}\([^;{{]*\)\s*\{{", source, re.M)
    if match is None:
        raise ValueError(name)
    tokens = re.compile(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]', re.S)
    depth = 0
    for token in tokens.finditer(source, source.index("{", match.start())):
        if token.group() == "{":
            depth += 1
        elif token.group() == "}":
            depth -= 1
            if depth == 0:
                return source[match.start():token.end()]
    raise ValueError(name)


runtime_header = (ROOT / "src/sensor/calibration/tcal_runtime.h").read_text()
constants = "\n".join(line for line in runtime_header.splitlines()
                      if line.startswith(("#define BOOT_CAL_", "#define RUNTIME_CAL_")))
state = source[source.index("static bool runtime_cal_enabled"):source.index("static bool tcal_compensation_enabled")]
preamble = r'''
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CONFIG_SENSOR_USE_TCAL 1
#include "sensor/calibration/calibration.h"
#include "sensor/calibration/bias_collect.h"
#include "sensor/calibration/tcal_mls_lut.h"
#include "connection/tracker_event_protocol.h"
#define LOG_INF(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define LOG_ERR(...) ((void)0)
#define LOG_DBG(...) ((void)0)
#define TCAL_BUFFER_SIZE 16
static struct {
    struct { bool enabled, completed, doffset_valid; unsigned attempt_count; float doffset[3]; } bootCalState;
    struct { bool valid; unsigned count; } tempCalState;
    struct { float temp; } tempCalPoints[TCAL_BUFFER_SIZE];
} retained_data;
static typeof(retained_data) *retained = &retained_data;
static int64_t clock_ms = 10000;
static float temperature = 25.0f;
static bool stationary = true;
static int collect_error, lookup_error, request_slot;
static unsigned event_count, terminal_count, begin_count, token_reads, request_count;
static uint8_t last_outcome, last_phase, last_reason, last_kind;
static int last_request;
static enum cal_request_origin last_origin;
static uint16_t current_operation = 41;
static int64_t k_uptime_get(void) { return clock_ms; }
static int64_t system_uptime_since_boot_ms(void) { return clock_ms; }
static void k_msleep(int ms) { clock_ms += ms; }
static float sensor_get_current_imu_temperature(void) { return temperature; }
bool wait_for_motion(bool motion, int count) { (void)motion; (void)count; return stationary; }
void sensor_request_fusion_bias_reset(void) {}
uint16_t sensor_calibration_current_operation(void) { token_reads++; return current_operation; }
int sensor_calibration_request(int id, enum cal_request_origin origin) {
    if (id == CAL_REQUEST_QUERY) return request_slot;
    request_count++;
    last_request = id;
    last_origin = origin;
    if (request_slot) return -1;
    request_slot = id;
    return 0;
}
int sensor_tcal_mls_lookup(float temp, float bias[3]) {
    (void)temp;
    bias[0] = 0.1f; bias[1] = 0.2f; bias[2] = 0.3f;
    return lookup_error;
}
static int sensor_boot_bias_collect(float *bias, float *temp) {
    bias[0] = 0.2f; bias[1] = 0.1f; bias[2] = 0.5f;
    *temp = temperature;
    return collect_error;
}
static int sensor_runtime_bias_collect(float *bias, float *temp) {
    return sensor_boot_bias_collect(bias, temp);
}
uint16_t cal_event_begin(uint8_t kind, uint8_t phase, uint8_t detail) {
    assert(kind == (CAL_KIND_TCAL_BOOT | CAL_EVENT_ORIGIN_AUTO));
    assert(phase == CAL_PHASE_WAIT_STILL && detail == 0);
    begin_count++; event_count++; last_kind = kind;
    return current_operation;
}
void cal_event_step(uint16_t op, uint8_t phase, uint8_t detail) {
    (void)phase; (void)detail;
    assert(op == current_operation);
    if (op) event_count++;
}
void cal_event_end(uint16_t op, uint8_t outcome, uint8_t phase, uint8_t reason) {
    assert(op == current_operation);
    if (!op) return;
    /* Observe live validity at the event boundary, not just after the call. */
    if (outcome == CAL_OUTCOME_SUCCESS) assert(retained->bootCalState.doffset_valid);
    event_count++; terminal_count++;
    last_outcome = outcome; last_phase = phase; last_reason = reason;
}
void tracker_events_notify(void) {}
'''
main = r'''
static void quality_points(void) {
    retained->tempCalState.count = 5;
    retained->tempCalState.valid = true;
    for (unsigned i = 0; i < 5; ++i) retained->tempCalPoints[i].temp = 23.0f + i;
}
static void expect_terminal(uint8_t outcome, uint8_t phase, uint8_t reason) {
    assert(terminal_count == 1);
    assert(last_outcome == outcome && last_phase == phase && last_reason == reason);
}
int main(int argc, char **argv) {
    assert(argc == 2);
    const char *scenario = argv[1];
    retained->bootCalState.enabled = true;
    quality_points();
    if (!strcmp(scenario, "boot_skip") || !strcmp(scenario, "runtime_skip")) {
        retained->tempCalState.count = 4;
        retained->tempCalPoints[4].temp = 0;
        int result = !strcmp(scenario, "boot_skip") ? sensor_perform_boot_calibration() : sensor_perform_runtime_calibration();
        assert(result == 0 && !retained->bootCalState.doffset_valid);
        assert(token_reads == 1);
        expect_terminal(CAL_OUTCOME_SKIPPED, CAL_PHASE_VALIDATE, CAL_REASON_NO_TCAL_COVERAGE);
    } else if (!strcmp(scenario, "boot_applied") || !strcmp(scenario, "runtime_applied")) {
        int result = !strcmp(scenario, "boot_applied") ? sensor_perform_boot_calibration() : sensor_perform_runtime_calibration();
        assert(result == 0 && token_reads == 1);
        assert(fabsf(retained->bootCalState.doffset[0] - 0.1f) < 0.00001f);
        assert(fabsf(retained->bootCalState.doffset[1] + 0.1f) < 0.00001f);
        assert(fabsf(retained->bootCalState.doffset[2] - 0.2f) < 0.00001f);
        expect_terminal(CAL_OUTCOME_SUCCESS, CAL_PHASE_APPLIED, CAL_REASON_NONE);
    } else if (!strcmp(scenario, "fit_error")) {
        lookup_error = -1;
        retained->bootCalState.doffset_valid = true;
        assert(sensor_perform_boot_calibration() != 0);
        assert(!retained->bootCalState.doffset_valid);
        expect_terminal(CAL_OUTCOME_FAILED, CAL_PHASE_VALIDATE, CAL_REASON_FIT_ERROR);
    } else if (!strcmp(scenario, "motion")) {
        stationary = false;
        assert(sensor_perform_boot_calibration() != 0);
        expect_terminal(CAL_OUTCOME_FAILED, CAL_PHASE_WAIT_STILL, CAL_REASON_MOTION);
    } else if (!strcmp(scenario, "temperature")) {
        temperature = NAN;
        assert(sensor_perform_runtime_calibration() != 0);
        expect_terminal(CAL_OUTCOME_FAILED, CAL_PHASE_WAIT_STILL, CAL_REASON_TEMPERATURE);
    } else if (!strcmp(scenario, "collection_reasons")) {
        const int errors[] = {-2, BIAS_COLLECT_INSUFFICIENT_SAMPLES};
        const uint8_t reasons[] = {CAL_REASON_SAMPLE_TIMEOUT, CAL_REASON_INSUFFICIENT_SAMPLES};
        for (unsigned i = 0; i < 2; ++i) {
            collect_error = errors[i];
            terminal_count = 0;
            assert(sensor_perform_boot_calibration() == errors[i]);
            expect_terminal(CAL_OUTCOME_FAILED, CAL_PHASE_COLLECT, reasons[i]);
            terminal_count = 0;
            assert(sensor_perform_runtime_calibration() == errors[i]);
            expect_terminal(CAL_OUTCOME_FAILED, CAL_PHASE_COLLECT, reasons[i]);
        }
    } else if (!strcmp(scenario, "fallback_busy")) {
        retained->tempCalState.count = 0;
        retained->bootCalState.attempt_count = BOOT_CAL_MAX_ATTEMPTS - 1;
        request_slot = CAL_REQUEST_TCAL_BOOT;
        collect_error = -1;
        assert(sensor_perform_boot_calibration() == collect_error);
        assert(last_request == CAL_REQUEST_IMU && last_origin == CAL_REQUEST_AUTO_SILENT);
        assert(request_slot == CAL_REQUEST_TCAL_BOOT && retained->bootCalState.completed);
        expect_terminal(CAL_OUTCOME_FAILED, CAL_PHASE_COLLECT, CAL_REASON_MOTION);
    } else if (!strcmp(scenario, "gate_coverage") || !strcmp(scenario, "gate_expired")) {
        bool expired = !strcmp(scenario, "gate_expired");
        if (expired) clock_ms = BOOT_CAL_TIME_WINDOW_END_MS;
        else { retained->tempCalState.count = 0; memset(retained->tempCalPoints, 0, sizeof(retained->tempCalPoints)); }
        for (unsigned i = 0; i < 1000; ++i) sensor_tcal_boot_calibration_check();
        assert(retained->bootCalState.completed && begin_count == 1);
        assert(last_kind == (CAL_KIND_TCAL_BOOT | CAL_EVENT_ORIGIN_AUTO));
        expect_terminal(CAL_OUTCOME_SKIPPED, CAL_PHASE_WAIT_STILL, expired ? CAL_REASON_EXPIRED : CAL_REASON_NO_TCAL_COVERAGE);
    } else if (!strcmp(scenario, "gate_active")) {
        request_slot = CAL_REQUEST_TCAL_BOOT;
        clock_ms = BOOT_CAL_TIME_WINDOW_END_MS;
        sensor_tcal_boot_calibration_check();
        assert(event_count == 0);
        assert(sensor_perform_boot_calibration() == 0);
        expect_terminal(CAL_OUTCOME_SUCCESS, CAL_PHASE_APPLIED, CAL_REASON_NONE);
    } else if (!strcmp(scenario, "auto_busy")) {
        request_slot = CAL_REQUEST_IMU;
        for (unsigned i = 0; i < 1000; ++i) sensor_tcal_boot_calibration_check();
        assert(!retained->bootCalState.completed && event_count == 0);
        request_slot = 0;
        sensor_tcal_boot_calibration_check();
        assert(last_request == CAL_REQUEST_TCAL_BOOT && last_origin == CAL_REQUEST_AUTO);
    } else if (!strcmp(scenario, "supplement_busy")) {
        retained->tempCalState.count = 0;
        tcal_auto_calibration_enabled = true;
        clock_ms = 100000;
        request_slot = CAL_REQUEST_IMU;
        for (unsigned i = 0; i < 1000; ++i) sensor_tcal_check_auto_calibration(temperature);
        assert(event_count == 0 && request_count == 1000);
        request_slot = 0;
        sensor_tcal_check_auto_calibration(temperature);
        assert(request_slot == CAL_REQUEST_IMU && last_origin == CAL_REQUEST_AUTO_SILENT);
    } else if (!strcmp(scenario, "runtime_disabled")) {
        retained->bootCalState.completed = true;
        for (unsigned i = 0; i < 1000; ++i) { clock_ms += 1000; sensor_runtime_calibration_check(true); }
        assert(request_count == 0 && event_count == 0);
    } else {
        assert(!"unknown scenario");
    }
    printf("tcal lifecycle: %s passed\n", scenario);
    return 0;
}
'''
parts = [preamble, constants, state]
parts.extend(function(name) for name in (
    "sensor_tcal_assess_quality", "sensor_tcal_calculate_doffset", "sensor_boot_cal_abandon",
    "sensor_tcal_boot_calibration_check", "sensor_perform_boot_calibration",
    "sensor_perform_runtime_calibration", "sensor_runtime_calibration_check",
    "sensor_tcal_check_auto_calibration",
))
scenarios = (
    "boot_skip", "runtime_skip", "boot_applied", "runtime_applied", "fit_error", "motion",
    "temperature", "fallback_busy", "gate_coverage", "gate_expired", "gate_active",
    "auto_busy", "supplement_busy", "runtime_disabled", "collection_reasons",
)
with tempfile.TemporaryDirectory(prefix="tcal-events-") as directory:
    tmp = Path(directory)
    unit = tmp / "tcal.c"
    unit.write_text("\n\n".join(parts) + main)
    binary = tmp / "tcal"
    subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
        "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-variable",
        "-g", "-O1", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
        "-fno-pie", "-no-pie", "-I", str(ROOT / "src"), str(unit), "-lm", "-o", str(binary),
    ], check=True)
    for scenario in scenarios:
        subprocess.run([str(binary), scenario], check=True)
