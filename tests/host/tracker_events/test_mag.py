#!/usr/bin/env python3
"""Exercise actual manual MAG and pose owners with sensor/storage/event leaves."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2]))


def function(source, name):
    match = re.search(rf"^(?:static )?(?:void|int|bool|float) {name}\([^;{{]*\)\s*\{{", source, re.M)
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


PRELUDE = r'''
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sensor/calibration/calibration.h"
#include "sensor/calibration/mag_common.h"
#include "connection/tracker_event_protocol.h"
#define CONFIG_CMSIS_DSP 0
#define DEBUG 0
#define K_MSEC(ms) (ms)
#define SYS_STATUS_CALIBRATION_RUNNING 1
#define SYS_LED_PRIORITY_SENSOR 1
#define SYS_LED_PATTERN_OFF 0
#define SYS_LED_PATTERN_LONG 1
#define SYS_LED_PATTERN_ON 2
#define SYS_LED_PATTERN_FLASH 3
#define SYS_LED_PATTERN_ONESHOT_PROGRESS 4
#define SYS_LED_PATTERN_ONESHOT_COMPLETE 5
#define WDT_CHANNEL_CALIBRATION 0
#define MAIN_MAG_BIAS_ID 1
#define LOG_INF(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define LOG_ERR(...) ((void)0)
#define printk(...) ((void)0)
static struct { double ata[100]; } mag_cal_workspace;
static struct { float magBAinv[4][3]; } storage;
static typeof(storage) *retained = &storage;
static bool running, admission, inject_motion;
static int led, mag_wait_error, solver_error;
static int64_t now;
static unsigned reads, pose_limit, motion_retries, ends, commits, solver_calls, token_reads;
static uint8_t outcome, reason, phase, completion_reason, pending_phase;
static uint16_t current_operation = 41, pending_operation;
static float pending_matrix[4][3];
uint16_t sensor_calibration_current_operation(void) { token_reads++; return current_operation; }
void cal_event_step(uint16_t op, uint8_t p, uint8_t detail) {
    assert(op == current_operation);
    pending_phase = p;
    if (p == CAL_PHASE_RETRY && detail == CAL_REASON_MOTION) {
        assert(ends == 0); motion_retries++;
    }
}
void cal_event_end(uint16_t op, uint8_t result, uint8_t p, uint8_t why) {
    assert(op == current_operation);
    if (ends) return; /* Same token terminal is idempotent, as in the core. */
    ends++; outcome=result; phase=p; reason=why ? why : completion_reason;
}
void cal_event_set_completion_reason(uint16_t op, uint8_t why) {
    assert(op == current_operation); completion_reason=why;
}
void tracker_events_notify(void) {}
static int64_t k_uptime_get(void) { return now; }
static void k_msleep(int ms) { now += ms; }
static void watchdog_feed(int channel) { (void)channel; }
static void wait_for_threads(void) {}
static bool get_status(int status) { (void)status; return running; }
static void set_status(int status, bool value) { (void)status; running=value; }
static void set_led(int pattern, int priority) { (void)priority; led=pattern; }
void sensor_calibration_samples_end(void) { admission=false; }
static void samples_begin(void) { admission=true; }
static int sensor_wait_mag(float m[3], int timeout) {
    (void)timeout; assert(admission); m[0]=.5f; m[1]=m[2]=0; return mag_wait_error;
}
static int sensor_wait_accel(float a[3], int timeout) {
    (void)timeout;
    unsigned i = reads++;
    unsigned pose = i / 700;
    if (pose >= pose_limit) pose=pose_limit-1;
    memset(a, 0, 3*sizeof(float));
    a[pose % 3] = pose < 3 ? 1 : -1;
    /* First stationary capture starts after 22 reads. One real vector jump
     * then stable data exercises retry without mocking the owner decision. */
    if (inject_motion && i == 22) { a[0]=0; a[1]=1; }
    now += 2;
    return 0;
}
void magneto_center_reset(mag_center_estimator_t *value) { memset(value,0,sizeof(*value)); }
static void magneto_online_snapshot_BAinv(float out[4][3]) { memcpy(out,storage.magBAinv,sizeof(storage.magBAinv)); }
static void magneto_online_replace_BAinv_and_reset(const float value[4][3], uint16_t op) {
    assert(op == current_operation); memcpy(pending_matrix,value,sizeof(pending_matrix)); pending_operation=op;
}
static void sensor_fusion_reset_mag_ref(void) {}
static void sensor_mag_ref_reset(void) {}
static void sensor_refresh_sensor_ids(void) {}
int sensor_calibration_validate_mag(float value[][3], bool restore) { (void)value; (void)restore; return 0; }
static int sys_write(int id, void *dst, const void *value, size_t size) {
    (void)id; memcpy(dst,value,size); return 0;
}
void sensor_calibration_identity_accel(float matrix[4][3]) {
    memset(matrix,0,12*sizeof(float)); matrix[1][0]=matrix[2][1]=matrix[3][2]=1;
}
int sensor_calibration_commit_accel(const float matrix[4][3], uint16_t op) {
    assert(pending_phase == CAL_PHASE_APPLY_PENDING && ends == 0);
    commits++; pending_operation=op; memcpy(pending_matrix,matrix,sizeof(pending_matrix)); return 0;
}
static void magneto_sample(double x,double y,double z,double *ata,double *norm,double *count) {
    (void)x;(void)y;(void)z;(void)ata; *norm += 1; *count += 1;
}
static int magneto_current_calibration(float matrix[4][3],double *ata,double norm,double count) {
    (void)ata;(void)norm; assert(count > 0); solver_calls++;
    sensor_calibration_identity_accel(matrix); return solver_error;
}
'''

HELPERS = r'''
static void sensor_sample_mag_magneto_sample(const float m[3]) {
    (void)m; sample_count++; /* Acquisition leaf: this test never starts MAG fitting. */
}
static void reset_case(void) {
    memset(&storage,0,sizeof(storage)); magneto_reset();
    running=admission=inject_motion=false; led=0; now=0;
    mag_wait_error=solver_error=0; reads=0; pose_limit=6;
    motion_retries=ends=commits=solver_calls=token_reads=0;
    outcome=reason=phase=completion_reason=pending_phase=0; pending_operation=0;
}
'''

TESTS = r'''
int main(void) {
    reset_case();
    samples_begin(); magneto_progress=0x80; running=true; led=SYS_LED_PATTERN_LONG;
    mag_wait_error=-ETIMEDOUT;
    assert(sensor_calibrate_mag() == -1);
    assert(token_reads == 1 && ends == 1 && outcome == CAL_OUTCOME_FAILED);
    assert(reason == CAL_REASON_SAMPLE_TIMEOUT && phase == CAL_PHASE_COLLECT);
    assert(!admission && !running && magneto_progress == 0 && led == SYS_LED_PATTERN_OFF);
    /* Next admitted session can collect rather than inheriting timeout state. */
    current_operation++;
    ends=token_reads=0; mag_wait_error=0;
    samples_begin(); magneto_progress=0x80;
    assert(sensor_calibrate_mag() == 1);
    assert(token_reads == 1 && ends == 0 && admission && running && sample_count == 1);

    reset_case(); inject_motion=true;
    float matrix[4][3]; int captured=0;
    assert(sensor_6_sideBias(matrix,&captured) == -3);
    assert(captured == 6 && motion_retries == 1 && ends == 0 && token_reads == 1);

    reset_case(); pose_limit=5;
    sensor_calibrate_6_side();
    assert(ends == 1 && outcome == CAL_OUTCOME_FAILED && reason == CAL_REASON_INSUFFICIENT_SAMPLES);
    assert(commits == 0 && solver_calls == 0);

    reset_case(); pose_limit=6;
    sensor_calibrate_6_side();
    assert(ends == 0 && commits == 1 && solver_calls == 1);
    assert(pending_operation == current_operation && completion_reason == CAL_REASON_PARTIAL);
    /* Applying belongs to imu_calibration's independently tested frame owner;
     * the pose wrapper may queue a partial candidate, never complete it here. */

    reset_case(); pose_limit=6; solver_error=-EDOM;
    sensor_calibrate_6_side();
    assert(ends == 1 && outcome == CAL_OUTCOME_FAILED && reason == CAL_REASON_FIT_ERROR);
    assert(commits == 0);
    puts("manual MAG timeout recovery / 18-pose retry and partial handoff: PASS");
    return 0;
}
'''


def main():
    cal = ROOT / "src/sensor/calibration"
    mag = (cal / "cal_mag.c").read_text()
    imu = (cal / "cal_imu.c").read_text()
    bias = (cal / "bias_collect.c").read_text()
    util = (ROOT / "src/util.c").read_text()
    state = mag[mag.index("uint8_t magneto_progress;"):mag.index("static void magneto_update_dir_range")]
    constants = "\n".join(line for line in mag.splitlines() if line.startswith(("#define CALIB_", "#define MIN_ORIENTATION_", "#define THRESHOLD_ACC", "#define SAMPLES_PER_")))
    constants += "\n" + next(line for line in imu.splitlines() if line.startswith("#define CALIB_MIN_POSES_FOR_PARTIAL"))
    source = "\n\n".join((
        PRELUDE, state, constants, "typedef struct { float x,y,z; } Vector3;",
        function(util,"v_diff_mag"), function(util,"v_epsilon"),
        function(mag,"magneto_reset"), function(mag,"magneto_min_dir_range"), HELPERS,
        function(mag,"manual_finish"), function(mag,"sensor_calibrate_mag"),
        function(bias,"isAccRest"), function(mag,"sensor_6_sideBias"),
        function(imu,"imu_step"), function(imu,"imu_failed"), function(imu,"sensor_calibrate_6_side"), TESTS,
    ))
    with tempfile.TemporaryDirectory(prefix="cal-mag-events-") as directory:
        path = Path(directory)
        (path / "test.c").write_text(source)
        cc = shlex.split(os.environ.get("CC", "cc"))
        subprocess.run(cc + ["-std=gnu11", "-Wall", "-Wextra", "-Werror", "-I", str(ROOT / "src"), str(path / "test.c"), "-lm", "-o", str(path / "test")], check=True)
        subprocess.run([str(path / "test")], check=True)


if __name__ == "__main__":
    main()
