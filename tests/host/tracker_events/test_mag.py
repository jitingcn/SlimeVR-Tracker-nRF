#!/usr/bin/env python3
"""Exercise actual manual MAG and pose owners with sensor/storage/event leaves."""
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
ROOT = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2]))


def function(source, name):
    return extract_block(source, rf"^(?:static )?(?:void|int|bool|float) {re.escape(name)}\([^;{{]*\)\s*\{{")


PRELUDE = r'''
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define CONFIG_SENSOR_USE_TCAL 1
#define CONFIG_SENSOR_TCAL_HEATED 0
#define CONFIG_SENSOR_POLY_TEMP_MIN 20
#define CONFIG_SENSOR_POLY_STEPS_PER_DEGREE 1
#define TCAL_BUFFER_SIZE 16
#define IS_ENABLED(value) 0
#include "sensor/calibration/calibration.h"
#include "sensor/calibration/bias_collect.h"
#include "sensor/calibration/tcal_mls_lut.h"
#include "sensor/calibration/mag_common.h"
#include "connection/tracker_event_protocol.h"
#define CONFIG_CMSIS_DSP 0
#define DEBUG 0
#define K_MSEC(ms) (ms)
#define SYS_STATUS_CALIBRATION_RUNNING 1
#define WDT_CHANNEL_CALIBRATION 0
#define MAIN_MAG_BIAS_ID 1
#define LOG_INF(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define LOG_ERR(...) ((void)0)
#define LOG_DBG(...) ((void)0)
#define MAIN_GYRO_TEMP_ID 2
#define printk(...) ((void)0)
static struct { double ata[100]; } mag_cal_workspace;
static struct {
    float magBAinv[4][3];
    float gyroTemp;
    struct { bool valid; unsigned count; } tempCalState;
    struct { float temp, bias[3]; } tempCalPoints[TCAL_BUFFER_SIZE];
} storage;
static typeof(storage) *retained = &storage;
static bool running, admission, inject_motion;
static int mag_wait_error, solver_error;
static int64_t now;
static unsigned reads, pose_limit, motion_retries, ends, commits, solver_calls, token_reads;
static uint8_t outcome, reason, phase, completion_reason, pending_phase;
static uint16_t current_operation = 41, pending_operation;
static float pending_matrix[4][3];
static bool pending_partial;
/* Semantic LED leaves: submission is observed per semantic. */
static unsigned led_results[LED_SEMANTIC_COUNT];
static unsigned led_requests[LED_SEMANTIC_COUNT];
static uint32_t led_identity;
uint32_t led_request_id(void) { return ++led_identity; }
uint32_t led_event_id(void) { return ++led_identity; }
struct led_token led_begin(enum led_owner owner, uint32_t request_id) {
    return (struct led_token){owner, ++led_identity, request_id};
}
enum led_admission led_state(struct led_token token, uint32_t revision, enum led_semantic semantic) {
    (void)token; (void)revision; (void)semantic; return LED_ADMITTED;
}
enum led_admission led_result(struct led_token token, uint32_t event_id, enum led_semantic semantic) {
    (void)event_id;
    if (token.session) led_results[semantic]++;
    return LED_ADMITTED;
}
enum led_admission led_request_event(enum led_owner owner, uint32_t request_id, uint32_t event_id,
                                     enum led_semantic semantic) {
    (void)owner; (void)request_id; (void)event_id;
    led_requests[semantic]++;
    return LED_ADMITTED;
}
uint16_t sensor_calibration_current_operation(void) { token_reads++; return current_operation; }
struct led_token sensor_calibration_current_feedback(void) {
    /* Admission captures the owner token once; a silent request carries none. */
    return current_operation ? (struct led_token){LED_OWNER_MAG, current_operation, 0} : (struct led_token){0};
}
int sensor_calibration_current_storage_error(void) { return 0; }
/* Request-generation guard: cal_imu captures before collection; the owner
 * validates it at commit. */
static bool generation_valid_result = true;
uint32_t sensor_calibration_current_generation(void) { return 1; }
bool sensor_calibration_generation_valid(uint32_t generation) {
    (void)generation; return generation_valid_result;
}
void sensor_calibration_invalidate_requests(void) {}
/* The manual mag fit write is guarded under a warm storage transaction. */
static unsigned warm_begin, warm_end, warm_depth;
void sys_warm_transaction_begin(void) { warm_begin++; warm_depth++; }
void sys_warm_transaction_mark(uint16_t id, const void *data, size_t size) {
    (void)id; (void)data; (void)size;
}
void sys_warm_transaction_end(bool schedule) {
    (void)schedule; assert(warm_depth); warm_end++; warm_depth--;
}
/* Late storage bookkeeping from the outer warm transaction; the manual
 * request's captured generation must still own the record. */
static uint32_t recorded_error_generation;
static int recorded_error;
static unsigned recorded_errors;
void sensor_calibration_record_storage_error(uint32_t generation, int error) {
    if (error >= 0) return; /* The private owner records only failed receipts. */
    assert(warm_depth); /* Bookkeeping must land before its storage barrier ends. */
    assert(generation == sensor_calibration_current_generation());
    recorded_error_generation = generation; recorded_error = error; recorded_errors++;
}
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
static void magneto_online_replace_BAinv_and_reset(const float value[4][3], uint16_t op,
                                                  struct led_token feedback) {
    assert(op == current_operation); (void)feedback;
    memcpy(pending_matrix,value,sizeof(pending_matrix)); pending_operation=op;
}
static void magneto_online_feedback_storage(struct led_token feedback, int result) {
    (void)feedback; (void)result; /* Warm/flash receipt is owned by the storage leaf. */
}
static void sensor_fusion_reset_mag_ref(void) {}
static void sensor_mag_ref_reset(void) {}
static void sensor_refresh_sensor_ids(void) {}
int sensor_calibration_validate_mag(float value[][3], bool restore) { (void)value; (void)restore; return 0; }
static int sys_write_error, warm_flush_error;
static unsigned sys_writes, warm_flushes;
static float durable_gyro_temp;
static int sys_write(int id, void *dst, const void *value, size_t size) {
    memmove(dst, value, size);
    sys_writes++;
    if (id == MAIN_GYRO_TEMP_ID && !sys_write_error) {
        assert(warm_depth && size == sizeof(durable_gyro_temp));
        memcpy(&durable_gyro_temp, value, size);
    }
    return sys_write_error;
}
int sys_flush_warm(void) {
    assert(warm_depth); warm_flushes++; return warm_flush_error;
}
void sensor_calibration_identity_accel(float matrix[4][3]) {
    memset(matrix,0,12*sizeof(float)); matrix[1][0]=matrix[2][1]=matrix[3][2]=1;
}
int sensor_calibration_commit_accel(const float matrix[4][3], uint16_t op,
                                   struct led_token feedback, bool partial, uint32_t generation) {
    assert(pending_phase == CAL_PHASE_APPLY_PENDING && ends == 0);
    assert(feedback.session != 0); /* Manual requests always carry an owner token. */
    assert(generation == 1); /* Actual worker captured the pre-collection generation. */
    commits++; pending_operation=op; pending_partial=partial;
    memcpy(pending_matrix,matrix,sizeof(pending_matrix)); return 0;
}
static void magneto_sample(double x,double y,double z,double *ata,double *norm,double *count) {
    (void)x;(void)y;(void)z;(void)ata; *norm += 1; *count += 1;
}
static int magneto_current_calibration(float matrix[4][3],double *ata,double norm,double count) {
    (void)ata;(void)norm; assert(count > 0); solver_calls++;
    sensor_calibration_identity_accel(matrix); return solver_error;
}
'''

IMU_LEAVES = r'''
tcal_temp_direction_t tcal_current_direction;
float tcal_direction_ref_temp, runtime_cal_last_temp;
static unsigned tcal_lock_depth, model_updates, measured_bias_marks, state_updates;
static bool tcal_auto_enabled;
static float measured_accel[3], measured_gyro[3], measured_temp;
static float pending_accel_bias[3], pending_gyro_bias[3];
static bool pending_persist_gyro;
static int pending_prior_error;
static uint32_t pending_generation;
static struct led_token pending_feedback;
bool wait_for_motion(bool motion, int samples) { (void)motion; (void)samples; return true; }
int sensor_offsetBias(float *a, float *g, float *temp, float *range) {
    memcpy(a, measured_accel, sizeof(measured_accel));
    memcpy(g, measured_gyro, sizeof(measured_gyro));
    *temp = measured_temp; *range = 0.125f;
    return 0;
}
bool sensor_tcal_get_auto_calibration(void) { return tcal_auto_enabled; }
void sensor_tcal_lock(void) { tcal_lock_depth++; }
void sensor_tcal_unlock(void) { assert(tcal_lock_depth); tcal_lock_depth--; }
void sensor_tcal_refresh_model(void) { assert(tcal_lock_depth); model_updates++; }
void sensor_tcal_mark_measured_bias(void) { assert(tcal_lock_depth); measured_bias_marks++; }
void update_tcal_state(void) { assert(warm_depth && !tcal_lock_depth); state_updates++; }
int sensor_calibration_commit_bias(const float a[3], const float g[3], bool persist,
                                  uint16_t op, struct led_token feedback, int prior_error,
                                  uint32_t generation) {
    assert(pending_phase == CAL_PHASE_APPLY_PENDING && ends == 0 && warm_depth);
    commits++;
    memcpy(pending_accel_bias, a, sizeof(pending_accel_bias));
    memcpy(pending_gyro_bias, g, sizeof(pending_gyro_bias));
    pending_persist_gyro = persist; pending_operation = op;
    pending_feedback = feedback; pending_prior_error = prior_error; pending_generation = generation;
    return 0;
}
'''

HELPERS = r'''
static void sensor_sample_mag_magneto_sample(const float m[3]) {
    (void)m; sample_count++; /* Acquisition leaf: this test never starts MAG fitting. */
}
static void reset_case(void) {
    memset(&storage,0,sizeof(storage)); magneto_reset();
    running=admission=inject_motion=false; now=0;
    mag_wait_error=solver_error=0; reads=0; pose_limit=6;
    motion_retries=ends=commits=solver_calls=token_reads=0;
    outcome=reason=phase=completion_reason=pending_phase=0; pending_operation=0;
    pending_partial=false; memset(led_results,0,sizeof(led_results));
    memset(led_requests,0,sizeof(led_requests));
    warm_begin=warm_end=warm_depth=0;
    recorded_error_generation=0; recorded_error=0; recorded_errors=0;
    sys_write_error=warm_flush_error=0; sys_writes=warm_flushes=0; durable_gyro_temp=0;
    tcal_current_direction=TCAL_DIR_UNKNOWN; tcal_direction_ref_temp=NAN;
    tcal_lock_depth=model_updates=measured_bias_marks=state_updates=0;
    tcal_auto_enabled=true; measured_temp=25.0f;
    const float accel[3]={.125f,-.25f,.5f}, gyro[3]={2.0f,-4.0f,.5f};
    memcpy(measured_accel,accel,sizeof(accel)); memcpy(measured_gyro,gyro,sizeof(gyro));
    memset(pending_accel_bias,0,sizeof(pending_accel_bias));
    memset(pending_gyro_bias,0,sizeof(pending_gyro_bias));
    pending_persist_gyro=false; pending_prior_error=0; pending_generation=0;
    pending_feedback=(struct led_token){0}; generation_valid_result=true;
}
'''

TESTS = r'''
static void expect_vector(const float actual[3], const float expected[3]) {
    for (unsigned axis=0; axis<3; axis++) assert(fabsf(actual[axis]-expected[axis]) < 0.000001f);
}
static void existing_tcal_point(void) {
    unsigned slot=TEMP_TO_IDX(measured_temp);
    retained->tempCalState.count=1;
    retained->tempCalState.valid=true;
    retained->tempCalPoints[slot].temp=measured_temp;
    const float previous[3]={10.0f,2.0f,-3.5f};
    memcpy(retained->tempCalPoints[slot].bias,previous,sizeof(previous));
    tcal_quality_t quality;
    assert(!sensor_tcal_assess_quality(measured_temp,&quality)); /* Sparse coverage forces an update. */
}
static void test_imu_tcal_preserves_measured_candidate(void) {
    static const struct {
        float reference;
        tcal_temp_direction_t direction;
        float blended[3];
    } cases[] = {
        {24.0f,TCAL_DIR_RISING,{4.4f,-2.2f,-.7f}},
        {26.0f,TCAL_DIR_FALLING,{8.8f,1.1f,-2.9f}},
        {25.0f,TCAL_DIR_UNKNOWN,{6.0f,-1.0f,-1.5f}},
    };
    for (unsigned i=0; i<sizeof(cases)/sizeof(cases[0]); i++) {
        reset_case(); existing_tcal_point();
        tcal_direction_ref_temp=cases[i].reference;
        sensor_calibrate_imu();
        assert(commits==1 && ends==0 && token_reads==1);
        assert(pending_operation==current_operation && pending_feedback.session==current_operation);
        assert(pending_generation==1 && !pending_persist_gyro && pending_prior_error==0);
        expect_vector(pending_accel_bias,measured_accel);
        expect_vector(pending_gyro_bias,measured_gyro); /* Captured before the worker blends its local vector. */
        expect_vector(retained->tempCalPoints[TEMP_TO_IDX(measured_temp)].bias,cases[i].blended);
        assert(tcal_current_direction==cases[i].direction && tcal_direction_ref_temp==measured_temp);
        assert(retained->tempCalState.count==1 && !retained->tempCalState.valid);
        assert(retained->tempCalPoints[TEMP_TO_IDX(measured_temp)].temp==measured_temp);
        assert(retained->gyroTemp==measured_temp && durable_gyro_temp==measured_temp);
        assert(model_updates==1 && measured_bias_marks==1 && state_updates==1);
        assert(warm_begin==warm_end && warm_depth==0 && tcal_lock_depth==0);
        assert(led_results[LED_SUCCESS]==0 && led_results[LED_PARTIAL]==0);
        assert(led_results[LED_APPLIED_NOT_SAVED]==0 && led_requests[LED_SUCCESS]==0);
    }
}
static void test_imu_tcal_late_storage_failure(void) {
    reset_case(); existing_tcal_point();
    tcal_direction_ref_temp=24.0f;
    warm_flush_error=-ENOSPC;
    sensor_calibrate_imu();
    assert(commits==1 && pending_prior_error==0); /* The error did not exist at commit. */
    assert(sys_writes==1 && warm_flushes==1 && durable_gyro_temp==measured_temp);
    assert(recorded_errors==1 && recorded_error_generation==pending_generation);
    assert(recorded_error==-ENOSPC);
    expect_vector(pending_gyro_bias,measured_gyro);
    const float blended[3]={4.4f,-2.2f,-.7f};
    expect_vector(retained->tempCalPoints[TEMP_TO_IDX(measured_temp)].bias,blended);
    assert(warm_begin==warm_end && warm_depth==0 && tcal_lock_depth==0 && ends==0);
    /* This extracted worker does not link the frame/persistence owner: it may
     * record the late failure, but must not fabricate a terminal receipt. */
    assert(led_results[LED_SUCCESS]==0 && led_requests[LED_SUCCESS]==0);
    assert(led_results[LED_PARTIAL]==0 && led_results[LED_APPLIED_NOT_SAVED]==0);
}
int main(void) {
    reset_case();
    samples_begin(); magneto_progress=0x80; running=true;
    mag_wait_error=-ETIMEDOUT;
    assert(sensor_calibrate_mag() == -1);
    assert(token_reads == 1 && ends == 1 && outcome == CAL_OUTCOME_FAILED);
    assert(reason == CAL_REASON_SAMPLE_TIMEOUT && phase == CAL_PHASE_COLLECT);
    assert(!admission && !running && magneto_progress == 0);
    /* The abandoned manual session submits its own failure receipt exactly once. */
    assert(led_results[LED_FAILED] == 1);
    assert(led_results[LED_SUCCESS] == 0);
    /* Next admitted session can collect rather than inheriting timeout state. */
    current_operation++;
    ends=token_reads=0; mag_wait_error=0;
    samples_begin(); magneto_progress=0x80;
    assert(sensor_calibrate_mag() == 1);
    assert(token_reads == 1 && ends == 0 && admission && running && sample_count == 1);

    reset_case(); inject_motion=true;
    float matrix[4][3]; int captured=0;
    assert(sensor_calibration_collect_accel_poses(matrix,&captured) == -3);
    assert(captured == 6 && motion_retries == 1 && ends == 0 && token_reads == 1);

    reset_case(); pose_limit=5;
    sensor_calibrate_accel();
    assert(ends == 1 && outcome == CAL_OUTCOME_FAILED && reason == CAL_REASON_INSUFFICIENT_SAMPLES);
    assert(commits == 0 && solver_calls == 0);

    reset_case(); pose_limit=6;
    sensor_calibrate_accel();
    assert(ends == 0 && commits == 1 && solver_calls == 1);
    assert(pending_operation == current_operation && completion_reason == CAL_REASON_PARTIAL);
    assert(pending_partial); /* The pose wrapper must hand partiality to the owner. */
    /* Applying belongs to imu_calibration's independently tested frame owner;
     * the pose wrapper may queue a partial candidate, never complete it here. */

    reset_case(); pose_limit=6; solver_error=-EDOM;
    sensor_calibrate_accel();
    assert(ends == 1 && outcome == CAL_OUTCOME_FAILED && reason == CAL_REASON_FIT_ERROR);
    assert(commits == 0);
    test_imu_tcal_preserves_measured_candidate();
    test_imu_tcal_late_storage_failure();
    puts("manual MAG recovery / partial handoff / IMU T-Cal measured candidate and late storage receipt: PASS");
    return 0;
}
'''


def main():
    cal = ROOT / "src/sensor/calibration"
    mag = (cal / "cal_mag.c").read_text()
    imu = (cal / "cal_imu.c").read_text()
    bias = (cal / "bias_collect.c").read_text()
    util = (ROOT / "src/util.c").read_text()
    runtime = (cal / "tcal_runtime.c").read_text()
    runtime_header = (cal / "tcal_runtime.h").read_text()
    tcal_constants = "\n".join(line for line in runtime_header.splitlines()
                              if line.startswith(("#define TEMP_TO_IDX", "#define BOOT_CAL_MIN_CURVE_POINTS",
                                                  "#define TCAL_HYSTERESIS_")))
    tcal_direction = runtime_header[runtime_header.index("typedef enum {"):
                                   runtime_header.index("void update_tcal_state")]
    state = mag[mag.index("uint8_t magneto_progress;"):mag.index("static void magneto_update_dir_range")]
    constants = "\n".join(line for line in mag.splitlines() if line.startswith(("#define CALIB_", "#define MIN_ORIENTATION_", "#define THRESHOLD_ACC", "#define SAMPLES_PER_")))
    constants += "\n" + next(line for line in imu.splitlines() if line.startswith("#define CALIB_MIN_POSES_FOR_PARTIAL"))
    source = "\n\n".join((
        PRELUDE, tcal_constants, tcal_direction, IMU_LEAVES,
        state, constants, "typedef struct { float x,y,z; } Vector3;",
        function(util,"v_diff_mag"), function(util,"v_epsilon"),
        function(mag,"magneto_reset"), function(mag,"magneto_min_dir_range"), HELPERS,
        function(mag,"manual_finish"), function(mag,"manual_cancelled"),
        function(mag,"sensor_calibrate_mag"),
        function(bias,"isAccRest"), function(mag,"sensor_calibration_collect_accel_poses"),
        function(imu,"imu_step"), function(imu,"imu_failed"), function(imu,"imu_cancelled"),
        function(runtime,"sensor_tcal_assess_quality"), function(imu,"sensor_calibrate_imu"),
        function(imu,"sensor_calibrate_accel"), TESTS,
    ))
    with tempfile.TemporaryDirectory(prefix="cal-mag-events-") as directory:
        path = Path(directory)
        (path / "test.c").write_text(source)
        cc = shlex.split(os.environ.get("CC", "cc"))
        subprocess.run(cc + ["-std=gnu11", "-Wall", "-Wextra", "-Werror", "-I", str(ROOT / "src"), str(path / "test.c"), "-lm", "-o", str(path / "test")], check=True)
        subprocess.run([str(path / "test")], check=True)


if __name__ == "__main__":
    main()
