#!/usr/bin/env python3
"""Real sensitivity worker/set/reset with real heated request admission.

SOURCE_ROOT can select a source tree; only RTOS, storage, sensors and event
transport are injected. A storage callback schedules worker completion while
an explicit replacement is pending, without modeling the admission policy.
"""
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

import test_requests as requests

ROOT = requests.ROOT
calibration = (ROOT / "src/sensor/calibration/calibration.c").read_text()
sensitivity = (ROOT / "src/sensor/calibration/cal_sens.c").read_text()

prelude = requests.PRELUDE
prelude = prelude.replace(
    "(void)lock; (void)wait; owner_depth++;",
    "(void)wait; (*lock)++; if (lock == &calibration_request_lock) owner_depth++;",
).replace(
    "(void)lock; assert(owner_depth == 1); owner_depth--;",
    "assert(*lock > 0); (*lock)--; if (lock == &calibration_request_lock) owner_depth--;",
)
prelude = prelude.replace("static void sensor_calibration_samples_end", "void sensor_calibration_samples_end")
prelude += "\nvoid sensor_tcal_feedback_cancel(void) {}\n"
if "sensitivity_maintenance" not in calibration:
    prelude = prelude.replace(", sensitivity_maintenance", "")

FIXTURE = r'''
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <float.h>
#include "sensor/calibration/cal_sample.h"
#define LOG_WRN(...) ((void)0)
#define printk(...) ((void)printf(__VA_ARGS__))
#undef LOG_INF
#define LOG_INF(...) ((void)printf(__VA_ARGS__))
#define K_MSEC(x) (x)
#define MAIN_GYRO_SENS_ID 1
#define WDT_CHANNEL_CALIBRATION 0
static int sensitivity_lock;
static uint32_t sensitivity_generation;
static uint16_t sensitivity_operation;
static struct led_token sensitivity_feedback;
static struct {
    float gyroSensScale[3], magBias[3], magBAinv[4][3];
    uint8_t mag_online_calibration_mode;
} data;
static typeof(data) *retained = &data;
static unsigned writes, retained_updates, successes, cancellations, failures;
static uint8_t phase, cancel_reason;
static uint16_t cancelled_operation;
static int replacement, spin_samples;
static bool complete_during_write;
static bool invalidate_on_validate;
static int64_t now_ms;
static const float degrees[3] = {10.5f, -2.1f, 15.0f};
static uint32_t gyro_sequence;
static bool replay_mode;
static unsigned replay_index, replay_count, delivery_batch = 1;
static struct sensor_gyro_sample replay[128];
static uint32_t sample_timestamp_us[128];
static uint32_t delivered_hash;
static unsigned imu_loads;
static float magBias[3];
#define MAG_ONLINE_CALIBRATION_DISABLED 2
#define MAG_ONLINE_CALIBRATION_DEFAULT 0
static void sensor_calibration_imu_load(void) {
    for (int i=0; i<3; i++) assert(sensor_calibration_sensitivity_valid(retained->gyroSensScale[i]));
    imu_loads++;
}
static void magneto_online_replace_BAinv_and_reset(float matrix[4][3], int owner, struct led_token token) {
    (void)matrix; (void)owner; (void)token;
}
static void magneto_online_runtime_configure(bool enabled) { (void)enabled; }
bool sensor_calibration_get_online_mag_enabled(void) { return false; }
static void magneto_online_snapshot_BAinv(float matrix[4][3]) { memset(matrix,0,12*sizeof(float)); }
static bool mag_bainv_structurally_ok(float matrix[4][3],float tolerance) { (void)matrix; (void)tolerance; return false; }
static void magneto_online_runtime_load_retained(void) {}
static int cal_online_mag_update_count(void) { return 0; }
static unsigned cal_online_mag_norm_count(void) { return 0; }
int sensor_calibration_set_sensitivity(const float values[3]);
int sensor_calibration_reset_sensitivity(void);
static void retained_update(void) { retained_updates++; }
static void cal_event_end(uint16_t operation, uint8_t outcome, uint8_t at, uint8_t reason) {
    (void)at;
    if (!operation) return;
    if (outcome == CAL_OUTCOME_SUCCESS) successes++;
    else if (outcome == CAL_OUTCOME_CANCELLED) {
        cancellations++; cancel_reason=reason; cancelled_operation=operation;
    } else failures++;
}
static void cal_event_step(uint16_t operation, uint8_t at, uint8_t detail) {
    (void)operation; (void)detail; phase=at;
    if (at == CAL_PHASE_VALIDATE && invalidate_on_validate) {
        invalidate_on_validate=false;
        sensor_calibration_invalidate_requests();
    }
    if (at == CAL_PHASE_VALIDATE && replacement) {
        int mode=replacement; replacement=0;
        assert((mode == 1 ? sensor_calibration_set_sensitivity(degrees) :
                           sensor_calibration_reset_sensitivity()) == 0);
    }
}
static int sys_write(unsigned id, void *destination, const void *value, size_t len) {
    assert(id == MAIN_GYRO_SENS_ID && !owner_depth);
    if (complete_during_write) {
        complete_during_write=false;
        /* The worker owns and clears its request independently of maintenance. */
        assert(sensor_calibration_request(CAL_REQUEST_CLEAR,CAL_REQUEST_USER)==0);
        assert(requested_calibration==0);
        sensor_tcal_heated_lock();
        assert(sensor_calibration_heated_reserve_locked()==-EBUSY);
        assert(sensor_calibration_maintenance_active_locked());
        sensor_tcal_heated_unlock();
        assert(sensor_calibration_request(CAL_REQUEST_IMU,CAL_REQUEST_USER)==-1);
        assert(sensor_request_calibration_sens(0,1)==-1);
        assert(sensor_request_calibration_mag()==-EBUSY);
        assert(sensor_calibration_maintenance_begin()==-EBUSY);
        assert(sensor_calibration_reset_sensitivity()==-EBUSY);
    }
    writes++; memcpy(destination,value,len); return 0;
}
bool wait_for_motion(bool moving, int count) { (void)moving; (void)count; return true; }
static int64_t k_uptime_get(void) { return now_ms; }
static void watchdog_feed(int channel) { (void)channel; }
/* Warm storage leaves: the worker brackets its final write in a transaction. */
static unsigned warm_begin, warm_end;
void sys_warm_transaction_begin(void) { warm_begin++; }
void sys_warm_transaction_mark(uint16_t id, const void *data, size_t size) {
    (void)id; (void)data; (void)size;
}
void sys_warm_transaction_end(bool schedule) { (void)schedule; warm_end++; }
int sensor_wait_gyro(struct sensor_gyro_sample *sample,k_timeout_t timeout) {
    (void)timeout;
    if (replay_mode) {
        assert(replay_index < replay_count);
        *sample = replay[replay_index];
        /* Delivery may be uniform, paired, or an entire FIFO at one instant;
         * the physical timestamp/interval/sequence/vector stream is identical. */
        unsigned group_end=((replay_index/delivery_batch)+1)*delivery_batch;
        if (group_end > replay_count) group_end=replay_count;
        now_ms=sample_timestamp_us[group_end-1]/1000;
        const unsigned char *bytes=(const unsigned char *)sample;
        for (size_t i=0; i<sizeof(*sample); i++) delivered_hash=(delivered_hash^bytes[i])*16777619U;
        replay_index++;
        return 0;
    }
    now_ms+=100;
    *sample=(struct sensor_gyro_sample){.dt=0.1f,.sequence=++gyro_sequence};
    if (phase==CAL_PHASE_WAIT_ROTATION ||
        (phase==CAL_PHASE_RECORD_ROTATION && spin_samples++<9)) sample->v[sens_cal_axis]=360;
    return 0;
}
'''

TEST = r'''
static void check_scales(bool reset) {
    for(int i=0;i<3;i++) {
        float expected=reset ? 1.0f : 1.0f/(1.0f-degrees[i]/(360.0f*CONFIG_SENSOR_SENS_REV));
        assert(fabsf(retained->gyroSensScale[i]-expected)<1e-6f);
    }
}
static void run_worker(int mode,bool finish_in_write) {
    now_ms=0; phase=0; spin_samples=0; replacement=mode; gyro_sequence=0;
    complete_during_write=finish_in_write;
    memset(led_results,0,sizeof(led_results));
    warm_begin=warm_end=0;
    unsigned previous_writes=writes, previous_success=successes;
    unsigned previous_cancel=cancellations, previous_updates=retained_updates;
    uint32_t previous_generation=sensitivity_generation;
    assert(sensor_request_calibration_sens(0,1)==0);
    uint16_t operation=sensor_calibration_current_operation();
    sensor_calibrate_sens();
    assert(!failures && writes==previous_writes+1);
    if(mode) {
        assert(successes==previous_success && retained_updates==previous_updates);
        assert(cancellations==previous_cancel+1 && cancelled_operation==operation);
        assert(cancel_reason==(mode==1 ? CAL_REASON_REPLACED : CAL_REASON_RESET));
        check_scales(mode==2);
    } else {
        assert(successes==previous_success+1 && retained_updates==previous_updates+1);
        assert(fabsf(retained->gyroSensScale[0]-1.0f)<1e-6f);
    }
    /* Manual set/reset replaces once even if the stale worker subsequently
     * reaches validation; only an actual automatic commit advances SIDE. */
    assert(sensitivity_generation==previous_generation+
        (mode ? 1U : (unsigned)CONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION));
    assert(requested_calibration==(finish_in_write ? 0 : CAL_REQUEST_GYRO_SENS));
    /* The guarded final write runs inside a warm transaction. */
    assert(warm_begin >= 1 && warm_end >= 1);
    /* Only a completed worker reports a green terminal; a replaced session does not. */
    assert(led_results[LED_SUCCESS] == (mode ? 0 : 1));
    sensor_tcal_heated_lock();
    assert(!sensor_calibration_maintenance_active_locked());
    sensor_tcal_heated_unlock();
    assert(sensor_calibration_request(CAL_REQUEST_CLEAR,CAL_REQUEST_USER)==0);
    sensor_tcal_heated_lock();
    assert(sensor_calibration_heated_reserve_locked()==0);
    sensor_calibration_heated_release_locked();
    sensor_tcal_heated_unlock();
}
static void rejected_replacement(void) {
    float previous[3]; memcpy(previous,data.gyroSensScale,sizeof(previous));
    unsigned previous_writes=writes;
    uint32_t previous_generation=sensitivity_generation;
    assert(sensor_calibration_set_sensitivity(degrees)==-EBUSY);
    assert(sensor_calibration_reset_sensitivity()==-EBUSY);
    assert(writes==previous_writes && !memcmp(previous,data.gyroSensScale,sizeof(previous)));
    assert(sensitivity_generation==previous_generation);
}
static void stale_epoch_does_not_overwrite_scales(void) {
    now_ms=0; phase=0; spin_samples=0; replacement=0; gyro_sequence=0;
    complete_during_write=false;
    assert(sensor_calibration_set_sensitivity(degrees)==0);
    float before[3]; memcpy(before,data.gyroSensScale,sizeof(before));
    unsigned previous_writes=writes, previous_updates=retained_updates;
    unsigned previous_success=successes, previous_cancel=cancellations;
    uint32_t previous_generation=sensitivity_generation;
    memset(led_results,0,sizeof(led_results));
    warm_begin=warm_end=0;
    assert(sensor_request_calibration_sens(0,1)==0);
    uint16_t operation=sensor_calibration_current_operation();
    invalidate_on_validate=true;
    sensor_calibrate_sens();
    assert(writes==previous_writes && retained_updates==previous_updates);
    assert(!memcmp(before,data.gyroSensScale,sizeof(before)));
    assert(successes==previous_success && cancellations==previous_cancel+1);
    assert(cancelled_operation==operation && cancel_reason==CAL_REASON_RESET);
    assert(led_results[LED_SUCCESS]==0 && warm_begin==warm_end);
    assert(sensitivity_generation==previous_generation);
    assert(sensor_calibration_request(CAL_REQUEST_CLEAR,CAL_REQUEST_USER)==0);
}
static void manual_rejects_invalid_without_mutation(void) {
    const float bad[] = {
        3600.0f, 1799.819946f, -1e30f, 1800.0f, NAN, INFINITY, -INFINITY,
        -360.0f, 360.0f,
    };
    float before[3]; memcpy(before,data.gyroSensScale,sizeof(before));
    unsigned prior_writes=writes, prior_begin=warm_begin, prior_end=warm_end;
    uint32_t prior_generation=sensitivity_generation;
    assert(sensor_calibration_set_sensitivity(NULL)==-EINVAL);
    for (unsigned axis=0;axis<3;axis++) {
        for (unsigned i=0;i<sizeof(bad)/sizeof(*bad);i++) {
            float values[3]={0};
            values[axis]=bad[i];
            assert(sensor_calibration_set_sensitivity(values)==-EINVAL);
            assert(!memcmp(before,data.gyroSensScale,sizeof(before)));
            assert(writes==prior_writes && warm_begin==prior_begin && warm_end==prior_end);
            assert(sensitivity_generation==prior_generation);
        }
    }
    assert(sensor_calibration_sensitivity_valid(0.9f));
    assert(sensor_calibration_sensitivity_valid(1.1f));
    assert(!sensor_calibration_sensitivity_valid(nextafterf(0.9f,0.0f)));
    assert(!sensor_calibration_sensitivity_valid(nextafterf(1.1f,2.0f)));
}
static void loaded_invalid_axes_are_healed_before_use(void) {
    const float bad[]={0.0f,-1.0f,0.89f,1.11f,9998.34f,1.8e-27f,FLT_MAX,NAN,INFINITY,-INFINITY};
    unsigned prior_writes=writes;
    for (unsigned axis=0;axis<3;axis++) {
        for (unsigned i=0;i<sizeof(bad)/sizeof(*bad);i++) {
            float expected[3]={0.9f,1.03f,1.1f};
            memcpy(data.gyroSensScale,expected,sizeof(expected));
            data.gyroSensScale[axis]=bad[i];
            expected[axis]=1.0f;
            unsigned prior_updates=retained_updates, prior_loads=imu_loads;
            sensor_calibration_read();
            assert(!memcmp(expected,data.gyroSensScale,sizeof(expected)));
            assert(retained_updates==prior_updates+1 && imu_loads==prior_loads+1);
            assert(writes==prior_writes); /* Healing is retained-load policy, not an NVS rewrite. */
            sensor_calibration_read();
            assert(retained_updates==prior_updates+1 && imu_loads==prior_loads+2);
        }
    }
    memset(data.gyroSensScale,0,sizeof(data.gyroSensScale));
    sensor_calibration_read();
    for (int i=0;i<3;i++) assert(data.gyroSensScale[i]==1.0f);
}
static void append_physical_sample(float rate) {
    unsigned i=replay_count++;
    assert(i<sizeof(replay)/sizeof(*replay));
    uint32_t previous=i ? sample_timestamp_us[i-1] : 0;
    sample_timestamp_us[i]=previous+125000;
    replay[i]=(struct sensor_gyro_sample){
        .v={rate+2.0f,-3.0f,0.5f},
        .dt=(float)(sample_timestamp_us[i]-previous)*1e-6f,
        .sequence=i+1,
    };
}
static void physical_stream(bool pause) {
    replay_count=0;
    for (int i=0;i<8;i++) append_physical_sample(0); /* One-second bias window. */
    for (int i=0;i<8;i++) append_physical_sample(360); /* Includes trigger's 45 degrees. */
    if (pause) {
        for (int i=0;i<4;i++) append_physical_sample(0); /* Half-dwell cannot finish. */
        append_physical_sample(64); /* Resume motion: reset stop dwell. */
    }
    for (int i=0;i<8;i++) append_physical_sample(0); /* Exactly one second to stop. */
}
static float run_physical_stream(unsigned batch, bool accepted_fit) {
    replay_mode=true; replay_index=0; delivery_batch=batch;
    now_ms=0; phase=0; replacement=0; spin_samples=0; delivered_hash=2166136261U;
    unsigned prior_writes=writes, prior_success=successes, prior_failure=failures;
    uint32_t prior_generation=sensitivity_generation;
    float before[3]; memcpy(before,data.gyroSensScale,sizeof(before));
    assert(sensor_request_calibration_sens(0,1)==0);
    sensor_calibrate_sens();
    if (accepted_fit) {
        assert(replay_index==replay_count); /* Entire stop dwell, no consumer-clock shortcut. */
        assert(writes==prior_writes+1 && successes==prior_success+1 && failures==prior_failure);
    } else {
        assert(writes==prior_writes && successes==prior_success && failures==prior_failure+1);
        assert(!memcmp(before,data.gyroSensScale,sizeof(before)));
    }
    assert(sensitivity_generation==prior_generation+
        (accepted_fit ? (unsigned)CONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION : 0U));
    assert(sensor_calibration_request(CAL_REQUEST_CLEAR,CAL_REQUEST_USER)==0);
    replay_mode=false;
    return data.gyroSensScale[0];
}
static void batching_trigger_and_stop_dwell_invariants(void) {
    for (unsigned pause=0;pause<2;pause++) {
        physical_stream(pause);
        float first=run_physical_stream(1,true);
        uint32_t hash=delivered_hash;
        assert(fabsf(first-(pause ? 360.0f/368.0f : 1.0f))<1e-6f);
        const unsigned batches[]={2,3,8,64};
        for (unsigned i=0;i<sizeof(batches)/sizeof(*batches);i++) {
            float batched=run_physical_stream(batches[i],true);
            assert(!memcmp(&first,&batched,sizeof(first)));
            assert(hash==delivered_hash); /* Same physical samples and timestamps. */
        }
    }
}
static void missing_intervals_reject_the_measurement(void) {
    const unsigned gaps[]={3,8,10,19}; /* Bias, trigger, rotation and stop dwell. */
    for (unsigned g=0;g<sizeof(gaps)/sizeof(*gaps);g++) {
        physical_stream(false);
        for (unsigned i=gaps[g];i<replay_count;i++) replay[i].sequence++;
        run_physical_stream(64,false);
    }
    const float invalid_dt[]={0.0f,-0.125f,NAN,INFINITY};
    for (unsigned i=0;i<sizeof(invalid_dt)/sizeof(*invalid_dt);i++) {
        physical_stream(false); replay[10].dt=invalid_dt[i];
        run_physical_stream(2,false);
    }
    /* Unsigned sequence wrap is contiguous, not a fabricated loss. */
    physical_stream(false);
    for (unsigned i=0;i<replay_count;i++) replay[i].sequence += UINT32_MAX-12U;
    assert(fabsf(run_physical_stream(64,true)-1.0f)<1e-6f);
}
int main(void) {
    run_worker(0,false); /* Control reaches a valid production worker commit. */
    run_worker(1,false); run_worker(2,false);
    run_worker(1,true); run_worker(2,true);
    stale_epoch_does_not_overwrite_scales();
    manual_rejects_invalid_without_mutation();
    loaded_invalid_axes_are_healed_before_use();
    batching_trigger_and_stop_dwell_invariants();
    missing_intervals_reject_the_measurement();
    sensor_tcal_heated_lock();
    assert(sensor_calibration_heated_reserve_locked()==0);
    sensor_tcal_heated_unlock();
    rejected_replacement();
    assert(imu_heated && requested_calibration==CAL_REQUEST_TCAL_HEATED);
    sensor_tcal_heated_lock(); sensor_calibration_heated_release_locked(); sensor_tcal_heated_unlock();
    assert(sensor_calibration_maintenance_begin()==0);
    rejected_replacement();
    assert(requested_calibration==CAL_REQUEST_MAINTENANCE);
    sensor_calibration_maintenance_end();
    reset_barrier=true; rejected_replacement(); reset_barrier=false;
    assert(sensor_calibration_request(CAL_REQUEST_IMU,CAL_REQUEST_USER)==0);
    rejected_replacement(); assert(requested_calibration==CAL_REQUEST_IMU);
    sensor_calibration_request(CAL_REQUEST_CLEAR,CAL_REQUEST_USER);
    assert(sensor_calibration_set_sensitivity(degrees)==0); check_scales(false);
    assert(sensor_calibration_reset_sensitivity()==0); check_scales(true);
    assert(sensor_calibration_request(CAL_REQUEST_IMU,CAL_REQUEST_USER)==0);
    return 0;
}
'''


def main():
    owners = (
        "calibration_mag_pending",
        "calibration_next_generation", "calibration_led_owner", "calibration_led_accept",
        "sensor_calibration_current_feedback", "sensor_calibration_current_generation",
        "sensor_calibration_generation_valid", "sensor_calibration_invalidate_requests",
        "sensor_tcal_heated_lock", "sensor_tcal_heated_unlock",
        "sensor_calibration_heated_reserve_locked", "sensor_calibration_heated_release_locked",
        "sensor_calibration_maintenance_begin", "sensor_calibration_maintenance_end",
        "sensor_calibration_maintenance_active_locked", "calibration_request_kind",
        "sensor_calibration_current_operation", "sensor_calibration_request",
        "sensor_request_calibration_sens", "sensor_request_calibration_mag",
    )
    # Baseline lacks these helpers; its real setters call ordinary maintenance.
    extra = tuple(name for name in (
        "sensor_calibration_sensitivity_maintenance_begin",
        "sensor_calibration_sensitivity_maintenance_end",
    ) if re.search(rf"^int {name}\(|^void {name}\(", calibration, re.M))
    bodies = "\n".join(requests.function(name, calibration) for name in owners + extra)
    constants = "\n".join(re.findall(r"^#define SENS_CAL_.*$", sensitivity, re.M))
    stream_struct = requests.extract_block(sensitivity, r"^struct sensitivity_stream\s*\{") + ";\n"
    finite = requests.function("v_finite", (ROOT / "src/util.c").read_text())
    snapshot_start = sensitivity.index("static struct sensitivity_snapshot {")
    snapshot = sensitivity[snapshot_start:sensitivity.index("\n#endif", snapshot_start)]
    load_bodies = (
        requests.function("sensor_calibration_sensitivity_valid", calibration)
        + "\n#if CONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION\n" + snapshot + "\n"
        + requests.function("sensor_calibration_sensitivity_init", sensitivity)
        + "\n#endif\n" + requests.function("sensor_calibration_read", calibration)
    )
    sens_bodies = "\n".join(requests.function(name, sensitivity) for name in (
        "sensitivity_replace_locked", "sensitivity_step", "sensitivity_failed",
        "sensor_calibration_set_sensitivity", "sensor_calibration_reset_sensitivity",
        "sensitivity_wait_sample", "sensor_calibrate_sens",
    ))
    source = prelude + bodies + FIXTURE + constants + "\n" + stream_struct + finite + load_bodies + sens_bodies + TEST
    with tempfile.TemporaryDirectory(prefix="sensitivity-maintenance-") as directory:
        path = Path(directory)
        (path / "test.c").write_text(source)
        kernel = path / "zephyr/kernel.h"
        kernel.parent.mkdir()
        kernel.write_text("typedef int k_timeout_t;\n")
        for side in (0, 1):
            for flags in ([], ["-ffast-math"]):
                subprocess.run(shlex.split(os.environ.get("CC") or "cc") + [
                    "-std=gnu11", "-Wall", "-Wextra", "-Werror", *flags,
                    "-DCONFIG_CONNECTION_RAW_COLLECTION=1", "-DCONFIG_SENSOR_USE_MAGNETOMETER=1",
                    "-DCONFIG_SENSOR_USE_SENS_CALIBRATION=1", "-DCONFIG_SENSOR_TCAL_HEATED=1",
                    f"-DCONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION={side}",
                    "-DCONFIG_SENSOR_USE_TCAL=0", "-I", str(path),
                    "-I", str(ROOT / "src"), str(path / "test.c"), "-lm", "-o", str(path / "test"),
                ], check=True)
                subprocess.run([str(path / "test")], check=True)
    print("sensitivity sample-time fits, admission/load and replacement/reset: PASS")


if __name__ == "__main__":
    main()
