#!/usr/bin/env python3
"""Real sensitivity worker/set/reset with real heated request admission.

SOURCE_ROOT can select a baseline tree; only RTOS, storage, sensors and event
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
if "sensitivity_maintenance" not in calibration:
    prelude = prelude.replace(", sensitivity_maintenance", "")

FIXTURE = r'''
#include <math.h>
#include <stdio.h>
#include <string.h>
#define LOG_WRN(...) ((void)0)
#define printk(...) ((void)printf(__VA_ARGS__))
#undef LOG_INF
#define LOG_INF(...) ((void)printf(__VA_ARGS__))
#define K_MSEC(x) (x)
#define MAIN_GYRO_SENS_ID 1
#define WDT_CHANNEL_CALIBRATION 0
#define SYS_LED_PATTERN_OFF 0
#define SYS_LED_PATTERN_LONG 1
#define SYS_LED_PATTERN_FLASH 2
#define SYS_LED_PATTERN_ON 3
#define SYS_LED_PATTERN_ONESHOT_COMPLETE 4
#define SYS_LED_PRIORITY_SENSOR 0
static int sensitivity_lock;
static uint32_t sensitivity_generation;
static uint16_t sensitivity_operation;
static struct { float gyroSensScale[3]; } data;
static typeof(data) *retained = &data;
static unsigned writes, retained_updates, successes, cancellations, failures;
static uint8_t phase, cancel_reason;
static uint16_t cancelled_operation;
static int replacement, spin_samples;
static bool complete_during_write;
static int64_t now_ms;
static const float degrees[3] = {10.5f, -2.1f, 15.0f};
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
    if (at == CAL_PHASE_VALIDATE && replacement) {
        int mode=replacement; replacement=0;
        assert((mode == 1 ? sensor_calibration_set_sensitivity(degrees) :
                           sensor_calibration_reset_sensitivity()) == 0);
    }
}
static int sys_write(unsigned id, void *destination, const void *value, size_t len) {
    assert(id == MAIN_GYRO_SENS_ID && !owner_depth);
    assert(sensitivity_lock == 1);
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
static bool wait_for_motion(bool moving, int count) { (void)moving; (void)count; return true; }
static int64_t k_uptime_get(void) { return now_ms; }
static int64_t k_uptime_ticks(void) { return now_ms; }
static int64_t k_ticks_to_us_near64(int64_t ticks) { return ticks*1000; }
static void set_led(int pattern,int priority) { (void)pattern; (void)priority; }
static void watchdog_feed(int channel) { (void)channel; }
static bool v_finite(const float *values,int count) {
    for(int i=0;i<count;i++) if(!isfinite(values[i])) return false;
    return true;
}
static int sensor_wait_gyro(float g[3],int timeout) {
    (void)timeout; now_ms+=100; g[0]=g[1]=g[2]=0;
    if (phase==CAL_PHASE_WAIT_ROTATION ||
        (phase==CAL_PHASE_RECORD_ROTATION && spin_samples++<10)) g[sens_cal_axis]=360;
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
    now_ms=0; phase=0; spin_samples=0; replacement=mode;
    complete_during_write=finish_in_write;
    unsigned previous_writes=writes, previous_success=successes;
    unsigned previous_cancel=cancellations, previous_updates=retained_updates;
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
    assert(requested_calibration==(finish_in_write ? 0 : CAL_REQUEST_GYRO_SENS));
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
    assert(sensor_calibration_set_sensitivity(degrees)==-EBUSY);
    assert(sensor_calibration_reset_sensitivity()==-EBUSY);
    assert(writes==previous_writes && !memcmp(previous,data.gyroSensScale,sizeof(previous)));
}
int main(void) {
    run_worker(0,false); /* Control reaches a valid production worker commit. */
    run_worker(1,false); run_worker(2,false);
    run_worker(1,true); run_worker(2,true);
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
    sens_bodies = "\n".join(requests.function(name, sensitivity) for name in (
        "sensitivity_replace_locked", "sensitivity_step", "sensitivity_failed",
        "sensor_calibration_set_sensitivity", "sensor_calibration_reset_sensitivity",
        "sensor_calibrate_sens",
    ))
    source = prelude + bodies + FIXTURE + constants + "\n" + sens_bodies + TEST
    with tempfile.TemporaryDirectory(prefix="sensitivity-maintenance-") as directory:
        path = Path(directory)
        (path / "test.c").write_text(source)
        subprocess.run(shlex.split(os.environ.get("CC") or "cc") + [
            "-std=gnu11", "-Wall", "-Wextra", "-Werror",
            "-DCONFIG_SENSOR_USE_SENS_CALIBRATION=1", "-DCONFIG_SENSOR_TCAL_HEATED=1",
            "-I", str(ROOT / "src"), str(path / "test.c"), "-lm", "-o", str(path / "test"),
        ], check=True)
        subprocess.run([str(path / "test")], check=True)
    print("sensitivity replacement/reset with heated admission: PASS")


if __name__ == "__main__":
    main()
