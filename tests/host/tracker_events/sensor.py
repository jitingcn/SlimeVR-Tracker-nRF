#!/usr/bin/env python3
"""Production motion/feed/publish/lifecycle bodies and real event scheduler."""
import importlib.util
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get('SOURCE_ROOT', HERE.parents[2]))
spec = importlib.util.spec_from_file_location('event_leaves', HERE / 'run.py')
leaves = importlib.util.module_from_spec(spec)
spec.loader.exec_module(leaves)
source = (ROOT / 'src/sensor/sensor.c').read_text()


def function(name):
    match = re.search(r'^(?:static )?(?:void|bool|float|uint32_t) ' + name + r'\([^;]*?\)\s*\{', source, re.M)
    opening = source.index('{', match.start())
    depth = 0
    for token in re.finditer(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|[{}]', source[opening:], re.S):
        if token.group() == '{':
            depth += 1
        elif token.group() == '}':
            depth -= 1
            if depth == 0:
                return source[match.start():opening + token.end()]
    raise ValueError(name)


loop = function('sensor_loop')
begin = loop.index('sensor_apply_calibration_frame();')
acquire = loop[begin:loop.index('if (sensor_startup_discard_until_ms', begin)]
publish = function('sensor_loop_publish')
begin = publish.index('// Get updated quaternion from fusion')
publish_policy = publish[begin:publish.index('sensor_diagnostics_output(', begin)]
begin = publish.index('#if CONFIG_SENSOR_USE_TCAL', publish.index('// Update orientation'))
calibration_policy = publish[begin:publish.index('// Periodic retained save', begin)]
rest = source[source.index('static struct sensor_rest_detector rest_detector;'):source.index('static int sensor_scan(void);')]
dwell = source[source.index('#ifndef SENSOR_REST_ENTER_STABLE_MS'):source.index('#define SENSOR_ACTIVITY_STARTUP_GUARD_MS')]
fixture = r'''
#include <math.h>
#include "util.h"
#include "sensor/motion_state.h"
#define CONFIG_DYNAMIC_ACTIVE_TIMEOUT 1
#define CONFIG_SENSOR_USE_TCAL 1
#define CONFIG_SENSOR_GYRO_OVERSAMPLING 1
#define CONFIG_SENSOR_ACCEL_OVERSAMPLING 1
#define SENSOR_ACTIVITY_STARTUP_GUARD_MS 5000
#define CONFIG_ACTIVE_TIMEOUT_MEANINGFUL_MOTION_MS 3000
#define WDT_CHANNEL_SENSOR 0
#define SENSOR_LIFE_IDLE 1
#define SENSOR_LIFE_SCAN_DONE 2
#define K_MSEC(x) (x)
#define atomic_set(p,v) (*(p)=(v))
#define atomic_get(p) (*(p))
#define FUSION_VQF 1
#define FUSION_EQF 2
static bool main_suspended, main_running=true, main_ok=true, sensor_sensor_scanning;
static int sensor_thread_id, sensor_life_events;
static unsigned suspended, resumed, idle_timeouts;
static bool detector_pending, detector_rest=true;
static float pose[4]={1},q[4],sensor_loop_avg_a[3]={0,0,1};
static bool local_rest,cal_rest;
static unsigned auto_calibrations;
static float temp=20;
static int64_t last_data_time;
static float gyro_actual_time=.006f,accel_actual_time=.01f;
static unsigned sensor_update_time_ms=6;
static bool sensor_session_woke_from_wom=true,sensor_session_meaningful_motion;
static struct sensor_activity_score sensor_session_activity_score={.last_update_ms=-1};
static struct {unsigned wom_idle_wake_streak;} retained_data;
static typeof(retained_data) *retained=&retained_data;
static void retained_update(void) {}
static bool take_observation(bool *out) {
    bool available=detector_pending; detector_pending=false;
    if(available && out) *out=detector_rest;
    return available;
}
static void get_quat(float *out) {memcpy(out,pose,sizeof(pose));}
static void update_gyro(float *g,float dt) {(void)g;(void)dt;}
static struct {bool (*take_rest_observation)(bool*); void (*get_quat)(float*); void (*update_gyro)(float*,float);}
    backend={take_observation,get_quat,update_gyro};
static const typeof(backend) *sensor_fusion=&backend;
static int fusion_id=FUSION_VQF;
static void sensor_diagnostics_on_cal_gyro(float *g) {(void)g;}
static void sensor_calibration_set_consumer_ready(bool x) {(void)x;}
static void sys_cancel_WOM(void) {}
static void watchdog_pause(int x) {(void)x;}
static void watchdog_resume(int x) {(void)x;}
static void k_thread_suspend(int *x) {(void)x; suspended++;}
static void k_thread_resume(int *x) {(void)x; assert(!main_suspended); resumed++;}
static int k_event_wait(int *event,int mask,bool reset,int timeout) {
    (void)event;(void)reset;assert(mask==SENSOR_LIFE_IDLE && timeout==5000);
    now_ms+=timeout; idle_timeouts++; return 0;
}
static int64_t k_uptime_ticks(void) {return now_ms;}
static void sensor_update_sensor_state(bool resting) {local_rest=resting;}
static void sensor_runtime_calibration_check(bool resting) {cal_rest=resting;}
static void sensor_tcal_continuous_motion_detected(void) {cal_rest=false;}
static void sensor_tcal_boot_calibration_check(void) {}
static void sensor_tcal_check_auto_calibration(float t) {(void)t;auto_calibrations++;}
static bool sensor_tcal_get_auto_calibration(void) {return false;}
typedef struct {uint32_t sensor_epoch; bool dc_active; int g_count,a_count;} sensor_loop_frame_t;
static void sensor_apply_calibration_frame(void) {}
static void sensor_loop_handle_data_collection(bool *active) {(void)active;}
'''
fixture += dwell + rest + function('feed_calibrated_gyro') + function('sensor_update_session_motion')
fixture += function('main_imu_suspend') + function('main_imu_resume')
fixture += r'''
static uint64_t sensor_window_acq_us, sensor_window_acq_max_us;
#define k_ticks_to_us_near64(x) (x)
static bool interrupt_acquire;
static void sensor_loop_acquire(sensor_loop_frame_t *frame) {
    if(interrupt_acquire) {
        detector_pending=true;
        main_imu_suspend(); assert(!detector_pending);
        now_ms+=2000; main_imu_resume();
    }
    frame->g_count=frame->a_count=1; detector_pending=true;
}
static void publish_observation(sensor_loop_frame_t *frame) {
'''
fixture += publish_policy + calibration_policy + '\n}\n'
fixture += 'static void frame_once(void) { sensor_loop_frame_t frame={0};\n' + acquire
fixture += r'''
    (void)acq_begin_ticks;
    sensor_motion_prepare(frame.sensor_epoch,now_ms);
    sensor_rest_detector_update_accel(&rest_detector,sensor_loop_avg_a,accel_actual_time);
    publish_observation(&frame);
}
static unsigned known,unknown,unavailable;
static void drain(unsigned duration) {
    uint32_t until=now_ms+duration;
    for(;now_ms<until;now_ms+=100) {
        struct tracker_event_tx tx;
        if(!tracker_events_select(now_ms,2,&tx)) continue;
        struct tracker_event event;
        assert(tracker_event_decode(tx.packet,sizeof(tx.packet),&event));
        assert(event.event==CAL_EVENT_STATE);
        if(event.phase<2) known++;
        else if(event.phase==2) unknown++;
        else unavailable++;
        tracker_events_complete(tx.token,true,now_ms);
    }
}
static void reset_motion(void) {
    sensor_motion_reset(); main_suspended=false;
    sensor_session_meaningful_motion=false;
    gyro_actual_time=.006f; accel_actual_time=.01f; sensor_update_time_ms=6;
    now_ms=0; pose[0]=1;pose[1]=pose[2]=pose[3]=0;
    sensor_loop_avg_a[0]=sensor_loop_avg_a[1]=0;sensor_loop_avg_a[2]=1;
    local_rest=cal_rest=false;
}
static void yaw(float degrees) {
    float half=degrees*(3.14159265358979323846f/360.0f);
    pose[0]=cosf(half);pose[1]=pose[2]=0;pose[3]=sinf(half);
}
static bool sample(uint32_t at,int gyro,int accel,float degrees) {
    now_ms=at; yaw(degrees);
    sensor_loop_frame_t frame={.sensor_epoch=tracker_events_sensor_epoch(),.a_count=accel};
    sensor_motion_prepare(frame.sensor_epoch,now_ms);
    /* Actual feed glue: a large noisy/bias-laden gyro is irrelevant when
     * fused orientation is stable. Fusion payload itself is tested in gyro_feed. */
    if(gyro) {float g[3]={at%2 ? 100 : -100,55,-23};feed_calibrated_gyro(g,gyro_actual_time,&frame.g_count);}
    if(accel) sensor_rest_detector_update_accel(&rest_detector,sensor_loop_avg_a,accel_actual_time);
    detector_pending=gyro||accel;
    publish_observation(&frame);
    return local_rest;
}
static float active_yaw_fixture(void) {
    struct sensor_rest_evidence evidence={.accel_valid=true};
    const float rad=3.14159265358979323846f/180.0f;
    for(float degrees=.001f;degrees<180;degrees+=.001f)
        if(sensor_motion_is_active(0,degrees*rad,&evidence)) return degrees;
    assert(!"no active orientation within physical range");
    return 0;
}
static void rest_motion_contracts(void) {
    const float exit_yaw=active_yaw_fixture();
    reset_motion();
    for(unsigned t=0;t<1000;t+=10) assert(!sample(t,1,1,0));
    assert(sample(1000,1,1,0));assert(cal_rest);
    for(unsigned t=1010;t<1260;t+=10) assert(sample(t,1,1,exit_yaw+.05f));
    assert(!sample(1260,1,1,exit_yaw+.05f));assert(!cal_rest);
    reset_motion();
    assert(!sample(0,1,1,0));
    for(unsigned t=10;t<1000;t+=10) assert(!sample(t,1,1,.10f));
    assert(sample(1000,1,1,.10f));
    /* Anchor confirmed entry at .10deg, not the candidate's0deg. Choose a
     * pose above the policy's exit gate from0 but below it from .10deg. */
    for(unsigned t=1010;t<=1310;t+=10) assert(sample(t,1,1,exit_yaw+.05f));
    for(unsigned t=1320;t<1570;t+=10) assert(sample(t,1,1,exit_yaw+.15f));
    assert(!sample(1570,1,1,exit_yaw+.15f)); /* exit dwell from confirmed ref */
    reset_motion();
    for(unsigned t=0;t<=1500;t+=10)sample(t,1,1,0);
    sensor_loop_avg_a[0]=.30f/CONST_EARTH_GRAVITY;
    for(unsigned t=1510;t<=2510;t+=10)assert(sample(t,1,1,0));
    /* A short residual excursion cannot bypass250ms exit dwell. */
    reset_motion();
    /* A slow continuous rate must still cross the .6deg entry gate before the
     * 1000ms dwell: .25dps is now quiet long enough to legitimately enter. */
    for(unsigned t=0;t<8000;t+=6) assert(!sample(t,1,1,.75f*t/1000));
    reset_motion();
    for(unsigned t=0;t<=8000;t+=6) sample(t,1,1,(t/6)%2 ? .05f : -.05f);
    assert(local_rest && !sensor_session_meaningful_motion);
    assert(sensor_session_activity_score.value_ms==0);
    /* q/-q is the same orientation at both the rest and scoring layer. */
    sensor_loop_frame_t frame={.sensor_epoch=tracker_events_sensor_epoch(),.g_count=1,.a_count=1};
    for(unsigned i=0;i<40;i++) {
        now_ms+=6;sensor_motion_prepare(frame.sensor_epoch,now_ms);
        sensor_rest_detector_update_accel(&rest_detector,sensor_loop_avg_a,accel_actual_time);
        for(unsigned j=0;j<4;j++)pose[j]=-pose[j];
        publish_observation(&frame);assert(local_rest);
    }
    assert(!sensor_session_meaningful_motion);
    reset_motion();
    for(unsigned t=0;t<=10000;t+=6)sample(t,1,1,.003f*t);
    assert(sensor_session_meaningful_motion);
    tracker_events_sensor_invalidate(TRACKER_REST_RESET);
    sample(11000,1,1,180);
    assert(sensor_session_meaningful_motion); /* one-way session latch */
}
static void freshness_contracts(void) {
    reset_motion();
    for(unsigned t=0;t<=2000;t+=10) assert(!sample(t,1,0,0));
    reset_motion();
    for(unsigned t=0;t<=1500;t+=10) sample(t,1,1,0);
    unsigned before=auto_calibrations;
    assert(!sample(1510,0,0,0));assert(auto_calibrations==before);
    assert(sample(1520,1,1,0)); /* short hold does not erase established rest */
    assert(!sample(1560,1,0,0)); /* accel expired (>30ms), gyro alone cannot renew */
    reset_motion();
    for(unsigned t=0;t<=1500;t+=10) sample(t,1,1,0);
    assert(!sample(1530,0,1,0)); /* gyro expired (>22ms), accel alone cannot renew */
    reset_motion();
    for(unsigned t=0;t<=1400;t+=10) sample(t,1,1,0);
    for(unsigned t=1410;t<=4000;t+=10) assert(!sample(t,0,0,0));
    assert(!sample(4010,1,1,90));
    for(unsigned t=4020;t<5010;t+=10) assert(!sample(t,1,1,90));
    assert(sample(5010,1,1,90)); /* fresh1000ms dwell from the4010 resume. */
    /* Healthy slower accel and33/100ms configured batches remain useful. */
    reset_motion();accel_actual_time=.05f;
    for(unsigned t=0;t<=1600;t+=10) sample(t,1,t%50==0,0);
    assert(local_rest);
    for(unsigned period=33;period<=100;period+=67) {
        reset_motion();sensor_update_time_ms=period;
        for(unsigned t=0;t<2000;t+=period)sample(t,1,1,0);
        assert(local_rest);
    }
}
static void angular_window_contracts(void) {
    reset_motion();
    float identity[4]={1,0,0,0};
    yaw(.015f); /*2.5dps over6ms, scalar rounds too close to1 for acos.*/
    assert(fabsf(sensor_motion_quat_angle(identity,pose)*180.0f/(float)M_PI/.006f-2.5f)<.0001f);
    bool rest;float rate,linear,lin[3]={0};
    for(unsigned t=0;t<=102;t+=6) {
        now_ms=t;yaw(.0025f*t);
        sensor_motion_prepare(tracker_events_sensor_epoch(),t);
        sensor_rest_detector_update_accel(&rest_detector,sensor_loop_avg_a,.01f);
        assert(sensor_motion_observe(tracker_events_sensor_epoch(),1,1,pose,lin,t,&rest,&rate,&linear));
        if(t<102)assert(rate<0);else assert(fabsf(rate-2.5f)<.0001f);
    }
    sensor_session_activity_score.value_ms=500;
    tracker_events_sensor_invalidate(TRACKER_REST_RESET);
    now_ms=108;yaw(90);sensor_motion_prepare(tracker_events_sensor_epoch(),now_ms);
    sensor_rest_detector_update_accel(&rest_detector,sensor_loop_avg_a,.01f);
    assert(sensor_motion_observe(tracker_events_sensor_epoch(),1,1,pose,lin,now_ms,&rest,&rate,&linear));
    assert(!rest && rate<0 && sensor_session_activity_score.value_ms==0);
    now_ms=1000;yaw(180);sensor_motion_prepare(tracker_events_sensor_epoch(),now_ms);
    sensor_rest_detector_update_accel(&rest_detector,sensor_loop_avg_a,.01f);
    assert(sensor_motion_observe(tracker_events_sensor_epoch(),1,1,pose,lin,now_ms,&rest,&rate,&linear));
    assert(!rest && rate<0);
}
int main(void) {
    assert(host_init()==0);
    interrupt_acquire=true;frame_once();
    assert(suspended==1 && resumed==1 && idle_timeouts==1);
    assert(!detector_pending && !local_rest && !cal_rest);
    drain(700);assert(known==0 && unknown>0);
    interrupt_acquire=false;frame_once();drain(300);assert(known>=2);
    unsigned before=known;
    main_imu_suspend();
    sensor_loop_frame_t stale={.sensor_epoch=tracker_events_sensor_epoch(),.g_count=1,.a_count=1};
    detector_pending=true;publish_observation(&stale);
    assert(!detector_pending && !local_rest && !cal_rest);
    drain(700);assert(known==before);
    main_imu_resume();detector_pending=true;publish_observation(&stale);
    drain(700);assert(known==before && !local_rest && !cal_rest);
    backend.take_rest_observation=NULL;frame_once();drain(300);
    assert(unavailable>0 && known>before);
    backend.take_rest_observation=take_observation;
    rest_motion_contracts();freshness_contracts();angular_window_contracts();
    puts("PASS actual sensor rest/activity/freshness/local eligibility and epoch publication contracts");
}
'''
with tempfile.TemporaryDirectory(prefix='sensor-events-') as tmp:
    tmp = Path(tmp)
    (tmp / 'leaves.h').write_text(leaves.LEAVES)
    for name in ('kernel.h', 'init.h', 'random/random.h', 'logging/log.h'):
        header = tmp / 'zephyr' / name
        header.parent.mkdir(parents=True, exist_ok=True)
        header.write_text('#include "leaves.h"\n')
    c = tmp / 'sensor.c'
    c.write_text('#include "leaves.h"\n#include "' + str(ROOT / 'src/connection/tracker_events.c') + '"\n' + fixture + '\n#include "' + str(ROOT / 'src/util.c') + '"\n')
    binary = tmp / 'sensor'
    subprocess.run(shlex.split(os.environ.get('CC', 'cc')) + ['-std=gnu11', '-O0', '-g', '-Wall', '-Wextra', '-Wno-unused-function', '-Wno-unused-variable', '-I', str(tmp), '-I', str(ROOT / 'src'), str(c), str(ROOT / 'src/sensor/motion_state.c'), '-lm', '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
