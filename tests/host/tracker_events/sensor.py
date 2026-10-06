#!/usr/bin/env python3
"""Production motion/feed/publish/lifecycle bodies and real event scheduler."""
import importlib.util
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
spec = importlib.util.spec_from_file_location('event_leaves', HERE / 'run.py')
leaves = importlib.util.module_from_spec(spec)
spec.loader.exec_module(leaves)
source = Path(os.environ.get('SENSOR_SOURCE', ROOT / 'src/sensor/sensor.c')).read_text()
power = (ROOT / 'src/system/power.c').read_text()


def function(name, source=source):
    return extract_block(source, r'^(?:static )?(?:void|int|int64_t|bool|float|uint32_t) ' + re.escape(name) + r'\([^;]*?\)\s*\{')


loop = function('sensor_loop')
begin = loop.index('sensor_apply_calibration_frame();')
acquire = loop[begin:loop.index('if (sensor_startup_discard_until_ms', begin)]
publish = function('sensor_loop_publish')
begin = publish.index('// Get updated quaternion from fusion')
publish_policy = publish[begin:publish.index('sensor_diagnostics_output(', begin)]
begin = publish.index('// Update orientation')
transmit_policy = publish[begin:publish.index('// Periodic retained save', begin)]
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
#define CONFIG_USE_ACTIVE_TIMEOUT 1
#define CONFIG_SLEEP_ON_ACTIVE_TIMEOUT 1
#define CONFIG_USE_IMU_WAKE_UP 1
#define CONFIG_ACTIVE_TIMEOUT_DELAY 600000
#define CONFIG_ACTIVE_TIMEOUT_THRESHOLD 15000
#define CONFIG_ACTIVE_TIMEOUT_REPEAT_WAKE_COUNT 3
#define CONFIG_ACTIVE_TIMEOUT_REPEAT_WAKE_DELAY 15000
#define CONFIG_ACTIVE_TIMEOUT_IDLE_WAKE_DELAY 15000
#define CONFIG_SENSOR_LP_TIMEOUT 500
#define CONFIG_DELAY_SLEEP_ON_STATUS 1
#define IMU_INT_EXISTS 1
#define K_FOREVER (-1)
#define K_MUTEX_DEFINE(name) struct {bool held;} name
#define k_mutex_lock(p,t) do {(void)(t);assert(!(p)->held);(p)->held=true;} while(0)
#define k_mutex_unlock(p) do {assert((p)->held);(p)->held=false;} while(0)
struct k_sem {unsigned count;};
#define K_SEM_DEFINE(name,initial,limit) struct k_sem name={initial}
static void k_sem_give(struct k_sem *p) {p->count++;}
#include "system/power_request.h"
#include "system/status.h"
static int status_state;
int get_status(enum sys_status mask) {return (status_state&mask)!=0;}
static bool test_mode_get(void) {return false;}
static bool esb_ota_is_active(void) {return false;}
static bool connection_get_ota_suppressed(void) {return false;}
static bool esb_ready(void) {return true;}
#ifndef MAX
#define MAX(a,b) ((a)>(b)?(a):(b))
#endif
#define SENSOR_ACTIVITY_STARTUP_GUARD_MS 5000
#define CONFIG_ACTIVE_TIMEOUT_MEANINGFUL_MOTION_MS 3000
#define WDT_CHANNEL_SENSOR 0
#define SENSOR_LIFE_IDLE 1
#define SENSOR_LIFE_SCAN_DONE 2
#define K_MSEC(x) (x)
#define atomic_set(p,v) (*(p)=(v))
#define atomic_get(p) (*(p))
#define atomic_clear(p) atomic_set(p,0)
#define FUSION_VQF 1
#define FUSION_EQF 2
static bool main_suspended, main_running=true, main_ok=true, sensor_sensor_scanning;
static int output_ready;
static int sensor_thread_id, sensor_life_events;
static unsigned suspended, resumed, idle_timeouts;
static bool detector_pending, detector_rest=true;
static float pose[4]={1},pose6[4]={1},q[4],sensor_loop_avg_a[3]={0,0,1};
static bool independent_heading, invalid_6d;
static float last_q[4]={1},last_lin_a[3],transmitted_q[4];
static int64_t last_sensor_send_time,last_suspend_attempt_time,sensor_data_time;
static float sensor_window_fused_angle_rad;
static unsigned sensor_window_publishes,transmissions;
static int test_mode_min_send_interval_ms(void) {return 1000;}
static void sensor_compute_device_and_reported_quat(float *src,float *device,float *reported) {
    memcpy(device,src,4*sizeof(float));memcpy(reported,src,4*sizeof(float));
}
static void sensor_rotate_sensor_vector_to_device_frame(float *src,float *out) {(void)src;(void)out;}
static void connection_update_sensor_data(float *pose,float *lin,int64_t time) {
    (void)lin;(void)time;memcpy(transmitted_q,pose,sizeof(transmitted_q));transmissions++;
}
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
static void get_quat6(float *out) {
    memcpy(out,pose6,sizeof(pose6));
    if(invalid_6d) out[0]=NAN;
}
static void update_gyro(float *g,float dt) {(void)g;(void)dt;}
static struct {bool (*take_rest_observation)(bool*); void (*get_quat)(float*); void (*update_gyro)(float*,float); void (*get_quat6)(float*);}
    backend={take_observation,get_quat,update_gyro,get_quat6};
static const typeof(backend) *sensor_fusion=&backend;
static int fusion_id=FUSION_VQF;
static void sensor_diagnostics_on_cal_gyro(float *g) {(void)g;}
static void sensor_calibration_set_consumer_ready(bool x) {(void)x;}
static void watchdog_pause(int x) {(void)x;}
static void watchdog_resume(int x) {(void)x;}
static void k_thread_suspend(int *x) {(void)x; suspended++;}
static void k_thread_resume(int *x) {(void)x; assert(!main_suspended); resumed++;}
static int k_event_wait(int *event,int mask,bool reset,int timeout) {
    (void)event;(void)reset;assert(mask==SENSOR_LIFE_IDLE && timeout==5000);
    now_ms+=timeout; idle_timeouts++; return 0;
}
static int64_t k_uptime_ticks(void) {return now_ms;}
static void sensor_update_sensor_state(bool resting);
static void sensor_runtime_calibration_check(bool resting) {cal_rest=resting;}
static void sensor_tcal_continuous_motion_detected(void) {cal_rest=false;}
static void sensor_tcal_boot_calibration_check(void) {}
static void sensor_tcal_check_auto_calibration(float t) {(void)t;auto_calibrations++;}
static bool sensor_tcal_get_auto_calibration(void) {return false;}
typedef struct {uint32_t sensor_epoch; bool dc_active; int g_count,a_count;} sensor_loop_frame_t;
static void sensor_apply_calibration_frame(void) {}
static bool collection_failure, acquisition_failure;
static unsigned frame_waits, frame_acquires, frame_publishes;
static void sensor_loop_handle_data_collection(bool *active) {
    (void)active;
    if(collection_failure) main_ok=false;
}
static void sensor_loop_wait(int64_t time_begin) {
    assert(time_begin<=now_ms);
    frame_waits++;
}
'''
fixture += function('status_ready', (ROOT / 'src/system/status.c').read_text()) + '\n'
for pattern in (r'^static struct power_request_mailbox power_requests;',
                r'^static K_SEM_DEFINE\(power_wake_sem,.*?;',
                r'^static K_MUTEX_DEFINE\(power_plan_lock\);',
                r'^static (?:bool|int64_t) wom_[^;]+;',
                r'^#define WOM_ELIGIBILITY_LEASE_MS .*$'):
    fixture += '\n'.join(re.findall(pattern, power, re.MULTILINE)) + '\n'
for name in ('sys_cancel_WOM_locked', 'sys_cancel_WOM', 'sys_wom_ready', 'sys_plan_WOM'):
    fixture += function(name, power) + '\n'
for name in ('sensor_sensor_mode', 'sensor_sensor_timeout'):
    fixture += extract_block(source, rf'^enum {name} \{{', semicolon=True) + '\n'
for name in ('sensor_mode', 'sensor_timeout', 'was_ota_suppressed'):
    fixture += re.search(rf'^static [^\n]* {name}[^;]*;', source, re.MULTILINE).group() + '\n'
fixture += function('sensor_get_active_timeout_delay') + '\n' + function('sensor_update_sensor_state') + '\n'
fixture += dwell + '\n' + rest + '\n' + function('feed_calibrated_gyro') + '\n' + function('sensor_update_session_motion') + '\n'
fixture += function('main_imu_suspend') + '\n' + function('main_imu_resume') + '\n'
fixture += r'''
static uint64_t sensor_window_acq_us, sensor_window_acq_max_us;
#define k_ticks_to_us_near64(x) (x)
static bool interrupt_acquire;
static void sensor_loop_acquire(sensor_loop_frame_t *frame) {
    frame_acquires++;
    if(acquisition_failure) {main_ok=false;return;}
    if(interrupt_acquire) {
        detector_pending=true;
        assert(main_imu_suspend()==0); assert(!detector_pending);
        now_ms+=2000; main_imu_resume();
    }
    frame->g_count=frame->a_count=1; detector_pending=true;
}
static void publish_observation(sensor_loop_frame_t *frame) {
    frame_publishes++;
    local_rest=false;
'''
fixture += publish_policy + 'local_rest=resting;\n' + transmit_policy + '\n}\n'
fixture += 'static void frame_once(void) { for(unsigned iteration=0;iteration<1;iteration++) { int64_t time_begin=now_ms; sensor_loop_frame_t frame={0};\n' + acquire
fixture += r'''
    (void)acq_begin_ticks;
    sensor_motion_prepare(frame.sensor_epoch,now_ms);
    sensor_rest_detector_update_accel(&rest_detector,sensor_loop_avg_a,accel_actual_time);
    publish_observation(&frame);
    sensor_loop_wait(time_begin);
    }
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
    sys_cancel_WOM();status_state=0;
    sensor_mode=SENSOR_SENSOR_MODE_LOW_NOISE;sensor_timeout=SENSOR_SENSOR_TIMEOUT_IMU;
    last_data_time=last_sensor_send_time=0;
    independent_heading=invalid_6d=false;backend.get_quat6=get_quat6;
    sensor_session_woke_from_wom=true;
    sensor_session_meaningful_motion=false;
    gyro_actual_time=.006f; accel_actual_time=.01f; sensor_update_time_ms=6;
    now_ms=0; pose[0]=1;pose[1]=pose[2]=pose[3]=0;
    sensor_loop_avg_a[0]=sensor_loop_avg_a[1]=0;sensor_loop_avg_a[2]=1;
    memcpy(pose6,pose,sizeof(pose6));memcpy(last_q,pose,sizeof(last_q));
    memset(last_lin_a,0,sizeof(last_lin_a));
    local_rest=cal_rest=false;
}
static void yaw(float degrees) {
    float half=degrees*(3.14159265358979323846f/360.0f);
    pose[0]=cosf(half);pose[1]=pose[2]=0;pose[3]=sinf(half);
    if(!independent_heading) memcpy(pose6,pose,sizeof(pose6));
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
        for(unsigned j=0;j<4;j++)pose6[j]=-pose6[j];
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
static void magnetic_heading_sleep_contracts(void) {
    /* Keep 6D stationary while magnetic correction moves 9D beyond the
     * rest entry gate. Exercise the actual publisher and power planner. */
    reset_motion();independent_heading=true;
    sensor_session_woke_from_wom=false; /* full normal 10-minute policy */
    status_state=SYS_STATUS_USB_CONNECTED|SYS_STATUS_SERIAL_ACTIVE;
    assert(!status_ready());
    pose6[0]=2; /* publisher normalizes non-unit finite IMU attitude */
    unsigned before=transmissions;
    for(unsigned t=0;t<=602000;t+=10) {
        bool resting=sample(t,1,1,.75f*t/1000);
        if(t>=1000) assert(resting && cal_rest);
    }
    assert(wom_planned && wom_announced && wom_force);
    assert(power_requests.request==SYS_POWER_REQ_WOM_FORCE);
    assert(sensor_timeout==SENSOR_SENSOR_TIMEOUT_ACTIVITY_ELAPSED);
    assert(wom_deadline==last_data_time+600000 && now_ms>wom_deadline);
    assert(transmissions>before+600);
    assert(sensor_motion_quat_angle(transmitted_q,pose)<.01f);
    assert(sensor_motion_quat_angle(transmitted_q,pose6)>1);
    assert(fabsf(q[0]*q[0]+q[3]*q[3]-1)<.00001f);
    assert(!sensor_session_meaningful_motion && sensor_session_activity_score.value_ms==0);

    /* Heading correction must not promote an idle wake to real activity. */
    reset_motion();independent_heading=true;
    for(unsigned t=0;t<=17000;t+=10) sample(t,1,1,3.0f*t/1000);
    assert(local_rest && wom_planned && wom_force);
    assert(!sensor_session_meaningful_motion && sensor_session_activity_score.value_ms==0);

    reset_motion();sensor_session_woke_from_wom=false;
    for(unsigned t=0;t<=602000;t+=10) {
        assert(!sample(t,1,1,.75f*t/1000));
        assert(!wom_planned);
    }
    /* NULL is the EqF contract, not fabricated 6D: preserve legacy policy. */
    reset_motion();backend.get_quat6=NULL;
    for(unsigned t=0;t<=17000;t+=10) sample(t,1,1,0);
    assert(local_rest && wom_planned);
    reset_motion();backend.get_quat6=NULL;
    for(unsigned t=0;t<=17000;t+=10) assert(!sample(t,1,1,.75f*t/1000));
    assert(!wom_planned);

    /* Invalid available 6D and stale channels cannot earn sleep even with
     * valid corrected output continuing to transmit. */
    for(unsigned fault=0;fault<5;fault++) {
        reset_motion();independent_heading=true;sensor_session_woke_from_wom=false;
        for(unsigned t=0;t<=602000;t+=10) {
            if(t==2000) {
                if(fault==0) invalid_6d=true;
                if(fault==1) memset(pose6,0,sizeof(pose6));
                if(fault==2) pose6[0]=INFINITY;
            }
            sample(t,!(t>=2000 && fault==3),!(t>=2000 && fault==4),.75f*t/1000);
            if(t>=2100) assert(!local_rest && !cal_rest && !wom_planned);
        }
        assert(output_ready && !wom_planned);
    }
}
static void pm_frame_failure_contracts(void) {
    unsigned waits=frame_waits, acquires=frame_acquires, publishes=frame_publishes;
    collection_failure=true;
    frame_once();
    assert(!main_ok && frame_waits==waits+1 && frame_acquires==acquires && frame_publishes==publishes);
    collection_failure=false; main_ok=true; acquisition_failure=true;
    frame_once();
    assert(!main_ok && frame_waits==waits+2 && frame_acquires==acquires+1 && frame_publishes==publishes);
    acquisition_failure=false; main_ok=true;
    frame_once();
    assert(frame_waits==waits+3 && frame_acquires==acquires+2 && frame_publishes==publishes+1);
}
int main(void) {
    assert(host_init()==0);
    interrupt_acquire=true;frame_once();
    assert(suspended==1 && resumed==1 && idle_timeouts==1);
    assert(!detector_pending && !local_rest && !cal_rest);
    drain(700);assert(known==0 && unknown>0);
    interrupt_acquire=false;frame_once();drain(300);assert(known>=2);
    unsigned before=known;
    assert(main_imu_suspend()==0);
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
    pm_frame_failure_contracts();
    magnetic_heading_sleep_contracts();
    puts("PASS actual sensor rest/activity/freshness/local eligibility and epoch publication contracts");
}
'''
with tempfile.TemporaryDirectory(prefix='sensor-events-') as tmp:
    tmp = Path(tmp)
    (tmp / 'leaves.h').write_text(leaves.LEAVES)
    for name in ('kernel.h', 'spinlock.h', 'init.h', 'random/random.h', 'logging/log.h'):
        header = tmp / 'zephyr' / name
        header.parent.mkdir(parents=True, exist_ok=True)
        header.write_text('#include "leaves.h"\n')
    c = tmp / 'sensor.c'
    c.write_text('#include "leaves.h"\n#include "' + str(ROOT / 'src/connection/tracker_events.c') + '"\n' + fixture + '\n#include "' + str(ROOT / 'src/util.c') + '"\n')
    binary = tmp / 'sensor'
    subprocess.run(shlex.split(os.environ.get('CC', 'cc')) + ['-std=gnu11', '-O0', '-g', '-Wall', '-Wextra', '-Wno-unused-function', '-Wno-unused-variable', '-I', str(tmp), '-I', str(ROOT / 'src'), str(c), str(ROOT / 'src/sensor/motion_state.c'), '-lm', '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)

# The heater consumes a different, coherent temperature observation contract.
# Exercise production publication/getter/lifecycle bodies independently of the
# fusion event fixture; physical reads and the core gate are hardware leaves.
temperature_header = (ROOT / 'src/sensor/sensor.h').read_text()
observation = extract_block(temperature_header, r'struct sensor_temperature_observation \{', semicolon=True)
temperature_fixture = r'''
#include "util.h"
#define K_FOREVER (-1)
#define K_MUTEX_DEFINE(name) struct { bool held; } name
#define k_mutex_lock(lock, timeout) do { (void)(timeout); assert(!(lock)->held); (lock)->held=true; } while(0)
#define k_mutex_unlock(lock) do { assert((lock)->held); (lock)->held=false; } while(0)
#define atomic_get(ptr) (*(ptr))
#define TCAL_HEATED_STOP_SENSOR_STOP 1
static bool main_ok=true, main_suspended, power_ready=true, ota_active, suppressed, core_ready;
static int off_error;
static float sensor_tcal_temp_raw, sensor_tcal_temp;
static bool heater_power_ready(void) {return power_ready;}
static bool esb_ota_is_active(void) {return ota_active;}
static bool connection_get_ota_suppressed(void) {return suppressed;}
static void sensor_tcal_heated_set_ready(bool ready) {core_ready=ready;}
static int sensor_tcal_heated_abort(int reason) {(void)reason; core_ready=false; return off_error;}
'''
temperature_fixture += observation + '\n'
temperature_fixture += source[source.index('static struct k_spinlock temperature_observation_lock;'):
                              source.index('int sensor_get_imu_temperature_observation(')]
for name in ('sensor_get_imu_temperature_observation', 'sensor_temperature_invalidate',
             'sensor_temperature_resume', 'sensor_temperature_read_epoch', 'sensor_temperature_publish'):
    temperature_fixture += function(name) + '\n'
temperature_fixture += r'''
int main(void) {
    struct sensor_temperature_observation sample={0};
    assert(sensor_get_imu_temperature_observation(NULL,2000)==-EINVAL);
    assert(sensor_get_imu_temperature_observation(&sample,-1)==-EINVAL);
    assert(sensor_get_imu_temperature_observation(&sample,2000)==-EAGAIN);
    now_ms=100; sensor_tcal_temp_raw=25; sensor_tcal_temp=24.5f;
    sensor_temperature_resume();
    uint32_t epoch=sensor_temperature_read_epoch();
    sensor_temperature_publish(epoch,now_ms);
    assert(sensor_get_imu_temperature_observation(&sample,2000)==0);
    assert(sample.raw_c==25 && sample.filtered_c==24.5f && sample.sequence==1 && sample.sampled_at_ms==100);
    now_ms=2100;
    assert(sensor_get_imu_temperature_observation(&sample,2000)==0);
    now_ms++;
    assert(sensor_get_imu_temperature_observation(&sample,2000)==-EAGAIN);
    now_ms=99;
    assert(sensor_get_imu_temperature_observation(&sample,2000)==-EAGAIN);
    now_ms=2200;
    sensor_temperature_publish(epoch,now_ms); /* Equal numeric value, new read. */
    assert(sensor_get_imu_temperature_observation(&sample,2000)==0);
    assert(sample.sequence==2 && sample.sampled_at_ms==2200);
    assert(sensor_temperature_invalidate()==0);
    sensor_temperature_resume();
    sensor_temperature_publish(epoch,now_ms); /* In-flight read crossing stop. */
    assert(sensor_get_imu_temperature_observation(&sample,2000)==-EAGAIN);
    epoch=sensor_temperature_read_epoch();
    main_suspended=true;
    sensor_temperature_publish(epoch,now_ms);
    assert(sensor_get_imu_temperature_observation(&sample,2000)==-EAGAIN);
    main_suspended=false; main_ok=false;
    sensor_temperature_publish(epoch,now_ms);
    assert(sensor_get_imu_temperature_observation(&sample,2000)==-EAGAIN);
    main_ok=true; sensor_tcal_temp_raw=NAN;
    sensor_temperature_publish(epoch,now_ms);
    assert(sensor_get_imu_temperature_observation(&sample,2000)==-EAGAIN);
    sensor_tcal_temp_raw=25; sensor_tcal_temp=INFINITY;
    sensor_temperature_publish(epoch,now_ms);
    assert(sensor_get_imu_temperature_observation(&sample,2000)==-EAGAIN);
    sensor_tcal_temp=24.5f;
    sensor_temperature_publish(epoch,now_ms);
    assert(sensor_get_imu_temperature_observation(&sample,2000)==0 && sample.sequence==3);
    off_error=-ETIMEDOUT;
    assert(sensor_temperature_invalidate()==-ETIMEDOUT);
    assert(sensor_get_imu_temperature_observation(&sample,2000)==-EAGAIN);
    puts("PASS actual temperature observation freshness, sequence, lifecycle and off-error contracts");
}
'''
with tempfile.TemporaryDirectory(prefix='sensor-temperature-') as tmp:
    tmp = Path(tmp)
    (tmp / 'leaves.h').write_text(leaves.LEAVES)
    c = tmp / 'temperature.c'
    # v_finite needs only the standard bool/size/int/memcpy declarations
    # already supplied by leaves.h, not util.c's unrelated Zephyr helpers.
    c.write_text('#include <math.h>\n#include "leaves.h"\n' + temperature_fixture +
                 '\n' + function('v_finite', (ROOT / 'src/util.c').read_text()) + '\n')
    binary = tmp / 'temperature'
    subprocess.run(shlex.split(os.environ.get('CC', 'cc')) + [
        '-std=gnu11', '-O2', '-ffast-math', '-g', '-Wall', '-Wextra',
        '-Wno-unused-function', '-Wno-unused-variable', '-I', str(tmp),
        '-I', str(ROOT / 'src'), str(c), '-lm', '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
