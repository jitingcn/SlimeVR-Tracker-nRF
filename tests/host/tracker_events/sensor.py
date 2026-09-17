#!/usr/bin/env python3
"""Actual sensor frame/suspend bodies feeding the production event scheduler."""
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
    start = re.search(r'^(?:static )?void ' + name + r'\([^;]*?\)\s*\{', source, re.M).start()
    opening = source.index('{', start)
    depth = 0
    for token in re.finditer(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|[{}]', source[opening:], re.S):
        if token.group() == '{':
            depth += 1
        elif token.group() == '}':
            depth -= 1
            if depth == 0:
                return source[start:opening + token.end()]
    raise ValueError(name)


loop = function('sensor_loop')
begin = loop.index('sensor_apply_calibration_frame();')
end = loop.index('if (sensor_startup_discard_until_ms', begin)
acquire = loop[begin:end]
publish = function('sensor_loop_publish')
begin = publish.index('int64_t now = k_uptime_get();')
end = publish.index('sensor_update_sensor_state(', begin)
publish = publish[begin:end]
fixture = r'''
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
static bool take_observation(bool *out) {
    bool available=detector_pending; detector_pending=false;
    if(available && out) *out=detector_rest;
    return available;
}
static struct {bool (*take_rest_observation)(bool*);} backend={take_observation};
static const typeof(backend) *sensor_fusion=&backend;
static int fusion_id=FUSION_VQF;
static void sensor_calibration_set_consumer_ready(bool x) {(void)x;}
static void watchdog_pause(int x) {(void)x;}
static void watchdog_resume(int x) {(void)x;}
static void k_thread_suspend(int *x) {(void)x; suspended++;}
static void k_thread_resume(int *x) {(void)x; assert(!main_suspended); resumed++;}
static int k_event_wait(int *event,int mask,bool reset,int timeout) {
    (void)event;(void)reset;assert(mask==SENSOR_LIFE_IDLE && timeout==5000);
    now_ms+=timeout; idle_timeouts++; return 0; /* blocked FIFO: forced suspend */
}
static int64_t k_uptime_ticks(void) {return now_ms;}
static bool sensor_update_resting_state(float *q,float *a,int64_t now,float *g,float *l) {
    (void)q;(void)a;(void)now; *g=*l=0; return true;
}
typedef struct {uint32_t sensor_epoch; bool dc_active; int g_count,a_count;} sensor_loop_frame_t;
static void sensor_apply_calibration_frame(void) {}
static void sensor_loop_handle_data_collection(bool *active) {(void)active;}
'''
fixture += function('main_imu_suspend') + '\n' + function('main_imu_resume')
fixture += r'''
static uint64_t sensor_window_acq_us, sensor_window_acq_max_us;
#define k_ticks_to_us_near64(x) (x)
static bool interrupt_acquire;
static void sensor_loop_acquire(sensor_loop_frame_t *frame) {
    if(interrupt_acquire) {
        detector_pending=true;
        main_imu_suspend(); /* forced suspend while this frame's acquire is blocked */
        assert(!detector_pending);
        now_ms+=2000;
        main_imu_resume(); /* the old acquisition stack continues after resume */
    }
    frame->g_count=1; detector_pending=true;
}
static void publish_observation(sensor_loop_frame_t *frame) {
    float q[4]={1},lin_a[3]={0};
'''
fixture += publish + '\n}\n'
fixture += 'static void frame_once(void) { sensor_loop_frame_t frame={0};\n' + acquire
fixture += '\n(void)acq_begin_ticks; publish_observation(&frame); }\n'
fixture += r'''
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
int main(void) {
    assert(host_init()==0);
    interrupt_acquire=true; frame_once();
    assert(suspended==1 && resumed==1 && idle_timeouts==1);
    assert(!detector_pending); /* stale publication still consumed the detector */
    drain(700);
    assert(known==0 && unknown>0); /* old stack never revives rest with new uptime */
    interrupt_acquire=false; frame_once(); drain(300);
    assert(known>=2); /* only the first genuinely new frame recovers both domains */
    unsigned before=known;
    main_imu_suspend();
    sensor_loop_frame_t stale={.sensor_epoch=tracker_events_sensor_epoch(),.g_count=1};
    detector_pending=true; publish_observation(&stale);
    assert(!detector_pending);
    drain(700); assert(known==before); /* suspended frame cannot publish even matching epoch */
    main_imu_resume();
    detector_pending=true; publish_observation(&stale);
    drain(700); assert(known==before); /* resume invalidates even work captured while suspended */
    backend.take_rest_observation=NULL;
    frame_once(); drain(300);
    assert(unavailable>0 && known>before); /* missing fusion never blocks tracker freshness */
    puts("PASS actual sensor capture/acquire forced-suspend timeout/resume old-stack publication and fresh recovery");
}
'''
with tempfile.TemporaryDirectory(prefix='sensor-events-') as tmp:
    tmp = Path(tmp)
    (tmp / 'leaves.h').write_text(leaves.LEAVES)
    for name in ('kernel.h','init.h','random/random.h','logging/log.h'):
        header = tmp / 'zephyr' / name
        header.parent.mkdir(parents=True, exist_ok=True)
        header.write_text('#include "leaves.h"\n')
    c = tmp / 'sensor.c'
    c.write_text('#include "leaves.h"\n#include "' + str(ROOT / 'src/connection/tracker_events.c') + '"\n' + fixture)
    binary = tmp / 'sensor'
    subprocess.run(shlex.split(os.environ.get('CC','cc')) + ['-std=gnu11','-O0','-g','-Wall','-Wextra','-Wno-unused-function','-Wno-unused-variable','-I',str(tmp),'-I',str(ROOT / 'src'),str(c),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
