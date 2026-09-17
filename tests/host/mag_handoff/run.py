#!/usr/bin/env python3
"""Exercise production magnetic-domain rebases and gravity evidence on host."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

ROOT = Path(os.environ.get('SOURCE_ROOT', Path(__file__).resolve().parents[3]))

def function(source, name):
    match = re.search(rf'^(?:static )?(?:ALWAYS_INLINE )?(?:void|bool|float) {name}\([^;{{]*\)\s*\{{', source, re.M)
    if not match:
        raise ValueError(name)
    depth = 0
    start = source.index('{', match.start())
    for token in re.finditer(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|[{}]', source[start:], re.S):
        if token.group() == '{':
            depth += 1
        elif token.group() == '}':
            depth -= 1
            if not depth:
                return source[match.start():start + token.end()]
    raise ValueError(name)

preamble = '''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
'''
vqf = (ROOT / 'src/sensor/fusion/vqf/vqf.c').read_text()
vqf_test = preamble + '''
#include "vqf.h"
static vqf_params_t params;
static vqf_state_t state;
static vqf_coeffs_t coeffs;
static bool rest_observation_pending;
static float last_a[3];
#define ARG_UNUSED(x) ((void)(x))
#define IS_ENABLED(x) 0
#define DEG_TO_RAD 0.01745329251994329577f
#define CONST_EARTH_GRAVITY 9.80665f
static void vqf_track_rest_diag(void) {}
#define ALWAYS_INLINE inline
#define BUILD_ASSERT(c,m) _Static_assert(c,m)
#define printk(...) ((void)0)
#define VQF_MEM_SIZE (sizeof(vqf_state_t) + sizeof(vqf_coeffs_t))
struct retained_data { unsigned char fusion_data[2048]; };
static float vqf_init_gyr_time, vqf_init_acc_time, vqf_init_mag_time;
static uint32_t rest_enter_count, rest_exit_count;
static float rest_total_s, rest_last_enter_time, rest_last_duration_s, uptime_s;
static bool prev_rest_detected;
static uint8_t rest_event_idx, rest_event_total;
''' + '\n'.join(function(vqf, name) for name in (
    'set_params', 'vqf_float_finite', 'vqf_vec3_finite', 'vqf_loaded_state_valid',
    'vqf_safe_init_time', 'vqf_init', 'vqf_load', 'vqf_update_gyro', 'vqf_update_accel',
    'vqf_get_rest_detected', 'vqf_take_rest_observation', 'vqf_rebase_mag')) + r'''
int main(void) {
    vqf_init(.01f,.01f,.01f);
    bool rest = true;
    float zero[3]={0}, invalid[3]={NAN,0,1}, still[3]={0,0,1};
    assert(!vqf_take_rest_observation(&rest) && rest);
    vqf_update_accel(zero,.01f); vqf_update_accel(invalid,.01f);
    vqf_update_gyro(invalid,.01f);
    assert(!vqf_take_rest_observation(&rest));
    for(int i=0;i<500;i++) { vqf_update_gyro(zero,.01f); vqf_update_accel(still,.01f); }
    assert(vqf_take_rest_observation(&rest) && rest);
    assert(!vqf_take_rest_observation(&rest));
    float moving[3]={200,0,0}; vqf_update_gyro(moving,.01f);
    assert(vqf_take_rest_observation(&rest) && !rest);
    vqf_update_accel(still,.01f);
    assert(vqf_take_rest_observation(NULL) && !vqf_take_rest_observation(&rest));
    unsigned char saved[VQF_MEM_SIZE];
    memcpy(saved,&state,sizeof(state)); memcpy(saved+sizeof(state),&coeffs,sizeof(coeffs));
    vqf_update_accel(still,.01f); vqf_load(saved);
    assert(!vqf_take_rest_observation(&rest));
    vqf_state_t bad_state; memcpy(&bad_state,saved,sizeof(bad_state));
    bad_state.bias[0]=NAN; memcpy(saved,&bad_state,sizeof(bad_state));
    vqf_update_accel(still,.01f); vqf_load(saved);
    assert(!vqf_take_rest_observation(&rest)); /* rejection/reinit early return */
    vqf_update_accel(still,.01f); vqf_init(.01f,.01f,.01f);
    assert(!vqf_take_rest_observation(&rest));
    float g[3] = {.12f, -.21f, .4f}, a[3] = {0, 0, 9.81f};
    for (int i=0;i<100;i++) { updateGyr(&params,&state,&coeffs,g); updateAcc(&params,&state,&coeffs,a); }
    state.delta = .7f;
    float q[4], bias[3], cov[9];
    getQuat6D(&state,q); memcpy(bias,state.bias,sizeof(bias)); memcpy(cov,state.biasP,sizeof(cov));
    state.magCandidateT=100; state.magUndisturbedT=100; state.magRejectT=10000;
    state.kMagInit=1;
    for (int i=0;i<4;i++) state.magNormDipLpState[i]=12345;
    vqf_rebase_mag(2,0);
    float after[4]; getQuat6D(&state,after);
    assert(!memcmp(q,after,sizeof(q)) && !memcmp(bias,state.bias,sizeof(bias)));
    assert(!memcmp(cov,state.biasP,sizeof(cov)) && state.delta==.7f);
    assert(state.magCandidateT==0 && state.magUndisturbedT==0 && state.magRejectT==0);
    for (int i=0;i<4;i++) assert(isnan(state.magNormDipLpState[i]));
    float m[3]={2,0,0}; updateMag(&params,&state,&coeffs,m);
    assert(state.delta==.7f); /* no first-feed startup gain override */
    assert(state.magNormDip[0] < 3 && state.magNormDip[0] > 1);
    vqf_rebase_mag(0,0); updateMag(&params,&state,&coeffs,m);
    assert(state.delta==.7f && state.magDistDetected);
    puts("VQF magnetic-only rebase preserves attitude/bias and rejects startup snap");
}
'''
eqf = (ROOT / 'src/sensor/fusion/eqf/eqf.c').read_text()
eqf = re.sub(r'^#include[^\n]*\n', '', eqf, flags=re.M).split('const sensor_fusion_t sensor_fusion_eqf')[0]
eqf_test = preamble + '''
#define BUILD_ASSERT(c,m) _Static_assert(c,m)
#define CONST_EARTH_GRAVITY 9.80665f
struct retained_data { unsigned char fusion_data[1024]; };
''' + eqf + r'''
int main(void) {
    eqf_init(.01f,.01f,.01f);
    bool rest=true;
    float zero[3]={0}, still[3]={0,0,1}, invalid[3]={0,0,9};
    assert(!eqf_take_rest_observation(&rest) && rest);
    eqf_update_accel(still,.01f); eqf_update_gyro(zero,.01f);
    assert(!eqf_take_rest_observation(&rest)); /* INIT is not an observation. */
    mode=EQF_RUNNING;
    eqf_update_gyro(zero,.01f);
    eqf_update_accel(invalid,.01f);
    assert(!eqf_take_rest_observation(&rest)); /* gyro-only and rejected accel */
    eqf_update_accel(still,.01f);
    assert(eqf_take_rest_observation(&rest));
    assert(!eqf_take_rest_observation(&rest));
    eqf_update_accel(still,.01f);
    assert(eqf_take_rest_observation(NULL) && !eqf_take_rest_observation(&rest));
    eqf_update_accel(still,.01f); eqf_init(.01f,.01f,.01f);
    assert(!eqf_take_rest_observation(&rest));
    mode=EQF_RUNNING; eqf_update_accel(still,.01f);
    unsigned char saved[1024]={0}; eqf_load(saved); /* Invalid-load early return. */
    assert(!eqf_take_rest_observation(&rest));
    mode=EQF_RUNNING; eqf_update_accel(still,.01f); eqf_save(saved); eqf_load(saved);
    assert(!eqf_take_rest_observation(&rest));
    st.a_vec[0]=.01f;
    float A[9], b[3], P[36];
    memcpy(A,st.A,sizeof(A)); memcpy(b,st.a_vec,sizeof(b)); memcpy(P,st.P,sizeof(P));
    mag_norm_dip_lp_init=true; mag_candidate_t=100; mag_reject_t=100;
    eqf_rebase_mag(2,0);
    assert(!mag_norm_dip_lp_init && mag_candidate_t==0 && mag_reject_t==0);
    assert(!memcmp(A,st.A,sizeof(A)) && !memcmp(b,st.a_vec,sizeof(b)) && !memcmp(P,st.P,sizeof(P)));
    float m[3]={0,2,0}; eqf_update_mag(m,.01f);
    assert(!memcmp(A,st.A,sizeof(A)) && !memcmp(b,st.a_vec,sizeof(b)));
    eqf_rebase_mag(0,0); eqf_update_mag(m,.01f);
    assert(!memcmp(A,st.A,sizeof(A)) && !mag_ref_valid && mag_dist_detected);
    puts("EqF rebase preserves attitude/bias/covariance and waits for independent reacquisition");
}
'''
sensor = (ROOT / 'src/sensor/sensor.c').read_text()
gravity_test = preamble + r'''
static float quaternion[4]={1,0,0,0};
static bool resting;
static void get_q(float *q) { memcpy(q,quaternion,sizeof(quaternion)); }
static bool get_rest(void) { return resting; }
static struct {void (*get_quat6)(float*); bool (*get_rest_detected)(void);} backend={get_q,get_rest};
static const typeof(backend) *sensor_fusion=&backend;
''' + function(sensor, 'sensor_mag_gravity') + r'''
int main(void) {
    float up[3], a[3]={0,0,1};
    assert(sensor_mag_gravity(a,1,up));
    float mx=up[0],my=up[1],mz=up[2];
    float aligned[3]={SENSOR_MAGNETOMETER_AXES_ALIGNMENT};
    assert(fabsf(aligned[0])<1e-6f && fabsf(aligned[1])<1e-6f && fabsf(aligned[2]-1)<1e-6f);
    quaternion[0]=quaternion[2]=sqrtf(.5f); a[0]=-1; a[1]=a[2]=0;
    assert(sensor_mag_gravity(a,1,up));
    mx=up[0]; my=up[1]; mz=up[2];
    float tilted[3]={SENSOR_MAGNETOMETER_AXES_ALIGNMENT};
    assert(fabsf(tilted[0]+1)<1e-6f && fabsf(tilted[1])<1e-6f && fabsf(tilted[2])<1e-6f);
    quaternion[0]=1; quaternion[2]=0; a[0]=0; a[2]=1;
    assert(!sensor_mag_gravity(a,0,up));
    a[0]=1; a[2]=0; assert(!sensor_mag_gravity(a,1,up));
    a[0]=NAN; assert(!sensor_mag_gravity(a,1,up));
    backend.get_quat6=NULL; a[0]=0; a[2]=1;
    assert(!sensor_mag_gravity(a,1,up)); resting=true;
    assert(sensor_mag_gravity(a,1,up)); a[2]=1.3f;
    assert(!sensor_mag_gravity(a,1,up));
    puts("Gravity evidence: signed inverse mapping, freshness, angle/norm and EqF rest gate");
}
'''
with tempfile.TemporaryDirectory(prefix='mag-handoff-') as tmp:
    tmp = Path(tmp)
    cc = shlex.split(os.environ.get('CC', 'cc'))
    jobs = [('vqf', vqf_test, ['-I'+str(ROOT/'vqf-c/src'), str(ROOT/'vqf-c/src/vqf.c')]),
            ('eqf', eqf_test, [])]
    for index, axes in enumerate(('mx,my,mz','my,-mx,-mz','my,mx,mz','-mx,my,mz')):
        jobs.append((f'gravity{index}', gravity_test, ['-DSENSOR_MAGNETOMETER_AXES_ALIGNMENT='+axes]))
    for name, text, extra in jobs:
        source = tmp/(name+'.c'); source.write_text(text)
        executable = tmp/name
        subprocess.run(cc+['-std=gnu11','-O1','-g',str(source)]+extra+['-lm','-o',str(executable)],check=True)
        subprocess.run([str(executable)],check=True)
