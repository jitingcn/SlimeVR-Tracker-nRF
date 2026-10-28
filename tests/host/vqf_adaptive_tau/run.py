#!/usr/bin/env python3
"""Exercise the real VQF wrapper/core, including A/P6 and retained lifecycle."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(os.environ.get('SOURCE_ROOT', Path(__file__).resolve().parents[3]))
PARAMETER_FIELDS = (
    'tauAcc', 'biasClip', 'biasForgettingTime', 'biasSigmaInit', 'biasSigmaMotion',
    'biasSigmaRest', 'biasVerticalForgettingFactor', 'motionBiasEstEnabled',
    'restBiasEstEnabled', 'restFilterTau', 'restMinT', 'restThGyr', 'restThAcc',
    'magDistRejectionEnabled', 'tauMag', 'magCurrentTau', 'magNormTh', 'magDipTh',
    'magRefTau', 'magNewTime', 'magNewFirstTime', 'magNewMinGyr',
    'magMinUndisturbedTime', 'magMaxRejectionTime', 'magRejectionFactor',
)


def translation_unit():
    source = (ROOT / 'src/sensor/fusion/vqf/vqf.c').read_text()
    # Remove complete unrelated constructs, never splice through conditional
    # guards: the public quaternion/linear-ACC interfaces stay real firmware.
    helper = ROOT / 'tests/host/harness/python/c_extract.py'
    spec = importlib.util.spec_from_file_location('vqf_host_c_extract', helper)
    extract = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(extract)
    for pattern, semicolon in (
        (r'^void vqf_get_debug_info\([^;{}]*\)\s*\{', False),
        (r'^const sensor_fusion_t sensor_fusion_vqf\s*=\s*\{', True),
    ):
        block = extract.extract_block(source, pattern, semicolon=semicolon)
        source = source.replace(block, '', 1)
    source = re.sub(r'^#include[^\n]*\n', '', source, flags=re.M)
    threshold_export = r'''
#ifdef VQF_HOST_THRESHOLD_OVERRIDE
static bool host_threshold_override_active;
static float host_threshold_gyr,host_threshold_acc;
void firmware_strategy_a_override_thresholds(float gyr,float acc) {
    assert(vqf_float_finite(gyr) && gyr>0 && vqf_float_finite(acc) && acc>0);
    host_threshold_gyr=gyr;host_threshold_acc=acc;host_threshold_override_active=true;
}
void firmware_strategy_a_clear_threshold_override(void) {
    host_threshold_override_active=false;
}
static void firmware_apply_threshold_override(void) {
    if(host_threshold_override_active)
        setRestDetectionThresholds(&params,host_threshold_gyr,host_threshold_acc);
}
#endif
'''
    parameter_export = (
        '\nconst char *firmware_strategy_a_param_names(void) { return "' +
        ','.join(PARAMETER_FIELDS) + '"; }\nvoid firmware_strategy_a_params(float *out) {\n'
        '    vqf_init(.01f,.01f,.01f);\n'
        '#ifdef VQF_HOST_THRESHOLD_OVERRIDE\n'
        '    firmware_apply_threshold_override();\n'
        '#endif\n' +
        ''.join(f'    out[{index}]=(float)params.{name};\n'
                for index,name in enumerate(PARAMETER_FIELDS)) + '}\n'
    )
    return r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "vqf.h"
#define IS_ENABLED(x) (x)
#define ARG_UNUSED(x) ((void)(x))
#define ALWAYS_INLINE inline
#define BUILD_ASSERT(c,m) _Static_assert(c,m)
#define printk(...) ((void)0)
#define CONST_EARTH_GRAVITY 9.80665f
#ifndef CONFIG_SENSOR_VQF_OUTPUT_OBSERVER
#define CONFIG_SENSOR_VQF_OUTPUT_OBSERVER 0
#endif
#if CONFIG_SENSOR_VQF_OUTPUT_OBSERVER
struct retained_data { unsigned char fusion_data[904]; };
#else
struct retained_data { unsigned char fusion_data[840]; };
#endif
/* Runtime mode belongs to the existing sensor owner, not the fusion payload. */
static bool host_mag_available, host_mag_enabled;
bool sensor_get_mag_available(void) { return host_mag_available; }
bool sensor_get_mag_enabled(void) { return host_mag_enabled; }
void vqf_get_quat(float *q);
''' + source + threshold_export + parameter_export + r'''
_Static_assert(VQF_MEM_SIZE==sizeof(((struct retained_data *)0)->fusion_data), "Persisted VQF ABI changed");
void firmware_run(const float *g, const float *a, int n, float gt, float at, float *q) {
    vqf_init(gt, at, gt);
#ifdef VQF_HOST_THRESHOLD_OVERRIDE
    firmware_apply_threshold_override();
#endif
    for (int i=0;i<n;i++) {
        float gyro[3], accel[3];
        memcpy(gyro,g+3*i,sizeof(gyro)); memcpy(accel,a+3*i,sizeof(accel));
        vqf_update_gyro(gyro,gt);
        if (accel[0]!=0 || accel[1]!=0 || accel[2]!=0) vqf_update_accel(accel,at);
        vqf_get_quat(q+4*i);
    }
}
static double firmware_health[4];
static void check_state(void) {
#if !CONFIG_VQF_ADAPTIVE_TAU_ACC
    assert(params.tauAcc==3.264f);
#endif
    float q[4]; getQuat6D(&state,q);
    for(int j=0;j<4;j++) assert(vqf_float_finite(q[j]));
    float scale=1;
    for(int j=0;j<3;j++) {
        assert(vqf_float_finite(state.bias[j]));
        for(int k=0;k<3;k++) {
            assert(vqf_float_finite(state.biasP[3*j+k]));
            scale=fmaxf(scale,fabsf(state.biasP[3*j+k]));
        }
    }
    double p[9];
    for(int j=0;j<9;j++) p[j]=state.biasP[j]/scale;
    for(int j=0;j<3;j++) {
        assert(p[4*j]>=-1e-6);
        firmware_health[1]=fmin(firmware_health[1],p[4*j]);
        for(int k=j+1;k<3;k++) {
            assert(fabs(p[3*j+k]-p[3*k+j])<5e-5);
            double minor=p[4*j]*p[4*k]-p[3*j+k]*p[3*k+j];
            assert(minor>=-1e-5);
            firmware_health[2]=fmin(firmware_health[2],minor);
        }
    }
    double det=p[0]*(p[4]*p[8]-p[5]*p[7])-p[1]*(p[3]*p[8]-p[5]*p[6])+p[2]*(p[3]*p[7]-p[4]*p[6]);
    assert(det>=-1e-5);
    firmware_health[3]=fmin(firmware_health[3],det);
    firmware_health[0]++;
}
void firmware_strategy_a_health(double *out) {
    memcpy(out,firmware_health,sizeof(firmware_health));
}
static void firmware_replay(const float *g,const float *a,const float *t,int n,float gt,float at,float *q,float *observations) {
    vqf_init(gt,at,gt);
#ifdef VQF_HOST_THRESHOLD_OVERRIDE
    firmware_apply_threshold_override();
#endif
    firmware_health[0]=0;
    firmware_health[1]=firmware_health[2]=firmware_health[3]=1;
    for(int i=0;i<n;i++) {
        float gyro[3],accel[3];
        memcpy(gyro,g+3*i,sizeof(gyro));memcpy(accel,a+3*i,sizeof(accel));
        if(t) vqf_update_temperature(t[i]);
        vqf_update_gyro(gyro,gt);
        if(accel[0]!=0 || accel[1]!=0 || accel[2]!=0) vqf_update_accel(accel,at);
        vqf_get_quat(q+4*i);
        check_state();
        if(observations) {
            bool rest=false;
            observations[5*i]=getRestDetected(&state);
            observations[5*i+1]=vqf_get_rest_detected();
            observations[5*i+2]=vqf_take_rest_observation(&rest);
            observations[5*i+3]=rest;
            observations[5*i+4]=vqf_take_rest_observation(&rest);
        }
    }
}
void firmware_strategy_a_run(const float *g,const float *a,const float *t,int n,float gt,float at,float *q) {
    firmware_replay(g,a,t,n,gt,at,q,NULL);
}
void firmware_strategy_p6_rest_run(const float *g,const float *a,const float *t,int n,float gt,float at,float *q,float *observations) {
    firmware_replay(g,a,t,n,gt,at,q,observations);
}
static void scalar_filter_native(void) {
    /* A separate native gyro-LP instance is the oracle, not a duplicated
     * Butterworth formula. Exercise startup, steady recurrence and re-init. */
    for(int pass=0;pass<2;pass++) {
        float dt=pass ? .0025f:.01f,tau=.037f;
        vqf_params_t p;vqf_state_t s;vqf_coeffs_t c;
        init_params(&p);p.restFilterTau=tau;
        initVqf(&p,&s,&c,dt,dt,dt);
        struct {
            vqf_double_t before,filter_state[3],after;
        } guarded={.before=123,.after=456};
        vqf_double_t b[3],a[2];
        initScalarFilter(tau,dt,b,a,guarded.filter_state);
        double sum=0;
        for(int i=0;i<100;i++) {
            float input=(i%7)*.125f,g[3]={input,0,0};
            float output=updateScalarFilter(input,tau,dt,b,a,guarded.filter_state);
            updateGyr(&p,&s,&c,g);
            assert(output==s.restLastGyrLp[0]);
            sum+=input;
            if((i+1)*dt<tau) assert(output==(float)(sum/(i+1)));
            assert(guarded.before==123 && guarded.after==456);
        }
        initScalarFilter(tau,dt,b,a,guarded.filter_state);
        assert(updateScalarFilter(7,tau,dt,b,a,guarded.filter_state)==7);
        assert(updateScalarFilter(9,tau,dt,b,a,guarded.filter_state)==8);
        assert(guarded.before==123 && guarded.after==456);
    }
}
static void check_constant_history(const vqf_real_t *values,int n,
                                   const vqf_coeffs_t *c,const vqf_double_t *history) {
    for(int j=0;j<n;j++) {
        /* A constant unity-DC input must retain both delay elements, not
         * merely avoid a discontinuity in the first output sample. */
        assert(fabs(history[2*j]-values[j]*(1-c->accLpB[0]))<1e-11);
        assert(fabs(history[2*j+1]-values[j]*(c->accLpB[2]-c->accLpA[1]))<1e-11);
        vqf_double_t scalar[3]={history[2*j],history[2*j+1],0};
        for(int k=0;k<200;k++)
            assert(fabs(updateScalarFilter(values[j],1,c->accTs,c->accLpB,c->accLpA,scalar)-values[j])<2e-6);
    }
}
static void seed_constant_history(const vqf_real_t *values,int n,
                                  const vqf_coeffs_t *c,vqf_double_t *history) {
    for(int j=0;j<n;j++) {
        history[2*j]=values[j]*(1-c->accLpB[0]);
        history[2*j+1]=values[j]*(c->accLpB[2]-c->accLpA[1]);
    }
}
static void coefficient_change_constant(void) {
    const float taus[]={.5f,1,2,3.264f,4,8};
    for(int rate=0;rate<2;rate++) for(int old=0;old<6;old++) for(int next=0;next<6;next++) {
        if(old==next) continue;
        const float dt=rate ? .0025f:.01f;
        vqf_params_t p;vqf_state_t s;vqf_coeffs_t c;
        init_params(&p);p.tauAcc=taus[old];initVqf(&p,&s,&c,dt,dt,dt);
        const float acc[3]={.25f,-.75f,9.80665f};
        const float R[9]={.8f,-.6f,.1f,.6f,.8f,-.2f,.2f,-.1f,1};
        const float bias[2]={.003f,-.002f};
        memcpy(s.lastAccLp,acc,sizeof(acc));
        memcpy(s.motionBiasEstLastR,R,sizeof(R));
        memcpy(s.motionBiasEstLastBiasLp,bias,sizeof(bias));
        seed_constant_history(acc,3,&c,s.accLpState);
        seed_constant_history(R,9,&c,s.motionBiasEstRLpState);
        seed_constant_history(bias,2,&c,s.motionBiasEstBiasLpState);
        setTauAcc(&p,&s,&c,taus[next]);
        check_constant_history(acc,3,&c,s.accLpState);
        check_constant_history(R,9,&c,s.motionBiasEstRLpState);
        check_constant_history(bias,2,&c,s.motionBiasEstBiasLpState);
    }
}
static void rotation_for_state(vqf_state_t *s,float R[9]) {
    float q[4];getQuat6D(s,q);
    R[0]=1-2*(q[2]*q[2])-2*(q[3]*q[3]);
    R[1]=2*(q[2]*q[1]-q[0]*q[3]);
    R[2]=2*(q[0]*q[2]+q[3]*q[1]);
    R[3]=2*(q[0]*q[3]+q[2]*q[1]);
    R[4]=1-2*(q[1]*q[1])-2*(q[3]*q[3]);
    R[5]=2*(q[2]*q[3]-q[1]*q[0]);
    R[6]=2*(q[3]*q[1]-q[0]*q[2]);
    R[7]=2*(q[0]*q[1]+q[2]*q[3]);
    R[8]=1-2*(q[1]*q[1])-2*(q[2]*q[2]);
}
static void assert_motion_cache_cold(const vqf_state_t *s) {
    assert(isnan(s->motionBiasEstRLpState[0]) && isnan(s->motionBiasEstBiasLpState[0]));
    for(int j=0;j<9;j++) assert(s->motionBiasEstLastR[j]==0);
    for(int j=0;j<2;j++) assert(s->motionBiasEstLastBiasLp[j]==0);
}
static void motion_output_cache(void) {
    vqf_params_t p;vqf_state_t s;vqf_coeffs_t c;
    init_params(&p);p.tauAcc=.5f;p.restMinT=0;p.restThAcc=1000;
    initVqf(&p,&s,&c,.01f,.01f,.01f);
    vqf_double_t b[3],a[2],oracle[11][3];
    for(int j=0;j<11;j++) initScalarFilter(p.tauAcc,c.accTs,b,a,oracle[j]);
    const float acc[3]={.04f,-.02f,9.8f};
    s.bias[0]=.003f;s.bias[1]=-.002f;s.bias[2]=.001f;
    for(int i=0;i<300;i++) {
        /* Force a rotating attitude and a genuine RBE every sample. The
         * Kalman branch replaces its local R with identity, not the LP R. */
        float yaw=.3f+i*.002f;
        s.gyrQuat[0]=cosf(yaw/2);s.gyrQuat[1]=s.gyrQuat[2]=0;s.gyrQuat[3]=sinf(yaw/2);
        float previous_bias[3],R[9];
        memcpy(previous_bias,s.bias,sizeof(previous_bias));
        updateAcc(&p,&s,&c,acc);
        assert(s.restDetected);
        rotation_for_state(&s,R);
        float projected[2]={
            R[0]*previous_bias[0]+R[1]*previous_bias[1]+R[2]*previous_bias[2],
            R[3]*previous_bias[0]+R[4]*previous_bias[1]+R[5]*previous_bias[2]};
        for(int j=0;j<9;j++)
            assert(s.motionBiasEstLastR[j]==updateScalarFilter(R[j],p.tauAcc,c.accTs,b,a,oracle[j]));
        for(int j=0;j<2;j++)
            assert(s.motionBiasEstLastBiasLp[j]==updateScalarFilter(projected[j],p.tauAcc,c.accTs,b,a,oracle[9+j]));
    }
    assert(fabs(s.motionBiasEstLastR[1])>.1f);
    /* A second filter cannot overwrite the first instance's cached outputs. */
    vqf_state_t held=s,other;vqf_params_t other_p=p;vqf_coeffs_t other_c=c;
    initVqf(&other_p,&other,&other_c,.01f,.01f,.01f);
    updateAcc(&other_p,&other,&other_c,acc);
    assert(!memcmp(held.motionBiasEstLastR,s.motionBiasEstLastR,sizeof(s.motionBiasEstLastR)));
    assert(!memcmp(held.motionBiasEstLastBiasLp,s.motionBiasEstLastBiasLp,sizeof(s.motionBiasEstLastBiasLp)));
    resetState(&p,&s,&c);assert_motion_cache_cold(&s);
    setTauAcc(&p,&s,&c,1);assert_motion_cache_cold(&s); /* before any ACC */
    updateAcc(&p,&s,&c,acc);
    held=s;setTauAcc(&p,&s,&c,2); /* averaging markers/sums remain untouched */
    assert(!memcmp(held.motionBiasEstRLpState,s.motionBiasEstRLpState,sizeof(s.motionBiasEstRLpState)));
    assert(!memcmp(held.motionBiasEstBiasLpState,s.motionBiasEstBiasLpState,sizeof(s.motionBiasEstBiasLpState)));
    setMotionBiasEstEnabled(&p,&s,false);assert_motion_cache_cold(&s);
    updateAcc(&p,&s,&c,acc); /* rest-only runtime still advances both LPs */
    held=s;setMotionBiasEstEnabled(&p,&s,false);
    assert(!memcmp(&held,&s,sizeof(s))); /* idempotent setter does not reset */
    setMotionBiasEstEnabled(&p,&s,true);assert_motion_cache_cold(&s);
    setTauAcc(&p,&s,&c,.5f);assert_motion_cache_cold(&s);
}
static void retained_motion_cache(void) {
    struct {unsigned char before[16],data[VQF_MEM_SIZE],after[16];} saved;
    float acc[3]={.03f,-.04f,9.8f},gyro[3]={.01f,-.02f,.03f};
    for(int samples=0;samples<=300;samples=samples ? 300:1) {
        vqf_init(.01f,.01f,.01f);
        setTauAcc(&params,&state,&coeffs,.5f);
        for(int i=0;i<samples;i++) {
            updateGyr(&params,&state,&coeffs,gyro);
            updateAcc(&params,&state,&coeffs,acc);
        }
        vqf_state_t expected=state;vqf_params_t expected_p=params;vqf_coeffs_t expected_c=coeffs;
        setTauAcc(&expected_p,&expected,&expected_c,3.264f);
        memset(&saved,0xA5,sizeof(saved));vqf_save(saved.data);
        for(int j=0;j<16;j++) assert(saved.before[j]==0xA5 && saved.after[j]==0xA5);
        vqf_saved_header_t header;memcpy(&header,saved.data,sizeof(header));
        assert(header.magic==VQF_SAVED_MAGIC && header.version==VQF_SAVED_VERSION && header.size==VQF_MEM_SIZE);
        vqf_init(.01f,.01f,.01f);vqf_load(saved.data);
        assert(params.tauAcc==3.264f);
        assert(!memcmp(expected.motionBiasEstLastR,state.motionBiasEstLastR,sizeof(state.motionBiasEstLastR)));
        assert(!memcmp(expected.motionBiasEstLastBiasLp,state.motionBiasEstLastBiasLp,sizeof(state.motionBiasEstLastBiasLp)));
        assert(!memcmp(expected.accLpState,state.accLpState,sizeof(state.accLpState)));
        assert(!memcmp(expected.motionBiasEstRLpState,state.motionBiasEstRLpState,sizeof(state.motionBiasEstRLpState)));
        assert(!memcmp(expected.motionBiasEstBiasLpState,state.motionBiasEstBiasLpState,sizeof(state.motionBiasEstBiasLpState)));
        if(samples==300) break;
    }
    /* All envelope fields gate payload interpretation. Legacy identity-
     * quaternion raw blocks must be rejected without reading a new payload. */
    for(int field=0;field<3;field++) {
        vqf_saved_header_t header;memcpy(&header,saved.data,sizeof(header));
        if(field==0) header.magic^=1;
        if(field==1) header.version++;
        if(field==2) header.size--;
        unsigned char invalid[VQF_MEM_SIZE];memcpy(invalid,saved.data,sizeof(invalid));
        memcpy(invalid,&header,sizeof(header));vqf_load(invalid);
        assert_motion_cache_cold(&state);assert(state.bias[0]==0 && params.tauAcc==3.264f);
    }
    unsigned char legacy[784]={0};float identity=1;
    memcpy(legacy,&identity,sizeof(identity));vqf_load(legacy);
    assert_motion_cache_cold(&state);assert(state.bias[0]==0);
    /* Last outputs are finite state, unlike legitimate LP NaN markers. */
    vqf_state_t poisoned;memcpy(&poisoned,saved.data+sizeof(vqf_saved_header_t),sizeof(poisoned));
    poisoned.motionBiasEstLastR[2]=NAN;
    memcpy(saved.data+sizeof(vqf_saved_header_t),&poisoned,sizeof(poisoned));vqf_load(saved.data);
    assert_motion_cache_cold(&state);
}
#if CONFIG_SENSOR_VQF_ADAPTIVE_BIAS
static void assert_policy_cold(void) {
    assert(!motion_bias_policy.started && !motion_bias_policy.rest_update);
    assert(motion_bias_policy.since_rest==0 && motion_bias_policy.log_scale==0);
    assert(motion_bias_policy.rest_retention>0 && motion_bias_policy.rest_retention<1);
    assert(!predictive_bias_policy.temperature_available && !predictive_bias_policy.have);
    assert(!predictive_bias_policy.in_rest && !predictive_bias_policy.last_have);
    assert(!predictive_bias_policy.trial_have && predictive_bias_policy.trial_tests==0);
    assert(predictive_bias_policy.gain==0 && predictive_bias_policy.block_n==0 && predictive_bias_policy.n==0);
    for(int j=0;j<3;j++) assert(predictive_bias_policy.beta[j]==0 && predictive_bias_policy.offset[j]==0);
    assert(!training_temperature_policy.decided && !training_temperature_policy.matched);
    assert(vqf_double_nan(training_temperature_policy.filter_state[0]));
    assert(vqf_double_nan(training_temperature_policy.filter_state[1]));
#if CONFIG_SENSOR_VQF_REST_TRANSITION
    assert(rest_transition_policy.age==0 && rest_transition_policy.variance==0);
    assert(rest_transition_policy.hard_timer==0 && !rest_transition_policy.hard_rest);
    assert(!state.restDetected && state.restT==0);
    for(int j=0;j<3;j++) assert(rest_transition_policy.mean[j]==0);
#endif
}
static void strategy_recency(void) {
    /* Exercise nonzero motion history across accepted RBE and the next MBE.
     * An early return is essential: retained log_scale must not alter sigmaRest
     * or receive the additional motion IIR during an admitted rest update. */
    vqf_init(.0025f,.01f,.0025f);
    motion_bias_policy.since_rest=180.f;
    motion_bias_policy.log_scale=-.2f;
    motion_bias_policy.excitation=.75f;
    state.restDetected=true;
    float age=180.f,log=-.2f,rest_w[3]={coeffs.biasRestW,coeffs.biasRestW,coeffs.biasRestW};
    const float zero[9]={0};
    for(int i=0;i<3;i++) {
        float w[3];memcpy(w,rest_w,sizeof(w));
        age*=motion_bias_policy.rest_retention;log*=motion_bias_policy.rest_retention;
        vqf_bias_policy_weights(&params,&state,&coeffs,zero,zero,w);
        assert(motion_bias_policy.rest_update);
        assert(motion_bias_policy.since_rest==age && motion_bias_policy.log_scale==log);
        assert(!memcmp(w,rest_w,sizeof(w)));
    }
    state.restDetected=false;
    age+=coeffs.accTs;
    float old_log=log;
    float w[3]={0};
    vqf_bias_policy_weights(&params,&state,&coeffs,zero,zero,w);
    /* The next motion update must advance history, relax this negative scale
     * toward zero, and apply it only to motion weights. Do not duplicate the
     * tuned gain/age constants in the lifecycle test. */
    assert(motion_bias_policy.log_scale>old_log && motion_bias_policy.log_scale<0);
    float variance=params.biasSigmaMotion*expf(motion_bias_policy.log_scale)*100.f;variance*=variance;
    float weight=variance*variance/coeffs.biasV+variance;
    assert(!motion_bias_policy.rest_update);
    assert(motion_bias_policy.since_rest==age);
    assert(w[0]==weight && w[1]==weight);
    assert(w[2]==weight/fmaxf(params.biasVerticalForgettingFactor,1e-10f));
    /* Native rest with RBE disabled must still take the original motion path. */
    state.restDetected=true;params.restBiasEstEnabled=false;
    vqf_bias_policy_weights(&params,&state,&coeffs,zero,zero,w);
    assert(!motion_bias_policy.rest_update && motion_bias_policy.since_rest==age+coeffs.accTs);
    /* The optional benchmark's independent filter cannot mutate live policy. */
    typeof(motion_bias_policy) before=motion_bias_policy;
    vqf_state_t foreign=state;
    memcpy(w,rest_w,sizeof(w));
    vqf_bias_policy_weights(&params,&foreign,&coeffs,zero,zero,w);
    assert(!memcmp(&before,&motion_bias_policy,sizeof(before)) && !memcmp(w,rest_w,sizeof(w)));
    vqf_init(.0025f,.005f,.0025f);assert_policy_cold();
}
static void warm_policy(void) {
    /* Actual admitted-rest samples, not direct writes into predictor internals. */
    float g[3]={0,0,0},a[3]={0,0,1};
    for(int i=0;i<12000;i++) {
        float t=20.f+i*.0005f;
        g[2]=.003f*(t-20.f);
        vqf_update_temperature(t);
        vqf_update_gyro(g,.01f);
        vqf_update_accel(a,.01f);
    }
    assert(motion_bias_policy.started && predictive_bias_policy.have);
    assert(predictive_bias_policy.last_have && predictive_bias_policy.n>=3);
    assert(predictive_bias_policy.gain>0);
    check_state();
}
/* Isolate the thermal policy at its admitted LP-gyro/temperature seam. These
 * are observations, never writes into its fit, noise or validation statistics.
 * Exact quarter-second ticks and binary-valued samples make block boundaries
 * and zero-mean noise deterministic in both A-only and P6 configurations. */
static void thermal_rest(bool admitted) {
    motion_bias_policy.rest_update=admitted;
    state.restDetected=admitted;
#if CONFIG_SENSOR_VQF_REST_TRANSITION
    rest_transition_policy.hard_rest=admitted;
#endif
}
static void thermal_sample(float t,const float g[3]) {
    vqf_update_temperature(t);
    predictive_bias_policy.temperature=t;
    memcpy(state.restLastGyrLp,g,sizeof(state.restLastGyrLp));
    vqf_predictive_observe();
}
static void thermal_enter(void) {
    /* These controlled whitening fixtures do not execute a matched gyro/T
     * history. Deliberately choose the production fast-T fallback first. */
    vqf_update_gyro((float[3]){0,0,0},coeffs.gyrTs);
    assert(training_temperature_policy.decided && !training_temperature_policy.matched);
    thermal_rest(true);
    vqf_update_temperature(20.f);
    vqf_predictive_predict();
    for(int i=0;i<11;i++) thermal_sample(20.f,(float[3]){0,0,0});
}
static void thermal_begin(void) {
    vqf_init(.25f,.25f,.25f);
    thermal_enter();
}
static void thermal_leave(void) {
    thermal_rest(false);
    vqf_predictive_observe();
}
static const float thermal_slope=0x1p-13f,thermal_noise=0x1p-12f;
static void thermal_block(float t,const float mean[3],const float amplitude[3]) {
    for(int i=0;i<8;i++) {
        float g[3];
        for(int j=0;j<3;j++) g[j]=mean[j]+(i%2 ? amplitude[j]:-amplitude[j]);
        thermal_sample(t,g);
    }
}
static void thermal_train(const float amplitude[3]) {
    for(int k=0;k<4;k++) {
        float mean[3]={k*thermal_slope,k*thermal_slope,0};
        thermal_block(20.f+k,mean,amplitude);
    }
}
static void thermal_validate(const float amplitude[3]) {
    /* Quiet x follows the learned slope. y reverses: its unweighted loss
     * exceeds the x benefit, so only evidence that y is noisy can admit it. */
    for(int k=4;k<6;k++) {
        float mean[3]={k*thermal_slope,(4-k)*thermal_slope,0};
        thermal_block(20.f+k,mean,amplitude);
    }
}
struct thermal_result {float gain,applied[3];};
static struct thermal_result thermal_prediction(void) {
    struct thermal_result result={.gain=predictive_bias_policy.gain};
    float before[3],after[3],target=predictive_bias_policy.temperature+1;
    vqf_get_gyro_bias(before);
    thermal_leave();
    vqf_update_temperature(target);
    predictive_bias_policy.temperature=target;
    vqf_predictive_predict();
    vqf_get_gyro_bias(after);
    for(int j=0;j<3;j++) result.applied[j]=after[j]-before[j];
    return result;
}
static struct thermal_result thermal_decision(bool noisy_x,bool changed_validation) {
    thermal_begin();
    float training[3]={noisy_x ? thermal_noise:0,noisy_x ? 0:thermal_noise,0};
    thermal_train(training);
    assert(predictive_bias_policy.trial_have);
    /* The first heldout block cannot apply a two-block model. Its noise and
     * training update also must not change the trial frozen before it. */
    float amplitude[3]={changed_validation ? 8*thermal_noise:training[0],
                        changed_validation ? 0:training[1],0};
    float first[3]={4*thermal_slope,0,0};
    thermal_block(24.f,first,amplitude);
    assert(predictive_bias_policy.gain==0);
    thermal_block(25.f,(float[3]){5*thermal_slope,-thermal_slope,0},amplitude);
    return thermal_prediction();
}
static struct thermal_result thermal_frozen_model(void) {
    thermal_begin();
    thermal_train((float[3]){0,0,0});
    /* A same-temperature block is training, not validation. Its large shift
     * reverses the newly fitted slope, but cannot revise the already frozen
     * positive trial. The following negative increments must reject that
     * original trial; fitting to these heldout blocks first would admit it. */
    thermal_block(23.f,(float[3]){-8*thermal_slope,-8*thermal_slope,0},(float[3]){0,0,0});
    assert(predictive_bias_policy.gain==0);
    thermal_block(24.f,(float[3]){-9*thermal_slope,-9*thermal_slope,0},(float[3]){0,0,0});
    assert(predictive_bias_policy.gain==0);
    thermal_block(25.f,(float[3]){-10*thermal_slope,-10*thermal_slope,0},(float[3]){0,0,0});
    return thermal_prediction();
}
static struct thermal_result thermal_history(bool completed,int partial,int reset) {
    thermal_begin();
    if(completed)
        thermal_block(20.f,(float[3]){0,0,0},(float[3]){0,thermal_noise,0});
    for(int i=0;i<partial;i++)
        thermal_sample(20.f,(float[3]){i%2 ? 8*thermal_noise:-8*thermal_noise,0,0});
    thermal_leave();
    if(reset==1) {
        unsigned char saved[VQF_MEM_SIZE];vqf_save(saved);vqf_load(saved);
    } else if(reset==2) {
        vqf_set_gyro_bias((float[3]){0,0,0});
    } else if(reset==3) {
        vqf_init(.25f,.25f,.25f);
    }
    thermal_enter();
    thermal_train((float[3]){0,0,0});
    thermal_validate((float[3]){0,0,0});
    return thermal_prediction();
}
static struct thermal_result thermal_partial_rebase(bool rebase) {
    thermal_begin();
    const float amplitude[3]={0,thermal_noise,0},delta_dps[3]={-.025f,.0125f,-.00625f};
    float shift[3]={0,0,0};
    thermal_train(amplitude);
    for(int k=4;k<8;k++) {
        float mean[3]={k*thermal_slope,(k<6 ? 4-k:4+5*(k-6))*thermal_slope,0};
        for(int i=0;i<8;i++) {
            if(k==4 && i==4 && rebase) {
                assert(predictive_bias_policy.block_n>0 && predictive_bias_policy.block_age<2);
                vqf_rebase_gyro_bias(delta_dps);
                for(int j=0;j<3;j++) shift[j]=delta_dps[j]*DEG_TO_RAD;
            }
            float g[3];
            for(int j=0;j<3;j++)
                g[j]=mean[j]+(i%2 ? amplitude[j]:-amplitude[j])+shift[j];
            thermal_sample(20.f+k,g);
        }
    }
    /* The first completed heldout trial re-fits and freezes a second trial.
     * Only that second decision consumes the rebased partial block's variance,
     * so comparing just the first decision would miss a broken second moment. */
    return thermal_prediction();
}
static void strategy_thermal_evidence(void) {
    struct thermal_result quiet=thermal_decision(false,false);
    struct thermal_result reversed=thermal_decision(true,false);
    assert(quiet.gain>6 && reversed.gain==0);
    assert(quiet.applied[0]>0 && quiet.applied[1]>0);
    assert(quiet.applied[0]==quiet.applied[1]); /* one shared gain, not axis gains */
    for(int j=0;j<3;j++) assert(reversed.applied[j]==0);
    struct thermal_result changed=thermal_decision(false,true);
    assert(changed.gain==quiet.gain);
    for(int j=0;j<3;j++) assert(changed.applied[j]==quiet.applied[j]);
    struct thermal_result frozen=thermal_frozen_model();
    assert(frozen.gain==0);
    for(int j=0;j<3;j++) assert(frozen.applied[j]==0);
    struct thermal_result history=thermal_history(true,0,0);
    struct thermal_result discarded=thermal_history(true,7,0);
    struct thermal_result incomplete=thermal_history(false,7,0);
    assert(history.gain>6 && incomplete.gain==0);
    assert(discarded.gain==history.gain);
    for(int j=0;j<3;j++) {
        assert(discarded.applied[j]==history.applied[j]);
        assert(incomplete.applied[j]==0);
    }
    /* Whole-policy resets clear completed history; ordinary rest exit and
     * its new fit do not. Save/load must not retain these transient weights. */
    for(int reset=1;reset<=3;reset++) {
        struct thermal_result cold=thermal_history(true,7,reset);
        assert(cold.gain==0);
        for(int j=0;j<3;j++) assert(cold.applied[j]==0);
    }
    struct thermal_result original=thermal_partial_rebase(false);
    struct thermal_result translated=thermal_partial_rebase(true);
    assert(original.gain>0);
    assert(fabsf(translated.gain-original.gain)<2e-4f);
    for(int j=0;j<3;j++) assert(fabsf(translated.applied[j]-original.applied[j])<2e-6f);
}
static void training_temperature_trace(float dt) {
    vqf_init(dt,dt,dt);
    assert_policy_cold();
    vqf_params_t p=params;vqf_state_t reference;vqf_coeffs_t c;
    p.restFilterTau=params.restFilterTau*.45f;
    initVqf(&p,&reference,&c,dt,dt,dt);
    float g[3]={0,0,0},a[3]={0,0,1};
    double startup_sum=0;
    unsigned different=0,trained=0,anchored=0;
    for(int i=0;i<1600;i++) {
        vqf_update_temperature(20.f+i*.002f+(i>=35 ? .25f:0));
        if(i==17) {
            typeof(training_temperature_policy) held=training_temperature_policy;
            float fast=predictive_bias_policy.temperature;
            for(int k=0;k<100;k++) vqf_update_temperature(22.f+k*.001f);
            vqf_update_temperature(NAN);vqf_update_temperature(INFINITY);
            assert(predictive_bias_policy.temperature==fast);
            assert(!memcmp(&held,&training_temperature_policy,sizeof(held)));
        }
        vqf_update_gyro(g,dt);
        assert(training_temperature_policy.decided && training_temperature_policy.matched);
        float fast=predictive_bias_policy.temperature,input[3]={fast,0,0};
        updateGyr(&p,&reference,&c,input);
        float training=vqf_training_temperature();
        assert(training==reference.restLastGyrLp[0]);
        startup_sum+=fast;
        if(i<10) assert(training==(float)(startup_sum/(i+1)));
        if(training!=fast) different++;
        typeof(training_temperature_policy) held=training_temperature_policy;
        double block_t=predictive_bias_policy.block_t,block_n=predictive_bias_policy.block_n;
        vqf_update_accel(a,dt);
        assert(!memcmp(&held,&training_temperature_policy,sizeof(held)));
        if(motion_bias_policy.rest_update && vqf_hard_rest_detected()) {
            assert(predictive_bias_policy.anchor==fast);
            anchored++;
        }
        if(predictive_bias_policy.block_n==block_n+1) {
            assert(predictive_bias_policy.block_t==block_t+training);
            trained++;
        }
        if(i==23 || i==800) {
            /* Preserve running-mean and settled filter history across both
             * coordinate rebases; neither changes temperature coordinates. */
            vqf_rebase_gyro_bias((float[3]){.02f,-.01f,.03f});
            g[0]+=.02f;g[1]-=.01f;g[2]+=.03f;
            vqf_rebase_mag(48.f,.25f);
            assert(!memcmp(&held,&training_temperature_policy,sizeof(held)));
        }
        if(i==400) {
            unsigned char saved[VQF_MEM_SIZE];vqf_save(saved);
            assert(!memcmp(&held,&training_temperature_policy,sizeof(held)));
        }
    }
    assert(different>100 && trained>0 && anchored>0);
    check_state();
}
static void training_temperature_lifecycle(void) {
    float g[3]={0,0,0},a[3]={0,0,1},invalid[3]={NAN,0,0};
    /* Rejected gyro must not consume the only startup eligibility decision. */
    vqf_init(.01f,.01f,.01f);
    vqf_update_gyro(invalid,.01f);
    assert_policy_cold();
    vqf_update_temperature(20.f);vqf_update_gyro(g,.01f);
    assert(training_temperature_policy.matched && vqf_training_temperature()==20.f);
    /* A missing first observation permanently selects the ORIGINAL fast-T
     * training path, not a no-learning path or a later fabricated restart. */
    vqf_init(.01f,.01f,.01f);
    for(int i=0;i<100;i++) {
        vqf_update_temperature(NAN);
        vqf_update_gyro(g,.01f);vqf_update_accel(a,.01f);
    }
    assert(!predictive_bias_policy.have && predictive_bias_policy.gain==0);
    assert(training_temperature_policy.decided && !training_temperature_policy.matched);
    warm_policy();
    assert(!training_temperature_policy.matched);
    assert(vqf_training_temperature()==predictive_bias_policy.temperature);
    /* A valid retained block can be genuinely cold. Save exactly the retained
     * ABI, prove adjacent bytes survive, then restore after warming live state. */
    vqf_init(.01f,.01f,.01f);
    struct {
        unsigned char before[16],data[VQF_MEM_SIZE],after[16];
    } saved;
    memset(&saved,0xA5,sizeof(saved));vqf_save(saved.data);
    for(int i=0;i<16;i++) assert(saved.before[i]==0xA5 && saved.after[i]==0xA5);
    vqf_update_temperature(23.f);vqf_update_gyro(g,.01f);
    vqf_load(saved.data);assert_policy_cold();
    vqf_update_temperature(21.f);vqf_update_gyro(g,.01f);
    assert(training_temperature_policy.matched && vqf_training_temperature()==21.f);
    /* Both a partly averaged native LP and a settled LP are warm history.
     * A single NaN-marker check would wrongly match the former on restore. */
    for(int samples=1;samples<=512;samples*=512) {
        vqf_init(.01f,.01f,.01f);
        for(int i=0;i<samples;i++) {
            vqf_update_temperature(20.f);
            vqf_update_gyro(g,.01f);vqf_update_accel(a,.01f);
        }
        vqf_state_t native_before=state;
        vqf_save(saved.data);vqf_load(saved.data);assert_policy_cold();
        assert(!memcmp(native_before.restGyrLpState,state.restGyrLpState,sizeof(state.restGyrLpState)));
        assert(!memcmp(native_before.restLastGyrLp,state.restLastGyrLp,sizeof(state.restLastGyrLp)));
        vqf_update_temperature(20.f);vqf_update_gyro(g,.01f);
        assert(!training_temperature_policy.matched);
        warm_policy(); /* Warm restore still trains and applies a real model. */
        assert(vqf_training_temperature()==predictive_bias_policy.temperature);
    }
    /* Explicit bias replacement resets policy, NOT the native gyro LP. */
    vqf_init(.01f,.01f,.01f);
    vqf_update_temperature(20.f);vqf_update_gyro(g,.01f);
    vqf_set_gyro_bias(g);assert_policy_cold();
    vqf_update_temperature(20.f);vqf_update_gyro(g,.01f);
    assert(!training_temperature_policy.matched);
    warm_policy();
    assert(vqf_training_temperature()==predictive_bias_policy.temperature);
    /* Changed sampling periods must rebuild coefficients and startup state.
     * The independent native oracle also catches an extra step at either
     * temperature-only callbacks or accelerometer cadence. */
    training_temperature_trace(.01f);
    training_temperature_trace(.0025f);
}
static void strategy_lifecycle(void) {
    vqf_init(.01f,.01f,.01f);assert_policy_cold();warm_policy();
    motion_bias_policy.since_rest=180.f;motion_bias_policy.log_scale=-.2f;
    unsigned char saved[VQF_MEM_SIZE];vqf_save(saved);
    vqf_state_t before=state;
    vqf_init(.0025f,.005f,.0025f);assert_policy_cold();
    vqf_load(saved);assert_policy_cold();
    assert(!memcmp(before.bias,state.bias,sizeof(state.bias)));
    assert(!memcmp(before.biasP,state.biasP,sizeof(state.biasP)));
    warm_policy();
    float bias[3]={.01f,.02f,.03f},old_tau=params.tauAcc;
    before=state;vqf_set_gyro_bias(bias);assert_policy_cold();
    assert(params.tauAcc==old_tau && !memcmp(before.biasP,state.biasP,sizeof(state.biasP)));
    for(int j=0;j<3;j++) assert(fabsf(state.bias[j]-bias[j]*DEG_TO_RAD)<1e-9f);
    warm_policy();
    float beta[3],offset[3],last[3],gain=predictive_bias_policy.gain;
    double means[3],block[3],count=predictive_bias_policy.block_n;
    memcpy(beta,predictive_bias_policy.beta,sizeof(beta));memcpy(offset,predictive_bias_policy.offset,sizeof(offset));
    memcpy(last,predictive_bias_policy.last_g,sizeof(last));memcpy(means,predictive_bias_policy.mean_g,sizeof(means));
    memcpy(block,predictive_bias_policy.block_g,sizeof(block));before=state;
    motion_bias_policy.since_rest=120.f;motion_bias_policy.log_scale=-.1f;
    typeof(motion_bias_policy) motion_before=motion_bias_policy;
#if CONFIG_SENSOR_VQF_REST_TRANSITION
    assert(rest_transition_policy.age>1);
    typeof(rest_transition_policy) transition_before=rest_transition_policy;
#endif
    const float delta[3]={.02f,-.01f,.03f};
    vqf_rebase_gyro_bias(delta);
    assert(predictive_bias_policy.gain==gain);
    assert(!memcmp(beta,predictive_bias_policy.beta,sizeof(beta)));
    assert(!memcmp(offset,predictive_bias_policy.offset,sizeof(offset)));
    assert(!memcmp(before.biasP,state.biasP,sizeof(state.biasP)));
    assert(!memcmp(&motion_before,&motion_bias_policy,sizeof(motion_before)));
#if CONFIG_SENSOR_VQF_REST_TRANSITION
    assert(rest_transition_policy.age==transition_before.age);
    assert(rest_transition_policy.variance==transition_before.variance);
    assert(rest_transition_policy.hard_timer==transition_before.hard_timer);
    assert(rest_transition_policy.hard_rest==transition_before.hard_rest);
#endif
    for(int j=0;j<3;j++) {
        float d=delta[j]*DEG_TO_RAD;
        assert(fabsf(state.bias[j]-(before.bias[j]+d))<1e-9f);
        assert(fabsf(predictive_bias_policy.last_g[j]-(last[j]+d))<1e-9f);
        assert(fabs(predictive_bias_policy.mean_g[j]-(means[j]+d))<1e-12);
        assert(fabs(predictive_bias_policy.block_g[j]-(block[j]+count*d))<1e-12);
#if CONFIG_SENSOR_VQF_REST_TRANSITION
        assert(fabsf(rest_transition_policy.mean[j]-(transition_before.mean[j]+d))<1e-9f);
#endif
    }
    /* Magnetic reference rebasing must not reset inertial policy or consume
     * the sample-driven observation used by calibration and power. */
    motion_before=motion_bias_policy;
    typeof(predictive_bias_policy) predictive_before=predictive_bias_policy;
#if CONFIG_SENSOR_VQF_REST_TRANSITION
    transition_before=rest_transition_policy;
#endif
    bool pending_before=rest_observation_pending,rest_before=vqf_get_rest_detected();
    before=state;
    vqf_rebase_mag(48.f,.25f);
    assert(!memcmp(&motion_before,&motion_bias_policy,sizeof(motion_before)));
    assert(!memcmp(&predictive_before,&predictive_bias_policy,sizeof(predictive_before)));
#if CONFIG_SENSOR_VQF_REST_TRANSITION
    assert(!memcmp(&transition_before,&rest_transition_policy,sizeof(transition_before)));
#endif
    assert(!memcmp(before.bias,state.bias,sizeof(state.bias)));
    assert(!memcmp(before.biasP,state.biasP,sizeof(state.biasP)));
    assert(!memcmp(before.gyrQuat,state.gyrQuat,sizeof(state.gyrQuat)));
    assert(!memcmp(before.accQuat,state.accQuat,sizeof(state.accQuat)));
    assert(rest_observation_pending==pending_before && vqf_get_rest_detected()==rest_before);
    vqf_init(.01f,.01f,.01f);assert_policy_cold();
    vqf_update_temperature(NAN);vqf_update_temperature(INFINITY);
    assert(!predictive_bias_policy.temperature_available);
    vqf_update_temperature(0.f);vqf_update_temperature(NAN);
    assert(predictive_bias_policy.temperature_available && predictive_bias_policy.hold==0);
}
#endif
#if CONFIG_SENSOR_VQF_REST_TRANSITION
static void transition_original_failure(void) {
    const float dt=.0025f, rad=DEG_TO_RAD;
    const float zero[3]={0,0,0};
    vqf_init(dt,.01f,.01f);
    float threshold=params.restThGyr*rad;

    /* Deliberately disagree with the stored residual in BOTH directions.
     * The supplied core predicate, not a reconstruction in the hook, owns
     * the failure decision and the independent hard-rest timer. */
    state.restLastSquaredDeviations[0]=4*threshold*threshold;
    rest_transition_policy.hard_rest=true;
    rest_transition_policy.hard_timer=.75f;
    assert(!vqf_rest_transition_failure(&params,&state,zero,dt,false));
    assert(rest_transition_policy.hard_rest && rest_transition_policy.hard_timer==.75f);
    state.restLastSquaredDeviations[0]=0;
    rest_transition_policy.age=0;
    assert(vqf_rest_transition_failure(&params,&state,zero,dt,true));
    assert(!rest_transition_policy.hard_rest && rest_transition_policy.hard_timer==0);

    /* A foreign filter gets the original predicate without touching the live
     * policy, even if its residual and biasClip would suggest otherwise. */
    vqf_state_t foreign=state, live_before=state;
    typeof(rest_transition_policy) policy_before=rest_transition_policy;
    foreign.restLastGyrLp[0]=2*params.biasClip*rad;
    foreign.restLastSquaredDeviations[0]=4*threshold*threshold;
    assert(!vqf_rest_transition_failure(&params,&foreign,zero,dt,false));
    foreign.restLastSquaredDeviations[0]=0;
    assert(vqf_rest_transition_failure(&params,&foreign,zero,dt,true));
    assert(!memcmp(&live_before,&state,sizeof(state)));
    assert(!memcmp(&policy_before,&rest_transition_policy,sizeof(policy_before)));

    /* A qualified bounded slow-LP transition relaxes only fusion rest. Hard
     * rest still responds to the original failure, before EW moments advance. */
    float g[3]={(params.restThGyr+.1f)*rad,0,0};
    memset(state.restLastGyrLp,0,sizeof(state.restLastGyrLp));
    state.restLastSquaredDeviations[0]=g[0]*g[0];
    rest_transition_policy.age=1;
    rest_transition_policy.variance=0;
    memcpy(rest_transition_policy.mean,g,sizeof(g));
    rest_transition_policy.hard_rest=true;
    rest_transition_policy.hard_timer=.75f;
    assert(!vqf_rest_transition_failure(&params,&state,g,dt,true));
    assert(!rest_transition_policy.hard_rest && rest_transition_policy.hard_timer==0);
    assert(rest_transition_policy.age>1);

    /* Above-cap residuals and nonquiet moments remain failures. */
    float above=(params.restThGyr+.25f)*rad;
    state.restLastSquaredDeviations[0]=above*above;
    rest_transition_policy.age=1;
    rest_transition_policy.variance=0;
    memcpy(rest_transition_policy.mean,g,sizeof(g));
    assert(vqf_rest_transition_failure(&params,&state,g,dt,true));
    state.restLastSquaredDeviations[0]=g[0]*g[0];
    rest_transition_policy.age=1;
    rest_transition_policy.variance=(.09f*rad)*(.09f*rad);
    memcpy(rest_transition_policy.mean,g,sizeof(g));
    assert(vqf_rest_transition_failure(&params,&state,g,dt,true));
}

static void transition_core_bias_clip(void) {
    const float dt=.0025f, rad=DEG_TO_RAD;
    for(int clipped=0;clipped<2;clipped++) {
        vqf_init(dt,.01f,.01f);
        float base=clipped ? (params.biasClip+2.f)*rad : 0;
        float g[3]={base,0,0};
        for(int i=0;i<1600;i++) updateGyr(&params,&state,&coeffs,g);
        assert(fabsf(state.restLastGyrLp[0]-base)<1e-6f);

        if(!clipped) {
            /* A genuinely passing core predicate must also reach the hook as
             * false, without clearing either independent rest state. */
            state.restDetected=true;
            state.restT=.75f;
            rest_transition_policy.hard_rest=true;
            rest_transition_policy.hard_timer=.75f;
            updateGyr(&params,&state,&coeffs,g);
            float threshold=params.restThGyr*rad;
            assert(state.restLastSquaredDeviations[0]<threshold*threshold);
            assert(state.restDetected && state.restT==.75f);
            assert(rest_transition_policy.hard_rest && rest_transition_policy.hard_timer==.75f);
        }
        g[0]=state.restLastGyrLp[0]+(params.restThGyr+.1f)*rad;
        state.restDetected=true;
        state.restT=.75f;
        rest_transition_policy.age=1;
        rest_transition_policy.variance=0;
        memcpy(rest_transition_policy.mean,g,sizeof(g));
        rest_transition_policy.hard_rest=true;
        rest_transition_policy.hard_timer=.75f;
        typeof(rest_transition_policy) policy_before=rest_transition_policy;
        updateGyr(&params,&state,&coeffs,g);
        float threshold=params.restThGyr*rad, cap=(params.restThGyr+.2f)*rad;
        assert(state.restLastSquaredDeviations[0]>=threshold*threshold);
        assert(state.restLastSquaredDeviations[0]<cap*cap);
        assert((fabsf(state.restLastGyrLp[0])>params.biasClip*rad)==clipped);
        assert(!rest_transition_policy.hard_rest && rest_transition_policy.hard_timer==0);
        if(clipped) {
            assert(!state.restDetected && state.restT==0);
            /* Replay the actual hook on the just-filtered sample with its
             * original moments: relaxation succeeded, yet the core vetoed it. */
            rest_transition_policy=policy_before;
            assert(!vqf_rest_transition_failure(&params,&state,g,dt,true));
        } else {
            assert(state.restDetected && state.restT==.75f);
        }
    }
}

static void transition_consumers(void) {
    const float dt=.0025f;
    float g[3]={0,0,0},a[3]={0,0,1},invalid[3]={NAN,0,0};
    bool observed=true;
    unsigned relaxed_only=0;
    vqf_init(dt,.01f,.01f);
    assert(!vqf_take_rest_observation(&observed));
    for(int i=0;i<36000;i++) {
        g[2]=i>=12000 && i<24000 ? 30.f : 0.f;
        vqf_update_gyro(g,dt);
        if(i%4==0) vqf_update_accel(a,.01f);
        assert(vqf_take_rest_observation(&observed));
        assert(observed==vqf_get_rest_detected());
        assert(!vqf_take_rest_observation(&observed));
        if(state.restDetected && !vqf_get_rest_detected()) {
            assert(!observed);
            relaxed_only++;
        }
    }
    assert(relaxed_only>0);
    vqf_update_gyro(invalid,dt);
    vqf_update_accel(invalid,.01f);
    assert(!vqf_take_rest_observation(&observed));
    unsigned char saved[VQF_MEM_SIZE];vqf_save(saved);
    vqf_update_gyro(g,dt);
    vqf_load(saved);
    assert(!vqf_take_rest_observation(&observed) && !vqf_get_rest_detected());
    float zero_bias[3]={0,0,0};
    vqf_set_gyro_bias(zero_bias);
    assert_policy_cold();
    vqf_rebase_gyro_bias((float[3]){1,2,3});
    for(int j=0;j<3;j++) assert(rest_transition_policy.mean[j]==0);
}
#endif
int main(void) {
    scalar_filter_native();
    coefficient_change_constant();
    motion_output_cache();
    retained_motion_cache();
    float g[3]={0,0,80}, a[3]={0,0,1};
    vqf_init(.0025f,.01f,.01f);
    assert(params.restThGyr==.4024f && params.restThAcc==.107f);
    for (int i=0;i<12000;i++) {
        vqf_update_gyro(g,.0025f);
        if(i%4==0) vqf_update_accel(a,.01f);
        assert(params.tauAcc>=.5f && params.tauAcc<=8.f);
    }
#if CONFIG_VQF_ADAPTIVE_TAU_ACC
    assert(params.tauAcc<1.5f); /* rotation lowers the quiet target */
#else
    assert(params.tauAcc==3.264f);
#endif
    vqf_state_t before=state;
    float old_tau=params.tauAcc;
    float invalid[3]={NAN,0,0}, zero[3]={0,0,0};
    for(int i=0;i<100;i++) {
        vqf_update_gyro(invalid,.01f);
        vqf_update_accel(invalid,.01f);
        vqf_update_accel(zero,.01f);
    }
    assert(!memcmp(&state,&before,sizeof(state)) && params.tauAcc==old_tau);
    unsigned char saved[VQF_MEM_SIZE];
    vqf_save(saved);
    float q_before[4],q_after[4];getQuat6D(&state,q_before);
    vqf_load(saved);
    getQuat6D(&state,q_after);
    assert(!memcmp(before.gyrQuat,state.gyrQuat,sizeof(state.gyrQuat)));
    assert(!memcmp(before.accQuat,state.accQuat,sizeof(state.accQuat)));
    assert(!memcmp(q_before,q_after,sizeof(q_before)));
    assert(!memcmp(before.bias,state.bias,sizeof(state.bias)));
    assert(!memcmp(before.biasP,state.biasP,sizeof(state.biasP)));
    vqf_state_t fresh; vqf_coeffs_t expected;
    initVqf(&params,&fresh,&expected,coeffs.gyrTs,coeffs.accTs,coeffs.magTs);
    assert(!memcmp(expected.accLpB,coeffs.accLpB,sizeof(expected.accLpB)));
    assert(!memcmp(expected.accLpA,coeffs.accLpA,sizeof(expected.accLpA)));
    assert(params.tauAcc==3.264f);
    vqf_update_accel(a,.01f);
    assert(params.tauAcc==3.264f); /* restarted controller interval */
    before.bias[0]=NAN;
    memcpy(saved+sizeof(vqf_saved_header_t),&before,sizeof(before));
    vqf_load(saved);
    assert(state.bias[0]==0 && params.tauAcc==3.264f);
    for(int rate=100;rate<=400;rate*=2) {
        float dt=1.f/rate;
        vqf_init(dt,dt,dt);
        for(int i=0;i<40*rate;i++) {
            vqf_update_gyro(zero,dt); vqf_update_accel(a,dt);
        }
#if CONFIG_VQF_ADAPTIVE_TAU_ACC
        assert(fabsf(params.tauAcc-2.f)<.0002f);
#else
        assert(params.tauAcc==3.264f);
#endif
    }
#if CONFIG_SENSOR_VQF_ADAPTIVE_BIAS
    strategy_lifecycle();
    strategy_recency();
    training_temperature_lifecycle();
    strategy_thermal_evidence();
#endif
#if CONFIG_SENSOR_VQF_REST_TRANSITION
    transition_original_failure();
    transition_core_bias_clip();
    transition_consumers();
#endif
#ifdef VQF_HOST_THRESHOLD_OVERRIDE
    float exported_params[25];
    firmware_strategy_a_override_thresholds(.401f,.106f);
    firmware_strategy_a_params(exported_params);
    assert(params.restThGyr==.401f && params.restThAcc==.106f);
    firmware_strategy_a_clear_threshold_override();
    firmware_strategy_a_params(exported_params);
    assert(params.restThGyr==.4024f && params.restThAcc==.107f);
#endif
    puts("Production VQF tau/A/P6, exact LP coefficient adaptation/cache lifecycle, thermal whitening/training-temperature cadence/lifecycle/rebase, finite PSD, versioned retained/reset/setter and hard-rest consumers passed");
#if CONFIG_SENSOR_VQF_ADAPTIVE_BIAS
    printf("Training-temperature transient state: %zu bytes; native retained state: %zu + %zu bytes\n",
           sizeof(training_temperature_policy),sizeof(state),sizeof(coeffs));
#endif
    return 0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--export', type=Path, help='Also build shared wrappers for offline parity replay')
    parser.add_argument('--strategy-a-export', type=Path, help='Build A disabled/enabled exports with adaptive tau enabled')
    parser.add_argument('--strategy-p6-export', type=Path, help='Build P6, A-only, and default-off exports with adaptive tau enabled')
    parser.add_argument('--threshold-overrides', action='store_true', help='Compile test-only runtime threshold override; absent from default exports')
    parser.add_argument('--fixed-tau-ablation', action='store_true', help='Compile experimental fixed-tau A off/on pair only; not a supported production configuration')
    parser.add_argument('--output-observer', type=int, choices=(0, 1),
                        help='Public fixed-tau output: default on, except historical fixed-tau ablation defaults off')
    args = parser.parse_args()
    output_enabled = int(not args.fixed_tau_ablation) if args.output_observer is None else args.output_observer
    if args.strategy_p6_export and args.fixed_tau_ablation:
        parser.error('P6 export requires the production adaptive tau configuration')
    for destination in (args.export, args.strategy_a_export, args.strategy_p6_export):
        if destination and destination.exists() and any(destination.iterdir()):
            parser.error('Refusing to overwrite nonempty export directory: '+str(destination))
    paths = [ROOT/'src/sensor/fusion/vqf/vqf.c', ROOT/'vqf-c/src/vqf.c',
             ROOT/'vqf-c/src/vqf.h', Path(__file__).resolve()]
    source_hashes = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}
    builds = []
    with tempfile.TemporaryDirectory(prefix='vqf-adaptive-') as tmp:
        directory = Path(tmp)
        source = directory / 'wrapper.c'
        source.write_text(translation_unit())
        configurations = [(0, 0, 0), (0, 1, 0)] if args.fixed_tau_ablation else [
            (0, 0, 0), (1, 0, 0), (1, 1, 0), (1, 1, 1)]
        for tau_enabled, bias_enabled, transition_enabled in configurations:
            # Match GNU math constants of the frozen native reference.
            flags = ['-DCONFIG_VQF_ADAPTIVE_TAU_ACC='+str(tau_enabled),
                     '-DCONFIG_SENSOR_VQF_ADAPTIVE_BIAS='+str(bias_enabled),
                     '-DCONFIG_SENSOR_VQF_REST_TRANSITION='+str(transition_enabled),
                     '-DCONFIG_CONNECTION_RAW_COLLECTION=1',
                     '-DCONFIG_SENSOR_USE_MAGNETOMETER=1']
            flags.append('-DCONFIG_SENSOR_VQF_OUTPUT_OBSERVER='+str(output_enabled))
            if output_enabled:
                flags.extend(['-DVQF_OUTPUT_OBSERVER', '-DVQF_OUTPUT_TAU_ACC=3.264f'])
            if bias_enabled:
                flags.append('-DVQF_ADAPTIVE_BIAS')
            if transition_enabled:
                flags.append('-DVQF_REST_TRANSITION')
            if args.threshold_overrides:
                flags.append('-DVQF_HOST_THRESHOLD_OVERRIDE')
            common = ['rtk', 'proxy', 'cc', '-std=gnu11', '-O2', *flags,
                      '-I'+str(ROOT/'vqf-c/src'), str(source), str(ROOT/'vqf-c/src/vqf.c'), '-lm']
            binary = directory / f'check-{tau_enabled}-{bias_enabled}-{transition_enabled}'
            subprocess.run(common + ['-fsanitize=undefined', '-fno-sanitize-recover=all', '-o', str(binary)], check=True)
            subprocess.run(['rtk', 'proxy', str(binary)], check=True)
            destinations = []
            if args.export and not bias_enabled and not transition_enabled:
                destinations.append(args.export / f'firmware-{tau_enabled}.so')
            if args.strategy_a_export and not transition_enabled:
                if args.fixed_tau_ablation:
                    destinations.append(args.strategy_a_export / f'strategy-a-fixed-{bias_enabled}.so')
                elif tau_enabled:
                    destinations.append(args.strategy_a_export / f'strategy-a-{bias_enabled}.so')
            if args.strategy_p6_export and tau_enabled:
                profile = 'p6' if transition_enabled else 'a-only' if bias_enabled else 'default-off'
                destinations.append(args.strategy_p6_export / f'{profile}.so')
            for destination in destinations:
                destination.parent.mkdir(parents=True, exist_ok=True)
                subprocess.run(common + ['-shared', '-fPIC', '-o', str(destination)], check=True)
                builds.append({'tau_enabled':tau_enabled, 'bias_enabled':bias_enabled,
                               'transition_enabled':transition_enabled, 'command':common,
                               'effective_features':{
                                   'output_observer':bool(output_enabled),
                                   'output_capture':'public vqf_get_quat; runtime MAG unavailable/disabled',
                                   'mag_compiled':True, 'mag_runtime_active':False,
                                   'fast_rest_recovery':False, 'cycle_bias_observation':False,
                                   'retained_bytes':904 if output_enabled else 840,
                                   'retained_version':2 if output_enabled else 1,
                               },
                               'path':str(destination.resolve()),
                               'library_sha256':hashlib.sha256(destination.read_bytes()).hexdigest()})
        if any(hashlib.sha256(path.read_bytes()).hexdigest()!=source_hashes[str(path)] for path in paths):
            raise RuntimeError('Production/exporter source changed during compilation; exports are stale')
        for destination in {p for p in (args.export, args.strategy_a_export, args.strategy_p6_export) if p}:
            receipt = destination/'build.json'
            with receipt.open('x') as stream:
                json.dump({'source_hashes':source_hashes, 'builds':builds,
                           'sanitizer':'undefined, fno-sanitize-recover=all',
                           'output_observer':bool(output_enabled),
                           'fixed_tau_ablation':args.fixed_tau_ablation,
                           'retained_bytes':904 if output_enabled else 840}, stream, indent=2)
                stream.write('\n')


if __name__ == '__main__':
    main()
