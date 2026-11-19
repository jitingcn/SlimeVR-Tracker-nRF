#!/usr/bin/env python3
"""Exercise the production VQF wrapper and native filter lifecycle on host."""
import importlib.util
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

ROOT = Path(os.environ.get('SOURCE_ROOT', Path(__file__).resolve().parents[3]))


def translation_unit():
    source = (ROOT / 'src/sensor/fusion/vqf/vqf.c').read_text()
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
    return r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
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
struct retained_data { unsigned char fusion_data[904]; };
static bool host_mag_available, host_mag_enabled;
bool sensor_get_mag_available(void) { return host_mag_available; }
bool sensor_get_mag_enabled(void) { return host_mag_enabled; }
void vqf_get_quat(float *q);
''' + source + r'''
_Static_assert(VQF_MEM_SIZE == 840, "Persisted VQF payload changed");
_Static_assert(sizeof(((struct retained_data *)0)->fusion_data) == 904,
               "Retained VQF storage capacity changed");
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
    for(int samples=0;samples<=500;samples=samples ? 500:1) {
        vqf_init(.01f,.01f,.01f);
        for(int i=0;i<samples;i++) {
            updateGyr(&params,&state,&coeffs,gyro);
            updateAcc(&params,&state,&coeffs,acc);
        }
        vqf_state_t expected=state;vqf_coeffs_t expected_c=coeffs;
        memset(&saved,0xA5,sizeof(saved));vqf_save(saved.data);
        for(int j=0;j<16;j++) assert(saved.before[j]==0xA5 && saved.after[j]==0xA5);
        vqf_saved_header_t header;memcpy(&header,saved.data,sizeof(header));
        assert(header.magic==VQF_SAVED_MAGIC && header.version==VQF_SAVED_VERSION && header.size==VQF_MEM_SIZE);
        vqf_init(.01f,.01f,.01f);vqf_load(saved.data,.01f,.01f,.01f);
        assert(params.tauAcc==3.294f);
        assert(!memcmp(&expected_c,&coeffs,sizeof(coeffs)));
        assert(!memcmp(expected.motionBiasEstLastR,state.motionBiasEstLastR,sizeof(state.motionBiasEstLastR)));
        assert(!memcmp(expected.motionBiasEstLastBiasLp,state.motionBiasEstLastBiasLp,sizeof(state.motionBiasEstLastBiasLp)));
        assert(!memcmp(expected.accLpState,state.accLpState,sizeof(state.accLpState)));
        assert(!memcmp(expected.motionBiasEstRLpState,state.motionBiasEstRLpState,sizeof(state.motionBiasEstRLpState)));
        assert(!memcmp(expected.motionBiasEstBiasLpState,state.motionBiasEstBiasLpState,sizeof(state.motionBiasEstBiasLpState)));
        if(samples==500) break;
    }
    /* All envelope fields gate payload interpretation. Legacy identity-
     * quaternion raw blocks must be rejected without reading a new payload. */
    for(int field=0;field<3;field++) {
        vqf_saved_header_t header;memcpy(&header,saved.data,sizeof(header));
        if(field==0) header.magic^=1;
        if(field==1) header.version++;
        if(field==2) header.size--;
        unsigned char invalid[VQF_MEM_SIZE];memcpy(invalid,saved.data,sizeof(invalid));
        memcpy(invalid,&header,sizeof(header));vqf_load(invalid,.01f,.01f,.01f);
        assert_motion_cache_cold(&state);assert(state.bias[0]==0 && params.tauAcc==3.294f);
    }
    unsigned char legacy[784]={0};float identity=1;
    memcpy(legacy,&identity,sizeof(identity));vqf_load(legacy,.01f,.01f,.01f);
    assert_motion_cache_cold(&state);assert(state.bias[0]==0);
    /* Last outputs are finite state, unlike legitimate LP NaN markers. */
    vqf_state_t poisoned;memcpy(&poisoned,saved.data+sizeof(vqf_saved_header_t),sizeof(poisoned));
    poisoned.motionBiasEstLastR[2]=NAN;
    memcpy(saved.data+sizeof(vqf_saved_header_t),&poisoned,sizeof(poisoned));vqf_load(saved.data,.01f,.01f,.01f);
    assert_motion_cache_cold(&state);
}
static void fixed_parameters(void) {
    vqf_init(.0025f,.01f,.02f);
    assert(params.tauAcc==3.294f && params.biasClip==5.0f);
    assert(params.biasForgettingTime==235.58f && params.biasSigmaInit==.6585f);
    assert(params.biasSigmaMotion==.1118f && params.biasSigmaRest==.0028f);
    assert(params.biasVerticalForgettingFactor==.0003f);
    assert(params.motionBiasEstEnabled && params.restBiasEstEnabled);
    assert(params.restFilterTau==2.278f && params.restMinT==.118f);
    assert(params.restThGyr==.486f && params.restThAcc==.186f);
    assert(params.magDistRejectionEnabled && params.tauMag==9.f);
    assert(params.magCurrentTau==.20f && params.magNormTh==.10f && params.magDipTh==5.f);
    assert(params.magRefTau==10.f && params.magNewTime==15.f && params.magNewFirstTime==5.f);
    assert(params.magNewMinGyr==10.f && params.magMinUndisturbedTime==.5f);
    assert(params.magMaxRejectionTime==60.f && params.magRejectionFactor==1000.f);
}
static void native_inclination(void) {
    const float cases[][3]={
        {0,0,-1},{1e-4f,0,-1},{-1e-4f,0,-1},{0,1e-4f,-1},{0,-1e-4f,-1},
        {7e-5f,-7e-5f,-1},{-6e-5f,8e-5f,-1},{5e-7f,0,-1},{4e-4f,2e-4f,-1},
        {0,0,1},{1,0,0},{.6f,-.2f,.77f},{-.3f,.4f,-.866f}
    };
    for (unsigned i=0;i<sizeof(cases)/sizeof(*cases);i++) for (int yaw=0;yaw<2;yaw++) {
        vqf_params_t p;vqf_state_t s;vqf_coeffs_t c;
        init_params(&p);
        p.motionBiasEstEnabled=p.restBiasEstEnabled=p.magDistRejectionEnabled=false;
        initVqf(&p,&s,&c,.005f,.02f,.02f);
        if (yaw) { s.accQuat[0]=cosf(.31f);s.accQuat[3]=sinf(.31f); }
        for (int sample=0;sample<180;sample++) {
            updateAcc(&p,&s,&c,cases[i]);
            const float *q=s.accQuat, *a=cases[i];
            double norm=sqrt((double)a[0]*a[0]+(double)a[1]*a[1]+(double)a[2]*a[2]);
            double x=a[0]/norm,y=a[1]/norm,z=a[2]/norm;
            double tx=2*(q[2]*z-q[3]*y),ty=2*(q[3]*x-q[1]*z),tz=2*(q[1]*y-q[2]*x);
            double ex=x+q[0]*tx+q[2]*tz-q[3]*ty;
            double ey=y+q[0]*ty+q[3]*tx-q[1]*tz;
            double ez=z+q[0]*tz+q[1]*ty-q[2]*tx;
            double qnorm=0;
            for (int j=0;j<4;j++) { assert(vqf_float_finite(q[j]));qnorm+=(double)q[j]*q[j]; }
            assert(fabs(qnorm-1)<2e-6 && hypot(ex,ey)<6e-4 && fabs(ez-1)<2e-6);
        }
    }
}
static void public_quat_and_linear_accel(void) {
    float g[3]={15,-7,10},a[3]={.2f,-.1f,1.f};
    vqf_init(.0025f,.01f,.02f);
    for (int i=0;i<3000;i++) {
        vqf_update_gyro(g,.0025f);
        if (i%4==0) vqf_update_accel(a,.01f);
    }
    /* A restored nonzero delta must not leak into the disabled/unavailable 6D output. */
    state.delta=.7f;
    for (int available=0;available<2;available++) for (int enabled=0;enabled<2;enabled++) {
        host_mag_available=available;host_mag_enabled=enabled;
        float q[4],expected[4],linear[3];
        vqf_get_quat(q);
#if CONFIG_SENSOR_USE_MAGNETOMETER
        if (available && enabled) getQuat9D(&state,expected); else getQuat6D(&state,expected);
#else
        getQuat6D(&state,expected);
#endif
        assert(!memcmp(q,expected,sizeof(q)));
        vqf_get_lin_a(linear);
        float gravity[3]={2*(q[1]*q[3]-q[0]*q[2]),2*(q[2]*q[3]+q[0]*q[1]),
                          2*(q[0]*q[0]-.5f+q[3]*q[3])};
        for (int j=0;j<3;j++)
            assert(fabsf(linear[j]-(a[j]-gravity[j])*CONST_EARTH_GRAVITY)<3e-5f);
    }
}
static void check_covariance(void) {
    double scale=1, p[9];
    for (int j=0;j<3;j++) {
        assert(vqf_float_finite(state.bias[j]));
        for (int k=0;k<3;k++) {
            assert(vqf_float_finite(state.biasP[3*j+k]));
            scale=fmax(scale,fabs(state.biasP[3*j+k]));
        }
    }
    for (int j=0;j<9;j++) p[j]=state.biasP[j]/scale;
    for (int j=0;j<3;j++) {
        assert(p[4*j]>=-1e-6);
        for (int k=j+1;k<3;k++) {
            assert(fabs(p[3*j+k]-p[3*k+j])<5e-5);
            assert(p[4*j]*p[4*k]-p[3*j+k]*p[3*k+j]>=-1e-5);
        }
    }
    double det=p[0]*(p[4]*p[8]-p[5]*p[7])-p[1]*(p[3]*p[8]-p[5]*p[6])
        +p[2]*(p[3]*p[7]-p[4]*p[6]);
    assert(det>=-1e-5);
}
int main(void) {
    scalar_filter_native();
    coefficient_change_constant();
    motion_output_cache();
    retained_motion_cache();
    fixed_parameters();
    native_inclination();
    public_quat_and_linear_accel();
    float g[3]={0,0,80},a[3]={0,0,1},invalid[3]={NAN,0,0},zero[3]={0,0,0};
    vqf_init(.0025f,.01f,.01f);
    for (int i=0;i<12000;i++) {
        vqf_update_gyro(g,.0025f);
        if (i%4==0) vqf_update_accel(a,.01f);
        assert(params.tauAcc==3.294f);
        if (i%100==0) check_covariance();
    }
    vqf_state_t before=state;
    for (int i=0;i<100;i++) {
        vqf_update_gyro(invalid,.01f);
        vqf_update_accel(invalid,.01f);
        vqf_update_accel(zero,.01f);
    }
    assert(!memcmp(&state,&before,sizeof(state)));
    unsigned char saved[VQF_MEM_SIZE];vqf_save(saved);
    vqf_load(saved,.0025f,.01f,.01f);
    assert(!memcmp(&before,&state,sizeof(state)) && params.tauAcc==3.294f);
    float replacement[3]={.01f,.02f,.03f};
    vqf_set_gyro_bias(replacement);
    assert(!memcmp(before.biasP,state.biasP,sizeof(state.biasP)));
    assert(!memcmp(before.gyrQuat,state.gyrQuat,sizeof(state.gyrQuat)));
    assert(!memcmp(before.accQuat,state.accQuat,sizeof(state.accQuat)));
    for (int j=0;j<3;j++) assert(fabsf(state.bias[j]-replacement[j]*DEG_TO_RAD)<1e-9f);
    vqf_load(saved,.0025f,.01f,.01f);
    before.bias[0]=NAN;
    memcpy(saved+sizeof(vqf_saved_header_t),&before,sizeof(before));
    vqf_load(saved,.0025f,.01f,.01f);
    assert(state.bias[0]==0 && params.tauAcc==3.294f);
    for (int rate=100;rate<=400;rate*=2) {
        float dt=1.f/rate;
        vqf_init(dt,dt,dt);
        for (int i=0;i<40*rate;i++) { vqf_update_gyro(zero,dt);vqf_update_accel(a,dt); }
        assert(params.tauAcc==3.294f && vqf_get_rest_detected());
    }
    puts("native VQF parameters, core LP adaptation, retained state, rates, 6D/9D and inclination passed");
    return 0;
}
'''

FAST_MATH_TEST = r'''
int main(void) {
    const uint32_t bad_values[]={UINT32_C(0x7fc00001),UINT32_C(0x7f800000),UINT32_C(0xff800000)};
    vqf_init(.01f,.01f,.01f);
    for (int axis=0;axis<3;axis++) for (int kind=0;kind<3;kind++) {
        float input[3]={.1f,.2f,.3f};
        memcpy(&input[axis],&bad_values[kind],sizeof(input[axis]));
        vqf_state_t before=state;
        vqf_update_gyro(input,.01f);
        vqf_update_accel(input,.01f);
        assert(!memcmp(&before,&state,sizeof(state)));
    }
    unsigned char saved[VQF_MEM_SIZE];
    state.bias[1]=.1f;vqf_save(saved);
    memcpy(saved+sizeof(vqf_saved_header_t)+offsetof(vqf_state_t,bias)+sizeof(float),
           &bad_values[0],sizeof(float));
    vqf_load(saved,.01f,.01f,.01f);
    assert(state.bias[1]==0);
    puts("fast-math nonfinite samples and retained-state rejection passed");
    return 0;
}
'''


def main():
    with tempfile.TemporaryDirectory(prefix='vqf-native-') as tmp:
        directory = Path(tmp)
        source = directory / 'wrapper.c'
        source.write_text(translation_unit())
        cc = ['rtk', 'proxy', *shlex.split(os.environ.get('CC') or 'cc')]
        for mag in (0, 1):
            binary = directory / f'check-{mag}'
            common = ['-std=gnu11', '-O2', '-fsanitize=undefined', '-fno-sanitize-recover=all',
                      f'-DCONFIG_SENSOR_USE_MAGNETOMETER={mag}', '-DCONFIG_CONNECTION_RAW_COLLECTION=1',
                      '-I'+str(ROOT/'vqf-c/src')]
            subprocess.run(cc + common + [str(source), str(ROOT/'vqf-c/src/vqf.c'),
                                         '-lm', '-o', str(binary)], check=True)
            subprocess.run(['rtk', 'proxy', str(binary)], check=True)
            # Only the application uses fast-math; preserve native NaN startup arithmetic.
            fast_source = directory / 'fast-wrapper.c'
            fast_source.write_text(translation_unit().split('static void scalar_filter_native(void)', 1)[0]
                                   + FAST_MATH_TEST)
            core = directory / f'core-{mag}.o'
            wrapper = directory / f'fast-wrapper-{mag}.o'
            fast_binary = directory / f'fast-check-{mag}'
            subprocess.run(cc + common + ['-c', str(ROOT/'vqf-c/src/vqf.c'), '-o', str(core)], check=True)
            subprocess.run(cc + common + ['-ffast-math', '-c', str(fast_source),
                                          '-o', str(wrapper)], check=True)
            subprocess.run(cc + common + [str(wrapper), str(core), '-lm', '-o', str(fast_binary)], check=True)
            subprocess.run(['rtk', 'proxy', str(fast_binary)], check=True)


if __name__ == '__main__':
    main()
