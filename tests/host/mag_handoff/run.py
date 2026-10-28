#!/usr/bin/env python3
"""Exercise production magnetic-domain rebases and gravity evidence on host."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'harness/python'))
from c_extract import extract_block

ROOT = Path(os.environ.get('SOURCE_ROOT', Path(__file__).resolve().parents[3]))

def function(source, name):
    return extract_block(source, rf'^(?:static )?(?:ALWAYS_INLINE )?(?:void|bool|float|int) {re.escape(name)}\([^;{{]*\)\s*\{{')

def conditional(source, pattern):
    """Keep a complete source conditional, including its inactive branches."""
    matches = list(re.finditer(pattern, source, re.MULTILINE))
    if len(matches) != 1:
        raise ValueError(f'Expected one source conditional: {pattern}')
    start = matches[0].start()
    depth = 0
    for directive in re.finditer(r'^[ \t]*#[ \t]*(if|ifdef|ifndef|endif)\b[^\n]*',
                                 source[start:], re.MULTILINE):
        depth += -1 if directive[1] == 'endif' else 1
        if depth == 0:
            return source[start:start + directive.end()]
    raise ValueError('Unclosed source conditional')

preamble = '''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
'''
vqf = (ROOT / 'src/sensor/fusion/vqf/vqf.c').read_text()
output_layout = conditional(
    vqf, r'^#if IS_ENABLED\(CONFIG_SENSOR_VQF_OUTPUT_OBSERVER\)\nstatic vqf_output_t output_observer;')
retained_layout = vqf[vqf.index('/* Raw state layouts are not interchangeable'):
                     vqf.index('static float last_a')]
vqf_test = preamble + '''
#include "vqf.h"
static vqf_params_t params;
static vqf_state_t state;
static vqf_coeffs_t coeffs;
static bool rest_observation_pending;
static float last_a[3];
#define ARG_UNUSED(x) ((void)(x))
#define IS_ENABLED(x) 0
/* This selective native-MAG fixture intentionally disables app policies.
 * The full public output/904-byte lifecycle is covered by vqf_adaptive_tau. */
#define CONFIG_SENSOR_VQF_OUTPUT_OBSERVER 0
#define DEG_TO_RAD 0.01745329251994329577f
#define CONST_EARTH_GRAVITY 9.80665f
static void vqf_track_rest_diag(void) {}
#define ALWAYS_INLINE inline
#define BUILD_ASSERT(c,m) _Static_assert(c,m)
#define printk(...) ((void)0)
''' + output_layout + '\n' + retained_layout + '''
struct retained_data { unsigned char fusion_data[2048]; };
static uint32_t rest_enter_count, rest_exit_count;
static float rest_total_s, rest_last_enter_time, rest_last_duration_s, uptime_s;
static bool prev_rest_detected;
static uint8_t rest_event_idx, rest_event_total;
''' + '\n'.join(function(vqf, name) for name in (
    'set_params', 'vqf_float_finite', 'vqf_vec3_finite', 'vqf_loaded_state_valid',
    'vqf_init', 'vqf_load', 'vqf_save', 'vqf_update_gyro', 'vqf_update_accel',
    'vqf_hard_rest_detected', 'vqf_get_rest_detected', 'vqf_take_rest_observation', 'vqf_rebase_mag',
    'vqf_rebase_gyro_bias')) + r'''
static void check_gyro_rebase(int warmup) {
    vqf_init(.01f,.01f,.01f);
    /* The native moving vertical-bias prior is anchored at zero, not
     * translation invariant. Isolate the rest observer and LP histories. */
    params.motionBiasEstEnabled=false;
    vqf_state_t shifted=state;
    const float delta_dps[3]={.35f,-.2f,.15f};
    vqf_real_t delta[3], g[3]={.003f,-.002f,.001f}, translated[3];
    vqf_real_t a[3]={.1f,-.2f,9.8f};
    for(int j=0;j<3;j++) {
        delta[j]=delta_dps[j]*DEG_TO_RAD;
        shifted.bias[j]+=delta[j];
        translated[j]=g[j]+delta[j];
    }
    for(int i=0;i<warmup;i++) {
        updateGyr(&params,&state,&coeffs,g);
        updateAcc(&params,&state,&coeffs,a);
        updateGyr(&params,&shifted,&coeffs,translated);
        updateAcc(&params,&shifted,&coeffs,a);
    }
    vqf_state_t before=state;
    vqf_rebase_gyro_bias(delta_dps);
    assert(!memcmp(before.gyrQuat,state.gyrQuat,sizeof(state.gyrQuat)));
    assert(!memcmp(before.accQuat,state.accQuat,sizeof(state.accQuat)));
    assert(!memcmp(before.biasP,state.biasP,sizeof(state.biasP)));
    assert(before.restDetected==state.restDetected && before.restT==state.restT);
    assert(!memcmp(before.restLastSquaredDeviations,state.restLastSquaredDeviations,
                   sizeof(state.restLastSquaredDeviations)));
    if(!warmup) {
        assert(!memcmp(before.restGyrLpState,state.restGyrLpState,sizeof(state.restGyrLpState)));
        assert(!memcmp(before.motionBiasEstBiasLpState,state.motionBiasEstBiasLpState,
                       sizeof(state.motionBiasEstBiasLpState)));
    } else {
        for(int j=0;j<6;j++) {
            assert((isnan(state.restGyrLpState[j]) && isnan(shifted.restGyrLpState[j]))
                   || fabs(state.restGyrLpState[j]-shifted.restGyrLpState[j])<2e-5);
        }
        for(int j=0;j<4;j++) {
            assert((isnan(state.motionBiasEstBiasLpState[j]) && isnan(shifted.motionBiasEstBiasLpState[j]))
                   || fabs(state.motionBiasEstBiasLpState[j]-shifted.motionBiasEstBiasLpState[j])<2e-5);
        }
    }
    for(int j=0;j<9;j++)
        assert(fabs(state.motionBiasEstLastR[j]-shifted.motionBiasEstLastR[j])<2e-6);
    for(int j=0;j<2;j++)
        assert(fabs(state.motionBiasEstLastBiasLp[j]-shifted.motionBiasEstLastBiasLp[j])<2e-6);
    /* A coefficient change immediately after translation must use the
     * translated actual output, not the pre-handoff cached bias. */
    vqf_params_t shifted_params=params;vqf_coeffs_t shifted_coeffs=coeffs;
    setTauAcc(&params,&state,&coeffs,1.5f);
    setTauAcc(&shifted_params,&shifted,&shifted_coeffs,1.5f);
    for(int j=0;j<4;j++)
        assert((isnan(state.motionBiasEstBiasLpState[j]) && isnan(shifted.motionBiasEstBiasLpState[j]))
               || fabs(state.motionBiasEstBiasLpState[j]-shifted.motionBiasEstBiasLpState[j])<2e-5);
    /* Compare with a native replay that used translated coordinates from
     * its very first sample, including warm-up -> mature transitions. */
    for(int i=0;i<400;i++) {
        updateGyr(&params,&state,&coeffs,translated);
        updateAcc(&params,&state,&coeffs,a);
        updateGyr(&params,&shifted,&coeffs,translated);
        updateAcc(&params,&shifted,&coeffs,a);
        vqf_real_t q[4], expected[4];
        getQuat6D(&state,q); getQuat6D(&shifted,expected);
        for(int j=0;j<4;j++) assert(fabs(q[j]-expected[j])<2e-5);
        for(int j=0;j<3;j++) assert(fabs(state.bias[j]-shifted.bias[j])<2e-5);
        assert(state.restDetected==shifted.restDetected);
        assert(fabs(state.restLastSquaredDeviations[0]-shifted.restLastSquaredDeviations[0])<1e-8);
    }
}
static void check_static_baseline_handoff(void) {
    /* Production parameters, raw z=.4 dps, old offset=.1, new offset=.3.
     * Warm the real native observer for 30 s before the -.2 dps handoff. */
    vqf_init(.01f,.01f,.01f);
    vqf_real_t old_g[3]={0,0,.3f*DEG_TO_RAD};
    vqf_real_t new_g[3]={0,0,.1f*DEG_TO_RAD};
    vqf_real_t a[3]={0,0,9.80665f};
    setBiasEstimate(&state,old_g,.18f*DEG_TO_RAD);
    for(int i=0;i<3000;i++) {
        updateGyr(&params,&state,&coeffs,old_g);
        updateAcc(&params,&state,&coeffs,a);
    }
    assert(state.restDetected);
    vqf_real_t warm_bias[3];
    memcpy(warm_bias,state.bias,sizeof(warm_bias));
    setBiasEstimate(&state,warm_bias,.18f*DEG_TO_RAD);
    vqf_state_t control=state;
    float delta[3]={0,0,-.2f};
    vqf_rebase_gyro_bias(delta);
    for(int i=0;i<3000;i++) {
        updateGyr(&params,&state,&coeffs,new_g);
        updateAcc(&params,&state,&coeffs,a);
        updateGyr(&params,&control,&coeffs,old_g);
        updateAcc(&params,&control,&coeffs,a);
        assert(state.restDetected && control.restDetected);
        for(int j=0;j<3;j++)
            assert(fabs((new_g[j]-state.bias[j])-(old_g[j]-control.bias[j]))<2e-7);
        assert(fabs(state.restLastSquaredDeviations[0]-control.restLastSquaredDeviations[0])<1e-10);
        vqf_real_t q[4], expected[4];
        getQuat6D(&state,q); getQuat6D(&control,expected);
        for(int j=0;j<4;j++) assert(fabs(q[j]-expected[j])<2e-5);
    }
}
static void check_current_period_restore(void) {
    unsigned char saved[VQF_MEM_SIZE];
    vqf_params_t p; vqf_state_t s; vqf_coeffs_t c;
    init_params(&p); initVqf(&p,&s,&c,.0025f,.005f,.02f);
    s.bias[0]=.125f;
    vqf_saved_header_t header={VQF_SAVED_MAGIC,VQF_SAVED_VERSION,VQF_MEM_SIZE};
    memcpy(saved,&header,sizeof(header));
    memcpy(saved+sizeof(header),&s,sizeof(s));
    memcpy(saved+sizeof(header)+sizeof(s),&c,sizeof(c));
    /* This is the first wrapper call, as on a retained reboot. */
    vqf_load(saved,.0025f,.005f,.02f);
    assert(state.bias[0]==.125f && coeffs.gyrTs==.0025f);
    const float changed[4][3]={{.004f,.005f,.02f},{.0025f,.01f,.02f},
                               {.0025f,.005f,.04f},{.0025f,.005f,0}};
    for(int i=0;i<4;i++) {
        vqf_load(saved,changed[i][0],changed[i][1],changed[i][2]);
        assert(state.bias[0]==0 && coeffs.gyrTs==changed[i][0]);
        assert(coeffs.accTs==changed[i][1]);
        assert(coeffs.magTs==(changed[i][2] ? changed[i][2] : changed[i][0]));
    }
    header.magic=0; memcpy(saved,&header,sizeof(header));
    vqf_load(saved,.004f,.008f,0);
    assert(state.bias[0]==0 && coeffs.gyrTs==.004f && coeffs.accTs==.008f && coeffs.magTs==.004f);
    vqf_init(.004f,.008f,0); state.bias[0]=.25f; vqf_save(saved);
    vqf_load(saved,.004f,.008f,0);
    assert(state.bias[0]==.25f && coeffs.magTs==.004f);
}
int main(void) {
    check_current_period_restore();
    check_static_baseline_handoff();
    check_gyro_rebase(0);   /* no samples: preserve native NaN sentinels */
    check_gyro_rebase(1);   /* partial initialization: translate sums */
    check_gyro_rebase(243); /* cross the motion-LP initialization boundary */
    check_gyro_rebase(350); /* mature native biquad histories */
    vqf_init(.01f,.01f,.01f);
    float large_delta[3]={50,-60,70};
    vqf_rebase_gyro_bias(large_delta);
    for(int j=0;j<3;j++) assert(fabs(state.bias[j]-large_delta[j]*DEG_TO_RAD)<1e-6);
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
    vqf_save(saved);
    vqf_update_accel(still,.01f); vqf_load(saved,.01f,.01f,.01f);
    assert(!vqf_take_rest_observation(&rest));
    vqf_state_t bad_state; memcpy(&bad_state,saved+sizeof(vqf_saved_header_t),sizeof(bad_state));
    bad_state.bias[0]=NAN; memcpy(saved+sizeof(vqf_saved_header_t),&bad_state,sizeof(bad_state));
    vqf_update_accel(still,.01f); vqf_load(saved,.01f,.01f,.01f);
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
''' + function((ROOT / 'src/util.c').read_text(), 'v_finite') + '\n' + eqf + r'''
static void check_gyro_rebase(void) {
    eqf_init(.01f,.01f,.01f);
    float delta[3]={.35f,-.2f,.15f};
    eqf_rebase_gyro_bias(delta);
    assert(!rest_gyr_lp_init); /* no fabricated history */
    eqf_init(.01f,.01f,.01f);
    mode=EQF_RUNNING;
    /* Nonidentity body->earth transform catches a wrong-sign/frame rebase. */
    const float rotation[9]={0,-1,0,1,0,0,0,0,1};
    memcpy(st.A,rotation,sizeof(rotation));
    float original_bias[3]={.1f,-.3f,.2f};
    eqf_set_gyro_bias(original_bias);
    float g[3]={.4f,-.1f,.3f};
    eqf_update_gyro(g,.01f);
    eqf_saved_t before=st;
    float bias_before[3], bias_after[3], lp_before[3], dev_before=rest_gyr_dev;
    eqf_get_gyro_bias(bias_before);
    memcpy(lp_before,rest_gyr_lp,sizeof(lp_before));
    eqf_rebase_gyro_bias(delta);
    eqf_get_gyro_bias(bias_after);
    assert(!memcmp(before.A,st.A,sizeof(st.A)) && !memcmp(before.P,st.P,sizeof(st.P)));
    assert(rest_gyr_lp_init && rest_gyr_dev==dev_before);
    for(int j=0;j<3;j++) {
        assert(fabsf(bias_after[j]-bias_before[j]-delta[j])<1e-5f);
        assert(fabsf(rest_gyr_lp[j]-lp_before[j]-delta[j]*DEG_TO_RAD)<1e-7f);
        g[j]+=delta[j];
    }
    eqf_update_gyro(g,.01f);
    assert(rest_gyr_dev<1e-12f); /* no reference-change motion impulse */
    float large_delta[3]={50,-60,70};
    eqf_get_gyro_bias(bias_before);
    eqf_rebase_gyro_bias(large_delta);
    eqf_get_gyro_bias(bias_after);
    for(int j=0;j<3;j++) assert(fabsf(bias_after[j]-bias_before[j]-large_delta[j])<1e-4f);
}
static void check_current_period_restore(void) {
    unsigned char saved[1024]={0};
    eqf_saved_t sample={0};
    sample.A[0]=sample.A[4]=sample.A[8]=1;
    sample.a_vec[0]=.25f; sample.d_mag[0]=1; sample.mag_ref_norm=1;
    sample.magic=EQF_MAGIC; memcpy(saved,&sample,sizeof(sample));
    eqf_load(saved,.0025f,.005f,0);
    assert(mode==EQF_RUNNING && st.a_vec[0]==.25f);
    assert(dt_gyr==.0025f && dt_acc==.005f && dt_mag==0 && init_count==0);
    sample.magic=0; memcpy(saved,&sample,sizeof(sample));
    eqf_load(saved,.004f,.008f,.02f);
    assert(mode==EQF_INIT && st.A[0]==1 && st.a_vec[0]==0 && st.P[0]==.1f);
    assert(dt_gyr==.004f && dt_acc==.008f && dt_mag==.02f && init_count==0);
}
static void check_nonfinite_inputs(void) {
    const float values[3]={NAN,INFINITY,-INFINITY};
    for(int running=0;running<2;running++) {
        eqf_init(.0025f,.005f,.02f);
        if(running) mode=EQF_RUNNING;
        float g[3]={1,2,3}, a[3]={0,0,1}, m[3]={1,0,0};
        eqf_update_gyro(g,.0025f); eqf_update_accel(a,.005f); eqf_update_mag(m,.02f);
        eqf_saved_t before=st;
        float accel_before[3], mag_before[3], last_before[3], gyro_lp_before[3], accel_lp_before[3];
        memcpy(accel_before,acc_sum,sizeof(accel_before)); memcpy(mag_before,mag_sum,sizeof(mag_before));
        memcpy(last_before,last_a_ms2,sizeof(last_before)); memcpy(gyro_lp_before,rest_gyr_lp,sizeof(gyro_lp_before));
        memcpy(accel_lp_before,rest_acc_lp,sizeof(accel_lp_before));
        int acc_count=acc_init_count, mag_count=mag_init_count;
        float candidate=mag_candidate_t, maglp=mag_norm_lp, rest_time=rest_t;
        for(int axis=0;axis<3;axis++) for(int kind=0;kind<3;kind++) {
            float bad[3]={1,2,3}; bad[axis]=values[kind];
            eqf_update_gyro(bad,.0025f); eqf_update_accel(bad,.005f); eqf_update_mag(bad,.02f);
            eqf_set_gyro_bias(bad); eqf_rebase_gyro_bias(bad);
            eqf_rebase_mag(values[kind],0); eqf_rebase_mag(1,values[kind]);
            assert(!memcmp(&before,&st,sizeof(st)));
            assert(!memcmp(accel_before,acc_sum,sizeof(accel_before)) && acc_count==acc_init_count);
            assert(!memcmp(mag_before,mag_sum,sizeof(mag_before)) && mag_count==mag_init_count);
            assert(!memcmp(last_before,last_a_ms2,sizeof(last_before)));
            assert(!memcmp(gyro_lp_before,rest_gyr_lp,sizeof(gyro_lp_before)));
            assert(!memcmp(accel_lp_before,rest_acc_lp,sizeof(accel_lp_before)));
            assert(candidate==mag_candidate_t && maglp==mag_norm_lp && rest_time==rest_t);
        }
        if(running) {
            eqf_update_gyro(g,.0025f); assert(memcmp(&before,&st,sizeof(st)));
        } else {
            eqf_update_accel(a,.005f); assert(acc_init_count==acc_count+1);
        }
    }
}
static void check_poisoned_retained(void) {
    unsigned char saved[1024]={0};
    eqf_init(.01f,.01f,.01f); mode=EQF_RUNNING;
    float g[3]={1,2,3}; eqf_update_gyro(g,.01f); eqf_save(saved);
    eqf_saved_t good; memcpy(&good,saved,sizeof(good));
    const float values[3]={NAN,INFINITY,-INFINITY};
    for(int group=0;group<5;group++) for(int kind=0;kind<3;kind++) {
        eqf_saved_t bad=good;
        float *fields[5]={bad.A,bad.a_vec,bad.P,bad.d_mag,&bad.mag_ref_norm};
        const int sizes[5]={9,3,36,3,1};
        for(int i=0;i<sizes[group];i++) {
            bad=good; fields[group][i]=values[kind]; memcpy(saved,&bad,sizeof(bad));
            eqf_load(saved,.0025f,.005f,.02f);
            assert(mode==EQF_INIT && st.magic==0 && st.A[0]==1 && st.P[0]==.1f);
            assert(v_finite(st.A,9) && v_finite(st.a_vec,3) && v_finite(st.P,36));
            assert(v_finite(st.d_mag,3) && v_finite(&st.mag_ref_norm,1));
            assert(dt_gyr==.0025f && dt_acc==.005f && dt_mag==.02f);
        }
    }
    eqf_init(.01f,.01f,.01f); eqf_save(saved);
    eqf_load(saved,.004f,.008f,0);
    assert(mode==EQF_INIT && st.magic==0 && st.A[0]==1 && dt_gyr==.004f);
}
int main(void) {
    check_current_period_restore();
    check_nonfinite_inputs();
    check_poisoned_retained();
    check_gyro_rebase();
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
    unsigned char saved[1024]={0}; eqf_load(saved,.0025f,.005f,0); /* Invalid-load early return. */
    assert(!eqf_take_rest_observation(&rest));
    mode=EQF_RUNNING; eqf_update_accel(still,.01f); eqf_save(saved); eqf_load(saved,.0025f,.005f,0);
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
period_test = preamble + r'''
#define CONFIG_SENSOR_ACCEL_OVERSAMPLING 2
#define CONFIG_SENSOR_MAG_ODR 50
static float gyro_actual_time=.0025f, accel_actual_time=.005f, mag_actual_time=.04f;
static unsigned gyro_oversample_n=4;
static bool mag_available=true, mag_enabled=true;
''' + extract_block(sensor, r'^struct fusion_periods\s*\{', semicolon=True) + '\n' + \
    extract_block(sensor, r'^static struct fusion_periods sensor_fusion_periods\([^;{}]*\)\s*\{') + r'''
int main(void) {
    struct fusion_periods p=sensor_fusion_periods();
    assert(p.gyro==.01f && p.accel==.01f && p.mag==.04f);
    gyro_oversample_n=1; mag_enabled=false; p=sensor_fusion_periods();
    assert(p.gyro==.0025f && p.accel==.01f && p.mag==0);
    mag_enabled=true; mag_available=false; p=sensor_fusion_periods();
    assert(p.mag==0);
    mag_available=true; mag_actual_time=INFINITY; p=sensor_fusion_periods();
    assert(p.mag==.02f);
    puts("Current sensor periods match merge and magnetometer mode");
}
'''
mag_acquire_start = sensor.index('\t// Read magnetometer\n')
# Transition and acquisition bodies are production code; hardware, PM, storage,
# scheduling and LED leaves are modeled below.
mag_acquire_end = sensor.index('#if CONFIG_CONNECTION_RAW_COLLECTION', mag_acquire_start)
transition_test = preamble + r'''
#include <errno.h>
#define CONFIG_SENSOR_TCAL_HEATED 1
#define CONFIG_SENSOR_USE_TCAL 0
#define CONFIG_SENSOR_MAG_ODR 50
#define LOG_INF(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define LOG_ERR(...) ((void)0)
#define LED_OWNER_MAG 1
#define LED_PARTIAL 2
#define LED_ACCEPTED 3
#define LED_APPLIED_NOT_SAVED 4
#define MAG_ENABLED_ID 9
#define TRACKER_REST_RESET 7
typedef int atomic_t;
static int atomic_get(atomic_t *v) { return *v; }
static void atomic_set(atomic_t *v, int n) { *v=n; }
static void atomic_clear(atomic_t *v) { *v=0; }
static bool atomic_cas(atomic_t *v, int old, int next) {
    if (*v!=old) return false; *v=next; return true;
}
static bool mag_available=true, mag_enabled, sensor_sensor_init=true, main_ok=true;
static bool hardware_on, heated_resting, sensor_fusion_init=true, skip_fusion_save, mag_fusion_restarted;
static unsigned mag_setting_writes;
static atomic_t main_suspended, output_ready, mag_persistence_error, fusion_requests, mag_fusion_finish_pending;
static int mag_fusion_lock;
#define K_FOREVER 0
static int k_mutex_lock(int *lock,int timeout) { (void)lock; (void)timeout; return 0; }
static void k_mutex_unlock(int *lock) { (void)lock; }
static int magneto_progress, pm_error, restart_error, enable_error;
static int mag_reads, ref_resets, restart_calls, partial_events, saves, retained_updates;
static bool calibration_stale, gyro_reference_pending;
static bool interleave_initialization;
static void sensor_finish_mag_fusion_restart(bool restarted);
static float gyro_actual_time=.0025f, accel_actual_time=.005f, mag_actual_time=.02f;
static unsigned gyro_oversample_n=CONFIG_SENSOR_GYRO_OVERSAMPLING;
static int64_t next_mag_read_ticks, mag_read_period_ticks=1, last_mag_fusion_ticks;
#if CONFIG_SENSOR_GYRO_OVERSAMPLING > 1
static int gyro_oversample_count;
static float gyro_dq_acc[4], gyro_merge_bias_dps[3];
#endif
#if CONFIG_SENSOR_ACCEL_OVERSAMPLING > 1
static int accel_oversample_count;
static float accel_oversample_sum[3];
#endif
static struct {bool mag_enabled; int fusion_id; unsigned char fusion_data[8];} retained_data;
static typeof(retained_data) *retained=&retained_data;
static int fusion_id=4;
static void retained_update(void) { retained_updates++; }
static bool sensor_calibration_fusion_stale(void) { return calibration_stale; }
static bool sensor_calibration_gyro_reference_pending(void) { return gyro_reference_pending; }
static int main_imu_suspend(void) { main_suspended=1; return 0; }
static void main_imu_resume(void) { main_suspended=0; }
static void sensor_release_failed_restart(void) {}
static int sys_interface_resume(void) { return 0; }
static int sys_interface_suspend(void) { return pm_error; }
static int sensor_mag_runtime_enable(void) { if (enable_error) return enable_error; hardware_on=true; return 0; }
static void sensor_mag_runtime_disable(void) { hardware_on=false; }
static int sys_write(int id, void *dest, const void *src, size_t n) {
    assert(id==MAG_ENABLED_ID && skip_fusion_save && retained->fusion_id==0);
    if (interleave_initialization) {
        int before=ref_resets;
        sensor_finish_mag_fusion_restart(true);
        assert(skip_fusion_save && ref_resets==before);
    }
    memcpy(dest,src,n); return 0;
}
static int sys_request_system_reboot(void) { return 0; }
static int led_request_id(void) { return 0; }
static int led_event_id(void) { return 0; }
static void led_request_event(int owner,int req,int event,int outcome) {
    (void)owner; (void)req; (void)event; if(outcome==LED_PARTIAL) partial_events++;
}
static int sensor_operation_result(int owner,int err,bool applied) {
    (void)owner; (void)applied; return err;
}
static void sensor_refresh_sensor_ids(void) {}
static void sensor_mag_timing_reset(void) { next_mag_read_ticks=0; }
static void sensor_calibration_reset_gyro_reference(void) {}
static void sys_cancel_WOM(void) {}
static void tracker_events_sensor_invalidate(int reason) { (void)reason; }
static void tracker_events_notify(void) {}
static bool connection_get_data_collection(void) { return false; }
static bool connection_get_data_collection_batch(void) { return false; }
static void sensor_send_raw_metadata(void) {}
static int sensor_temperature_invalidate(void) { return restart_error; }
static void sensor_temperature_resume(void) {}
static bool sensor_fusion_get_mag_ref(float *norm,float *dip) { *norm=0; *dip=0; return false; }
typedef int k_spinlock_key_t;
static struct {int unused;} mag_ref_request_lock;
static bool mag_ref_request_pending;
static float mag_ref_request_norm, mag_ref_request_dip;
static k_spinlock_key_t k_spin_lock(void *lock) { (void)lock; return 0; }
static void k_spin_unlock(void *lock, k_spinlock_key_t key) { (void)lock; (void)key; }
static bool magneto_online_take_mag_ref(float *norm,float *dip) { (void)norm; (void)dip; return false; }
static float last_fusion_mag;
static void fusion_init(float g,float a,float m) { (void)g; (void)a; last_fusion_mag=m; restart_calls++; }
static void rebase_mag(float norm,float dip) { assert(norm==0 && dip==0); ref_resets++; }
static void save_fusion(void *data) {
    assert(!skip_fusion_save && !mag_ref_request_pending);
    ((unsigned char *)data)[0]=(unsigned char)(40+ ++saves);
}
static struct {void (*init)(float,float,float); void (*rebase_mag)(float,float); void (*save)(void *);} backend={
    fusion_init, rebase_mag, save_fusion
};
static typeof(backend) *sensor_fusion=&backend;
static bool read_mag(float *out) { assert(hardware_on); mag_reads++; out[0]=7; return true; }
static struct {bool (*mag_read)(float *);} mag_driver={read_mag};
static typeof(mag_driver) *sensor_mag=&mag_driver;
static int64_t k_uptime_ticks(void) { return 10; }
typedef struct {float raw_m[3]; bool new_mag_data;} sensor_loop_frame_t;
''' + extract_block(sensor, r'^struct fusion_periods\s*\{', semicolon=True) + '\n' + \
    extract_block(sensor, r'^static struct fusion_periods sensor_fusion_periods\([^;{}]*\)\s*\{') + '\n' + \
    '\n'.join(function(sensor, name) for name in (
        'sensor_fusion_set_mag_ref', 'sensor_fusion_reset_mag_ref', 'sensor_service_mag_ref',
        'sensor_finish_mag_fusion_restart', 'sensor_service_pending_mag_fusion',
        'sensor_retained_write', 'sensor_invalidate_mag_fusion')) + '\n' + \
    function(sensor, 'sensor_discard_gyro_window') + '\n' + \
    function(sensor, 'sensor_discard_partial_samples') + '\n' + \
    function(sensor, 'main_imu_restart') + '\n' + \
    function(sensor, 'sensor_set_mag_enabled') + '\n' + \
    'static void acquire(sensor_loop_frame_t *frame) {\n' + sensor[mag_acquire_start:mag_acquire_end] + '\n}\n' + r'''
static void pending_samples(void) {
#if CONFIG_SENSOR_GYRO_OVERSAMPLING > 1
    gyro_oversample_count=2; gyro_dq_acc[0]=.5f; gyro_merge_bias_dps[2]=3;
#endif
#if CONFIG_SENSOR_ACCEL_OVERSAMPLING > 1
    accel_oversample_count=1; accel_oversample_sum[1]=4;
#endif
}
static void assert_discarded(void) {
#if CONFIG_SENSOR_GYRO_OVERSAMPLING > 1
    assert(!gyro_oversample_count && gyro_dq_acc[0]==1 && gyro_dq_acc[1]==0);
    assert(gyro_merge_bias_dps[2]==0);
#endif
#if CONFIG_SENSOR_ACCEL_OVERSAMPLING > 1
    assert(!accel_oversample_count && accel_oversample_sum[1]==0);
#endif
}
int main(void) {
    sensor_loop_frame_t frame={0};
    retained->fusion_id=fusion_id; retained->fusion_data[0]=17;
    pm_error=-EIO; pending_samples();
    assert(sensor_set_mag_enabled(true)==-EIO);
    assert(mag_enabled && hardware_on && retained->mag_enabled && !main_suspended);
    assert(last_fusion_mag==.02f && restart_calls==1 && ref_resets==1 && partial_events==1);
    assert(!skip_fusion_save && retained->fusion_id==0);
    sensor_retained_write(); assert(retained->fusion_id==fusion_id && retained->fusion_data[0]==41);
    calibration_stale=true; sensor_retained_write(); assert(retained->fusion_id==0 && saves==1);
    calibration_stale=false; gyro_reference_pending=true;
    sensor_retained_write(); assert(retained->fusion_id==0 && saves==1);
    gyro_reference_pending=false;
    assert_discarded(); acquire(&frame);
    assert(frame.new_mag_data && frame.raw_m[0]==7 && mag_reads==1);
    pending_samples();
    assert(sensor_set_mag_enabled(false)==-EIO);
    assert(!mag_enabled && !hardware_on && !retained->mag_enabled && !main_suspended);
    assert(last_fusion_mag==0 && restart_calls==2 && ref_resets==2 && partial_events==2);
    assert_discarded(); acquire(&frame);
    assert(!frame.new_mag_data && mag_reads==1);
    pm_error=0; restart_error=-EBUSY; pending_samples();
    assert(sensor_set_mag_enabled(true)==-EBUSY);
    assert(mag_enabled && hardware_on && main_suspended && restart_calls==2);
    assert(skip_fusion_save && retained->fusion_id==0);
    sensor_retained_write(); assert(retained->fusion_id==0 && saves==1);
    restart_error=0; assert(main_imu_restart()==0); assert_discarded();
    assert(!skip_fusion_save && ref_resets==3);
    sensor_retained_write(); assert(retained->fusion_id==fusion_id && saves==2);
    main_suspended=0; mag_enabled=false; hardware_on=false; main_ok=false; retained->mag_enabled=false;
    interleave_initialization=true;
    assert(sensor_set_mag_enabled(true)==0);
    interleave_initialization=false;
    assert(skip_fusion_save && retained->fusion_id==0);
    sensor_retained_write(); assert(retained->fusion_id==0 && saves==2);
    main_ok=true; mag_enabled=true; assert(main_imu_restart()==0);
    assert(!skip_fusion_save); sensor_retained_write(); assert(saves==3 && retained->fusion_id==fusion_id);
    main_ok=false; mag_enabled=false; retained->mag_enabled=true;
    interleave_initialization=true;
    assert(sensor_set_mag_enabled(false)==0 && !retained->mag_enabled && skip_fusion_save);
    interleave_initialization=false;
    assert(retained->fusion_id==0 && mag_fusion_finish_pending==1);
    sensor_retained_write(); assert(retained->fusion_id==0 && saves==3);
    main_ok=true; sensor_service_pending_mag_fusion();
    assert(!skip_fusion_save && !mag_fusion_finish_pending);
    sensor_retained_write(); assert(retained->fusion_id==fusion_id && saves==4);
    mag_enabled=false; mag_available=false; hardware_on=false;
    assert(sensor_set_mag_enabled(true)==0);
    assert(mag_enabled && !hardware_on && !skip_fusion_save && last_fusion_mag==0);
    sensor_retained_write(); assert(saves==5 && retained->fusion_id==fusion_id);
    puts("PM mode and retained fusion gates survive failure, recovery and absent hardware");
}
'''
with tempfile.TemporaryDirectory(prefix='mag-handoff-') as tmp:
    tmp = Path(tmp)
    cc = shlex.split(os.environ.get('CC', 'cc'))
    jobs = [('vqf', vqf_test, ['-I'+str(ROOT/'vqf-c/src'), str(ROOT/'vqf-c/src/vqf.c')]),
            ('eqf', eqf_test, []), ('eqf_fast', eqf_test, ['-ffast-math']), ('periods', period_test, [])]
    for n in (1, 4):
        jobs.append((f'transition{n}', transition_test,
                     [f'-DCONFIG_SENSOR_GYRO_OVERSAMPLING={n}', f'-DCONFIG_SENSOR_ACCEL_OVERSAMPLING={n}']))
    for index, axes in enumerate(('mx,my,mz','my,-mx,-mz','my,mx,mz','-mx,my,mz')):
        jobs.append((f'gravity{index}', gravity_test, ['-DSENSOR_MAGNETOMETER_AXES_ALIGNMENT='+axes]))
    for name, text, extra in jobs:
        source = tmp/(name+'.c'); source.write_text(text)
        executable = tmp/name
        subprocess.run(cc+['-std=gnu11','-O1','-g','-DCONFIG_CONNECTION_RAW_COLLECTION=1','-DCONFIG_SENSOR_USE_MAGNETOMETER=1',str(source)]+extra+['-lm','-o',str(executable)],check=True)
        subprocess.run([str(executable)],check=True)
