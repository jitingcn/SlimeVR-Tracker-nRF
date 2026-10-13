#!/usr/bin/env python3
"""Exercise the production VQF wrapper's adaptive updates and retained lifecycle."""
import argparse
import os
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(os.environ.get('SOURCE_ROOT', Path(__file__).resolve().parents[3]))


def translation_unit():
    source = (ROOT / 'src/sensor/fusion/vqf/vqf.c').read_text()
    # Compile the actual wrapper through the accel entry point, including its
    # controller, init/load/save and input checks; only platform headers stubbed.
    source = source[:source.index('void vqf_update_mag(')]
    source = re.sub(r'^#include[^\n]*\n', '', source, flags=re.M)
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
struct retained_data { unsigned char fusion_data[2048]; };
''' + source + r'''
void firmware_run(const float *g, const float *a, int n, float gt, float at, float *q) {
    vqf_init(gt, at, gt);
    for (int i=0;i<n;i++) {
        float gyro[3], accel[3];
        memcpy(gyro,g+3*i,sizeof(gyro)); memcpy(accel,a+3*i,sizeof(accel));
        vqf_update_gyro(gyro,gt);
        if (accel[0]!=0 || accel[1]!=0 || accel[2]!=0) vqf_update_accel(accel,at);
        getQuat6D(&state,q+4*i);
    }
}
int main(void) {
    float g[3]={0,0,80}, a[3]={0,0,1};
    vqf_init(.0025f,.01f,.01f);
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
    memcpy(saved,&before,sizeof(before));
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
    puts("Production VQF adaptive input, bounds, rate and retained lifecycle passed");
    return 0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--export', type=Path, help='Also build shared wrappers for offline parity replay')
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='vqf-adaptive-') as tmp:
        directory = Path(tmp)
        source = directory / 'wrapper.c'
        source.write_text(translation_unit())
        for enabled in [0, 1]:
            # Match the GNU math constants of the frozen native reference.
            # Strict C hides M_SQRT2 and activates VQF's float fallback.
            common = ['rtk', 'proxy', 'cc', '-std=gnu11', '-O2', '-D' + f'CONFIG_VQF_ADAPTIVE_TAU_ACC={enabled}', '-I'+str(ROOT/'vqf-c/src'), str(source), str(ROOT/'vqf-c/src/vqf.c'), '-lm']
            binary = directory / f'check-{enabled}'
            subprocess.run(common + ['-fsanitize=undefined', '-fno-sanitize-recover=all', '-o', str(binary)], check=True)
            subprocess.run(['rtk', 'proxy', str(binary)], check=True)
            if args.export:
                args.export.mkdir(parents=True, exist_ok=True)
                subprocess.run(common + ['-shared', '-fPIC', '-o', str(args.export/f'firmware-{enabled}.so')], check=True)


if __name__ == '__main__':
    main()
