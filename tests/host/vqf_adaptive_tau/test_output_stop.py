#!/usr/bin/env python3
"""Real public-output, retained-lifecycle and finite STOP-gate regressions.

Reuse the production translation unit, not a research implementation. Platform
headers and single-owner atomics are the only shims. Histories come from actual
sample feeds; retained corruption is injected as bytes, including under app
fast-math. Paired feature builds also compare the native q/bias trajectory.
"""
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True
import run as application
from test_cycle_rebase import ATOMIC_LEAVES

ROOT = application.ROOT

# Standalone core + observer, with no application hooks. This also permits a
# saved core to be compared without mixing incompatible hook ABIs.
INCLINATION_TEST = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "vqf.h"

_Static_assert(sizeof(vqf_real_t) == 4 && sizeof(vqf_double_t) == 8,
               "Inclination regression requires the default mixed precision");

static float q_difference(const float a[4], const float b[4]) {
    float same = 0, opposite = 0;
    for (int j = 0; j < 4; j++) {
        same = fmaxf(same, fabsf(a[j] - b[j]));
        opposite = fmaxf(opposite, fabsf(a[j] + b[j]));
    }
    return fminf(same, opposite);
}
static void assert_unit(const float q[4]) {
    double norm2 = 0;
    for (int j = 0; j < 4; j++) {
        assert(isfinite(q[j]));
        norm2 += (double)q[j] * q[j];
    }
    assert(fabs(norm2 - 1) < 2e-6);
}
static void correction(const float after[4], const float before[4], float out[4]) {
    /* after * conjugate(before): compare rotations, not quaternion signs. */
    double w = after[0], x = after[1], y = after[2], z = after[3];
    double a = before[0], b = -before[1], c = -before[2], d = -before[3];
    out[0] = (float)(w*a - x*b - y*c - z*d);
    out[1] = (float)(w*b + x*a + y*d - z*c);
    out[2] = (float)(w*c - x*d + y*a + z*b);
    out[3] = (float)(w*d + x*c - y*b + z*a);
}
static void assert_gravity_up(const float q[4], const float acc[3]) {
    /* Independent quaternion/vector rotation; near 180 degrees Float32
     * rounds z to -1, so the native fallback's remaining error can be O(xy). */
    double n = sqrt((double)acc[0]*acc[0] + (double)acc[1]*acc[1] + (double)acc[2]*acc[2]);
    double x = acc[0]/n, y = acc[1]/n, z = acc[2]/n;
    double tx = 2*(q[2]*z - q[3]*y);
    double ty = 2*(q[3]*x - q[1]*z);
    double tz = 2*(q[1]*y - q[2]*x);
    double ex = x + q[0]*tx + q[2]*tz - q[3]*ty;
    double ey = y + q[0]*ty + q[3]*tx - q[1]*tz;
    double ez = z + q[0]*tz + q[1]*ty - q[2]*tx;
    assert(hypot(ex, ey) < 6e-4);
    assert(fabs(ez - 1) < 2e-6);
}
int main(void) {
    static const struct { const char *name; float acc[3]; } cases[] = {
        {"exact inversion", {0, 0, -1}},
        {"near +X", {1e-4f, 0, -1}},
        {"near -X", {-1e-4f, 0, -1}},
        {"near +Y", {0, 1e-4f, -1}},
        {"near -Y", {0, -1e-4f, -1}},
        {"near mixed XY", {7e-5f, -7e-5f, -1}},
        {"near opposite XY", {-6e-5f, 8e-5f, -1}},
        {"indeterminate axis", {5e-7f, 0, -1}},
        {"near standard branch", {4e-4f, 2e-4f, -1}},
        {"upright", {0, 0, 1}},
        {"horizontal", {1, 0, 0}},
        {"ordinary positive", {.6f, -.2f, .77f}},
        {"ordinary negative", {-.3f, .4f, -.866f}},
    };
    for (size_t i = 0; i < sizeof(cases)/sizeof(cases[0]); i++) {
        for (int yaw = 0; yaw < 2; yaw++) {
            vqf_params_t p; vqf_state_t s; vqf_coeffs_t c; vqf_output_t output;
            init_params(&p);
            p.tauAcc = VQF_OUTPUT_TAU_ACC;
            p.motionBiasEstEnabled = p.restBiasEstEnabled = p.magDistRejectionEnabled = false;
            initVqf(&p, &s, &c, .005f, .02f, .02f);
            if (yaw) {
                s.accQuat[0] = cosf(.31f);
                s.accQuat[3] = sinf(.31f);
            }
            initOutputObserver(&output, &s, &c);
            assert(!memcmp(s.accQuat, output.accQuat, sizeof(output.accQuat)));
            assert(!memcmp(c.accLpB, output.b, sizeof(output.b)));
            assert(!memcmp(c.accLpA, output.a, sizeof(output.a)));
            /* Include first-sample inversion and the actual transition from
             * running-mean startup to the steady Butterworth recurrence. */
            for (int sample = 0; sample < 180; sample++) {
                float native_before[4], output_before[4], native_corr[4], output_corr[4];
                memcpy(native_before, s.accQuat, sizeof(native_before));
                memcpy(output_before, output.accQuat, sizeof(output_before));
                updateAcc(&p, &s, &c, cases[i].acc);
                updateOutputObserver(&output, &s, &c, cases[i].acc);
                assert(!memcmp(s.accLpState, output.lp, sizeof(output.lp)));
                assert_unit(s.accQuat);
                assert_unit(output.accQuat);
                correction(s.accQuat, native_before, native_corr);
                correction(output.accQuat, output_before, output_corr);
                float attitude_error = q_difference(s.accQuat, output.accQuat);
                float correction_error = q_difference(native_corr, output_corr);
                if (attitude_error >= 2e-6f || correction_error >= 2e-6f)
                    fprintf(stderr, "inclination %s yaw=%d sample=%d: attitude=%g correction=%g\n",
                            cases[i].name, yaw, sample, attitude_error, correction_error);
                assert(attitude_error < 2e-6f && correction_error < 2e-6f);
                assert_gravity_up(s.accQuat, cases[i].acc);
                assert_gravity_up(output.accQuat, cases[i].acc);
            }
        }
    }
    puts("mixed-precision core/output inversion and ordinary inclination corrections passed");
    return 0;
}
'''

TEST = r'''
#include <stdlib.h>

#define GYR_DT .005f
#define ACC_DT .02f
#if CONFIG_SENSOR_VQF_OUTPUT_OBSERVER
#define SAVED_BYTES 904
#define SAVED_VERSION 2
#else
#define SAVED_BYTES 840
#define SAVED_VERSION 1
#endif

static float measured_acc[3];
static uint64_t native_hash;

static bool finite_double(double value) {
    uint64_t bits; memcpy(&bits, &value, sizeof(bits));
    return (bits & UINT64_C(0x7ff0000000000000)) != UINT64_C(0x7ff0000000000000);
}
static void hash_bytes(const void *data, size_t count) {
    const unsigned char *bytes = data;
    for (size_t j = 0; j < count; j++) {
        native_hash ^= bytes[j]; native_hash *= UINT64_C(1099511628211);
    }
}
static void record_native(void) {
    float q[4]; getQuat6D(&state, q);
    hash_bytes(q, sizeof(q));
    hash_bytes(state.bias, sizeof(state.bias));
    hash_bytes(state.biasP, sizeof(state.biasP));
}
static void init_fixture(void) {
    /* Consume any previous owner's request before beginning a new lifetime. */
    float invalid[3] = {NAN, 0, 0};
    vqf_update_gyro(invalid, GYR_DT);
    host_mag_available = host_mag_enabled = false;
    vqf_init(GYR_DT, ACC_DT, ACC_DT);
#if CONFIG_SENSOR_VQF_FAST_REST_RECOVERY
    assert(stop_macro_gate.peak2 == 0 && stop_macro_gate.remaining == 0);
    assert(stop_macro_gate.base_samples == 0);
#endif
}
static void send_acc(float a[3]) {
    vqf_update_accel(a, ACC_DT);
    if (vqf_vec3_finite(a) && (a[0] != 0 || a[1] != 0 || a[2] != 0))
        for (int j = 0; j < 3; j++) measured_acc[j] = a[j] * CONST_EARTH_GRAVITY;
}
static void flat_step(float z_dps, int row, bool accel) {
    float g[3] = {0, 0, z_dps}, a[3] = {0, 0, 1};
    vqf_update_gyro(g, GYR_DT);
    if (accel && row % 4 == 0) send_acc(a);
}
static void expected_public(float q[4]) {
#if CONFIG_SENSOR_VQF_OUTPUT_OBSERVER
#if CONFIG_SENSOR_USE_MAGNETOMETER
    if (host_mag_available && host_mag_enabled) { getQuat9D(&state, q); return; }
#endif
    getOutputQuat6D(&output_observer, &state, q);
#elif CONFIG_SENSOR_USE_MAGNETOMETER
    getQuat9D(&state, q);
#else
    getQuat6D(&state, q);
#endif
}
static void assert_public_and_linear(void) {
    float q[4], expected[4], linear[3];
    vqf_get_quat(q); expected_public(expected);
    assert(!memcmp(q, expected, sizeof(q)));
    vqf_get_lin_a(linear);
    float gravity[3] = {
        2 * (q[1] * q[3] - q[0] * q[2]),
        2 * (q[2] * q[3] + q[0] * q[1]),
        2 * (q[0] * q[0] - .5f + q[3] * q[3]),
    };
    for (int j = 0; j < 3; j++) {
        assert(vqf_float_finite(q[j]));
        assert(fabsf(linear[j] - (measured_acc[j] - gravity[j] * CONST_EARTH_GRAVITY)) < 3e-5f);
    }
    assert(vqf_float_finite(q[3]));
}
static float q_difference(const float a[4], const float b[4]) {
    float same = 0, opposite = 0;
    for (int j = 0; j < 4; j++) {
        same = fmaxf(same, fabsf(a[j] - b[j]));
        opposite = fmaxf(opposite, fabsf(a[j] + b[j]));
    }
    return fminf(same, opposite);
}
static void quaternion_product(const float a[4], const float b[4], float out[4]) {
    /* Independent double-precision Hamilton product, not a private core leaf.
     * Public-vs-native getters remain bit-exact; this algebra oracle allows
     * only Float32 rounding when the actual app uses fast-math.
     */
    double w = a[0], x = a[1], y = a[2], z = a[3];
    out[0] = (float)(w * b[0] - x * b[1] - y * b[2] - z * b[3]);
    out[1] = (float)(w * b[1] + x * b[0] + y * b[3] - z * b[2]);
    out[2] = (float)(w * b[2] - x * b[3] + y * b[0] + z * b[1]);
    out[3] = (float)(w * b[3] + x * b[2] - y * b[1] + z * b[0]);
}
static double yaw(const float q[4]) {
    return atan2(2. * ((double)q[0] * q[3] + (double)q[1] * q[2]),
                 1. - 2. * ((double)q[2] * q[2] + (double)q[3] * q[3]));
}
static void rich_vectors(int row, float g[3], float a[3]) {
    /* A changing specific-force direction/magnitude and multi-axis rotation
     * generate real noncommuting inclination corrections. No q/LP/policy seed.
     */
    double t = (row + 1) * (double)GYR_DT;
    g[0] = (float)(18 * sin(.83 * t));
    g[1] = (float)(13 * cos(.51 * t));
    g[2] = (float)(9 + 6 * sin(.37 * t));
    a[0] = (float)(.65 * sin(.71 * t));
    a[1] = (float)(.55 * cos(.49 * t));
    a[2] = (float)(1. + .32 * sin(.23 * t));
}
static void rich_step(int row) {
    float g[3], a[3]; rich_vectors(row, g, a);
    vqf_update_gyro(g, GYR_DT);
    if (row % 4 == 0) send_acc(a);
#if CONFIG_SENSOR_USE_MAGNETOMETER
    if (row % 16 == 0) {
        float m[3] = {.35f, -.42f, .18f};
        vqf_update_mag(m, .08f);
    }
#endif
}
static void native_trajectory(void) {
    init_fixture(); native_hash = UINT64_C(1469598103934665603);
    for (int row = 0; row < 6000; row++) {
        if (row < 1600) flat_step(0, row, true);
        else if (row < 1800) flat_step(40, row, true);
        else if (row < 5000) flat_step(.025f, row, true);
        else rich_step(row - 5000);
        record_native();
    }
    printf("native-q-bias=%016llx\n", (unsigned long long)native_hash);
}

#if CONFIG_SENSOR_VQF_OUTPUT_OBSERVER
static void fixed_output_oracle_and_cadence(void) {
    init_fixture();
    vqf_params_t fixed_params; init_params(&fixed_params);
    fixed_params.tauAcc = VQF_OUTPUT_TAU_ACC;
    fixed_params.motionBiasEstEnabled = fixed_params.restBiasEstEnabled = false;
    fixed_params.magDistRejectionEnabled = false;
    vqf_state_t fixed_state; vqf_coeffs_t fixed_coeffs;
    initVqf(&fixed_params, &fixed_state, &fixed_coeffs, GYR_DT, ACC_DT, ACC_DT);
    bool native_tau_changed = false, output_separated = false;
    for (int row = 0; row < 4800; row++) {
        float g[3], a[3]; rich_vectors(row, g, a);
        vqf_output_t before = output_observer;
        vqf_update_gyro(g, GYR_DT);
        /* No held ACC is replayed on a gyro-only tick. */
        assert(!memcmp(&before, &output_observer, sizeof(before)));
        if (row % 4 != 0) continue;
        memcpy(fixed_state.gyrQuat, state.gyrQuat, sizeof(state.gyrQuat));
        float physical[3];
        for (int j = 0; j < 3; j++) physical[j] = a[j] * CONST_EARTH_GRAVITY;
        updateAcc(&fixed_params, &fixed_state, &fixed_coeffs, physical);
        send_acc(a);
        /* Independent fixed-tau native filter, sharing only the actual gyro
         * prediction, is the oracle across bootstrap and steady recurrence.
         */
        assert(!memcmp(fixed_state.accLpState, output_observer.lp, sizeof(output_observer.lp)));
        assert(q_difference(fixed_state.accQuat, output_observer.accQuat) < 2e-5f);
        native_tau_changed |= fabsf(params.tauAcc - VQF_OUTPUT_TAU_ACC) > .1f;
        output_separated |= q_difference(state.accQuat, output_observer.accQuat) > 1e-3f;
        assert_public_and_linear();
    }
    assert(native_tau_changed && output_separated);
    assert(fabs(remainder(yaw(state.accQuat) - yaw(output_observer.accQuat), 2 * M_PI)) > 1e-4);
    assert(finite_double(output_observer.lp[0]));

    vqf_output_t before = output_observer;
    float invalid[][3] = {{0, 0, 0}, {NAN, 0, 1}, {0, INFINITY, 1}, {0, 0, -INFINITY}};
    for (size_t j = 0; j < sizeof(invalid) / sizeof(invalid[0]); j++) {
        send_acc(invalid[j]);
        assert(!memcmp(&before, &output_observer, sizeof(before)));
    }
    /* The helper is read-only with respect to the native filter and policy;
     * the paired builds additionally compare the full actual q/bias replay.
     */
    vqf_state_t native_before = state;
    vqf_coeffs_t coeffs_before = coeffs;
    vqf_params_t params_before = params;
    vqf_output_t separate = output_observer;
    float force[3] = {1, 2, 9};
    updateOutputObserver(&separate, &state, &coeffs, force);
    assert(!memcmp(&native_before, &state, sizeof(state)));
    assert(!memcmp(&coeffs_before, &coeffs, sizeof(coeffs)));
    assert(!memcmp(&params_before, &params, sizeof(params)));
    assert(!memcmp(&before, &output_observer, sizeof(before)));

    float q_before[4], actual[4], expected[4], g[3] = {17, -9, 23};
    vqf_get_quat(q_before);
    vqf_update_gyro(g, 17.f); /* Caller time is not an extra observer tick. */
    quaternion_product(before.accQuat, state.gyrQuat, expected);
    vqf_get_quat(actual);
    assert(q_difference(expected, actual) < 3e-7f);
    assert(q_difference(actual, q_before) > 1e-5f);
    assert(!memcmp(&before, &output_observer, sizeof(before)));
}
#endif

struct guarded_save {
    unsigned char before[16], data[SAVED_BYTES], after[16];
};
static void save_guarded(struct guarded_save *saved) {
    memset(saved, 0xa7, sizeof(*saved)); vqf_save(saved->data);
    for (size_t j = 0; j < sizeof(saved->before); j++)
        assert(saved->before[j] == 0xa7 && saved->after[j] == 0xa7);
    vqf_saved_header_t header; memcpy(&header, saved->data, sizeof(header));
    assert(header.magic == UINT32_C(0x32465156));
    assert(header.version == SAVED_VERSION && header.size == SAVED_BYTES);
}
static void assert_cold(void) {
    float q[4], identity[4] = {1, 0, 0, 0};
    vqf_get_quat(q); assert(!memcmp(q, identity, sizeof(q)));
    for (int j = 0; j < 3; j++) assert(state.bias[j] == 0);
#if CONFIG_SENSOR_VQF_OUTPUT_OBSERVER
    for (int j = 0; j < 6; j++) assert(!finite_double(output_observer.lp[j]));
#endif
#if CONFIG_SENSOR_VQF_FAST_REST_RECOVERY
    assert(stop_macro_gate.peak2 == 0 && stop_macro_gate.remaining == 0);
    assert(stop_macro_gate.base_samples == 0);
#endif
}
static void retained_stages(void) {
    for (int stage = 0; stage < 3; stage++) {
        init_fixture();
        int rows = stage == 0 ? 0 : stage == 1 ? 40 : 4800;
        for (int row = 0; row < rows; row++) rich_step(row);
        float public_before[4], native_before[4];
        vqf_get_quat(public_before); getQuat6D(&state, native_before);
#if CONFIG_SENSOR_VQF_OUTPUT_OBSERVER
        vqf_output_t output_before = output_observer;
        if (stage == 0) assert(!finite_double(output_before.lp[0]) && !finite_double(output_before.lp[1]));
        if (stage == 1) assert(!finite_double(output_before.lp[0]) && output_before.lp[1] == 10);
        if (stage == 2) {
            assert(finite_double(output_before.lp[0]));
            /* This would fail if load reconstructed output from native q. */
            assert(fabs(remainder(yaw(state.accQuat) - yaw(output_before.accQuat), 2 * M_PI)) > 1e-4);
        }
#endif
        struct guarded_save saved; save_guarded(&saved);
        vqf_init(GYR_DT, ACC_DT, ACC_DT); vqf_load(saved.data);
        float q[4]; vqf_get_quat(q); assert(!memcmp(q, public_before, sizeof(q)));
        getQuat6D(&state, q); assert(!memcmp(q, native_before, sizeof(q)));
#if CONFIG_SENSOR_VQF_OUTPUT_OBSERVER
        assert(!memcmp(output_before.lp, output_observer.lp, sizeof(output_before.lp)));
        assert(!memcmp(output_before.accQuat, output_observer.accQuat, sizeof(output_before.accQuat)));
        assert(!memcmp(output_before.b, output_observer.b, sizeof(output_before.b)));
        assert(!memcmp(output_before.a, output_observer.a, sizeof(output_before.a)));
#endif
        if (stage != 0) assert_public_and_linear();
        /* Header incompatibility must reject before interpreting the payload. */
        unsigned char bad[SAVED_BYTES]; memcpy(bad, saved.data, sizeof(bad));
        vqf_saved_header_t header; memcpy(&header, bad, sizeof(header));
        header.version = SAVED_VERSION == 2 ? 1 : 2;
        memcpy(bad, &header, sizeof(header)); vqf_load(bad); assert_cold();
        memcpy(bad, saved.data, sizeof(bad)); header.version = SAVED_VERSION; header.size--;
        memcpy(bad, &header, sizeof(header)); vqf_load(bad); assert_cold();
    }
}
#if CONFIG_SENSOR_VQF_OUTPUT_OBSERVER
static void store_double(unsigned char *where, uint64_t bits) { memcpy(where, &bits, sizeof(bits)); }
static void store_float(unsigned char *where, uint32_t bits) { memcpy(where, &bits, sizeof(bits)); }
static void retained_output_corruption(void) {
    /* Cold and running-mean NaNs are legitimate; all other payload words must
     * satisfy the corresponding state shape. Mutate actual saved fixtures.
     */
    for (int stage = 0; stage < 3; stage++) {
        init_fixture();
        for (int row = 0; row < (stage == 0 ? 0 : stage == 1 ? 40 : 4800); row++) rich_step(row);
        struct guarded_save saved; save_guarded(&saved);
        for (int mutation = 0; mutation < 22; mutation++) {
            unsigned char bad[SAVED_BYTES]; memcpy(bad, saved.data, sizeof(bad));
            unsigned char *lp = bad + 840, *q = lp + 48;
            if (mutation < 6) {
                /* Infinity is invalid even where a NaN startup marker belongs. */
                store_double(lp + 8 * mutation, UINT64_C(0x7ff0000000000000));
            } else if (mutation < 10) {
                store_float(q + 4 * (mutation - 6), UINT32_C(0x7fc00001));
            } else if (mutation < 14) {
                store_float(q + 4 * (mutation - 10), UINT32_C(0x7f800000));
            } else if (mutation == 14) {
                memset(q, 0, 16);
            } else if (mutation == 15) {
                for (int j = 0; j < 4; j++) store_float(q + 4 * j, UINT32_C(0x7f7fffff));
            } else if (stage == 1) {
                if (mutation == 16) store_double(lp + 8, 0); /* zero count */
                if (mutation == 17) { double count = -1; memcpy(lp + 8, &count, 8); }
                if (mutation == 18) { double count = ceil((double)VQF_OUTPUT_TAU_ACC / (double)ACC_DT) + 1; memcpy(lp + 8, &count, 8); }
                if (mutation == 19) store_double(lp + 8, UINT64_C(0x7ff8000000000001));
                if (mutation == 20) store_double(lp + 16, UINT64_C(0x7ff8000000000001));
                if (mutation == 21) store_double(lp + 40, 0); /* missing marker */
            } else if (stage == 0) {
                store_double(lp + 8 * (1 + (mutation - 16) % 5), 0);
            } else {
                store_double(lp + 8 * (mutation - 16), UINT64_C(0x7ff8000000000001));
            }
            vqf_load(bad); assert_cold();
            float a[3] = {.1f, -.2f, 1}; send_acc(a); assert_public_and_linear();
        }
    }
}
#endif

static void public_modes_and_external_retention(void) {
    init_fixture();
    host_mag_available = host_mag_enabled = true;
    for (int row = 0; row < 4800; row++) rich_step(row);
#if CONFIG_SENSOR_USE_MAGNETOMETER
    assert(fabsf(state.delta) > .01f); /* Generated by actual MAG updates. */
#if CONFIG_SENSOR_VQF_OUTPUT_OBSERVER
    float native[4], output[4], wrong[4];
    getQuat9D(&state, native); getOutputQuat6D(&output_observer, &state, output);
    float delta_q[4] = {cosf(.5f * state.delta), 0, 0, sinf(.5f * state.delta)};
    quaternion_product(delta_q, output, wrong);
    assert(q_difference(native, wrong) > 1e-4f);
    assert(fabs(remainder(yaw(state.accQuat) - yaw(output_observer.accQuat), 2 * M_PI)) > 1e-4);
    /* MAG-native publication must not suspend the independent fallback. */
    vqf_output_t shadow = output_observer, before = output_observer;
    for (int row = 4800; row < 5600; row++) {
        float g[3], a[3]; rich_vectors(row, g, a);
        vqf_update_gyro(g, GYR_DT);
        if (row % 4 == 0) {
            send_acc(a);
            for (int j = 0; j < 3; j++) a[j] *= CONST_EARTH_GRAVITY;
            updateOutputObserver(&shadow, &state, &coeffs, a);
            assert(!memcmp(shadow.lp, output_observer.lp, sizeof(shadow.lp)));
            assert(!memcmp(shadow.accQuat, output_observer.accQuat, sizeof(shadow.accQuat)));
            assert_public_and_linear();
        }
    }
    assert(memcmp(before.lp, output_observer.lp, sizeof(before.lp)));
#endif
#endif
    for (int available = 0; available < 2; available++) {
        for (int enabled = 0; enabled < 2; enabled++) {
            host_mag_available = available; host_mag_enabled = enabled;
            assert_public_and_linear();
            struct guarded_save saved; save_guarded(&saved);
            /* Availability/enabled belong to the sensor owner, never the
             * retained fusion frame. Restore while the owner chose another mode.
             */
            host_mag_available = !available; host_mag_enabled = !enabled;
            float expected[4]; expected_public(expected);
            vqf_init(GYR_DT, ACC_DT, ACC_DT); vqf_load(saved.data);
            assert(host_mag_available == !available && host_mag_enabled == !enabled);
            float actual[4]; vqf_get_quat(actual);
            assert(!memcmp(actual, expected, sizeof(actual)));
            assert_public_and_linear();
        }
    }
}

#if CONFIG_SENSOR_VQF_FAST_REST_RECOVERY
static bool recovery_probe(void) {
    /* Inspect the real observation leaf with the exact native RBE input. This
     * does not invent an admission or consume a completed-cycle observation.
     */
    assert(!cycle_bias_policy.locked);
    float e[3], w[3], old_e[3], old_w[3];
    for (int j = 0; j < 3; j++) {
        old_e[j] = e[j] = state.restLastGyrLp[j] - state.bias[j];
        old_w[j] = w[j] = coeffs.biasRestW;
    }
    vqf_rest_bias_observation(&params, &state, &coeffs, e, w);
    return memcmp(e, old_e, sizeof(e)) || memcmp(w, old_w, sizeof(w));
}
static void stop_admission_window(void) {
    /* Bracket the physical 5 dps boundary without depending on one rounded
     * representation of exactly 5 * DEG_TO_RAD squared.
     */
    init_fixture(); flat_step(4.99f, 1, false);
    assert(stop_macro_gate.peak2 == 0 && stop_macro_gate.remaining == 0);
    flat_step(5.01f, 1, false);
    assert(stop_macro_gate.peak2 < 0 && !recovery_probe());
    init_fixture();
    bool slow_gap_seen = false;
    for (int row = 0; row < 2400; row++) {
        flat_step(row < 1600 ? 0 : .3f, row, true);
        float gap = fabsf(rest_transition_policy.mean[2] - state.restLastGyrLp[2]);
        slow_gap_seen |= state.restDetected && gap > params.biasSigmaRest * DEG_TO_RAD;
        assert(stop_macro_gate.remaining == 0 && stop_macro_gate.peak2 == 0);
        assert(!recovery_probe());
    }
    assert(slow_gap_seen); /* A submacro slow-LP gap alone must not open STOP. */
    for (int row = 0; row < 200; row++) {
        flat_step(40, row, true);
        assert(stop_macro_gate.peak2 < 0 && !recovery_probe());
    }
    flat_step(0, 1, false);
    uint32_t window = stop_macro_gate.remaining;
    assert(window > 0 && window < 20000 && stop_macro_gate.base_samples > 0);
    assert(stop_macro_gate.peak2 > 0);
    bool admitted = false;
    for (uint32_t row = 0; row < window; row++) {
        uint32_t before = stop_macro_gate.remaining;
        flat_step(0, (int)row, true);
        assert(stop_macro_gate.remaining == before - 1);
        if (state.restDetected && recovery_probe()) admitted = true;
    }
    assert(admitted && stop_macro_gate.remaining == 0 && !recovery_probe());
    for (int row = 0; row < 800; row++) { flat_step(0, row, true); assert(!recovery_probe()); }
}
static void stop_countdown_cache_and_lifecycle(void) {
    init_fixture();
    /* No ACC/bias updates here: equal physical macro peaks really are equal. */
    for (int row = 0; row < 600; row++) flat_step(0, row, false);
    for (int row = 0; row < 200; row++) flat_step(20, row, false);
    flat_step(0, 1, false);
    uint32_t base = stop_macro_gate.base_samples;
    assert(base > 0 && stop_macro_gate.remaining == base);
    for (int row = 0; row < 20; row++) flat_step(20, row, false);
    assert(stop_macro_gate.peak2 < 0 && stop_macro_gate.base_samples == base);
    flat_step(0, 1, false);
    assert(stop_macro_gate.base_samples == base && stop_macro_gate.remaining == base);
    flat_step(40, 1, false);
    assert(stop_macro_gate.peak2 < 0 && stop_macro_gate.base_samples == 0);
    flat_step(0, 1, false);
    assert(stop_macro_gate.base_samples > base);

    typeof(stop_macro_gate) gate = stop_macro_gate;
    float zero[3] = {0}, bad[3] = {NAN, 0, 0}, inf[3] = {0, INFINITY, 0}, a[3] = {0, 0, 1};
    vqf_update_gyro(bad, 1000); vqf_update_gyro(inf, GYR_DT);
    vqf_update_accel(bad, ACC_DT); send_acc(zero); send_acc(a);
    vqf_request_motion_history_reset();
    assert(!memcmp(&gate, &stop_macro_gate, sizeof(gate)));
    vqf_update_accel(zero, 1000); /* Consumes notification, not a gyro LP tick. */
    assert(atomic_get(&cycle_reset_pending) == 0);
    assert(!memcmp(&gate, &stop_macro_gate, sizeof(gate)));
    vqf_discard_motion_history();
    float delta[3] = {.12f, -.08f, .06f};
#if CONFIG_SENSOR_VQF_OUTPUT_OBSERVER
    vqf_output_t output = output_observer;
#endif
    float q[4], after[4]; vqf_get_quat(q);
    vqf_rebase_gyro_bias(delta); vqf_get_quat(after);
    assert(!memcmp(q, after, sizeof(q)));
    assert(!memcmp(&gate, &stop_macro_gate, sizeof(gate)));
#if CONFIG_SENSOR_VQF_OUTPUT_OBSERVER
    assert(!memcmp(&output, &output_observer, sizeof(output)));
#endif
    /* Disable both legitimate native LP consumers through their native APIs. */
    setRestBiasEstEnabled(&params, &state, false);
    setMagDistRejectionEnabled(&params, &state, false);
    double lp[6]; memcpy(lp, state.restGyrLpState, sizeof(lp));
    for (int row = 0; row < 12; row++) vqf_update_gyro(delta, GYR_DT);
    assert(!memcmp(lp, state.restGyrLpState, sizeof(lp)));
    assert(!memcmp(&gate, &stop_macro_gate, sizeof(gate)));
    setRestBiasEstEnabled(&params, &state, true);
    vqf_update_gyro(delta, GYR_DT);
    assert(stop_macro_gate.remaining == gate.remaining - 1);

    /* Physical replacement max-seeds, invalidates the peak cache, and never
     * resets the independent output (unlike a matched coordinate transport).
     */
    gate = stop_macro_gate;
#if CONFIG_SENSOR_VQF_OUTPUT_OBSERVER
    output = output_observer;
#endif
    float replacement[3] = {0, 0, 4}; vqf_get_quat(q);
    vqf_set_gyro_bias(replacement); vqf_get_quat(after);
    assert(!memcmp(q, after, sizeof(q)));
    assert(stop_macro_gate.remaining >= gate.remaining);
    assert(fabsf(stop_macro_gate.peak2) >= fabsf(gate.peak2));
    assert(stop_macro_gate.base_samples == 0);
#if CONFIG_SENSOR_VQF_OUTPUT_OBSERVER
    assert(!memcmp(&output, &output_observer, sizeof(output)));
#endif

    /* A real warm retained LP seeds only finite homogeneous-history decay:
     * no macro certification/cache survives a load.
     */
    init_fixture();
    for (int row = 0; row < 600; row++) flat_step(0, row, false);
    for (int row = 0; row < 200; row++) flat_step(40, row, false);
    assert(stop_macro_gate.peak2 < 0);
    struct guarded_save saved; save_guarded(&saved);
    vqf_load(saved.data);
    assert(stop_macro_gate.peak2 > 0 && stop_macro_gate.remaining > 0);
    assert(stop_macro_gate.remaining < 20000 && stop_macro_gate.base_samples == 0);
    uint32_t count = stop_macro_gate.remaining;
    for (uint32_t row = 0; row < count; row++) {
        flat_step(0, 1, false); assert(stop_macro_gate.remaining == count - row - 1);
    }
    assert(!recovery_probe());
    /* A replacement on settled zero history independently creates a guard. */
    init_fixture();
    for (int row = 0; row < 600; row++) flat_step(0, row, false);
    vqf_set_gyro_bias(replacement);
    assert(stop_macro_gate.peak2 > 0 && stop_macro_gate.remaining > 0);
    assert(stop_macro_gate.base_samples == 0);
}
#endif

static void output_discard_preservation(void) {
#if CONFIG_SENSOR_VQF_OUTPUT_OBSERVER
    for (int action = 0; action < 5; action++) {
        init_fixture();
        for (int row = 0; row < 1200; row++) rich_step(row);
        vqf_output_t output = output_observer;
        float q[4], after[4], bad[3] = {NAN, 0, 0}; vqf_get_quat(q);
#if CONFIG_SENSOR_VQF_FAST_REST_RECOVERY
        typeof(stop_macro_gate) gate = stop_macro_gate;
#endif
        if (action == 0) vqf_discard_motion_history();
        if (action == 1) { vqf_request_motion_history_reset(); vqf_update_accel(bad, ACC_DT); }
        if (action == 2) vqf_update_gyro(bad, GYR_DT);
        if (action == 3) vqf_update_accel(bad, ACC_DT);
        if (action == 4) {
            float delta[3] = {.12f, -.08f, .06f}; vqf_rebase_gyro_bias(delta);
        }
        vqf_get_quat(after); assert(!memcmp(q, after, sizeof(q)));
        assert(!memcmp(&output, &output_observer, sizeof(output)));
#if CONFIG_SENSOR_VQF_FAST_REST_RECOVERY
        assert(!memcmp(&gate, &stop_macro_gate, sizeof(gate)));
#endif
    }
#endif
}
int main(void) {
    native_trajectory();
#if CONFIG_SENSOR_VQF_OUTPUT_OBSERVER
    fixed_output_oracle_and_cadence();
    retained_output_corruption();
#endif
    retained_stages();
    public_modes_and_external_retention();
    output_discard_preservation();
#if CONFIG_SENSOR_VQF_FAST_REST_RECOVERY
    stop_admission_window();
    stop_countdown_cache_and_lifecycle();
#endif
    puts("public output, retained lifecycle, MAG frame/LA and STOP gate/cache passed");
    return 0;
}
'''


def main():
    prefix = application.translation_unit().split('static void scalar_filter_native(void)', 1)[0]
    # Each STOP/MAG pair has an output-off/on control. These three pairs cover
    # independent STOP, both public MAG branches, and their complete composition.
    configurations = [(output, stop, mag) for stop, mag in ((0, 0), (1, 0), (1, 1))
                      for output in (0, 1)]
    with tempfile.TemporaryDirectory(prefix='vqf-output-stop-') as temporary:
        directory = Path(temporary)
        source = directory / 'output_stop.c'
        source.write_text(ATOMIC_LEAVES + prefix + TEST)
        compiler = ['rtk', 'proxy', *shlex.split(os.environ.get('CC') or 'cc')]
        inclination_source = directory / 'inclination.c'
        inclination_source.write_text(INCLINATION_TEST)
        inclination_core = Path(os.environ.get('VQF_INCLINATION_CORE', ROOT / 'vqf-c/src/vqf.c'))
        inclination_binary = directory / 'inclination'
        inclination_flags = [
            '-std=gnu11', '-O2', '-fsanitize=undefined', '-fno-sanitize-recover=all',
            '-DVQF_OUTPUT_OBSERVER', '-DVQF_OUTPUT_TAU_ACC=3.264f',
            '-I' + str(ROOT / 'vqf-c/src'),
        ]
        subprocess.run(compiler + inclination_flags + [str(inclination_source),
                       str(inclination_core), '-lm', '-o', str(inclination_binary)], check=True)
        print(f'Inclination core: {inclination_core}', flush=True)
        subprocess.run(['rtk', 'proxy', str(inclination_binary)], check=True)
        common = [
            '-std=gnu11', '-O2', '-fsanitize=undefined', '-fno-sanitize-recover=all',
            '-DCONFIG_VQF_ADAPTIVE_TAU_ACC=1', '-DCONFIG_SENSOR_VQF_ADAPTIVE_BIAS=1',
            '-DCONFIG_SENSOR_VQF_REST_TRANSITION=1', '-DCONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION=1',
            '-DVQF_ADAPTIVE_BIAS', '-DVQF_REST_TRANSITION', '-DVQF_CYCLE_BIAS_OBSERVATION',
            '-I' + str(ROOT / 'vqf-c/src'),
        ]
        hashes = {}
        for output, stop, mag in configurations:
            flags = common + [f'-DCONFIG_SENSOR_VQF_OUTPUT_OBSERVER={output}',
                              f'-DCONFIG_SENSOR_VQF_FAST_REST_RECOVERY={stop}',
                              f'-DCONFIG_SENSOR_USE_MAGNETOMETER={mag}']
            if output:
                flags.extend(['-DVQF_OUTPUT_OBSERVER', '-DVQF_OUTPUT_TAU_ACC=3.264f'])
            if stop:
                flags.append('-DVQF_FAST_REST_RECOVERY')
            name = f'{output}-{stop}-{mag}'
            core = directory / f'core-{name}.o'
            subprocess.run(compiler + flags + ['-c', str(ROOT / 'vqf-c/src/vqf.c'), '-o', str(core)], check=True)
            for fastmath in (False, True):
                wrapper = directory / f'app-{name}-{int(fastmath)}.o'
                binary = directory / f'check-{name}-{int(fastmath)}'
                # Keep core NaN startup arithmetic faithful; only the actual app
                # and its bit-finite retained rejection are stressed by fast-math.
                subprocess.run(compiler + flags + (['-ffast-math'] if fastmath else []) +
                               ['-c', str(source), '-o', str(wrapper)], check=True)
                subprocess.run(compiler + ['-fsanitize=undefined', '-fno-sanitize-recover=all',
                                           str(wrapper), str(core), '-lm', '-o', str(binary)], check=True)
                print(f'Output={output}, STOP={stop}, MAG={mag}, app fast-math={int(fastmath)}', flush=True)
                result = subprocess.run(['rtk', 'proxy', str(binary)], capture_output=True, text=True)
                print(result.stdout, end='', flush=True)
                if result.stderr:
                    print(result.stderr, end='', file=sys.stderr, flush=True)
                result.check_returncode()
                digest = next(line.split('=', 1)[1] for line in result.stdout.splitlines()
                              if line.startswith('native-q-bias='))
                key = stop, mag, fastmath
                if output:
                    assert digest == hashes[key], f'Output helper changed native q/bias: {key}'
                else:
                    hashes[key] = digest


if __name__ == '__main__':
    main()
