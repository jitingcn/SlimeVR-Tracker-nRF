#!/usr/bin/env python3
"""Actual application SIDE coordinate transport and discard regressions.

Standard-library-only driver; the unchanged run.py extractor supplies the real
application prefix, linked against the live native core. Only platform headers
and the single-owner RTOS atomic leaves are shimmed. No fabricated observer
state, callback-count oracle, parameter override, or frozen research dependency.
"""
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True
import run as application

ROOT = application.ROOT

ATOMIC_LEAVES = r'''
typedef int atomic_t;
static inline int atomic_set(atomic_t *p, int value) {
    int previous = *p; *p = value; return previous;
}
static inline int atomic_get(const atomic_t *p) { return *p; }
'''

TEST = r'''
#include <stdlib.h>

/* Explicit Float32 replay tolerances, not acceptance-threshold changes.
 * Compare q up to quaternion sign and compare bias after removing the imposed
 * input-coordinate origin. Immediate rebase continuity is stricter: exact q/P.
 */
#define QUATERNION_TOLERANCE_DEG .02
#define PHYSICAL_BIAS_TOLERANCE_DPS .001
#define TRANSPORT_TOLERANCE_RAD_S 3e-8f
#define GYR_DT .005f
#define ACC_DT .02f
#define TRACE_ROWS 20000

static const float step_delta[2][3] = {{.12f, -.08f, .06f}, {-.07f, .11f, -.04f}};
static double max_q_error_deg, max_bias_error_dps;

typedef typeof(cycle_bias_policy) cycle_t;
struct fixture {
    int row;
    double period;
    float origin[3];
};
struct sample_result {
    float q[4], bias[3];
};
static struct sample_result reference[TRACE_ROWS][2];

static void assert_cycle_observer_clear(void) {
    /* Invalid history cannot supply a partial cycle, confidence or RBE token.
     * Unused numeric caches and padding need not have a particular value.
     */
    assert(!cycle_bias_policy.started && !cycle_bias_policy.agree);
    assert(!cycle_bias_policy.locked && !cycle_bias_policy.info);
}
static void assert_cycle_clear(void) {
    assert_cycle_observer_clear();
    assert(cycle_bias_admission == 0);
}
static void assert_valid_history_unchanged(const cycle_t *before) {
    assert(cycle_bias_policy.started == before->started);
    assert(cycle_bias_policy.agree == before->agree);
    assert(cycle_bias_policy.locked == before->locked);
    assert(cycle_bias_policy.info == before->info);
    if (before->started) {
        assert(!memcmp(cycle_bias_policy.q, before->q, sizeof(before->q)));
        assert(cycle_bias_policy.elapsed == before->elapsed);
        assert(cycle_bias_policy.prevx == before->prevx);
        assert(!memcmp(cycle_bias_policy.b0, before->b0, sizeof(before->b0)));
    }
    if (before->agree || before->locked) {
        assert(cycle_bias_policy.period == before->period);
        assert(!memcmp(cycle_bias_policy.obs, before->obs, sizeof(before->obs)));
    }
}
static void assert_native_unchanged(vqf_state_t *before, float bias_tolerance) {
    float q[4], old_q[4];
    getQuat6D(&state, q); getQuat6D(before, old_q);
    assert(!memcmp(q, old_q, sizeof(q)));
    assert(!memcmp(state.gyrQuat, before->gyrQuat, sizeof(state.gyrQuat)));
    assert(!memcmp(state.accQuat, before->accQuat, sizeof(state.accQuat)));
    assert(!memcmp(state.biasP, before->biasP, sizeof(state.biasP)));
    for (int j = 0; j < 3; j++)
        assert(fabsf(state.bias[j] - before->bias[j]) <= bias_tolerance);
}
static void fixture_init(struct fixture *f, double period) {
    *f = (struct fixture){.period = period};
    /* A previous test must have let the real sample owner consume its request. */
    assert(atomic_get(&cycle_reset_pending) == 0);
    vqf_init(GYR_DT, ACC_DT, GYR_DT);
    assert_cycle_clear();
}
static double angle_at(const struct fixture *f, double t) {
    if (t <= 8.) return 0;
    return .15 * (sin(2 * M_PI * (t - 8.) / f->period + .137) - sin(.137));
}
static void fixture_vectors(const struct fixture *f, float g[3], float a[3]) {
    double t = (f->row + 1) * (double)GYR_DT;
    double angle = angle_at(f, t), previous = angle_at(f, t - GYR_DT);
    /* Exact interval-average body gyro for a one-axis closed rotation, with
     * fixed physical bias. ACC is body-frame gravity at the interval endpoint.
     * The only difference between paired runs is their imposed input origin.
     */
    g[0] = (float)((angle - previous) / GYR_DT) + .025f + f->origin[0];
    g[1] = -.015f + f->origin[1];
    g[2] = .020f + f->origin[2];
    a[0] = 0; a[1] = (float)sin(angle * M_PI / 180.);
    a[2] = (float)cos(angle * M_PI / 180.);
}
static void fixture_gyro(struct fixture *f, float a[3]) {
    float g[3]; fixture_vectors(f, g, a);
    vqf_update_gyro(g, GYR_DT);
    f->row++;
}
static void fixture_accel(const struct fixture *f, float a[3]) {
    if (f->row % 4 == 0) vqf_update_accel(a, ACC_DT);
}
static void assert_subsequent_accel_uses_native_history(float a[3]) {
    for (int i = 0; i < 2; i++) {
        vqf_update_accel(a, ACC_DT);
        assert_cycle_observer_clear();
        /* Fresh ACC must use only its current native selection, not the
         * completed-cycle token that existed before the discard.
         */
        assert(cycle_bias_admission ==
               (state.restDetected && params.restBiasEstEnabled ? 3u : 0u));
    }
}
static void transport(struct fixture *f, const float delta_dps[3]) {
    cycle_t expected = cycle_bias_policy;
    vqf_state_t before = state;
    bool rest_pending = rest_observation_pending;
    int queued = atomic_get(&cycle_reset_pending);
    unsigned admission = cycle_bias_admission;
    for (int j = 0; j < 3; j++) {
        float d = delta_dps[j] * DEG_TO_RAD;
        if (expected.started) expected.b0[j] += d;
        if (expected.agree || expected.locked) expected.obs[j] += d;
    }
    vqf_rebase_gyro_bias(delta_dps);
    /* Valid integrated orientation/phase, confidence and one-shot information
     * survive; paired replay below checks their subsequent q/physical-bias use.
     */
    for (int j = 0; j < 3; j++) {
        assert(fabsf(cycle_bias_policy.b0[j] - expected.b0[j]) <= TRANSPORT_TOLERANCE_RAD_S);
        assert(fabsf(cycle_bias_policy.obs[j] - expected.obs[j]) <= TRANSPORT_TOLERANCE_RAD_S);
        assert(fabsf(state.bias[j] - (before.bias[j] + delta_dps[j] * DEG_TO_RAD))
               <= TRANSPORT_TOLERANCE_RAD_S);
        expected.b0[j] = cycle_bias_policy.b0[j];
        expected.obs[j] = cycle_bias_policy.obs[j];
        before.bias[j] = state.bias[j];
        f->origin[j] += delta_dps[j];
    }
    assert_valid_history_unchanged(&expected);
    assert(cycle_bias_admission == admission);
    assert(atomic_get(&cycle_reset_pending) == queued);
    assert(rest_observation_pending == rest_pending);
    assert_native_unchanged(&before, 0);
}

enum target { UNSTARTED, PARTIAL, VALID_OBSERVATION, PENDING, UNUSED_BAD_OBSERVATION };
static bool at_target(enum target target) {
    switch (target) {
    case UNSTARTED:
        return cycle_bias_policy.axis != 3 && !cycle_bias_policy.started;
    case PARTIAL:
        return cycle_bias_policy.started && cycle_bias_policy.elapsed > .5f
            && cycle_bias_policy.agree && !cycle_bias_policy.locked;
    case VALID_OBSERVATION:
        return cycle_bias_policy.agree && !cycle_bias_policy.locked;
    case PENDING:
        return cycle_bias_policy.locked && cycle_bias_policy.info;
    case UNUSED_BAD_OBSERVATION:
        return cycle_bias_policy.started && cycle_bias_policy.period > 0
            && cycle_bias_policy.period < 2 && !cycle_bias_policy.agree && !cycle_bias_policy.locked
            && (fabsf(cycle_bias_policy.obs[0]) + fabsf(cycle_bias_policy.obs[1])
                + fabsf(cycle_bias_policy.obs[2])) > 1e-7f;
    }
    abort();
}
static void seek(struct fixture *f, enum target target, float a[3]) {
    for (int i = 0; i < TRACE_ROWS; i++) {
        fixture_gyro(f, a);
        if (at_target(target)) return; /* Observe before the real fresh ACC. */
        fixture_accel(f, a);
    }
    fprintf(stderr, "fixture did not reach target %d: row=%d started=%u agree=%u locked=%u info=%u period=%g\n",
            target, f->row, cycle_bias_policy.started, cycle_bias_policy.agree,
            cycle_bias_policy.locked, cycle_bias_policy.info, cycle_bias_policy.period);
    abort();
}
static void cold_and_unused_observations(void) {
    struct fixture f; float a[3];
    fixture_init(&f, 4.);
    transport(&f, step_delta[0]);
    assert_cycle_observer_clear();
    /* Axis selection/arming is real warm detector state, but b0/obs are still
     * cold until a crossing starts integration and a good observation exists.
     */
    seek(&f, UNSTARTED, a);
    float b0[3], obs[3];
    memcpy(b0, cycle_bias_policy.b0, sizeof(b0));
    memcpy(obs, cycle_bias_policy.obs, sizeof(obs));
    transport(&f, step_delta[1]);
    assert(!memcmp(b0, cycle_bias_policy.b0, sizeof(b0)));
    assert(!memcmp(obs, cycle_bias_policy.obs, sizeof(obs)));

    /* A genuinely solved but too-short cycle leaves a nonzero unused obs.
     * It is not a coordinate-bearing reference while agree==0 and !locked.
     */
    fixture_init(&f, 1.25);
    seek(&f, UNUSED_BAD_OBSERVATION, a);
    memcpy(obs, cycle_bias_policy.obs, sizeof(obs));
    transport(&f, step_delta[0]);
    assert(!memcmp(obs, cycle_bias_policy.obs, sizeof(obs)));

    fixture_init(&f, 4.);
    seek(&f, VALID_OBSERVATION, a);
    assert(cycle_bias_policy.started && !cycle_bias_policy.info);
    transport(&f, step_delta[0]);
}
static void pending_observation_once(void) {
    struct fixture f; float a[3], zero[3] = {0};
    fixture_init(&f, 4.); seek(&f, PENDING, a);
    transport(&f, step_delta[0]);
    cycle_t pending = cycle_bias_policy;
    vqf_update_accel(zero, ACC_DT);
    vqf_update_accel(zero, ACC_DT);
    assert_valid_history_unchanged(&pending);
    vqf_state_t before = state;
    vqf_update_accel(a, ACC_DT);
    assert(cycle_bias_policy.locked && !cycle_bias_policy.info);
    assert((cycle_bias_admission & 2u) != 0);
    assert(cycle_bias_policy.weight == coeffs.biasRestW * coeffs.accTs / cycle_bias_policy.period);
    assert(memcmp(before.biasP, state.biasP, sizeof(state.biasP)));
    float last_good[3]; memcpy(last_good, cycle_bias_policy.obs, sizeof(last_good));
    vqf_update_accel(a, ACC_DT); /* No new gyro/cycle: no second RBE token. */
    assert(!cycle_bias_policy.info && (cycle_bias_admission & 2u) == 0);
    assert(!memcmp(last_good, cycle_bias_policy.obs, sizeof(last_good)));
    /* Consumed information is not an invalid observation: last-good history
     * still provides the coordinate reference for the next agreement check.
     */
    transport(&f, step_delta[1]);
    assert(cycle_bias_policy.locked && !cycle_bias_policy.info);
    /* A later bad cycle must retain the established last-good reference too.
     * The shorter physical period is intentionally outside cycle acceptance,
     * not a test-only threshold override or fabricated policy-state seed.
     */
    f.period = 1.25;
    bool saw_bad_cycle = false;
    for (int i = 0; i < 1600; i++) {
        cycle_t previous = cycle_bias_policy;
        fixture_gyro(&f, a);
        if (cycle_bias_policy.locked && cycle_bias_policy.started
            && cycle_bias_policy.elapsed < previous.elapsed && !cycle_bias_policy.info) {
            assert(!memcmp(previous.obs, cycle_bias_policy.obs, sizeof(previous.obs)));
            transport(&f, step_delta[0]);
            assert(cycle_bias_policy.locked && !cycle_bias_policy.info);
            saw_bad_cycle = true;
            break;
        }
        fixture_accel(&f, a);
    }
    assert(saw_bad_cycle);
}
static void queued_reset_survives_transport(void) {
    for (int use_accel = 0; use_accel < 2; use_accel++) {
        struct fixture f; float a[3];
        fixture_init(&f, 4.); seek(&f, PENDING, a);
        cycle_t before = cycle_bias_policy;
        vqf_request_motion_history_reset();
        assert(atomic_get(&cycle_reset_pending) != 0);
        assert_valid_history_unchanged(&before);
        transport(&f, step_delta[0]);
        assert(cycle_bias_policy.info && atomic_get(&cycle_reset_pending) != 0);
        if (use_accel) {
            vqf_update_accel(a, ACC_DT);
            assert_cycle_observer_clear();
            /* Reset happened before this fresh ACC. Its real native update
             * legitimately publishes a new natural admission shadow; the
             * discarded cycle must not supply the current observation.
             */
            assert(cycle_bias_admission ==
                   (state.restDetected && params.restBiasEstEnabled ? 3u : 0u));
        } else {
            fixture_gyro(&f, a);
            /* The actual gyro may select an axis again, but cannot revive the
             * old completed observation or partial integration after reset.
             */
            assert_cycle_observer_clear();
        }
        assert(atomic_get(&cycle_reset_pending) == 0);
        assert_subsequent_accel_uses_native_history(a);
    }
}
static void discard_lifecycle_continuity(void) {
    for (int action = 0; action < 6; action++) {
        struct fixture f; float a[3];
        fixture_init(&f, 4.); seek(&f, PENDING, a);
        transport(&f, step_delta[0]);
        vqf_state_t before = state;
        switch (action) {
        case 0: {
            float current[3]; vqf_get_gyro_bias(current);
            vqf_set_gyro_bias(current); /* Set-current is a physical reset. */
            break;
        }
        case 1: {
            unsigned char saved[VQF_MEM_SIZE];
            vqf_save(saved); vqf_load(saved, GYR_DT, ACC_DT, GYR_DT);
            break;
        }
        case 2:
            /* Actual observer discard leaf used for sensitivity changes,
             * acquisition gaps and sleep; feed wiring is covered separately.
             */
            vqf_discard_motion_history();
            break;
        case 3: {
            float invalid[3] = {NAN, 0, 0};
            vqf_update_gyro(invalid, GYR_DT);
            break;
        }
        case 4: {
            float invalid[3] = {NAN, 0, 0};
            vqf_update_accel(invalid, ACC_DT);
            break;
        }
        case 5: {
            /* A measured reset is rebase THEN explicit residual replacement,
             * unlike pure reference transport. It must still destroy history.
             */
            transport(&f, step_delta[1]);
            float zero[3] = {0}; vqf_set_gyro_bias(zero);
            for (int j = 0; j < 3; j++) {
                assert(state.bias[j] == 0);
                before.bias[j] = 0;
            }
            break;
        }
        }
        assert_cycle_clear();
        assert_native_unchanged(&before, TRANSPORT_TOLERANCE_RAD_S);
        assert_subsequent_accel_uses_native_history(a);
    }
}
static double quaternion_error_deg(const float q[4], const float r[4]) {
    double minus = 0, plus = 0;
    for (int j = 0; j < 4; j++) {
        minus += (q[j] - (double)r[j]) * (q[j] - (double)r[j]);
        plus += (q[j] + (double)r[j]) * (q[j] + (double)r[j]);
    }
    /* Chord distance avoids an acos(dot) Float32 floor near identical q. */
    double chord = sqrt(fmin(minus, plus));
    return 4 * asin(fmin(1., chord / 2)) * 180 / M_PI;
}
static void record_or_compare(int pass, int row, int phase, const struct fixture *f) {
    struct sample_result result;
    getQuat6D(&state, result.q);
    for (int j = 0; j < 3; j++) result.bias[j] = state.bias[j] - f->origin[j] * DEG_TO_RAD;
    if (pass == 0) {
        reference[row][phase] = result;
        return;
    }
    double q_error = quaternion_error_deg(result.q, reference[row][phase].q);
    max_q_error_deg = fmax(max_q_error_deg, q_error);
    assert(q_error <= QUATERNION_TOLERANCE_DEG);
    for (int j = 0; j < 3; j++) {
        double bias_error = fabs(result.bias[j] - (double)reference[row][phase].bias[j]) * RAD_TO_DEG;
        max_bias_error_dps = fmax(max_bias_error_dps, bias_error);
        assert(bias_error <= PHYSICAL_BIAS_TOLERANCE_DPS);
    }
}
static void paired_actual_replay(void) {
    int partial_row = -1, pending_row = -1;
    unsigned reference_consumed = 0;
    for (int pass = 0; pass < 2; pass++) {
        struct fixture f; fixture_init(&f, 4.);
        unsigned consumed = 0;
        for (int row = 0; row < TRACE_ROWS; row++) {
            float a[3]; fixture_gyro(&f, a);
            if (!pass && partial_row < 0 && at_target(PARTIAL)) partial_row = row;
            if (!pass && pending_row < 0 && at_target(PENDING)) pending_row = row;
            if (pass && (row == partial_row || row == pending_row)) {
                assert(at_target(row == partial_row ? PARTIAL : PENDING));
                transport(&f, step_delta[row == partial_row ? 0 : 1]);
            }
            record_or_compare(pass, row, 0, &f);
            bool pending = cycle_bias_policy.info;
            fixture_accel(&f, a);
            if (pending && !cycle_bias_policy.info) {
                assert((cycle_bias_admission & 2u) != 0);
                consumed++;
            }
            record_or_compare(pass, row, 1, &f);
        }
        assert(partial_row >= 0 && pending_row > partial_row && consumed > 1);
        if (!pass) reference_consumed = consumed;
        else assert(consumed == reference_consumed);
    }
    printf("actual paired hot/pending replay: %u consumed cycles; max q %.9g deg / physical bias %.9g dps (limits %.9g / %.9g)\n",
           reference_consumed, max_q_error_deg, max_bias_error_dps,
           QUATERNION_TOLERANCE_DEG, PHYSICAL_BIAS_TOLERANCE_DPS);
}
static void unsupported_guard_crossing_exits(void) {
    struct fixture f; float a[3];
    fixture_init(&f, 4.); seek(&f, PENDING, a);
    /* Translation itself is legal and preserves the token. The next real raw
     * |g| exceeds the fixed 5 dps domain: discard is the correct exit, not a
     * reason to enlarge thresholds or demand unrestricted gauge invariance.
     */
    const float outside[3] = {0, 0, 5.5f};
    transport(&f, outside);
    assert(cycle_bias_policy.info);
    float obs[3], residual;
    assert(!vqf_cycle_observation(obs, &residual));
    fixture_gyro(&f, a);
    assert_cycle_clear();
    assert(atomic_get(&cycle_reset_pending) == 0);
}
int main(void) {
    cold_and_unused_observations();
    pending_observation_once();
    queued_reset_survives_transport();
    discard_lifecycle_continuity();
    paired_actual_replay();
    unsupported_guard_crossing_exits();
    puts("actual application SIDE transport: cold/unused/valid history, partial/pending replay, one-shot information, queued reset, discard continuity and hard-domain exit passed");
    return 0;
}
'''


def main():
    # Reuse the existing extraction unchanged, excluding its independent test
    # bodies/main. All application observers and callbacks remain real C.
    prefix = application.translation_unit().split('static void scalar_filter_native(void)', 1)[0]
    with tempfile.TemporaryDirectory(prefix='vqf-cycle-rebase-') as temporary:
        directory = Path(temporary)
        source = directory / 'cycle_rebase.c'
        source.write_text(ATOMIC_LEAVES + prefix + TEST)
        compiler = ['rtk', 'proxy', *shlex.split(os.environ.get('CC') or 'cc')]
        common = [
            '-std=gnu11', '-O2', '-fsanitize=undefined', '-fno-sanitize-recover=all',
            '-DCONFIG_VQF_ADAPTIVE_TAU_ACC=1', '-DCONFIG_SENSOR_VQF_ADAPTIVE_BIAS=1',
            '-DCONFIG_SENSOR_VQF_REST_TRANSITION=1', '-DCONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION=1',
            '-DVQF_ADAPTIVE_BIAS', '-DVQF_REST_TRANSITION', '-DVQF_CYCLE_BIAS_OBSERVATION',
            '-I' + str(ROOT / 'vqf-c/src'),
        ]
        for stop in (0, 1):
            flags = common + [f'-DCONFIG_SENSOR_VQF_FAST_REST_RECOVERY={stop}']
            if stop:
                flags.append('-DVQF_FAST_REST_RECOVERY')
            core = directory / f'core-{stop}.o'
            subprocess.run(compiler + flags + ['-c', str(ROOT / 'vqf-c/src/vqf.c'), '-o', str(core)], check=True)
            for fastmath in (False, True):
                wrapper = directory / f'application-{stop}-{int(fastmath)}.o'
                binary = directory / f'check-{stop}-{int(fastmath)}'
                # Keep the core's actual NaN startup-marker arithmetic intact;
                # stress application-only fast-math in a separate object.
                subprocess.run(compiler + flags + (['-ffast-math'] if fastmath else []) +
                               ['-c', str(source), '-o', str(wrapper)], check=True)
                subprocess.run(compiler + ['-fsanitize=undefined', '-fno-sanitize-recover=all',
                                           str(wrapper), str(core), '-lm', '-o', str(binary)], check=True)
                print(f'SIDE rebase: STOP={stop}, application fast-math={int(fastmath)}, core original arithmetic', flush=True)
                subprocess.run(['rtk', 'proxy', str(binary)], check=True)


if __name__ == '__main__':
    main()
