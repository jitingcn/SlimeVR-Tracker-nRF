/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include "util.h"

#include <math.h>
#include <zephyr/sys/printk.h>
#if defined(CONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION)
#include <zephyr/sys/atomic.h>
#endif
#if defined(CONFIG_VQF_BENCH)
#include <zephyr/kernel.h>
#include "thread_priority.h"
#endif

#if defined(CONFIG_VQF_BENCH) && defined(CONFIG_CPU_CORTEX_M_HAS_DWT)
#include <cmsis_core.h>
#endif

#include "../src/vqf.h" // conflicting with vqf.h in local path

#include "../vqf/vqf.h" // conflicting with vqf.h in vqf-c

#include "retained.h" // for BUILD_ASSERT on fusion_data size

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.01745329251994329577f /* (float)(M_PI / 180.0) */
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.29577951308232087680f /* (float)(180.0 / M_PI) */
#endif

static bool rest_observation_pending;

#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
/* Causal target: 2 * 4^(A-G), with A/G normalized acceleration deviation
 * and bias-corrected angular speed. No rest override or temperature input.
 * Log-tau responds faster upward than downward; gyro attitude propagation
 * remains immediate. See the offline response-gating experiment. */
#define ADAPTIVE_TAU_MIN 0.5f
#define ADAPTIVE_TAU_MAX 8.0f
#define ADAPTIVE_FEATURE_TIME 0.5f
#define ADAPTIVE_GYR_SCALE 20.0f /* deg/s */
#define ADAPTIVE_ACC_SCALE 2.0f /* m/s² */
#define ADAPTIVE_TAU_UP_TIME 0.5f
#define ADAPTIVE_TAU_DOWN_TIME 1.5f
#define ADAPTIVE_TAU_INTERVAL 0.5f
#endif


static vqf_params_t params;
static vqf_state_t state;
static vqf_coeffs_t coeffs;
#if IS_ENABLED(CONFIG_SENSOR_VQF_OUTPUT_OBSERVER)
static vqf_output_t output_observer;
BUILD_ASSERT(sizeof(output_observer) == 104, "Output observer runtime ABI changed");
#define VQF_OUTPUT_SAVED_SIZE 64u
#else
#define VQF_OUTPUT_SAVED_SIZE 0u
#endif

static ALWAYS_INLINE bool vqf_float_finite(float x);
#if IS_ENABLED(CONFIG_SENSOR_VQF_OUTPUT_OBSERVER) || IS_ENABLED(CONFIG_SENSOR_VQF_FAST_REST_RECOVERY)
static bool vqf_double_finite(double x)
{
	uint64_t bits;
	memcpy(&bits, &x, sizeof(bits));
	return (bits & UINT64_C(0x7ff0000000000000)) != UINT64_C(0x7ff0000000000000);
}
#endif
#if IS_ENABLED(CONFIG_SENSOR_VQF_OUTPUT_OBSERVER)
static bool vqf_output_load(const uint8_t *payload)
{
	/* Rebuild only coefficients; retain the actual output inclination/yaw gauge.
	 * Native accQuat cannot reconstruct that independent gauge. */
	initOutputObserver(&output_observer, &state, &coeffs);
	memcpy(output_observer.lp, payload, sizeof(output_observer.lp));
	memcpy(output_observer.accQuat, payload + sizeof(output_observer.lp),
	       sizeof(output_observer.accQuat));
	uint64_t bits;
	memcpy(&bits, &output_observer.lp[0], sizeof(bits));
	bool startup = (bits & UINT64_C(0x7fffffffffffffff)) > UINT64_C(0x7ff0000000000000);
	if (startup) {
		memcpy(&bits, &output_observer.lp[1], sizeof(bits));
		bool cold = (bits & UINT64_C(0x7fffffffffffffff)) > UINT64_C(0x7ff0000000000000);
		for (int j = cold ? 1 : 5; j < 6; j++) {
			memcpy(&bits, &output_observer.lp[j], sizeof(bits));
			if ((bits & UINT64_C(0x7fffffffffffffff)) <= UINT64_C(0x7ff0000000000000)) {
				return false;
			}
		}
		if (!cold) {
			if (!vqf_double_finite(output_observer.lp[1]) || output_observer.lp[1] <= 0
			    || output_observer.lp[1] * (vqf_double_t)coeffs.accTs >=
				       (vqf_double_t)VQF_OUTPUT_TAU_ACC) {
				return false;
			}
			for (int j = 2; j < 5; j++) {
				if (!vqf_double_finite(output_observer.lp[j])) {
					return false;
				}
			}
		}
	} else {
		for (int j = 0; j < 6; j++) {
			if (!vqf_double_finite(output_observer.lp[j])) {
				return false;
			}
		}
	}
	float norm2 = 0;
	for (int j = 0; j < 4; j++) {
		if (!vqf_float_finite(output_observer.accQuat[j])) {
			return false;
		}
		norm2 += output_observer.accQuat[j] * output_observer.accQuat[j];
	}
	return vqf_float_finite(norm2) && norm2 > 0;
}
#endif

#if IS_ENABLED(CONFIG_SENSOR_VQF_FAST_REST_RECOVERY)
/* Peak is the MAX over the native gyro-LP epoch; negative marks macro motion.
 * base_samples caches only ceil(tail)+1 for that peak/model/tolerance. Bootstrap
 * is fresh at each exit. Init/load/physical bias replacement invalidate it.
 * The app has no runtime gyro-LP coefficient or biasSigmaRest setters. */
static struct {
	float peak2;
	uint32_t remaining, base_samples;
} stop_macro_gate;
BUILD_ASSERT(sizeof(stop_macro_gate) == 12, "Finite STOP gate/cache ABI changed");

static void stop_macro_reset(void)
{
	memset(&stop_macro_gate, 0, sizeof(stop_macro_gate));
}

static bool stop_macro_model(double *C, double *radius, double *co, double *si)
{
	const double *a = coeffs.restGyrLpA, *b = coeffs.restGyrLpB;
	if (!(a[1] > 0 && a[1] < 1)) {
		return false;
	}
	*radius = sqrt(a[1]);
	*co = -a[0] / (2 * (*radius));
	double si2 = 1 - (*co) * (*co);
	if (!(si2 > 0)) {
		return false;
	}
	*si = sqrt(si2);
	double h1 = b[1] - a[0] * b[0], h2 = b[2] - a[0] * h1 - a[1] * b[0];
	double v = (h2 / (*radius) - h1 * (*co)) / (*si);
	*C = sqrt(h1 * h1 + v * v) / (1 - (*radius));
	double y0 = 1 - b[0], y1 = -a[0] * y0 + b[2] - a[1];
	double v0 = (y1 / (*radius) - y0 * (*co)) / (*si);
	*C = fmax(*C, sqrt(y0 * y0 + v0 * v0));
	return vqf_double_finite(*C);
}

static double stop_macro_bootstrap(void)
{
	if (vqf_double_finite(state.restGyrLpState[0])) {
		return 0;
	}
	double count = vqf_double_finite(state.restGyrLpState[1]) ? state.restGyrLpState[1] : 0;
	return ceil(fmax(0, (double)params.restFilterTau / (double)coeffs.gyrTs - count));
}

static uint32_t stop_macro_samples(double amplitude, double radius, double bootstrap)
{
	double tolerance = (double)params.biasSigmaRest * (double)DEG_TO_RAD;
	if (!(amplitude > tolerance && tolerance > 0 && radius > 0 && radius < 1)) {
		return 0;
	}
	double n = ceil(log(amplitude / tolerance) / -log(radius)) + bootstrap + 1;
	return vqf_double_finite(n) && n >= 0 && n <= UINT32_MAX ? (uint32_t)n : 0;
}

static void stop_macro_restore(bool preserve)
{
	/* A load seeds homogeneous-history decay, not a certified macro. Physical
	 * bias replacement max-seeds the existing guard. Matched rebase does neither. */
	stop_macro_gate.base_samples = 0;
	float oldpeak = preserve ? fabsf(stop_macro_gate.peak2) : 0;
	uint32_t oldremaining = preserve ? stop_macro_gate.remaining : 0;
	bool ongoing = preserve && stop_macro_gate.peak2 < 0;
	double C = 0, radius = 0, co = 0, si = 0, E2 = 0;
	if (!stop_macro_model(&C, &radius, &co, &si)) {
		if (!preserve) {
			stop_macro_reset();
		}
		return;
	}
	double bootstrap = stop_macro_bootstrap();
	for (int j = 0; j < 3; j++) {
		const vqf_double_t bias = (vqf_double_t)state.bias[j];
		double u, v;
		if (bootstrap > 0) {
			double count = vqf_double_finite(state.restGyrLpState[1]) ? state.restGyrLpState[1] : 0;
			u = count > 0 ? state.restGyrLpState[2 + j] / count - bias : 0;
			v = 0;
		} else {
			u = state.restGyrLpState[2 * j] - bias * (1 - coeffs.restGyrLpB[0]);
			double z = state.restGyrLpState[2 * j + 1]
				- bias * (coeffs.restGyrLpB[2] - coeffs.restGyrLpA[1]);
			v = ((-coeffs.restGyrLpA[0] * u + z) / radius - u * co) / si;
		}
		E2 += u * u + v * v;
	}
	double E = sqrt(E2), proxy = bootstrap > 0 ? E : E / C;
	if (!vqf_double_finite(proxy) || !vqf_float_finite((float)(proxy * proxy))) {
		if (!preserve) {
			stop_macro_reset();
		}
		return;
	}
	float peak = fmaxf(oldpeak, (float)(proxy * proxy));
	stop_macro_gate.peak2 = ongoing ? -peak : peak;
	uint32_t n = stop_macro_samples(bootstrap > 0 ? E * C : E, radius, bootstrap);
	stop_macro_gate.remaining = n > oldremaining ? n : oldremaining;
}

static void stop_macro_step(const float g[3])
{
	/* Count only actual native rest-GYR LP updates, never wallclock/suspend. */
	if (!(params.restBiasEstEnabled || params.magDistRejectionEnabled)) {
		return;
	}
	float rate2 = 0;
	for (int j = 0; j < 3; j++) {
		float v = g[j] - state.bias[j];
		rate2 += v * v;
	}
	if (!vqf_float_finite(rate2)) {
		return;
	}
	if (rate2 > 25 * DEG_TO_RAD * DEG_TO_RAD) {
		float peak = fmaxf(fabsf(stop_macro_gate.peak2), rate2);
		if (peak > fabsf(stop_macro_gate.peak2)) {
			stop_macro_gate.base_samples = 0;
		}
		stop_macro_gate.peak2 = -peak;
		return;
	}
	if (stop_macro_gate.peak2 < 0) {
		stop_macro_gate.peak2 = -stop_macro_gate.peak2;
		double C = 0, radius = 0, co = 0, si = 0, bootstrap = stop_macro_bootstrap();
		if (!stop_macro_gate.base_samples && stop_macro_model(&C, &radius, &co, &si)) {
			stop_macro_gate.base_samples = stop_macro_samples(
				sqrt((double)stop_macro_gate.peak2) * C, radius, 0);
		}
		uint32_t n = 0;
		if (stop_macro_gate.base_samples) {
			double total = (double)stop_macro_gate.base_samples + bootstrap;
			if (vqf_double_finite(total) && total >= 0 && total <= UINT32_MAX) {
				n = (uint32_t)total;
			}
		}
		if (n > stop_macro_gate.remaining) {
			stop_macro_gate.remaining = n;
		}
	} else if (stop_macro_gate.remaining) {
		stop_macro_gate.remaining--;
	}
}
#endif
#if IS_ENABLED(CONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION)
/* Frozen q-only completed-cycle observer. No history, solver, or retained data. */
static struct {
	float q[4], b0[3];
	float obs[3], acc[3];
	float elapsed, compensation, prevx, peak, oldpeak, period, obsage, weight;
	uint8_t axis:2, armed:1, started:1, agree:2, locked:1, info:1;
} cycle_bias_policy;
/* Bit 0: original natural RBE; bit 1: actual RBE. Thermal uses only bit 0. */
static unsigned int cycle_bias_admission;
static atomic_t cycle_reset_pending;
BUILD_ASSERT(sizeof(cycle_bias_policy) == 88, "Cycle observer layout changed");
BUILD_ASSERT(sizeof(cycle_bias_admission) == 4, "Cycle admission layout changed");
BUILD_ASSERT(sizeof(cycle_reset_pending) == 4, "Cycle reset notification layout changed");
void vqf_discard_motion_history(void);
#endif
/* Raw state layouts are not interchangeable across firmware revisions. */
typedef struct {
	uint32_t magic;
	uint16_t version;
	uint16_t size;
} vqf_saved_header_t;
#define VQF_SAVED_MAGIC 0x32465156u /* "VQF2" */
#if IS_ENABLED(CONFIG_SENSOR_VQF_OUTPUT_OBSERVER)
#define VQF_SAVED_VERSION 2u
#else
#define VQF_SAVED_VERSION 1u
#endif
#define VQF_MEM_SIZE (sizeof(vqf_saved_header_t) + sizeof(vqf_state_t) + sizeof(vqf_coeffs_t) + VQF_OUTPUT_SAVED_SIZE)
BUILD_ASSERT(sizeof(vqf_saved_header_t) == 8, "VQF retained header ABI changed");
BUILD_ASSERT(VQF_MEM_SIZE == 840 + VQF_OUTPUT_SAVED_SIZE, "VQF retained ABI changed");

static float last_a[3] = {0};
#if defined(CONFIG_VQF_BENCH)
static volatile float vqf_bench_sink;
static vqf_params_t vqf_bench_params;
static vqf_params_t vqf_bench_warm_params;
static vqf_state_t vqf_bench_state;
static vqf_state_t vqf_bench_warm_state;
static vqf_coeffs_t vqf_bench_coeffs;
static vqf_coeffs_t vqf_bench_warm_coeffs;
static float vqf_bench_quat[4];

static const vqf_real_t vqf_bench_gyr_samples[][3] = {
	{0.12f, -0.34f, 0.56f},
	{-1.10f, 0.45f, 0.08f},
	{0.78f, 0.11f, -0.29f},
	{-0.05f, 0.92f, 0.37f},
};

static const vqf_real_t vqf_bench_acc_samples[][3] = {
	{0.30f, 0.10f, 9.70f},
	{-0.25f, 0.45f, 9.63f},
	{0.60f, -0.15f, 9.55f},
	{-0.40f, -0.20f, 9.81f},
};

#if CONFIG_SENSOR_USE_MAGNETOMETER
static const vqf_real_t vqf_bench_mag_samples[][3] = {
	{0.32f, 0.05f, -0.41f},
	{0.28f, 0.10f, -0.39f},
	{0.35f, -0.02f, -0.43f},
	{0.30f, 0.08f, -0.40f},
};
#endif

static const size_t vqf_bench_sample_count = sizeof(vqf_bench_gyr_samples) / sizeof(vqf_bench_gyr_samples[0]);
#endif

#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
static float adaptive_gyr_signal, adaptive_acc_signal;
static float adaptive_log_tau, adaptive_clock;
static float adaptive_gyr_alpha, adaptive_acc_alpha;
static float adaptive_up_alpha, adaptive_down_alpha;

static void vqf_reset_adaptive_tau(void)
{
	adaptive_gyr_signal = 0.0f;
	adaptive_acc_signal = 0.0f;
	adaptive_log_tau = logf(params.tauAcc);
	adaptive_clock = 0.0f;
	/* Periods are fixed by init/load, just like the underlying VQF updates.
	 * Avoid recomputing exponentials at the IMU sample rate. */
	adaptive_gyr_alpha = 1.0f - expf(-coeffs.gyrTs / ADAPTIVE_FEATURE_TIME);
	adaptive_acc_alpha = 1.0f - expf(-coeffs.accTs / ADAPTIVE_FEATURE_TIME);
	adaptive_up_alpha = 1.0f - expf(-coeffs.accTs / ADAPTIVE_TAU_UP_TIME);
	adaptive_down_alpha = 1.0f - expf(-coeffs.accTs / ADAPTIVE_TAU_DOWN_TIME);
}
#endif

#if IS_ENABLED(CONFIG_SENSOR_VQF_REST_TRANSITION)
/* Exact offline P6: prior 0.25 s EW moments can admit a bounded slow-LP
 * transition for fusion bias only. Original hard rest remains independent. */
static struct {
	float mean[3], variance, alpha, age, hard_timer;
	bool hard_rest;
} rest_transition_policy;

static void vqf_reset_rest_transition(void)
{
	memset(&rest_transition_policy, 0, sizeof(rest_transition_policy));
	rest_transition_policy.alpha = 1 - expf(-coeffs.gyrTs / .25f);
	/* Eligibility depends on transient moments; never revive a saved relaxed
	 * admission after load or an explicit physical bias replacement. */
	state.restT = 0;
	state.restDetected = false;
}

bool vqf_rest_transition_failure(vqf_params_t *p, vqf_state_t *s,
				 const vqf_real_t gyr[3], vqf_real_t Ts, bool original)
{
	const float rad = .017453292519943295f;
	float threshold = p->restThGyr * rad;
	if (s != &state) {
		return original; /* Benchmark filters must not mutate live policy. */
	}
	float clip = p->biasClip * rad;
	if (original || fabsf(s->restLastGyrLp[0]) > clip
	    || fabsf(s->restLastGyrLp[1]) > clip || fabsf(s->restLastGyrLp[2]) > clip) {
		rest_transition_policy.hard_timer = 0;
		rest_transition_policy.hard_rest = false;
	}
	bool failure = original;
	/* Qualify against moments from BEFORE the current gyro sample. */
	if (rest_transition_policy.age >= 1.0f
	    && rest_transition_policy.variance < (.08f * rad) * (.08f * rad)) {
		float fast_residual = 0, gap = 0;
		for (int j = 0; j < 3; j++) {
			float d = gyr[j] - rest_transition_policy.mean[j];
			fast_residual += d * d;
			d = rest_transition_policy.mean[j] - s->restLastGyrLp[j];
			gap += d * d;
		}
		float cap = threshold + .2f * rad;
		if (original && s->restLastSquaredDeviations[0] < cap * cap
		    && fast_residual < threshold * threshold && gap > (.1f * rad) * (.1f * rad)) {
			failure = false;
		}
	}
	/* Advance one stable EW central-variance recurrence per gyro, only after
	 * qualification. Keep the native slow-LP RBE observation unchanged. */
	if (rest_transition_policy.age == 0) {
		memcpy(rest_transition_policy.mean, gyr, sizeof(rest_transition_policy.mean));
	} else {
		float delta2 = 0;
		for (int j = 0; j < 3; j++) {
			float d = gyr[j] - rest_transition_policy.mean[j];
			delta2 += d * d;
			rest_transition_policy.mean[j] += rest_transition_policy.alpha * d;
		}
		rest_transition_policy.variance = (1 - rest_transition_policy.alpha)
			* (rest_transition_policy.variance + rest_transition_policy.alpha * delta2);
	}
	rest_transition_policy.age += Ts;
	return failure;
}

void vqf_rest_transition_hard_acc(vqf_params_t *p, vqf_state_t *s, vqf_real_t Ts)
{
	if (s != &state) {
		return;
	}
	/* Called exactly once with the CURRENT nonzero acceleration residual. */
	if (s->restLastSquaredDeviations[1] >= p->restThAcc * p->restThAcc) {
		rest_transition_policy.hard_timer = 0;
		rest_transition_policy.hard_rest = false;
	} else {
		rest_transition_policy.hard_timer += Ts;
		if (rest_transition_policy.hard_timer >= p->restMinT) {
			rest_transition_policy.hard_rest = true;
		}
	}
}
#endif /* CONFIG_SENSOR_VQF_REST_TRANSITION */

static bool vqf_hard_rest_detected(void)
{
#if IS_ENABLED(CONFIG_SENSOR_VQF_REST_TRANSITION)
	return rest_transition_policy.hard_rest;
#else
	return state.restDetected;
#endif
}

#if IS_ENABLED(CONFIG_SENSOR_VQF_ADAPTIVE_BIAS)
/* Strategy A: motion-bias mode 661 with gradual 16-second rest recency, and
 * conservative thermal bounds with causal noise-whitened heldout scoring.
 * Statistics stay out of retained RAM. */
static struct {
	float gravity[3], alpha, rest_retention, excitation, log_scale, since_rest;
	bool started, rest_update;
} motion_bias_policy;

static struct {
	bool temperature_available, have, in_rest, last_have, trial_have;
	unsigned int trial_tests;
	float temperature, hold, anchor, offset[3], beta[3], gain;
	float last_t, last_g[3], trial_beta[3], min_t, max_t, temperature_alpha;
	double age, block_age, block_n, block_t, block_g[3], block_gg[3];
	double n, mean_t, mean_g[3], tt, tg[3];
	double noise_n, noise[3], trial_weight[3], trial_const[3], trial_model[3];
} predictive_bias_policy;

/* Only the thermal training regressor gets the additional native Butterworth
 * stage. Prediction and anchoring keep their existing fast 1-second signal. */
static struct {
	vqf_double_t b[3], a[2], filter_state[3];
	float temperature, tau;
	bool decided, matched;
} training_temperature_policy;

static ALWAYS_INLINE bool vqf_double_nan(vqf_double_t x)
{
	uint64_t bits;
	BUILD_ASSERT(sizeof(x) == sizeof(bits), "VQF native filter state must be double");
	memcpy(&bits, &x, sizeof(bits));
	return (bits & UINT64_C(0x7FFFFFFFFFFFFFFF)) > UINT64_C(0x7FF0000000000000);
}

static void vqf_training_temperature_decide(void)
{
	if (!training_temperature_policy.decided) {
		/* A policy reset does not reset retained/native gyro-LP history.
		 * Both startup slots must still be NaN: slot 0 alone also marks
		 * an already-started running mean. Decide before the first gyro. */
		training_temperature_policy.matched = predictive_bias_policy.have
			&& vqf_double_nan(state.restGyrLpState[0])
			&& vqf_double_nan(state.restGyrLpState[1]);
		training_temperature_policy.decided = true;
	}
}

static void vqf_training_temperature_update(void)
{
	if (training_temperature_policy.matched) {
		training_temperature_policy.temperature = updateScalarFilter(
			predictive_bias_policy.temperature, training_temperature_policy.tau, coeffs.gyrTs,
			training_temperature_policy.b, training_temperature_policy.a,
			training_temperature_policy.filter_state);
	}
}

static float vqf_training_temperature(void)
{
	/* Late temperature or warm native history cannot be reconstructed.
	 * Keep the original fast-T training for that whole policy lifetime. */
	return training_temperature_policy.matched ? training_temperature_policy.temperature
		: predictive_bias_policy.temperature;
}

static void vqf_predictive_clear_block(void)
{
	predictive_bias_policy.block_age = 0;
	predictive_bias_policy.block_n = 0;
	predictive_bias_policy.block_t = 0;
	memset(predictive_bias_policy.block_g, 0, sizeof(predictive_bias_policy.block_g));
	memset(predictive_bias_policy.block_gg, 0, sizeof(predictive_bias_policy.block_gg));
}

static void vqf_predictive_clear_fit(void)
{
	predictive_bias_policy.n = 0;
	predictive_bias_policy.mean_t = 0;
	predictive_bias_policy.tt = 0;
	memset(predictive_bias_policy.mean_g, 0, sizeof(predictive_bias_policy.mean_g));
	memset(predictive_bias_policy.tg, 0, sizeof(predictive_bias_policy.tg));
	predictive_bias_policy.min_t = INFINITY;
	predictive_bias_policy.max_t = -INFINITY;
}

static void vqf_predictive_clear_trial(void)
{
	predictive_bias_policy.trial_have = false;
	predictive_bias_policy.trial_tests = 0;
	memset(predictive_bias_policy.trial_const, 0, sizeof(predictive_bias_policy.trial_const));
	memset(predictive_bias_policy.trial_model, 0, sizeof(predictive_bias_policy.trial_model));
}

static void vqf_reset_bias_policy(void)
{
	memset(&motion_bias_policy, 0, sizeof(motion_bias_policy));
	memset(&predictive_bias_policy, 0, sizeof(predictive_bias_policy));
	memset(&training_temperature_policy, 0, sizeof(training_temperature_policy));
	training_temperature_policy.tau = params.restFilterTau * .45f;
	initScalarFilter(training_temperature_policy.tau, coeffs.gyrTs,
			 training_temperature_policy.b, training_temperature_policy.a,
			 training_temperature_policy.filter_state);
	motion_bias_policy.alpha = 1 - expf(-coeffs.accTs / .5f);
	motion_bias_policy.rest_retention = expf(-coeffs.accTs / 16.f);
	predictive_bias_policy.temperature_alpha = 1 - expf(-coeffs.gyrTs);
	vqf_predictive_clear_fit();
#if IS_ENABLED(CONFIG_SENSOR_VQF_REST_TRANSITION)
	vqf_reset_rest_transition();
#endif
#if IS_ENABLED(CONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION)
	vqf_discard_motion_history();
#endif
}

void vqf_bias_policy_features(vqf_state_t *s, vqf_coeffs_t *c, const vqf_real_t earth[3])
{
	ARG_UNUSED(c);
	ARG_UNUSED(earth); /* Mode 661 does not use the experimental accel-gap gate. */
	if (s != &state) {
		return; /* The optional benchmark has its own native filter state. */
	}
	float q[4];
	getQuat6D(s, q);
	float gravity[3] = {
		2 * (q[1] * q[3] - q[0] * q[2]),
		2 * (q[0] * q[1] + q[2] * q[3]),
		q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3],
	};
	if (!motion_bias_policy.started) {
		memcpy(motion_bias_policy.gravity, gravity, sizeof(gravity));
		motion_bias_policy.started = true;
	}
	float norm2 = 0;
	for (int i = 0; i < 3; i++) {
		motion_bias_policy.gravity[i] += motion_bias_policy.alpha * (gravity[i] - motion_bias_policy.gravity[i]);
		norm2 += motion_bias_policy.gravity[i] * motion_bias_policy.gravity[i];
	}
	float variance = fmaxf(0, 1 - norm2);
	motion_bias_policy.excitation = variance / (variance + .00625f);
}

void vqf_bias_policy_weights(vqf_params_t *p, vqf_state_t *s, vqf_coeffs_t *c,
			    const vqf_real_t R[9], const vqf_real_t e[3], vqf_real_t w[3])
{
	ARG_UNUSED(R);
	ARG_UNUSED(e);
	if (s != &state) {
		return;
	}
#if IS_ENABLED(CONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION)
	motion_bias_policy.rest_update = (cycle_bias_admission & 2u) != 0;
#else
	motion_bias_policy.rest_update = s->restDetected && p->restBiasEstEnabled;
#endif
	if (motion_bias_policy.rest_update) {
		/* Recency is bookkeeping, not rest probability or measurement noise.
		 * Preserve the admitted native rest weights and skip the motion IIR. */
		motion_bias_policy.since_rest *= motion_bias_policy.rest_retention;
		motion_bias_policy.log_scale *= motion_bias_policy.rest_retention;
		return;
	}
	motion_bias_policy.since_rest += c->accTs;
	float gate = motion_bias_policy.since_rest / (motion_bias_policy.since_rest + 300.0f);
	float target = -.55f * motion_bias_policy.excitation * gate;
	target = fmaxf(logf(.5f), fminf(logf(2.f), target));
	motion_bias_policy.log_scale += motion_bias_policy.alpha * (target - motion_bias_policy.log_scale);
	float scale = expf(motion_bias_policy.log_scale);
	if (scale == 1) {
		return;
	}
	float sigma = p->biasSigmaMotion * scale;
	float variance = sigma * 100.f;
	variance *= variance;
	float weight = variance * variance / c->biasV + variance;
	w[0] = w[1] = weight;
	w[2] = weight / fmaxf(p->biasVerticalForgettingFactor, 1e-10f);
}

static void vqf_predictive_predict(void)
{
	if (!predictive_bias_policy.have && predictive_bias_policy.temperature_available) {
		predictive_bias_policy.temperature = predictive_bias_policy.hold;
		predictive_bias_policy.have = true;
	}
	/* Causal last-observation hold, including gyro steps without a new reading. */
	if (predictive_bias_policy.have) {
		predictive_bias_policy.temperature += predictive_bias_policy.temperature_alpha
			* (predictive_bias_policy.hold - predictive_bias_policy.temperature);
	}
	if (!predictive_bias_policy.have || vqf_hard_rest_detected() || predictive_bias_policy.gain <= 0) {
		return;
	}
	float delta = fmaxf(-5, fminf(5, predictive_bias_policy.temperature - predictive_bias_policy.anchor));
	float off[3], n2 = 0;
	for (int j = 0; j < 3; j++) {
		off[j] = predictive_bias_policy.beta[j] * delta * predictive_bias_policy.gain;
		n2 += off[j] * off[j];
	}
	float rad = .017453292519943295f, limit = .03f * rad;
	float scale = fminf(1, limit / fmaxf(sqrtf(n2), 1e-20f)), clip = params.biasClip * rad;
	for (int j = 0; j < 3; j++) {
		off[j] *= scale;
		state.bias[j] = fmaxf(-clip, fminf(clip, state.bias[j] + off[j] - predictive_bias_policy.offset[j]));
		predictive_bias_policy.offset[j] = off[j];
	}
}

static void vqf_predictive_finish_block(void)
{
	float t = (float)(predictive_bias_policy.block_t / predictive_bias_policy.block_n), g[3];
	for (int j = 0; j < 3; j++) {
		g[j] = (float)(predictive_bias_policy.block_g[j] / predictive_bias_policy.block_n);
	}
	/* Score the frozen slope before this arrived block enters training. The
	 * matched model and constant both reanchor on the previous observed block. */
	if (predictive_bias_policy.trial_have && predictive_bias_policy.last_have) {
		float delta = t - predictive_bias_policy.last_t;
		if (fabsf(delta) >= .01f) {
			for (int j = 0; j < 3; j++) {
				double dc = g[j] - predictive_bias_policy.last_g[j];
				double dm = dc - (double)(predictive_bias_policy.trial_beta[j] * delta);
				predictive_bias_policy.trial_const[j] += dc * dc;
				predictive_bias_policy.trial_model[j] += dm * dm;
			}
			predictive_bias_policy.trial_tests++;
			if (predictive_bias_policy.trial_tests >= 2) {
				double floor = (double).0002f * .017453292519943295, lc = 0, lm = 0;
				for (int j = 0; j < 3; j++) {
					lc += predictive_bias_policy.trial_weight[j] * predictive_bias_policy.trial_const[j];
					lm += predictive_bias_policy.trial_weight[j] * predictive_bias_policy.trial_model[j];
				}
				double skill = (lc - lm) / (lc + predictive_bias_policy.trial_tests * floor * floor);
				if (skill > 0) {
					memcpy(predictive_bias_policy.beta, predictive_bias_policy.trial_beta,
					       sizeof(predictive_bias_policy.beta));
					predictive_bias_policy.gain = 8.0 * fmin(1, skill);
				} else {
					predictive_bias_policy.gain = 0;
				}
				vqf_predictive_clear_trial();
			}
		}
	}
	/* Completed-block central variances survive rest episodes. The current
	 * block cannot influence the weights of the trial it just validated. */
	predictive_bias_policy.noise_n++;
	for (int j = 0; j < 3; j++) {
		double variance = (predictive_bias_policy.block_gg[j]
			- predictive_bias_policy.block_g[j] * predictive_bias_policy.block_g[j] / predictive_bias_policy.block_n)
			/ fmax(1, predictive_bias_policy.block_n - 1);
		variance = fmax(0, variance);
		predictive_bias_policy.noise[j] += (variance - predictive_bias_policy.noise[j])
			/ predictive_bias_policy.noise_n;
	}
	predictive_bias_policy.n++;
	double dx = (double)t - predictive_bias_policy.mean_t;
	predictive_bias_policy.mean_t += dx / predictive_bias_policy.n;
	predictive_bias_policy.tt += dx * ((double)t - predictive_bias_policy.mean_t);
	predictive_bias_policy.min_t = fminf(predictive_bias_policy.min_t, t);
	predictive_bias_policy.max_t = fmaxf(predictive_bias_policy.max_t, t);
	for (int j = 0; j < 3; j++) {
		double dy = (double)g[j] - predictive_bias_policy.mean_g[j];
		predictive_bias_policy.mean_g[j] += dy / predictive_bias_policy.n;
		predictive_bias_policy.tg[j] += dx * ((double)g[j] - predictive_bias_policy.mean_g[j]);
	}
	predictive_bias_policy.last_t = t;
	memcpy(predictive_bias_policy.last_g, g, sizeof(g));
	predictive_bias_policy.last_have = true;
	if (!predictive_bias_policy.trial_have && predictive_bias_policy.age >= 10.0
	    && predictive_bias_policy.n >= 3 && predictive_bias_policy.max_t - predictive_bias_policy.min_t >= .1f) {
		float n2 = 0;
		for (int j = 0; j < 3; j++) {
			predictive_bias_policy.trial_beta[j] = (float)(predictive_bias_policy.tg[j]
				/ (predictive_bias_policy.tt + predictive_bias_policy.n * .05 * .05));
			n2 += predictive_bias_policy.trial_beta[j] * predictive_bias_policy.trial_beta[j];
		}
		float rad = .017453292519943295f;
		float scale = fminf(1, .018f * rad / fmaxf(sqrtf(n2), 1e-20f));
		for (int j = 0; j < 3; j++) {
			predictive_bias_policy.trial_beta[j] *= scale;
		}
		/* Freeze noise weights with the slope, before any heldout block. */
		memset(predictive_bias_policy.trial_const, 0, sizeof(predictive_bias_policy.trial_const));
		memset(predictive_bias_policy.trial_model, 0, sizeof(predictive_bias_policy.trial_model));
		double floor = (double).0006f * .017453292519943295, sum = 0;
		for (int j = 0; j < 3; j++) {
			predictive_bias_policy.trial_weight[j] = 1 / fmax(predictive_bias_policy.noise[j], floor * floor);
			sum += predictive_bias_policy.trial_weight[j];
		}
		for (int j = 0; j < 3; j++) {
			predictive_bias_policy.trial_weight[j] *= 3 / sum;
		}
		predictive_bias_policy.trial_have = true;
	}
	vqf_predictive_clear_block();
}

static void vqf_predictive_observe(void)
{
	if (!predictive_bias_policy.have) {
		return;
	}
#if IS_ENABLED(CONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION)
	bool natural_rest = (cycle_bias_admission & 1u) != 0;
#else
	bool natural_rest = motion_bias_policy.rest_update;
#endif
	if (!natural_rest || !vqf_hard_rest_detected()) {
		predictive_bias_policy.in_rest = false;
		vqf_predictive_clear_block();
		return;
	}
	predictive_bias_policy.anchor = predictive_bias_policy.temperature;
	memset(predictive_bias_policy.offset, 0, sizeof(predictive_bias_policy.offset));
	if (!predictive_bias_policy.in_rest) {
		predictive_bias_policy.in_rest = true;
		predictive_bias_policy.age = 0;
		vqf_predictive_clear_block();
		vqf_predictive_clear_fit();
	}
	predictive_bias_policy.age += (double)coeffs.accTs;
	if (predictive_bias_policy.age < 3) {
		return;
	}
	predictive_bias_policy.block_age += (double)coeffs.accTs;
	predictive_bias_policy.block_n++;
	predictive_bias_policy.block_t += (double)vqf_training_temperature();
	for (int j = 0; j < 3; j++) {
		double observation = (double)state.restLastGyrLp[j];
		predictive_bias_policy.block_g[j] += observation;
		predictive_bias_policy.block_gg[j] += observation * observation;
	}
	if (predictive_bias_policy.block_age + 1e-6 >= 2.0) {
		vqf_predictive_finish_block();
	}
}

static void vqf_rebase_bias_policy(const vqf_real_t delta[3])
{
	/* Coordinate translations leave central variance, centered covariance,
	 * frozen weights, slopes and validation losses unchanged. Translate the
	 * partial second moment using the OLD first moment before its sum. */
	for (int j = 0; j < 3; j++) {
		double d = delta[j];
		predictive_bias_policy.block_gg[j] += 2 * d * predictive_bias_policy.block_g[j]
			+ predictive_bias_policy.block_n * d * d;
		predictive_bias_policy.block_g[j] += predictive_bias_policy.block_n * d;
		if (predictive_bias_policy.n > 0) {
			predictive_bias_policy.mean_g[j] += d;
		}
		if (predictive_bias_policy.last_have) {
			predictive_bias_policy.last_g[j] += delta[j];
		}
#if IS_ENABLED(CONFIG_SENSOR_VQF_REST_TRANSITION)
		if (rest_transition_policy.age > 0) {
			rest_transition_policy.mean[j] += delta[j];
		}
#endif
	}
}
#endif /* CONFIG_SENSOR_VQF_ADAPTIVE_BIAS */

/* Rest detection diagnostics */
static uint32_t rest_enter_count;
static uint32_t rest_exit_count;
static float rest_total_s;
static float rest_last_enter_time; /* uptime when last rest started */
static float rest_last_duration_s;
static float uptime_s;
static bool prev_rest_detected;

/* Circular rest event log */
#define REST_EVENT_LOG_SIZE 5
static struct {
	float time_s;
	bool entered;
} rest_event_log[REST_EVENT_LOG_SIZE];
static uint8_t rest_event_idx;   /* next write position */
static uint8_t rest_event_total; /* total events (up to log size) */

void vqf_update_sensor_ids(int imu)
{
	ARG_UNUSED(imu);
}

static void set_params()
{
	init_params(&params);
	params.tauAcc = 3.264f;
	params.biasClip = 5.0f;
	params.biasForgettingTime = 162.0f;
	params.biasSigmaInit = 1.32f;
	params.biasSigmaMotion = 0.1539f;
	params.biasSigmaRest = 0.0155f;
	params.biasVerticalForgettingFactor = 0.0001f;
	params.motionBiasEstEnabled = true;
	params.restBiasEstEnabled = true;
	params.restFilterTau = 2.6f;
	params.restMinT = 0.25f;
	params.restThGyr = 0.4024f;
	params.restThAcc = 0.107f;
	params.magDistRejectionEnabled = true;
	params.tauMag = 9.0f;
	params.magCurrentTau = 0.20f;
	params.magNormTh = 0.10f;
	params.magDipTh = 5.0f;
	params.magRefTau = 10.0f;
	params.magNewTime = 15.0f;
	params.magNewFirstTime = 5.0f;
	params.magNewMinGyr = 10.0f;
	params.magMinUndisturbedTime = 0.5f;
	params.magMaxRejectionTime = 60.0f;
	params.magRejectionFactor = 1000.0f;
}

void vqf_init(float g_time, float a_time, float m_time)
{
	rest_observation_pending = false;
	set_params();
	initVqf(&params, &state, &coeffs, g_time, a_time, m_time);
#if IS_ENABLED(CONFIG_SENSOR_VQF_FAST_REST_RECOVERY)
	stop_macro_reset();
#endif
#if IS_ENABLED(CONFIG_SENSOR_VQF_OUTPUT_OBSERVER)
	initOutputObserver(&output_observer, &state, &coeffs);
#endif
#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
	vqf_reset_adaptive_tau();
#endif
#if IS_ENABLED(CONFIG_SENSOR_VQF_ADAPTIVE_BIAS)
	vqf_reset_bias_policy();
#endif
	rest_enter_count = 0;
	rest_exit_count = 0;
	rest_total_s = 0;
	rest_last_enter_time = 0;
	rest_last_duration_s = 0;
	uptime_s = 0;
	prev_rest_detected = false;
	rest_event_idx = 0;
	rest_event_total = 0;
}

/* Finite test by exponent bits (all ones = NaN/±inf). The ARM isfinite()
 * builtin compiles to vcmp+bhi, which lets NaN through (an unordered
 * compare never sets the branch condition) — test the bit pattern. */
static ALWAYS_INLINE bool vqf_float_finite(float x)
{
	uint32_t bits;
	memcpy(&bits, &x, sizeof(bits));
	return (bits & 0x7F800000u) != 0x7F800000u;
}

static ALWAYS_INLINE bool vqf_vec3_finite(const float v[3])
{
	return vqf_float_finite(v[0]) && vqf_float_finite(v[1]) && vqf_float_finite(v[2]);
}

#if IS_ENABLED(CONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION)
void vqf_discard_motion_history(void)
{
	/* No elapsed wall clock is inferred across sleep or acquisition loss. */
	memset(&cycle_bias_policy, 0, sizeof(cycle_bias_policy));
	cycle_bias_policy.axis = 3;
	cycle_bias_policy.q[0] = 1;
	cycle_bias_admission = 0;
}

void vqf_request_motion_history_reset(void)
{
	/* A suspended owner may be inside an estimator transaction. Do not touch
	 * its live policy until that transaction finishes at a sample boundary. */
	atomic_set(&cycle_reset_pending, 1);
}

static void vqf_cycle_apply_pending_reset(void)
{
	if (atomic_get(&cycle_reset_pending) && atomic_set(&cycle_reset_pending, 0)) {
		vqf_discard_motion_history();
	}
}

static void vqf_cycle_start(void)
{
	memset(cycle_bias_policy.q, 0, sizeof(cycle_bias_policy.q));
	cycle_bias_policy.q[0] = 1;
	memcpy(cycle_bias_policy.b0, state.bias, sizeof(cycle_bias_policy.b0));
	cycle_bias_policy.elapsed = cycle_bias_policy.compensation = cycle_bias_policy.peak = 0;
}

static void vqf_cycle_integrate(const float g[3], float dt)
{
	if (dt <= 0) {
		return;
	}
	float v[3], a2 = 0;
	for (int j = 0; j < 3; j++) {
		v[j] = (g[j] - cycle_bias_policy.b0[j]) * dt;
		a2 += v[j] * v[j];
	}
	float a4 = a2 * a2;
	float w = cycle_bias_policy.q[0], x = cycle_bias_policy.q[1];
	float y = cycle_bias_policy.q[2], z = cycle_bias_policy.q[3];
	/* Ordered right multiplication by Exp((g-b0)dt), small-angle series. */
	float c = 1 - a2 / 8 + a4 / 384 - a4 * a2 / 46080;
	float s = .5f - a2 / 48 + a4 / 3840 - a4 * a2 / 645120;
	float u = v[0] * s, vv = v[1] * s, r = v[2] * s;
	cycle_bias_policy.q[0] = w * c - x * u - y * vv - z * r;
	cycle_bias_policy.q[1] = w * u + x * c + y * r - z * vv;
	cycle_bias_policy.q[2] = w * vv - x * r + y * c + z * u;
	cycle_bias_policy.q[3] = w * r + x * vv - y * u + z * c;
	float norm = sqrtf(cycle_bias_policy.q[0] * cycle_bias_policy.q[0]
		+ cycle_bias_policy.q[1] * cycle_bias_policy.q[1]
		+ cycle_bias_policy.q[2] * cycle_bias_policy.q[2]
		+ cycle_bias_policy.q[3] * cycle_bias_policy.q[3]);
	for (int j = 0; j < 4; j++) {
		cycle_bias_policy.q[j] /= norm;
	}
	float add = dt - cycle_bias_policy.compensation, total = cycle_bias_policy.elapsed + add;
	cycle_bias_policy.compensation = (total - cycle_bias_policy.elapsed) - add;
	cycle_bias_policy.elapsed = total;
}

static bool vqf_cycle_observation(float obs[3], float *residual)
{
	float n = sqrtf(cycle_bias_policy.q[1] * cycle_bias_policy.q[1]
		+ cycle_bias_policy.q[2] * cycle_bias_policy.q[2]
		+ cycle_bias_policy.q[3] * cycle_bias_policy.q[3]);
	float sign = cycle_bias_policy.q[0] < 0 ? -1.f : 1.f;
	float scale = n > 1e-12f ? 2 * atan2f(n, sign * cycle_bias_policy.q[0]) / n : 2.f;
	float r[3];
	for (int j = 0; j < 3; j++) {
		r[j] = sign * cycle_bias_policy.q[j + 1] * scale;
	}
	*residual = sqrtf(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
	if (!(cycle_bias_policy.elapsed > 0) || !vqf_float_finite(*residual)) {
		return false;
	}
	for (int j = 0; j < 3; j++) {
		/* Approximate closure feedback, not an exact SO(3) root or body mean. */
		obs[j] = cycle_bias_policy.b0[j];
		obs[j] += r[j] / cycle_bias_policy.elapsed;
		if (!vqf_float_finite(obs[j]) || fabsf(obs[j]) > 5 * DEG_TO_RAD) {
			return false;
		}
	}
	return true;
}

static void vqf_cycle_step(const float g[3])
{
	float dt = coeffs.gyrTs, rate2 = 0, b02 = 0;
	for (int j = 0; j < 3; j++) {
		rate2 += g[j] * g[j];
		b02 += cycle_bias_policy.b0[j] * cycle_bias_policy.b0[j];
	}
	if (!(dt > 0 && dt <= .01f) || !vqf_float_finite(b02)
	    || b02 > 75 * DEG_TO_RAD * DEG_TO_RAD || !vqf_float_finite(rate2)
	    || rate2 > 25 * DEG_TO_RAD * DEG_TO_RAD) {
		vqf_discard_motion_history();
		return;
	}
	cycle_bias_policy.obsage += dt;
	float limit = cycle_bias_policy.locked ? 2.f * cycle_bias_policy.period * 1.1f : 8.f;
	if (cycle_bias_policy.axis != 3 && cycle_bias_policy.obsage > limit) {
		vqf_discard_motion_history();
	}
	/* Pending observation age is not regime confidence age. Never reuse it. */
	if (cycle_bias_policy.locked && cycle_bias_policy.info
	    && cycle_bias_policy.obsage > cycle_bias_policy.period * 1.1f) {
		cycle_bias_policy.info = 0;
	}
	if (cycle_bias_policy.axis == 3) {
		int k = 0;
		for (int j = 1; j < 3; j++) {
			if (fabsf(rest_transition_policy.mean[j] - state.restLastGyrLp[j])
			    > fabsf(rest_transition_policy.mean[k] - state.restLastGyrLp[k])) {
				k = j;
			}
		}
		if (fabsf(rest_transition_policy.mean[k] - state.restLastGyrLp[k]) <= .04f * DEG_TO_RAD) {
			return;
		}
		cycle_bias_policy.axis = k;
		cycle_bias_policy.obsage = 0;
	}
	float x = rest_transition_policy.mean[cycle_bias_policy.axis] - state.restLastGyrLp[cycle_bias_policy.axis];
	if (x < -.02f * DEG_TO_RAD) {
		cycle_bias_policy.armed = 1;
	}
	cycle_bias_policy.peak = fmaxf(cycle_bias_policy.peak, fabsf(x));
	bool crossing = cycle_bias_policy.armed && cycle_bias_policy.prevx < 0 && x >= 0;
	float fraction = crossing ? -cycle_bias_policy.prevx / (x - cycle_bias_policy.prevx) : 1.f;
	if (cycle_bias_policy.started) {
		vqf_cycle_integrate(g, dt * fraction);
	}
	if (crossing) {
		cycle_bias_policy.armed = 0;
		if (!cycle_bias_policy.started) {
			cycle_bias_policy.obsage = 0;
		} else {
			float obs[3], residual = 0, T = cycle_bias_policy.elapsed;
			bool solved = vqf_cycle_observation(obs, &residual);
			bool good = solved && T >= 2.f && T <= 6.667f;
			if (cycle_bias_policy.period > 0
			    && fmaxf(T / cycle_bias_policy.period, cycle_bias_policy.period / T) > 1.05f) {
				good = false;
			}
			if (cycle_bias_policy.oldpeak > 0
			    && fmaxf(cycle_bias_policy.peak / cycle_bias_policy.oldpeak,
				     cycle_bias_policy.oldpeak / cycle_bias_policy.peak) > 1.20f) {
				good = false;
			}
			float accgap = 0, obsgap = 0, correction = 0;
			for (int j = 0; j < 3; j++) {
				float a = state.restLastAccLp[j] - cycle_bias_policy.acc[j];
				accgap += a * a;
				if (solved) {
					a = obs[j] - cycle_bias_policy.obs[j];
					obsgap += a * a;
					a = obs[j] - cycle_bias_policy.b0[j];
					correction += a * a;
				}
			}
			if (accgap > .05f * .05f
			    || (cycle_bias_policy.agree && obsgap > .01f * .01f * DEG_TO_RAD * DEG_TO_RAD)
			    || correction > .2f * .2f * DEG_TO_RAD * DEG_TO_RAD
			    || residual > T * .2f * DEG_TO_RAD) {
				good = false;
			}
			if (good) {
				cycle_bias_policy.obsage = 0;
				if (cycle_bias_policy.agree < 3) {
					cycle_bias_policy.agree++;
				}
			} else {
				cycle_bias_policy.info = 0;
				if (!cycle_bias_policy.locked) {
					cycle_bias_policy.agree = 0;
				}
			}
			if (solved && (good || !cycle_bias_policy.locked)) {
				memcpy(cycle_bias_policy.obs, obs, sizeof(obs));
			}
			if (good && cycle_bias_policy.agree >= 3) {
				cycle_bias_policy.locked = cycle_bias_policy.info = 1;
				cycle_bias_policy.obsage = 0;
			}
			/* Bad cycles cannot refresh an established regime's good history. */
			if (good || !cycle_bias_policy.locked) {
				cycle_bias_policy.period = T;
				cycle_bias_policy.oldpeak = cycle_bias_policy.peak;
			}
		}
		memcpy(cycle_bias_policy.acc, state.restLastAccLp, sizeof(cycle_bias_policy.acc));
		vqf_cycle_start();
		cycle_bias_policy.started = 1;
		vqf_cycle_integrate(g, dt * (1 - fraction));
	}
	cycle_bias_policy.prevx = x;
}

bool vqf_rest_bias_select(vqf_params_t *p, vqf_state_t *s)
{
	bool natural = s->restDetected && p->restBiasEstEnabled;
	if (s != &state) {
		return natural;
	}
	bool selected = p->restBiasEstEnabled && (cycle_bias_policy.locked ? cycle_bias_policy.info : natural);
	cycle_bias_admission = (natural ? 1u : 0u) | (selected ? 2u : 0u);
	return selected;
}
#endif

#if IS_ENABLED(CONFIG_SENSOR_VQF_FAST_REST_RECOVERY) || IS_ENABLED(CONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION)
void vqf_rest_bias_observation(vqf_params_t *p, vqf_state_t *s, vqf_coeffs_t *c,
			       vqf_real_t e[3], vqf_real_t w[3])
{
	if (s != &state) {
		return;
	}
#if IS_ENABLED(CONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION)
	if (cycle_bias_policy.locked) {
		/* One completed nonoverlap cycle, consumed only on fresh nonzero ACC. */
		cycle_bias_policy.weight = c->biasRestW * c->accTs / cycle_bias_policy.period;
		for (int j = 0; j < 3; j++) {
			e[j] = cycle_bias_policy.obs[j] - s->bias[j];
			w[j] = cycle_bias_policy.weight;
		}
		cycle_bias_policy.info = 0;
		return;
	}
#endif
#if IS_ENABLED(CONFIG_SENSOR_VQF_FAST_REST_RECOVERY)
	if (stop_macro_gate.remaining == 0 || stop_macro_gate.peak2 < 0) {
		return;
	}
	if (s->restT < 2.f * p->restMinT || rest_transition_policy.age < 1.f
	    || rest_transition_policy.variance >= (.08f * DEG_TO_RAD) * (.08f * DEG_TO_RAD)) {
		return;
	}
	float support2 = 0, slow2 = 0, gap2 = 0;
	for (int j = 0; j < 3; j++) {
		float d = rest_transition_policy.mean[j] - s->bias[j];
		support2 += d * d;
		d = s->restLastGyrLp[j] - s->bias[j];
		slow2 += d * d;
		d = rest_transition_policy.mean[j] - s->restLastGyrLp[j];
		gap2 += d * d;
	}
	float sigma = p->biasSigmaRest * DEG_TO_RAD;
	if (support2 > fmaxf((.08f * DEG_TO_RAD) * (.08f * DEG_TO_RAD), slow2)
	    || gap2 <= sigma * sigma) {
		return;
	}
	bool strong = gap2 > (.1f * DEG_TO_RAD) * (.1f * DEG_TO_RAD);
	/* Explicit recovery heuristic, not calibrated physical covariance. */
	float recovery_weight = 0;
	if (strong) {
		float recovery_sigma = p->biasSigmaRest * 100.f;
		recovery_weight = recovery_sigma * recovery_sigma;
	}
	for (int j = 0; j < 3; j++) {
		e[j] = rest_transition_policy.mean[j] - s->bias[j];
		if (strong) {
			w[j] = recovery_weight;
		}
	}
#endif
}
#endif

void vqf_update_temperature(float temperature_c)
{
#if IS_ENABLED(CONFIG_SENSOR_VQF_ADAPTIVE_BIAS)
	if (vqf_float_finite(temperature_c)) {
		predictive_bias_policy.hold = temperature_c;
		predictive_bias_policy.temperature_available = true;
	}
#else
	ARG_UNUSED(temperature_c);
#endif
}

static bool vqf_loaded_state_valid(void)
{
	/* Reject NaN/inf retained state: it propagates permanently (clip()
	 * and the Kalman update carry NaN). LP filter *states* are excluded:
	 * they legitimately hold NaN markers during initialization. */
	for (size_t i = 0; i < 3; i++) {
		if (!vqf_float_finite(state.bias[i]) || !vqf_float_finite(state.restLastGyrLp[i])
		    || !vqf_float_finite(state.restLastAccLp[i]) || !vqf_float_finite(state.lastAccLp[i])) {
			return false;
		}
	}
	for (size_t i = 0; i < 4; i++) {
		if (!vqf_float_finite(state.gyrQuat[i]) || !vqf_float_finite(state.accQuat[i])) {
			return false;
		}
	}
	for (size_t i = 0; i < 9; i++) {
		if (!vqf_float_finite(state.biasP[i])) {
			return false;
		}
	}
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
	for (size_t i = 0; i < 9; i++) {
		if (!vqf_float_finite(state.motionBiasEstLastR[i])) {
			return false;
		}
	}
	for (size_t i = 0; i < 2; i++) {
		if (!vqf_float_finite(state.motionBiasEstLastBiasLp[i])) {
			return false;
		}
	}
#endif
	return vqf_float_finite(coeffs.gyrTs) && coeffs.gyrTs > 0.0f
	       && vqf_float_finite(coeffs.accTs) && coeffs.accTs > 0.0f
	       && vqf_float_finite(coeffs.magTs);
}


void vqf_load(const void *data, float g_time, float a_time, float m_time)
{
	rest_observation_pending = false;
	BUILD_ASSERT(
		VQF_MEM_SIZE <= sizeof(((struct retained_data *)0)->fusion_data),
		"VQF state+coeffs exceeds fusion_data buffer in retained memory"
	);
	vqf_saved_header_t header;
	memcpy(&header, data, sizeof(header));
	if (header.magic != VQF_SAVED_MAGIC || header.version != VQF_SAVED_VERSION
	    || header.size != VQF_MEM_SIZE) {
		/* Never interpret another version/configuration's payload layout. */
		printk("VQF: rejecting incompatible retained layout, reinitializing\n");
		vqf_init(g_time, a_time, m_time);
		return;
	}
	const uint8_t *payload = (const uint8_t *)data + sizeof(header);
	memcpy(&coeffs, payload + sizeof(state), sizeof(coeffs));
	/* Match the normalization performed by initVqf, including disabled mag. */
	float gyr_ts = g_time > 0.0f ? g_time : 0.01f;
	float acc_ts = a_time > 0.0f ? a_time : gyr_ts;
	float mag_ts = m_time > 0.0f ? m_time : gyr_ts;
	if (coeffs.gyrTs != gyr_ts || coeffs.accTs != acc_ts || coeffs.magTs != mag_ts) {
		printk("VQF: rejecting retained sampling periods, reinitializing\n");
		vqf_init(g_time, a_time, m_time);
		return;
	}
	set_params();
	memcpy(&state, payload, sizeof(state));
	if (!vqf_loaded_state_valid()) {
		printk("VQF: rejecting non-finite retained state, reinitializing\n");
		vqf_init(g_time, a_time, m_time);
		return;
	}
	/* Retained memory stores state+coeffs, not params or the controller.
	 * Rebase saved adaptive LP histories to the startup tau without changing
	 * attitude/bias. Force setTauAcc past its params-only equality shortcut. */
	float startup_tau = params.tauAcc;
	params.tauAcc = 0.0f;
	setTauAcc(&params, &state, &coeffs, startup_tau);
#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
	vqf_reset_adaptive_tau();
#endif
#if IS_ENABLED(CONFIG_SENSOR_VQF_OUTPUT_OBSERVER)
	if (!vqf_output_load(payload + sizeof(state) + sizeof(coeffs))) {
		printk("VQF: rejecting invalid output retained state, reinitializing\n");
		vqf_init(g_time, a_time, m_time);
		return;
	}
#endif
#if IS_ENABLED(CONFIG_SENSOR_VQF_ADAPTIVE_BIAS)
	vqf_reset_bias_policy();
#endif
#if IS_ENABLED(CONFIG_SENSOR_VQF_FAST_REST_RECOVERY)
	stop_macro_restore(false);
#endif
	rest_enter_count = 0;
	rest_exit_count = 0;
	rest_total_s = 0;
	rest_last_enter_time = 0;
	rest_last_duration_s = 0;
	uptime_s = 0;
	prev_rest_detected = false;
	rest_event_idx = 0;
	rest_event_total = 0;
}

void vqf_save(void *data)
{
	BUILD_ASSERT(
		VQF_MEM_SIZE <= sizeof(((struct retained_data *)0)->fusion_data),
		"VQF state+coeffs exceeds fusion_data buffer in retained memory"
	);
	const vqf_saved_header_t header = {
		.magic = VQF_SAVED_MAGIC,
		.version = VQF_SAVED_VERSION,
		.size = VQF_MEM_SIZE,
	};
	memcpy(data, &header, sizeof(header));
	uint8_t *payload = (uint8_t *)data + sizeof(header);
	memcpy(payload, &state, sizeof(state));
	memcpy(payload + sizeof(state), &coeffs, sizeof(coeffs));
#if IS_ENABLED(CONFIG_SENSOR_VQF_OUTPUT_OBSERVER)
	memcpy(payload + sizeof(state) + sizeof(coeffs), output_observer.lp, sizeof(output_observer.lp));
	memcpy(payload + sizeof(state) + sizeof(coeffs) + sizeof(output_observer.lp),
	       output_observer.accQuat, sizeof(output_observer.accQuat));
#endif
}

void vqf_update_gyro(float *g, float time)
{
	ARG_UNUSED(time);
#if IS_ENABLED(CONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION)
	vqf_cycle_apply_pending_reset();
#endif
	if (!vqf_vec3_finite(g)) {
#if IS_ENABLED(CONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION)
		vqf_discard_motion_history();
#endif
		return;
	}
	float g_rad[3] = {0};
	// g is in deg/s, convert to rad/s
	for (int i = 0; i < 3; i++) {
		g_rad[i] = g[i] * DEG_TO_RAD;
	}
#if IS_ENABLED(CONFIG_SENSOR_VQF_ADAPTIVE_BIAS)
	vqf_predictive_predict();
	vqf_training_temperature_decide();
#endif
	/* Fixed coeffs->gyrTs path (caller-dt / lastGyrTsUs synth disabled for A/B). */
	updateGyr(&params, &state, &coeffs, g_rad);
#if IS_ENABLED(CONFIG_SENSOR_VQF_FAST_REST_RECOVERY)
	stop_macro_step(g_rad);
#endif
#if IS_ENABLED(CONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION)
	vqf_cycle_step(g_rad);
#endif
#if IS_ENABLED(CONFIG_SENSOR_VQF_ADAPTIVE_BIAS)
	vqf_training_temperature_update();
#endif
#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
	float speed_squared = 0.0f;
	for (int i = 0; i < 3; i++) {
		float corrected = g_rad[i] - state.bias[i];
		speed_squared += corrected * corrected;
	}
	float speed = sqrtf(speed_squared) * RAD_TO_DEG;
	adaptive_gyr_signal += adaptive_gyr_alpha * (speed - adaptive_gyr_signal);
#endif
	rest_observation_pending = true;
}

#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
/* Called only for valid, nonzero acceleration samples, before updateAcc. */
static void vqf_pre_accel_update(const float a_m_s2[3])
{
	float norm = sqrtf(a_m_s2[0] * a_m_s2[0] + a_m_s2[1] * a_m_s2[1] + a_m_s2[2] * a_m_s2[2]);
	float deviation = fabsf(norm - CONST_EARTH_GRAVITY);
	adaptive_acc_signal += adaptive_acc_alpha * (deviation - adaptive_acc_signal);
	float g = adaptive_gyr_signal / (adaptive_gyr_signal + ADAPTIVE_GYR_SCALE);
	float a = adaptive_acc_signal / (adaptive_acc_signal + ADAPTIVE_ACC_SCALE);
	float fraction = 0.5f + 0.5f * (a - g);
	float target = logf(ADAPTIVE_TAU_MIN) + fraction * logf(ADAPTIVE_TAU_MAX / ADAPTIVE_TAU_MIN);
	float alpha = target > adaptive_log_tau ? adaptive_up_alpha : adaptive_down_alpha;
	adaptive_log_tau += alpha * (target - adaptive_log_tau);
	adaptive_clock += coeffs.accTs;
	if (adaptive_clock >= ADAPTIVE_TAU_INTERVAL) {
		adaptive_clock = fmodf(adaptive_clock, ADAPTIVE_TAU_INTERVAL);
		float tau = fmaxf(ADAPTIVE_TAU_MIN, fminf(ADAPTIVE_TAU_MAX, expf(adaptive_log_tau)));
		/* Updates both inclination and motion-bias LP states continuously. */
		setTauAcc(&params, &state, &coeffs, tau);
	}
}
#endif /* CONFIG_VQF_ADAPTIVE_TAU_ACC */

/**
 * @brief Track rest detection transitions and accumulate diagnostics.
 * Called after each accelerometer update (which runs rest detection).
 */
static void vqf_track_rest_diag(void)
{
	float dt = coeffs.accTs;
	uptime_s += dt;

	bool cur = vqf_hard_rest_detected();
	if (cur && !prev_rest_detected) {
		/* entering rest */
		rest_enter_count++;
		rest_last_enter_time = uptime_s;
		rest_event_log[rest_event_idx].time_s = uptime_s;
		rest_event_log[rest_event_idx].entered = true;
		rest_event_idx = (rest_event_idx + 1) % REST_EVENT_LOG_SIZE;
		if (rest_event_total < REST_EVENT_LOG_SIZE) {
			rest_event_total++;
		}
	} else if (!cur && prev_rest_detected) {
		/* leaving rest */
		rest_exit_count++;
		rest_last_duration_s = uptime_s - rest_last_enter_time;
		rest_event_log[rest_event_idx].time_s = uptime_s;
		rest_event_log[rest_event_idx].entered = false;
		rest_event_idx = (rest_event_idx + 1) % REST_EVENT_LOG_SIZE;
		if (rest_event_total < REST_EVENT_LOG_SIZE) {
			rest_event_total++;
		}
	}
	if (cur) {
		rest_total_s += dt;
	}
	prev_rest_detected = cur;
}

void vqf_update_accel(float *a, float time)
{
	ARG_UNUSED(time);
#if IS_ENABLED(CONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION)
	vqf_cycle_apply_pending_reset();
#endif
	if (!vqf_vec3_finite(a)) {
#if IS_ENABLED(CONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION)
		vqf_discard_motion_history();
#endif
		return;
	}
	float a_m_s2[3] = {0};
	// a is in g, convert to m/s^2
	for (int i = 0; i < 3; i++) {
		a_m_s2[i] = a[i] * CONST_EARTH_GRAVITY;
	}
	if (a_m_s2[0] != 0 || a_m_s2[1] != 0 || a_m_s2[2] != 0) {
		memcpy(last_a, a_m_s2, sizeof(a_m_s2));
#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
		vqf_pre_accel_update(a_m_s2);
#endif
	}
#if IS_ENABLED(CONFIG_SENSOR_VQF_ADAPTIVE_BIAS)
	motion_bias_policy.rest_update = false;
#endif
	/* Fixed coeffs->accTs path (caller-dt / lastAccTsUs synth disabled for A/B). */
	updateAcc(&params, &state, &coeffs, a_m_s2);
#if IS_ENABLED(CONFIG_SENSOR_VQF_OUTPUT_OBSERVER)
	updateOutputObserver(&output_observer, &state, &coeffs, a_m_s2);
#endif
	if (a_m_s2[0] != 0 || a_m_s2[1] != 0 || a_m_s2[2] != 0) {
#if IS_ENABLED(CONFIG_SENSOR_VQF_ADAPTIVE_BIAS)
		vqf_predictive_observe();
#endif
		rest_observation_pending = true;
	}
	vqf_track_rest_diag();
}

#if CONFIG_SENSOR_USE_MAGNETOMETER
void vqf_update_mag(float *m, float time)
{
	if (!vqf_vec3_finite(m)) {
		return;
	}
	// Use the caller-supplied time step when valid so that VQF time accumulators
	// (magCandidateT, magRejectT, etc.) and gain k run at the correct real-time
	// rate even when the sensor loop runs faster or slower than the mag ODR.
	//
	// Build a synthetic cumulative microsecond timestamp from the elapsed time so
	// updateMagTs can derive the correct dt internally (avoids calling the static
	// updateMag_internal directly).
	if (time > 0.0f && time < 10.0f) {
		uint64_t synth_ts = state.lastMagTsUs + (uint64_t)(time * 1e6f);
		if (synth_ts == 0) {
			synth_ts = 1; // avoid the "uninitialized" sentinel value
		}
		updateMagTs(&params, &state, &coeffs, m, synth_ts);
	} else {
		updateMag(&params, &state, &coeffs, m);
	}
}
#endif

void vqf_update(float *g, float *a, float *m, float time)
{
	// TODO: time unused?
	// TODO: gyro is a different rate to the others, should they be separated
	if (g[0] != 0 || g[1] != 0 || g[2] != 0) { // ignore zeroed gyro
		vqf_update_gyro(g, time);
	}
	vqf_update_accel(a, time);
#if CONFIG_SENSOR_USE_MAGNETOMETER
	vqf_update_mag(m, time);
#else
	(void)m;
#endif
}

void vqf_get_gyro_bias(float *g_off)
{
	getBiasEstimate(&state, &coeffs, g_off);
	// VQF internal unit is rad/s, fusion interface expects deg/s
	for (int i = 0; i < 3; i++) {
		g_off[i] *= RAD_TO_DEG;
	}
}

void vqf_set_gyro_bias(float *g_off)
{
	if (!vqf_vec3_finite(g_off)) {
		return;
	}
	float g_off_rad[3];
	// fusion interface receives values in deg/s, VQF requires rad/s
	for (int i = 0; i < 3; i++) {
		g_off_rad[i] = g_off[i] * DEG_TO_RAD;
	}
	setBiasEstimate(&state, g_off_rad, -1);
#if IS_ENABLED(CONFIG_SENSOR_VQF_ADAPTIVE_BIAS)
	vqf_reset_bias_policy();
#endif
#if IS_ENABLED(CONFIG_SENSOR_VQF_FAST_REST_RECOVERY)
	stop_macro_restore(true);
#endif
}

void vqf_rebase_gyro_bias(const float delta_dps[3])
{
	if (!vqf_vec3_finite(delta_dps)) {
		return;
	}
	/* Change input coordinates, not the physical bias estimate. Do not use
	 * setBiasEstimate(), reset covariance, or clip the translated residual. */
	vqf_real_t delta[3];
	for (int i = 0; i < 3; i++) {
		delta[i] = delta_dps[i] * DEG_TO_RAD;
		state.bias[i] += delta[i];
		state.restLastGyrLp[i] += delta[i];
	}

	/* filterVec initially stores [NaN, count, sums...], then switches to
	 * transposed direct-form II pairs. No samples means all-NaN: leave it
	 * untouched so the first sample initializes in the new coordinates.
	 * Use the bit-based finite check even for the native NaN markers. */
	if (!vqf_float_finite((float)state.restGyrLpState[0])) {
		if (vqf_float_finite((float)state.restGyrLpState[1])) {
			for (int i = 0; i < 3; i++) {
				state.restGyrLpState[2 + i] += state.restGyrLpState[1] * (vqf_double_t)delta[i];
			}
		}
	} else {
		for (int i = 0; i < 3; i++) {
			state.restGyrLpState[2 * i] += (vqf_double_t)delta[i] * ((vqf_double_t)1 - coeffs.restGyrLpB[0]);
			state.restGyrLpState[2 * i + 1] += (vqf_double_t)delta[i] * (coeffs.restGyrLpB[2] - coeffs.restGyrLpA[1]);
		}
	}

	/* LP(R*b) becomes LP(R*b) + LP(R)*delta. Both native filters advance
	 * together, including their averaging/count initialization. Translate
	 * the two earth-horizontal rows using their actual R history, not the
	 * current attitude or a constant steady-state approximation. */
	if (!vqf_float_finite((float)state.motionBiasEstBiasLpState[0])) {
		if (vqf_float_finite((float)state.motionBiasEstBiasLpState[1])) {
			for (int row = 0; row < 2; row++) {
				for (int col = 0; col < 3; col++) {
					state.motionBiasEstBiasLpState[2 + row] +=
						state.motionBiasEstRLpState[2 + 3 * row + col] * (vqf_double_t)delta[col];
				}
			}
		}
	} else {
		for (int row = 0; row < 2; row++) {
			for (int col = 0; col < 3; col++) {
				for (int k = 0; k < 2; k++) {
					state.motionBiasEstBiasLpState[2 * row + k] +=
						state.motionBiasEstRLpState[2 * (3 * row + col) + k] * (vqf_double_t)delta[col];
				}
			}
		}
	}
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
	for (int row = 0; row < 2; row++) {
		state.motionBiasEstLastBiasLp[row] += state.motionBiasEstLastR[3 * row] * delta[0]
			+ state.motionBiasEstLastR[3 * row + 1] * delta[1]
			+ state.motionBiasEstLastR[3 * row + 2] * delta[2];
	}
#endif
#if IS_ENABLED(CONFIG_SENSOR_VQF_ADAPTIVE_BIAS)
	vqf_rebase_bias_policy(delta);
#endif
#if IS_ENABLED(CONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION)
	/* Pure input-coordinate translation: keep the integrated corrected rate
	 * and one-shot observation. Physical replacement/discontinuity paths still
	 * discard history; queued owner-reset notifications remain untouched. */
	for (int i = 0; i < 3; i++) {
		if (cycle_bias_policy.started) {
			cycle_bias_policy.b0[i] += delta[i];
		}
		if (cycle_bias_policy.agree > 0 || cycle_bias_policy.locked) {
			cycle_bias_policy.obs[i] += delta[i];
		}
	}
#endif
}

void vqf_get_lin_a(float *lin_a)
{
	float q[4] = {0};
	vqf_get_quat(q);

	float vec_gravity[3] = {0};
	vec_gravity[0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
	vec_gravity[1] = 2.0f * (q[2] * q[3] + q[0] * q[1]);
	vec_gravity[2] = 2.0f * (q[0] * q[0] - 0.5f + q[3] * q[3]);

	//	float *a = state.lastAccLp; // not usable, rotated by inertial frame
	float *a = last_a;
	for (int i = 0; i < 3; i++) {
		lin_a[i] = a[i] - vec_gravity[i] * CONST_EARTH_GRAVITY; // gravity vector to m/s^2 before subtracting
	}
}

void vqf_get_quat(float *q)
{
#if IS_ENABLED(CONFIG_SENSOR_VQF_OUTPUT_OBSERVER)
#if CONFIG_SENSOR_USE_MAGNETOMETER
	/* The output inclination has its own yaw gauge: native delta cannot be
	 * prepended safely. Preserve the entire native frame for runtime 9D. */
	if (sensor_get_mag_available() && sensor_get_mag_enabled()) {
		getQuat9D(&state, q);
		return;
	}
#endif
	getOutputQuat6D(&output_observer, &state, q);
#elif CONFIG_SENSOR_USE_MAGNETOMETER
	getQuat9D(&state, q);
#else
	getQuat6D(&state, q);
#endif
}

bool vqf_get_rest_detected(void)
{
	return vqf_hard_rest_detected();
}

static bool vqf_take_rest_observation(bool *out)
{
	bool pending = rest_observation_pending;
	rest_observation_pending = false;
	if (pending && out) {
		*out = vqf_get_rest_detected();
	}
	return pending;
}

#if CONFIG_SENSOR_USE_MAGNETOMETER
bool vqf_get_mag_dist_detected(void)
{
	return getMagDistDetected(&state);
}

static void vqf_rebase_mag(float norm, float dip)
{
	setMagRef(&state, norm, dip);
	state.magDistDetected = true;
	state.magUndisturbedT = 0.0f;
	state.magRejectT = 0.0f;
	state.magCandidateNorm = -1.0f;
	state.magCandidateDip = 0.0f;
	state.magCandidateT = 0.0f;
	state.magNormDip[0] = state.magNormDip[1] = 0.0f;
	for (unsigned i = 0; i < 4; i++) {
		state.magNormDipLpState[i] = NAN;
	}
	state.kMagInit = 0.0f;
	state.lastMagDisAngle = 0.0f;
	state.lastMagCorrAngularRate = 0.0f;
	state.lastMagTsUs = 0;
}
#endif

static void vqf_get_quat6(float *q)
{
	getQuat6D(&state, q);
}

#if CONFIG_SENSOR_USE_MAGNETOMETER
void vqf_get_mag_ref(float *norm, float *dip)
{
	*norm = getMagRefNorm(&state);
	*dip = getMagRefDip(&state);
}
#endif

void vqf_get_debug_info(vqf_debug_info_t *info)
{
	if (!info) {
		return;
	}

	info->rest_detected = vqf_hard_rest_detected();
	getRelativeRestDeviations(&params, &state, info->rest_deviations);
	info->bias_sigma = getBiasEstimate(&state, &coeffs, info->bias);

#if CONFIG_SENSOR_USE_MAGNETOMETER
	// Heading correction state
	info->delta = getDelta(&state);

	// Magnetic disturbance / reference
	info->mag_dist_detected = getMagDistDetected(&state);
	info->mag_ref_norm = getMagRefNorm(&state);
	info->mag_ref_dip = getMagRefDip(&state);

	// Current magnetic field (after optional magCurrentTau LPF)
	info->mag_norm = state.magNormDip[0];
	info->mag_dip = state.magNormDip[1];

	// Heading correction diagnostics (from last magnetometer update)
	info->mag_dis_angle = state.lastMagDisAngle;
	info->mag_corr_rate = state.lastMagCorrAngularRate;

	// Disturbance rejection timers
	info->mag_undisturbed_t = state.magUndisturbedT;
	info->mag_reject_t = state.magRejectT;

	// Candidate field tracking
	info->mag_candidate_norm = state.magCandidateNorm;
	info->mag_candidate_dip = state.magCandidateDip;
	info->mag_candidate_t = state.magCandidateT;

	// Filter gains
	info->mag_k = coeffs.kMag;
	info->mag_k_init = state.kMagInit;
#else
	info->delta = 0.0f;
	info->mag_dist_detected = false;
	info->mag_ref_norm = info->mag_ref_dip = info->mag_norm = info->mag_dip = 0.0f;
	info->mag_dis_angle = info->mag_corr_rate = 0.0f;
	info->mag_undisturbed_t = info->mag_reject_t = 0.0f;
	info->mag_candidate_norm = info->mag_candidate_dip = info->mag_candidate_t = 0.0f;
	info->mag_k = info->mag_k_init = 0.0f;
#endif

	// Convert bias from rad/s to °/s
	for (int i = 0; i < 3; i++) {
		info->bias[i] *= RAD_TO_DEG;
	}
	info->bias_sigma *= RAD_TO_DEG;

#if CONFIG_SENSOR_USE_MAGNETOMETER
	// Convert rad-based angles to degrees
	info->delta *= RAD_TO_DEG;
	info->mag_ref_dip *= RAD_TO_DEG;
	info->mag_dip *= RAD_TO_DEG;
	info->mag_dis_angle *= RAD_TO_DEG;

	// Convert angular rates from rad/s to °/s
	info->mag_corr_rate *= RAD_TO_DEG;

	// Convert candidate dip from rad to degrees
	info->mag_candidate_dip *= RAD_TO_DEG;
#endif

#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
	// Adaptive tauAcc state
	info->tau_acc = params.tauAcc;
	info->acc_deviation = adaptive_acc_signal / (adaptive_acc_signal + ADAPTIVE_ACC_SCALE);
	info->angular_speed = adaptive_gyr_signal / (adaptive_gyr_signal + ADAPTIVE_GYR_SCALE);
#endif

	// Rest detection diagnostics
	info->rest_enter_count = rest_enter_count;
	info->rest_exit_count = rest_exit_count;
	info->rest_total_s = rest_total_s;
	info->rest_last_duration_s = rest_last_duration_s;
	info->uptime_s = uptime_s;

	// Copy rest event log (oldest first)
	info->rest_event_count = rest_event_total;
	for (uint8_t i = 0; i < REST_EVENT_LOG_SIZE; i++) {
		uint8_t src;
		if (rest_event_total >= REST_EVENT_LOG_SIZE) {
			src = (rest_event_idx + i) % REST_EVENT_LOG_SIZE;
		} else {
			src = i;
		}
		info->rest_events[i].time_s = rest_event_log[src].time_s;
		info->rest_events[i].entered = rest_event_log[src].entered;
	}

	// Kalman filter P diagonal
	info->biasP[0] = state.biasP[0];
	info->biasP[1] = state.biasP[4];
	info->biasP[2] = state.biasP[8];
}

#if defined(CONFIG_VQF_BENCH)
static ALWAYS_INLINE void vqf_bench_timer_prepare(void)
{
#if defined(CONFIG_CPU_CORTEX_M_HAS_DWT)
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
}

static ALWAYS_INLINE uint32_t vqf_bench_timer_begin(unsigned int *irq_key)
{
	*irq_key = irq_lock();
#if defined(CONFIG_CPU_CORTEX_M_HAS_DWT)
	DWT->CYCCNT = 0;
	return 0;
#else
	return k_cycle_get_32();
#endif
}

static ALWAYS_INLINE uint32_t vqf_bench_timer_end(unsigned int irq_key, uint32_t start_cycles)
{
#if defined(CONFIG_CPU_CORTEX_M_HAS_DWT)
	uint32_t elapsed_cycles = DWT->CYCCNT;
	irq_unlock(irq_key);
	return elapsed_cycles;
#else
	uint32_t elapsed_cycles = k_cycle_get_32() - start_cycles;
	irq_unlock(irq_key);
	return elapsed_cycles;
#endif
}

static ALWAYS_INLINE uint32_t vqf_bench_timer_hz(void)
{
#if defined(CONFIG_CPU_CORTEX_M_HAS_DWT)
	return SystemCoreClock ? SystemCoreClock : CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC;
#else
	return CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC;
#endif
}

static void vqf_bench_print_stats(const char *name, uint32_t iterations, uint32_t total_cycles, uint32_t timer_hz)
{
	uint32_t avg_cycles_int = 0;
	uint32_t avg_cycles_frac = 0;
	uint32_t total_us_int = 0;
	uint32_t total_us_frac = 0;
	uint32_t avg_us_int = 0;
	uint32_t avg_us_frac = 0;

	if (iterations) {
		uint64_t avg_cycles_x1000 = (total_cycles * 1000ULL) / iterations;
		avg_cycles_int = (uint32_t)(avg_cycles_x1000 / 1000ULL);
		avg_cycles_frac = (uint32_t)(avg_cycles_x1000 % 1000ULL);
	}

	if (timer_hz) {
		uint64_t total_us_x1000 = (total_cycles * 1000000000ULL) / timer_hz;
		total_us_int = (uint32_t)(total_us_x1000 / 1000ULL);
		total_us_frac = (uint32_t)(total_us_x1000 % 1000ULL);

		if (iterations) {
			uint64_t avg_us_x1000 = total_us_x1000 / iterations;
			avg_us_int = (uint32_t)(avg_us_x1000 / 1000ULL);
			avg_us_frac = (uint32_t)(avg_us_x1000 % 1000ULL);
		}
	}

	printk(
		"  %-10s t_c=%10u avg_c=%4u.%03u t_us=%8u.%03u avg_us=%4u.%03u\n",
		name,
		total_cycles,
		avg_cycles_int,
		avg_cycles_frac,
		total_us_int,
		total_us_frac,
		avg_us_int,
		avg_us_frac
	);
}

#define VQF_BENCH_BATCH_SIZE 16U

typedef enum {
	VQF_BENCH_UPDATE_GYR,
	VQF_BENCH_UPDATE_ACC,
#if CONFIG_SENSOR_USE_MAGNETOMETER
	VQF_BENCH_UPDATE_MAG,
	VQF_BENCH_GET_QUAT9D,
#endif
} vqf_bench_op_t;

static uint32_t vqf_bench_measure(
	vqf_bench_op_t op,
	uint32_t iterations,
	const vqf_real_t gyr_samples[][3],
	const vqf_real_t acc_samples[][3],
	const vqf_real_t mag_samples[][3],
	size_t sample_count,
	vqf_params_t *bench_params,
	vqf_state_t *bench_state,
	vqf_coeffs_t *bench_coeffs,
	float quat[4]
)
{
#if !CONFIG_SENSOR_USE_MAGNETOMETER
	(void)mag_samples;
	(void)quat;
#endif
	uint32_t total_cycles = 0;
	uint32_t remaining = iterations;
	uint32_t sample_offset = 0;

	while (remaining > 0) {
		uint32_t batch_len = remaining > VQF_BENCH_BATCH_SIZE ? VQF_BENCH_BATCH_SIZE : remaining;
		unsigned int irq_key;
		uint32_t start_cycles = vqf_bench_timer_begin(&irq_key);

		for (uint32_t i = 0; i < batch_len; i++, sample_offset++) {
			size_t sample_idx = sample_offset % sample_count;

			switch (op) {
			case VQF_BENCH_UPDATE_GYR:
				updateGyr(bench_params, bench_state, bench_coeffs, gyr_samples[sample_idx]);
				break;
			case VQF_BENCH_UPDATE_ACC:
				updateAcc(bench_params, bench_state, bench_coeffs, acc_samples[sample_idx]);
				break;
#if CONFIG_SENSOR_USE_MAGNETOMETER
			case VQF_BENCH_UPDATE_MAG:
				updateMag(bench_params, bench_state, bench_coeffs, mag_samples[sample_idx]);
				break;
			case VQF_BENCH_GET_QUAT9D:
				getQuat9D(bench_state, quat);
				vqf_bench_sink += quat[sample_offset & 3U];
				break;
#endif
			}
		}

		total_cycles += vqf_bench_timer_end(irq_key, start_cycles);
		remaining -= batch_len;
	}

	return total_cycles;
}

void vqf_run_benchmark(uint32_t iterations)
{
	vqf_bench_sink = 0.0f;
	uint32_t timer_hz;
	uint32_t elapsed_cycles;
	k_tid_t bench_thread = k_current_get();
	int bench_thread_prio = k_thread_priority_get(bench_thread);
	bool bench_prio_changed = false;

	const vqf_real_t (*mag_samples)[3] = NULL;
#if CONFIG_SENSOR_USE_MAGNETOMETER
	mag_samples = vqf_bench_mag_samples;
#endif

	if (iterations == 0) {
		iterations = 1000;
	}

	if (bench_thread_prio < VQF_BENCH_THREAD_PRIORITY) {
		k_thread_priority_set(bench_thread, VQF_BENCH_THREAD_PRIORITY);
		bench_prio_changed = true;
	}

	set_params();
	vqf_bench_params = params;
	initVqf(&vqf_bench_params, &vqf_bench_state, &vqf_bench_coeffs, 0.001f, 0.001f, 0.01f);

	for (size_t i = 0; i < vqf_bench_sample_count * 8; i++) {
		const size_t idx = i % vqf_bench_sample_count;
		updateGyr(&vqf_bench_params, &vqf_bench_state, &vqf_bench_coeffs, vqf_bench_gyr_samples[idx]);
		updateAcc(&vqf_bench_params, &vqf_bench_state, &vqf_bench_coeffs, vqf_bench_acc_samples[idx]);
#if CONFIG_SENSOR_USE_MAGNETOMETER
		updateMag(&vqf_bench_params, &vqf_bench_state, &vqf_bench_coeffs, vqf_bench_mag_samples[idx]);
		getQuat9D(&vqf_bench_state, vqf_bench_quat);
#else
		getQuat6D(&vqf_bench_state, vqf_bench_quat);
#endif
		vqf_bench_sink += vqf_bench_quat[0];
	}

	vqf_bench_warm_params = vqf_bench_params;
	vqf_bench_warm_state = vqf_bench_state;
	vqf_bench_warm_coeffs = vqf_bench_coeffs;
	vqf_bench_timer_prepare();
	timer_hz = vqf_bench_timer_hz();

	printk(
		"VQF benchmark (%u iterations, CMSIS-DSP=%s, timer=%s)\n",
		iterations,
#ifdef CONFIG_CMSIS_DSP
		"on"
#else
		"off"
#endif
		,
#if defined(CONFIG_CPU_CORTEX_M_HAS_DWT)
		"DWT CYCCNT"
#else
		"system timer"
#endif
	);

	vqf_bench_params = vqf_bench_warm_params;
	vqf_bench_state = vqf_bench_warm_state;
	vqf_bench_coeffs = vqf_bench_warm_coeffs;
	elapsed_cycles = vqf_bench_measure(
		VQF_BENCH_UPDATE_GYR,
		iterations,
		vqf_bench_gyr_samples,
		vqf_bench_acc_samples,
		mag_samples,
		vqf_bench_sample_count,
		&vqf_bench_params,
		&vqf_bench_state,
		&vqf_bench_coeffs,
		vqf_bench_quat
	);
	vqf_bench_print_stats("updateGyr", iterations, elapsed_cycles, timer_hz);

	vqf_bench_params = vqf_bench_warm_params;
	vqf_bench_state = vqf_bench_warm_state;
	vqf_bench_coeffs = vqf_bench_warm_coeffs;
	elapsed_cycles = vqf_bench_measure(
		VQF_BENCH_UPDATE_ACC,
		iterations,
		vqf_bench_gyr_samples,
		vqf_bench_acc_samples,
		mag_samples,
		vqf_bench_sample_count,
		&vqf_bench_params,
		&vqf_bench_state,
		&vqf_bench_coeffs,
		vqf_bench_quat
	);
	vqf_bench_print_stats("updateAcc", iterations, elapsed_cycles, timer_hz);

#if CONFIG_SENSOR_USE_MAGNETOMETER
	vqf_bench_params = vqf_bench_warm_params;
	vqf_bench_state = vqf_bench_warm_state;
	vqf_bench_coeffs = vqf_bench_warm_coeffs;
	elapsed_cycles = vqf_bench_measure(
		VQF_BENCH_UPDATE_MAG,
		iterations,
		vqf_bench_gyr_samples,
		vqf_bench_acc_samples,
		mag_samples,
		vqf_bench_sample_count,
		&vqf_bench_params,
		&vqf_bench_state,
		&vqf_bench_coeffs,
		vqf_bench_quat
	);
	vqf_bench_print_stats("updateMag", iterations, elapsed_cycles, timer_hz);

	vqf_bench_params = vqf_bench_warm_params;
	vqf_bench_state = vqf_bench_warm_state;
	vqf_bench_coeffs = vqf_bench_warm_coeffs;
	elapsed_cycles = vqf_bench_measure(
		VQF_BENCH_GET_QUAT9D,
		iterations,
		vqf_bench_gyr_samples,
		vqf_bench_acc_samples,
		mag_samples,
		vqf_bench_sample_count,
		&vqf_bench_params,
		&vqf_bench_state,
		&vqf_bench_coeffs,
		vqf_bench_quat
	);
	vqf_bench_print_stats("getQuat9D", iterations, elapsed_cycles, timer_hz);
#endif

	printk("  checksum: %.6f\n", (double)vqf_bench_sink);

	if (bench_prio_changed) {
		k_thread_priority_set(bench_thread, bench_thread_prio);
	}
}
#endif

const sensor_fusion_t sensor_fusion_vqf = {
	.init = vqf_init,
	.load = vqf_load,
	.save = vqf_save,

	.update_gyro = vqf_update_gyro,
	.update_accel = vqf_update_accel,
#if CONFIG_SENSOR_USE_MAGNETOMETER
	.update_mag = vqf_update_mag,
#endif
	.update = vqf_update,

	.get_gyro_bias = vqf_get_gyro_bias,
	.set_gyro_bias = vqf_set_gyro_bias,
	.rebase_gyro_bias = vqf_rebase_gyro_bias,

	.get_lin_a = vqf_get_lin_a,
	.get_quat = vqf_get_quat,

	.get_rest_detected = vqf_get_rest_detected,
	.take_rest_observation = vqf_take_rest_observation,
#if CONFIG_SENSOR_USE_MAGNETOMETER
	.get_mag_dist_detected = vqf_get_mag_dist_detected,
#endif
	.get_quat6 = vqf_get_quat6,
#if CONFIG_SENSOR_USE_MAGNETOMETER
	.rebase_mag = vqf_rebase_mag,
	.get_mag_ref = vqf_get_mag_ref,
#endif
};
