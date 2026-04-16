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
#include <zephyr/kernel.h>

#include "../src/vqf.h" // conflicting with vqf.h in local path

#include "../vqf/vqf.h" // conflicting with vqf.h in vqf-c

#include "retained.h" // for BUILD_ASSERT on fusion_data size

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (M_PI / 180.0f)
#endif

#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
/* ---------- Adaptive tauAcc configuration ---------- */
/*
 * Three-regime adaptive tauAcc strategy:
 *
 * 1. REST (VQF rest detection active):
 *    tauAcc = TAU_ACC_REST.
 *    At true rest the accelerometer perfectly reflects gravity,
 *    so a small tau enables fast inclination convergence while
 *    the rest bias estimator handles gyro drift.
 *
 * 2. GENTLE MOTION (accel close to gravity, low linear acceleration):
 *    tauAcc = TAU_ACC_GENTLE (high).
 *    Micro-movements (breathing, subtle swaying) contaminate the
 *    accelerometer slightly.  A high tau avoids these perturbations
 *    being coupled into heading via the bias estimator R-matrix.
 *
 * 3. AGGRESSIVE MOTION (significant linear acceleration):
 *    tauAcc = TAU_ACC_AGGRESSIVE (lower).
 *    Active deliberate motion—fast inclination correction helps
 *    prevent pitch/roll drift from coupling into heading.
 *
 * The accel deviation |‖a‖ - g| drives a [0,1] motion_intensity
 * with fast-attack / slow-release dynamics.  During motion, tauAcc
 * is linearly interpolated from GENTLE (0) to AGGRESSIVE (1).
 * When rest is detected, TAU_ACC_REST overrides.
 */
#define ADAPTIVE_TAU_ACC_REST       6.0f   /* tauAcc when at rest (seconds) */
#define ADAPTIVE_TAU_ACC_GENTLE     5.8f   /* tauAcc during gentle motion (seconds) */
#define ADAPTIVE_TAU_ACC_AGGRESSIVE 3.0f   /* tauAcc under aggressive motion (seconds) */
#define ADAPTIVE_TAU_ACC_LEVELS     14      /* quantization levels */
#define ADAPTIVE_ACC_DEV_TH         3.0f   /* accel deviation threshold (m/s²) */
#define ADAPTIVE_ATTACK_ALPHA       0.15f  /* fast attack coefficient (per sample) */
#define ADAPTIVE_RELEASE_ALPHA      0.008f /* slow release coefficient (per sample) */

/*
 * tauAcc transition smoothing: prevents Butterworth LP filter transients
 * when switching between motion regimes (e.g. rest→aggressive).
 * Uses asymmetric exponential smoothing: slow decrease, fast increase.
 *
 * At 100 Hz accel ODR (CONFIG_SENSOR_ACCEL_ODR):
 *   ALPHA_DOWN = 0.004 → τ ≈ 2.5s, 95% settling ≈ 7.5s
 *   ALPHA_UP   = 0.008 → τ ≈ 1.25s, 95% settling ≈ 3.75s
 */
#define TAU_SMOOTH_ALPHA_DOWN  0.004f  /* tauAcc decrease smoothing (per sample) */
#define TAU_SMOOTH_ALPHA_UP    0.008f  /* tauAcc increase smoothing (per sample) */
#endif /* CONFIG_VQF_ADAPTIVE_TAU_ACC */

static uint8_t imu_id;

static vqf_params_t params;
static vqf_state_t state;
static vqf_coeffs_t coeffs;
#define VQF_MEM_SIZE (sizeof(vqf_state_t) + sizeof(vqf_coeffs_t))

static float last_a[3] = {0};
static volatile float vqf_bench_sink;

#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
/* Adaptive tauAcc state */
static float motion_intensity;       /* low-pass filtered motion intensity [0,1] */
static float current_tau_level = -1; /* current quantized level (-1 = unset) */
static float smoothed_tau;           /* smoothed tauAcc for gradual transitions */
#endif

/* Rest detection diagnostics */
static uint32_t rest_enter_count;
static uint32_t rest_exit_count;
static float rest_total_s;
static float rest_last_enter_time;   /* uptime when last rest started */
static float rest_last_duration_s;
static float uptime_s;
static bool prev_rest_detected;

/* Circular rest event log */
#define REST_EVENT_LOG_SIZE 5
static struct {
	float time_s;
	bool entered;
} rest_event_log[REST_EVENT_LOG_SIZE];
static uint8_t rest_event_idx;  /* next write position */
static uint8_t rest_event_total; /* total events (up to log size) */

void vqf_update_sensor_ids(int imu)
{
	imu_id = imu;
}

static void set_params()
{
	init_params(&params);
	params.tauAcc = 3.6f;
	params.biasClip = 2.0f;
	params.biasForgettingTime = 100.0f;
	params.biasSigmaInit = 1.2f;
	params.biasSigmaMotion = 0.15f;
	params.biasSigmaRest = 0.03f;
	params.biasVerticalForgettingFactor = 0.0001f;
	params.motionBiasEstEnabled = true;
	params.restBiasEstEnabled = true;
	params.restFilterTau = 1.8f;
	params.restMinT = 2.8f;
	params.restThAcc = 0.20f;
	params.restThGyr = 0.65f;
	params.magDistRejectionEnabled = true;
	params.tauMag = 12.0f;
	params.magCurrentTau = 0.20f;
	params.magNormTh = 0.09f;
	params.magDipTh = 9.0f;
	params.magNewFirstTime = 6.0f;
	params.magNewMinGyr = 15.0f;
	params.magMinUndisturbedTime = 0.8f;
	params.magMaxRejectionTime = 1800.0f;
	params.magRejectionFactor = 900.0f;
}

void vqf_init(float g_time, float a_time, float m_time)
{
	set_params();
	initVqf(&params, &state, &coeffs, g_time, a_time, m_time);
#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
	motion_intensity = 0.0f;
	current_tau_level = -1.0f;
	smoothed_tau = params.tauAcc;
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

void vqf_load(const void *data)
{
	BUILD_ASSERT(VQF_MEM_SIZE <= sizeof(((struct retained_data *)0)->fusion_data),
		     "VQF state+coeffs exceeds fusion_data buffer in retained memory");
	set_params();
	memcpy(&state, data, sizeof(state));
	memcpy(&coeffs, (uint8_t *)data + sizeof(state), sizeof(coeffs));
#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
	motion_intensity = 0.0f;
	current_tau_level = -1.0f;
	smoothed_tau = params.tauAcc;
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
	BUILD_ASSERT(VQF_MEM_SIZE <= sizeof(((struct retained_data *)0)->fusion_data),
		     "VQF state+coeffs exceeds fusion_data buffer in retained memory");
	memcpy(data, &state, sizeof(state));
	memcpy((uint8_t *)data + sizeof(state), &coeffs, sizeof(coeffs));
}

void vqf_update_gyro(float *g, float time)
{
	ARG_UNUSED(time);
	float g_rad[3] = {0};
	// g is in deg/s, convert to rad/s
	for (int i = 0; i < 3; i++)
		g_rad[i] = g[i] * DEG_TO_RAD;
	updateGyr(&params, &state, &coeffs, g_rad);
}

void vqf_update_gyro_ts(float *g, uint64_t timestamp_us)
{
	float g_rad[3] = {0};
	for (int i = 0; i < 3; i++)
		g_rad[i] = g[i] * DEG_TO_RAD;
	updateGyrTs(&params, &state, &coeffs, g_rad, timestamp_us);
}

#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
/**
 * @brief Pre-accel-update processing: adaptive tauAcc (experimental).
 *
 * Called before each accelerometer update to adjust VQF parameters based on
 * current motion intensity and temperature dynamics.
 *
 * @param a_m_s2 accelerometer reading in m/s²
 */
static void vqf_pre_accel_update(const float a_m_s2[3])
{
	/* --- Adaptive tauAcc based on motion intensity --- */
	float a_norm = sqrtf(a_m_s2[0] * a_m_s2[0] +
			     a_m_s2[1] * a_m_s2[1] +
			     a_m_s2[2] * a_m_s2[2]);
	float a_dev = fabsf(a_norm - CONST_EARTH_GRAVITY);
	float alpha_inst = fminf(a_dev / ADAPTIVE_ACC_DEV_TH, 1.0f);

	/* Attack-release envelope: fast increase, slow decrease */
	if (alpha_inst > motion_intensity) {
		motion_intensity += ADAPTIVE_ATTACK_ALPHA * (alpha_inst - motion_intensity);
	} else {
		motion_intensity += ADAPTIVE_RELEASE_ALPHA * (alpha_inst - motion_intensity);
	}

	/*
	 * Compute target tauAcc:
	 * - Rest detected: use rest tauAcc
	 * - Motion: linearly interpolate from GENTLE (intensity=0) to
	 *   AGGRESSIVE (intensity=1).  This supports both increasing and
	 *   decreasing curves depending on how the defines are set.
	 */
	float target_tau;
	if (state.restDetected) {
		target_tau = ADAPTIVE_TAU_ACC_REST;
	} else {
		target_tau = ADAPTIVE_TAU_ACC_GENTLE +
			     (ADAPTIVE_TAU_ACC_AGGRESSIVE - ADAPTIVE_TAU_ACC_GENTLE) *
				     motion_intensity;
	}

	/*
	 * Smooth tauAcc transitions to avoid Butterworth LP filter transients.
	 * Decrease (rest→motion) is slow to prevent heading coupling from the
	 * sudden acc correction gain change.  Increase (motion→rest) is fast
	 * so the filter settles quickly at rest.
	 */
	float tau_alpha = (target_tau < smoothed_tau) ? TAU_SMOOTH_ALPHA_DOWN
						      : TAU_SMOOTH_ALPHA_UP;
	smoothed_tau += tau_alpha * (target_tau - smoothed_tau);

	/*
	 * Quantize target_tau directly to avoid unnecessary setTauAcc() calls.
	 * The step size is derived from the full possible range of tauAcc values.
	 */
	float tau_min = fminf(fminf(ADAPTIVE_TAU_ACC_REST, ADAPTIVE_TAU_ACC_GENTLE),
			      ADAPTIVE_TAU_ACC_AGGRESSIVE);
	float tau_max = fmaxf(fmaxf(ADAPTIVE_TAU_ACC_REST, ADAPTIVE_TAU_ACC_GENTLE),
			      ADAPTIVE_TAU_ACC_AGGRESSIVE);
	float tau_step = (tau_max - tau_min) / ADAPTIVE_TAU_ACC_LEVELS;
	float quantized_tau;
	if (tau_step > 0.001f) {
		quantized_tau = tau_min +
				roundf((smoothed_tau - tau_min) / tau_step) * tau_step;
		quantized_tau = fmaxf(tau_min, fminf(quantized_tau, tau_max));
	} else {
		quantized_tau = smoothed_tau;
	}

	if (quantized_tau != current_tau_level) {
		current_tau_level = quantized_tau;
		setTauAcc(&params, &state, &coeffs, quantized_tau);
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

	bool cur = state.restDetected;
	if (cur && !prev_rest_detected) {
		/* entering rest */
		rest_enter_count++;
		rest_last_enter_time = uptime_s;
		rest_event_log[rest_event_idx].time_s = uptime_s;
		rest_event_log[rest_event_idx].entered = true;
		rest_event_idx = (rest_event_idx + 1) % REST_EVENT_LOG_SIZE;
		if (rest_event_total < REST_EVENT_LOG_SIZE)
			rest_event_total++;
	} else if (!cur && prev_rest_detected) {
		/* leaving rest */
		rest_exit_count++;
		rest_last_duration_s = uptime_s - rest_last_enter_time;
		rest_event_log[rest_event_idx].time_s = uptime_s;
		rest_event_log[rest_event_idx].entered = false;
		rest_event_idx = (rest_event_idx + 1) % REST_EVENT_LOG_SIZE;
		if (rest_event_total < REST_EVENT_LOG_SIZE)
			rest_event_total++;
	}
	if (cur) {
		rest_total_s += dt;
	}
	prev_rest_detected = cur;
}

void vqf_update_accel(float *a, float time)
{
	ARG_UNUSED(time);
	float a_m_s2[3] = {0};
	// a is in g, convert to m/s^2
	for (int i = 0; i < 3; i++)
		a_m_s2[i] = a[i] * CONST_EARTH_GRAVITY;
	if (a_m_s2[0] != 0 || a_m_s2[1] != 0 || a_m_s2[2] != 0)
		memcpy(last_a, a_m_s2, sizeof(a_m_s2));
#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
	vqf_pre_accel_update(a_m_s2);
#endif
	updateAcc(&params, &state, &coeffs, a_m_s2);
	vqf_track_rest_diag();
}

void vqf_update_accel_ts(float *a, uint64_t timestamp_us)
{
	float a_m_s2[3] = {0};
	for (int i = 0; i < 3; i++)
		a_m_s2[i] = a[i] * CONST_EARTH_GRAVITY;
	if (a_m_s2[0] != 0 || a_m_s2[1] != 0 || a_m_s2[2] != 0)
		memcpy(last_a, a_m_s2, sizeof(a_m_s2));
#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
	vqf_pre_accel_update(a_m_s2);
#endif
	updateAccTs(&params, &state, &coeffs, a_m_s2, timestamp_us);
	vqf_track_rest_diag();
}

void vqf_update_mag(float *m, float time)
{
	// Use the caller-supplied time step when valid so that VQF time accumulators
	// (magCandidateT, magRejectT, etc.) and gain k run at the correct real-time
	// rate even when the sensor loop runs faster or slower than the mag ODR.
	//
	// Build a synthetic cumulative microsecond timestamp from the elapsed time so
	// updateMagTs can derive the correct dt internally (avoids calling the static
	// updateMag_internal directly).
	if (time > 0.0f && time < 10.0f) {
		uint64_t synth_ts = state.lastMagTsUs + (uint64_t)(time * 1e6f);
		if (synth_ts == 0)
			synth_ts = 1; // avoid the "uninitialized" sentinel value
		updateMagTs(&params, &state, &coeffs, m, synth_ts);
	} else {
		updateMag(&params, &state, &coeffs, m);
	}
}

void vqf_update_mag_ts(float *m, uint64_t timestamp_us)
{
	updateMagTs(&params, &state, &coeffs, m, timestamp_us);
}

void vqf_update(float *g, float *a, float *m, float time)
{
	// TODO: time unused?
	// TODO: gyro is a different rate to the others, should they be separated
	if (g[0] != 0 || g[1] != 0 || g[2] != 0) // ignore zeroed gyro
		vqf_update_gyro(g, time);
	vqf_update_accel(a, time);
	vqf_update_mag(m, time);
}

void vqf_get_gyro_bias(float *g_off)
{
	getBiasEstimate(&state, &coeffs, g_off);
	// VQF internal unit is rad/s, fusion interface expects deg/s
	for (int i = 0; i < 3; i++)
		g_off[i] *= 180.0f / M_PI;
}

void vqf_set_gyro_bias(float *g_off)
{
	float g_off_rad[3];
	// fusion interface receives values in deg/s, VQF requires rad/s
	for (int i = 0; i < 3; i++)
		g_off_rad[i] = g_off[i] * DEG_TO_RAD;
	setBiasEstimate(&state, g_off_rad, -1);
}

void vqf_update_gyro_sanity(float *g, float *m)
{
	// TODO: does vqf tell us a "recovery state"
	return;
}

int vqf_get_gyro_sanity(void)
{
	// TODO: does vqf tell us a "recovery state"
	return 0;
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
	for (int i = 0; i < 3; i++)
		lin_a[i] = a[i] - vec_gravity[i] * CONST_EARTH_GRAVITY; // gravity vector to m/s^2 before subtracting
}

void vqf_get_quat(float *q)
{
	getQuat9D(&state, q);
}

bool vqf_get_rest_detected(void)
{
	return getRestDetected(&state);
}

void vqf_get_relative_rest_deviations(float *out)
{
	getRelativeRestDeviations(&params, &state, out);
}

void vqf_get_debug_info(vqf_debug_info_t *info)
{
	if (!info) return;

	info->rest_detected = getRestDetected(&state);
	getRelativeRestDeviations(&params, &state, info->rest_deviations);
	info->bias_sigma = getBiasEstimate(&state, &coeffs, info->bias);

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

	// Convert bias from rad/s to °/s
	for (int i = 0; i < 3; i++) {
		info->bias[i] *= 180.0f / M_PI;
	}
	info->bias_sigma *= 180.0f / M_PI;

	// Convert rad-based angles to degrees
	info->delta *= 180.0f / M_PI;
	info->mag_ref_dip *= 180.0f / M_PI;
	info->mag_dip *= 180.0f / M_PI;
	info->mag_dis_angle *= 180.0f / M_PI;

	// Convert angular rates from rad/s to °/s
	info->mag_corr_rate *= 180.0f / M_PI;

	// Convert candidate dip from rad to degrees
	info->mag_candidate_dip *= 180.0f / M_PI;

#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
	// Adaptive tauAcc state
	info->tau_acc = params.tauAcc;
	info->motion_intensity = motion_intensity;
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
		if (rest_event_total >= REST_EVENT_LOG_SIZE)
			src = (rest_event_idx + i) % REST_EVENT_LOG_SIZE;
		else
			src = i;
		info->rest_events[i].time_s = rest_event_log[src].time_s;
		info->rest_events[i].entered = rest_event_log[src].entered;
	}

	// Kalman filter P diagonal
	info->biasP[0] = state.biasP[0];
	info->biasP[1] = state.biasP[4];
	info->biasP[2] = state.biasP[8];
}

static uint32_t vqf_bench_elapsed_cycles(uint32_t start_cycles)
{
	return k_cycle_get_32() - start_cycles;
}

static void vqf_bench_print_stats(const char *name, uint32_t iterations, uint32_t total_cycles)
{
	uint32_t avg_cycles_int = 0;
	uint32_t avg_cycles_frac = 0;

	if (iterations) {
		uint64_t avg_cycles_x1000 = ((uint64_t)total_cycles * 1000ULL) / iterations;
		avg_cycles_int = (uint32_t)(avg_cycles_x1000 / 1000ULL);
		avg_cycles_frac = (uint32_t)(avg_cycles_x1000 % 1000ULL);
	}

	printk(
		"  %-10s total_cycles=%10u avg_cycles=%3u.%03u\n",
		name,
		total_cycles,
		avg_cycles_int,
		avg_cycles_frac
	);
}

void vqf_run_benchmark(uint32_t iterations)
{
	vqf_bench_sink = 0.0f;
	vqf_params_t bench_params;
	vqf_state_t bench_state;
	vqf_coeffs_t bench_coeffs;
	float quat[4];
	uint32_t start_cycles;
	uint32_t elapsed_cycles;

	if (iterations == 0) {
		iterations = 1000;
	}

	const vqf_real_t gyr_samples[][3] = {
		{0.12f, -0.34f, 0.56f},
		{-1.10f, 0.45f, 0.08f},
		{0.78f, 0.11f, -0.29f},
		{-0.05f, 0.92f, 0.37f},
	};
	const vqf_real_t acc_samples[][3] = {
		{0.30f, 0.10f, 9.70f},
		{-0.25f, 0.45f, 9.63f},
		{0.60f, -0.15f, 9.55f},
		{-0.40f, -0.20f, 9.81f},
	};
	const vqf_real_t mag_samples[][3] = {
		{0.32f, 0.05f, -0.41f},
		{0.28f, 0.10f, -0.39f},
		{0.35f, -0.02f, -0.43f},
		{0.30f, 0.08f, -0.40f},
	};
	const size_t sample_count = sizeof(gyr_samples) / sizeof(gyr_samples[0]);

	set_params();
	bench_params = params;
	initVqf(&bench_params, &bench_state, &bench_coeffs, 0.001f, 0.001f, 0.01f);

	for (size_t i = 0; i < sample_count * 8; i++) {
		const size_t idx = i % sample_count;
		updateGyr(&bench_params, &bench_state, &bench_coeffs, gyr_samples[idx]);
		updateAcc(&bench_params, &bench_state, &bench_coeffs, acc_samples[idx]);
		updateMag(&bench_params, &bench_state, &bench_coeffs, mag_samples[idx]);
		getQuat9D(&bench_state, quat);
		vqf_bench_sink += quat[0];
	}

	printk("VQF benchmark (%u iterations, CMSIS-DSP=%s)\n", iterations,
#ifdef CONFIG_CMSIS_DSP
		"on"
#else
		"off"
#endif
	);

	start_cycles = k_cycle_get_32();
	for (uint32_t i = 0; i < iterations; i++) {
		updateGyr(&bench_params, &bench_state, &bench_coeffs, gyr_samples[i % sample_count]);
	}
	elapsed_cycles = vqf_bench_elapsed_cycles(start_cycles);
	vqf_bench_print_stats("updateGyr", iterations, elapsed_cycles);

	start_cycles = k_cycle_get_32();
	for (uint32_t i = 0; i < iterations; i++) {
		updateAcc(&bench_params, &bench_state, &bench_coeffs, acc_samples[i % sample_count]);
	}
	elapsed_cycles = vqf_bench_elapsed_cycles(start_cycles);
	vqf_bench_print_stats("updateAcc", iterations, elapsed_cycles);

	start_cycles = k_cycle_get_32();
	for (uint32_t i = 0; i < iterations; i++) {
		updateMag(&bench_params, &bench_state, &bench_coeffs, mag_samples[i % sample_count]);
	}
	elapsed_cycles = vqf_bench_elapsed_cycles(start_cycles);
	vqf_bench_print_stats("updateMag", iterations, elapsed_cycles);

	start_cycles = k_cycle_get_32();
	for (uint32_t i = 0; i < iterations; i++) {
		getQuat9D(&bench_state, quat);
		vqf_bench_sink += quat[i & 3];
	}
	elapsed_cycles = vqf_bench_elapsed_cycles(start_cycles);
	vqf_bench_print_stats("getQuat9D", iterations, elapsed_cycles);

	printk("  checksum: %.6f\n", (double)vqf_bench_sink);
}

const sensor_fusion_t sensor_fusion_vqf = {
	vqf_init,
	vqf_load,
	vqf_save,

	vqf_update_gyro,
	vqf_update_accel,
	vqf_update_mag,
	vqf_update,

	vqf_get_gyro_bias,
	vqf_set_gyro_bias,

	vqf_update_gyro_sanity,
	vqf_get_gyro_sanity,

	vqf_get_lin_a,
	vqf_get_quat
};
