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

static vqf_params_t params;
static vqf_state_t state;
static vqf_coeffs_t coeffs;
/* Raw state layouts are not interchangeable across firmware revisions. */
typedef struct {
	uint32_t magic;
	uint16_t version;
	uint16_t size;
} vqf_saved_header_t;
#define VQF_SAVED_MAGIC 0x32465156u /* "VQF2" */
#define VQF_SAVED_VERSION 3u
#define VQF_MEM_SIZE (sizeof(vqf_saved_header_t) + sizeof(vqf_state_t) + sizeof(vqf_coeffs_t))
BUILD_ASSERT(sizeof(vqf_saved_header_t) == 8, "VQF retained header ABI changed");
BUILD_ASSERT(VQF_MEM_SIZE == 840, "VQF retained ABI changed");

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
	params.tauAcc = 3.294f;
	params.biasClip = 5.0f;
	params.biasForgettingTime = 235.58f;
	params.biasSigmaInit = 0.6585f;
	params.biasSigmaMotion = 0.1118f;
	params.biasSigmaRest = 0.0028f;
	params.biasVerticalForgettingFactor = 0.0003f;
	params.motionBiasEstEnabled = true;
	params.restBiasEstEnabled = true;
	params.restFilterTau = 2.278f;
	params.restMinT = 0.118f;
	params.restThGyr = 0.486f;
	params.restThAcc = 0.186f;
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
}

void vqf_update_gyro(float *g, float time)
{
	ARG_UNUSED(time);
	if (!vqf_vec3_finite(g)) {
		return;
	}
	float g_rad[3] = {0};
	// g is in deg/s, convert to rad/s
	for (int i = 0; i < 3; i++) {
		g_rad[i] = g[i] * DEG_TO_RAD;
	}
	/* Gyro and accelerometer use their configured native VQF periods. */
	updateGyr(&params, &state, &coeffs, g_rad);
	rest_observation_pending = true;
}


/**
 * @brief Track rest detection transitions and accumulate diagnostics.
 * Called after each accelerometer update (which runs rest detection).
 */
static void vqf_track_rest_diag(void)
{
	float dt = coeffs.accTs;
	uptime_s += dt;

	bool cur = getRestDetected(&state);
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
	if (!vqf_vec3_finite(a)) {
		return;
	}
	float a_m_s2[3] = {0};
	// a is in g, convert to m/s^2
	for (int i = 0; i < 3; i++) {
		a_m_s2[i] = a[i] * CONST_EARTH_GRAVITY;
	}
	if (a_m_s2[0] != 0 || a_m_s2[1] != 0 || a_m_s2[2] != 0) {
		memcpy(last_a, a_m_s2, sizeof(a_m_s2));
	}
	updateAcc(&params, &state, &coeffs, a_m_s2);
	if (a_m_s2[0] != 0 || a_m_s2[1] != 0 || a_m_s2[2] != 0) {
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
#if CONFIG_SENSOR_USE_MAGNETOMETER
	if (sensor_get_mag_available() && sensor_get_mag_enabled()) {
		getQuat9D(&state, q);
		return;
	}
#endif
	getQuat6D(&state, q);
}

bool vqf_get_rest_detected(void)
{
	return getRestDetected(&state);
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

	info->rest_detected = getRestDetected(&state);
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
