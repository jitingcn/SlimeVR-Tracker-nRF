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
#include "globals.h"
#include "util.h"

#include "sensor/sensors_enum.h"
#include "../src/vqf.h" // conflicting with vqf.h in local path

#include "../vqf/vqf.h" // conflicting with vqf.h in vqf-c

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (M_PI / 180.0f)
#endif

static uint8_t imu_id;

static vqf_params_t params;
static vqf_state_t state;
static vqf_coeffs_t coeffs;

static float last_a[3] = {0};

void vqf_update_sensor_ids(int imu)
{
	imu_id = imu;
}

static void set_params()
{
	init_params(&params);
	params.biasClip = 2.5f;
	params.tauMag = 10.0f; // best result for VQF from paper
	params.biasForgettingTime = 120.0f;
	params.biasSigmaInit = 2.0f;
	params.biasSigmaMotion = 0.20f;
	params.biasSigmaRest = 0.03f;
	params.biasVerticalForgettingFactor = 0.00001f;
	params.motionBiasEstEnabled = true;
	params.restBiasEstEnabled = true;
	params.restFilterTau = 1.4f;
	params.restMinT = 2.8f;
	params.restThAcc = 0.25f;
	params.restThGyr = 0.8f;
	params.tauAcc = 3.6f;
}

void vqf_init(float g_time, float a_time, float m_time)
{
	set_params();
	initVqf(&params, &state, &coeffs, g_time, a_time, m_time);
}

void vqf_load(const void *data)
{
	set_params();
	memcpy(&state, data, sizeof(state));
	memcpy(&coeffs, (uint8_t *)data + sizeof(state), sizeof(coeffs));
}

void vqf_save(void *data)
{
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

void vqf_update_accel(float *a, float time)
{
	ARG_UNUSED(time);
	float a_m_s2[3] = {0};
	// a is in g, convert to m/s^2
	for (int i = 0; i < 3; i++)
		a_m_s2[i] = a[i] * CONST_EARTH_GRAVITY;
	if (a_m_s2[0] != 0 || a_m_s2[1] != 0 || a_m_s2[2] != 0)
		memcpy(last_a, a_m_s2, sizeof(a_m_s2));
	updateAcc(&params, &state, &coeffs, a_m_s2);
}

void vqf_update_accel_ts(float *a, uint64_t timestamp_us)
{
	float a_m_s2[3] = {0};
	for (int i = 0; i < 3; i++)
		a_m_s2[i] = a[i] * CONST_EARTH_GRAVITY;
	if (a_m_s2[0] != 0 || a_m_s2[1] != 0 || a_m_s2[2] != 0)
		memcpy(last_a, a_m_s2, sizeof(a_m_s2));
	updateAccTs(&params, &state, &coeffs, a_m_s2, timestamp_us);
}

void vqf_update_mag(float *m, float time)
{
	ARG_UNUSED(time);
	updateMag(&params, &state, &coeffs, m);
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
