/* SPDX-License-Identifier: MIT */
#include "sensor/calibration/mag_fit.h"
#include "sensor/magneto/magneto1_4.h"
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void *k_malloc(size_t size)
{
	return malloc(size);
}
void k_free(void *ptr)
{
	free(ptr);
}
static float samples[256][3];
static unsigned polls, cancel_at;
static bool sample(void *ctx, unsigned i, float out[3])
{
	(void)ctx;
	memcpy(out, samples[i], sizeof(samples[i]));
	return true;
}
static bool poll_fit(void *ctx)
{
	(void)ctx;
	return ++polls != cancel_at;
}
static const float trusted[4][3] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
static void sphere(bool belt)
{
	for (unsigned i = 0; i < 256; i++) {
		float z = 1 - 2 * (i + .5f) / 256, phi = i * 2.39996323f;
		if (belt) {
			z *= .015f;
		}
		float r = sqrtf(1 - z * z);
		samples[i][0] = .48f * r * cosf(phi) + .035f;
		samples[i][1] = .53f * r * sinf(phi) - .025f;
		samples[i][2] = .50f * z + .015f;
	}
	polls = cancel_at = 0;
}
static void unchanged_failure(const float previous[4][3], float h, int expected)
{
	float out[4][3], before[4][3];
	memset(out, 0x5a, sizeof(out));
	memcpy(before, out, sizeof(out));
	struct mag_fit_result result = {17, 18, 19}, saved = result;
	int err = magneto_robust_fit(256, sample, poll_fit, NULL, previous, h, out, &result);
	assert(err < 0);
	if (expected) {
		assert(err == expected);
	}
	assert(memcmp(out, before, sizeof(out)) == 0);
	assert(memcmp(&result, &saved, sizeof(result)) == 0);
}
static void good_fit(bool bootstrap, bool outliers)
{
	sphere(false);
	if (outliers) {
		for (unsigned i = 0; i < 5; i++) {
			for (unsigned j = 0; j < 3; j++) {
				samples[i][j] *= 12;
			}
		}
	}
	float out[4][3];
	struct mag_fit_result result;
	assert(
		magneto_robust_fit(256, sample, poll_fit, NULL, bootstrap ? NULL : trusted, bootstrap ? 0 : .5f, out, &result)
		== 0
	);
	assert(isfinite(result.condition) && result.condition <= 1000);
	assert(result.rms < .03f);
	assert(result.field_norm == .5f);
	assert(fabsf(out[0][0] - .035f) < .012f);
	assert(fabsf(out[0][1] + .025f) < .012f);
	assert(fabsf(out[0][2] - .015f) < .012f);
	for (unsigned i = 0; i < 3; i++) {
		for (unsigned j = 0; j < 3; j++) {
			assert(isfinite(out[i + 1][j]));
			assert(fabsf(out[i + 1][j] - out[j + 1][i]) < 1e-6f);
		}
	}
	float a = out[1][0], b = out[1][1], c = out[1][2], d = out[2][1], e = out[2][2], f = out[3][2];
	assert(a > 0 && a * d - b * b > 0 && a * (d * f - e * e) - b * (b * f - c * e) + c * (b * e - c * d) > 0);
	assert(fabsf(out[1][0] * .48f / result.field_norm - 1) < .025f);
	assert(fabsf(out[2][1] * .53f / result.field_norm - 1) < .025f);
	assert(fabsf(out[3][2] * .50f / result.field_norm - 1) < .025f);
}
static float holdout_rms(const float model[4][3], float h)
{
	float sum = 0;
	for (unsigned i = 0; i < 511; i++) {
		float z = 1 - 2 * (i + .37f) / 511, phi = i * 2.39996323f + .43f, r = sqrtf(1 - z * z);
		float raw[3] = {.48f * r * cosf(phi) + .035f, .53f * r * sinf(phi) - .025f, .50f * z + .015f};
		float n2 = 0;
		for (unsigned a = 0; a < 3; a++) {
			float v = 0;
			for (unsigned b = 0; b < 3; b++) {
				v += model[a + 1][b] * (raw[b] - model[0][b]);
			}
			n2 += v * v;
		}
		float e = sqrtf(n2) / h - 1;
		sum += e * e;
	}
	return sqrtf(sum / 511);
}
static void compare_contamination(void)
{
	sphere(false);
	for (unsigned i = 0; i < 256; i += 31) {
		samples[i][0] += .25f;
		samples[i][1] -= .16f;
		samples[i][2] += .12f;
	}
	double sum = 0, count = 0;
	memset(mag_cal_workspace.ata, 0, sizeof(mag_cal_workspace.ata));
	for (unsigned i = 0; i < 256; i++) {
		magneto_sample(samples[i][0], samples[i][1], samples[i][2], mag_cal_workspace.ata, &sum, &count);
	}
	float old[4][3], fitted[4][3];
	struct mag_fit_result result;
	int legacy = magneto_current_calibration(old, mag_cal_workspace.ata, sum, count);
	int robust = magneto_robust_fit(256, sample, poll_fit, NULL, trusted, .5f, fitted, &result);
	assert(robust == 0);
	float current = holdout_rms(fitted, result.field_norm);
	printf(
		"contaminated clean holdout: legacy err=%d RMS=%g; robust RMS=%g\n",
		legacy,
		legacy ? NAN : holdout_rms(old, (float)(sum / count)),
		current
	);
	assert(current < .02f);
	if (!legacy) {
		assert(current < holdout_rms(old, (float)(sum / count)));
	}
}
/* The raw mean radius changes with direction occupancy and hard iron. Neither
 * may set the corrected radius, even when the old model cannot reach the bias. */
static void normalized_recovery(void)
{
	const float bias[3] = {.8f, -.5f, .6f};
	const float radii[3] = {.48f, .53f, .50f};
	float means[2];
	for (unsigned distribution = 0; distribution < 2; distribution++) {
		float sum = 0;
		for (unsigned i = 0; i < 256; i++) {
			float u = (i + .5f) / 256;
			float z = 2 * (distribution ? sqrtf(u) : u) - 1;
			float phi = i * 2.39996323f, r = sqrtf(1 - z * z);
			samples[i][0] = radii[0] * r * cosf(phi) + bias[0];
			samples[i][1] = radii[1] * r * sinf(phi) + bias[1];
			samples[i][2] = radii[2] * z + bias[2];
			sum += sqrtf(samples[i][0] * samples[i][0] + samples[i][1] * samples[i][1] + samples[i][2] * samples[i][2]);
		}
		means[distribution] = sum / 256;
		polls = cancel_at = 0;
		unchanged_failure(trusted, .5f, 0);
		float out[4][3];
		struct mag_fit_result result;
		assert(magneto_robust_fit(256, sample, poll_fit, NULL, NULL, 0, out, &result) == 0);
		assert(result.field_norm == .5f);
		for (unsigned i = 0; i < 3; i++) {
			assert(fabsf(out[0][i] - bias[i]) < .005f);
			for (unsigned j = 0; j < 3; j++) {
				assert(fabsf(out[i + 1][j] - (i == j ? .5f / radii[i] : 0)) < .005f);
			}
		}
	}
	assert(fabsf(means[0] - means[1]) > .05f);
}

static void scaled_prior(void)
{
	sphere(false);
	float previous[4][3], out[4][3];
	struct mag_fit_result result;
	memcpy(previous, trusted, sizeof(previous));
	/* An old radius-2 model is physically equivalent to the radius-.5 prior.
	 * Its unconverted scale would violate the update trust region. */
	for (unsigned i = 1; i < 4; i++) {
		previous[i][i - 1] *= 4;
	}
	assert(magneto_robust_fit(256, sample, poll_fit, NULL, previous, 2, out, &result) == 0);
	assert(result.field_norm == .5f);
	assert(holdout_rms(out, .5f) < .005f);
	assert(fabsf(out[0][0] - .035f) < .005f);
	assert(fabsf(out[0][1] + .025f) < .005f);
	assert(fabsf(out[0][2] - .015f) < .005f);
}

int main(void)
{
	good_fit(false, false);
	good_fit(true, false);
	good_fit(false, true);
	good_fit(true, true);
	compare_contamination();
	normalized_recovery();
	scaled_prior();
	sphere(true);
	unchanged_failure(trusted, .5f, 0);
	for (unsigned i = 0; i < 256; i++) {
		for (unsigned j = 0; j < 3; j++) {
			samples[i][j] = (i & (1u << j)) ? .28867513f : -.28867513f;
		}
	}
	unchanged_failure(trusted, .5f, 0);
	sphere(false);
	cancel_at = 3;
	unchanged_failure(trusted, .5f, -ECANCELED);
	sphere(false);
	cancel_at = 3;
	unchanged_failure(NULL, 0, -ECANCELED);
	sphere(false);
	samples[7][1] = NAN;
	unchanged_failure(trusted, .5f, 0);
	sphere(false);
	float bad[4][3];
	memcpy(bad, trusted, sizeof(bad));
	bad[2][1] = -1;
	unchanged_failure(bad, .5f, 0);
	puts("mag_fit: recovery, outliers, rank, cancellation and unchanged-output contracts passed");
	return 0;
}
