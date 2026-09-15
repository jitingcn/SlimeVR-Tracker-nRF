#include <assert.h>
#include <errno.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "magneto1_4.h"
#include "mymathlib_matrix.h"

struct samples {
	double ata[100];
	double norm_sum;
	double count;
	double bias[3];
	double transform[3][3];
	double radius;
};

static bool allocation_failure;
static unsigned outstanding_allocations;
static unsigned qr_calls, hessenberg_calls, decomposition_calls, inverse_calls;
static unsigned fail_qr_at, fail_hessenberg_at, fail_decomposition_at, fail_inverse_at;
static unsigned complex_qr_at, negative_qr_at, zero_vectors_at, nonfinite_qr_at;
static bool nested_solve;

void *k_malloc(size_t size)
{
	if (allocation_failure) {
		return NULL;
	}
	void *ptr = malloc(size);
	assert(ptr != NULL);
	outstanding_allocations++;
	return ptr;
}

void k_free(void *ptr)
{
	if (ptr != NULL) {
		assert(outstanding_allocations > 0);
		outstanding_allocations--;
		free(ptr);
	}
}

static int solve(float output[4][3], struct samples *samples)
{
#if TEST_LEGACY
	/* Allows the same valid-data probe to run against preserved void-API sources.
	 * Failure cases deliberately fail the new observable contract in this mode. */
	magneto_current_calibration(output, samples->ata, samples->norm_sum, samples->count);
	return 0;
#else
	return magneto_current_calibration(output, samples->ata, samples->norm_sum, samples->count);
#endif
}

static void sample_point(const struct samples *samples, unsigned i, double point[3])
{
	double z = 1.0 - 2.0 * ((double)i + 0.5) / samples->count;
	double phi = (double)i * 2.39996322972865332;
	double xy = sqrt(1.0 - z * z);
	double unit[3] = {xy * cos(phi), xy * sin(phi), z};
	for (int row = 0; row < 3; row++) {
		point[row] = samples->bias[row];
		for (int col = 0; col < 3; col++) {
			point[row] += samples->transform[row][col] * samples->radius * unit[col];
		}
	}
}

static void make_samples(struct samples *samples, unsigned variant)
{
	memset(samples, 0, sizeof(*samples));
	samples->radius = 0.6 + 0.15 * variant;
	samples->bias[0] = 0.16 + 0.02 * variant;
	samples->bias[1] = -0.11 - 0.015 * variant;
	samples->bias[2] = 0.07 + 0.01 * variant;
	double transform[3][3] = {
		{1.2 + 0.02 * variant, 0.08, -0.04},
		{0.08, 0.9 + 0.01 * variant, 0.06},
		{-0.04, 0.06, 1.1 - 0.01 * variant},
	};
	memcpy(samples->transform, transform, sizeof(transform));
	unsigned count = 257 + variant * 11;
	double accumulated_count = 0;
	samples->count = count;
	for (unsigned i = 0; i < count; i++) {
		double point[3];
		sample_point(samples, i, point);
		magneto_sample(point[0], point[1], point[2], samples->ata, &samples->norm_sum, &accumulated_count);
	}
	assert(accumulated_count == samples->count);
}

static void check_calibration(const float output[4][3], const struct samples *samples)
{
	double expected_norm = samples->norm_sum / samples->count;
	for (int i = 0; i < 3; i++) {
		assert(fabs(output[0][i] - samples->bias[i]) < 2e-5);
	}
	for (unsigned i = 0; i < (unsigned)samples->count; i++) {
		double point[3], calibrated[3] = {0};
		sample_point(samples, i, point);
		for (int row = 0; row < 3; row++) {
			for (int col = 0; col < 3; col++) {
				calibrated[row] += output[row + 1][col] * (point[col] - output[0][col]);
			}
		}
		double norm = sqrt(calibrated[0] * calibrated[0] + calibrated[1] * calibrated[1]
						   + calibrated[2] * calibrated[2]);
		assert(isfinite(norm));
		assert(fabs(norm - expected_norm) < expected_norm * 2e-5);
	}
}

int __real_QR_Hessenberg_Matrix(double *, double *, double[], double[], int, int);
int __wrap_QR_Hessenberg_Matrix(double *H, double *S, double real[], double imag[], int n, int iterations)
{
	unsigned call = ++qr_calls;
	if (nested_solve) {
		nested_solve = false;
		struct samples samples;
		float output[4][3];
		make_samples(&samples, 4);
		assert(solve(output, &samples) == 0);
		check_calibration(output, &samples);
	}
	if (call == fail_qr_at) {
		/* Exercise the real QR helper's nonconvergence return without relying
		 * on machine-dependent convergence of a specially chosen dataset. */
		return __real_QR_Hessenberg_Matrix(H, S, real, imag, n, 0);
	}
	int err = __real_QR_Hessenberg_Matrix(H, S, real, imag, n, iterations);
	if (err == 0) {
		if (call == complex_qr_at) {
			for (int i = 0; i < n; i++) {
				imag[i] = 1.0;
			}
		}
		if (call == negative_qr_at) {
			real[0] = -1.0;
		}
		if (call == zero_vectors_at) {
			memset(S, 0, (size_t)n * n * sizeof(*S));
		}
		if (call == nonfinite_qr_at) {
			real[0] = NAN;
		}
	}
	return err;
}

#if TEST_LEGACY
int __real_Hessenberg_Form_Elementary(double *, double *, int);
int __wrap_Hessenberg_Form_Elementary(double *A, double *S, int n)
#else
int __real_Hessenberg_Form_Elementary(double *, double *, int, int[]);
int __wrap_Hessenberg_Form_Elementary(double *A, double *S, int n, int perm[])
#endif
{
	if (++hessenberg_calls == fail_hessenberg_at) {
		return -1;
	}
#if TEST_LEGACY
	return __real_Hessenberg_Form_Elementary(A, S, n);
#else
	return __real_Hessenberg_Form_Elementary(A, S, n, perm);
#endif
}

int __real_Choleski_LU_Decomposition(double *, int);
int __wrap_Choleski_LU_Decomposition(double *A, int n)
{
	if (++decomposition_calls == fail_decomposition_at) {
		return -1;
	}
	return __real_Choleski_LU_Decomposition(A, n);
}

int __real_Choleski_LU_Inverse(double *, int);
int __wrap_Choleski_LU_Inverse(double *A, int n)
{
	if (++inverse_calls == fail_inverse_at) {
		return -1;
	}
	return __real_Choleski_LU_Inverse(A, n);
}

static void reset_faults(void)
{
	assert(outstanding_allocations == 0);
	qr_calls = hessenberg_calls = decomposition_calls = inverse_calls = 0;
	fail_qr_at = fail_hessenberg_at = fail_decomposition_at = fail_inverse_at = 0;
	complex_qr_at = negative_qr_at = zero_vectors_at = nonfinite_qr_at = 0;
	allocation_failure = nested_solve = false;
}

static void expect_failure(struct samples *samples, int expected_error)
{
	float output[4][3], saved[4][3];
	double saved_ata[100];
	for (int row = 0; row < 4; row++) {
		for (int col = 0; col < 3; col++) {
			output[row][col] = (float)(row * 3 + col + 1) / 16.0f;
		}
	}
	memcpy(saved, output, sizeof(saved));
	memcpy(saved_ata, samples->ata, sizeof(saved_ata));
	int err = solve(output, samples);
	assert(err == expected_error);
	assert(memcmp(output, saved, sizeof(output)) == 0);
	assert(memcmp(samples->ata, saved_ata, sizeof(saved_ata)) == 0);
	assert(outstanding_allocations == 0);
}

static void test_valid(bool dump)
{
	for (unsigned variant = 0; variant < 8; variant++) {
		reset_faults();
		struct samples samples;
		float output[4][3];
		double saved_ata[100];
		make_samples(&samples, variant);
		memcpy(saved_ata, samples.ata, sizeof(saved_ata));
		assert(solve(output, &samples) == 0);
		check_calibration(output, &samples);
		assert(memcmp(samples.ata, saved_ata, sizeof(saved_ata)) == 0);
		assert(outstanding_allocations == 0);
		if (dump) {
			for (int row = 0; row < 4; row++) {
				for (int col = 0; col < 3; col++) {
					printf("%a\n", (double)output[row][col]);
				}
			}
		}
	}
}

static void test_invalid(void)
{
	struct samples samples;
	reset_faults();
	make_samples(&samples, 0);
	samples.ata[7] = NAN;
	expect_failure(&samples, -EINVAL);
	make_samples(&samples, 0);
	samples.norm_sum = INFINITY;
	expect_failure(&samples, -EINVAL);
	samples.norm_sum = 1;
	samples.count = 0;
	expect_failure(&samples, -EINVAL);
	samples.count = -1;
	expect_failure(&samples, -EINVAL);
	samples.count = DBL_MAX;
	samples.norm_sum = DBL_MIN;
	expect_failure(&samples, -ERANGE);
	make_samples(&samples, 0);
	samples.norm_sum = DBL_MIN;
	expect_failure(&samples, -ERANGE);
	samples.norm_sum = DBL_MAX / 100;
	expect_failure(&samples, -ERANGE);
	for (int i = 0; i < 100; i++) {
		samples.ata[i] = DBL_MAX;
	}
	samples.norm_sum = 100;
	samples.count = 100;
	expect_failure(&samples, -EDOM);
}

static void test_singular(void)
{
	struct samples samples = {0};
	reset_faults();
	for (int i = 0; i < 128; i++) {
		magneto_sample(0.5, 0.25, 0.125, samples.ata, &samples.norm_sum, &samples.count);
	}
	expect_failure(&samples, -EDOM);
	memset(&samples, 0, sizeof(samples));
	for (int i = 0; i < 128; i++) {
		double angle = i * 0.1;
		magneto_sample(cos(angle), sin(angle), 0, samples.ata, &samples.norm_sum, &samples.count);
	}
	expect_failure(&samples, -EDOM);
}

static void test_allocation(void)
{
	struct samples samples;
	reset_faults();
	make_samples(&samples, 0);
	allocation_failure = true;
	expect_failure(&samples, -ENOMEM);
	allocation_failure = false;
	float output[4][3];
	assert(solve(output, &samples) == 0);
	check_calibration(output, &samples);
	assert(outstanding_allocations == 0);
}

static void test_helpers(void)
{
	struct samples samples;
	make_samples(&samples, 0);
	for (unsigned stage = 1; stage <= 2; stage++) {
		reset_faults();
		fail_qr_at = stage;
		expect_failure(&samples, -EAGAIN);
		reset_faults();
		fail_hessenberg_at = stage;
		expect_failure(&samples, -EDOM);
		reset_faults();
		fail_decomposition_at = stage;
		expect_failure(&samples, -EDOM);
		reset_faults();
		fail_inverse_at = stage;
		expect_failure(&samples, -EDOM);
		reset_faults();
		complex_qr_at = stage;
		expect_failure(&samples, -EDOM);
		reset_faults();
		zero_vectors_at = stage;
		expect_failure(&samples, -EDOM);
		reset_faults();
		nonfinite_qr_at = stage;
		expect_failure(&samples, -EDOM);
	}
	reset_faults();
	negative_qr_at = 2;
	expect_failure(&samples, -EDOM);
}

static void test_reentrant(void)
{
	struct samples samples;
	float output[4][3];
	reset_faults();
	make_samples(&samples, 0);
	nested_solve = true;
	assert(solve(output, &samples) == 0);
	check_calibration(output, &samples);
	assert(outstanding_allocations == 0);
}

int main(int argc, char **argv)
{
	if (argc == 2 && strcmp(argv[1], "--dump-valid") == 0) {
		test_valid(true);
		return 0;
	}
	const char *selected = argc == 2 ? argv[1] : "all";
	bool all = strcmp(selected, "all") == 0;
	if (all || strcmp(selected, "valid") == 0) test_valid(false);
	if (all || strcmp(selected, "invalid") == 0) test_invalid();
	if (all || strcmp(selected, "singular") == 0) test_singular();
	if (all || strcmp(selected, "allocation") == 0) test_allocation();
	if (all || strcmp(selected, "helpers") == 0) test_helpers();
	if (all || strcmp(selected, "reentrant") == 0) test_reentrant();
	puts("magneto calibration contract passed");
	return 0;
}
