#ifndef TEST_IMU_CALIBRATION_UTIL_H
#define TEST_IMU_CALIBRATION_UTIL_H

#include <stdbool.h>
#include <stddef.h>
#include <math.h>

static inline bool v_finite(const float *values, size_t count)
{
	for (size_t i = 0; i < count; i++) {
		if (!isfinite(values[i])) {
			return false;
		}
	}
	return true;
}

static inline bool v_epsilon(const float *a, const float *b, float epsilon)
{
	float x = a[0] - b[0];
	float y = a[1] - b[1];
	float z = a[2] - b[2];
	return sqrtf(x * x + y * y + z * z) < epsilon;
}

static inline float v_avg(const float *a)
{
	return (a[0] + a[1] + a[2]) / 3;
}

static inline void apply_BAinv(float xyz[3], float matrix[4][3])
{
	float temp[3];
	for (int i = 0; i < 3; i++) {
		temp[i] = xyz[i] - matrix[0][i];
	}
	xyz[0] = matrix[1][0] * temp[0] + matrix[1][1] * temp[1] + matrix[1][2] * temp[2];
	xyz[1] = matrix[2][0] * temp[0] + matrix[2][1] * temp[1] + matrix[2][2] * temp[2];
	xyz[2] = matrix[3][0] * temp[0] + matrix[3][1] * temp[1] + matrix[3][2] * temp[2];
}

#endif
