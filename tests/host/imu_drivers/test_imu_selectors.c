#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "sensor/imu/LSM6DSM.h"
#include "sensor/imu/LSM6DSO.h"
#include "sensor/imu/ICM42686.h"
#include "sensor/imu/ICM42688.h"

uint8_t last_accel_mode = 0xff, last_gyro_mode = 0xff;
uint8_t last_accel_odr = 0xff, last_gyro_odr = 0xff;
float accel_sensitivity, gyro_sensitivity;
void host_bus_reset(void);
void host_bus_set_register(uint8_t, uint8_t);
uint8_t host_bus_register(uint8_t);

static void close_period(float actual, float expected)
{
	assert(fabsf(actual - expected) <= expected * 1e-6f);
}

struct lsm_case {
	int (*init)(float, float, float, float *, float *);
	void (*fs)(float, float, float *, float *);
	uint8_t ctrl1, ctrl2;
	bool trimmed;
};

static void test_lsm(const struct lsm_case *d)
{
	const float limits[] = {.3f / 1000, .6f / 1000, 1.2f / 1000, 2.4f / 1000,
		4.8f / 1000, 9.6f / 1000, 19.2f / 1000, 38.4f / 1000};
	const float periods[] = {.15 / 1000, .3 / 1000, .6 / 1000, 1.2 / 1000,
		2.4 / 1000, 4.8 / 1000, 9.6 / 1000, 19.2 / 1000, 38.4 / 1000};
	for (unsigned i = 0; i < 8; i++) {
		for (unsigned side = 0; side < 3; side++) {
			float requested = side == 0 ? nextafterf(limits[i], 0)
				: side == 1 ? limits[i] : nextafterf(limits[i], INFINITY);
			unsigned selected = i + (side != 0);
			host_bus_reset();
			float a, g;
			assert(d->init(0, requested, requested, &a, &g) == 0);
			assert((host_bus_register(d->ctrl1) & 0xf0) == 0xa0 - selected * 0x10);
			assert((host_bus_register(d->ctrl2) & 0xf0) == 0xa0 - selected * 0x10);
			close_period(a, periods[selected]);
			close_period(g, selected == 0 ? 1.0 / 6660 : periods[selected]);
		}
	}
	const float accel_requests[] = {2, 2.1f, 4, 4.1f, 8, 8.1f};
	const float accel_ranges[] = {2, 4, 4, 8, 8, 16};
	const uint8_t accel_codes[] = {0, 8, 8, 12, 12, 4};
	const float gyro_requests[] = {250, 251, 500, 501, 1000, 1001};
	const float gyro_ranges[] = {250, 500, 500, 1000, 1000, 2000};
	const uint8_t gyro_codes[] = {0, 4, 4, 8, 8, 12};
	for (unsigned i = 0; i < 6; i++) {
		float ar, gr, a, g;
		d->fs(accel_requests[i], gyro_requests[i], &ar, &gr);
		assert(ar == accel_ranges[i] && gr == gyro_ranges[i]);
		host_bus_reset();
		assert(d->init(0, .005f, .005f, &a, &g) == 0);
		assert((host_bus_register(d->ctrl1) & 0x0c) == accel_codes[i]);
		assert((host_bus_register(d->ctrl2) & 0x0c) == gyro_codes[i]);
		assert(accel_sensitivity == ar / 32768.0f);
		assert(gyro_sensitivity == 35.0f * gr / 1000000.0f);
	}
	float a, g;
	host_bus_reset();
	if (d->trimmed)
		host_bus_set_register(LSM6DSO_INTERNAL_FREQ_FINE, 100);
	assert(d->init(0, .075f, .075f, &a, &g) == 0);
	assert((host_bus_register(d->ctrl1) & 0xf0) == (d->trimmed ? 0x10 : 0x20));
	close_period(a, d->trimmed ? .08f / 1.15f : .0384f);
	close_period(g, d->trimmed ? .08f / 1.15f : .0384f);
	/* Trim changes period outputs, not the existing period threshold tests. */
	host_bus_reset();
	if (d->trimmed)
		host_bus_set_register(LSM6DSO_INTERNAL_FREQ_FINE, (uint8_t)-100);
	assert(d->init(0, .0048f, .0048f, &a, &g) == 0);
	assert((host_bus_register(d->ctrl1) & 0xf0) == 0x50);
	close_period(a, d->trimmed ? .0048f / .85f : .0048f);
}

static void test_lsm_low_rate_rounding(const struct lsm_case *d)
{
	/* DSO truncates requested Hz to int BEFORE dividing by signed trim, then
	 * truncates again. Preserve that policy, not a floating-Hz approximation:
	 * +100 switches at 15 Hz, -100 at 12 Hz; DSM stays at 13 Hz.
	 * These inputs straddle each integer transition without reciprocal ties. */
	static const struct {
		int8_t trim;
		float requested_hz;
		uint8_t dsm_code, dso_code;
		float dsm_period, dso_period;
	} cases[] = {
		{0, 12.99f, 0x10, 0x10, .08f, .08f},
		{0, 13.01f, 0x20, 0x20, .0384f, .0384f},
		{100, 14.99f, 0x20, 0x10, .0384f, .06956521739f},
		{100, 15.01f, 0x20, 0x20, .0384f, .03339130435f},
		{-100, 11.99f, 0x10, 0x10, .08f, .09411764706f},
		{-100, 12.01f, 0x10, 0x20, .08f, .04517647059f},
	};
	for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
		host_bus_reset();
		host_bus_set_register(LSM6DSO_INTERNAL_FREQ_FINE, (uint8_t)cases[i].trim);
		float a, g;
		float requested = 1.0f / cases[i].requested_hz;
		assert(d->init(0, requested, requested, &a, &g) == 0);
		uint8_t code = d->trimmed ? cases[i].dso_code : cases[i].dsm_code;
		float period = d->trimmed ? cases[i].dso_period : cases[i].dsm_period;
		assert((host_bus_register(d->ctrl1) & 0xf0) == code);
		assert((host_bus_register(d->ctrl2) & 0xf0) == code);
		close_period(a, period);
		close_period(g, period);
	}
}

static void test_icm(int (*init)(float, float, float, float *, float *), uint8_t accel, uint8_t gyro)
{
	const float rates[] = {32000, 16000, 8000, 4000, 2000, 1000, 500, 200, 100, 50, 25, 12.5f};
	const uint8_t codes[] = {1, 2, 3, 4, 5, 6, 15, 7, 8, 9, 10, 11};
	for (unsigned i = 0; i < 12; i++) {
		for (unsigned side = 0; side < 3; side++) {
			float requested = side == 0 ? rates[i] * .999f : side == 1 ? rates[i] : rates[i] * 1.001f;
			unsigned selected = side == 2 && i ? i - 1 : i;
			host_bus_reset();
			float a, g;
			assert(init(0, 1.0f / requested, 1.0f / requested, &a, &g) == 0);
			assert((host_bus_register(accel) & 15) == codes[selected]);
			assert((host_bus_register(gyro) & 15) == codes[selected]);
			close_period(a, 1.0f / rates[selected]);
			close_period(g, 1.0f / rates[selected]);
		}
	}
}

static void test_icm_external_clock(
	int (*init)(float, float, float, float *, float *),
	uint8_t accel, uint8_t gyro, uint8_t fifo_count)
{
	/* Literal register/period expectations, independent of the shared helper.
	 * Each row is an exact nominal-rate boundary after external-clock scaling:
	 * 1000 -> 500 Hz (including nonsequential code 0x0f), 500 -> 200 Hz,
	 * and 25 -> 12.5 Hz. Faster requests must select the preceding rate.
	 * A 10-ppm offset straddles the boundary despite float reciprocal rounding. */
	static const struct {
		float clock_hz, boundary_period;
		uint8_t faster_code, boundary_code;
		float faster_period, boundary_actual;
	} cases[] = {
		{16000, .004f, 0x06, 0x0f, .002f, .004f},
		{16000, .01f, 0x0f, 0x07, .004f, .01f},
		{16000, .16f, 0x0a, 0x0b, .08f, .16f},
		{64000, .001f, 0x06, 0x0f, .0005f, .001f},
		{64000, .0025f, 0x0f, 0x07, .001f, .0025f},
		{64000, .04f, 0x0a, 0x0b, .02f, .04f},
	};
	for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
		for (unsigned side = 0; side < 3; side++) {
			float requested = cases[i].boundary_period
				* (side == 0 ? .99999f : side == 1 ? 1.0f : 1.00001f);
			host_bus_reset();
			/* Nonempty FIFO keeps init on CLKIN instead of its fallback path. */
			host_bus_set_register(fifo_count + 1, 1);
			float a, g;
			assert(init(cases[i].clock_hz, requested, requested, &a, &g) == 0);
			uint8_t code = side == 0 ? cases[i].faster_code : cases[i].boundary_code;
			float period = side == 0 ? cases[i].faster_period : cases[i].boundary_actual;
			assert((host_bus_register(accel) & 0x0f) == code);
			assert((host_bus_register(gyro) & 0x0f) == code);
			close_period(a, period);
			close_period(g, period);
		}
	}
}

int main(void)
{
	const struct lsm_case dsm = {lsm6dsm_init, lsm6dsm_update_fs, LSM6DSM_CTRL1, LSM6DSM_CTRL2, false};
	const struct lsm_case dso = {lsm6dso_init, lsm6dso_update_fs, LSM6DSO_CTRL1, LSM6DSO_CTRL2, true};
	test_lsm(&dsm);
	test_lsm(&dso);
	test_lsm_low_rate_rounding(&dsm);
	test_lsm_low_rate_rounding(&dso);
	test_icm(icm42686_init, ICM42686_ACCEL_CONFIG0, ICM42686_GYRO_CONFIG0);
	test_icm(icm_init, ICM42688_ACCEL_CONFIG0, ICM42688_GYRO_CONFIG0);
	test_icm_external_clock(icm42686_init, ICM42686_ACCEL_CONFIG0,
		ICM42686_GYRO_CONFIG0, ICM42686_FIFO_COUNTH);
	test_icm_external_clock(icm_init, ICM42688_ACCEL_CONFIG0,
		ICM42688_GYRO_CONFIG0, ICM42688_FIFO_COUNTH);
	puts("Actual-driver selector boundaries passed");
	return 0;
}
