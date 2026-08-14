#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "sensor/imu/BMI270.h"
#include "sensor/imu/ICM42686.h"
#include "sensor/imu/ICM42688.h"
#include "sensor/imu/ICM45686.h"

void host_bus_reset(void);
size_t host_bus_write_count(void);
void host_bus_fail_transaction(size_t transaction);
uint8_t host_bus_write_reg(size_t index);
void host_bus_set_register(uint8_t reg, uint8_t value);
uint8_t host_bus_write_value(size_t index);

static void fail(const char *driver, const char *message)
{
	fprintf(stderr, "%s: %s\n", driver, message);
	exit(1);
}

static void expect_close(const char *driver, float actual, float expected)
{
	if (fabsf(actual - expected) > 0.00001f)
		fail(driver, "unexpected actual_time");
}

static uint8_t last_value(uint8_t reg)
{
	for (size_t i = host_bus_write_count(); i > 0; --i)
		if (host_bus_write_reg(i - 1) == reg)
			return host_bus_write_value(i - 1);
	return 0xff;
}

static size_t writes_to(uint8_t reg)
{
	size_t count = 0;
	for (size_t i = 0; i < host_bus_write_count(); ++i)
		count += host_bus_write_reg(i) == reg;
	return count;
}

struct driver_case {
	const char *name;
	int (*update)(float, float, float *, float *);
	uint8_t accel_reg;
	uint8_t gyro_reg;
	uint8_t power_reg;
	uint8_t accel_mask;
	uint8_t gyro_mask;
	uint8_t code_12_5;
	uint8_t code_25;
	uint8_t code_100;
	uint8_t code_200;
	uint8_t code_800;
	float actual_800_hz;
	uint8_t power_ln;
	uint8_t power_off;
	uint8_t power_standby;
};

static void expect_rate(const struct driver_case *d, float requested_hz, float actual_hz, uint8_t code)
{
	float ignored_at, ignored_gt;
	d->update(0.0f, 0.0f, &ignored_at, &ignored_gt);
	float at = -1.0f, gt = -1.0f;
	host_bus_reset();
	if (d->update(1.0f / requested_hz, 1.0f / requested_hz, &at, &gt) != 0)
		fail(d->name, "update failed");
	if ((last_value(d->accel_reg) & d->accel_mask) != code ||
		(last_value(d->gyro_reg) & d->gyro_mask) != code)
		fail(d->name, "wrong ODR register code");
	expect_close(d->name, at, 1.0f / actual_hz);
	expect_close(d->name, gt, 1.0f / actual_hz);

	host_bus_reset();
	at = gt = -1.0f;
	d->update(1.0f / requested_hz, 1.0f / requested_hz, &at, &gt);
	if (host_bus_write_count() != 0)
		fail(d->name, "cache hit emitted a transaction");
	expect_close(d->name, at, 1.0f / actual_hz);
	expect_close(d->name, gt, 1.0f / actual_hz);
}

static void test_icm(const struct driver_case *d)
{
	expect_rate(d, 12.5f, 12.5f, d->code_12_5);

	float at, gt;
	d->update(0.0f, 0.0f, &at, &gt);
	host_bus_reset();
	d->update(0.1f, 0.1f, &at, &gt);
	if ((last_value(d->accel_reg) & d->accel_mask) != d->code_12_5 ||
		(last_value(d->gyro_reg) & d->gyro_mask) != d->code_12_5)
		fail(d->name, "10 Hz did not clamp to 12.5 Hz");
	expect_close(d->name, at, 0.08f);
	expect_close(d->name, gt, 0.08f);

	expect_rate(d, 25.0f, 25.0f, d->code_25);
	expect_rate(d, 100.0f, 100.0f, d->code_100);
	expect_rate(d, 200.0f, 200.0f, d->code_200);
	expect_rate(d, 800.0f, d->actual_800_hz, d->code_800);

	host_bus_reset();
	d->update(1.0f, 1.0f, &at, &gt);
	if ((last_value(d->accel_reg) & d->accel_mask) != d->code_12_5)
		fail(d->name, "below-minimum request did not clamp");
	expect_close(d->name, at, 0.08f);

	host_bus_reset();
	d->update(0.0f, 0.0f, &at, &gt);
	if (last_value(d->power_reg) != d->power_off || at != 0.0f || gt != 0.0f)
		fail(d->name, "off mode incorrect");

	host_bus_reset();
	d->update(0.0f, INFINITY, &at, &gt);
	if (last_value(d->power_reg) != d->power_standby || at != 0.0f || gt != 0.0f)
		fail(d->name, "standby mode was hidden by ODR cache");
	if (writes_to(d->power_reg) != 1)
		fail(d->name, "standby power transaction missing");
}
static void test_cache_invalidates_after_bus_failure(const struct driver_case *d)
{
	float at, gt;
	d->update(0.0f, 0.0f, &at, &gt);
	host_bus_reset();
	host_bus_fail_transaction(2);
	if (d->update(0.01f, 0.01f, &at, &gt) >= 0)
		fail(d->name, "injected transaction failure was ignored");
	host_bus_reset();
	if (d->update(0.01f, 0.01f, &at, &gt) != 0 || host_bus_write_count() == 0)
		fail(d->name, "failed transaction left a valid cache entry");
}


static void test_clock_scale(void)
{
	float at, gt;
	host_bus_reset();
	host_bus_set_register(0x72, 0xE9);
	host_bus_set_register(ICM45686_FIFO_COUNT_0, 1);
	icm45_init(16000.0f, 0.08f, 0.08f, &at, &gt);
	if ((last_value(ICM45686_ACCEL_CONFIG0) & 0x0f) != ACCEL_ODR_25Hz ||
		(last_value(ICM45686_GYRO_CONFIG0) & 0x0f) != GYRO_ODR_25Hz)
		fail("ICM45686", "clock-scaled ODR register code incorrect");
	expect_close("ICM45686", at, 0.08f);
	expect_close("ICM45686", gt, 0.08f);

	host_bus_reset();
	host_bus_set_register(0x72, 0xE9);
	icm45_init(0.0f, 0.08f, 0.08f, &at, &gt);
	if ((last_value(ICM45686_ACCEL_CONFIG0) & 0x0f) != ACCEL_ODR_12_5Hz)
		fail("ICM45686", "internal-clock scale was not reset");
}

static void test_bmi(void)
{
	const char *name = "BMI270";
	float at, gt;
	struct { float requested_hz, expected_hz; uint8_t accel, gyro; } cases[] = {
		{12.5f, 12.5f, ODR_12p5, ODR_25},
		{10.0f, 12.5f, ODR_12p5, ODR_25},
		{25.0f, 25.0f, ODR_25, ODR_25},
		{100.0f, 100.0f, ODR_100, ODR_100},
		{200.0f, 200.0f, ODR_200, ODR_200},
		{800.0f, 800.0f, ODR_800, ODR_800},
		{0.1f, 0.78125f, ODR_0p78, ODR_25},
	};
	for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
		bmi_update_odr(0.0f, 0.0f, &at, &gt);
		host_bus_reset();
		bmi_update_odr(1.0f / cases[i].requested_hz, 1.0f / cases[i].requested_hz, &at, &gt);
		if ((last_value(BMI270_ACC_CONF) & 0x0f) != cases[i].accel ||
			(last_value(BMI270_GYR_CONF) & 0x0f) != cases[i].gyro)
			fail(name, "wrong ODR register code");
		expect_close(name, at, 1.0f / cases[i].expected_hz);
		expect_close(name, gt, 1.0f / (cases[i].requested_hz < 25.0f ? 25.0f : cases[i].expected_hz));
	}

	bmi_update_odr(0.0f, 0.0f, &at, &gt);
	bmi_update_odr(0.08f, 0.08f, &at, &gt);
	host_bus_reset();
	bmi_update_odr(0.08f, 0.08f, &at, &gt);
	if (host_bus_write_count() != 0)
		fail(name, "cache hit emitted a transaction");
	expect_close(name, at, 0.08f);
	expect_close(name, gt, 0.04f);

	host_bus_reset();
	bmi_update_odr(0.0f, INFINITY, &at, &gt);
	if (last_value(BMI270_PWR_CTRL) != 0x08 || at != 0.0f || gt != 0.0f)
		fail(name, "off semantics incorrect");
}

int main(void)
{
	test_bmi();
	const struct driver_case cases[] = {
		{"ICM42686", icm42686_update_odr, ICM42686_ACCEL_CONFIG0, ICM42686_GYRO_CONFIG0,
		 ICM42686_PWR_MGMT0, 0x0f, 0x0f, ICM42686_AODR_12_5Hz, ICM42686_AODR_25Hz,
		 ICM42686_AODR_100Hz, ICM42686_AODR_200Hz, ICM42686_AODR_1kHz, 1000.0f,
		 (ICM42686_gMode_LN << 2) | ICM42686_aMode_LN,
		 (ICM42686_gMode_OFF << 2) | ICM42686_aMode_OFF,
		 (ICM42686_gMode_SBY << 2) | ICM42686_aMode_OFF},
		{"ICM42688", icm_update_odr, ICM42688_ACCEL_CONFIG0, ICM42688_GYRO_CONFIG0,
		 ICM42688_PWR_MGMT0, 0x0f, 0x0f, AODR_12_5Hz, AODR_25Hz, AODR_100Hz,
		 AODR_200Hz, AODR_1kHz, 1000.0f, (gMode_LN << 2) | aMode_LN,
		 (gMode_OFF << 2) | aMode_OFF, (gMode_SBY << 2) | aMode_OFF},
		{"ICM45686", icm45_update_odr, ICM45686_ACCEL_CONFIG0, ICM45686_GYRO_CONFIG0,
		 ICM45686_PWR_MGMT0, 0x0f, 0x0f, ACCEL_ODR_12_5Hz, ACCEL_ODR_25Hz,
		 ACCEL_ODR_100Hz, ACCEL_ODR_200Hz, ACCEL_ODR_800Hz, 800.0f,
		 (GYRO_MODE_LN << 2) | ACCEL_MODE_LN,
		 (GYRO_MODE_OFF << 2) | ACCEL_MODE_OFF,
		 (GYRO_MODE_STANDBY << 2) | ACCEL_MODE_OFF},
	};
	for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
		test_icm(&cases[i]);
		test_cache_invalidates_after_bus_failure(&cases[i]);
	}
	test_clock_scale();
	puts("IMU ODR host tests passed");
	return 0;
}
