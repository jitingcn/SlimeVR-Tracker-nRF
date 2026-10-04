#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <hal/nrf_gpio.h>

#include "sensor/imu/BMI270.h"
#include "sensor/imu/ICM42686.h"
#include "sensor/imu/ICM42688.h"
#include "sensor/imu/LSM6DSO.h"

/* LSM6DSO shares these caches with the production LSM6DSV module. */
uint8_t last_accel_mode = 0xff;
uint8_t last_gyro_mode = 0xff;
uint8_t last_accel_odr = 0xff;
uint8_t last_gyro_odr = 0xff;

void host_bus_reset(void);
void host_bus_fail_read(int reg);
void host_bus_set_register(uint8_t reg, uint8_t value);
uint8_t host_bus_register(uint8_t reg);
size_t host_bus_write_count(void);
uint8_t host_bus_write_reg(size_t index);
uint8_t host_bus_write_value(size_t index);
void host_bus_fifo(uint8_t reg, bool bytes);
size_t host_bus_interval_bytes(void);
size_t host_bus_interval_reads(void);

static unsigned writes_value(uint8_t reg, uint8_t value)
{
	unsigned count = 0;
	for (size_t i = 0; i < host_bus_write_count(); i++)
		count += host_bus_write_reg(i) == reg && host_bus_write_value(i) == value;
	return count;
}

struct icm_case {
	int (*init)(float, float, float, float *, float *);
	uint16_t (*fifo_read)(uint8_t *, uint16_t);
	void (*accel_read)(float *);
	void (*gyro_read)(float *);
	uint8_t (*drdy)(uint16_t);
	uint8_t count, clock_pin, accel, gyro, watermark, source;
	float accel_range, gyro_range;
};

static void test_icm(const struct icm_case *d)
{
	float a, g;
	host_bus_reset();
	host_bus_fail_read(d->count);
	assert(d->init(32768, .0025f, .0025f, &a, &g) == -1);
	/* Failure must not trigger the internal-clock fallback writes. */
	assert(writes_value(d->clock_pin, 0) == 0);
	host_bus_reset();
	assert(d->init(32768, .0025f, .0025f, &a, &g) == 0);
	assert(writes_value(d->clock_pin, 0) == 1);
	host_bus_reset();
	host_bus_set_register(d->count + 1, 1);
	assert(d->init(32768, .0025f, .0025f, &a, &g) == 0);
	assert(writes_value(d->clock_pin, 0) == 0);
	assert(fabsf(a - .002f * 32000 / 32768) < 1e-7f);

	float v[3];
	host_bus_reset();
	host_bus_set_register(d->accel, 0x40);
	host_bus_set_register(d->accel + 2, 0x80);
	d->accel_read(v);
	assert(v[0] == d->accel_range / 2 && v[1] == -d->accel_range && v[2] == 0);
	host_bus_set_register(d->gyro, 0x40);
	d->gyro_read(v);
	assert(v[0] == d->gyro_range / 2);
	host_bus_fail_read(d->accel);
	d->accel_read(v);
	assert(v[0] == 0 && v[1] == 0 && v[2] == 0);
	host_bus_fail_read(d->gyro);
	d->gyro_read(v);
	assert(v[0] == 0 && v[1] == 0 && v[2] == 0);
	host_bus_fail_read(-1);
	assert(d->drdy(0xFAB) == (NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW));
	assert(host_bus_register(d->watermark) == 0xAB);
	assert(host_bus_register(d->watermark + 1) == 0x0F);
	assert(host_bus_register(d->source) == 0x04);

	/* Large hardware counts are clamped before multiplication/narrowing. */
	uint8_t guarded[44];
	memset(guarded, 0xCC, sizeof(guarded));
	host_bus_reset();
	host_bus_fifo(d->count, false);
	host_bus_set_register(d->count, 0xFF);
	host_bus_set_register(d->count + 1, 0xFF);
	assert(d->fifo_read(guarded + 1, 41) == 2);
	assert(host_bus_interval_bytes() == 40);
	assert(guarded[0] == 0xCC && guarded[41] == 0xCC && guarded[42] == 0xCC);
}

static void test_bmi_boundary(unsigned reported, unsigned capacity)
{
	uint8_t guarded[64];
	assert(capacity + 2 <= sizeof(guarded));
	memset(guarded, 0xCC, sizeof(guarded));
	host_bus_reset();
	host_bus_fifo(BMI270_FIFO_LENGTH_0, true);
	host_bus_set_register(BMI270_FIFO_LENGTH_0, reported);
	host_bus_set_register(BMI270_FIFO_LENGTH_0 + 1, reported >> 8);
	unsigned packets = reported / 12;
	if (packets > capacity / 12) packets = capacity / 12;
	assert(bmi_fifo_read(guarded + 1, capacity) == packets);
	assert(host_bus_interval_bytes() == packets * 12);
	assert(host_bus_interval_reads() == (packets != 0));
	assert(guarded[0] == 0xCC);
	for (unsigned i = packets * 12 + 1; i < sizeof(guarded); i++) assert(guarded[i] == 0xCC);
}

int main(int argc, char **argv)
{
	const struct icm_case drivers[] = {
		{icm_init, icm_fifo_read, icm_accel_read, icm_gyro_read, icm_setup_DRDY,
		 ICM42688_FIFO_COUNTH, ICM42688_INTF_CONFIG5, ICM42688_ACCEL_DATA_X1,
		 ICM42688_GYRO_DATA_X1, ICM42688_FIFO_CONFIG2, ICM42688_INT_SOURCE0, 16, 2000},
		{icm42686_init, icm42686_fifo_read, icm42686_accel_read, icm42686_gyro_read, icm42686_setup_DRDY,
		 ICM42686_FIFO_COUNTH, ICM42686_INTF_CONFIG5, ICM42686_ACCEL_DATA_X1,
		 ICM42686_GYRO_DATA_X1, ICM42686_FIFO_CONFIG2, ICM42686_INT_SOURCE0, 32, 4000},
	};
	if (argc == 1 || !strcmp(argv[1], "icm")) {
		for (unsigned i = 0; i < sizeof(drivers) / sizeof(drivers[0]); i++) test_icm(&drivers[i]);
	}
	if (argc == 1 || !strcmp(argv[1], "bmi")) {
		for (unsigned count = 1; count < 12; count++) test_bmi_boundary(count, 12);
		test_bmi_boundary(0, 12);
		test_bmi_boundary(12, 12);
		test_bmi_boundary(13, 12);
		test_bmi_boundary(25, 24);
		test_bmi_boundary(25, 25);
		test_bmi_boundary(24, 11);
		test_bmi_boundary(24, 0);
	}

	if (argc == 1 || !strcmp(argv[1], "lsm")) {
		float a = -42, g = -43;
		host_bus_reset();
		host_bus_fail_read(LSM6DSO_INTERNAL_FREQ_FINE);
		assert(lsm6dso_init(0, .0025f, .0025f, &a, &g) == -1);
		assert(a == -42 && g == -43);
		assert(host_bus_write_count() == 1); /* no ODR/FIFO writes after the failed probe */
		host_bus_reset();
		host_bus_set_register(LSM6DSO_INTERNAL_FREQ_FINE, (uint8_t)-10);
		assert(lsm6dso_init(0, .0025f, .0025f, &a, &g) == 0);
		assert(a > 0 && g > 0);
	}
	puts("IMU I/O: failed probes, zero-only fallback, full-packet capacity and family behavior pass");
	return 0;
}
