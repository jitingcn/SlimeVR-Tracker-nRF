/* Link the complete production BMI270 driver. Only the bus and time leaves are
 * modeled; ASIC config upload, retrim, ODR restoration and errors run unchanged. */
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sensor/imu/BMI270.h"

static uint8_t registers[256];
static const uint8_t measured_gain[3] = {0x12, 0x34, 0x56};
static bool in_crt, reinitializing, fail_reinit;
static unsigned uploaded_bytes;

int sensor_interface_spi_configure(enum sensor_interface_dev dev, uint32_t frequency, uint32_t dummy_reads)
{
	(void)dev; (void)frequency; (void)dummy_reads;
	reinitializing = in_crt;
	return 0;
}

int ssi_reg_write_byte(enum sensor_interface_dev dev, uint8_t reg, uint8_t value)
{
	(void)dev;
	if (fail_reinit && reinitializing && reg == BMI270_ACC_RANGE) {
		return -EIO;
	}
	if (reg == BMI270_CMD && value == 0xB6) {
		memset(registers, 0, sizeof(registers));
		uploaded_bytes = 0;
	} else if (reg == BMI270_CMD && value == 0x02) {
		/* The hardware completes CRT successfully before the next poll. */
		registers[BMI270_GYR_CRT_CONF] = 0;
		registers[BMI270_GYR_GAIN_STATUS] = 0;
		memcpy(&registers[BMI270_GYR_USR_GAIN], measured_gain, sizeof(measured_gain));
	} else {
		registers[reg] = value;
		if (reg == BMI270_INIT_CTRL && value == 1 && uploaded_bytes != 0) {
			registers[BMI270_INTERNAL_STATUS] = 1;
		}
	}
	return 0;
}

int ssi_reg_read_byte(enum sensor_interface_dev dev, uint8_t reg, uint8_t *value)
{
	(void)dev;
	*value = registers[reg];
	return 0;
}

int ssi_burst_write(enum sensor_interface_dev dev, uint8_t reg, const uint8_t *buf, uint32_t len)
{
	(void)dev;
	if (reg == BMI270_INIT_DATA) {
		uploaded_bytes += len;
	} else {
		assert((unsigned)reg + len <= sizeof(registers));
		memcpy(&registers[reg], buf, len);
	}
	return 0;
}

int ssi_burst_read(enum sensor_interface_dev dev, uint8_t reg, uint8_t *buf, uint32_t len)
{
	(void)dev;
	assert((unsigned)reg + len <= sizeof(registers));
	memcpy(buf, &registers[reg], len);
	return 0;
}

int main(int argc, char **argv)
{
	fail_reinit = argc > 1 && strcmp(argv[1], "reinit-error") == 0;
	float accel_time, gyro_time;
	assert(bmi_init(0, 1.0f / 100.0f, 1.0f / 200.0f, &accel_time, &gyro_time) == 0);
	uint8_t accel_config = registers[BMI270_ACC_CONF];
	uint8_t gyro_config = registers[BMI270_GYR_CONF];
	uint8_t power_config = registers[BMI270_PWR_CTRL];
	in_crt = true;
	uint8_t gain[4] = {0};
	int result = bmi_crt(gain);
	if (fail_reinit) {
		assert(result == -EIO);
		/* A failed reinit must not leave the ODR cache claiming success. */
		registers[BMI270_ACC_CONF] = 0;
		registers[BMI270_GYR_CONF] = 0;
		fail_reinit = false;
		assert(bmi_update_odr(accel_time, gyro_time, &accel_time, &gyro_time) == 0);
	} else {
		assert(result == 0);
		assert(gain[0] == 1);
		assert(memcmp(gain + 1, measured_gain, sizeof(measured_gain)) == 0);
	}
	assert(registers[BMI270_INTERNAL_STATUS] == 1);
	assert(registers[BMI270_ACC_CONF] == accel_config);
	assert(registers[BMI270_GYR_CONF] == gyro_config);
	assert(registers[BMI270_PWR_CTRL] == power_config);
	puts("BMI270 retrim bus regression passed");
	return 0;
}
