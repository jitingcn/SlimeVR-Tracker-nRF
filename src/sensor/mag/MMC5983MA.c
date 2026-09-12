/* 09/23/2017 Copyright Tlera Corporation

	Created by Kris Winer

  The MMC5983MA is a low power magnetometer, here used as 3 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/
#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include "MMC5983MA.h"

static const float sensitivity = (1.0f / 16384.0f); // mag sensitivity if using 18 bit data (16384 Counts/G)
static const float offset = 131072.0f; // mag range unsigned to signed

static uint16_t last_state = 0xffff;
static float last_continuous_period_s = 0;
static uint8_t last_offset_temperature_raw = 0xff;
static int64_t oneshot_trigger_ms = 0;
static bool oneshot_pending;
static bool oneshot_failed;
static bool auto_set_reset = true;

LOG_MODULE_REGISTER(MMC5983MA, LOG_LEVEL_DBG);

static int mmc_SET(void);
static int mmc_RESET(void);

int mmc_init(float period_s, float *actual_period_s)
{
	int err = mmc_SET();
	if (err)
		return err;

	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5983MA_CONTROL_0, MMC5983MA_CTRL0_AUTO_SR_EN);
	if (err) {
		LOG_ERR("Communication error");
		return err;
	}

	last_state = 0xffff;
	last_continuous_period_s = 0;
	last_offset_temperature_raw = 0xff;
	oneshot_trigger_ms = 0;
	oneshot_pending = false;
	oneshot_failed = false;
	auto_set_reset = true;
	return mmc_update_odr(period_s, actual_period_s);
}

void mmc_shutdown(void)
{
	// reset device
	last_state = 0xffff;
	oneshot_trigger_ms = 0;
	oneshot_pending = false;
	oneshot_failed = false;
	int err = ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_MAG,
		MMC5983MA_CONTROL_1,
		MMC5983MA_CTRL1_SW_RESET
	); // Reset completion is not polled here.
	if (err)
		LOG_ERR("Communication error");
}

int mmc_update_odr(float period_s, float *actual_period_s)
{
	int requested_odr_hz; // Truncate fractional Hz before selecting a supported rate.
	uint8_t odr_code;
	uint8_t bandwidth_code;
	uint8_t set_interval_code = MSET_2000;       // longest periodic SET interval: 2000 samples
	if (period_s <= 0 || period_s == INFINITY) { // off interpreted as oneshot
		requested_odr_hz = 0;
	} else
		requested_odr_hz = 1 / period_s;

	if (requested_odr_hz > 200) { // 1000Hz*0.5ms/1000ms = 50% active
		odr_code = MODR_1000Hz;
		bandwidth_code = MBW_800Hz; // Max MBW_800Hz
		period_s = 1.0 / 1000;
	} else if (requested_odr_hz > 100) // Nominal working state, this should use as low power as possible
	{                                  // 200Hz*0.5ms/1000ms = 10% active
		odr_code = MODR_200Hz;
		bandwidth_code = MBW_800Hz; // Existing BW=11 choice; Rev. A lists BW=01 for 200 Hz.
		period_s = 1.0 / 200;
	} else if (requested_odr_hz > 50) { // 100Hz*2ms/1000ms = 20% active
		odr_code = MODR_100Hz;
		bandwidth_code = MBW_400Hz; // 0.8mG
		period_s = 1.0 / 100;
	} else if (requested_odr_hz > 20) { // 50Hz*4ms/1000ms = 20% active
		odr_code = MODR_50Hz;
		bandwidth_code = MBW_200Hz; // 0.6mG
		period_s = 1.0 / 50;
	} else if (requested_odr_hz > 10) { // 20Hz*8ms/1000ms = 16% active
		odr_code = MODR_20Hz;
		bandwidth_code = MBW_100Hz; // 0.4mG
		period_s = 1.0 / 20;
	} else if (requested_odr_hz > 1) { // 10Hz*8ms/1000ms = 8% active
		odr_code = MODR_10Hz;
		bandwidth_code = MBW_100Hz;
		period_s = 1.0 / 10;
	} else if (requested_odr_hz > 0) { // 1Hz*8ms/1000ms = 0.8% active
		odr_code = MODR_1Hz;
		bandwidth_code = MBW_100Hz;
		period_s = 1.0 / 1;
	} else {
		odr_code = MODR_ONESHOT;
		bandwidth_code = MBW_800Hz;
		period_s = INFINITY;
	}

	uint16_t state = odr_code | ((uint16_t)bandwidth_code << 8);
	if (last_state == state) {
		*actual_period_s = period_s;
		return 0; /* already configured — success for err|= callers */
	}

	// Configure bandwidth before enabling the selected continuous rate.
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5983MA_CONTROL_1, bandwidth_code);
	if (err)
		goto error;
	err = ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_MAG,
		MMC5983MA_CONTROL_2,
		MMC5983MA_CTRL2_EN_PRD_SET | (set_interval_code << 4) | (odr_code ? MMC5983MA_CTRL2_CMM_EN : 0) | odr_code
	);
	if (err)
		goto error;

	last_state = state;
	oneshot_trigger_ms = 0;
	oneshot_pending = false;
	oneshot_failed = false;
	if (period_s == 0 || (period_s > 0 && period_s < INFINITY)) {
		last_continuous_period_s = period_s;
	}
	*actual_period_s = period_s;
	return 0;

error:
	last_state = 0xffff;
	LOG_ERR("Communication error");
	return err;
}

void mmc_mag_oneshot(void)
{
	/* This full-byte write changes CONTROL_0 independently of the ODR cache. */
	last_state = 0xffff;
	// Apply the current auto SET/RESET policy and trigger a magnetic measurement.
	int err = ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_MAG,
		MMC5983MA_CONTROL_0,
		(auto_set_reset ? MMC5983MA_CTRL0_AUTO_SR_EN : 0) | MMC5983MA_CTRL0_TAKE_MEAS_M
	);
	oneshot_failed = err != 0;
	oneshot_pending = true;
	oneshot_trigger_ms = k_uptime_get();
	if (err)
		LOG_ERR("Communication error");
}

bool mmc_mag_read(float m[3])
{
	if (oneshot_pending) {
		if (oneshot_failed) {
			oneshot_pending = false;
			oneshot_failed = false;
			return false;
		}

		uint8_t status = 0;
		int64_t deadline_ms = oneshot_trigger_ms + 2;
		while (!(status & MMC5983MA_STATUS_MEAS_M_DONE)) {
			int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, MMC5983MA_STATUS, &status);
			if (err) {
				LOG_ERR("Communication error");
				oneshot_pending = false;
				return false;
			}
			if (k_uptime_get() >= deadline_ms) {
				LOG_ERR("Read timeout");
				oneshot_pending = false;
				return false;
			}
		}
		oneshot_pending = false;
	}

	uint8_t rawData[7]; // x/y/z mag register data stored here
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, MMC5983MA_XOUT_0, &rawData[0], 7); // Read the 7 raw data registers into data array
	if (err)
	{
		LOG_ERR("Communication error");
		return false;
	}
	mmc_mag_process(rawData, m);
	return true;
}

// MMC must trigger the measurement, which will take significant time
// instead, the temperature is read from the last measurement and then another measurement is immediately triggered
float mmc_temp_read(float bias[3])
{
	uint8_t rawTemp;
	int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, MMC5983MA_TOUT, &rawTemp);
	if (err) {
		LOG_ERR("Communication error");
		return NAN;
	}
	// Temperature output, unsigned format. The range is -75~125°C, about 0.8°C/LSB, 00000000 stands for -75°C
	float temp = rawTemp;
	temp *= 0.8f;
	temp -= 75;

	// USING SET AND RESET TO REMOVE BRIDGE OFFSET in datasheet
	// The gate is the configured rate (<50 Hz), not a direct motion check.
	if (last_offset_temperature_raw != rawTemp && last_continuous_period_s > 1.0f / 50) {
		float set_sample[3], reset_sample[3];
		float new_bias[3];
		float actual_period_s;
		float restore_period_s = last_continuous_period_s;

		err = mmc_update_odr(INFINITY, &actual_period_s);
		auto_set_reset = false;
		if (!err)
			err = mmc_RESET();
		if (!err) {
			mmc_mag_oneshot();
			err = mmc_mag_read(reset_sample) ? 0 : -EIO;
		}
		if (!err)
			err = mmc_SET();
		if (!err) {
			mmc_mag_oneshot();
			err = mmc_mag_read(set_sample) ? 0 : -EIO;
		}
		if (!err) {
			for (int i = 0; i < 3; i++)
				new_bias[i] = (set_sample[i] + reset_sample[i]) / 2;
		}

		auto_set_reset = true;
		int restore_err = mmc_update_odr(restore_period_s, &actual_period_s);
		if (err || restore_err) {
			LOG_ERR("Communication error");
			return NAN;
		}
		memcpy(bias, new_bias, sizeof(new_bias));
		last_offset_temperature_raw = rawTemp;
	}

	// Enable auto SET/RESET and trigger the next temperature measurement.
	err = ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_MAG,
		MMC5983MA_CONTROL_0,
		MMC5983MA_CTRL0_AUTO_SR_EN | MMC5983MA_CTRL0_TAKE_MEAS_T
	);
	if (err) {
		LOG_ERR("Communication error");
		return NAN;
	}
	return temp;
}

void mmc_mag_process(uint8_t *raw_m, float m[3])
{
	uint32_t rawMag[3];
	rawMag[0] = (uint32_t)(raw_m[0] << 10 | raw_m[1] << 2 | (raw_m[6] & 0xC0) >> 6); // Turn the 18 bits into a unsigned 32-bit value
	rawMag[1] = (uint32_t)(raw_m[2] << 10 | raw_m[3] << 2 | (raw_m[6] & 0x30) >> 4); // Turn the 18 bits into a unsigned 32-bit value
	rawMag[2] = (uint32_t)(raw_m[4] << 10 | raw_m[5] << 2 | (raw_m[6] & 0x0C) >> 2); // Turn the 18 bits into a unsigned 32-bit value
	for (int i = 0; i < 3; i++) // x, y, z
		m[i] = ((float)rawMag[i] - offset) * sensitivity;
}

static int mmc_SET(void)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5983MA_CONTROL_0, MMC5983MA_CTRL0_SET);
	if (err) {
		LOG_ERR("Communication error");
		return err;
	}
	k_busy_wait(1); // Rev. A, Internal Control 0: SET self-clears after the 500 ns pulse.
	return 0;
}

static int mmc_RESET(void)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5983MA_CONTROL_0, MMC5983MA_CTRL0_RESET);
	if (err) {
		LOG_ERR("Communication error");
		return err;
	}
	k_busy_wait(1); // Rev. A, Internal Control 0: RESET self-clears after the 500 ns pulse.
	return 0;
}

const sensor_mag_t sensor_mag_mmc5983ma = {
	*mmc_init,
	*mmc_shutdown,

	*mmc_update_odr,

	*mmc_mag_oneshot,
	*mmc_mag_read,
	*mmc_temp_read,

	*mmc_mag_process,
	6, 7 // if only reading 6 bytes, the data will be lower precision
};
