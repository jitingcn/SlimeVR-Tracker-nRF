/* 09/23/2017 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Ladybug default), respectively, and it uses the Ladybug STM32L432 Breakout Board.
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
static float last_time = 0;
static uint8_t last_rawTemp = 0xff;
static int64_t oneshot_trigger_time = 0;
static bool oneshot_pending;
static bool oneshot_failed;
static bool auto_set_reset = true;

LOG_MODULE_REGISTER(MMC5983MA, LOG_LEVEL_DBG);

static int mmc_SET(void);
static int mmc_RESET(void);

int mmc_init(float time, float *actual_time)
{
	int err = mmc_SET();
	if (err)
		return err;

	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5983MA_CONTROL_0, 0x20);
	if (err) {
		LOG_ERR("Communication error");
		return err;
	}

	last_state = 0xffff;
	last_time = 0;
	last_rawTemp = 0xff;
	oneshot_trigger_time = 0;
	oneshot_pending = false;
	oneshot_failed = false;
	auto_set_reset = true;
	return mmc_update_odr(time, actual_time);
}

void mmc_shutdown(void)
{
	// reset device
	last_state = 0xffff;
	oneshot_trigger_time = 0;
	oneshot_pending = false;
	oneshot_failed = false;
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5983MA_CONTROL_1, 0x80); // Don't need to wait for MMC to finish reset
	if (err)
		LOG_ERR("Communication error");
}

int mmc_update_odr(float time, float *actual_time)
{
	int ODR;
	uint8_t MODR;
	uint8_t MBW;
	uint8_t MSET = MSET_2000; // always use lowest SET/RESET interval
	if (time <= 0 || time == INFINITY) // off interpreted as oneshot
		ODR = 0;
	else
		ODR = 1 / time;

	if (ODR > 200) // TODO: this sucks
	{ // 1000Hz*0.5ms/1000ms = 50% active
		MODR = MODR_1000Hz;
		MBW = MBW_800Hz; // Max MBW_800Hz
		time = 1.0 / 1000;
	}
	else if (ODR > 100) // Nominal working state, this should use as low power as possible
	{ // 200Hz*0.5ms/1000ms = 10% active
		MODR = MODR_200Hz;
		MBW = MBW_800Hz; // Max MBW_200Hz, MBW_800Hz is used since the RMS noise is still only 1.2mG
		time = 1.0 / 200;
	}
	else if (ODR > 50)
	{ // 100Hz*2ms/1000ms = 20% active
		MODR = MODR_100Hz;
		MBW = MBW_400Hz; // 0.8mG
		time = 1.0 / 100;
	}
	else if (ODR > 20)
	{ // 50Hz*4ms/1000ms = 20% active
		MODR = MODR_50Hz;
		MBW = MBW_200Hz; // 0.6mG
		time = 1.0 / 50;
	}
	else if (ODR > 10)
	{ // 20Hz*8ms/1000ms = 16% active
		MODR = MODR_20Hz;
		MBW = MBW_100Hz; // 0.4mG
		time = 1.0 / 20;
	}
	else if (ODR > 1)
	{ // 10Hz*8ms/1000ms = 8% active
		MODR = MODR_10Hz;
		MBW = MBW_100Hz;
		time = 1.0 / 10;
	}
	else if (ODR > 0)
	{ // 1Hz*8ms/1000ms = 0.8% active
		MODR = MODR_1Hz;
		MBW = MBW_100Hz;
		time = 1.0 / 1;
	}
	else
	{
		MODR = MODR_ONESHOT;
		MBW = MBW_800Hz;
		time = INFINITY;
	}

	uint16_t state = MODR | ((uint16_t)MBW << 8);
	if (last_state == state) {
		*actual_time = time;
		return 0; /* already configured — success for err|= callers */
	}

	// Configure bandwidth before enabling the selected continuous rate.
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5983MA_CONTROL_1, MBW);
	if (err)
		goto error;
	err = ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_MAG,
		MMC5983MA_CONTROL_2,
		0x80 | (MSET << 4) | (MODR ? 0x08 : 0) | MODR
	);
	if (err)
		goto error;

	last_state = state;
	oneshot_trigger_time = 0;
	oneshot_pending = false;
	oneshot_failed = false;
	if (time == 0 || (time > 0 && time < INFINITY))
		last_time = time;
	*actual_time = time;
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
	// enable auto set/reset (bit 5 == 1) and trigger oneshot
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5983MA_CONTROL_0, (auto_set_reset ? 0x20 : 0) | 0x01);
	oneshot_failed = err != 0;
	oneshot_pending = true;
	oneshot_trigger_time = k_uptime_get();
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
		int64_t timeout = oneshot_trigger_time + 2;
		while (!(status & 0x01)) {
			int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, MMC5983MA_STATUS, &status);
			if (err) {
				LOG_ERR("Communication error");
				oneshot_pending = false;
				return false;
			}
			if (k_uptime_get() >= timeout) {
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
	if (last_rawTemp != rawTemp && last_time > 1.0f / 50) // calculate offset at low motion only
	{ // TODO: does the temp register have hysteresis?
		float mPos[3], mNeg[3];
		float new_bias[3];
		float actual_time;
		float restore_time = last_time;

		err = mmc_update_odr(INFINITY, &actual_time);
		auto_set_reset = false;
		if (!err)
			err = mmc_RESET();
		if (!err) {
			mmc_mag_oneshot();
			err = mmc_mag_read(mNeg) ? 0 : -EIO;
		}
		if (!err)
			err = mmc_SET();
		if (!err) {
			mmc_mag_oneshot();
			err = mmc_mag_read(mPos) ? 0 : -EIO;
		}
		if (!err) {
			for (int i = 0; i < 3; i++)
				new_bias[i] = (mPos[i] + mNeg[i]) / 2;
		}

		auto_set_reset = true;
		int restore_err = mmc_update_odr(restore_time, &actual_time);
		if (err || restore_err) {
			LOG_ERR("Communication error");
			return NAN;
		}
		memcpy(bias, new_bias, sizeof(new_bias));
		last_rawTemp = rawTemp;
	}

	// enable auto set/reset (bit 5 == 1) and trigger measurement
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5983MA_CONTROL_0, 0x20 | 0x02);
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
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5983MA_CONTROL_0, 0x08);
	if (err) {
		LOG_ERR("Communication error");
		return err;
	}
	k_busy_wait(1); // self clearing after 500 ns
	return 0;
}

static int mmc_RESET(void)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5983MA_CONTROL_0, 0x10);
	if (err) {
		LOG_ERR("Communication error");
		return err;
	}
	k_busy_wait(1); // self clearing after 500 ns
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
