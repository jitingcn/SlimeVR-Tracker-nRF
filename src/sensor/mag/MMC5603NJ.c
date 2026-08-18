#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include "MMC5603NJ.h"

static const float sensitivity = (1.0f / 16384.0f); // mag sensitivity if using 20 bit data (16384 Counts/G)
static const float offset = 524288.0f; // mag range unsigned to signed

static uint16_t last_state = 0xffff;
static float last_time = 0;
static uint8_t last_rawTemp = 0xff;
static int64_t oneshot_trigger_time = 0;
static bool oneshot_pending;
static bool oneshot_failed;
static bool auto_set_reset = true;

LOG_MODULE_REGISTER(MMC5603NJ, LOG_LEVEL_DBG);

static int mmc5603_SET(void);
static int mmc5603_RESET(void);

int mmc5603_init(float time, float *actual_time)
{
	int err = mmc5603_SET();
	if (err)
		return err;

	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_CONTROL_0, MCTRL0_AUTO_SR_EN);
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
	return mmc5603_update_odr(time, actual_time);
}

void mmc5603_shutdown(void)
{
	// reset device
	last_state = 0xffff;
	oneshot_trigger_time = 0;
	oneshot_pending = false;
	oneshot_failed = false;
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_CONTROL_1, MCTRL1_SW_RESET); // Don't need to wait for MMC to finish reset
	if (err)
		LOG_ERR("Communication error");
}

int mmc5603_update_odr(float time, float *actual_time)
{
	int ODR;
	uint8_t MODR;
	uint8_t MBW;
	uint8_t HPOWER = 0;
	uint8_t MSET = MSET_2000; // always use lowest SET/RESET interval

	if (time <= 0 || time == INFINITY) // off interpreted as oneshot
		ODR = 0;
	else
		ODR = 1 / time;

	if (ODR > 255)
	{ // 1000Hz*1.2ms/1000ms = 120% active, hpower required
		MODR = 255;
		MBW = MBW_1_2ms;
		HPOWER = MCTRL2_HPOWER;
		time = 1.0 / 1000;
	}
	else if (ODR > 150) // Nominal working state, this should use as low power as possible
	{ // 255Hz*2ms/1000ms = 51% active
		MODR = 255;
		MBW = MBW_2_0ms;
		time = 1.0 / 255;
	}
	else if (ODR > 75)
	{ // 150Hz*3.5ms/1000ms = 52% active
		MODR = 150;
		MBW = MBW_3_5ms;
		time = 1.0 / 150;
	}
	else if (ODR > 50)
	{ // 75Hz*6.6ms/1000ms = 50% active
		MODR = 75;
		MBW = MBW_6_6ms; // 1.5mG RMS noise
		time = 1.0 / 75;
	}
	else if (ODR > 20)
	{ // 50Hz*6.6ms/1000ms = 33% active
		MODR = 50;
		MBW = MBW_6_6ms;
		time = 1.0 / 50;
	}
	else if (ODR > 10)
	{ // 20Hz*6.6ms/1000ms = 13% active
		MODR = 20;
		MBW = MBW_6_6ms;
		time = 1.0 / 20;
	}
	else if (ODR > 1)
	{ // 10Hz*6.6ms/1000ms = 7% active
		MODR = 10;
		MBW = MBW_6_6ms;
		time = 1.0 / 10;
	}
	else if (ODR > 0)
	{ // 1Hz*6.6ms/1000ms = 0.7% active
		MODR = 1;
		MBW = MBW_6_6ms;
		time = 1.0 / 1;
	}
	else
	{ // oneshot uses the fastest measurement time
		MODR = 0;
		MBW = MBW_1_2ms;
		time = INFINITY;
	}

	uint16_t state = MODR | ((uint16_t)MBW << 8) | ((HPOWER != 0) ? (1U << 10) : 0);
	if (last_state == state) {
		*actual_time = time;
		return 0; /* already configured — success for err|= callers */
	}

	// Order required by the datasheet: bandwidth, ODR, period enable, continuous mode.
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_CONTROL_1, MBW);
	if (err)
		goto error;
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_ODR, MODR);
	if (err)
		goto error;
	err = ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_MAG,
		MMC5603NJ_CONTROL_0,
		MCTRL0_AUTO_SR_EN | (MODR ? MCTRL0_CMM_FREQ_EN : 0)
	);
	if (err)
		goto error;
	err = ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_MAG,
		MMC5603NJ_CONTROL_2,
		HPOWER | (MODR ? (MCTRL2_CMM_EN | MCTRL2_EN_PRD_SET | MSET) : 0)
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

void mmc5603_mag_oneshot(void)
{
	/* This full-byte write clears persistent CMM_FREQ_EN in CONTROL_0. */
	last_state = 0xffff;
	// enable auto set/reset and trigger oneshot
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_CONTROL_0, (auto_set_reset ? MCTRL0_AUTO_SR_EN : 0) | MCTRL0_TAKE_MEAS_M);
	oneshot_failed = err != 0;
	oneshot_pending = true;
	oneshot_trigger_time = k_uptime_get();
	if (err)
		LOG_ERR("Communication error");
}

bool mmc5603_mag_read(float m[3])
{
	if (oneshot_pending) {
		if (oneshot_failed) {
			oneshot_pending = false;
			oneshot_failed = false;
			return false;
		}

		uint8_t status = 0;
		int64_t timeout = oneshot_trigger_time + 4;
		while (!(status & MSTAT_MEAS_M_DONE)) {
			int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_STATUS, &status);
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

	uint8_t rawData[9]; // x/y/z mag register data stored here
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_XOUT_0, &rawData[0], 9); // Read the 9 raw data registers into data array
	if (err)
	{
		LOG_ERR("Communication error");
		return false;
	}
	mmc5603_mag_process(rawData, m);
	return true;
}

// MMC must trigger the measurement, which will take significant time
// instead, the temperature is read from the last measurement and then another measurement is immediately triggered
float mmc5603_temp_read(float bias[3])
{
	uint8_t rawTemp;
	int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_TOUT, &rawTemp);
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

		err = mmc5603_update_odr(INFINITY, &actual_time);
		auto_set_reset = false;
		if (!err)
			err = mmc5603_RESET();
		if (!err) {
			mmc5603_mag_oneshot();
			err = mmc5603_mag_read(mNeg) ? 0 : -EIO;
		}
		if (!err)
			err = mmc5603_SET();
		if (!err) {
			mmc5603_mag_oneshot();
			err = mmc5603_mag_read(mPos) ? 0 : -EIO;
		}
		if (!err) {
			for (int i = 0; i < 3; i++)
				new_bias[i] = (mPos[i] + mNeg[i]) / 2;
		}

		auto_set_reset = true;
		int restore_err = mmc5603_update_odr(restore_time, &actual_time);
		if (err || restore_err) {
			LOG_ERR("Communication error");
			return NAN;
		}
		memcpy(bias, new_bias, sizeof(new_bias));
		last_rawTemp = rawTemp;
	}

	// enable auto set/reset and trigger measurement
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_CONTROL_0, MCTRL0_AUTO_SR_EN | MCTRL0_TAKE_MEAS_T);
	if (err) {
		LOG_ERR("Communication error");
		return NAN;
	}
	return temp;
}

void mmc5603_mag_process(uint8_t *raw_m, float m[3])
{
	uint32_t rawMag[3];
	rawMag[0] = (uint32_t)(raw_m[0] << 12 | raw_m[1] << 4 | (raw_m[6] & 0xF0) >> 4); // Turn the 20 bits into a unsigned 32-bit value
	rawMag[1] = (uint32_t)(raw_m[2] << 12 | raw_m[3] << 4 | (raw_m[7] & 0xF0) >> 4); // Turn the 20 bits into a unsigned 32-bit value
	rawMag[2] = (uint32_t)(raw_m[4] << 12 | raw_m[5] << 4 | (raw_m[8] & 0xF0) >> 4); // Turn the 20 bits into a unsigned 32-bit value
	for (int i = 0; i < 3; i++) // x, y, z
		m[i] = ((float)rawMag[i] - offset) * sensitivity;
}

static int mmc5603_SET(void)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_CONTROL_0, MCTRL0_DO_SET);
	if (err) {
		LOG_ERR("Communication error");
		return err;
	}
	k_busy_wait(1000); // self clearing after 375 ns, t_SR 1ms before other operations
	return 0;
}

static int mmc5603_RESET(void)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_CONTROL_0, MCTRL0_DO_RESET);
	if (err) {
		LOG_ERR("Communication error");
		return err;
	}
	k_busy_wait(1000); // self clearing after 375 ns, t_SR 1ms before other operations
	return 0;
}

const sensor_mag_t sensor_mag_mmc5603nj = {
	*mmc5603_init,
	*mmc5603_shutdown,

	*mmc5603_update_odr,

	*mmc5603_mag_oneshot,
	*mmc5603_mag_read,
	*mmc5603_temp_read,

	*mmc5603_mag_process,
	6, 9 // if only reading 6 bytes, the data will be lower precision
};
