#include <math.h>

#include <zephyr/logging/log.h>

#include "MMC5603NJ.h"

static const float sensitivity = (1.0f / 16384.0f); // mag sensitivity if using 20 bit data (16384 Counts/G)
static const float offset = 524288.0f; // mag range unsigned to signed

static uint16_t last_state = 0xffff;
static float last_time = 0;
static uint8_t last_rawTemp = 0xff;
static int64_t oneshot_trigger_time = 0;
static bool auto_set_reset = true;

LOG_MODULE_REGISTER(MMC5603NJ, LOG_LEVEL_DBG);

static void mmc5603_SET(void);
static void mmc5603_RESET(void);

int mmc5603_init(float time, float *actual_time)
{
	// moved here, it is mmc specific
	mmc5603_SET(); // "deGauss" magnetometer

	// enable auto set/reset
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_CONTROL_0, MCTRL0_AUTO_SR_EN);
	if (err)
		LOG_ERR("Communication error");

	last_state = 0xffff; // reset last state
	err |= mmc5603_update_odr(time, actual_time);
	return (err < 0 ? err : 0);
}

void mmc5603_shutdown(void)
{
	// reset device
	last_state = 0xffff; // reset last state
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
	last_time = time;

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

	uint16_t state = MODR | (HPOWER << 1); // HPOWER is bit 7, shift it out of MODR byte range
	if (last_state == state) {
		*actual_time = time;
		return 0; /* already configured — success for err|= callers */
	}

	// set magnetometer bandwidth
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_CONTROL_1, MBW);

	// set sample rate, then start the measurement period calculation
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_ODR, MODR);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_CONTROL_0, MCTRL0_AUTO_SR_EN | (MODR ? MCTRL0_CMM_FREQ_EN : 0));

	// enable continuous measurement mode if ODR is set
	// enable periodic SET, set set/reset rate
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_CONTROL_2, HPOWER | (MODR ? (MCTRL2_CMM_EN | MCTRL2_EN_PRD_SET | MSET) : 0));
	if (err) {
		LOG_ERR("Communication error");
		return err;
	}

	last_state = state;
	*actual_time = time;
	return 0;
}

void mmc5603_mag_oneshot(void)
{
	// enable auto set/reset and trigger oneshot
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_CONTROL_0, (auto_set_reset ? MCTRL0_AUTO_SR_EN : 0) | MCTRL0_TAKE_MEAS_M);
	oneshot_trigger_time = k_uptime_get();
	if (err)
		LOG_ERR("Communication error");
}

bool mmc5603_mag_read(float m[3])
{
	int err = 0;
	uint8_t status = oneshot_trigger_time ? 0x00 : MSTAT_MEAS_M_DONE;
	int64_t timeout = oneshot_trigger_time + 4; // 4ms timeout (1.2ms measurement at BW=11)
	if (k_uptime_get() >= timeout) // already passed timeout
		oneshot_trigger_time = 0;
	while ((~status & MSTAT_MEAS_M_DONE) && k_uptime_get() < timeout) // wait for oneshot to complete or timeout
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_STATUS, &status);
	if (oneshot_trigger_time ? k_uptime_get() >= timeout : false)
		LOG_ERR("Read timeout");
	oneshot_trigger_time = 0;
	uint8_t rawData[9]; // x/y/z mag register data stored here
	err |= ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_XOUT_0, &rawData[0], 9); // Read the 9 raw data registers into data array
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
	// Temperature output, unsigned format. The range is -75~125°C, about 0.8°C/LSB, 00000000 stands for -75°C
	float temp = rawTemp;
	temp *= 0.8f;
	temp -= 75;

	// USING SET AND RESET TO REMOVE BRIDGE OFFSET in datasheet
	if (last_rawTemp != rawTemp && last_time > 1.0f / 50) // calculate offset at low motion only
	{ // TODO: does the temp register have hysteresis?
		float mPos[3], mNeg[3];
		float actual_time;

		last_rawTemp = rawTemp;
		for (int i = 0; i < 3; i++) // clear stored offset
			bias[i] = 0;

		err |= mmc5603_update_odr(INFINITY, &actual_time); // set oneshot mode
		auto_set_reset = false; // disable auto set/reset

		mmc5603_RESET();
		mmc5603_mag_oneshot();
		mmc5603_mag_read(mNeg);

		mmc5603_SET();
		mmc5603_mag_oneshot();
		mmc5603_mag_read(mPos);

		for (int i = 0; i < 3; i++) // store bridge offset for future readings
			bias[i] = (mPos[i] + mNeg[i]) / 2; // ((+H + Offset) + (-H + Offset)) / 2

		auto_set_reset = true; // enable auto set/reset
		err |= mmc5603_update_odr(last_time, &actual_time); // reset odr
	}

	// enable auto set/reset and trigger measurement
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_CONTROL_0, MCTRL0_AUTO_SR_EN | MCTRL0_TAKE_MEAS_T);
	if (err < 0)
		LOG_ERR("Communication error");
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

static void mmc5603_SET(void)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_CONTROL_0, MCTRL0_DO_SET);
	if (err)
		LOG_ERR("Communication error");
	k_busy_wait(1000); // self clearing after 375 ns, t_SR 1ms before other operations
}

static void mmc5603_RESET(void)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_CONTROL_0, MCTRL0_DO_RESET);
	if (err)
		LOG_ERR("Communication error");
	k_busy_wait(1000); // self clearing after 375 ns, t_SR 1ms before other operations
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
