#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include "MMC5603NJ.h"
#include "mag_frame.h"

static const float sensitivity = (1.0f / 16384.0f); // mag sensitivity if using 20 bit data (16384 Counts/G)
static const float offset = 524288.0f;              // mag range unsigned to signed

static struct mag_frame_filter frame_filter;
static uint16_t last_state = 0xffff;
static float last_continuous_period_s = 0;
static uint8_t last_offset_temperature_raw = 0xff;
static int64_t oneshot_trigger_ms = 0;
static bool oneshot_pending;
static bool oneshot_failed;
static bool auto_set_reset = true;

LOG_MODULE_REGISTER(MMC5603NJ, LOG_LEVEL_DBG);

static int mmc5603_SET(void);
static int mmc5603_RESET(void);

int mmc5603_init(float period_s, float *actual_period_s)
{
	int err = mmc5603_SET();
	if (err) {
		return err;
	}

	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_CONTROL_0, MCTRL0_AUTO_SR_EN);
	if (err) {
		return err;
	}

	last_state = 0xffff;
	last_continuous_period_s = 0;
	last_offset_temperature_raw = 0xff;
	oneshot_trigger_ms = 0;
	oneshot_pending = false;
	oneshot_failed = false;
	auto_set_reset = true;
	return mmc5603_update_odr(period_s, actual_period_s);
}

void mmc5603_shutdown(void)
{
	// reset device
	last_state = 0xffff;
	oneshot_trigger_ms = 0;
	oneshot_pending = false;
	oneshot_failed = false;
	ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_MAG,
		MMC5603NJ_CONTROL_1,
		MCTRL1_SW_RESET
	); // Reset completion is not polled here.
}

int mmc5603_update_odr(float period_s, float *actual_period_s)
{
	int requested_odr_hz; // Raise fractional Hz to the next integer ODR.
	uint8_t odr_code;
	uint8_t bandwidth_code;
	uint8_t high_power_bit = 0;
	uint8_t set_interval_code = MSET_2000; // longest periodic SET interval: 2000 samples

	if (period_s <= 0 || period_s == INFINITY) { // off interpreted as oneshot
		requested_odr_hz = 0;
	} else if (period_s < 1.0f / 1000) {
		requested_odr_hz = 1000;
	} else {
		requested_odr_hz = (int)ceilf(1 / period_s - 0.001f);
	}

	// The ODR register takes any integer rate in 1..255 Hz. Each bandwidth
	// is kept at or below about half active time.
	if (requested_odr_hz > 255) { // 1000Hz*1.2ms/1000ms = 120% active, hpower required
		odr_code = 255;
		bandwidth_code = MBW_1_2ms;
		high_power_bit = MCTRL2_HPOWER;
		period_s = 1.0 / 1000;
	} else if (requested_odr_hz > 0) {
		odr_code = requested_odr_hz;
		if (requested_odr_hz > 150) { // 255Hz*2ms/1000ms = 51% active
			bandwidth_code = MBW_2_0ms;
		} else if (requested_odr_hz > 75) { // 150Hz*3.5ms/1000ms = 52% active
			bandwidth_code = MBW_3_5ms;
		} else { // 75Hz*6.6ms/1000ms = 50% active, 1.5mG RMS noise
			bandwidth_code = MBW_6_6ms;
		}
		period_s = 1.0f / odr_code;
	} else { // oneshot uses the fastest measurement time
		odr_code = 0;
		bandwidth_code = MBW_1_2ms;
		period_s = INFINITY;
	}

	uint16_t state = odr_code | ((uint16_t)bandwidth_code << 8) | ((high_power_bit != 0) ? (1U << 10) : 0);
	if (last_state == state) {
		*actual_period_s = period_s;
		return 0; /* already configured — success for err|= callers */
	}

	// Set bandwidth, then ODR -> Cmm_freq_en -> Cmm_en (Rev. B, p. 13).
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_CONTROL_1, bandwidth_code);
	if (err) {
		goto error;
	}
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_ODR, odr_code);
	if (err) {
		goto error;
	}
	err = ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_MAG,
		MMC5603NJ_CONTROL_0,
		MCTRL0_AUTO_SR_EN | (odr_code ? MCTRL0_CMM_FREQ_EN : 0)
	);
	if (err) {
		goto error;
	}
	err = ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_MAG,
		MMC5603NJ_CONTROL_2,
		high_power_bit | (odr_code ? (MCTRL2_CMM_EN | MCTRL2_EN_PRD_SET | set_interval_code) : 0)
	);
	if (err) {
		goto error;
	}

	last_state = state;
	oneshot_trigger_ms = 0;
	oneshot_pending = false;
	oneshot_failed = false;
	mag_frame_reset(&frame_filter, odr_code ? period_s : 0.0f);
	if (period_s == 0 || (period_s > 0 && period_s < INFINITY)) {
		last_continuous_period_s = period_s;
	}
	*actual_period_s = period_s;
	return 0;

error:
	last_state = 0xffff;
	return err;
}

void mmc5603_mag_oneshot(void)
{
	/* Reapply rate configuration after this full-byte command. Cmm_freq_en
	 * self-clears after calculating the period (Rev. B, Internal Control 0). */
	last_state = 0xffff;
	// enable auto set/reset and trigger oneshot
	int err = ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_MAG,
		MMC5603NJ_CONTROL_0,
		(auto_set_reset ? MCTRL0_AUTO_SR_EN : 0) | MCTRL0_TAKE_MEAS_M
	);
	oneshot_failed = err != 0;
	oneshot_pending = true;
	oneshot_trigger_ms = k_uptime_get();
}

bool mmc5603_mag_read(float m[3])
{
	bool was_oneshot = oneshot_pending;
	if (oneshot_pending) {
		if (oneshot_failed) {
			oneshot_pending = false;
			oneshot_failed = false;
			return false;
		}

		uint8_t status = 0;
		int64_t deadline_ms = oneshot_trigger_ms + 4;
		while (!(status & MSTAT_MEAS_M_DONE)) {
			int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_STATUS, &status);
			if (err) {
				oneshot_pending = false;
				return false;
			}
			if (status & MSTAT_MEAS_M_DONE) {
				break;
			}
			if (k_uptime_get() >= deadline_ms) {
				LOG_ERR("Read timeout");
				oneshot_pending = false;
				return false;
			}
			k_usleep(100);
		}
		oneshot_pending = false;
	} else if (last_state == 0xffff || (last_state & 0xff) == 0) {
		return false;
	}

	uint8_t rawData[9]; // x/y/z mag register data stored here
	int err = was_oneshot ? ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_XOUT_0, rawData, sizeof(rawData))
			      : ssi_burst_read_frame(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_XOUT_0, 0, 1, rawData, sizeof(rawData));
	if (err) {
		return false;
	}
	// Continuous output has no data-ready bit; the read schedule can repeat a sample.
	if (!was_oneshot && !mag_frame_is_new(&frame_filter, rawData, sizeof(rawData))) {
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

		err = mmc5603_update_odr(INFINITY, &actual_period_s);
		auto_set_reset = false;
		if (!err) {
			err = mmc5603_RESET();
		}
		if (!err) {
			mmc5603_mag_oneshot();
			err = mmc5603_mag_read(reset_sample) ? 0 : -EIO;
		}
		if (!err) {
			err = mmc5603_SET();
		}
		if (!err) {
			mmc5603_mag_oneshot();
			err = mmc5603_mag_read(set_sample) ? 0 : -EIO;
		}
		if (!err) {
			for (int i = 0; i < 3; i++) {
				new_bias[i] = (set_sample[i] + reset_sample[i]) / 2;
			}
		}

		auto_set_reset = true;
		int restore_err = mmc5603_update_odr(restore_period_s, &actual_period_s);
		if (err || restore_err) {
			return NAN;
		}
		memcpy(bias, new_bias, sizeof(new_bias));
		last_offset_temperature_raw = rawTemp;
	}

	// enable auto set/reset and trigger measurement
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_CONTROL_0, MCTRL0_AUTO_SR_EN | MCTRL0_TAKE_MEAS_T);
	if (err) {
		return NAN;
	}
	return temp;
}

void mmc5603_mag_process(uint8_t *raw_m, float m[3])
{
	uint32_t rawMag[3];
	rawMag[0] = (uint32_t)(raw_m[0] << 12 | raw_m[1] << 4
						   | (raw_m[6] & 0xF0) >> 4); // Turn the 20 bits into a unsigned 32-bit value
	rawMag[1] = (uint32_t)(raw_m[2] << 12 | raw_m[3] << 4
						   | (raw_m[7] & 0xF0) >> 4); // Turn the 20 bits into a unsigned 32-bit value
	rawMag[2] = (uint32_t)(raw_m[4] << 12 | raw_m[5] << 4
						   | (raw_m[8] & 0xF0) >> 4); // Turn the 20 bits into a unsigned 32-bit value
	for (int i = 0; i < 3; i++) {                     // x, y, z
		m[i] = ((float)rawMag[i] - offset) * sensitivity;
	}
}

static int mmc5603_SET(void)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_CONTROL_0, MCTRL0_DO_SET);
	if (err) {
		return err;
	}
	k_busy_wait(1000); // Rev. B, p. 15: t_SR >= 1 ms; the SET pulse itself is 375 ns
	return 0;
}

static int mmc5603_RESET(void)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603NJ_CONTROL_0, MCTRL0_DO_RESET);
	if (err) {
		return err;
	}
	k_busy_wait(1000); // Rev. B, p. 15: t_SR >= 1 ms; the RESET pulse itself is 375 ns
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
	6,
	9 // if only reading 6 bytes, the data will be lower precision
};
