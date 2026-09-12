#include <math.h>

#include <zephyr/logging/log.h>

#include "IST8306.h"
#include "sensor/sensor_none.h"

static const float sensitivity = 0.3; // uT/LSB

static uint8_t last_mode_code = 0xff;
static int64_t oneshot_trigger_ms = 0;
static bool oneshot_pending;
static bool oneshot_failed;

LOG_MODULE_REGISTER(IST8306, LOG_LEVEL_DBG);

int ist8306_init(float period_s, float *actual_period_s)
{
	last_mode_code = 0xff; // invalidate mode cache
	oneshot_pending = false;
	oneshot_failed = false;
	int err = ist8306_update_odr(period_s, actual_period_s);
	return (err < 0 ? err : 0);
}

void ist8306_shutdown(void)
{
	last_mode_code = 0xff; // invalidate mode cache
	oneshot_pending = false;
	oneshot_failed = false;
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, IST8306_CNTL3, 0x01); // soft reset
	if (err) {
		LOG_ERR("Communication error");
	}
}

int ist8306_update_odr(float period_s, float *actual_period_s)
{
	int requested_odr_hz; // Truncate fractional Hz before selecting a supported rate.
	uint8_t noise_filter_code;
	uint8_t mode_code;
	uint8_t oversampling_code;

	if (period_s <= 0 || period_s == INFINITY) // standby mode or single measurement mode
	{
		noise_filter_code = NSF_Low; // High Speed
		oversampling_code = OSR_8;
		mode_code = MODE_STANDBY;
		requested_odr_hz = 0;
	} else {
		requested_odr_hz = 1 / period_s;
	}

	if (period_s <= 0) {
		period_s = 0; // off
	} else if (requested_odr_hz > 100) {
		noise_filter_code = NSF_Low; // High Speed
		oversampling_code = OSR_8;
		mode_code = MODE_CMM_200Hz;
		period_s = 1.0 / 200;
	} else if (requested_odr_hz > 50) {
		noise_filter_code = NSF_Low; // Normal
		oversampling_code = OSR_16;
		mode_code = MODE_CMM_100Hz;
		period_s = 1.0 / 100;
	} else if (requested_odr_hz > 20) {
		noise_filter_code = NSF_Low; // Normal
		oversampling_code = OSR_16;
		mode_code = MODE_CMM_50Hz;
		period_s = 1.0 / 50;
	} else if (requested_odr_hz > 10) {
		noise_filter_code = NSF_Low; // Normal
		oversampling_code = OSR_16;
		mode_code = MODE_CMM_20Hz;
		period_s = 1.0 / 20;
	} else if (requested_odr_hz > 0) {
		noise_filter_code = NSF_Medium; // Low noise
		oversampling_code = OSR_32;
		mode_code = MODE_CMM_10Hz;
		period_s = 1.0 / 10;
	} else {
		noise_filter_code = NSF_Low; // High Speed
		oversampling_code = OSR_8;
		mode_code = MODE_SINGLE;
		period_s = INFINITY;
	}

	uint8_t requested_mode_code = mode_code;
	if (last_mode_code == requested_mode_code) {
		*actual_period_s = period_s;
		return 0; /* already configured */
	}

	if (mode_code == MODE_SINGLE) {
		mode_code = MODE_STANDBY; // set STBY, oneshot will set SMM
	}

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, IST8306_CNTL1, noise_filter_code << 5);
	if (err)
		goto error;
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, IST8306_CNTL2, mode_code);
	if (err)
		goto error;
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, IST8306_OSRCNTL, oversampling_code);
	if (err)
		goto error;

	last_mode_code = requested_mode_code;
	oneshot_pending = false;
	oneshot_failed = false;
	*actual_period_s = period_s;
	return 0;
error:
	last_mode_code = 0xff;
	LOG_ERR("Communication error");
	return err;
}

void ist8306_mag_oneshot(void)
{
	last_mode_code = 0xff;
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, IST8306_CNTL2, MODE_SINGLE); // set single measurement mode
	oneshot_trigger_ms = k_uptime_get();
	oneshot_pending = true;
	oneshot_failed = err != 0;
	if (err)
		LOG_ERR("Communication error");
}

bool ist8306_mag_read(float m[3])
{
	if (oneshot_pending && oneshot_failed) {
		oneshot_pending = false;
		oneshot_failed = false;
		return false;
	}

	uint8_t frame[7];
	if (oneshot_pending) {
		int64_t deadline_ms = oneshot_trigger_ms + 5;
		while (true) {
			int err = ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, IST8306_STAT, frame, sizeof(frame));
			if (err) {
				LOG_ERR("Communication error");
				oneshot_pending = false;
				return false;
			}
			if (frame[0] & 0x01)
				break;
			if (k_uptime_get() >= deadline_ms) {
				LOG_ERR("Read timeout");
				oneshot_pending = false;
				return false;
			}
		}
		oneshot_pending = false;
	} else {
		int err = ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, IST8306_STAT, frame, sizeof(frame));
		if (err) {
			LOG_ERR("Communication error");
			return false;
		}
		if (!(frame[0] & 0x01))
			return false;
	}
	ist8306_mag_process(&frame[1], m);
	return true;
}

void ist8306_mag_process(uint8_t *raw_m, float m[3])
{
	for (int i = 0; i < 3; i++) // x, y, z
	{
		m[i] = (int16_t)((((uint16_t)raw_m[(i * 2) + 1]) << 8) | raw_m[i * 2]);
		m[i] *= sensitivity; //LSB to uT
		m[i] /= 100; // uT to gauss
	}
}

const sensor_mag_t sensor_mag_ist8306 = {
	*ist8306_init,
	*ist8306_shutdown,

	*ist8306_update_odr,

	*ist8306_mag_oneshot,
	*ist8306_mag_read,
	*mag_none_temp_read,

	*ist8306_mag_process,
	7, 7
};
