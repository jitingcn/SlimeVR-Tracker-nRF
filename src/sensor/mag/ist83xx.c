#include <math.h>
#include <zephyr/logging/log.h>

#include "IST8306.h"
#include "IST8308.h"
#include "ist83xx.h"

LOG_MODULE_REGISTER(IST83xx, LOG_LEVEL_DBG);

int ist83xx_init(struct ist83xx_state *state, bool drive_200, float period_s, float *actual_period_s)
{
	state->last_mode_code = 0xff; // invalidate mode cache
	state->oneshot_pending = false;
	state->oneshot_failed = false;
	int err = 0;
	if (drive_200) {
		err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, IST8308_CNTL4, DR_200);
		if (err) return err;
	}
	err = ist83xx_update_odr(state, period_s, actual_period_s);
	return (err < 0 ? err : 0);
}

void ist83xx_shutdown(struct ist83xx_state *state)
{
	state->last_mode_code = 0xff; // invalidate mode cache
	state->oneshot_pending = false;
	state->oneshot_failed = false;
	ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, IST8306_CNTL3, 0x01); // soft reset
}

int ist83xx_update_odr(struct ist83xx_state *state, float period_s, float *actual_period_s)
{
	float requested_odr_hz;
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
	if (state->last_mode_code == requested_mode_code) {
		*actual_period_s = period_s;
		return 0; /* already configured */
	}

	if (mode_code == MODE_SINGLE) {
		mode_code = MODE_STANDBY; // set STBY, oneshot will set SMM
	}

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, IST8306_CNTL1, noise_filter_code << 5);
	if (err) {
		goto error;
	}
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, IST8306_CNTL2, mode_code);
	if (err) {
		goto error;
	}
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, IST8306_OSRCNTL, oversampling_code);
	if (err) {
		goto error;
	}

	state->last_mode_code = requested_mode_code;
	state->oneshot_pending = false;
	state->oneshot_failed = false;
	mag_frame_reset(&state->frame_filter, period_s);
	*actual_period_s = period_s;
	return 0;
error:
	state->last_mode_code = 0xff;
	return err;
}

void ist83xx_mag_oneshot(struct ist83xx_state *state)
{
	state->last_mode_code = 0xff;
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, IST8306_CNTL2, MODE_SINGLE); // set single measurement mode
	state->oneshot_trigger_ms = k_uptime_get();
	state->oneshot_pending = true;
	state->oneshot_failed = err != 0;
}

bool ist83xx_mag_read(struct ist83xx_state *state, float sensitivity, float m[3])
{
	if (state->oneshot_pending && state->oneshot_failed) {
		state->oneshot_pending = false;
		state->oneshot_failed = false;
		return false;
	}

	uint8_t frame[7];
	if (state->oneshot_pending) {
		int64_t deadline_ms = state->oneshot_trigger_ms + 5;
		while (true) {
			int err = ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, IST8306_STAT, frame, sizeof(frame));
			if (err) {
				state->oneshot_pending = false;
				return false;
			}
			if (frame[0] & 0x01) {
				break;
			}
			if (k_uptime_get() >= deadline_ms) {
				LOG_ERR("Read timeout");
				state->oneshot_pending = false;
				return false;
			}
		}
		state->oneshot_pending = false;
	} else if (sensor_interface_mag_free_running()) {
		// Unobserved hub reads consume the read-clear DRDY; detect new data by payload.
		if (state->last_mode_code == 0xff || state->last_mode_code == MODE_STANDBY
			|| state->last_mode_code == MODE_SINGLE
			|| ssi_burst_read_frame(SENSOR_INTERFACE_DEV_MAG, IST8306_STAT, 0, 1, frame, sizeof(frame))
			|| !mag_frame_is_new(&state->frame_filter, &frame[1], sizeof(frame) - 1)) {
			return false;
		}
	} else {
		int err = ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, IST8306_STAT, frame, sizeof(frame));
		if (err) {
			return false;
		}
		if (!(frame[0] & 0x01)) {
			return false;
		}
	}
	ist83xx_mag_process(&frame[1], sensitivity, m);
	return true;
}

void ist83xx_mag_process(uint8_t *raw_m, float sensitivity, float m[3])
{
	for (int i = 0; i < 3; i++) // x, y, z
	{
		m[i] = (int16_t)((((uint16_t)raw_m[(i * 2) + 1]) << 8) | raw_m[i * 2]);
		m[i] *= sensitivity; // LSB to uT
		m[i] /= 100;         // uT to gauss
	}
}

