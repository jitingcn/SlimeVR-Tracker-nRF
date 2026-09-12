#include <math.h>

#include <zephyr/logging/log.h>

#include "AK09940.h"

static const float sensitivity = 10; // nT/LSB

static uint8_t last_mode_code = 0xff;
static int64_t oneshot_trigger_ms = 0;
static bool oneshot_pending;
static bool oneshot_failed;

LOG_MODULE_REGISTER(AK09940, LOG_LEVEL_DBG);

int ak_init(float period_s, float *actual_period_s)
{
	last_mode_code = 0xff; // invalidate mode cache
	oneshot_pending = false;
	oneshot_failed = false;
	int err = ak_update_odr(period_s, actual_period_s);
	return (err < 0 ? err : 0);
}

void ak_shutdown(void)
{
	last_mode_code = 0xff; // invalidate mode cache
	oneshot_pending = false;
	oneshot_failed = false;
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, AK09940_CNTL4, 0x01);
	if (err)
		LOG_ERR("Communication error");
}

int ak_update_odr(float period_s, float *actual_period_s)
{
	int requested_odr_hz; // Truncate fractional Hz before selecting a supported rate.
	uint8_t mode_code;

	if (period_s <= 0 || period_s == INFINITY) // power down mode or single measurement mode
	{
		mode_code = MODE_PDM;
		requested_odr_hz = 0;
	} else {
		requested_odr_hz = 1 / period_s;
	}

	if (period_s <= 0) {
		period_s = 0;                    // off
	} else if (requested_odr_hz > 100) { // only up to 200Hz supported with MT_LND2
		mode_code = MODE_CMM5_200Hz;
		period_s = 1.0 / 200;
	} else if (requested_odr_hz > 50) {
		mode_code = MODE_CMM4_100Hz;
		period_s = 1.0 / 100;
	} else if (requested_odr_hz > 20) {
		mode_code = MODE_CMM3_50Hz;
		period_s = 1.0 / 50;
	} else if (requested_odr_hz > 10) {
		mode_code = MODE_CMM2_20Hz;
		period_s = 1.0 / 20;
	} else if (requested_odr_hz > 0) {
		mode_code = MODE_CMM1_10Hz;
		period_s = 1.0 / 10;
	} else {
		mode_code = MODE_SMM;
		/* Single-measurement throughput remains unverified; do not advertise INFINITY. */
		period_s = 0;
	}

	uint8_t requested_mode_code = mode_code;
	if (last_mode_code == requested_mode_code) {
		*actual_period_s = period_s;
		return 0; /* already configured */
	}

	if (mode_code == MODE_SMM) {
		mode_code = MODE_PDM; // set PDM, oneshot will set SMM
	}

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, AK09940_CNTL3, MT_LND2 << 5 | mode_code);
	if (err) {
		last_mode_code = 0xff;
		LOG_ERR("Communication error");
		return err;
	}

	last_mode_code = requested_mode_code;
	oneshot_pending = false;
	oneshot_failed = false;
	*actual_period_s = period_s;
	return 0;
}

void ak_mag_oneshot(void)
{
	last_mode_code = 0xff;
	int err = ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_MAG,
		AK09940_CNTL3,
		MT_LND2 << 5 | MODE_SMM
	); // low-noise drive 2, single measurement
	oneshot_trigger_ms = k_uptime_get();
	oneshot_pending = true;
	oneshot_failed = err != 0;
	if (err)
		LOG_ERR("Communication error");
}

bool ak_mag_read(float m[3])
{
	if (oneshot_pending && oneshot_failed) {
		oneshot_pending = false;
		oneshot_failed = false;
		return false;
	}

	uint8_t frame[12];
	if (oneshot_pending) {
		int64_t deadline_ms = oneshot_trigger_ms + 2;
		while (true) {
			int err = ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, AK09940_ST1, frame, sizeof(frame));
			if (err) {
				LOG_ERR("Communication error");
				oneshot_pending = false;
				return false;
			}
			if (frame[0] & AK09940_ST1_DRDY)
				break;
			if (k_uptime_get() >= deadline_ms) {
				LOG_ERR("Read timeout");
				oneshot_pending = false;
				return false;
			}
		}
		oneshot_pending = false;
	} else {
		int err = ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, AK09940_ST1, frame, sizeof(frame));
		if (err) {
			LOG_ERR("Communication error");
			return false;
		}
		if (!(frame[0] & AK09940_ST1_DRDY))
			return false;
	}

	uint8_t st2 = frame[11];
	if (st2 & (AK09940_ST2_INV | AK09940_ST2_DOR))
		return false;
	for (int i = 0; i < 3; i++) {
		uint32_t raw = (uint32_t)frame[1 + i * 3]
			| (uint32_t)frame[2 + i * 3] << 8
			| (uint32_t)frame[3 + i * 3] << 16;
		if (raw == 0x1ffff)
			return false;
	}
	ak_mag_process(&frame[1], m);
	return true;
}

float ak_temp_read(float bias[3])
{
	(void)bias;
	uint8_t rawTemp;
	int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, AK09940_TMPS, &rawTemp);
	if (err)
	{
		LOG_ERR("Communication error");
		return NAN;
	}
	// Temperature [˚C] = 30 – (TMPS) / 1.7
	// TMPS is a signed 8-bit two's-complement value; byte order does not apply.
	float temp = (int8_t)rawTemp;
	temp /= -1.7f;
	temp += 30;
	return temp;
}

void ak_mag_process(uint8_t *raw_m, float m[3])
{
	for (int i = 0; i < 3; i++) {
		uint32_t raw = (uint32_t)raw_m[i * 3]
			| (uint32_t)raw_m[i * 3 + 1] << 8
			| (uint32_t)raw_m[i * 3 + 2] << 16;
		raw &= 0x3ffff;
		int32_t signed_raw = (raw & (1U << 17)) ? (int32_t)raw - (1 << 18) : (int32_t)raw;
		m[i] = signed_raw * sensitivity / 100000.0f;
	}
}

const sensor_mag_t sensor_mag_ak09940 = {
	*ak_init,
	*ak_shutdown,

	*ak_update_odr,

	*ak_mag_oneshot,
	*ak_mag_read,
	*ak_temp_read,

	*ak_mag_process,
	12, 12
};
