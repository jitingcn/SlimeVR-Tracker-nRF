#include <math.h>

#include <zephyr/logging/log.h>

#include "AK09940.h"

static const float sensitivity = 10; // nT/LSB

static uint8_t last_odr = 0xff;
//static uint8_t last_rawTemp = 0xff;
static int64_t oneshot_trigger_time = 0;
static bool oneshot_pending;
static bool oneshot_failed;

LOG_MODULE_REGISTER(AK09940, LOG_LEVEL_DBG);

int ak_init(float time, float *actual_time)
{
	last_odr = 0xff; // reset last odr
	oneshot_pending = false;
	oneshot_failed = false;
	int err = ak_update_odr(time, actual_time);
	return (err < 0 ? err : 0);
}

void ak_shutdown(void)
{
	last_odr = 0xff; // reset last odr
	oneshot_pending = false;
	oneshot_failed = false;
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, AK09940_CNTL4, 0x01);
	if (err)
		LOG_ERR("Communication error");
}

int ak_update_odr(float time, float *actual_time)
{
	int ODR;
	uint8_t MODE;

	if (time <= 0 || time == INFINITY) // power down mode or single measurement mode
	{
		MODE = MODE_PDM;
		ODR = 0;
	}
	else
	{
		ODR = 1 / time;
	}

	if (time <= 0)
	{
		time = 0; // off
	}
	else if (ODR > 100) // TODO: this sucks
	{ // only up to 200Hz supported with MT_LND2
		MODE = MODE_CMM5_200Hz;
		time = 1.0 / 200;
	}
	else if (ODR > 50)
	{
		MODE = MODE_CMM4_100Hz;
		time = 1.0 / 100;
	}
	else if (ODR > 20)
	{
		MODE = MODE_CMM3_50Hz;
		time = 1.0 / 50;
	}
	else if (ODR > 10)
	{
		MODE = MODE_CMM2_20Hz;
		time = 1.0 / 20;
	}
	else if (ODR > 0)
	{
		MODE = MODE_CMM1_10Hz;
		time = 1.0 / 10;
	}
	else
	{
		MODE = MODE_SMM;
//		time = INFINITY;
		time = 0; // unsure if SMM is working at the needed rate
	}

	uint8_t desired = MODE;
	if (last_odr == desired) {
		*actual_time = time;
		return 0; /* already configured */
	}

	if (MODE == MODE_SMM)
		MODE = MODE_PDM; // set PDM, oneshot will set SMM

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, AK09940_CNTL3, MT_LND2 << 5 | MODE);
	if (err) {
		last_odr = 0xff;
		LOG_ERR("Communication error");
		return err;
	}

	last_odr = desired;
	oneshot_pending = false;
	oneshot_failed = false;
	*actual_time = time;
	return 0;
}

void ak_mag_oneshot(void)
{
	last_odr = 0xff;
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, AK09940_CNTL3, MT_LND2 << 5 | MODE_SMM); // single measurement mode (does not change MT2)
	oneshot_trigger_time = k_uptime_get();
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
		int64_t timeout = oneshot_trigger_time + 2;
		while (true) {
			int err = ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, AK09940_ST1, frame, sizeof(frame));
			if (err) {
				LOG_ERR("Communication error");
				oneshot_pending = false;
				return false;
			}
			if (frame[0] & AK09940_ST1_DRDY)
				break;
			if (k_uptime_get() >= timeout) {
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
	// Measurement data is stored in two’s complement and Little Endian format.
	float temp = (int8_t)rawTemp;
	temp /= -1.7f;
	temp += 30;
	// TODO: see pg.24
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
