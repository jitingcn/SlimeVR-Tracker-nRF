#include <math.h>

#include <zephyr/logging/log.h>

#include "IST8308.h"
#include "IST8306.h" // Common functions
#include "sensor/sensor_none.h"

static const float sensitivity = 0.075; // uT/LSB

static uint8_t last_odr = 0xff;
static int64_t oneshot_trigger_time = 0;
static bool oneshot_pending;
static bool oneshot_failed;

LOG_MODULE_REGISTER(IST8308, LOG_LEVEL_DBG);

int ist8308_init(float time, float *actual_time)
{
	last_odr = 0xff; // reset last odr
	oneshot_pending = false;
	oneshot_failed = false;
//	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, IST8306_ACTR, 0x00); // exit suspend
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, IST8308_CNTL4, DR_200); // set DR
	if (err) {
		LOG_ERR("Communication error");
		return (err < 0 ? err : 0);
	}
	err = ist8308_update_odr(time, actual_time);
	return (err < 0 ? err : 0);
}

void ist8308_shutdown(void)
{
	last_odr = 0xff; // reset last odr
	oneshot_pending = false;
	oneshot_failed = false;
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, IST8306_CNTL3, 0x01); // soft reset
//	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, IST8306_ACTR, 0x02); // suspend
	if (err)
		LOG_ERR("Communication error");
}

int ist8308_update_odr(float time, float *actual_time)
{
	int ODR;
	uint8_t NSF;
	uint8_t MODE;
	uint8_t OSR;

	if (time <= 0 || time == INFINITY) // standby mode or single measurement mode
	{
		NSF = NSF_Low; // High Speed
		OSR = OSR_8;
		MODE = MODE_STANDBY;
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
	{
		NSF = NSF_Low; // High Speed
		OSR = OSR_8;
		MODE = MODE_CMM_200Hz;
		time = 1.0 / 200;
	}
	else if (ODR > 50)
	{
		NSF = NSF_Low; // Normal
		OSR = OSR_16;
		MODE = MODE_CMM_100Hz;
		time = 1.0 / 100;
	}
	else if (ODR > 20)
	{
		NSF = NSF_Low; // Normal
		OSR = OSR_16;
		MODE = MODE_CMM_50Hz;
		time = 1.0 / 50;
	}
	else if (ODR > 10)
	{
		NSF = NSF_Low; // Normal
		OSR = OSR_16;
		MODE = MODE_CMM_20Hz;
		time = 1.0 / 20;
	}
	else if (ODR > 0)
	{
		NSF = NSF_Medium; // Low noise
		OSR = OSR_32;
		MODE = MODE_CMM_10Hz;
		time = 1.0 / 10;
	}
	else
	{
		NSF = NSF_Low; // High Speed
		OSR = OSR_8;
		MODE = MODE_SINGLE;
		time = INFINITY;
	}

	uint8_t desired = MODE;
	if (last_odr == desired) {
		*actual_time = time;
		return 0; /* already configured */
	}

	if (MODE == MODE_SINGLE)
		MODE = MODE_STANDBY; // set STBY, oneshot will set SMM

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, IST8306_CNTL1, NSF << 5);
	if (err)
		goto error;
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, IST8306_CNTL2, MODE);
	if (err)
		goto error;
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, IST8306_OSRCNTL, OSR);
	if (err)
		goto error;

	last_odr = desired;
	oneshot_pending = false;
	oneshot_failed = false;
	*actual_time = time;
	return 0;
error:
	last_odr = 0xff;
	LOG_ERR("Communication error");
	return err;
}

void ist8308_mag_oneshot(void)
{
	last_odr = 0xff;
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, IST8306_CNTL2, MODE_SINGLE); // set single measurement mode
	oneshot_trigger_time = k_uptime_get();
	oneshot_pending = true;
	oneshot_failed = err != 0;
	if (err)
		LOG_ERR("Communication error");
}

bool ist8308_mag_read(float m[3])
{
	if (oneshot_pending && oneshot_failed) {
		oneshot_pending = false;
		oneshot_failed = false;
		return false;
	}

	uint8_t frame[7];
	if (oneshot_pending) {
		int64_t timeout = oneshot_trigger_time + 5;
		while (true) {
			int err = ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, IST8306_STAT, frame, sizeof(frame));
			if (err) {
				LOG_ERR("Communication error");
				oneshot_pending = false;
				return false;
			}
			if (frame[0] & 0x01)
				break;
			if (k_uptime_get() >= timeout) {
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
	ist8308_mag_process(&frame[1], m);
	return true;
}

void ist8308_mag_process(uint8_t *raw_m, float m[3])
{
	for (int i = 0; i < 3; i++) // x, y, z
	{
		m[i] = (int16_t)((((uint16_t)raw_m[(i * 2) + 1]) << 8) | raw_m[i * 2]);
		m[i] *= sensitivity; //LSB to uT
		m[i] /= 100; // uT to gauss
	}
}

const sensor_mag_t sensor_mag_ist8308 = {
	*ist8308_init,
	*ist8308_shutdown,

	*ist8308_update_odr,

	*ist8308_mag_oneshot,
	*ist8308_mag_read,
	*mag_none_temp_read,

	*ist8308_mag_process,
	7, 7
};
