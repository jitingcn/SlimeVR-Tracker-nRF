#include <math.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include "QMC5883L.h"
#include "sensor/sensor_none.h"

// QMC5883L 3-Axis Magnetic Sensor
// I2C address 0x0D, Chip ID register 0x0D = 0xFF

#define QMC5883L_XOUT_L 0x00

#define QMC5883L_STATUS 0x06
#define QMC5883L_STATUS_DRDY 0x01
#define QMC5883L_STATUS_OVL  0x02
#define QMC5883L_STATUS_DOR  0x04

#define QMC5883L_TOUT_L 0x07

#define QMC5883L_CTRL_1  0x09
#define QMC5883L_CTRL_2  0x0A
#define QMC5883L_SET_RST 0x0B
#define QMC5883L_CHIP_ID 0x0D

#define QMC5883L_MODE_STANDBY    0x00
#define QMC5883L_MODE_CONTINUOUS 0x01

#define QMC5883L_ODR_10HZ  0x00
#define QMC5883L_ODR_50HZ  0x01
#define QMC5883L_ODR_100HZ 0x02
#define QMC5883L_ODR_200HZ 0x03

#define QMC5883L_ODR_10Hz  QMC5883L_ODR_10HZ
#define QMC5883L_ODR_50Hz  QMC5883L_ODR_50HZ
#define QMC5883L_ODR_100Hz QMC5883L_ODR_100HZ
#define QMC5883L_ODR_200Hz QMC5883L_ODR_200HZ

#define QMC5883L_RNG_2G 0x00
#define QMC5883L_RNG_8G 0x01

#define QMC5883L_OSR_512 0x00
#define QMC5883L_OSR_256 0x01
#define QMC5883L_OSR_128 0x02
#define QMC5883L_OSR_64  0x03

#define QMC5883L_SOFT_RST 0x80
#define QMC5883L_ROL_PNT  0x40
#define QMC5883L_INT_ENB  0x01

static const float sensitivity = 1.0f / 3000.0f; // 3000 LSB/G at ±8G range

static uint8_t last_odr = 0xff;
static bool lastOvfl = false;
static int64_t oneshot_trigger_time = 0;
static uint8_t last_rawData[6];
static bool last_rawData_valid = false;
static int64_t last_mag_time_ms;
static int32_t mag_period_ms = 50; // default 10Hz

LOG_MODULE_REGISTER(QMC5883L, LOG_LEVEL_INF);

int qmc5883l_init(float time, float *actual_time)
{
	last_odr = 0xff;
	lastOvfl = false;
	oneshot_trigger_time = 0;
	last_rawData_valid = false;
	last_mag_time_ms = 0;

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883L_SET_RST, 0x01);
	if (err)
		LOG_ERR("Communication error");

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883L_CTRL_2, QMC5883L_ROL_PNT | QMC5883L_INT_ENB);
	if (err)
		LOG_ERR("Communication error");

	err |= qmc5883l_update_odr(time, actual_time);
	return (err < 0 ? err : 0);
}

void qmc5883l_shutdown(void)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883L_CTRL_1, QMC5883L_MODE_STANDBY);
	if (err)
		LOG_ERR("Communication error");
}

int qmc5883l_update_odr(float time, float *actual_time)
{
	int ODR;
	uint8_t MODR;
	uint8_t MODE;

	if (time <= 0 || time == INFINITY)
	{
		MODE = QMC5883L_MODE_STANDBY;
		ODR = 0;
	}
	else
	{
		MODE = QMC5883L_MODE_CONTINUOUS;
		ODR = 1 / time;
	}

	if (MODE == QMC5883L_MODE_STANDBY)
	{
		MODR = QMC5883L_ODR_200Hz;
		time = INFINITY;
	}
	else if (ODR > 100)
	{
		MODR = QMC5883L_ODR_200Hz;
		time = 1.0f / 200;
	}
	else if (ODR > 50)
	{
		MODR = QMC5883L_ODR_100Hz;
		time = 1.0f / 100;
	}
	else if (ODR > 10)
	{
		MODR = QMC5883L_ODR_50Hz;
		time = 1.0f / 50;
	}
	else
	{
		MODR = QMC5883L_ODR_10Hz;
		time = 1.0f / 10;
	}

	uint8_t STAT = (QMC5883L_OSR_512 << 6) | (QMC5883L_RNG_8G << 4) | (MODR << 2) | MODE;
	if (last_odr == STAT) {
		*actual_time = time;
		return 0; /* already configured — success for err|= callers */
	}

	int err;
	if (MODE == QMC5883L_MODE_STANDBY)
		err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883L_CTRL_1, QMC5883L_MODE_STANDBY);
	else
		err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883L_CTRL_1, STAT);
	if (err) {
		LOG_ERR("Communication error");
		return err;
	}

	last_odr = STAT;
	oneshot_trigger_time = 0;

	if (MODE != QMC5883L_MODE_STANDBY)
		mag_period_ms = (int32_t)(time * 1000);

	*actual_time = time;
	return 0;
}

void qmc5883l_mag_oneshot(void)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883L_CTRL_1, (QMC5883L_OSR_512 << 6) | (QMC5883L_RNG_8G << 4) | (QMC5883L_ODR_200Hz << 2) | QMC5883L_MODE_CONTINUOUS);
	oneshot_trigger_time = k_uptime_get();
	if (err)
		LOG_ERR("Communication error");
}

bool qmc5883l_mag_read(float m[3])
{
	int err = 0;
	if (oneshot_trigger_time)
	{
		uint8_t status = 0;
		int64_t timeout = oneshot_trigger_time + 10;
		while ((status & QMC5883L_STATUS_DRDY) == 0)
		{
			err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883L_STATUS, &status);
			if (k_uptime_get() > timeout)
			{
				LOG_WRN("Data ready status timeout!");
				break;
			}
		}
		oneshot_trigger_time = 0;
		if (err)
		{
			LOG_ERR("Communication error");
			return false;
		}
		if ((status & QMC5883L_STATUS_OVL) && !lastOvfl)
		{
			LOG_INF("Magnetometer overflow");
			lastOvfl = true;
		}
		else
		{
			lastOvfl = false;
		}
	}
	uint8_t rawData[6];
	err |= ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, QMC5883L_XOUT_L, rawData, 6);
	if (err)
		LOG_ERR("Communication error");

	int64_t now = k_uptime_get();
	if (last_rawData_valid && memcmp(rawData, last_rawData, 6) == 0)
	{
		if ((now - last_mag_time_ms) < (mag_period_ms + mag_period_ms / 10))
			return false;
		last_mag_time_ms += mag_period_ms;
		if (now - last_mag_time_ms > mag_period_ms * 2)
			last_mag_time_ms = now;
	}
	else
	{
		last_mag_time_ms = now;
	}
	memcpy(last_rawData, rawData, 6);
	last_rawData_valid = true;
	qmc5883l_mag_process(rawData, m);
	return true;
}

void qmc5883l_mag_process(uint8_t *raw_m, float m[3])
{
	for (int i = 0; i < 3; i++)
	{
		m[i] = ((int16_t *)raw_m)[i];
		m[i] *= sensitivity;
	}
}

const sensor_mag_t sensor_mag_qmc5883l = {
	qmc5883l_init,
	qmc5883l_shutdown,

	qmc5883l_update_odr,

	qmc5883l_mag_oneshot,
	qmc5883l_mag_read,
	mag_none_temp_read,

	qmc5883l_mag_process,
	6, 6
};
