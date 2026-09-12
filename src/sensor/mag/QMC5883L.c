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

static uint8_t last_config_code = 0xff;
static bool last_overflow = false;
static int64_t oneshot_trigger_ms = 0;
static bool oneshot_pending;
static bool oneshot_failed;
static uint8_t last_raw_sample[6];
static bool last_raw_sample_valid = false;
static int64_t last_mag_time_ms;
static int32_t mag_period_ms = 50; // initial duplicate-suppression period until configured

LOG_MODULE_REGISTER(QMC5883L, LOG_LEVEL_INF);

int qmc5883l_init(float period_s, float *actual_period_s)
{
	last_config_code = 0xff;
	last_overflow = false;
	oneshot_trigger_ms = 0;
	oneshot_pending = false;
	oneshot_failed = false;
	last_raw_sample_valid = false;
	last_mag_time_ms = 0;

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883L_SET_RST, 0x01);
	if (err) {
		LOG_ERR("Communication error");
		return err;
	}

	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883L_CTRL_2, QMC5883L_ROL_PNT | QMC5883L_INT_ENB);
	if (err) {
		LOG_ERR("Communication error");
		return err;
	}

	return qmc5883l_update_odr(period_s, actual_period_s);
}

void qmc5883l_shutdown(void)
{
	last_config_code = 0xff;
	oneshot_trigger_ms = 0;
	oneshot_pending = false;
	oneshot_failed = false;
	last_raw_sample_valid = false;
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883L_CTRL_1, QMC5883L_MODE_STANDBY);
	if (err)
		LOG_ERR("Communication error");
}

int qmc5883l_update_odr(float period_s, float *actual_period_s)
{
	int requested_odr_hz; // Truncate fractional Hz before selecting a supported rate.
	uint8_t odr_code;
	uint8_t mode_code;

	if (period_s <= 0 || period_s == INFINITY) {
		mode_code = QMC5883L_MODE_STANDBY;
		requested_odr_hz = 0;
	} else {
		mode_code = QMC5883L_MODE_CONTINUOUS;
		requested_odr_hz = 1 / period_s;
	}

	if (mode_code == QMC5883L_MODE_STANDBY) {
		odr_code = QMC5883L_ODR_200Hz;
		period_s = INFINITY;
	} else if (requested_odr_hz > 100) {
		odr_code = QMC5883L_ODR_200Hz;
		period_s = 1.0f / 200;
	} else if (requested_odr_hz > 50) {
		odr_code = QMC5883L_ODR_100Hz;
		period_s = 1.0f / 100;
	} else if (requested_odr_hz > 10) {
		odr_code = QMC5883L_ODR_50Hz;
		period_s = 1.0f / 50;
	} else {
		odr_code = QMC5883L_ODR_10Hz;
		period_s = 1.0f / 10;
	}

	uint8_t config_code = (QMC5883L_OSR_512 << 6) | (QMC5883L_RNG_8G << 4) | (odr_code << 2) | mode_code;
	if (last_config_code == config_code) {
		*actual_period_s = period_s;
		return 0; /* already configured — success for err|= callers */
	}

	int err;
	if (mode_code == QMC5883L_MODE_STANDBY) {
		err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883L_CTRL_1, QMC5883L_MODE_STANDBY);
	} else
		err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883L_CTRL_1, config_code);
	if (err) {
		LOG_ERR("Communication error");
		last_config_code = 0xff;
		return err;
	}

	last_config_code = config_code;
	oneshot_trigger_ms = 0;
	oneshot_pending = false;
	oneshot_failed = false;
	last_raw_sample_valid = false;

	if (mode_code != QMC5883L_MODE_STANDBY) {
		mag_period_ms = (int32_t)(period_s * 1000);
	}

	*actual_period_s = period_s;
	return 0;
}

void qmc5883l_mag_oneshot(void)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883L_CTRL_1, (QMC5883L_OSR_512 << 6) | (QMC5883L_RNG_8G << 4) | (QMC5883L_ODR_200Hz << 2) | QMC5883L_MODE_CONTINUOUS);
	last_config_code = 0xff;
	oneshot_failed = err != 0;
	oneshot_pending = true;
	oneshot_trigger_ms = k_uptime_get();
	if (err)
		LOG_ERR("Communication error");
}

bool qmc5883l_mag_read(float m[3])
{
	bool was_oneshot = oneshot_pending;
	if (oneshot_pending)
	{
		if (oneshot_failed) {
			oneshot_pending = false;
			oneshot_failed = false;
			return false;
		}

		uint8_t status = 0;
		int64_t deadline_ms = oneshot_trigger_ms + 10;
		while ((status & QMC5883L_STATUS_DRDY) == 0)
		{
			int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883L_STATUS, &status);
			if (err) {
				LOG_ERR("Communication error");
				oneshot_pending = false;
				return false;
			}
			if (k_uptime_get() >= deadline_ms) {
				LOG_WRN("Data ready status timeout!");
				oneshot_pending = false;
				return false;
			}
		}
		oneshot_trigger_ms = 0;
		oneshot_pending = false;
		if (status & QMC5883L_STATUS_OVL) {
			if (!last_overflow) {
				LOG_INF("Magnetometer overflow");
			}
			last_overflow = true;
			return false;
		}
		last_overflow = false;
	}
	if (!was_oneshot && sensor_interface_get_spec(SENSOR_INTERFACE_DEV_MAG) != SENSOR_INTERFACE_SPEC_EXT) {
		uint8_t status = 0;
		int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883L_STATUS, &status);
		if (err) {
			LOG_ERR("Communication error");
			return false;
		}
		if (!(status & QMC5883L_STATUS_DRDY) || (status & QMC5883L_STATUS_OVL))
			return false;
	}
	uint8_t rawData[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, QMC5883L_XOUT_L, rawData, 6);
	if (err) {
		LOG_ERR("Communication error");
		return false;
	}

	int64_t now = k_uptime_get();
	if (last_raw_sample_valid && memcmp(rawData, last_raw_sample, 6) == 0) {
		if ((now - last_mag_time_ms) < (mag_period_ms + mag_period_ms / 10))
			return false;
		last_mag_time_ms += mag_period_ms;
		if (now - last_mag_time_ms > mag_period_ms * 2)
			last_mag_time_ms = now;
	} else {
		last_mag_time_ms = now;
	}
	memcpy(last_raw_sample, rawData, 6);
	last_raw_sample_valid = true;
	qmc5883l_mag_process(rawData, m);
	return true;
}

void qmc5883l_mag_process(uint8_t *raw_m, float m[3])
{
	for (int i = 0; i < 3; i++)
	{
		uint16_t raw = ((uint16_t)raw_m[i * 2 + 1] << 8) | raw_m[i * 2];
		m[i] = (int16_t)raw;
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
