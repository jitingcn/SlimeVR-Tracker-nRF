#include <math.h>

#include <zephyr/logging/log.h>

#include "LIS3MDL.h"

static const float sensitivity = 1.0 / 3421; // Always 8G (FS = ±8 gauss: 3421 LSB/Gauss)

static uint16_t last_state = 0xffff;
static bool oneshot_pending;
static bool oneshot_failed;
static int64_t oneshot_deadline_ms;

LOG_MODULE_REGISTER(LIS3MDL, LOG_LEVEL_DBG);

int lis3_init(float period_s, float *actual_period_s)
{
	last_state = 0xffff;
	oneshot_pending = false;
	oneshot_failed = false;

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, LIS3MDL_CTRL_REG1, 0x80); // enable temp sensor
	if (err) {
		goto error;
	}
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, LIS3MDL_CTRL_REG2, FS_8G << 5);
	if (err) {
		goto error;
	}
	err = lis3_update_odr(period_s, actual_period_s);
	return (err < 0 ? err : 0);

error:
	last_state = 0xffff;
	LOG_ERR("Communication error");
	return (err < 0 ? err : 0);
}

void lis3_shutdown(void)
{
	last_state = 0xffff;
	oneshot_pending = false;
	oneshot_failed = false;
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, LIS3MDL_CTRL_REG2, 0x04);
	if (err) {
		LOG_ERR("Communication error");
	}
}

int lis3_update_odr(float period_s, float *actual_period_s)
{
	int requested_odr_hz = 0; // Preserve integer-Hz truncation, including sub-Hz requests.
	uint8_t performance_code = OM_UHP;
	uint8_t odr_code = DO_0_625Hz;
	uint8_t fast_odr_enabled = 0;
	uint8_t mode_code;
	bool single = period_s == INFINITY;

	if (period_s <= 0) // off
	{
		mode_code = MD_POWER_DOWN;
	} else if (period_s == INFINITY) // oneshot/single
	{
		mode_code = MD_POWER_DOWN;
	} else {
		mode_code = MD_CONTINUOUS_CONV;
		requested_odr_hz = 1 / period_s;
	}

	if (mode_code == MD_POWER_DOWN) {
		period_s = single ? INFINITY : 0;
	} else if (requested_odr_hz > 560) {
		performance_code = OM_LP;
		fast_odr_enabled = 1;
		period_s = 1.0 / 1000;
	} else if (requested_odr_hz > 300) {
		performance_code = OM_MP;
		fast_odr_enabled = 1;
		period_s = 1.0 / 560;
	} else if (requested_odr_hz > 155) {
		performance_code = OM_HP;
		fast_odr_enabled = 1;
		period_s = 1.0 / 300;
	} else if (requested_odr_hz > 80) {
		performance_code = OM_UHP;
		fast_odr_enabled = 1;
		period_s = 1.0 / 155;
	} else if (requested_odr_hz > 40) {
		odr_code = DO_80Hz;
		period_s = 1.0 / 80;
	} else if (requested_odr_hz > 20) {
		odr_code = DO_40Hz;
		period_s = 1.0 / 40;
	} else if (requested_odr_hz > 10) {
		odr_code = DO_20Hz;
		period_s = 1.0 / 20;
	} else if (requested_odr_hz > 5) {
		odr_code = DO_10Hz;
		period_s = 1.0 / 10;
	} else if (requested_odr_hz > 2.5) {
		odr_code = DO_5Hz;
		period_s = 1.0 / 5;
	} else if (requested_odr_hz > 1.25) {
		odr_code = DO_2_5Hz;
		period_s = 1.0 / 2.5;
	} else if (requested_odr_hz > 0.625) {
		odr_code = DO_1_25Hz;
		period_s = 1.0 / 1.25;
	} else if (requested_odr_hz > 0) {
		odr_code = DO_0_625Hz;
		period_s = 1.0 / 0.625;
	} else {
		odr_code = 0;
		period_s = INFINITY;
	}

	uint8_t ctrl = performance_code << 5 | odr_code << 2 | fast_odr_enabled << 1;
	uint16_t state = ((uint16_t)mode_code << 8) | ctrl;
	if (last_state == state) {
		*actual_period_s = period_s;
		return 0; /* already configured */
	}

	int err = ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_MAG,
		LIS3MDL_CTRL_REG1,
		0x80 | ctrl
	); // temp, X/Y operating mode, and ODR
	if (err) {
		goto error;
	}
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, LIS3MDL_CTRL_REG3, mode_code); // set measurement mode
	if (err) {
		goto error;
	}
	err = ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_MAG,
		LIS3MDL_CTRL_REG4,
		performance_code << 2
	); // set Z-axis operating mode
	if (err) {
		goto error;
	}
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, LIS3MDL_CTRL_REG5, LIS3MDL_CTRL_REG5_BDU);
	if (err) {
		goto error;
	}

	last_state = state;
	oneshot_pending = false;
	oneshot_failed = false;
	*actual_period_s = period_s;
	return 0;
error:
	last_state = 0xffff;
	LOG_ERR("Communication error");
	return err;
}

void lis3_mag_oneshot(void)
{
	// Request one conversion by writing the single-conversion mode.
	last_state = 0xffff;
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, LIS3MDL_CTRL_REG3, MD_SINGLE_CONV);
	oneshot_failed = err != 0;
	oneshot_pending = true;
	oneshot_deadline_ms = k_uptime_get() + 20;
	if (err) {
		LOG_ERR("Communication error");
	}
}

bool lis3_mag_read(float m[3])
{
	if (oneshot_pending) {
		if (oneshot_failed) {
			oneshot_pending = false;
			oneshot_failed = false;
			return false;
		}

		uint8_t mode = MD_SINGLE_CONV;
		while ((mode & 0x03) == MD_SINGLE_CONV) {
			int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, LIS3MDL_CTRL_REG3, &mode);
			if (err) {
				LOG_ERR("Communication error");
				oneshot_pending = false;
				return false;
			}
			if (k_uptime_get() >= oneshot_deadline_ms) {
				LOG_ERR("Read timeout");
				oneshot_pending = false;
				return false;
			}
		}
		oneshot_pending = false;
	} else if (last_state == 0xffff || (last_state >> 8) != MD_CONTINUOUS_CONV) {
		return false;
	}

	uint8_t frame[7];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, LIS3MDL_STATUS_REG, frame, sizeof(frame));
	if (err) {
		LOG_ERR("Communication error");
		return false;
	}
	if (!(frame[0] & LIS3MDL_STATUS_ZYXDA)) {
		return false;
	}
	lis3_mag_process(&frame[1], m);
	return true;
}

float lis3_temp_read(float bias[3])
{
	(void)bias;
	uint8_t rawTemp[2];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, LIS3MDL_TEMP_OUT_L, &rawTemp[0], 2);
	if (err) {
		LOG_ERR("Communication error");
		return NAN;
	}
	// The output value is a signed 16-bit word in two's complement.
	// The four most significant bits contain a copy of the sign bit.
	// The nominal sensitivity is 8 LSB/°C
	float temp = (int16_t)((((uint16_t)rawTemp[1]) << 8) | rawTemp[0]);
	return 25.0f + temp / 8.0f;
}

void lis3_mag_process(uint8_t *raw_m, float m[3])
{
	for (int i = 0; i < 3; i++) // x, y, z
	{
		m[i] = (int16_t)((((uint16_t)raw_m[(i * 2) + 1]) << 8) | raw_m[i * 2]);
		m[i] *= sensitivity;
	}
}

const sensor_mag_t sensor_mag_lis3mdl
	= {*lis3_init,
	   *lis3_shutdown,

	   *lis3_update_odr,

	   *lis3_mag_oneshot,
	   *lis3_mag_read,
	   *lis3_temp_read,

	   *lis3_mag_process,
	   7,
	   7};
