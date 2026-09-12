#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "LIS2MDL.h"

static const float sensitivity = 1.5 / 1000; // ~1.5 mgauss/LSB -> 0.0015 G/LSB

/* High-resolution continuous turn-on ~9.4 ms (AN5069). */
#define LIS2MDL_TURN_ON_MS 10

static uint8_t last_cfg_a = 0xff;

LOG_MODULE_REGISTER(LIS2MDL, LOG_LEVEL_DBG);

// CFG_REG_C: BDU always set; 4WSPI + I2C_DIS only in SPI mode
// (I2C mode must not set I2C_DIS or it cuts off the bus)
static uint8_t lis2_cfg_c(void)
{
	uint8_t cfg_c = CFG_C_BDU;
	if (sensor_interface_get_spec(SENSOR_INTERFACE_DEV_MAG) == SENSOR_INTERFACE_SPEC_SPI)
		cfg_c |= CFG_C_4WSPI | CFG_C_I2C_DIS;
	return cfg_c;
}

static int lis2_soft_reset(void)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, LIS2MDL_CFG_REG_A, CFG_A_SOFT_RST);
	if (err)
		return err;

	/* Datasheet: SOFT_RST self-clears after ~5 µs. */
	k_busy_wait(10);

	for (int i = 0; i < 20; i++) {
		uint8_t cfg_a = 0;
		err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, LIS2MDL_CFG_REG_A, &cfg_a);
		if (err)
			return err;
		if (!(cfg_a & CFG_A_SOFT_RST))
			return 0;
		k_busy_wait(10);
	}

	LOG_WRN("SOFT_RST did not clear");
	return -1;
}

int lis2_init(float period_s, float *actual_period_s)
{
	last_cfg_a = 0xff;

	int err = lis2_soft_reset();
	if (err) {
		LOG_ERR("Soft reset failed");
		return err;
	}

	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, LIS2MDL_CFG_REG_C, lis2_cfg_c());
	if (err) {
		LOG_ERR("Communication error");
		return err;
	}

	// CFG_REG_B: LPF (ODR/4 bandwidth) + OFF_CANC (internal bias cancellation)
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, LIS2MDL_CFG_REG_B, CFG_B_LPF | CFG_B_OFF_CANC);
	if (err) {
		LOG_ERR("Communication error");
		return err;
	}

	err = lis2_update_odr(period_s, actual_period_s);
	return (err < 0 ? err : 0);
}

void lis2_shutdown(void)
{
	last_cfg_a = 0xff;
	int err = lis2_soft_reset();
	if (err)
		LOG_ERR("Communication error");
}

int lis2_update_odr(float period_s, float *actual_period_s)
{
	int requested_odr_hz; // Truncate fractional Hz before selecting a supported rate.
	uint8_t odr_code;
	uint8_t mode_code;

	if (period_s <= 0) // off
	{
		mode_code = MD_IDLE;
		requested_odr_hz = 0;
	} else if (period_s == INFINITY) // oneshot/single — keep continuous + fixed ODR
	{
		mode_code = MD_CONTINUOUS;
		requested_odr_hz = 0;
	} else {
		mode_code = MD_CONTINUOUS;
		requested_odr_hz = 1 / period_s;
	}

	if (mode_code == MD_IDLE) {
		odr_code = 0;
		period_s = 0; // off
	} else if (requested_odr_hz > 50) {
		odr_code = ODR_100Hz;
		period_s = 1.0 / 100;
	} else if (requested_odr_hz > 20) {
		odr_code = ODR_50Hz;
		period_s = 1.0 / 50;
	} else if (requested_odr_hz > 10) {
		odr_code = ODR_20Hz;
		period_s = 1.0 / 20;
	} else if (requested_odr_hz > 0) {
		odr_code = ODR_10Hz;
		period_s = 1.0 / 10;
	} else {
		/* INFINITY path: continuous at lowest fixed ODR */
		odr_code = ODR_10Hz;
		period_s = 1.0 / 10;
	}

	uint8_t cfg_a;
	if (mode_code == MD_IDLE) {
		cfg_a = MD_IDLE;
	} else
		cfg_a = CFG_A_COMP_TEMP_EN | (odr_code << 2) | MD_CONTINUOUS;

	if (last_cfg_a == cfg_a) {
		*actual_period_s = period_s;
		return 0; /* already configured */
	}

	bool was_idle = (last_cfg_a == 0xff) || ((last_cfg_a & 0x03) == MD_IDLE);
	int err;

	if (mode_code == MD_CONTINUOUS) {
		err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, LIS2MDL_CFG_REG_C, lis2_cfg_c());
		if (err)
			goto error;
	}

	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, LIS2MDL_CFG_REG_A, cfg_a);
	if (err)
		goto error;

	/* First sample after continuous enable needs turn-on delay. */
	if (mode_code == MD_CONTINUOUS && was_idle) {
		k_msleep(LIS2MDL_TURN_ON_MS);
	}

	last_cfg_a = cfg_a;
	*actual_period_s = period_s;
	return 0;
error:
	last_cfg_a = 0xff;
	LOG_ERR("Communication error");
	return err;
}

void lis2_mag_oneshot(void)
{
	/* Continuous driver: no single-shot trigger. Kept for sensor_mag_t ABI. */
}

bool lis2_mag_read(float m[3])
{
	uint8_t frame[7];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, LIS2MDL_STATUS_REG, frame, sizeof(frame));
	if (err) {
		LOG_ERR("Communication error");
		return false;
	}
	if (!(frame[0] & STATUS_ZYXDA))
		return false;
	lis2_mag_process(&frame[1], m);
	return true;
}

float lis2_temp_read(float bias[3])
{
	(void)bias;
	uint8_t rawTemp[2];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, LIS2MDL_TEMP_OUT_L_REG, &rawTemp[0], 2);
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

void lis2_mag_process(uint8_t *raw_m, float m[3])
{
	for (int i = 0; i < 3; i++) // x, y, z
	{
		m[i] = (int16_t)((((uint16_t)raw_m[(i * 2) + 1]) << 8) | raw_m[i * 2]);
		m[i] *= sensitivity;
	}
}

const sensor_mag_t sensor_mag_lis2mdl = {
	*lis2_init,
	*lis2_shutdown,

	*lis2_update_odr,

	*lis2_mag_oneshot,
	*lis2_mag_read,
	*lis2_temp_read,

	*lis2_mag_process,
	7, 7
};
