#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "LIS2MDL.h"
#include "LIS3MDL.h" // Common functions

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

int lis2_init(float time, float *actual_time)
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

	err = lis2_update_odr(time, actual_time);
	return (err < 0 ? err : 0);
}

void lis2_shutdown(void)
{
	last_cfg_a = 0xff;
	int err = lis2_soft_reset();
	if (err)
		LOG_ERR("Communication error");
}

int lis2_update_odr(float time, float *actual_time)
{
	int ODR;
	uint8_t MODR;
	uint8_t MD;

	if (time <= 0) // off
	{
		MD = MD_IDLE;
		ODR = 0;
	}
	else if (time == INFINITY) // oneshot/single — keep continuous + fixed ODR
	{
		MD = MD_CONTINUOUS;
		ODR = 0;
	}
	else
	{
		MD = MD_CONTINUOUS;
		ODR = 1 / time;
	}

	if (MD == MD_IDLE)
	{
		MODR = 0;
		time = 0; // off
	}
	else if (ODR > 50) // TODO: this sucks
	{
		MODR = ODR_100Hz;
		time = 1.0 / 100;
	}
	else if (ODR > 20)
	{
		MODR = ODR_50Hz;
		time = 1.0 / 50;
	}
	else if (ODR > 10)
	{
		MODR = ODR_20Hz;
		time = 1.0 / 20;
	}
	else if (ODR > 0)
	{
		MODR = ODR_10Hz;
		time = 1.0 / 10;
	}
	else
	{
		/* INFINITY path: continuous at lowest fixed ODR */
		MODR = ODR_10Hz;
		time = 1.0 / 10;
	}

	uint8_t cfg_a;
	if (MD == MD_IDLE)
		cfg_a = MD_IDLE;
	else
		cfg_a = CFG_A_COMP_TEMP_EN | (MODR << 2) | MD_CONTINUOUS;

	if (last_cfg_a == cfg_a) {
		*actual_time = time;
		return 0; /* already configured */
	}

	bool was_idle = (last_cfg_a == 0xff) || ((last_cfg_a & 0x03) == MD_IDLE);
	int err;

	if (MD == MD_CONTINUOUS) {
		err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, LIS2MDL_CFG_REG_C, lis2_cfg_c());
		if (err)
			goto error;
	}

	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, LIS2MDL_CFG_REG_A, cfg_a);
	if (err)
		goto error;

	/* First sample after continuous enable needs turn-on delay. */
	if (MD == MD_CONTINUOUS && was_idle)
		k_msleep(LIS2MDL_TURN_ON_MS);

	last_cfg_a = cfg_a;
	*actual_time = time;
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
	// The output value is expressed as a signed 16-bit byte in two’s complement.
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
