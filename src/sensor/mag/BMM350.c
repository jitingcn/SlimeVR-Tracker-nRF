/*
Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.

BSD-3-Clause

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#include <errno.h>
#include <math.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "BMM350.h"

#define BMM350_OTP_WORDS 32
#define BMM350_XY_SENSITIVITY 0.007069979f
#define BMM350_Z_SENSITIVITY 0.007174964f
#define BMM350_TEMP_SENSITIVITY 0.000981282f

struct bmm350_compensation {
	float temp_offset;
	float temp_sensitivity;
	float offset[3];
	float sensitivity[3];
	float tco[3];
	float tcs[3];
	float t0;
	float cross_xy;
	float cross_yx;
	float cross_zx;
	float cross_zy;
};

static struct bmm350_compensation compensation;
static bool compensation_valid;
static float last_temperature_c = 25.0f;
static uint16_t last_state = UINT16_MAX;
static bool oneshot_pending;
static bool oneshot_failed;

LOG_MODULE_REGISTER(BMM350, LOG_LEVEL_DBG);

static int32_t bmm3_sign_extend(uint32_t value, uint8_t bits)
{
	uint32_t sign = 1U << (bits - 1U);
	return (int32_t)((value ^ sign) - sign);
}

static int32_t decode_s24(const uint8_t raw[3])
{
	uint32_t value = (uint32_t)raw[0] | (uint32_t)raw[1] << 8 | (uint32_t)raw[2] << 16;
	return bmm3_sign_extend(value, 24);
}

static int bmm3_read(uint8_t reg, uint8_t *data, uint32_t length)
{
	return ssi_burst_read_dummy(SENSOR_INTERFACE_DEV_MAG, reg, 2, data, length);
}

static int bmm3_wait_pmu(uint8_t command, int32_t timeout_ms)
{
	int64_t deadline = k_uptime_get() + timeout_ms;
	while (true) {
		uint8_t status = 0;
		int err = bmm3_read(BMM350_PMU_CMD_STATUS_0, &status, 1);
		if (err) {
			return err;
		}
		if (status & 0x10) {
			return -EIO;
		}
		if (!(status & 0x01)) {
			return ((status >> 5) & 0x07) == command ? 0 : -EIO;
		}
		if (k_uptime_get() >= deadline) {
			return -ETIMEDOUT;
		}
		k_msleep(1);
	}
}

static int bmm3_set_power_mode(uint8_t mode)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, BMM350_PMU_CMD, mode);
	if (err) {
		return err;
	}
	return bmm3_wait_pmu(mode, mode == PMU_CMD_NM ? 40 : 20);
}

static int bmm3_read_otp_word(uint8_t index, uint16_t *word)
{
	int err
		= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, BMM350_OTP_CMD_REG, BMM350_OTP_CMD_DIR_READ | (index & 0x1f));
	if (err) {
		return err;
	}

	bool ready = false;
	for (int attempt = 0; attempt < 3; attempt++) {
		k_usleep(300);
		uint8_t status = 0;
		err = bmm3_read(BMM350_OTP_STATUS_REG, &status, 1);
		if (err) {
			return err;
		}
		if (status & BMM350_OTP_STATUS_ERROR) {
			return -EIO;
		}
		if (status & BMM350_OTP_STATUS_CMD_DONE) {
			ready = true;
			break;
		}
	}
	if (!ready) {
		return -ETIMEDOUT;
	}

	uint8_t data[2];
	err = bmm3_read(BMM350_OTP_DATA_MSB_REG, data, sizeof(data));
	if (err) {
		return err;
	}
	*word = ((uint16_t)data[0] << 8) | data[1];
	return 0;
}

static void bmm3_parse_compensation(const uint16_t otp[BMM350_OTP_WORDS], struct bmm350_compensation *out)
{
	uint16_t offset_x = otp[0x0e] & 0x0fff;
	uint16_t offset_y = ((otp[0x0e] & 0xf000) >> 4) | (otp[0x0f] & 0x00ff);
	uint16_t offset_z = (otp[0x0f] & 0x0f00) | (otp[0x10] & 0x00ff);
	out->offset[0] = (float)bmm3_sign_extend(offset_x, 12);
	out->offset[1] = (float)bmm3_sign_extend(offset_y, 12);
	out->offset[2] = (float)bmm3_sign_extend(offset_z, 12);
	out->temp_offset = (float)bmm3_sign_extend(otp[0x0d] & 0xff, 8) / 5.0f;

	out->sensitivity[0] = (float)bmm3_sign_extend((otp[0x10] >> 8) & 0xff, 8) / 256.0f;
	out->sensitivity[1] = (float)bmm3_sign_extend(otp[0x11] & 0xff, 8) / 256.0f + 0.01f;
	out->sensitivity[2] = (float)bmm3_sign_extend((otp[0x11] >> 8) & 0xff, 8) / 256.0f;
	out->temp_sensitivity = (float)bmm3_sign_extend((otp[0x0d] >> 8) & 0xff, 8) / 512.0f;

	for (int axis = 0; axis < 3; axis++) {
		uint16_t word = otp[0x12 + axis];
		out->tco[axis] = (float)bmm3_sign_extend(word & 0xff, 8) / 32.0f;
		out->tcs[axis] = (float)bmm3_sign_extend((word >> 8) & 0xff, 8) / 16384.0f;
	}
	out->tcs[2] -= 0.0001f;
	out->t0 = (float)bmm3_sign_extend(otp[0x18], 16) / 512.0f + 23.0f;

	out->cross_xy = (float)bmm3_sign_extend(otp[0x15] & 0xff, 8) / 800.0f;
	out->cross_yx = (float)bmm3_sign_extend((otp[0x15] >> 8) & 0xff, 8) / 800.0f;
	out->cross_zx = (float)bmm3_sign_extend(otp[0x16] & 0xff, 8) / 800.0f;
	out->cross_zy = (float)bmm3_sign_extend((otp[0x16] >> 8) & 0xff, 8) / 800.0f;
}

static float bmm3_compensate_temperature(int32_t raw_temperature)
{
	float temperature = (float)raw_temperature * BMM350_TEMP_SENSITIVITY - 25.49f;
	return (1.0f + compensation.temp_sensitivity) * temperature + compensation.temp_offset;
}

static bool bmm3_compensate_mag(const uint8_t raw[9], float temperature, float m[3])
{
	float value[3] = {
		(float)decode_s24(&raw[0]) * BMM350_XY_SENSITIVITY,
		(float)decode_s24(&raw[3]) * BMM350_XY_SENSITIVITY,
		(float)decode_s24(&raw[6]) * BMM350_Z_SENSITIVITY,
	};
	float delta_t = temperature - compensation.t0;
	for (int axis = 0; axis < 3; axis++) {
		float denominator = 1.0f + compensation.tcs[axis] * delta_t;
		if (fabsf(denominator) < 1.0e-6f) {
			return false;
		}
		value[axis] = ((1.0f + compensation.sensitivity[axis]) * value[axis] + compensation.offset[axis]
					   + compensation.tco[axis] * delta_t)
					/ denominator;
	}

	float cross_denominator = 1.0f - compensation.cross_yx * compensation.cross_xy;
	if (fabsf(cross_denominator) < 1.0e-6f) {
		return false;
	}
	float x = (value[0] - compensation.cross_xy * value[1]) / cross_denominator;
	float y = (value[1] - compensation.cross_yx * value[0]) / cross_denominator;
	float z = value[2]
			+ (value[0] * (compensation.cross_yx * compensation.cross_zy - compensation.cross_zx)
			   - value[1] * (compensation.cross_zy - compensation.cross_xy * compensation.cross_zx))
				  / cross_denominator;
	m[0] = x / 100.0f;
	m[1] = y / 100.0f;
	m[2] = z / 100.0f;
	return true;
}

static int bmm3_magnetic_reset(void)
{
	int err = bmm3_set_power_mode(PMU_CMD_SUS);
	if (err) {
		return err;
	}
	err = bmm3_set_power_mode(PMU_CMD_BR);
	if (err) {
		return err;
	}
	return bmm3_set_power_mode(PMU_CMD_FGR);
}

int bmm3_init(float time, float *actual_time)
{
	last_state = UINT16_MAX;
	oneshot_pending = false;
	oneshot_failed = false;
	compensation_valid = false;
	last_temperature_c = 25.0f;

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, BMM350_CMD, BMM350_CMD_SOFTRESET);
	if (err) {
		goto error;
	}
	k_msleep(24);

	uint8_t chip_id = 0;
	err = bmm3_read(BMM350_CHIP_ID_REG, &chip_id, 1);
	if (err) {
		goto error;
	}
	if (chip_id != BMM350_CHIP_ID) {
		err = -ENODEV;
		goto error;
	}

	uint16_t otp[BMM350_OTP_WORDS];
	for (uint8_t index = 0; index < BMM350_OTP_WORDS; index++) {
		err = bmm3_read_otp_word(index, &otp[index]);
		if (err) {
			goto error;
		}
	}
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, BMM350_OTP_CMD_REG, BMM350_OTP_CMD_PWR_OFF);
	if (err) {
		goto error;
	}

	err = bmm3_magnetic_reset();
	if (err) {
		goto error;
	}
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, BMM350_INT_CTRL, 0x86);
	if (err) {
		goto error;
	}
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, BMM350_PMU_CMD_AXIS_EN, BMM350_AXIS_EN_XYZ);
	if (err) {
		goto error;
	}

	bmm3_parse_compensation(otp, &compensation);
	compensation_valid = true;
	err = bmm3_update_odr(time, actual_time);
	if (err) {
		goto error;
	}
	return 0;

error:
	last_state = UINT16_MAX;
	compensation_valid = false;
	LOG_ERR("Initialization failed: %d", err);
	return err;
}

void bmm3_shutdown(void)
{
	last_state = UINT16_MAX;
	oneshot_pending = false;
	oneshot_failed = false;
	compensation_valid = false;
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, BMM350_OTP_CMD_REG, BMM350_OTP_CMD_PWR_OFF);
	if (!err) {
		err = bmm3_set_power_mode(PMU_CMD_SUS);
	}
	if (err) {
		LOG_ERR("Communication error");
	}
}

int bmm3_update_odr(float time, float *actual_time)
{
	float requested_hz = 0.0f;
	uint8_t target_mode = PMU_CMD_SUS;
	uint8_t odr = AGGR_ODR_200Hz;
	uint8_t averaging = AGGR_NO_AVG;
	bool configure_aggregation = time == INFINITY || time > 0.0f;

	if (time > 0.0f && time != INFINITY) {
		target_mode = PMU_CMD_NM;
		requested_hz = 1.0f / time;
	}
	if (time <= 0.0f) {
		time = 0.0f;
		configure_aggregation = false;
	} else if (time == INFINITY) {
		time = INFINITY;
	} else if (requested_hz > 200.0f) {
		odr = AGGR_ODR_400Hz;
		averaging = AGGR_NO_AVG;
		time = 1.0f / 400.0f;
	} else if (requested_hz > 100.0f) {
		odr = AGGR_ODR_200Hz;
		averaging = AGGR_AVG_2;
		time = 1.0f / 200.0f;
	} else if (requested_hz > 50.0f) {
		odr = AGGR_ODR_100Hz;
		averaging = AGGR_AVG_4;
		time = 1.0f / 100.0f;
	} else if (requested_hz > 25.0f) {
		odr = AGGR_ODR_50Hz;
		averaging = AGGR_AVG_8;
		time = 1.0f / 50.0f;
	} else if (requested_hz > 12.5f) {
		odr = AGGR_ODR_25Hz;
		averaging = AGGR_AVG_8;
		time = 1.0f / 25.0f;
	} else if (requested_hz > 6.25f) {
		odr = AGGR_ODR_12_5Hz;
		averaging = AGGR_AVG_8;
		time = 1.0f / 12.5f;
	} else if (requested_hz > 3.125f) {
		odr = AGGR_ODR_6_25Hz;
		averaging = AGGR_AVG_8;
		time = 1.0f / 6.25f;
	} else if (requested_hz > 1.5625f) {
		odr = AGGR_ODR_3_125Hz;
		averaging = AGGR_AVG_8;
		time = 1.0f / 3.125f;
	} else {
		odr = AGGR_ODR_1_5625Hz;
		averaging = AGGR_AVG_8;
		time = 1.0f / 1.5625f;
	}

	uint8_t aggregation = (averaging << 4) | odr;
	uint16_t state = ((uint16_t)target_mode << 8) | (configure_aggregation ? aggregation : 0xff);
	if (last_state == state) {
		*actual_time = time;
		return 0;
	}

	int err;
	if (configure_aggregation) {
		err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, BMM350_PMU_CMD_AGGR_SET, aggregation);
		if (err) {
			goto error;
		}
		err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, BMM350_PMU_CMD, PMU_CMD_UPD_OAE);
		if (err) {
			goto error;
		}
		err = bmm3_wait_pmu(PMU_CMD_UPD_OAE, 20);
		if (err) {
			goto error;
		}
	}

	err = bmm3_set_power_mode(PMU_CMD_SUS);
	if (err) {
		goto error;
	}
	if (target_mode == PMU_CMD_NM) {
		err = bmm3_set_power_mode(PMU_CMD_NM);
		if (err) {
			goto error;
		}
	}

	last_state = state;
	oneshot_pending = false;
	oneshot_failed = false;
	*actual_time = time;
	return 0;

error:
	last_state = UINT16_MAX;
	LOG_ERR("Communication error");
	return err;
}

void bmm3_mag_oneshot(void)
{
	last_state = UINT16_MAX;
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, BMM350_PMU_CMD, PMU_CMD_FM_FAST);
	oneshot_pending = true;
	oneshot_failed = err != 0;
	if (err) {
		LOG_ERR("Communication error");
	}
}

bool bmm3_mag_read(float m[3])
{
	if (!compensation_valid) {
		return false;
	}
	if (oneshot_pending && oneshot_failed) {
		oneshot_pending = false;
		oneshot_failed = false;
		return false;
	}

	int64_t deadline = k_uptime_get() + 20;
	uint8_t frame[13];
	while (true) {
		int err = bmm3_read(BMM350_INT_STATUS, frame, sizeof(frame));
		if (err) {
			LOG_ERR("Communication error");
			oneshot_pending = false;
			return false;
		}
		if (frame[0] & BMM350_DRDY_DATA_REG) {
			break;
		}
		if (!oneshot_pending) {
			return false;
		}
		if (k_uptime_get() >= deadline) {
			LOG_ERR("Read timeout");
			oneshot_pending = false;
			return false;
		}
		k_usleep(100);
	}
	oneshot_pending = false;

	float temperature = bmm3_compensate_temperature(decode_s24(&frame[10]));
	float next_m[3];
	if (!bmm3_compensate_mag(&frame[1], temperature, next_m)) {
		return false;
	}
	last_temperature_c = temperature;
	m[0] = next_m[0];
	m[1] = next_m[1];
	m[2] = next_m[2];
	return true;
}

float bmm3_temp_read(float bias[3])
{
	(void)bias;
	if (!compensation_valid) {
		return NAN;
	}
	uint8_t raw[3];
	int err = bmm3_read(BMM350_TEMP_XLSB, raw, sizeof(raw));
	if (err) {
		LOG_ERR("Communication error");
		return NAN;
	}
	last_temperature_c = bmm3_compensate_temperature(decode_s24(raw));
	return last_temperature_c;
}

void bmm3_mag_process(uint8_t *raw_m, float m[3])
{
	if (!compensation_valid || !bmm3_compensate_mag(raw_m, last_temperature_c, m)) {
		m[0] = 0.0f;
		m[1] = 0.0f;
		m[2] = 0.0f;
	}
}

const sensor_mag_t sensor_mag_bmm350
	= {bmm3_init,
	   bmm3_shutdown,
	   bmm3_update_odr,
	   bmm3_mag_oneshot,
	   bmm3_mag_read,
	   bmm3_temp_read,
	   bmm3_mag_process,
	   3,
	   15};
