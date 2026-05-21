/* 01/14/2022 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Ladybug default), respectively,
  and it uses the Ladybug STM32L432 Breakout Board.
  The ICM42686 is a combo sensor with embedded accel and gyro,
  here used as 6 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/
#include <math.h>

#include <zephyr/logging/log.h>
#include <hal/nrf_gpio.h>

#include "ICM42686.h"
#include "sensor/sensor_none.h"

#define PACKET_SIZE 20

static const float accel_sensitivity = 16.0f / 32768.0f; // Always 16G
static const float gyro_sensitivity = 2000.0f / 32768.0f; // Always 2000dps

static const float accel_sensitivity_32 = 32.0f / ((uint32_t)2<<30); // 32G forced
static const float gyro_sensitivity_32 = 4000.0f / ((uint32_t)2<<30); // 4000dps forced

static const uint16_t times[] = {
	32000, 16000, 8000, 4000, 2000, 1000,
	500, 200, 100, 50, 25, 2, 0
};

static const uint8_t odrs[] = {
	ICM42686_AODR_32kHz,
	ICM42686_AODR_16kHz,
	ICM42686_AODR_8kHz,
	ICM42686_AODR_4kHz,
	ICM42686_AODR_2kHz,
	ICM42686_AODR_1kHz,
	ICM42686_AODR_500Hz,
	ICM42686_AODR_200Hz,
	ICM42686_AODR_100Hz,
	ICM42686_AODR_50Hz,
	ICM42686_AODR_25Hz,
	ICM42686_AODR_12_5Hz
};

static uint8_t last_accel_odr = 0xff;
static uint8_t last_gyro_odr = 0xff;
static const float clock_reference = 32000;
static float clock_scale = 1; // ODR is scaled by clock_rate/clock_reference

#define FIFO_MULT 0.00075f     // assuming i2c fast mode
#define FIFO_MULT_SPI 0.0001f  // ~24MHz

static float fifo_multiplier_factor = FIFO_MULT;
static float fifo_multiplier = 0;

LOG_MODULE_REGISTER(ICM42686, LOG_LEVEL_DBG);

int icm42686_init(float clock_rate, float accel_time, float gyro_time,
				  float *accel_actual_time, float *gyro_actual_time)
{
	// setup interface for SPI
	if (!sensor_interface_spi_configure(SENSOR_INTERFACE_DEV_IMU, MHZ(24), 0))
		fifo_multiplier_factor = FIFO_MULT_SPI; // SPI mode
	else
		fifo_multiplier_factor = FIFO_MULT; // I2C mode

	int err = 0;

	// Optional WHO_AM_I read for debug.
#ifdef ICM42686_WHO_AM_I
	uint8_t whoami = 0;
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, ICM42686_WHO_AM_I, &whoami);
	LOG_INF("ICM42686 WHO_AM_I = 0x%02X", whoami);
#endif

	// FIFO_COUNT and FIFO_WM use records
	err |= ssi_reg_update_byte(SENSOR_INTERFACE_DEV_IMU,
							   ICM42686_INTF_CONFIG0,
							   0x40,
							   0x40);

	// Datasheet DS-000348: INT_CONFIG1.INT_ASYNC_RESET must be 0 for proper INT1/INT2 behavior.
	err |= ssi_reg_update_byte(SENSOR_INTERFACE_DEV_IMU,
							   ICM42686_INT_CONFIG1,
							   0x10,
							   0x00);

	if (clock_rate > 0)
	{
		clock_scale = clock_rate / clock_reference;

		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
								  ICM42686_REG_BANK_SEL,
								  0x01); // select register bank 1

		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
								  ICM42686_INTF_CONFIG5,
								  0x04); // use CLKIN

		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
								  ICM42686_REG_BANK_SEL,
								  0x00); // select register bank 0

		err |= ssi_reg_update_byte(SENSOR_INTERFACE_DEV_IMU,
								   ICM42686_INTF_CONFIG1,
								   0x04,
								   0x04); // use CLKIN
	}

	last_accel_odr = 0xff;
	last_gyro_odr = 0xff;

	err |= icm42686_update_odr(accel_time, gyro_time,
							   accel_actual_time, gyro_actual_time);

	k_msleep(1);

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM42686_FIFO_CONFIG1,
							  0x13); // enable FIFO hires (A+G packet format matches parser)

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM42686_FIFO_CONFIG,
							  1 << 6); // begin FIFO stream

	// Verify external CLKIN is actually working by checking FIFO output
	if (clock_rate > 0)
	{
		k_msleep(10);

		uint8_t rawCount[2];
		ssi_burst_read(SENSOR_INTERFACE_DEV_IMU,
					   ICM42686_FIFO_COUNTH,
					   rawCount,
					   2);

		uint16_t fifo_count = (uint16_t)(rawCount[0] << 8 | rawCount[1]);

		if (fifo_count == 0)
		{
			LOG_WRN("External CLKIN not working, falling back to internal clock");

			clock_scale = 1;

			err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
									  ICM42686_REG_BANK_SEL,
									  0x01);

			err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
									  ICM42686_INTF_CONFIG5,
									  0x00);

			err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
									  ICM42686_REG_BANK_SEL,
									  0x00);

			err |= ssi_reg_update_byte(SENSOR_INTERFACE_DEV_IMU,
									   ICM42686_INTF_CONFIG1,
									   0x04,
									   0x00);

			last_accel_odr = 0xff;
			last_gyro_odr = 0xff;

			err |= icm42686_update_odr(accel_time, gyro_time,
									   accel_actual_time, gyro_actual_time);
		}
		else
		{
			LOG_INF("External CLKIN verified: FIFO count=%d", fifo_count);
		}
	}

	if (err)
		LOG_ERR("Communication error");

	return (err < 0 ? err : 0);
}

void icm42686_shutdown(void)
{
	last_accel_odr = 0xff;
	last_gyro_odr = 0xff;

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
								 ICM42686_DEVICE_CONFIG,
								 0x01); // soft reset

	if (err)
		LOG_ERR("Communication error");
}

void icm42686_update_fs(float accel_range, float gyro_range,
						float *accel_actual_range, float *gyro_actual_range)
{
	ARG_UNUSED(accel_range);
	ARG_UNUSED(gyro_range);

	*accel_actual_range = 32;   // always 32g in hires
	*gyro_actual_range = 4000;  // always 4000dps in hires
}

int icm42686_update_odr(float accel_time, float gyro_time,
						float *accel_actual_time, float *gyro_actual_time)
{
	int ODR;

	uint8_t Ascale = ICM42686_AFS_32G;
	uint8_t Gscale = ICM42686_GFS_4000DPS;

	uint8_t aMode;
	uint8_t gMode;

	uint8_t AODR = 0;
	uint8_t GODR = 0;

	// Calculate accel
	if (accel_time <= 0 || accel_time == INFINITY)
	{
		aMode = ICM42686_aMode_OFF;
		accel_time = 0;
	}
	else
	{
		aMode = ICM42686_aMode_LN;
		ODR = 1 / accel_time;
		ODR /= clock_scale;

		for (int i = 1; i < ARRAY_SIZE(times); i++)
		{
			if (ODR <= (i > 11 ? times[i] / 25.0 : times[i]))
				continue;

			AODR = odrs[i - 1];
			accel_time = i > 12 ? times[i - 1] / 25.0 : 1.0 / times[i - 1];
			break;
		}
	}

	accel_time /= clock_scale;

	// Calculate gyro
	if (gyro_time <= 0)
	{
		gMode = ICM42686_gMode_OFF;
		gyro_time = 0;
	}
	else if (gyro_time == INFINITY)
	{
		gMode = ICM42686_gMode_SBY;
		gyro_time = 0;
	}
	else
	{
		gMode = ICM42686_gMode_LN;
		ODR = 1 / gyro_time;
		ODR /= clock_scale;

		for (int i = 1; i < ARRAY_SIZE(times); i++)
		{
			if (ODR <= (i > 11 ? times[i] / 25.0 : times[i]))
				continue;

			GODR = odrs[i - 1];
			gyro_time = i > 12 ? times[i - 1] / 25.0 : 1.0 / times[i - 1];
			break;
		}
	}

	gyro_time /= clock_scale;

	if (last_accel_odr == AODR && last_gyro_odr == GODR)
		return 1;

	int err = 0;

	// only if the power mode has changed
	if (last_accel_odr == 0xff ||
		last_gyro_odr == 0xff ||
		(last_accel_odr == 0 ? 0 : 1) != (AODR == 0 ? 0 : 1) ||
		(last_gyro_odr == 0 ? 0 : 1) != (GODR == 0 ? 0 : 1))
	{
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
								  ICM42686_PWR_MGMT0,
								  gMode << 2 | aMode);

		k_busy_wait(250);
	}

	last_accel_odr = AODR;
	last_gyro_odr = GODR;

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM42686_ACCEL_CONFIG0,
							  Ascale << 5 | AODR);

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM42686_GYRO_CONFIG0,
							  Gscale << 5 | GODR);

	if (err)
		LOG_ERR("Communication error");

	*accel_actual_time = accel_time;
	*gyro_actual_time = gyro_time;

	// extra read packets by ODR time
	if (accel_time == 0 && gyro_time != 0)
		fifo_multiplier = fifo_multiplier_factor / gyro_time;
	else if (accel_time != 0 && gyro_time == 0)
		fifo_multiplier = fifo_multiplier_factor / accel_time;
	else if (gyro_time > accel_time)
		fifo_multiplier = fifo_multiplier_factor / accel_time;
	else if (accel_time > gyro_time)
		fifo_multiplier = fifo_multiplier_factor / gyro_time;
	else
		fifo_multiplier = 0;

	return 0;
}

uint16_t icm42686_fifo_read(uint8_t *data, uint16_t len)
{
	int err = 0;
	uint16_t total = 0;
	uint16_t packets = UINT16_MAX;

	while (packets > 0 && len >= PACKET_SIZE)
	{
		uint8_t rawCount[2];

		err |= ssi_burst_read(SENSOR_INTERFACE_DEV_IMU,
							  ICM42686_FIFO_COUNTH,
							  &rawCount[0],
							  2);

		packets = (uint16_t)(rawCount[0] << 8 | rawCount[1]);

		if (!packets)
			break;

		float extra_read_packets = packets * fifo_multiplier;
		packets += extra_read_packets;

		uint16_t count = packets * PACKET_SIZE;
		uint16_t limit = len / PACKET_SIZE;

		if (packets > limit)
		{
			LOG_WRN("FIFO read buffer limit reached, %d packets dropped",
					packets - limit);

			packets = limit;
			count = packets * PACKET_SIZE;
		}

		err |= ssi_burst_read_interval(SENSOR_INTERFACE_DEV_IMU,
									   ICM42686_FIFO_DATA,
									   data,
									   count,
									   PACKET_SIZE);

		if (err)
			LOG_ERR("Communication error");

		data += packets * PACKET_SIZE;
		len -= packets * PACKET_SIZE;
		total += packets;
	}

	return total;
}

static const uint8_t invalid[6] = {
	0x80, 0x00,
	0x80, 0x00,
	0x80, 0x00
};

int icm42686_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3])
{
	index *= PACKET_SIZE;

	if ((data[index] & 0x80) == 0x80)
		return 1; // Skip empty packets

	if ((data[index] & 0x7F) == 0x7F)
		return 1; // Skip empty packets

	float a_raw[3] = {0};
	float g_raw[3] = {0};

	if (memcmp(&data[index + 1], invalid, sizeof(invalid)))
	{
		for (int i = 0; i < 3; i++)
		{
			a_raw[i] = (int32_t)((((uint32_t)data[index + 1 + (i * 2)]) << 24) |
								 (((uint32_t)data[index + 2 + (i * 2)]) << 16) |
								 (((uint32_t)data[index + 17 + i] & 0xF0) << 8));
		}
	}

	if (memcmp(&data[index + 7], invalid, sizeof(invalid)))
	{
		for (int i = 0; i < 3; i++)
		{
			g_raw[i] = (int32_t)((((uint32_t)data[index + 7 + (i * 2)]) << 24) |
								 (((uint32_t)data[index + 8 + (i * 2)]) << 16) |
								 (((uint32_t)data[index + 17 + i] & 0x0F) << 12));
		}
	}
	else if (!memcmp(&data[index + 1], invalid, sizeof(invalid)))
	{
		return 1;
	}

	for (int i = 0; i < 3; i++)
	{
		a_raw[i] *= accel_sensitivity_32;
		g_raw[i] *= gyro_sensitivity_32;
	}

	memcpy(a, a_raw, sizeof(a_raw));
	memcpy(g, g_raw, sizeof(g_raw));

	return 0;
}

void icm42686_accel_read(float a[3])
{
	uint8_t rawAccel[6];

	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU,
							 ICM42686_ACCEL_DATA_X1,
							 &rawAccel[0],
							 6);

	if (err)
		LOG_ERR("Communication error");

	for (int i = 0; i < 3; i++)
	{
		a[i] = (int16_t)((((uint16_t)rawAccel[i * 2]) << 8) |
						  rawAccel[1 + (i * 2)]);

		a[i] *= accel_sensitivity;
	}
}

void icm42686_gyro_read(float g[3])
{
	uint8_t rawGyro[6];

	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU,
							 ICM42686_GYRO_DATA_X1,
							 &rawGyro[0],
							 6);

	if (err)
		LOG_ERR("Communication error");

	for (int i = 0; i < 3; i++)
	{
		g[i] = (int16_t)((((uint16_t)rawGyro[i * 2]) << 8) |
						  rawGyro[1 + (i * 2)]);

		g[i] *= gyro_sensitivity;
	}
}

float icm42686_temp_read(void)
{
	uint8_t rawTemp[2];

	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU,
							 ICM42686_TEMP_DATA1,
							 &rawTemp[0],
							 2);

	if (err)
		LOG_ERR("Communication error");

	float temp = (int16_t)((((uint16_t)rawTemp[0]) << 8) | rawTemp[1]);

	temp /= 132.48f;
	temp += 25;

	return temp;
}

uint8_t icm42686_setup_DRDY(uint16_t threshold)
{
	uint8_t buf[2];

	buf[0] = threshold & 0xFF;
	buf[1] = (threshold >> 8) & 0x0F;

	int err = ssi_burst_write(SENSOR_INTERFACE_DEV_IMU,
							  ICM42686_FIFO_CONFIG2,
							  buf,
							  2);

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM42686_INT_SOURCE0,
							  0x04); // FIFO threshold interrupt

	if (err)
		LOG_ERR("Communication error");

	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW;
}

uint8_t icm42686_setup_WOM(void)
{
	uint8_t interrupts;

	int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU,
								ICM42686_INT_STATUS,
								&interrupts);

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM42686_INT_SOURCE0,
							  0x00); // disable default interrupt

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM42686_ACCEL_CONFIG0,
							  ICM42686_AFS_8G << 5 |
							  ICM42686_AODR_200Hz);

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM42686_PWR_MGMT0,
							  ICM42686_aMode_LP);

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM42686_INTF_CONFIG1,
							  0x00); // set low power clock

	k_msleep(1);

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM42686_REG_BANK_SEL,
							  0x04); // select register bank 4

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM42686_ACCEL_WOM_X_THR,
							  0x08);

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM42686_ACCEL_WOM_Y_THR,
							  0x08);

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM42686_ACCEL_WOM_Z_THR,
							  0x08);

	k_msleep(1);

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM42686_REG_BANK_SEL,
							  0x00); // select register bank 0

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM42686_INT_SOURCE1,
							  0x07); // enable WOM interrupt

	k_msleep(50);

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM42686_SMD_CONFIG,
							  0x01); // enable WOM feature

	if (err)
		LOG_ERR("Communication error");

	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW;
}

const sensor_imu_t sensor_imu_icm42686 = {
	icm42686_init,
	icm42686_shutdown,

	icm42686_update_fs,
	icm42686_update_odr,

	icm42686_fifo_read,
	icm42686_fifo_process,
	icm42686_accel_read,
	icm42686_gyro_read,
	icm42686_temp_read,

	icm42686_setup_DRDY,
	icm42686_setup_WOM,

	imu_none_ext_setup,
	imu_none_ext_passthrough
};
