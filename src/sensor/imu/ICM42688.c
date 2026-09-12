/* 01/14/2022 Copyright Tlera Corporation

	Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Ladybug default), respectively, and it uses the Ladybug STM32L432 Breakout
  Board. The ICM42688 is a combo sensor with embedded accel and gyro, here used as 6 DoF in a 9 DoF absolute orientation
  solution.

  Library may be used freely and without limit with attribution.

*/
#include <math.h>

#include <zephyr/logging/log.h>
#include <hal/nrf_gpio.h>

#include "ICM42688.h"
#include "icm426xx_hires.h"
#include "sensor/sensor_none.h"

#define PACKET_SIZE ICM426XX_HIRES_PACKET_SIZE
#define ICM42688_FIFO_COUNT_RECORDS 0x40
#define ICM42688_FIFO_HIRES_EN 0x10
#define ICM42688_FIFO_TEMP_EN 0x04

static const float accel_sensitivity = 16.0f / 32768.0f;  // Always 16G
static const float gyro_sensitivity = 2000.0f / 32768.0f; // Always 2000dps

static const float accel_sensitivity_32 = 16.0f / ((uint32_t)2 << 30);  // 16G forced
static const float gyro_sensitivity_32 = 2000.0f / ((uint32_t)2 << 30); // 2000dps forced

static const float odr_hz[]
	= {32000.0f, 16000.0f, 8000.0f, 4000.0f, 2000.0f, 1000.0f, 500.0f, 200.0f, 100.0f, 50.0f, 25.0f, 12.5f};
static const uint8_t odrs[]
	= {AODR_32kHz,
	   AODR_16kHz,
	   AODR_8kHz,
	   AODR_4kHz,
	   AODR_2kHz,
	   AODR_1kHz,
	   AODR_500Hz,
	   AODR_200Hz,
	   AODR_100Hz,
	   AODR_50Hz,
	   AODR_25Hz,
	   AODR_12_5Hz};

static uint8_t last_accel_odr = 0xff;
static uint8_t last_gyro_odr = 0xff;
static uint8_t last_accel_mode = 0xff;
static uint8_t last_gyro_mode = 0xff;
static const float clock_reference = 32000;
static float clock_scale = 1; // ODR is scaled by clock_rate/clock_reference

// Existing per-packet transfer-time estimates; timing rationale remains unverified.
#define FIFO_MULT 0.00075f    // seconds, I2C
#define FIFO_MULT_SPI 0.0001f // seconds, SPI

static float fifo_multiplier_factor = FIFO_MULT;
static float fifo_multiplier = 0;

static float fifo_temp;
static bool fifo_temp_valid;

LOG_MODULE_REGISTER(ICM42688, LOG_LEVEL_DBG);

int icm_init(
	float clock_rate,
	float accel_period_s,
	float gyro_period_s,
	float *accel_actual_period_s,
	float *gyro_actual_period_s
)
{
	fifo_temp_valid = false;
	// setup interface for SPI
	if (!sensor_interface_spi_configure(SENSOR_INTERFACE_DEV_IMU, MHZ(24), 0)) {
		fifo_multiplier_factor = FIFO_MULT_SPI; // SPI mode
	} else {
		fifo_multiplier_factor = FIFO_MULT; // I2C mode
	}
	int err = 0;
	err |= ssi_reg_update_byte(
		SENSOR_INTERFACE_DEV_IMU,
		ICM42688_INTF_CONFIG0,
		ICM42688_FIFO_COUNT_RECORDS,
		ICM42688_FIFO_COUNT_RECORDS
	); // FIFO_COUNT and FIFO_WM use records
	clock_scale = 1.0f;
	if (clock_rate > 0) {
		clock_scale = clock_rate / clock_reference;
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_REG_BANK_SEL, 0x01); // select register bank 1
		err |= ssi_reg_write_byte(
			SENSOR_INTERFACE_DEV_IMU,
			ICM42688_INTF_CONFIG5,
			0x04
		); // use CLKIN (set PIN9_FUNCTION to CLKIN)
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
		err |= ssi_reg_update_byte(
			SENSOR_INTERFACE_DEV_IMU,
			ICM42688_INTF_CONFIG1,
			0x04,
			0x04
		); // use CLKIN (set RTC_MODE to require RTC clock input)
	}
	last_accel_odr = 0xff; // reset last odr
	last_gyro_odr = 0xff;  // reset last odr
	last_accel_mode = 0xff;
	last_gyro_mode = 0xff;
	err |= icm_update_odr(accel_period_s, gyro_period_s, accel_actual_period_s, gyro_actual_period_s);
	// Keep reset-default filter bandwidths; the historical ODR/10 override was inactive.
	k_msleep(1); // Existing pre-FIFO delay; startup-margin rationale remains unverified.
	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		ICM42688_FIFO_CONFIG1,
		ICM42688_FIFO_HIRES_EN | ICM42688_FIFO_TEMP_EN
	); // Full-byte 0x14 configuration: hires + temperature; accel/gyro enable bits remain clear.
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_FIFO_CONFIG, 1 << 6); // begin FIFO stream

	// Use FIFO activity as the existing CLKIN fallback heuristic, not a clock measurement.
	if (clock_rate > 0) {
		k_msleep(10); // wait for FIFO samples to accumulate
		uint8_t raw_count[2];
		ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, ICM42688_FIFO_COUNTH, raw_count, 2);
		uint16_t fifo_count = (uint16_t)(raw_count[0] << 8 | raw_count[1]);
		if (fifo_count == 0) {
			LOG_WRN("External CLKIN not working, falling back to internal clock");
			clock_scale = 1;
			// Disable CLKIN: revert PIN9_FUNCTION and RTC_MODE
			err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_REG_BANK_SEL, 0x01);
			err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_INTF_CONFIG5, 0x00);
			err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_REG_BANK_SEL, 0x00);
			err |= ssi_reg_update_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_INTF_CONFIG1, 0x04, 0x00);
			// Recalculate ODR without clock scaling
			last_accel_odr = 0xff;
			last_gyro_odr = 0xff;
			last_accel_mode = 0xff;
			last_gyro_mode = 0xff;
			err |= icm_update_odr(accel_period_s, gyro_period_s, accel_actual_period_s, gyro_actual_period_s);
		} else {
			LOG_INF("External CLKIN verified: FIFO count=%d", fifo_count);
		}
	}

	if (err) {
		LOG_ERR("Communication error");
	}
	return (err < 0 ? err : 0);
}

void icm_shutdown(void)
{
	fifo_temp_valid = false;
	last_accel_odr = 0xff; // reset last odr
	last_gyro_odr = 0xff;  // reset last odr
	last_accel_mode = 0xff;
	last_gyro_mode = 0xff;
	int err = ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		ICM42688_DEVICE_CONFIG,
		0x01
	); // Don't need to wait for ICM to finish reset
	if (err) {
		LOG_ERR("Communication error");
	}
}

void icm_update_fs(float accel_range, float gyro_range, float *accel_actual_range, float *gyro_actual_range)
{
	ARG_UNUSED(accel_range);
	ARG_UNUSED(gyro_range);
	*accel_actual_range = 16;  // always 16g in hires
	*gyro_actual_range = 2000; // always 2000dps in hires
}

int icm_update_odr(float accel_period_s, float gyro_period_s, float *accel_actual_period_s, float *gyro_actual_period_s)
{
	float requested_odr_hz;
	uint8_t accel_fs_bits = AFS_16G;    // set highest
	uint8_t gyro_fs_bits = GFS_2000DPS; // set highest
	uint8_t accel_mode;
	uint8_t gyro_mode;
	uint8_t accel_odr_bits = 0;
	uint8_t gyro_odr_bits = 0;

	// Calculate accel
	if (accel_period_s <= 0 || accel_period_s == INFINITY) // off, standby interpreted as off
	{
		accel_mode = aMode_OFF;
		accel_period_s = 0;
	} else {
		accel_mode = aMode_LN;
		requested_odr_hz = (1.0f / accel_period_s) / clock_scale;
		size_t selected = 0;
		for (size_t i = 1; i < ARRAY_SIZE(odr_hz); i++) {
			if (requested_odr_hz > odr_hz[i]) {
				break;
			}
			selected = i;
		}
		accel_odr_bits = odrs[selected];
		accel_period_s = 1.0f / odr_hz[selected];
	}
	accel_period_s /= clock_scale; // scale clock

	// Calculate gyro
	if (gyro_period_s <= 0) // off
	{
		gyro_mode = gMode_OFF;
		gyro_period_s = 0;
	} else if (gyro_period_s == INFINITY) // standby
	{
		gyro_mode = gMode_SBY;
		gyro_period_s = 0;
	} else {
		gyro_mode = gMode_LN;
		requested_odr_hz = (1.0f / gyro_period_s) / clock_scale;
		size_t selected = 0;
		for (size_t i = 1; i < ARRAY_SIZE(odr_hz); i++) {
			if (requested_odr_hz > odr_hz[i]) {
				break;
			}
			selected = i;
		}
		gyro_odr_bits = odrs[selected];
		gyro_period_s = 1.0f / odr_hz[selected];
	}
	gyro_period_s /= clock_scale; // scale clock

	if (last_accel_odr == accel_odr_bits && last_gyro_odr == gyro_odr_bits && last_accel_mode == accel_mode
		&& last_gyro_mode == gyro_mode) {
		*accel_actual_period_s = accel_period_s;
		*gyro_actual_period_s = gyro_period_s;
		return 0; /* already configured — success for err|= callers */
	}

	int err = 0;
	// only if the power mode has changed
	if (last_accel_mode != accel_mode || last_gyro_mode != gyro_mode) {
		err |= ssi_reg_write_byte(
			SENSOR_INTERFACE_DEV_IMU,
			ICM42688_PWR_MGMT0,
			gyro_mode << 2 | accel_mode
		);                // set accel and gyro modes
		k_busy_wait(250); // wait >200us (datasheet 14.36)
	}

	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		ICM42688_ACCEL_CONFIG0,
		accel_fs_bits << 5 | accel_odr_bits
	); // set accel ODR and FS
	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		ICM42688_GYRO_CONFIG0,
		gyro_fs_bits << 5 | gyro_odr_bits
	); // set gyro ODR and FS
	if (err) {
		last_accel_odr = 0xff;
		last_gyro_odr = 0xff;
		last_accel_mode = 0xff;
		last_gyro_mode = 0xff;
		LOG_ERR("Communication error");
		return err;
	}

	last_accel_odr = accel_odr_bits;
	last_gyro_odr = gyro_odr_bits;
	last_accel_mode = accel_mode;
	last_gyro_mode = gyro_mode;
	*accel_actual_period_s = accel_period_s;
	*gyro_actual_period_s = gyro_period_s;

	// extra read packets by ODR time
	if (accel_period_s == 0 && gyro_period_s != 0) {
		fifo_multiplier = fifo_multiplier_factor / gyro_period_s;
	} else if (accel_period_s != 0 && gyro_period_s == 0) {
		fifo_multiplier = fifo_multiplier_factor / accel_period_s;
	} else if (gyro_period_s > accel_period_s) {
		fifo_multiplier = fifo_multiplier_factor / accel_period_s;
	} else if (accel_period_s > gyro_period_s) {
		fifo_multiplier = fifo_multiplier_factor / gyro_period_s;
	} else {
		fifo_multiplier = 0;
	}

	return 0;
}

uint16_t icm_fifo_read(uint8_t *data, uint16_t capacity_bytes)
{
	fifo_temp_valid = false;
	uint16_t total_packets = 0;
	uint16_t packet_count = UINT16_MAX;
	while (packet_count > 0 && capacity_bytes >= PACKET_SIZE) {
		uint8_t raw_count[2];
		int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, ICM42688_FIFO_COUNTH, &raw_count[0], 2);
		if (err) {
			fifo_temp_valid = false;
			LOG_ERR("Failed to read FIFO count");
			return total_packets;
		}
		packet_count = (uint16_t)(raw_count[0] << 8 | raw_count[1]); // Big-endian record count.
		if (!packet_count) {                                         // nothing to do
			break;
		}
		float extra_read_packets = packet_count * fifo_multiplier;
		packet_count += extra_read_packets;
		uint16_t byte_count = packet_count * PACKET_SIZE;
		uint16_t packet_capacity = capacity_bytes / PACKET_SIZE;
		if (packet_count > packet_capacity) {
			LOG_WRN("FIFO read buffer limit reached, %d packets dropped", packet_count - packet_capacity);
			packet_count = packet_capacity;
			byte_count = packet_count * PACKET_SIZE;
		}
		err = ssi_burst_read_interval(SENSOR_INTERFACE_DEV_IMU, ICM42688_FIFO_DATA, data, byte_count, PACKET_SIZE);
		if (err) {
			fifo_temp_valid = false;
			LOG_ERR("Communication error");
			return total_packets;
		}
		if (!icm426xx_hires_temperature(data, packet_count, &fifo_temp)) {
			fifo_temp_valid = true;
		}
		data += packet_count * PACKET_SIZE;
		capacity_bytes -= packet_count * PACKET_SIZE;
		total_packets += packet_count;
	}
	return total_packets;
}

int icm_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3])
{
	const uint16_t packet_offset = index * PACKET_SIZE;
	const uint8_t *packet = &data[packet_offset];
	return icm426xx_hires_decode(packet, accel_sensitivity_32, gyro_sensitivity_32, a, g);
}

void icm_accel_read(float a[3])
{
	uint8_t raw_accel[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, ICM42688_ACCEL_DATA_X1, &raw_accel[0], 6);
	if (err) {
		LOG_ERR("Communication error");
		memset(a, 0, 3 * sizeof(*a));
		return;
	}
	for (int i = 0; i < 3; i++) // x, y, z
	{
		a[i] = (int16_t)((((uint16_t)raw_accel[i * 2]) << 8) | raw_accel[1 + (i * 2)]);
		a[i] *= accel_sensitivity;
	}
}

void icm_gyro_read(float g[3])
{
	uint8_t raw_gyro[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, ICM42688_GYRO_DATA_X1, &raw_gyro[0], 6);
	if (err) {
		LOG_ERR("Communication error");
		memset(g, 0, 3 * sizeof(*g));
		return;
	}
	for (int i = 0; i < 3; i++) // x, y, z
	{
		g[i] = (int16_t)((((uint16_t)raw_gyro[i * 2]) << 8) | raw_gyro[1 + (i * 2)]);
		g[i] *= gyro_sensitivity;
	}
}

float icm_temp_read(void)
{
	if (fifo_temp_valid) {
		return fifo_temp;
	}

	uint8_t raw_temp[2];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, ICM42688_TEMP_DATA1, &raw_temp[0], 2);
	if (err) {
		LOG_ERR("Communication error");
		return NAN;
	}
	// Temperature in Degrees Centigrade = (TEMP_DATA / 132.48) + 25
	float temp = (int16_t)((((uint16_t)raw_temp[0]) << 8) | raw_temp[1]);
	temp /= 132.48f;
	temp += 25;
	return temp;
}

uint8_t icm_setup_DRDY(uint16_t threshold)
{
	uint8_t buf[2];
	buf[0] = threshold & 0xFF;
	buf[1] = (threshold >> 8) & 0x0F;
	int err = ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM42688_FIFO_CONFIG2, buf, 2);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_INT_SOURCE0, 0x04); // FIFO threshold interrupt
	if (err) {
		LOG_ERR("Communication error");
	}
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
}

uint8_t icm_setup_WOM(void)
{
	uint8_t interrupts;
	int err
		= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_INT_STATUS, &interrupts); // clear reset done int flag
	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		ICM42688_INT_SOURCE0,
		0x00
	); // disable default interrupt (RESET_DONE)
	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		ICM42688_ACCEL_CONFIG0,
		AFS_8G << 5 | AODR_200Hz
	);                                                                                 // set accel ODR and FS
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_PWR_MGMT0, aMode_LP); // set accel and gyro modes
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_INTF_CONFIG1, 0x00);  // set low power clock
	k_msleep(1);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_REG_BANK_SEL, 0x04); // select register bank 4
	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		ICM42688_ACCEL_WOM_X_THR,
		0x08
	); // set wake thresholds // 8 x 3.9 mg is ~31.25 mg
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_ACCEL_WOM_Y_THR, 0x08); // set wake thresholds
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_ACCEL_WOM_Z_THR, 0x08); // set wake thresholds
	k_msleep(1);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_INT_SOURCE1, 0x07);  // enable WOM interrupt
	k_msleep(50); // Existing WOM settling delay; required margin remains unverified.
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM42688_SMD_CONFIG, 0x01); // enable WOM feature
	if (err) {
		LOG_ERR("Communication error");
	}
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
}

const sensor_imu_t sensor_imu_icm42688
	= {*icm_init,
	   *icm_shutdown,

	   *icm_update_fs,
	   *icm_update_odr,

	   *icm_fifo_read,
	   *icm_fifo_process,
	   *icm_accel_read,
	   *icm_gyro_read,
	   *icm_temp_read,

	   *icm_setup_DRDY,
	   *icm_setup_WOM,

	   *imu_none_ext_setup};
