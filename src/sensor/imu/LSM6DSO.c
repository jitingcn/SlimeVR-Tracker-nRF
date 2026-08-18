#include <math.h>
#include <string.h>

#include <zephyr/logging/log.h>
#include <hal/nrf_gpio.h>

#include "LSM6DSO.h"
#include "LSM6DSV.h" // Common functions
#include "sensor/sensor_none.h"

#define PACKET_SIZE 7

static uint8_t accel_fs = DSO_FS_XL_16G;
static uint8_t gyro_fs = DSO_FS_G_2000DPS;

static float freq_scale = 1; // ODR is scaled by INTERNAL_FREQ_FINE

#define LSM6DSO_SHUB_XLDA_TIMEOUT_MS 80
#define LSM6DSO_SHUB_OP_TIMEOUT_MS 20

LOG_MODULE_REGISTER(LSM6DSO, LOG_LEVEL_DBG);

int lsm6dso_init(float clock_rate, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	// setup interface for SPI
	sensor_interface_spi_configure(SENSOR_INTERFACE_DEV_IMU, MHZ(10), 0);
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_CTRL3, 0x74); // freeze register until done reading, increment register address during multi-byte access (BDU, IF_INC), INT H_LACTIVE active low, PP_OD open-drain
	if (err)
		LOG_ERR("Communication error");
	last_accel_odr = 0xff; // reset last odr
	last_gyro_odr = 0xff; // reset last odr
	int8_t internal_freq_fine;
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_INTERNAL_FREQ_FINE, &internal_freq_fine); // affects ODR
	freq_scale = 1.0f + 0.0015f * (float)internal_freq_fine;
	err |= lsm6dso_update_odr(accel_time, gyro_time, accel_actual_time, gyro_actual_time);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_FIFO_CTRL4, 0x06); // enable Continuous mode
	if (err)
		LOG_ERR("Communication error");
	return (err < 0 ? err : 0);
}

void lsm6dso_update_fs(float accel_range, float gyro_range, float *accel_actual_range, float *gyro_actual_range)
{
	if (accel_range > 8)
	{
		accel_fs = DSO_FS_XL_16G;
		accel_range = 16;
	}
	else if (accel_range > 4)
	{
		accel_fs = DSO_FS_XL_8G;
		accel_range = 8;
	}
	else if (accel_range > 2)
	{
		accel_fs = DSO_FS_XL_4G;
		accel_range = 4;
	}
	else
	{
		accel_fs = DSO_FS_XL_2G;
		accel_range = 2;
	}

	if (gyro_range > 1000)
	{
		gyro_fs = DSO_FS_G_2000DPS;
		gyro_range = 2000;
	}
	else if (gyro_range > 500)
	{
		gyro_fs = DSO_FS_G_1000DPS;
		gyro_range = 1000;
	}
	else if (gyro_range > 250)
	{
		gyro_fs = DSO_FS_G_500DPS;
		gyro_range = 500;
	}
	else
	{
		gyro_fs = DSO_FS_G_250DPS;
		gyro_range = 250;
	}

	accel_sensitivity = accel_range / 32768.0f;
	gyro_sensitivity = 35.0f * gyro_range / 1000000.0f;

	*accel_actual_range = accel_range;
	*gyro_actual_range = gyro_range;
}

int lsm6dso_update_odr(float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	int ODR;
	uint8_t OP_MODE_XL;
	uint8_t OP_MODE_G;
	uint8_t ODR_XL;
	uint8_t ODR_G;
	uint8_t GYRO_SLEEP = DSO_OP_MODE_G_AWAKE;

	// Calculate accel
	if (accel_time <= 0 || accel_time == INFINITY) // off, standby interpreted as off
	{
		// set High perf mode and off odr on XL
		OP_MODE_XL = DSO_OP_MODE_XL_HP;
		ODR_XL = DSO_ODR_OFF;
		ODR = 0;
	}
	else
	{
		// set High perf mode and select odr on XL
		OP_MODE_XL = DSO_OP_MODE_XL_HP;
		ODR = 1 / accel_time;
		ODR /= freq_scale; // scale by internal freq adjustment
	}

	if (ODR == 0)
	{
		accel_time = 0; // off
		ODR_XL = DSO_ODR_OFF;
	}
	else if (accel_time < 0.3f / 1000) // in this case it seems better to compare accel_time
	{
		ODR_XL = DSO_ODR_6_66kHz; // TODO: this is absolutely awful
		accel_time = 0.15 / 1000;
	}
	else if (accel_time < 0.6f / 1000)
	{
		ODR_XL = DSO_ODR_3_33kHz;
		accel_time = 0.3 / 1000;
	}
	else if (accel_time < 1.2f / 1000)
	{
		ODR_XL = DSO_ODR_1_66kHz;
		accel_time = 0.6 / 1000;
	}
	else if (accel_time < 2.4f / 1000)
	{
		ODR_XL = DSO_ODR_833Hz;
		accel_time = 1.2 / 1000;
	}
	else if (accel_time < 4.8f / 1000)
	{
		ODR_XL = DSO_ODR_416Hz;
		accel_time = 2.4 / 1000;
	}
	else if (accel_time < 9.6f / 1000)
	{
		ODR_XL = DSO_ODR_208Hz;
		accel_time = 4.8 / 1000;
	}
	else if (accel_time < 19.2f / 1000)
	{
		ODR_XL = DSO_ODR_104Hz;
		accel_time = 9.6 / 1000;
	}
	else if (accel_time < 38.4f / 1000)
	{
		ODR_XL = DSO_ODR_52Hz;
		accel_time = 19.2 / 1000;
	}
	else if (ODR > 12.5)
	{
		ODR_XL = DSO_ODR_26Hz;
		accel_time = 38.4 / 1000;
	}
	else
	{
		ODR_XL = DSO_ODR_12_5Hz;
		accel_time = 1.0 / 12.5; // 13Hz -> 76.8 / 1000
	}
	accel_time /= freq_scale; // scale by internal freq adjustment

	// Calculate gyro
	if (gyro_time <= 0) // off
	{
		OP_MODE_G = DSO_OP_MODE_G_HP;
		ODR_G = DSO_ODR_OFF;
		ODR = 0;
	}
	else if (gyro_time == INFINITY) // sleep
	{
		OP_MODE_G = DSO_OP_MODE_G_NP;
		GYRO_SLEEP = DSO_OP_MODE_G_SLEEP;
		ODR_G = last_gyro_odr; // using last ODR
		ODR = -1; /* not off: skip ODR_OFF overwrite below */
	}
	else
	{
		OP_MODE_G = DSO_OP_MODE_G_HP;
		ODR_G = 0; // the compiler complains unless I do this
		ODR = 1 / gyro_time;
		ODR /= freq_scale; // scale by internal freq adjustment
	}

	if (ODR == 0)
	{
		gyro_time = 0; // off
		ODR_G = DSO_ODR_OFF;
	}
	else if (ODR < 0)
	{
		/* sleep: keep ODR_G = last_gyro_odr */
		gyro_time = INFINITY;
	}
	else if (gyro_time < 0.3f / 1000) // in this case it seems better to compare gyro_time
	{
		ODR_G = DSO_ODR_6_66kHz; // TODO: this is absolutely awful
		gyro_time = 1.0 / 6660;
	}
	else if (gyro_time < 0.6f / 1000)
	{
		ODR_G = DSO_ODR_3_33kHz;
		gyro_time = 0.3 / 1000;
	}
	else if (gyro_time < 1.2f / 1000)
	{
		ODR_G = DSO_ODR_1_66kHz;
		gyro_time = 0.6 / 1000;
	}
	else if (gyro_time < 2.4f / 1000)
	{
		ODR_G = DSO_ODR_833Hz;
		gyro_time = 1.2 / 1000;
	}
	else if (gyro_time < 4.8f / 1000)
	{
		ODR_G = DSO_ODR_416Hz;
		gyro_time = 2.4 / 1000;
	}
	else if (gyro_time < 9.6f / 1000)
	{
		ODR_G = DSO_ODR_208Hz;
		gyro_time = 4.8 / 1000;
	}
	else if (gyro_time < 19.2f / 1000)
	{
		ODR_G = DSO_ODR_104Hz;
		gyro_time = 9.6 / 1000;
	}
	else if (gyro_time < 38.4f / 1000)
	{
		ODR_G = DSO_ODR_52Hz;
		gyro_time = 19.2 / 1000;
	}
	else if (ODR > 12.5)
	{
		ODR_G = DSO_ODR_26Hz;
		gyro_time = 38.4 / 1000;
	}
	else
	{
		ODR_G = DSO_ODR_12_5Hz;
		gyro_time = 1.0 / 12.5; // 13Hz -> 76.8 / 1000
	}
	gyro_time /= freq_scale; // scale by internal freq adjustment

	if (last_accel_mode == OP_MODE_XL && last_gyro_mode == OP_MODE_G && last_accel_odr == ODR_XL && last_gyro_odr == ODR_G) {
		*accel_actual_time = accel_time;
		*gyro_actual_time = gyro_time;
		return 0; /* already configured — success for err|= callers */
	}

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_CTRL1, ODR_XL | accel_fs); // set accel ODR and FS
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_CTRL6, OP_MODE_XL); // set accelerator perf mode

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_CTRL2, ODR_G | gyro_fs); // set gyro ODR and mode
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_CTRL7, OP_MODE_G); // set gyroscope perf mode
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_CTRL4, GYRO_SLEEP); // set gyroscope awake/sleep mode

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_FIFO_CTRL3, (ODR_XL >> 4) | ODR_G); // set accel and gyro batch rate
	if (err) {
		LOG_ERR("Communication error");
		return err;
	}

	last_accel_mode = OP_MODE_XL;
	last_gyro_mode = OP_MODE_G;
	last_accel_odr = ODR_XL;
	last_gyro_odr = ODR_G;
	*accel_actual_time = accel_time;
	*gyro_actual_time = gyro_time;

	return 0;
}

uint16_t lsm6dso_fifo_read(uint8_t *data, uint16_t len)
{
	uint16_t total = 0;
	uint16_t count = UINT16_MAX;
	while (count > 0 && len >= PACKET_SIZE)
	{
		uint8_t rawCount[2];
		int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_FIFO_STATUS1, &rawCount[0], 2);
		if (err)
		{
			LOG_ERR("Failed to read FIFO status");
			return total;
		}
		if (rawCount[1] & BIT(3))
			LOG_WRN("FIFO overrun latched");
		if (rawCount[1] & BIT(5))
			LOG_WRN("FIFO full");
		if (rawCount[1] & BIT(6))
			LOG_WRN("FIFO overrun");
		count = (uint16_t)((rawCount[1] & 0x03) << 8 | rawCount[0]);
		if (!count) // nothing to do
			break;
		uint16_t limit = len / PACKET_SIZE;
		if (count > limit)
		{
			LOG_WRN("FIFO read buffer limit reached, %d packets dropped", count - limit);
			count = limit;
		}
		err = ssi_burst_read_interval(
			SENSOR_INTERFACE_DEV_IMU,
			LSM6DSO_FIFO_DATA_OUT_TAG,
			data,
			count * PACKET_SIZE,
			PACKET_SIZE
		);
		if (err)
		{
			LOG_ERR("Communication error");
			return total;
		}
		data += count * PACKET_SIZE;
		len -= count * PACKET_SIZE;
		total += count;
	}
	return total;
}

uint8_t lsm6dso_setup_WOM(void)
{ // TODO: should be off by the time WOM will be setup
//	ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_CTRL1, ODR_OFF); // set accel off
//	ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_CTRL2, ODR_OFF); // set gyro off

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_CTRL1, DSO_ODR_208Hz | DSO_FS_XL_8G); // set accel ODR and FS
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_CTRL6, DSO_OP_MODE_XL_NP); // set accel perf mode
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_CTRL5, 0x80); // enable accel ULP // TODO: for LSM6DSR/ISM330DHCX this bit may be required to be 0
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_CTRL8, 0xF4); // set HPCF_XL to the lowest bandwidth, enable HP_REF_MODE (set HP_REF_MODE_XL, HP_SLOPE_XL_EN, HPCF_XL nonzero)
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_TAP_CFG0, 0x10); // set SLOPE_FDS
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_WAKE_UP_THS, 0x01); // set threshold, 1 * 31.25 mg is ~31.25 mg
	k_msleep(12); // need to wait for accel to settle

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_TAP_CFG2, 0x80); // enable interrupts
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_MD1_CFG, 0x20); // route wake-up to INT1
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_CTRL3, 0x30); // INT H_LACTIVE active low, PP_OD open-drain
	if (err)
		LOG_ERR("Communication error");
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
}

int lsm6dso_ext_setup(void)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_FUNC_CFG_ACCESS, 0x40); // switch to sensor hub registers
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_MASTER_CONFIG, 0x08); // SHUB_PU_EN, disable I2C master
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_FUNC_CFG_ACCESS, 0x00); // switch to normal registers
	k_usleep(350);
	// One-shot sensor-hub transactions start on accel data-ready, so scan with
	// accel running before lsm6dso_init() reconfigures the final runtime ODR.
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_CTRL1, DSO_ODR_208Hz | DSO_FS_XL_8G);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_CTRL6, DSO_OP_MODE_XL_HP);
	k_msleep(5);
	if (err)
		LOG_ERR("Communication error");
	sensor_interface_ext_configure(&sensor_ext_lsm6dso);
	return err;
}

int lsm6dso_ext_passthrough(bool passthrough)
{
	int err = 0;
	if (passthrough)
	{
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_FUNC_CFG_ACCESS, 0x40); // switch to sensor hub registers
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_MASTER_CONFIG, 0x10); // passthrough on
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_FUNC_CFG_ACCESS, 0x00); // switch to normal registers
	}
	else
	{
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_FUNC_CFG_ACCESS, 0x40); // switch to sensor hub registers
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_MASTER_CONFIG, 0x08); // SHUB_PU_EN, passthrough off
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_FUNC_CFG_ACCESS, 0x00); // switch to normal registers
	}
	if (err)
		LOG_ERR("Communication error");
	return 0;
}

int lsm6dso_ext_write(const uint8_t addr, const uint8_t *buf, uint32_t num_bytes)
{
	if (num_bytes != 2)
	{
		LOG_ERR("Unsupported write");
		return -1;
	}
	// Configure transaction and begin one-shot write.
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_FUNC_CFG_ACCESS, 0x40); // switch to sensor hub registers
	uint8_t slv0[3] = {(addr << 1) | 0x00, buf[0], 0x00 | 0x00}; // write, SHUB_ODR = 104Hz, reading no bytes
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_SLV0_ADD, slv0, 3);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_DATAWRITE_SLV0, buf[1]);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_MASTER_CONFIG, 0x4C); // WRITE_ONCE, SHUB_PU_EN, enable I2C master
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_FUNC_CFG_ACCESS, 0x00); // switch to normal registers
	uint8_t tmp;
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_OUTX_H_A, &tmp); // clear current XLDA
	uint8_t status = 0;
	int64_t timeout = k_uptime_get() + LSM6DSO_SHUB_XLDA_TIMEOUT_MS;
	while (!(status & 0x01) && k_uptime_get() < timeout) // wait for new XLDA
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_STATUS_REG, &status);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_FUNC_CFG_ACCESS, 0x40); // switch to sensor hub registers
	status = 0;
	timeout = k_uptime_get() + LSM6DSO_SHUB_OP_TIMEOUT_MS;
	while (!(status & 0x80) && k_uptime_get() < timeout) // WR_ONCE_DONE
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_STATUS_MASTER, &status);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_MASTER_CONFIG, 0x08); // SHUB_PU_EN, disable I2C master
	k_usleep(350);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_FUNC_CFG_ACCESS, 0x00); // switch to normal registers
	if (status & 0x04) // SLAVE0_NACK
	{
		LOG_DBG("Ext I2C write NACK from address 0x%02X", addr);
		return -1;
	}
	if (!(status & 0x80))
	{
		LOG_ERR("Write timeout");
		return -1;
	}
	return err;
}

int lsm6dso_ext_write_read(const uint8_t addr, const void *write_buf, size_t num_write, void *read_buf, size_t num_read)
{
	if (num_write != 1 || num_read < 1 || num_read > 8)
	{
		LOG_ERR("Unsupported write_read");
		return -1;
	}
	uint8_t sub_addr = ((const uint8_t *)write_buf)[0];

	// Configure transaction and begin one-shot read.
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_FUNC_CFG_ACCESS, 0x40); // switch to sensor hub registers
	uint8_t slv0[3] = {(addr << 1) | 0x01, sub_addr, 0x00 | num_read}; // read, SHUB_ODR = 104Hz, reading num_read bytes
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_SLV0_ADD, slv0, 3);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_MASTER_CONFIG, 0x4C); // WRITE_ONCE, SHUB_PU_EN, enable I2C master
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_FUNC_CFG_ACCESS, 0x00); // switch to normal registers
	uint8_t tmp;
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_OUTX_H_A, &tmp); // clear current XLDA
	uint8_t status = 0;
	int64_t timeout = k_uptime_get() + LSM6DSO_SHUB_XLDA_TIMEOUT_MS;
	while (!(status & 0x01) && k_uptime_get() < timeout) // wait for new XLDA
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_STATUS_REG, &status);
	status = 0;
	timeout = k_uptime_get() + LSM6DSO_SHUB_OP_TIMEOUT_MS;
	while (!(status & 0x01) && k_uptime_get() < timeout) // SENS_HUB_ENDOP
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_STATUS_MASTER_MAINPAGE, &status);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_FUNC_CFG_ACCESS, 0x40); // switch to sensor hub registers
	uint8_t master_status = 0;
	ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_STATUS_MASTER, &master_status);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_MASTER_CONFIG, 0x08); // SHUB_PU_EN, disable I2C master
	k_usleep(350);
	if ((master_status & 0x04) || !(status & 0x01)) // SLAVE0_NACK or timeout
	{
		if (master_status & 0x04)
			LOG_DBG("Ext I2C NACK from address 0x%02X", addr);
		else
			LOG_DBG("Ext I2C read timeout for address 0x%02X", addr);
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_FUNC_CFG_ACCESS, 0x00);
		memset(read_buf, 0, num_read);
		return -1;
	}
	err |= ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_SENSOR_HUB_1, read_buf, num_read);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_FUNC_CFG_ACCESS, 0x00); // switch to normal registers
	return err;
}

const sensor_imu_t sensor_imu_lsm6dso = {
	*lsm6dso_init,
	*lsm_shutdown,

	*lsm6dso_update_fs,
	*lsm6dso_update_odr,

	*lsm6dso_fifo_read,
	*lsm_fifo_process,
	*lsm_accel_read,
	*lsm_gyro_read,
	*lsm_temp_read,

	*lsm_setup_DRDY,
	*lsm6dso_setup_WOM,

	*lsm6dso_ext_setup,
	*lsm6dso_ext_passthrough
};

const sensor_ext_ssi_t sensor_ext_lsm6dso = {
	*lsm6dso_ext_write,
	*lsm6dso_ext_write_read,
	8
};
