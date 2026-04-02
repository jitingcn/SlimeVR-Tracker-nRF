#include <math.h>
#include <string.h>

#include <zephyr/logging/log.h>
#include <hal/nrf_gpio.h>

#include "LSM6DSV.h"
#include "sensor/sensor_none.h"

#define PACKET_SIZE 7
LOG_MODULE_REGISTER(LSM6DSV, LOG_LEVEL_DBG);

// TODO: shared with LSM
float accel_sensitivity = 16.0f / 32768.0f; // Default 16G (FS = ±16 g: 0.488 mg/LSB)
float gyro_sensitivity = 0.070f; // Default 2000dps (FS = ±2000 dps: 70 mdps/LSB)

static uint8_t accel_fs = FS_XL_16G;
static uint8_t gyro_fs = FS_G_2000DPS;

static const float odr_rates[] = {7680.0f, 3840.0f, 1920.0f, 960.0f, 480.0f, 240.0f, 120.0f, 60.0f, 30.0f, 15.0f, 7.5f, 1.875f};
static const uint8_t odr_values[] = {ODR_7_68kHz, ODR_3_84kHz, ODR_1_92kHz, ODR_960Hz, ODR_480Hz, ODR_240Hz, ODR_120Hz, ODR_60Hz, ODR_30Hz, ODR_15Hz, ODR_7_5Hz, ODR_1_875Hz};

// Store chip type: 0x70 for LSM6DSV, 0x71 for LSM6DSV16B/ISM330BX
static uint8_t chip_who_am_i = 0x70;

// TODO: shared with LSM
uint8_t last_accel_mode = 0xff;
uint8_t last_gyro_mode = 0xff;
uint8_t last_accel_odr = 0xff;
uint8_t last_gyro_odr = 0xff;

static float freq_scale = 1; // ODR is scaled by INTERNAL_FREQ_FINE

#define LSM6DSV_FIFO_MODE_BYPASS 0x00
#define LSM6DSV_FIFO_MODE_CONTINUOUS 0x06
#define LSM6DSV_UNKNOWN_TAG_RESYNC_THRESHOLD 4

static uint8_t lsm_unknown_tag_count = 0;

static int lsm_fifo_resync(const char *reason)
{
	LOG_WRN("Resyncing FIFO: %s", reason);
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FIFO_CTRL4, LSM6DSV_FIFO_MODE_BYPASS);
	k_usleep(350);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FIFO_CTRL4, LSM6DSV_FIFO_MODE_CONTINUOUS);
	k_usleep(350);
	lsm_unknown_tag_count = 0;
	if (err)
		LOG_ERR("FIFO resync failed");
	return err;
}

// Sensor hub continuous read mode state
// When active, lsm_ext_write_read() reads SENSOR_HUB registers directly
// instead of performing a slow one-shot cycle (~8ms wait for XLDA)
static bool ext_continuous_active = false;
static uint8_t ext_cont_addr = 0;
static uint8_t ext_cont_sub = 0;
static uint8_t ext_cont_len = 0;
static void lsm_ext_stop_continuous(void);

// Scanning mode: when true, one-shot reads never start continuous mode.
// Set during ext_setup() for device scanning, cleared by lsm_init() for normal operation.
static bool ext_scanning_mode = true;

int lsm_init(float clock_rate, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	// setup interface for SPI
	LOG_INF("Initializing LSM6DSV...");
	sensor_interface_spi_configure(SENSOR_INTERFACE_DEV_IMU, MHZ(10), 0);

	// sensor_init() already issued shutdown/reset before calling init.
	// Continue from the post-reset state and rebuild runtime configuration below.
	ext_continuous_active = false;
	ext_scanning_mode = false; // After init, switch to operational mode for immediate continuous

	int err = 0;

	// Read WHO_AM_I to verify communication
	uint8_t who_am_i = 0;
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_WHO_AM_I, &who_am_i);
	LOG_INF("WHO_AM_I = 0x%02X (expected 0x70/0x71)", who_am_i);
	if (who_am_i != 0x70 && who_am_i != 0x71) // 0x70 for LSM6DSV, 0x71 for LSM6DSV16B/ISM330BX
	{
		LOG_ERR("Invalid WHO_AM_I value");
		return -1;
	}

	// Store chip type for data order handling
	chip_who_am_i = who_am_i;

	// Power on sensors first by setting power mode and ODR (following ICM45686 pattern)
	// OP_MODE_XL_HP (000) = high-performance mode (default, valid for all ODRs 7.5Hz-7.68kHz)
	// OP_MODE_G_HP (000) = high-performance mode (default, valid for all ODRs 7.5Hz-7.68kHz)
	// Using 15Hz initially to power on sensors with minimal current
	LOG_INF("Powering on sensors in high-performance mode...");
	uint8_t ctrl1_val = (OP_MODE_XL_HP << 4) | ODR_15Hz;
	uint8_t ctrl2_val = (OP_MODE_G_HP << 4) | ODR_15Hz;
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL1, ctrl1_val); // accel HP mode, 15Hz
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL2, ctrl2_val); // gyro HP mode, 15Hz

	// Read back to verify
	uint8_t ctrl1_readback = 0, ctrl2_readback = 0;
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL1, &ctrl1_readback);
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL2, &ctrl2_readback);
	LOG_INF("CTRL1 write=0x%02X readback=0x%02X, CTRL2 write=0x%02X readback=0x%02X",
		ctrl1_val, ctrl1_readback, ctrl2_val, ctrl2_readback);

	if (err)
		LOG_ERR("Communication error during power-on");

	// Wait for gyroscope startup AFTER powering on (Ton = 30ms typical, 45ms max)
	LOG_INF("Waiting for gyroscope startup (30ms)...");
	k_msleep(30);

	// Now configure FS and other settings after sensors are powered and stable
	// Configure gyro FS + LPF1 bandwidth in CTRL6
	// LPF1_G_BW[2:0] (bits [6:4]): 010 = ~ODR/4 bandwidth (Table 63)
	// FS_G[3:0] (bits [3:0]): gyro full-scale selection
	uint8_t ctrl6_val = (0x02 << 4) | gyro_fs; // LPF1_G_BW=010, gyro FS
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL6, ctrl6_val);

	// Enable gyro digital LPF1 in CTRL7
	// LPF1_G_EN (bit 0): 1 = enable gyro LPF1 filter
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL7, 0x01);

	// Configure accel FS + LPF2 bandwidth in CTRL8
	// HP_LPF2_XL_BW[2:0] (bits [7:5]): 000 = ODR/4 bandwidth (Table 68)
	// FS_XL[1:0] (bits [1:0]): accel full-scale selection
	uint8_t ctrl8_val = (0x00 << 5) | accel_fs; // HP_LPF2_XL_BW=000 (ODR/4), accel FS
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL8, ctrl8_val);

	// Enable accel LPF2 (low-pass mode) in CTRL9
	// LPF2_XL_EN (bit 3): 1 = enable second-stage LPF2
	// HP_SLOPE_XL_EN (bit 4): 0 = low-pass filter path selected
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL9, 0x08); // LPF2_XL_EN=1

	// Read back to verify FS + LPF configuration
	uint8_t ctrl6_readback = 0, ctrl7_readback = 0, ctrl8_readback = 0, ctrl9_readback = 0;
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL6, &ctrl6_readback);
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL7, &ctrl7_readback);
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL8, &ctrl8_readback);
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL9, &ctrl9_readback);
	LOG_INF("CTRL6 write=0x%02X rb=0x%02X, CTRL7 write=0x01 rb=0x%02X, CTRL8 write=0x%02X rb=0x%02X, CTRL9 write=0x08 rb=0x%02X",
		ctrl6_val, ctrl6_readback, ctrl7_readback, ctrl8_val, ctrl8_readback, ctrl9_readback);

	if (err)
		LOG_ERR("Communication error during FS configuration");

	last_accel_odr = 0xff; // reset last odr to force update
	last_gyro_odr = 0xff; // reset last odr to force update
	// Re-enable SHUB_PU_EN if sensor hub (ext interface) was configured during scan.
	// The pre-init shutdown reset clears IF_CFG, so restore auxiliary I2C pull-ups here.
	uint8_t if_cfg = 0x18; // INT H_LACTIVE active low, PP_OD open-drain
	if (sensor_interface_ext_get() != NULL)
		if_cfg |= 0x40; // SHUB_PU_EN: enable internal pull-up for auxiliary I2C
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_IF_CFG, if_cfg);

	// Read internal frequency calibration
	int8_t internal_freq_fine;
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_INTERNAL_FREQ_FINE, &internal_freq_fine); // affects ODR
	freq_scale = 1.0f + 0.0013f * (float)internal_freq_fine;
	LOG_INF("INTERNAL_FREQ_FINE = %d, freq_scale = %.6f", internal_freq_fine, (double)freq_scale);

	// Update to target ODR
	err |= lsm_update_odr(accel_time, gyro_time, accel_actual_time, gyro_actual_time);

	// Enable FIFO in continuous mode
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FIFO_CTRL4, LSM6DSV_FIFO_MODE_CONTINUOUS);
	lsm_unknown_tag_count = 0;

	// Read back to verify FIFO mode
	uint8_t fifo_ctrl4_readback = 0;
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FIFO_CTRL4, &fifo_ctrl4_readback);
	LOG_INF("FIFO_CTRL4 write=0x%02X readback=0x%02X", LSM6DSV_FIFO_MODE_CONTINUOUS, fifo_ctrl4_readback);

	if (err)
		LOG_ERR("Communication error during initialization");
	else
		LOG_INF("LSM6DSV initialization complete");
	return (err < 0 ? err : 0);
}

void lsm_shutdown(void)
{
	ext_continuous_active = false;
	lsm_unknown_tag_count = 0;
	last_accel_odr = 0xff; // reset last odr
	last_gyro_odr = 0xff; // reset last odr
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL3, 0x01); // SW_RESET
	k_msleep(2); // Wait for reset to complete before the next init path continues
	if (err)
		LOG_ERR("Communication error");
}

void lsm_update_fs(float accel_range, float gyro_range, float *accel_actual_range, float *gyro_actual_range)
{
	if (accel_range > 8)
	{
		accel_fs = FS_XL_16G;
		accel_range = 16;
	}
	else if (accel_range > 4)
	{
		accel_fs = FS_XL_8G;
		accel_range = 8;
	}
	else if (accel_range > 2)
	{
		accel_fs = FS_XL_4G;
		accel_range = 4;
	}
	else
	{
		accel_fs = FS_XL_2G;
		accel_range = 2;
	}

	if (gyro_range > 2000)
	{
		gyro_fs = FS_G_4000DPS;
		gyro_range = 4000;
	}
	else if (gyro_range > 1000)
	{
		gyro_fs = FS_G_2000DPS;
		gyro_range = 2000;
	}
	else if (gyro_range > 500)
	{
		gyro_fs = FS_G_1000DPS;
		gyro_range = 1000;
	}
	else if (gyro_range > 250)
	{
		gyro_fs = FS_G_500DPS;
		gyro_range = 500;
	}
	else if (gyro_range > 125)
	{
		gyro_fs = FS_G_250DPS;
		gyro_range = 250;
	}
	else
	{
		gyro_fs = FS_G_125DPS;
		gyro_range = 125;
	}

	accel_sensitivity = accel_range / 32768.0f;
	gyro_sensitivity = 35.0f * gyro_range / 1000000.0f;

	*accel_actual_range = accel_range;
	*gyro_actual_range = gyro_range;
}

int lsm_update_odr(float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	int ODR;
	uint8_t OP_MODE_XL;
	uint8_t OP_MODE_G;
	uint8_t ODR_XL = ODR_OFF;
	uint8_t ODR_G = ODR_OFF;

	// Calculate accel
	// Note: freq_scale adjusts the actual output rate (e.g., 960Hz * 0.96 = ~923Hz)
	// but does NOT affect ODR register selection - we select based on nominal rate
	if (accel_time <= 0 || accel_time == INFINITY) // off, standby interpreted as off
	{
		OP_MODE_XL = OP_MODE_XL_HP;
		accel_time = 0;
	}
	else
	{
		OP_MODE_XL = OP_MODE_XL_HP;
		ODR = 1 / accel_time;
		for (int i = 0; i < ARRAY_SIZE(odr_rates); i++)
		{
			if (i + 1 < ARRAY_SIZE(odr_rates) && ODR <= odr_rates[i + 1])
				continue;
			ODR_XL = odr_values[i];
			accel_time = 1.0f / odr_rates[i];
			break;
		}
	}
	accel_time /= freq_scale; // scale by internal freq adjustment

	// Calculate gyro
	// Note: freq_scale adjusts the actual output rate but does NOT affect ODR register selection
	if (gyro_time <= 0) // off
	{
		OP_MODE_G = OP_MODE_G_HP;
		gyro_time = 0;
	}
	else if (gyro_time == INFINITY) // sleep
	{
		OP_MODE_G = OP_MODE_G_SLEEP;
		ODR_G = last_gyro_odr; // using last ODR
		gyro_time = 0;
	}
	else
	{
		OP_MODE_G = OP_MODE_G_HP;
		ODR = 1 / gyro_time;
		for (int i = 0; i < ARRAY_SIZE(odr_rates); i++)
		{
			if (i + 1 < ARRAY_SIZE(odr_rates) && ODR <= odr_rates[i + 1])
				continue;
			ODR_G = odr_values[i];
			gyro_time = 1.0f / odr_rates[i];
			break;
		}
	}
	gyro_time /= freq_scale; // scale by internal freq adjustment

	if (last_accel_mode == OP_MODE_XL && last_gyro_mode == OP_MODE_G && last_accel_odr == ODR_XL && last_gyro_odr == ODR_G) // if both were already configured
		return 1;

	last_accel_mode = OP_MODE_XL;
	last_gyro_mode = OP_MODE_G;
	last_accel_odr = ODR_XL;
	last_gyro_odr = ODR_G;

	uint8_t ctrl1_config = OP_MODE_XL << 4 | ODR_XL;
	uint8_t ctrl2_config = OP_MODE_G << 4 | ODR_G;
	uint8_t fifo_ctrl3_config = ODR_XL | (ODR_G << 4);

	LOG_INF("CTRL1 write=0x%02X (OP_MODE_XL=%d, ODR_XL=%d), CTRL2 write=0x%02X (OP_MODE_G=%d, ODR_G=%d)",
		ctrl1_config, OP_MODE_XL, ODR_XL, ctrl2_config, OP_MODE_G, ODR_G);

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL1, ctrl1_config); // set accel ODR and mode
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL2, ctrl2_config); // set gyro ODR and mode
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FIFO_CTRL3, fifo_ctrl3_config); // set accel and gyro batch rate

	// Read back to verify
	uint8_t ctrl1_readback = 0, ctrl2_readback = 0, fifo_ctrl3_readback = 0;
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL1, &ctrl1_readback);
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL2, &ctrl2_readback);
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FIFO_CTRL3, &fifo_ctrl3_readback);
	LOG_INF("CTRL1 readback=0x%02X, CTRL2 readback=0x%02X, FIFO_CTRL3 readback=0x%02X",
		ctrl1_readback, ctrl2_readback, fifo_ctrl3_readback);

	if (err)
		LOG_ERR("Communication error");

	*accel_actual_time = accel_time;
	*gyro_actual_time = gyro_time;

	return 0;
}

uint16_t lsm_fifo_read(uint8_t *data, uint16_t len)
{
	int err = 0;

	// Read FIFO status registers (STATUS1 + STATUS2) to get word count and status flags
	uint8_t rawStatus[2];
	err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FIFO_STATUS1, rawStatus, 2);
	if (err)
	{
		LOG_ERR("Failed to read FIFO status");
		return 0;
	}

	// Parse FIFO word count (10-bit field: bits 1:0 of STATUS2 and all 8 bits of STATUS1)
	uint16_t count = (uint16_t)((rawStatus[1] & LSM6DSV_FIFO_DIFF_8) << 8 | rawStatus[0]);

	// Early return if FIFO is empty
	if (count == 0)
		return 0;

	// Check for FIFO overflow (latched status, cleared on read)
	if (rawStatus[1] & LSM6DSV_FIFO_OVR_LATCHED)
	{
		LOG_WRN("FIFO overflow detected - data may be lost");
		lsm_fifo_resync("overflow detected");
		return 0;
	}

	// Limit read to available buffer space
	uint16_t limit = len / PACKET_SIZE;
	if (count > limit)
	{
		LOG_WRN("FIFO buffer limit exceeded: count=%u limit=%u, resyncing FIFO", count, limit);
		lsm_fifo_resync("software buffer limit exceeded");
		return 0;
	}

	// Batch read all packets in one SPI transaction
	// LSM6DSV supports continuous read from FIFO_DATA_OUT_TAG register
	uint16_t bytes_to_read = count * PACKET_SIZE;
	err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FIFO_DATA_OUT_TAG, data, bytes_to_read);
	if (err)
	{
		LOG_ERR("Failed to read FIFO data");
		lsm_fifo_resync("FIFO data read failed");
		return 0;
	}

	lsm_unknown_tag_count = 0;
	return count;
}

int lsm_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3])
{
	index *= PACKET_SIZE;
	uint8_t tag = data[index] >> 3;  // TAG_SENSOR[4:0]

	switch (tag)
	{
	case LSM6DSV_TAG_ACCEL_NC:
		// Parse accelerometer data
		// LSM6DSV16B (0x71): Z, Y, X order in FIFO
		// LSM6DSV (0x70): X, Y, Z order in FIFO
		if (chip_who_am_i == 0x71)
		{
			// LSM6DSV16B: Read Z, Y, X and store as X, Y, Z
			a[2] = (int16_t)((((uint16_t)data[index + 2]) << 8) | data[index + 1]); // Z
			a[1] = (int16_t)((((uint16_t)data[index + 4]) << 8) | data[index + 3]); // Y
			a[0] = (int16_t)((((uint16_t)data[index + 6]) << 8) | data[index + 5]); // X
		}
		else
		{
			// LSM6DSV: Normal X, Y, Z order
			for (int i = 0; i < 3; i++)
			{
				a[i] = (int16_t)((((uint16_t)data[index + 2 + (i * 2)]) << 8) | data[index + 1 + (i * 2)]);
			}
		}
		for (int i = 0; i < 3; i++)
			a[i] *= accel_sensitivity;
		return 0;

	case LSM6DSV_TAG_GYRO_NC:
		// Parse gyroscope data
		// Both LSM6DSV and LSM6DSV16B: X, Y, Z order in FIFO
		for (int i = 0; i < 3; i++)
		{
			g[i] = (int16_t)((((uint16_t)data[index + 2 + (i * 2)]) << 8) | data[index + 1 + (i * 2)]);
			g[i] *= gyro_sensitivity;
		}
		return 0;

	case LSM6DSV_TAG_FIFO_EMPTY:
		// FIFO empty marker - skip silently
		lsm_unknown_tag_count = 0;
		return 1;

	case LSM6DSV_TAG_TEMP:
	case LSM6DSV_TAG_TIMESTAMP:
	case LSM6DSV_TAG_CFG_CHANGE:
		// Known non-sensor data tags - skip
		lsm_unknown_tag_count = 0;
		return 1;

	default:
		// Unknown or unsupported tag (compressed data, sensor hub, SFLP, etc.)
		lsm_unknown_tag_count++;
		LOG_DBG("Skipping FIFO packet with tag 0x%02X (unknown_count=%u)", tag, lsm_unknown_tag_count);
		if (lsm_unknown_tag_count >= LSM6DSV_UNKNOWN_TAG_RESYNC_THRESHOLD)
			lsm_fifo_resync("consecutive unknown FIFO tags");
		return 1;
	}
}

void lsm_accel_read(float a[3])
{
	uint8_t rawAccel[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_OUTX_L_A, &rawAccel[0], 6);
	if (err)
		LOG_ERR("Communication error");

	// LSM6DSV16B (0x71): Z, Y, X order in registers (reading from OUTX_L_A gets Z, Y, X)
	// LSM6DSV (0x70): X, Y, Z order in registers
	if (chip_who_am_i == 0x71)
	{
		// LSM6DSV16B: Read Z, Y, X and store as X, Y, Z
		a[2] = (int16_t)((((uint16_t)rawAccel[1]) << 8) | rawAccel[0]); // Z
		a[1] = (int16_t)((((uint16_t)rawAccel[3]) << 8) | rawAccel[2]); // Y
		a[0] = (int16_t)((((uint16_t)rawAccel[5]) << 8) | rawAccel[4]); // X
	}
	else
	{
		// LSM6DSV: Normal X, Y, Z order
		for (int i = 0; i < 3; i++) // x, y, z
		{
			a[i] = (int16_t)((((uint16_t)rawAccel[1 + (i * 2)]) << 8) | rawAccel[i * 2]);
		}
	}

	for (int i = 0; i < 3; i++)
		a[i] *= accel_sensitivity;
}

void lsm_gyro_read(float g[3])
{
	uint8_t rawGyro[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_OUTX_L_G, &rawGyro[0], 6);
	if (err)
		LOG_ERR("Communication error");
	for (int i = 0; i < 3; i++) // x, y, z
	{
		g[i] = (int16_t)((((uint16_t)rawGyro[1 + (i * 2)]) << 8) | rawGyro[i * 2]);
		g[i] *= gyro_sensitivity;
	}
}

float lsm_temp_read(void)
{
	uint8_t rawTemp[2];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_OUT_TEMP_L, &rawTemp[0], 2);
	if (err)
		LOG_ERR("Communication error");
	// TSen Temperature sensitivity 256 LSB/°C
	// The output of the temperature sensor is 0 LSB (typ.) at 25°C
	float temp = (int16_t)((((uint16_t)rawTemp[1]) << 8) | rawTemp[0]);
	temp /= 256;
	temp += 25;
	return temp;
}

uint8_t lsm_setup_DRDY(uint16_t threshold)
{
	uint8_t buf[2];
	buf[0] = ((threshold >> 8) & 0x03) | (last_gyro_odr > last_accel_odr ? 0x20 : 0x00); // use gyro for BDR if gyro rate is higher // NOTE: using 0x03 for DSV, but DSO allows 0x07
	buf[1] = threshold & 0xFF;
	int err = ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_COUNTER_BDR_REG1, buf, 2);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_INT1_CTRL, 0x40); // COUNTER_BDR interrupt
	if (err)
		LOG_ERR("Communication error");
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
}

uint8_t lsm_setup_WOM(void)
{ // TODO: should be off by the time WOM will be setup
//	ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL1, ODR_OFF); // set accel off
//	ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL2, ODR_OFF); // set gyro off

	// Enter a clean WOM state explicitly. This path is exercised right before system-off,
	// and if FIFO / BDR / sensor-hub state is left running, wake-up can boot into a state
	// where FIFO no longer starts producing packets reliably.
	lsm_ext_stop_continuous();
	lsm_unknown_tag_count = 0;
	last_accel_mode = 0xff;
	last_gyro_mode = 0xff;
	last_accel_odr = 0xff;
	last_gyro_odr = 0xff;

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_INT1_CTRL, 0x00); // disable FIFO/BDR interrupt routing
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MD1_CFG, 0x00); // clear previous wake routing first
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNCTIONS_ENABLE, 0x00); // clear embedded interrupt enables
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FIFO_CTRL3, 0x00); // stop FIFO batching for accel/gyro
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FIFO_CTRL4, LSM6DSV_FIFO_MODE_BYPASS); // flush FIFO / disable streaming
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL2, OP_MODE_G_HP << 4 | ODR_OFF); // gyro fully off in WOM
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL8, 0xE0 | FS_XL_8G); // set accel FS, set HP_LPF2_XL_BW to lowest bandwidth, enable HP_REF_MODE (set HP_LPF2_XL_BW)
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL1, OP_MODE_XL_LP1 << 4 | ODR_240Hz); // set accel low power mode 1, set accel ODR (enable accel)
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL9, 0x50); // enable HP_REF_MODE (set HP_REF_MODE_XL and HP_SLOPE_XL_EN)
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_TAP_CFG0, 0x10); // set SLOPE_FDS
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_WAKE_UP_THS, 0x04); // set threshold, 4 * 7.8125 mg is ~31.25 mg
	k_msleep(11); // need to wait for accel to settle

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNCTIONS_ENABLE, 0x80); // enable interrupts
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MD1_CFG, 0x20); // route wake-up to INT1
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_IF_CFG, 0x18); // INT H_LACTIVE active low, PP_OD open-drain
	if (err)
		LOG_ERR("Communication error");
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
}

int lsm_ext_setup(void)
{
	// Reset I2C master state for clean device discovery (important after WOM wakeup)
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x40); // switch to sensor hub registers
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x00); // disable I2C master
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00); // switch to normal registers
	k_usleep(350);
	// Sensor hub requires the internal oscillator to be running.
	// Start accelerometer at high ODR for fast I2C master one-shot transactions during scanning.
	// Each one-shot is triggered by accel data-ready, so higher ODR = faster scans.
	// lsm_init() will reconfigure ODR for normal operation.
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL1, (OP_MODE_XL_HP << 4) | ODR_480Hz);
	k_msleep(5); // wait for oscillator startup
	// enable internal pull-up for auxiliary I2C
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_IF_CFG, 0x58); // SHUB_PU_EN, INT H_LACTIVE active low, PP_OD open-drain
	if (err)
		LOG_ERR("Communication error");
	// Reset to scanning mode for clean device discovery
	ext_continuous_active = false;
	ext_scanning_mode = true;
	sensor_interface_ext_configure(&sensor_ext_lsm6dsv);
	return 0;
}

int lsm_ext_passthrough(bool passthrough)
{
	int err = 0;
	if (passthrough)
	{
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x40); // switch to sensor hub registers
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x10); // passthrough on
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00); // switch to normal registers
	}
	else
	{
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x40); // switch to sensor hub registers
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x00); // passthrough off
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00); // switch to normal registers
	}
	if (err)
		LOG_ERR("Communication error");
	return 0;
}

/** Stop continuous sensor hub reading if active */
static void lsm_ext_stop_continuous(void)
{
	if (!ext_continuous_active)
		return;
	ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x40);
	ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x00);
	ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00);
	k_usleep(350);
	ext_continuous_active = false;
}

/** Start continuous sensor hub reading for addr/sub_addr/num_bytes */
static int lsm_ext_start_continuous(uint8_t addr, uint8_t sub_addr, uint8_t num_bytes)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x40);
	uint8_t slv0[3] = {(addr << 1) | 0x01, sub_addr, 0xA0 | num_bytes}; // rw_0=1 for read, SHUB_ODR = 240Hz, num_bytes to read
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SLV0_ADD, slv0, 3);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x04); // MASTER_ON only (continuous)
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00);
	if (!err) {
		ext_continuous_active = true;
		ext_cont_addr = addr;
		ext_cont_sub = sub_addr;
		ext_cont_len = num_bytes;
		LOG_DBG("Sensor hub continuous read started (addr=0x%02X, reg=0x%02X, len=%u)", addr, sub_addr, num_bytes);
	}
	return err;
}

int lsm_ext_write(const uint8_t addr, const uint8_t *buf, uint32_t num_bytes)
{
	if (num_bytes != 2)
	{
		LOG_ERR("Unsupported write");
		return -1;
	}
	// Stop continuous mode before writing (I2C master must be reconfigured)
	lsm_ext_stop_continuous();
	// Configure transaction and begin one-shot (AN5922, page 80, One-shot write routine)
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x40); // switch to sensor hub registers
	// SLV0_ADD format: bits[7:1]=slave_addr[6:0], bit0=rw_0 (0=write, 1=read)
	uint8_t slv0[3] = {(addr << 1) | 0x00, buf[0], 0xA0 | 0x00}; // rw_0=0 for write, SHUB_ODR = 240Hz, reading no bytes
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SLV0_ADD, slv0, 3);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_DATAWRITE_SLV0, buf[1]);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x44); // WRITE_ONCE(0x40) + MASTER_ON(0x04)
	// Wait for transaction: write is triggered on accel XLDA, needs up to 67ms at 15Hz ODR
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00); // switch to normal registers
	uint8_t tmp;
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_OUTX_H_A, &tmp); // clear current XLDA
	uint8_t status = 0;
	int64_t timeout = k_uptime_get() + 10;
	while (!(status & 0x01) && k_uptime_get() < timeout) // wait for new XLDA
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_STATUS_REG, &status);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x40); // switch to sensor hub registers
	status = 0;
	timeout = k_uptime_get() + 10;
	while (!(status & 0x80) && k_uptime_get() < timeout) // WR_ONCE_DONE
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_STATUS_MASTER, &status);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x00); // disable I2C master
	k_usleep(350);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00); // switch to normal registers
	if (status & 0x04) // SLAVE0_NACK
	{
		LOG_DBG("Ext I2C write NACK from address 0x%02X", addr);
		return -1;
	}
	if (~status & 0x80)
	{
		LOG_ERR("Write timeout");
		return -1;
	}
	return err;
}

int lsm_ext_write_read(const uint8_t addr, const void *write_buf, size_t num_write, void *read_buf, size_t num_read)
{
	if (num_write != 1 || num_read < 1 || num_read > 8)
	{
		LOG_ERR("Unsupported write_read");
		return -1;
	}

	uint8_t sub_addr = ((const uint8_t *)write_buf)[0];

	// Fast path: if continuous mode is active and request matches, read SENSOR_HUB directly
	// This avoids the ~8ms one-shot cycle (waiting for XLDA) and takes only ~20us via SPI
	if (ext_continuous_active && addr == ext_cont_addr &&
	    sub_addr == ext_cont_sub && num_read == ext_cont_len)
	{
		int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x40);
		err |= ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SENSOR_HUB_1, read_buf, num_read);
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00);
		return err;
	}

	// If continuous mode was active for different params, stop it first
	lsm_ext_stop_continuous();

	// One-shot read (AN5922, page 79, One-shot read routine)
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x40); // switch to sensor hub registers
	// SLV0_ADD format: bits[7:1]=slave_addr[6:0], bit0=rw_0 (0=write, 1=read)
	uint8_t slv0[3] = {(addr << 1) | 0x01, sub_addr, 0xA0 | num_read}; // rw_0=1 for read, SHUB_ODR = 240Hz, reading num_read bytes
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SLV0_ADD, slv0, 3);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x44); // WRITE_ONCE(0x40) + MASTER_ON(0x04)
	// Wait for transaction (AN5922 One-shot read routine):
	// START_CONFIG=0: sensor hub triggers on accel/gyro data-ready
	// lsm_ext_setup() ensures accel is running at >=15Hz before scan
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00); // switch to normal registers
	uint8_t tmp;
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_OUTX_H_A, &tmp); // clear current XLDA by reading accel data
	uint8_t status = 0;
	int64_t timeout = k_uptime_get() + 10; // 10ms timeout
	while (!(status & 0x01) && k_uptime_get() < timeout) // wait for new XLDA (accelerometer data ready)
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_STATUS_REG, &status);
	status = 0;
	timeout = k_uptime_get() + 10;
	while (!(status & 0x01) && k_uptime_get() < timeout) // SENS_HUB_ENDOP (bit 0)
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_STATUS_MASTER_MAINPAGE, &status);
	// Read data
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x40); // switch to sensor hub registers
	// Check for NACK and timeout before reading data
	uint8_t master_status = 0;
	ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_STATUS_MASTER, &master_status);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x00); // disable I2C master
	k_usleep(350);
	if ((master_status & 0x04) || !(status & 0x01)) // SLAVE0_NACK or   timeout
	{
		if (master_status & 0x04)
			LOG_DBG("Ext I2C NACK from address 0x%02X", addr);
		else
			LOG_DBG("Ext I2C read timeout for address 0x%02X", addr);
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00);
		memset(read_buf, 0, num_read);
		return -1;
	}
	err |= ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SENSOR_HUB_1, read_buf, num_read);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00); // switch to normal registers

	// In operational mode, start continuous immediately for fast subsequent reads.
	// In scanning mode, skip continuous to avoid start/stop overhead per address.
	if (!err && !ext_scanning_mode)
		lsm_ext_start_continuous(addr, sub_addr, num_read);

	return err;
}

const sensor_imu_t sensor_imu_lsm6dsv = {
	lsm_init,
	lsm_shutdown,

	lsm_update_fs,
	lsm_update_odr,

	lsm_fifo_read,
	lsm_fifo_process,
	lsm_accel_read,
	lsm_gyro_read,
	lsm_temp_read,

	lsm_setup_DRDY,
	lsm_setup_WOM,

	lsm_ext_setup,
	lsm_ext_passthrough
};

const sensor_ext_ssi_t sensor_ext_lsm6dsv = {
	lsm_ext_write,
	lsm_ext_write_read,
	8
};
