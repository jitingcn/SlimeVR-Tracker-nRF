#include <errno.h>
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
float gyro_sensitivity = 0.070f;            // Default 2000dps (FS = ±2000 dps: 70 mdps/LSB)

static uint8_t accel_fs = FS_XL_16G;
static uint8_t gyro_fs = FS_G_2000DPS;

static const float odr_rates[]
	= {7680.0f, 3840.0f, 1920.0f, 960.0f, 480.0f, 240.0f, 120.0f, 60.0f, 30.0f, 15.0f, 7.5f, 1.875f};
static const uint8_t odr_values[]
	= {ODR_7_68kHz,
	   ODR_3_84kHz,
	   ODR_1_92kHz,
	   ODR_960Hz,
	   ODR_480Hz,
	   ODR_240Hz,
	   ODR_120Hz,
	   ODR_60Hz,
	   ODR_30Hz,
	   ODR_15Hz,
	   ODR_7_5Hz,
	   ODR_1_875Hz};

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
	if (err) {
		LOG_ERR("FIFO resync failed");
	}
	return err;
}

// One unread sensor-hub acquisition epoch may be prefetched. Hardware runs
// periodically until consumption, but a returned epoch is never reused.
static bool ext_continuous_active = false;
static uint8_t ext_cont_addr = 0;
static uint8_t ext_cont_sub = 0;
static uint8_t ext_cont_len = 0;
static bool ext_prefetch_enabled = true;
static int lsm_ext_stop_continuous(void);

// Scanning mode: when true, synchronous reads never start continuous mode.
// Set during ext_setup() for device scanning, cleared by lsm_init() for normal operation.
static bool ext_scanning_mode = true;

#define LSM6DSV_SHUB_XLDA_TIMEOUT_MS 80
#define LSM6DSV_SHUB_OP_TIMEOUT_MS 20

int lsm_init(float clock_rate, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	(void)clock_rate; // This device uses its internal clock.
	// setup interface for SPI
	LOG_INF("Initializing LSM6DSV...");
	sensor_interface_spi_configure(SENSOR_INTERFACE_DEV_IMU, MHZ(10), 0);

	// sensor_init() already issued shutdown/reset before calling init.
	// Continue from the post-reset state and rebuild runtime configuration below.
	int err = lsm_ext_stop_continuous();
	if (err)
		return err;
	ext_scanning_mode = false; // Keep the magnetometer's prefetch policy across IMU init.

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

	// Leave WOM without a soft reset. Clear wake routing and FIFO state, then
	// start accel/gyro at the target ODR while the remaining configuration runs.
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_INT1_CTRL, 0x00);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MD1_CFG, 0x00);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNCTIONS_ENABLE, 0x00);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FIFO_CTRL3, 0x00);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FIFO_CTRL4, LSM6DSV_FIFO_MODE_BYPASS);

	// Re-enable SHUB_PU_EN if sensor hub (ext interface) was configured during scan.
	uint8_t if_cfg = 0x18; // INT H_LACTIVE active low, PP_OD open-drain
	if (sensor_interface_ext_get() != NULL) {
		if_cfg |= 0x40; // SHUB_PU_EN: enable internal pull-up for auxiliary I2C
	}
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_IF_CFG, if_cfg);

	uint8_t internal_freq_raw = 0;
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_INTERNAL_FREQ_FINE, &internal_freq_raw); // affects ODR
	int8_t internal_freq_fine = (int8_t)internal_freq_raw;
	freq_scale = 1.0f + 0.0013f * (float)internal_freq_fine;
	LOG_INF("INTERNAL_FREQ_FINE = %d, freq_scale = %.6f", internal_freq_fine, (double)freq_scale);

	last_accel_mode = 0xff;
	last_gyro_mode = 0xff;
	last_accel_odr = 0xff;
	last_gyro_odr = 0xff;
	err |= lsm_update_odr(accel_time, gyro_time, accel_actual_time, gyro_actual_time);

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
	LOG_INF(
		"CTRL6 write=0x%02X rb=0x%02X, CTRL7 write=0x01 rb=0x%02X, CTRL8 write=0x%02X rb=0x%02X, CTRL9 write=0x08 "
		"rb=0x%02X",
		ctrl6_val,
		ctrl6_readback,
		ctrl7_readback,
		ctrl8_val,
		ctrl8_readback,
		ctrl9_readback
	);

	if (err) {
		LOG_ERR("Communication error during FS configuration");
	}

	// Enable FIFO in continuous mode
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FIFO_CTRL4, LSM6DSV_FIFO_MODE_CONTINUOUS);
	lsm_unknown_tag_count = 0;

	// Read back to verify FIFO mode
	uint8_t fifo_ctrl4_readback = 0;
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FIFO_CTRL4, &fifo_ctrl4_readback);
	LOG_INF("FIFO_CTRL4 write=0x%02X readback=0x%02X", LSM6DSV_FIFO_MODE_CONTINUOUS, fifo_ctrl4_readback);

	if (err) {
		LOG_ERR("Communication error during initialization");
	} else {
		LOG_INF("LSM6DSV initialization complete");
	}
	return (err < 0 ? err : 0);
}

void lsm_shutdown(void)
{
	int err = lsm_ext_stop_continuous();
	lsm_unknown_tag_count = 0;
	last_accel_odr = 0xff;                                                       // reset last odr
	last_gyro_odr = 0xff;                                                        // reset last odr
	// A failed stop may have left the sensor-hub page selected. Reset recovery
	// must still be attempted, but CTRL3 is safe only after a checked page select.
	int reset_err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00);
	if (!reset_err)
		reset_err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL3, 0x01); // SW_RESET
	if (!err)
		err = reset_err;
	k_msleep(2); // Wait for reset to complete before the next init path continues
	if (err) {
		LOG_ERR("Communication error");
	}
}

void lsm_update_fs(float accel_range, float gyro_range, float *accel_actual_range, float *gyro_actual_range)
{
	if (accel_range > 8) {
		accel_fs = FS_XL_16G;
		accel_range = 16;
	} else if (accel_range > 4) {
		accel_fs = FS_XL_8G;
		accel_range = 8;
	} else if (accel_range > 2) {
		accel_fs = FS_XL_4G;
		accel_range = 4;
	} else {
		accel_fs = FS_XL_2G;
		accel_range = 2;
	}

	if (gyro_range > 2000) {
		gyro_fs = FS_G_4000DPS;
		gyro_range = 4000;
	} else if (gyro_range > 1000) {
		gyro_fs = FS_G_2000DPS;
		gyro_range = 2000;
	} else if (gyro_range > 500) {
		gyro_fs = FS_G_1000DPS;
		gyro_range = 1000;
	} else if (gyro_range > 250) {
		gyro_fs = FS_G_500DPS;
		gyro_range = 500;
	} else if (gyro_range > 125) {
		gyro_fs = FS_G_250DPS;
		gyro_range = 250;
	} else {
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
	} else {
		OP_MODE_XL = OP_MODE_XL_HP;
		ODR = 1 / accel_time;
		for (size_t i = 0; i < ARRAY_SIZE(odr_rates); i++) {
			if (i + 1 < ARRAY_SIZE(odr_rates) && ODR <= odr_rates[i + 1]) {
				continue;
			}
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
	} else if (gyro_time == INFINITY) // sleep
	{
		OP_MODE_G = OP_MODE_G_SLEEP;
		ODR_G = last_gyro_odr; // using last ODR
		gyro_time = 0;
	} else {
		OP_MODE_G = OP_MODE_G_HP;
		ODR = 1 / gyro_time;
		for (size_t i = 0; i < ARRAY_SIZE(odr_rates); i++) {
			if (i + 1 < ARRAY_SIZE(odr_rates) && ODR <= odr_rates[i + 1]) {
				continue;
			}
			ODR_G = odr_values[i];
			gyro_time = 1.0f / odr_rates[i];
			break;
		}
	}
	gyro_time /= freq_scale; // scale by internal freq adjustment

	if (last_accel_mode == OP_MODE_XL && last_gyro_mode == OP_MODE_G && last_accel_odr == ODR_XL
		&& last_gyro_odr == ODR_G) {
		*accel_actual_time = accel_time;
		*gyro_actual_time = gyro_time;
		return 0; /* already configured — success for err|= callers */
	}

	uint8_t ctrl1_config = OP_MODE_XL << 4 | ODR_XL;
	uint8_t ctrl2_config = OP_MODE_G << 4 | ODR_G;
	uint8_t fifo_ctrl3_config = ODR_XL | (ODR_G << 4);

	LOG_INF(
		"CTRL1 write=0x%02X (OP_MODE_XL=%d, ODR_XL=%d), CTRL2 write=0x%02X (OP_MODE_G=%d, ODR_G=%d)",
		ctrl1_config,
		OP_MODE_XL,
		ODR_XL,
		ctrl2_config,
		OP_MODE_G,
		ODR_G
	);

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL1, ctrl1_config); // set accel ODR and mode
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL2, ctrl2_config);    // set gyro ODR and mode
	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		LSM6DSV_FIFO_CTRL3,
		fifo_ctrl3_config
	); // set accel and gyro batch rate

	// Read back to verify
	uint8_t ctrl1_readback = 0, ctrl2_readback = 0, fifo_ctrl3_readback = 0;
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL1, &ctrl1_readback);
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL2, &ctrl2_readback);
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FIFO_CTRL3, &fifo_ctrl3_readback);
	LOG_INF(
		"CTRL1 readback=0x%02X, CTRL2 readback=0x%02X, FIFO_CTRL3 readback=0x%02X",
		ctrl1_readback,
		ctrl2_readback,
		fifo_ctrl3_readback
	);

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

uint16_t lsm_fifo_read(uint8_t *data, uint16_t len)
{
	int err = 0;

	// Read FIFO status registers (STATUS1 + STATUS2) to get word count and status flags
	uint8_t rawStatus[2];
	err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FIFO_STATUS1, rawStatus, 2);
	if (err) {
		LOG_ERR("Failed to read FIFO status");
		return 0;
	}

	// Parse FIFO word count (9-bit field: bit 0 of STATUS2 and all 8 bits of STATUS1)
	uint16_t count = (uint16_t)((rawStatus[1] & LSM6DSV_FIFO_DIFF_8) << 8 | rawStatus[0]);

	// Early return if FIFO is empty
	if (count == 0) {
		return 0;
	}

	// Check for FIFO overflow (latched status, cleared on read)
	if (rawStatus[1] & LSM6DSV_FIFO_OVR_LATCHED) {
		LOG_WRN("FIFO overflow detected - data may be lost");
		lsm_fifo_resync("overflow detected");
		return 0;
	}

	// Limit read to available buffer space; drain what fits instead of
	// resyncing and dropping the whole batch (a corrupted DIFF_FIFO or a
	// long preemption would otherwise cost 2x350us of resync sleeps).
	uint16_t limit = len / PACKET_SIZE;
	if (count > limit) {
		LOG_WRN("FIFO read buffer limit reached, %u packets dropped", count - limit);
		count = limit;
	}

	// Batch read all packets in one SPI transaction
	// LSM6DSV supports continuous read from FIFO_DATA_OUT_TAG register
	uint16_t bytes_to_read = count * PACKET_SIZE;
	err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FIFO_DATA_OUT_TAG, data, bytes_to_read);
	if (err) {
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
	uint8_t tag = data[index] >> 3; // TAG_SENSOR[4:0]

	switch (tag) {
	case LSM6DSV_TAG_ACCEL_NC:
		// Parse accelerometer data
		// LSM6DSV16B (0x71): Z, Y, X order in FIFO
		// LSM6DSV (0x70): X, Y, Z order in FIFO
		if (chip_who_am_i == 0x71) {
			// LSM6DSV16B: Read Z, Y, X and store as X, Y, Z
			a[2] = (int16_t)((((uint16_t)data[index + 2]) << 8) | data[index + 1]); // Z
			a[1] = (int16_t)((((uint16_t)data[index + 4]) << 8) | data[index + 3]); // Y
			a[0] = (int16_t)((((uint16_t)data[index + 6]) << 8) | data[index + 5]); // X
		} else {
			// LSM6DSV: Normal X, Y, Z order
			for (int i = 0; i < 3; i++) {
				a[i] = (int16_t)((((uint16_t)data[index + 2 + (i * 2)]) << 8) | data[index + 1 + (i * 2)]);
			}
		}
		for (int i = 0; i < 3; i++) {
			a[i] *= accel_sensitivity;
		}
		return 0;

	case LSM6DSV_TAG_GYRO_NC:
		// Parse gyroscope data
		// Both LSM6DSV and LSM6DSV16B: X, Y, Z order in FIFO
		for (int i = 0; i < 3; i++) {
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
		if (lsm_unknown_tag_count >= LSM6DSV_UNKNOWN_TAG_RESYNC_THRESHOLD) {
			lsm_fifo_resync("consecutive unknown FIFO tags");
		}
		return 1;
	}
}

void lsm_accel_read(float a[3])
{
	uint8_t rawAccel[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_OUTX_L_A, &rawAccel[0], 6);
	if (err) {
		LOG_ERR("Communication error");
		memset(a, 0, 3 * sizeof(*a));
		return;
	}

	// LSM6DSV16B (0x71): Z, Y, X order in registers (reading from OUTX_L_A gets Z, Y, X)
	// LSM6DSV (0x70): X, Y, Z order in registers
	if (chip_who_am_i == 0x71) {
		// LSM6DSV16B: Read Z, Y, X and store as X, Y, Z
		a[2] = (int16_t)((((uint16_t)rawAccel[1]) << 8) | rawAccel[0]); // Z
		a[1] = (int16_t)((((uint16_t)rawAccel[3]) << 8) | rawAccel[2]); // Y
		a[0] = (int16_t)((((uint16_t)rawAccel[5]) << 8) | rawAccel[4]); // X
	} else {
		// LSM6DSV: Normal X, Y, Z order
		for (int i = 0; i < 3; i++) // x, y, z
		{
			a[i] = (int16_t)((((uint16_t)rawAccel[1 + (i * 2)]) << 8) | rawAccel[i * 2]);
		}
	}

	for (int i = 0; i < 3; i++) {
		a[i] *= accel_sensitivity;
	}
}

void lsm_gyro_read(float g[3])
{
	uint8_t rawGyro[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_OUTX_L_G, &rawGyro[0], 6);
	if (err) {
		LOG_ERR("Communication error");
		memset(g, 0, 3 * sizeof(*g));
		return;
	}
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
	if (err) {
		LOG_ERR("Communication error");
		return NAN;
	}
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
	buf[0] = ((threshold >> 8) & 0x03)
		   | (last_gyro_odr > last_accel_odr
				  ? 0x20
				  : 0x00); // use gyro for BDR if gyro rate is higher // NOTE: using 0x03 for DSV, but DSO allows 0x07
	buf[1] = threshold & 0xFF;
	int err = ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_COUNTER_BDR_REG1, buf, 2);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_INT1_CTRL, 0x40); // COUNTER_BDR interrupt
	if (err) {
		LOG_ERR("Communication error");
	}
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
}

uint8_t lsm_setup_WOM(void)
{   // TODO: should be off by the time WOM will be setup
	//	ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL1, ODR_OFF); // set accel off
	//	ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL2, ODR_OFF); // set gyro off

	// Enter a clean WOM state explicitly. This path is exercised right before system-off,
	// and if FIFO / BDR / sensor-hub state is left running, wake-up can boot into a state
	// where FIFO no longer starts producing packets reliably.
	int err = lsm_ext_stop_continuous();
	if (err) {
		LOG_ERR("Failed to stop sensor hub: %d", err);
		return 0xFF;
	}
	lsm_unknown_tag_count = 0;
	last_accel_mode = 0xff;
	last_gyro_mode = 0xff;
	last_accel_odr = 0xff;
	last_gyro_odr = 0xff;

	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_INT1_CTRL, 0x00); // disable FIFO/BDR interrupt routing
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MD1_CFG, 0x00);  // clear previous wake routing first
	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		LSM6DSV_FUNCTIONS_ENABLE,
		0x00
	);                                                                             // clear embedded interrupt enables
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FIFO_CTRL3, 0x00); // stop FIFO batching for accel/gyro
	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		LSM6DSV_FIFO_CTRL4,
		LSM6DSV_FIFO_MODE_BYPASS
	); // flush FIFO / disable streaming
	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		LSM6DSV_CTRL2,
		OP_MODE_G_HP << 4 | ODR_OFF
	); // gyro fully off in WOM
	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		LSM6DSV_CTRL8,
		0xE0 | FS_XL_8G
	); // set accel FS, set HP_LPF2_XL_BW to lowest bandwidth, enable HP_REF_MODE (set HP_LPF2_XL_BW)
	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		LSM6DSV_CTRL1,
		OP_MODE_XL_LP1 << 4 | ODR_240Hz
	); // set accel low power mode 1, set accel ODR (enable accel)
	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		LSM6DSV_CTRL9,
		0x50
	); // enable HP_REF_MODE (set HP_REF_MODE_XL and HP_SLOPE_XL_EN)
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_TAP_CFG0, 0x10); // set SLOPE_FDS
	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		LSM6DSV_WAKE_UP_THS,
		0x04
	);            // set threshold, 4 * 7.8125 mg is ~31.25 mg
	k_msleep(11); // need to wait for accel to settle

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNCTIONS_ENABLE, 0x80); // enable interrupts
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MD1_CFG, 0x20);          // route wake-up to INT1
	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		LSM6DSV_IF_CFG,
		0x18
	); // INT H_LACTIVE active low, PP_OD open-drain
	if (err) {
		LOG_ERR("Communication error");
	}
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
}

int lsm_ext_setup(enum sensor_ext_mode mode)
{
	if (mode != SENSOR_EXT_MODE_OFF && mode != SENSOR_EXT_MODE_I2C_PASSTHROUGH &&
	    mode != SENSOR_EXT_MODE_I2CM_PROXY)
		return -EINVAL;
	int err = lsm_ext_stop_continuous();
	if (err)
		return err;
	if (mode == SENSOR_EXT_MODE_OFF || mode == SENSOR_EXT_MODE_I2C_PASSTHROUGH) {
		err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, LSM6DSV_SHUB_REG_ACCESS);
		if (!err)
			err = ssi_reg_write_byte(
				SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG,
				mode == SENSOR_EXT_MODE_I2C_PASSTHROUGH ? 0x10 : 0x00);
		int restore_err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00);
		if (!err)
			err = restore_err;
		if (err)
			lsm_ext_stop_continuous();
		return err;
	}

	// Synchronous sensor-hub transactions need an accel data-ready trigger.
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL1, (OP_MODE_XL_HP << 4) | ODR_480Hz);
	if (err)
		return err;
	k_msleep(5);
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_IF_CFG, 0x58);
	if (err) {
		LOG_ERR("Communication error");
		return err;
	}

	ext_prefetch_enabled = true;
	ext_scanning_mode = true;
	sensor_interface_ext_configure(&sensor_ext_lsm6dsv);
	return 0;
}

/** Quiesce the hub, including a transaction left running by a failed transfer. */
static int lsm_ext_stop_continuous(void)
{
	// Invalidate before any fallible I/O: the physical master may stop even if
	// restoring the main page fails, or its state may be unknown after an error.
	ext_continuous_active = false;
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, LSM6DSV_SHUB_REG_ACCESS);
	if (!err)
		err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x00);
	int restore_err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00);
	k_usleep(350);
	if (err || restore_err)
		return err ? err : restore_err;
	return 0;
}

static int lsm_ext_set_prefetch(bool enabled)
{
	int err = lsm_ext_stop_continuous();
	if (!err)
		ext_prefetch_enabled = enabled;
	return err;
}

/** Error cleanup never replays an external transaction. */
static int lsm_ext_fail(int err)
{
	int stop_err = lsm_ext_stop_continuous();
	return err ? err : stop_err;
}

/** Poll one register with a wall-clock bound, aborting on the first bus error. */
static int lsm_ext_wait(uint8_t reg, uint8_t mask, int timeout_ms, uint8_t *status)
{
	int64_t deadline = k_uptime_get() + timeout_ms;
	while (k_uptime_get() < deadline) {
		int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, reg, status);
		if (err)
			return err;
		if (k_uptime_get() >= deadline)
			return -ETIMEDOUT;
		if (reg == LSM6DSV_STATUS_MASTER && (*status & LSM6DSV_SLAVE0_NACK))
			return -EIO;
		if (*status & mask)
			return 0;
	}
	return -ETIMEDOUT;
}

static int lsm_ext_wait_xlda(void)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00);
	uint8_t status;
	if (!err)
		err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_OUTX_H_A, &status);
	if (!err)
		err = lsm_ext_wait(LSM6DSV_STATUS_REG, 0x01, LSM6DSV_SHUB_XLDA_TIMEOUT_MS, &status);
	if (!err)
		err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, LSM6DSV_SHUB_REG_ACCESS);
	return err;
}

/**
 * Start a new acquisition epoch, leaving the sensor-hub page selected.
 * The master must already be stopped/settled and the hub page selected.
 * RST_MASTER_REGS resets the master logic/output (DS13476, MASTER_CONFIG).
 * Verify the completion baseline rather than assuming either STATUS alias
 * is read-clear or that MASTER_ON clears an old completion.
 */
static int lsm_ext_arm_quiesced(uint8_t addr, uint8_t sub_addr, uint8_t num_read, const uint8_t *write_data)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, LSM6DSV_RST_MASTER_REGS);
	if (!err)
		err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x00);
	uint8_t status = 0;
	if (!err)
		err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_STATUS_MASTER, &status);
	if (!err && (status & (LSM6DSV_SENS_HUB_ENDOP | LSM6DSV_WR_ONCE_DONE | LSM6DSV_SLAVE0_NACK)))
		err = -EIO;
	uint8_t slv0[3] = {(addr << 1) | (write_data ? 0 : 1), sub_addr, LSM6DSV_SHUB_ODR_240HZ | num_read};
	if (!err)
		err = ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SLV0_ADD, slv0, sizeof(slv0));
	if (!err && write_data)
		err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_DATAWRITE_SLV0, *write_data);
	if (!err)
		err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG,
					LSM6DSV_MASTER_ON | (write_data ? LSM6DSV_WRITE_ONCE : 0));
	return err ? lsm_ext_fail(err) : 0;
}

static int lsm_ext_begin(uint8_t addr, uint8_t sub_addr, uint8_t num_read, const uint8_t *write_data)
{
	int err = lsm_ext_stop_continuous();
	if (err)
		return err;
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, LSM6DSV_SHUB_REG_ACCESS);
	return err ? lsm_ext_fail(err) : lsm_ext_arm_quiesced(addr, sub_addr, num_read, write_data);
}

/** Rearm the already-quiesced hub after consuming its unread epoch. */
static int lsm_ext_start_continuous(uint8_t addr, uint8_t sub_addr, uint8_t num_bytes)
{
	// NUMOP is three bits; bit 3 is FIFO batching, not length 8.
	if (!ext_prefetch_enabled || addr > 0x7f || num_bytes < 1 || num_bytes > 7)
		return -EINVAL;
	int err = lsm_ext_arm_quiesced(addr, sub_addr, num_bytes, NULL);
	if (err)
		return err;
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00);
	if (err)
		return lsm_ext_fail(err);
	ext_cont_addr = addr;
	ext_cont_sub = sub_addr;
	ext_cont_len = num_bytes;
	ext_continuous_active = true;
	return 0;
}

int lsm_ext_write(const uint8_t addr, const uint8_t *buf, uint32_t num_bytes)
{
	if (!buf || addr > 0x7f || num_bytes != 2)
		return -EINVAL;
	int err = lsm_ext_begin(addr, buf[0], 0, &buf[1]);
	if (err)
		return err;
	err = lsm_ext_wait_xlda();
	uint8_t status;
	if (!err)
		err = lsm_ext_wait(LSM6DSV_STATUS_MASTER, LSM6DSV_WR_ONCE_DONE,
				   LSM6DSV_SHUB_OP_TIMEOUT_MS, &status);
	// Keep the terminal DONE+NACK snapshot; never replay an ambiguous write.
	return lsm_ext_fail(err);
}

int lsm_ext_write_read(const uint8_t addr, const void *write_buf, size_t num_write, void *read_buf, size_t num_read)
{
	if (!write_buf || !read_buf || addr > 0x7f || num_write != 1 || num_read < 1 || num_read > 7)
		return -EINVAL;
	uint8_t sub_addr = ((const uint8_t *)write_buf)[0];
	bool pending = ext_prefetch_enabled && ext_continuous_active &&
		       addr == ext_cont_addr && sub_addr == ext_cont_sub && num_read == ext_cont_len;
	// Consume software ownership before any fallible page/status/data access.
	ext_continuous_active = false;
	int err;
	if (pending) {
		err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, LSM6DSV_SHUB_REG_ACCESS);
	} else {
		err = lsm_ext_begin(addr, sub_addr, num_read, NULL);
		if (!err)
			err = lsm_ext_wait_xlda();
	}
	uint8_t status;
	if (!err)
		err = lsm_ext_wait(LSM6DSV_STATUS_MASTER, LSM6DSV_SENS_HUB_ENDOP,
				   LSM6DSV_SHUB_OP_TIMEOUT_MS + (pending ? LSM6DSV_SHUB_XLDA_TIMEOUT_MS : 0), &status);
	if (err)
		return lsm_ext_fail(err);

	// Freeze the output before copying it. WRITE_ONCE limits writes only:
	// under preemption the periodic hub may perform multiple physical reads
	// before this disable, even with prefetch disabled (e.g. ICT single mode).
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x00);
	if (err)
		return lsm_ext_fail(err);
	k_usleep(350);
	// A later operation could have failed between completion and stopping.
	// Check its error without requiring DONE again: the captured completion
	// above remains authoritative even if reading STATUS clears it.
	uint8_t stopped_status;
	err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_STATUS_MASTER, &stopped_status);
	if (!err && ((status | stopped_status) & LSM6DSV_SLAVE0_NACK))
		err = -EIO;
	if (!err)
		err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SENSOR_HUB_1, read_buf, num_read);
	if (err)
		return lsm_ext_fail(err);

	// Reset/rearm gives the next request a provably separate completion epoch.
	// Scanning and destructive-read sensors must not deliberately rearm.
	if (!ext_scanning_mode && ext_prefetch_enabled)
		return lsm_ext_start_continuous(addr, sub_addr, num_read);
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00);
	return err ? lsm_ext_fail(err) : 0;
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
};

const sensor_ext_ssi_t sensor_ext_lsm6dsv = {
	.ext_write = lsm_ext_write,
	.ext_write_read = lsm_ext_write_read,
	.ext_burst = 7,
	.ext_set_prefetch = lsm_ext_set_prefetch,
};
