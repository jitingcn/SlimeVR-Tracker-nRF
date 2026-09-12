#include <math.h>
#include <string.h>

#include <zephyr/logging/log.h>
#include <hal/nrf_gpio.h>

#include "LSM6DSM.h"
#include "LSM6DSV.h" // Common functions
#include "sensor/sensor_none.h"

#define PACKET_SIZE 7              // first byte is pattern, only 6 actual sample bytes
#define FIFO_SANITY_MAX_SAMPLES 32 // guard against corrupted DIFF_FIFO reads

static uint8_t accel_fs = DSM_FS_XL_16G;
static uint8_t gyro_fs = DSM_FS_G_2000DPS;

static uint8_t fifo_pattern_length;
static bool fifo_pattern_gyro_dominant; // if more samples are gyro than accel in a pattern

LOG_MODULE_REGISTER(LSM6DSM, LOG_LEVEL_DBG);

int lsm6dsm_init(
	float clock_rate,
	float accel_period_s,
	float gyro_period_s,
	float *accel_actual_period_s,
	float *gyro_actual_period_s
)
{
	// setup interface for SPI
	sensor_interface_spi_configure(SENSOR_INTERFACE_DEV_IMU, MHZ(10), 0);
	int err = ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		LSM6DSM_CTRL3,
		0x74
	); // freeze register until done reading, increment register address during multi-byte access (BDU, IF_INC), INT
	   // H_LACTIVE active low, PP_OD open-drain
	if (err) {
		LOG_ERR("Communication error");
	}
	last_accel_odr = 0xff; // reset last odr
	last_gyro_odr = 0xff;  // reset last odr
	err |= lsm6dsm_update_odr(accel_period_s, gyro_period_s, accel_actual_period_s, gyro_actual_period_s);
	if (err) {
		LOG_ERR("Communication error");
	}
	return (err < 0 ? err : 0);
}

void lsm6dsm_update_fs(float accel_range, float gyro_range, float *accel_actual_range, float *gyro_actual_range)
{
	if (accel_range > 8) {
		accel_fs = DSM_FS_XL_16G;
		accel_range = 16;
	} else if (accel_range > 4) {
		accel_fs = DSM_FS_XL_8G;
		accel_range = 8;
	} else if (accel_range > 2) {
		accel_fs = DSM_FS_XL_4G;
		accel_range = 4;
	} else {
		accel_fs = DSM_FS_XL_2G;
		accel_range = 2;
	}

	if (gyro_range > 1000) {
		gyro_fs = DSM_FS_G_2000DPS;
		gyro_range = 2000;
	} else if (gyro_range > 500) {
		gyro_fs = DSM_FS_G_1000DPS;
		gyro_range = 1000;
	} else if (gyro_range > 250) {
		gyro_fs = DSM_FS_G_500DPS;
		gyro_range = 500;
	} else {
		gyro_fs = DSM_FS_G_250DPS;
		gyro_range = 250;
	}

	accel_sensitivity = accel_range / 32768.0f;
	gyro_sensitivity = 35.0f * gyro_range / 1000000.0f;

	*accel_actual_range = accel_range;
	*gyro_actual_range = gyro_range;
}

int lsm6dsm_update_odr(
	float accel_period_s,
	float gyro_period_s,
	float *accel_actual_period_s,
	float *gyro_actual_period_s
)
{
	int requested_odr_hz;
	uint8_t accel_mode;
	uint8_t gyro_mode;
	uint8_t accel_odr_bits;
	uint8_t gyro_odr_bits;
	uint8_t gyro_sleep_bits = DSM_OP_MODE_G_AWAKE;

	// Calculate accel
	if (accel_period_s <= 0 || accel_period_s == INFINITY) // off, standby interpreted as off
	{
		// set High perf mode and off odr on XL
		accel_mode = DSM_OP_MODE_XL_HP;
		accel_odr_bits = DSM_ODR_OFF;
		requested_odr_hz = 0;
	} else {
		// set High perf mode and select odr on XL
		accel_mode = DSM_OP_MODE_XL_HP;
		requested_odr_hz = 1 / accel_period_s;
	}

	if (requested_odr_hz == 0) {
		accel_period_s = 0; // off
		accel_odr_bits = DSM_ODR_OFF;
	} else if (accel_period_s < 0.3f / 1000) // Preserve the period thresholds and rounded period outputs.
	{
		accel_odr_bits = DSM_ODR_6_66kHz;
		accel_period_s = 0.15 / 1000;
	} else if (accel_period_s < 0.6f / 1000) {
		accel_odr_bits = DSM_ODR_3_33kHz;
		accel_period_s = 0.3 / 1000;
	} else if (accel_period_s < 1.2f / 1000) {
		accel_odr_bits = DSM_ODR_1_66kHz;
		accel_period_s = 0.6 / 1000;
	} else if (accel_period_s < 2.4f / 1000) {
		accel_odr_bits = DSM_ODR_833Hz;
		accel_period_s = 1.2 / 1000;
	} else if (accel_period_s < 4.8f / 1000) {
		accel_odr_bits = DSM_ODR_416Hz;
		accel_period_s = 2.4 / 1000;
	} else if (accel_period_s < 9.6f / 1000) {
		accel_odr_bits = DSM_ODR_208Hz;
		accel_period_s = 4.8 / 1000;
	} else if (accel_period_s < 19.2f / 1000) {
		accel_odr_bits = DSM_ODR_104Hz;
		accel_period_s = 9.6 / 1000;
	} else if (accel_period_s < 38.4f / 1000) {
		accel_odr_bits = DSM_ODR_52Hz;
		accel_period_s = 19.2 / 1000;
	} else if (requested_odr_hz > 12.5) {
		accel_odr_bits = DSM_ODR_26Hz;
		accel_period_s = 38.4 / 1000;
	} else {
		accel_odr_bits = DSM_ODR_12_5Hz;
		accel_period_s = 1.0 / 12.5;
	}

	// Calculate gyro
	if (gyro_period_s <= 0) // off
	{
		gyro_mode = DSM_OP_MODE_G_HP;
		gyro_odr_bits = DSM_ODR_OFF;
		requested_odr_hz = 0;
	} else if (gyro_period_s == INFINITY) // sleep
	{
		gyro_mode = DSM_OP_MODE_G_NP;
		gyro_sleep_bits = DSM_OP_MODE_G_SLEEP;
		gyro_odr_bits = last_gyro_odr; // using last ODR
		requested_odr_hz = -1;         /* not off: skip ODR_OFF overwrite below */
	} else {
		gyro_mode = DSM_OP_MODE_G_HP;
		gyro_odr_bits = 0; // Initialized before the rate-selection branches below.
		requested_odr_hz = 1 / gyro_period_s;
	}

	if (requested_odr_hz == 0) {
		gyro_period_s = 0; // off
		gyro_odr_bits = DSM_ODR_OFF;
	} else if (requested_odr_hz < 0) {
		/* Sleep retains the previously selected gyro ODR. */
		gyro_period_s = INFINITY;
	} else if (gyro_period_s < 0.3f / 1000) // Preserve the period thresholds and rounded period outputs.
	{
		gyro_odr_bits = DSM_ODR_6_66kHz;
		gyro_period_s = 1.0 / 6660;
	} else if (gyro_period_s < 0.6f / 1000) {
		gyro_odr_bits = DSM_ODR_3_33kHz;
		gyro_period_s = 0.3 / 1000;
	} else if (gyro_period_s < 1.2f / 1000) {
		gyro_odr_bits = DSM_ODR_1_66kHz;
		gyro_period_s = 0.6 / 1000;
	} else if (gyro_period_s < 2.4f / 1000) {
		gyro_odr_bits = DSM_ODR_833Hz;
		gyro_period_s = 1.2 / 1000;
	} else if (gyro_period_s < 4.8f / 1000) {
		gyro_odr_bits = DSM_ODR_416Hz;
		gyro_period_s = 2.4 / 1000;
	} else if (gyro_period_s < 9.6f / 1000) {
		gyro_odr_bits = DSM_ODR_208Hz;
		gyro_period_s = 4.8 / 1000;
	} else if (gyro_period_s < 19.2f / 1000) {
		gyro_odr_bits = DSM_ODR_104Hz;
		gyro_period_s = 9.6 / 1000;
	} else if (gyro_period_s < 38.4f / 1000) {
		gyro_odr_bits = DSM_ODR_52Hz;
		gyro_period_s = 19.2 / 1000;
	} else if (requested_odr_hz > 12.5) {
		gyro_odr_bits = DSM_ODR_26Hz;
		gyro_period_s = 38.4 / 1000;
	} else {
		gyro_odr_bits = DSM_ODR_12_5Hz;
		gyro_period_s = 1.0 / 12.5;
	}

	if (last_accel_mode == accel_mode && last_gyro_mode == gyro_mode && last_accel_odr == accel_odr_bits
		&& last_gyro_odr == gyro_odr_bits) {
		*accel_actual_period_s = accel_period_s;
		*gyro_actual_period_s = gyro_period_s;
		return 0; /* already configured — success for err|= callers */
	}

	uint8_t fifo_odr_bits = MAX(accel_odr_bits, gyro_odr_bits);

	// Convert the ODR-code distance to the FIFO decimation field encoding.
	uint8_t gyro_decimation_bits = (fifo_odr_bits - gyro_odr_bits) >> 4;
	gyro_decimation_bits = gyro_decimation_bits >= 6
							 ? 0
							 : (gyro_decimation_bits >= 2 ? gyro_decimation_bits + 2 : gyro_decimation_bits + 1);

	uint8_t accel_decimation_bits = (fifo_odr_bits - accel_odr_bits) >> 4;
	accel_decimation_bits = accel_decimation_bits >= 6
							  ? 0
							  : (accel_decimation_bits >= 2 ? accel_decimation_bits + 2 : accel_decimation_bits + 1);

	if (gyro_period_s > accel_period_s) {
		fifo_pattern_gyro_dominant = false;
		fifo_pattern_length = accel_period_s == 0 ? 1 : gyro_period_s / accel_period_s + 0.5f;
	} else {
		fifo_pattern_gyro_dominant = true;
		fifo_pattern_length = gyro_period_s == 0 ? 1 : accel_period_s / gyro_period_s + 0.5f;
	}

	int err = ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		LSM6DSM_CTRL1,
		accel_odr_bits | accel_fs
	);                                                                              // set accel ODR and FS
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_CTRL6, accel_mode); // set accelerator perf mode

	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		LSM6DSM_CTRL2,
		gyro_odr_bits | gyro_fs
	);                                                                             // set gyro ODR and mode
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_CTRL7, gyro_mode); // set gyroscope perf mode
	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		LSM6DSM_CTRL4,
		gyro_sleep_bits
	); // set gyroscope awake/sleep mode

	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		LSM6DSM_FIFO_CTRL3,
		(gyro_decimation_bits << 3) | accel_decimation_bits
	); // set decimation
	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		LSM6DSM_FIFO_CTRL5,
		(fifo_odr_bits >> 1) | 0x06
	); // set FIFO ODR, FIFO Continuous mode
	if (err) {
		LOG_ERR("Communication error");
		return err;
	}

	last_accel_mode = accel_mode;
	last_gyro_mode = gyro_mode;
	last_accel_odr = accel_odr_bits;
	last_gyro_odr = gyro_odr_bits;
	*accel_actual_period_s = accel_period_s;
	*gyro_actual_period_s = gyro_period_s;

	return 0;
}

uint16_t lsm6dsm_fifo_read(uint8_t *data, uint16_t capacity_bytes)
{
	int err = 0;
	uint16_t total_packets = 0;
	/* Samples per FIFO pattern cycle; tags cycle in FIFO order and
	 * fifo_process maps them to gyro/accel as before. */
	const uint8_t cycle_samples = (uint8_t)(fifo_pattern_length + 1);

	while (capacity_bytes >= PACKET_SIZE) {
		uint8_t status[4];
		int64_t st0 = k_uptime_ticks();
		err |= ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_FIFO_STATUS1, status, 4);
		{
			uint64_t st_us = k_ticks_to_us_near64(k_uptime_ticks() - st0);
			if (st_us > 2000) {
				LOG_DBG("Slow I2C read: %llu us (status, 4 B)", (unsigned long long)st_us);
			}
		}
		if (err) {
			LOG_ERR("Communication error");
			break;
		}

		uint16_t words = (uint16_t)((status[1] & 7) << 8 | status[0]); // DIFF_FIFO[10:0]
		if (!words) {                                                  // nothing to do
			break;
		}

		uint16_t pattern = (uint16_t)((status[3] & 3) << 8 | status[2]); // FIFO_PATTERN[9:0]

		/* pattern % 3 is the word offset in a sample; skip a partial
		 * sample so the read starts on a sample boundary. */
		uint16_t tag = pattern / 3;
		if (pattern % 3 != 0) // misaligned!
		{
			LOG_WRN("FIFO not aligned");
			uint16_t skip_words = (uint16_t)(3 - (pattern % 3));
			uint8_t discard[8];
			err |= ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_FIFO_DATA_OUT_L, discard, skip_words * 2);
			words = (words > skip_words) ? (uint16_t)(words - skip_words) : 0;
			tag = (uint16_t)((tag + 1) % cycle_samples);
		}

		uint16_t samples = words / 3;
		uint16_t packet_capacity = capacity_bytes / PACKET_SIZE;
		if (samples > packet_capacity) {
			LOG_WRN("FIFO read buffer limit reached, %d packets dropped", samples - packet_capacity);
			samples = packet_capacity;
		}
		/* Bound each batch when a status read reports an implausibly large word count. */
		if (samples > FIFO_SANITY_MAX_SAMPLES) {
			LOG_DBG("FIFO status spike: %u words -> clamped to %u samples", words, FIFO_SANITY_MAX_SAMPLES);
			samples = FIFO_SANITY_MAX_SAMPLES;
		}
		if (samples == 0) {
			break;
		}

		/* Hardware supplies untagged 16-bit words; prepend a software
		 * pattern index to each six-byte sample for fifo_process. */
		uint8_t raw[24 * 6];
		while (samples > 0) {
			uint16_t n = samples;
			if (n > (uint16_t)(sizeof(raw) / 6)) {
				n = (uint16_t)(sizeof(raw) / 6);
			}
			int64_t d0 = k_uptime_ticks();
			err |= ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_FIFO_DATA_OUT_L, raw, n * 6);
			if (err) {
				LOG_ERR("Communication error");
				return total_packets;
			}
			{
				uint64_t d_us = k_ticks_to_us_near64(k_uptime_ticks() - d0);
				if (d_us > 2000) {
					LOG_DBG("Slow I2C read: %llu us (data, %u B)", (unsigned long long)d_us, n * 6);
				}
			}
			for (uint16_t i = 0; i < n; i++) {
				data[i * PACKET_SIZE] = (uint8_t)((tag + i) % cycle_samples);
				memcpy(&data[i * PACKET_SIZE + 1], &raw[i * 6], PACKET_SIZE - 1);
			}
			samples -= n;
			tag = (uint16_t)((tag + n) % cycle_samples);
			data += n * PACKET_SIZE;
			capacity_bytes -= n * PACKET_SIZE;
			total_packets += n;
		}
	}
	return total_packets;
}

int lsm6dsm_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3])
{
	const uint16_t packet_offset = index * PACKET_SIZE;
	const uint8_t *packet = &data[packet_offset];
	uint8_t pattern = packet[0]; // Software pattern index, not a hardware FIFO tag.
	if ((pattern == 0 && (fifo_pattern_length != 1 || fifo_pattern_gyro_dominant))
		|| (pattern > 1 && fifo_pattern_gyro_dominant)) {
		for (int i = 0; i < 3; i++) // x, y, z
		{
			g[i] = (int16_t)((((uint16_t)packet[2 + (i * 2)]) << 8) | packet[1 + (i * 2)]);
			g[i] *= gyro_sensitivity;
		}
		return 0;
	} else {
		for (int i = 0; i < 3; i++) // x, y, z
		{
			a[i] = (int16_t)((((uint16_t)packet[2 + (i * 2)]) << 8) | packet[1 + (i * 2)]);
			a[i] *= accel_sensitivity;
		}
		return 0;
	}
	return 1;
}

/* LSM6DSM does not have COUNTER_BDR, FIFO threshold uses word count, or 3 words per sensor sample
 * ex. if gyro odr is higher, and pattern length is 3:
 * first timestamp will have both gyro and accel
 * second (and above) timestamp will contain only gyro for the rest of the pattern length
 * 0->(0+0)*3, 1->(1+1)*3, 2->(1+2)*3, 3->(1+3)*3, 4->(2+4)*3
 * this assumes pattern always begins at 0 on fifo read, which it will not.
 */
uint8_t lsm6dsm_setup_DRDY(uint16_t threshold)
{
	threshold = ((threshold + fifo_pattern_length - 1) / fifo_pattern_length + threshold) * 3;
	uint8_t buf[2];
	buf[0] = threshold & 0xFF;
	buf[1] = (threshold >> 8) & 0x07;
	int err = ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_FIFO_CTRL1, buf, 2);
	/* LSM6DSM has no COUNTER_BDR (unlike LSM6DSV); use pulsed INT1_DRDY_G for
	 * per-sample wakeup, with INT1_FTH as the burst/catch-up backup. */
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_INT1_CTRL, 0x0A);      // INT1_FTH | INT1_DRDY_G
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_DRDY_PULSE_CFG, 0x80); // pulsed DRDY (75 us)
	if (err) {
		LOG_ERR("Communication error");
	}
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
}

uint8_t lsm6dsm_setup_WOM(void)
{

	int err = ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		LSM6DSM_CTRL1,
		DSM_ODR_208Hz | DSM_FS_XL_8G
	);                                                                                     // set accel ODR and FS
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_CTRL6, DSM_OP_MODE_XL_NP); // set accel perf mode
	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		LSM6DSM_CTRL8,
		0x74
	); // set HPCF_XL to the lowest bandwidth, enable HP_REF_MODE (set HP_REF_MODE, HP_SLOPE_XL_EN, HPCF_XL nonzero)
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_TAP_CFG, 0x10); // set SLOPE_FDS
	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		LSM6DSM_WAKE_UP_THS,
		0x01
	);            // set threshold, 1 * 31.25 mg is ~31.25 mg
	k_msleep(12); // need to wait for accel to settle

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_TAP_CFG, 0x90); // enable interrupts (keep SLOPE_FDS)
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_MD1_CFG, 0x20); // route wake-up to INT1
	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		LSM6DSM_CTRL3,
		0x30
	); // INT H_LACTIVE active low, PP_OD open-drain
	if (err) {
		LOG_ERR("Communication error");
	}
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
}

static int lsm6dsm_ext_setup(enum sensor_ext_mode mode)
{
	if (mode != SENSOR_EXT_MODE_OFF && mode != SENSOR_EXT_MODE_I2C_PASSTHROUGH) {
		return -1;
	}

	/* MASTER_CONFIG passthrough is effective only while CTRL10_C.FUNC_EN is set. */
	int err = ssi_reg_update_byte(SENSOR_INTERFACE_DEV_IMU, 0x19, 0x04, 0x04);
	err |= ssi_reg_update_byte(
		SENSOR_INTERFACE_DEV_IMU,
		0x1A,
		0x05,
		mode == SENSOR_EXT_MODE_I2C_PASSTHROUGH ? 0x04 : 0x00
	);
	if (err) {
		LOG_ERR("Communication error");
	}
	return err;
}

const sensor_imu_t sensor_imu_lsm6dsm
	= {*lsm6dsm_init,
	   *lsm_shutdown,

	   *lsm6dsm_update_fs,
	   *lsm6dsm_update_odr,

	   *lsm6dsm_fifo_read,
	   *lsm6dsm_fifo_process,
	   *lsm_accel_read,
	   *lsm_gyro_read,
	   *lsm_temp_read,

	   *lsm6dsm_setup_DRDY,
	   *lsm6dsm_setup_WOM,

	   *lsm6dsm_ext_setup};
