#include <errno.h>
#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_gpio.h>

#include "ICM45686.h"
#include "sensor/sensor_none.h"

#define ICM45686_FIFO_PACKET_SIZE 20

// DS-000577: UI registers use the configured +/-32 g and +/-4000 dps ranges.
static const float accel_g_per_lsb = 32.0f / 32768.0f;
static const float gyro_dps_per_lsb = 4000.0f / 32768.0f;

// FIFO 20-bit samples are left-aligned in int32_t: full scale is 2^31 counts.
static const float accel_g_per_left_aligned_lsb = 32.0f / ((uint32_t)2 << 30);
static const float gyro_dps_per_left_aligned_lsb = 4000.0f / ((uint32_t)2 << 30);

static const float odr_hz[] = {6400.0f, 3200.0f, 1600.0f, 800.0f, 400.0f, 200.0f, 100.0f, 50.0f, 25.0f, 12.5f};
static const uint8_t odr_codes[]
	= {ACCEL_ODR_6_4kHz,
	   ACCEL_ODR_3_2kHz,
	   ACCEL_ODR_1_6kHz,
	   ACCEL_ODR_800Hz,
	   ACCEL_ODR_400Hz,
	   ACCEL_ODR_200Hz,
	   ACCEL_ODR_100Hz,
	   ACCEL_ODR_50Hz,
	   ACCEL_ODR_25Hz,
	   ACCEL_ODR_12_5Hz};

static uint8_t last_accel_odr = 0xff;
static uint8_t last_gyro_odr = 0xff;
static uint8_t last_accel_mode = 0xff;
static uint8_t last_gyro_mode = 0xff;
static const float clock_reference_hz = 32000;
static float clock_scale = 1; // ODR multiplier: clock_rate_hz / clock_reference_hz.

// Host-triggered, one-read-ahead I2CM state, not autonomous sensor-hub sampling.
// A matching request validates completion, reads I2CM_RD_DATA, then starts the
// next GO. External data is not enabled in FIFO_CONFIG3 or decoded by FIFO APIs.
static bool ext_read_ahead_active = false;
static uint8_t ext_read_ahead_addr = 0;
static uint8_t ext_read_ahead_reg = 0;
static uint8_t ext_read_ahead_len = 0;
static bool ext_scanning_mode = true;
static bool ext_prefetch_enabled = true;
static int icm45_ext_drain_read_ahead(void);

// Cache the mean temperature of accepted-header packets in the latest FIFO read.
// Empty or failed acquisition invalidates the cache.
static float fifo_temp_c = 25.0f;
static bool fifo_temp_valid = false;

static void icm45_cache_fifo_temp(const uint8_t *data, uint16_t packets)
{
	fifo_temp_valid = false;
	int32_t raw_temp_sum = 0;
	uint16_t valid_packets = 0;

	for (uint16_t i = 0; i < packets; i++) {
		const uint8_t *packet = &data[i * ICM45686_FIFO_PACKET_SIZE];
		if (packet[0] != 0x78 && packet[0] != 0x7A) {
			continue;
		}

		int16_t raw_temp = (int16_t)((((uint16_t)packet[13]) << 8) | packet[14]);
		raw_temp_sum += raw_temp;
		valid_packets++;
	}

	if (valid_packets == 0) {
		return;
	}

	fifo_temp_c = ((float)raw_temp_sum / (128.0f * valid_packets)) + 25.0f;
	fifo_temp_valid = true;
}

LOG_MODULE_REGISTER(ICM45686, LOG_LEVEL_DBG);

// All IREG accesses use the bank helpers below, including runtime verification.
static int icm45_bank_write(uint8_t bank, uint8_t reg, const uint8_t *buf, uint32_t num_bytes)
{
	if (num_bytes == 0 || buf == NULL) {
		return -EINVAL;
	}

	// Address and first DATA must share one burst to avoid unintended prefetch.
	uint8_t ireg_buf[3] = {bank, reg, buf[0]};
	int err = ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, ireg_buf, 3);
	k_busy_wait(4);
	if (err) {
		return err;
	}
	for (uint32_t i = 1; i < num_bytes; i++) {
		err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_DATA, buf[i]);
		k_busy_wait(4);
		if (err) {
			return err;
		}
	}
	return 0;
}

static int icm45_bank_write_byte(uint8_t bank, uint8_t reg, uint8_t value)
{
	return icm45_bank_write(bank, reg, &value, 1);
}

static int icm45_bank_read(uint8_t bank, uint8_t reg, uint8_t *buf, uint32_t num_bytes)
{
	if (num_bytes == 0 || buf == NULL) {
		return -EINVAL;
	}

	uint8_t ireg_buf[2] = {bank, reg};
	int err = ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, ireg_buf, 2);
	k_busy_wait(4);
	if (err) {
		return err;
	}
	for (uint32_t i = 0; i < num_bytes; i++) {
		err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_DATA, &buf[i]);
		k_busy_wait(4);
		if (err) {
			return err;
		}
	}
	return 0;
}

static int icm45_bank_read_byte(uint8_t bank, uint8_t reg, uint8_t *value)
{
	return icm45_bank_read(bank, reg, value, 1);
}

static int icm45686_ireg_read8(uint16_t ireg_addr, uint8_t *out)
{
	return icm45_bank_read_byte(ireg_addr >> 8, ireg_addr & 0xFF, out);
}

static int icm45686_ireg_write8(uint16_t ireg_addr, uint8_t value)
{
	return icm45_bank_write_byte(ireg_addr >> 8, ireg_addr & 0xFF, value);
}

static int icm45686_set_src_ctrl(uint8_t gyro_sel, uint8_t accel_sel)
{
	// GYRO_SRC_CTRL: IPREG_SYS1_REG_166 (0x00A6) bits [6:5]
	// ACCEL_SRC_CTRL: IPREG_SYS2_REG_123 (0x007B) bits [1:0]
	// Encoding cross-checked against TDK inv_imu_defs.h:
	// 0: Interpolator and FIR filter off
	// 1: Interpolator off and FIR filter on
	// 2: Interpolator on and FIR filter on
	const uint16_t gyro_src_ctrl_addr = 0xA400u + 0x00A6u;
	const uint16_t accel_src_ctrl_addr = 0xA500u + 0x007Bu;

	gyro_sel &= 0x03;  // Limit to 2 bits
	accel_sel &= 0x03; // Limit to 2 bits

	uint8_t gyro_reg = 0, accel_reg = 0;
	int err = 0;

	err = icm45686_ireg_read8(gyro_src_ctrl_addr, &gyro_reg);
	if (err) {
		return err;
	}
	err = icm45686_ireg_read8(accel_src_ctrl_addr, &accel_reg);
	if (err) {
		LOG_WRN("SRC_CTRL pre-read failed (err=%d)", err);
		return err;
	}

	uint8_t gyro_new = (uint8_t)((gyro_reg & ~(0x03u << 5)) | (uint8_t)(gyro_sel << 5));
	uint8_t accel_new = (uint8_t)((accel_reg & ~0x03u) | accel_sel);

	LOG_INF(
		"SRC_CTRL set: GYRO 0x%02X->0x%02X (sel=%u), ACCEL 0x%02X->0x%02X (sel=%u)",
		gyro_reg,
		gyro_new,
		gyro_sel,
		accel_reg,
		accel_new,
		accel_sel
	);

	err = icm45686_ireg_write8(gyro_src_ctrl_addr, gyro_new);
	if (err) {
		return err;
	}
	err = icm45686_ireg_write8(accel_src_ctrl_addr, accel_new);
	if (err) {
		LOG_WRN("SRC_CTRL write failed (err=%d)", err);
		return err;
	}

	uint8_t gyro_readback = 0, accel_readback = 0;
	int readback_err = 0;
	readback_err = icm45686_ireg_read8(gyro_src_ctrl_addr, &gyro_readback);
	if (readback_err) {
		return readback_err;
	}
	readback_err = icm45686_ireg_read8(accel_src_ctrl_addr, &accel_readback);
	if (!readback_err) {
		LOG_INF(
			"SRC_CTRL post-read: GYRO=0x%02X (sel=%u) ACCEL=0x%02X (sel=%u)",
			gyro_readback,
			(gyro_readback >> 5) & 0x03,
			accel_readback,
			accel_readback & 0x03
		);
	} else {
		LOG_WRN("SRC_CTRL post-read failed (err=%d)", readback_err);
	}
	return readback_err;
}

static int icm45686_set_ui_lpfbw_sel(uint8_t gyro_sel, uint8_t accel_sel)
{
	// Gyro UI LPF BW: IPREG_SYS1_REG_172 (0x00AC) bits [2:0]
	// Accel UI LPF BW: IPREG_SYS2_REG_131 (0x0083) bits [2:0]
	// 0=bypass, 1=ODR/4, 2=ODR/8, 3=ODR/16, 4=ODR/32, 5=ODR/64, 6=ODR/128.
	// The helper preserves all three selector bits; code 7 is not used here.
	const uint16_t gyro_ui_lpfbw_addr = 0xA400u + 0x00ACu;
	const uint16_t accel_ui_lpfbw_addr = 0xA500u + 0x0083u;

	gyro_sel &= 0x07;
	accel_sel &= 0x07;

	uint8_t gyro_reg = 0, accel_reg = 0;
	int err = 0;
	err = icm45686_ireg_read8(gyro_ui_lpfbw_addr, &gyro_reg);
	if (err) {
		return err;
	}
	err = icm45686_ireg_read8(accel_ui_lpfbw_addr, &accel_reg);
	if (err) {
		LOG_WRN("UI_LPFBW pre-read failed (err=%d)", err);
		return err;
	}

	uint8_t gyro_new = (uint8_t)((gyro_reg & ~0x07u) | gyro_sel);
	uint8_t accel_new = (uint8_t)((accel_reg & ~0x07u) | accel_sel);

	LOG_INF(
		"UI_LPFBW set: GYRO 0x%02X->0x%02X (sel=%u), ACCEL 0x%02X->0x%02X (sel=%u)",
		gyro_reg,
		gyro_new,
		gyro_sel,
		accel_reg,
		accel_new,
		accel_sel
	);

	err = icm45686_ireg_write8(gyro_ui_lpfbw_addr, gyro_new);
	if (err) {
		return err;
	}
	err = icm45686_ireg_write8(accel_ui_lpfbw_addr, accel_new);
	if (err) {
		LOG_WRN("UI_LPFBW write failed (err=%d)", err);
		return err;
	}

	uint8_t gyro_readback = 0, accel_readback = 0;
	int readback_err = 0;
	readback_err = icm45686_ireg_read8(gyro_ui_lpfbw_addr, &gyro_readback);
	if (readback_err) {
		return readback_err;
	}
	readback_err = icm45686_ireg_read8(accel_ui_lpfbw_addr, &accel_readback);
	if (!readback_err) {
		LOG_INF(
			"UI_LPFBW post-read: GYRO=0x%02X (sel=%u) ACCEL=0x%02X (sel=%u)",
			gyro_readback,
			gyro_readback & 0x07,
			accel_readback,
			accel_readback & 0x07
		);
	} else {
		LOG_WRN("UI_LPFBW post-read failed (err=%d)", readback_err);
	}
	return readback_err;
}

static void icm45_invalidate_odr_cache(void)
{
	last_accel_odr = 0xff;
	last_gyro_odr = 0xff;
	last_accel_mode = 0xff;
	last_gyro_mode = 0xff;
}

// Quiesce FIFO, WOM and interrupt routing before selecting the sampling clock.
static int icm45_disable_acquisition(void)
{
	int err;
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_CONFIG3, 0x00);
	if (err) {
		return err;
	}
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_CONFIG0, 0x00);
	if (err) {
		return err;
	}
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_TMST_WOM_CONFIG, 0x00);
	if (err) {
		return err;
	}
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_INT1_CONFIG0, 0x00);
	if (err) {
		return err;
	}
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_INT1_CONFIG1, 0x00);
	if (err) {
		return err;
	}
	return 0;
}

// Configure host/AUX pins and sensor-register/FIFO byte order without changing ODR.
static int icm45_configure_clock_and_pads(float clock_rate_hz)
{
	int err;

	clock_scale = 1.0f;
	if (clock_rate_hz > 0) {
		clock_scale = clock_rate_hz / clock_reference_hz;
		err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_IOC_PAD_SCENARIO_OVRD, 0x06);
		if (err) {
			return err;
		}
		err = ssi_reg_update_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_RTC_CONFIG, 0x20, 0x20);
		if (err) {
			return err;
		}
	}
	// Disable internal pull resistors for AP pins, preserving AUX1 I2CM mode.
	err = icm45_bank_write_byte(ICM45686_IPREG_BAR, ICM45686_IPREG_BAR_REG_58, 0xD9 & ~0x48);
	if (err) {
		return err;
	}
	err = icm45_bank_write_byte(ICM45686_IPREG_BAR, ICM45686_IPREG_BAR_REG_59, 0xB6 & ~0x92);
	if (err) {
		return err;
	}
	err = icm45_bank_write_byte(ICM45686_IPREG_BAR, ICM45686_IPREG_BAR_REG_60, ICM45686_BIT_AUX1_I2CM_MODE);
	if (err) {
		return err;
	}
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_IOC_PAD_SCENARIO_AUX_OVRD, 0x17);
	if (err) {
		return err;
	}
	// OSC_ID_OVRD=2 uses the gyro oscillator.
	err = ssi_reg_update_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_REG_MISC1, 0x0F, 0x02);
	if (err) {
		return err;
	}
	err = icm45_bank_write_byte(ICM45686_IPREG_TOP1, ICM45686_SREG_CTRL, 0x02); // big endian
	if (err) {
		return err;
	}
	return 0;
}

int icm45_init(
	float clock_rate_hz,
	float accel_period_s,
	float gyro_period_s,
	float *accel_actual_period_s,
	float *gyro_actual_period_s
)
{
	fifo_temp_c = 25.0f;
	fifo_temp_valid = false;
	sensor_interface_spi_configure(SENSOR_INTERFACE_DEV_IMU, MHZ(24), 0);

	int err = icm45_ext_drain_read_ahead();
	if (err) {
		return err;
	}

	// sensor_init() already issued shutdown/reset before calling init.
	uint8_t who_am_i = 0;
	err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, 0x72, &who_am_i);
	if (err) {
		return err;
	}
	LOG_INF("WHO_AM_I = 0x%02X (expected 0xE9/0xE7)", who_am_i);
	if (who_am_i != 0xE9 && who_am_i != 0xE7) {
		LOG_ERR("Invalid WHO_AM_I value");
		return -ENODEV;
	}

	err = icm45_disable_acquisition();
	if (err) {
		return err;
	}
	err = icm45_configure_clock_and_pads(clock_rate_hz);
	if (err) {
		return err;
	}

	icm45_invalidate_odr_cache();
	err = icm45_update_odr(accel_period_s, gyro_period_s, accel_actual_period_s, gyro_actual_period_s);
	if (err) {
		return err;
	}
	// SRC_CTRL=1 keeps FIR enabled and disables interpolation; UI LPF=1 is ODR/4.
	err = icm45686_set_src_ctrl(ICM45686_SRC_INTERPOLATOR_OFF_FIR_ON, ICM45686_SRC_INTERPOLATOR_OFF_FIR_ON);
	if (err) {
		return err;
	}
	err = icm45686_set_ui_lpfbw_sel(ICM45686_UI_LPF_ODR_DIV_4, ICM45686_UI_LPF_ODR_DIV_4);
	if (err) {
		return err;
	}

	/* Stop-on-full permits draining all 20-byte frames without the streaming
	 * read-empty corruption (AN-000364 v1.5 section 2.16). Keep 2K depth. */
	err = ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		ICM45686_FIFO_CONFIG0,
		ICM45686_FIFO_MODE_STOP_ON_FULL | ICM45686_FIFO_DEPTH_2K
	);
	if (err) {
		return err;
	}
	err = ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		ICM45686_FIFO_CONFIG3,
		ICM45686_FIFO_IF_EN | ICM45686_FIFO_ACCEL_EN | ICM45686_FIFO_GYRO_EN | ICM45686_FIFO_HIRES_EN
	);
	if (err) {
		return err;
	}

	if (clock_rate_hz > 0) {
		k_msleep(6);
		uint8_t raw_count[2];
		err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_COUNT_0, raw_count, 2);
		if (err) {
			return err;
		}
		uint16_t fifo_count = (uint16_t)(raw_count[0] << 8 | raw_count[1]);
		if (fifo_count == 0) {
			LOG_WRN("External CLKIN not working, falling back to internal clock");
			clock_scale = 1;
			err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_IOC_PAD_SCENARIO_OVRD, 0x00);
			if (err) {
				return err;
			}
			err = ssi_reg_update_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_RTC_CONFIG, 0x20, 0x00);
			if (err) {
				return err;
			}
			icm45_invalidate_odr_cache();
			err = icm45_update_odr(accel_period_s, gyro_period_s, accel_actual_period_s, gyro_actual_period_s);
			if (err) {
				return err;
			}
		} else {
			LOG_INF("External CLKIN verified: FIFO count=%d", fifo_count);
		}
	}
	ext_scanning_mode = false;
	return 0;
}

void icm45_shutdown(void)
{
	int err = icm45_ext_drain_read_ahead();
	icm45_invalidate_odr_cache();
	fifo_temp_c = 25.0f;
	fifo_temp_valid = false;
	if (err) {
		LOG_ERR("Failed to stop I2CM: %d", err);
	}
	// Global reset terminates even a stuck/uncertain GO; unlike command changes,
	// reset recovery must remain available when the bounded idle drain fails.
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_REG_MISC2, 0x02); // Soft reset
	k_msleep(2); // Retain the reset settling delay; its 2 ms margin is not verified here.
	if (err) {
		LOG_ERR("Communication error");
	}
}

void icm45_update_fs(float accel_range, float gyro_range, float *accel_actual_range, float *gyro_actual_range)
{
	ARG_UNUSED(accel_range);
	ARG_UNUSED(gyro_range);
	// This driver fixes the high-resolution FIFO range rather than selecting a range.
	*accel_actual_range = 32;
	*gyro_actual_range = 4000;
}

int icm45_update_odr(
	float accel_period_s,
	float gyro_period_s,
	float *accel_actual_period_s,
	float *gyro_actual_period_s
)
{
	float requested_odr_hz;
	// Match the fixed full-scale factors used by the high-resolution FIFO decoder.
	uint8_t accel_fs_code = ACCEL_UI_FS_SEL_32G;
	uint8_t gyro_fs_code = GYRO_UI_FS_SEL_4000DPS;
	uint8_t accel_mode;
	uint8_t gyro_mode;
	uint8_t accel_odr_code = 0;
	uint8_t gyro_odr_code = 0;

	// Calculate accel
	if (accel_period_s <= 0 || accel_period_s == INFINITY) // off, standby interpreted as off
	{
		accel_mode = ACCEL_MODE_OFF;
		accel_period_s = 0; // off
	} else {
		accel_mode = ACCEL_MODE_LN;
		requested_odr_hz = (1.0f / accel_period_s) / clock_scale;
		size_t selected = 0;
		for (size_t i = 1; i < ARRAY_SIZE(odr_hz); i++) {
			if (requested_odr_hz > odr_hz[i]) {
				break;
			}
			selected = i;
		}
		accel_odr_code = odr_codes[selected];
		accel_period_s = 1.0f / odr_hz[selected];
	}
	accel_period_s /= clock_scale; // scale clock

	// Calculate gyro
	if (gyro_period_s <= 0) // off
	{
		gyro_mode = GYRO_MODE_OFF;
		gyro_period_s = 0;                // off
	} else if (gyro_period_s == INFINITY) // standby
	{
		gyro_mode = GYRO_MODE_STANDBY;
		gyro_period_s = 0; // off
	} else {
		gyro_mode = GYRO_MODE_LN;
		requested_odr_hz = (1.0f / gyro_period_s) / clock_scale;
		size_t selected = 0;
		for (size_t i = 1; i < ARRAY_SIZE(odr_hz); i++) {
			if (requested_odr_hz > odr_hz[i]) {
				break;
			}
			selected = i;
		}
		gyro_odr_code = odr_codes[selected];
		gyro_period_s = 1.0f / odr_hz[selected];
	}
	gyro_period_s /= clock_scale; // scale clock

	if (last_accel_odr == accel_odr_code && last_gyro_odr == gyro_odr_code && last_accel_mode == accel_mode
		&& last_gyro_mode == gyro_mode) {
		*accel_actual_period_s = accel_period_s;
		*gyro_actual_period_s = gyro_period_s;
		return 0; /* already configured — success for err|= callers */
	}

	int err = 0;
	// only if the power mode has changed
	if (last_accel_mode != accel_mode || last_gyro_mode != gyro_mode) {
		uint8_t pwr_mgmt = gyro_mode << 2 | accel_mode;
		LOG_INF("PWR_MGMT0 write = 0x%02X (GYRO_MODE=%d, ACCEL_MODE=%d)", pwr_mgmt, gyro_mode, accel_mode);
		err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_PWR_MGMT0, pwr_mgmt); // set accel and gyro modes
		if (err) {
			goto fail;
		}
		k_busy_wait(250); // Inherited power-transition delay; required minimum remains unverified.
		// Read back to verify
		uint8_t pwr_mgmt_readback = 0;
		err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_PWR_MGMT0, &pwr_mgmt_readback);
		if (err) {
			goto fail;
		}
		LOG_INF("PWR_MGMT0 readback = 0x%02X", pwr_mgmt_readback);
	}

	uint8_t accel_config = accel_fs_code << 4 | accel_odr_code;
	uint8_t gyro_config = gyro_fs_code << 4 | gyro_odr_code;
	LOG_INF(
		"ACCEL_CONFIG0 write = 0x%02X (ODR=%d), GYRO_CONFIG0 write = 0x%02X (ODR=%d)",
		accel_config,
		accel_odr_code,
		gyro_config,
		gyro_odr_code
	);
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_ACCEL_CONFIG0, accel_config); // set accel ODR and FS
	if (err) {
		goto fail;
	}
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_GYRO_CONFIG0, gyro_config); // set gyro ODR and FS
	if (err) {
		goto fail;
	}
	// Read back to verify
	uint8_t accel_config_readback = 0, gyro_config_readback = 0;
	err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_ACCEL_CONFIG0, &accel_config_readback);
	if (err) {
		goto fail;
	}
	err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_GYRO_CONFIG0, &gyro_config_readback);
	if (err) {
		goto fail;
	}
	LOG_INF(
		"ACCEL_CONFIG0 readback = 0x%02X, GYRO_CONFIG0 readback = 0x%02X",
		accel_config_readback,
		gyro_config_readback
	);

	last_accel_odr = accel_odr_code;
	last_gyro_odr = gyro_odr_code;
	last_accel_mode = accel_mode;
	last_gyro_mode = gyro_mode;
	*accel_actual_period_s = accel_period_s;
	*gyro_actual_period_s = gyro_period_s;

	return 0;

fail:
	icm45_invalidate_odr_cache();
	LOG_ERR("Communication error");
	return err;
}

uint16_t icm45_fifo_read(uint8_t *data, uint16_t len)
{
	fifo_temp_valid = false;
	int err = 0;
	uint8_t raw_count[2];
	/* Retain TDK's conservative double read. AN-000364 v1.5 section 2.2
	 * requires it specifically for stop-on-full mode with 8-byte frames. */
	err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_COUNT_0, &raw_count[0], 2);
	err |= ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_COUNT_0, &raw_count[0], 2);
	if (err) {
		LOG_ERR("Failed to read FIFO count");
		return 0;
	}
	uint16_t packets = (uint16_t)(raw_count[0] << 8 | raw_count[1]); // Big-endian frame count.

	if (packets == 0) {
		return 0;
	}

	// Cap the requested read; excess frames are not consumed by this transfer.
	uint16_t limit = len / ICM45686_FIFO_PACKET_SIZE;
	if (packets > limit) {
		LOG_WRN("FIFO read buffer limit reached, %d packets retained", packets - limit);
		packets = limit;
	}

	if (packets == 0) {
		return 0;
	}
	uint16_t count = packets * ICM45686_FIFO_PACKET_SIZE;
	err = ssi_burst_read_interval(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_DATA, data, count, ICM45686_FIFO_PACKET_SIZE);
	if (err) {
		LOG_ERR("Communication error");
		return 0;
	}
	icm45_cache_fifo_temp(data, packets);

	return packets;
}

static const uint8_t invalid_axes_be16[6] = {0x80, 0x00, 0x80, 0x00, 0x80, 0x00};
static int64_t icm45_last_bad_header_ms = 0;

int icm45_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3])
{
	// Preserve the existing 16-bit byte-offset arithmetic.
	uint16_t packet_offset = index * ICM45686_FIFO_PACKET_SIZE;
	const uint8_t *packet = &data[packet_offset];
	// 0x78 includes timestamp; 0x7A additionally sets accel_odr_different (bit 1).
	if (packet[0] != 0x78 && packet[0] != 0x7A) {
		// Rate-limit: a corrupted FIFO_COUNT can otherwise spam one warning per packet
		if (k_uptime_get() - icm45_last_bad_header_ms >= 1000) {
			LOG_WRN("Invalid FIFO header: 0x%02X (expected 0x78 or 0x7A)", packet[0]);
			icm45_last_bad_header_ms = k_uptime_get();
		}
		return 1; // Skip invalid header
	}

	// Debug: Log first packet gyro data bytes to see what we're getting
	static int debug_count = 0;
	if (debug_count < 3) {
		LOG_INF(
			"FIFO packet[%d]: header=0x%02X, gyro bytes: %02X %02X %02X %02X %02X %02X",
			debug_count,
			packet[0],
			packet[7],
			packet[8],
			packet[9],
			packet[10],
			packet[11],
			packet[12]
		);
		debug_count++;
	}
	// Layout: header[0], accel[1:6], gyro[7:12], temperature[13:14],
	// timestamp[15:16], shared accel/gyro low nibbles[17:19].
	// Header rejection also skips a 0x7F-filled frame; no separate empty marker test.
	// Assemble signed 20-bit samples in bits [31:12], then scale in float.
	// A channel is invalid only when all three upper 16-bit values are 0x8000.
	// One invalid channel produces zeros; two invalid channels leave outputs untouched.
	float accel_raw[3] = {0};
	float gyro_raw[3] = {0};
	if (memcmp(&packet[1], invalid_axes_be16, sizeof(invalid_axes_be16))) // valid accel data
	{
		for (int axis = 0; axis < 3; axis++) {
			accel_raw[axis]
				= (int32_t)((((uint32_t)packet[1 + (axis * 2)]) << 24) | (((uint32_t)packet[2 + (axis * 2)]) << 16)
							| (((uint32_t)packet[17 + axis] & 0xF0) << 8));
		}
	}
	if (memcmp(&packet[7], invalid_axes_be16, sizeof(invalid_axes_be16))) // valid gyro data
	{
		for (int axis = 0; axis < 3; axis++) {
			gyro_raw[axis]
				= (int32_t)((((uint32_t)packet[7 + (axis * 2)]) << 24) | (((uint32_t)packet[8 + (axis * 2)]) << 16)
							| (((uint32_t)packet[17 + axis] & 0x0F) << 12));
		}
	} else {
		static int gyro_invalid_count = 0;
		if (gyro_invalid_count < 5) {
			LOG_WRN("Gyro data marked as invalid (0x80 pattern)");
			gyro_invalid_count++;
		}
		if (!memcmp(&packet[1], invalid_axes_be16, sizeof(invalid_axes_be16))) // both channels invalid
		{
			return 1;
		}
	}
	for (int axis = 0; axis < 3; axis++) {
		accel_raw[axis] *= accel_g_per_left_aligned_lsb;
		gyro_raw[axis] *= gyro_dps_per_left_aligned_lsb;
	}
	memcpy(a, accel_raw, sizeof(accel_raw));
	memcpy(g, gyro_raw, sizeof(gyro_raw));
	return 0;
}

void icm45_accel_read(float a[3])
{
	uint8_t raw_accel[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, ICM45686_ACCEL_DATA_X1_UI, &raw_accel[0], 6);
	if (err) {
		LOG_ERR("Communication error");
		memset(a, 0, 3 * sizeof(*a));
		return;
	}
	for (int i = 0; i < 3; i++) // x, y, z
	{
		a[i] = (int16_t)((((uint16_t)raw_accel[i * 2]) << 8) | raw_accel[1 + (i * 2)]);
		a[i] *= accel_g_per_lsb;
	}
}

void icm45_gyro_read(float g[3])
{
	uint8_t raw_gyro[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, ICM45686_GYRO_DATA_X1_UI, &raw_gyro[0], 6);
	if (err) {
		LOG_ERR("Communication error");
		memset(g, 0, 3 * sizeof(*g));
		return;
	}
	for (int i = 0; i < 3; i++) // x, y, z
	{
		g[i] = (int16_t)((((uint16_t)raw_gyro[i * 2]) << 8) | raw_gyro[1 + (i * 2)]);
		g[i] *= gyro_dps_per_lsb;
	}
}

float icm45_temp_read(void)
{
	if (fifo_temp_valid) {
		return fifo_temp_c;
	}

	uint8_t raw_temp[2];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, ICM45686_TEMP_DATA1_UI, &raw_temp[0], 2);
	if (err) {
		LOG_ERR("Communication error");
		return NAN;
	}
	// Temperature in Degrees Centigrade = (TEMP_DATA / 128) + 25
	float temp = (int16_t)((((uint16_t)raw_temp[0]) << 8) | raw_temp[1]);
	temp /= 128;
	temp += 25;
	return temp;
}

uint8_t icm45_setup_DRDY(uint16_t threshold)
{
	/* FIFO_WR_WM_GT_TH defaults to equality (DS-000577 section 17.31).
	 * All N frames are readable; zero disables the watermark. */
	uint8_t buf[2];
	buf[0] = threshold & 0xFF;
	buf[1] = threshold >> 8;
	int err = ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_CONFIG1_0, buf, 2);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_INT1_CONFIG0, 0x02); // FIFO threshold interrupt
	if (err) {
		LOG_ERR("Communication error");
	}
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
}

uint8_t icm45_setup_WOM(void) // TODO: check if working
{
	int err = icm45_ext_drain_read_ahead();
	if (err) {
		goto fail;
	}
	// Disable FIFO acquisition and flush before entering WOM mode.
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_CONFIG3, 0x00);
	if (err) {
		goto fail;
	}
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_CONFIG0, 0x00);
	if (err) {
		goto fail;
	}
	uint8_t interrupts;
	err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_INT1_STATUS0, &interrupts);
	if (err) {
		goto fail;
	}
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_INT1_CONFIG0, 0x00);
	if (err) {
		goto fail;
	}
	err = ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		ICM45686_ACCEL_CONFIG0,
		ACCEL_UI_FS_SEL_8G << 4 | ACCEL_ODR_200Hz
	);
	if (err) {
		goto fail;
	}
	last_accel_mode = 0xff;
	last_gyro_mode = 0xff;
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_PWR_MGMT0, ACCEL_MODE_LP);
	if (err) {
		goto fail;
	}
	// ACCEL_LP_AVG_SEL=1x; leave the default AULP clock unchanged.
	err = icm45_bank_write_byte(ICM45686_IPREG_SYS2, ICM45686_IPREG_SYS2_REG_129, 0x00);
	if (err) {
		goto fail;
	}
	// Three individually spaced DATA writes: 7 x 3.9 mg is ~27.3 mg.
	const uint8_t thresholds[3] = {0x07, 0x07, 0x07};
	err = icm45_bank_write(ICM45686_IPREG_TOP1, ICM45686_ACCEL_WOM_X_THR, thresholds, sizeof(thresholds));
	if (err) {
		goto fail;
	}
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_TMST_WOM_CONFIG, 0x14);
	if (err) {
		goto fail;
	}
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_INT1_CONFIG1, 0x0E);
	if (err) {
		goto fail;
	}
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW;

fail:
	LOG_ERR("WOM setup failed: %d", err);
	return 0xFF;
}

/** Capture the terminal STATUS once: DONE is cleared by a STATUS read. */
static int icm45_i2cm_wait_done(uint8_t *status)
{
	int64_t deadline = k_uptime_get() + 10;
	for (;;) {
		int err = icm45_bank_read_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_STATUS, status);
		if (err) {
			return err;
		}
		if (!(*status & ICM45686_BIT_I2CM_STATUS_BUSY)) {
			return 0;
		}
		if (k_uptime_get() >= deadline) {
			return -ETIMEDOUT;
		}
	}
}

/** Wait and validate one transaction without re-reading either read-clear status. */
static int icm45_i2cm_complete(void)
{
	uint8_t status;
	int err = icm45_i2cm_wait_done(&status);
	if (err) {
		return err;
	}
	if (status & ICM45686_BIT_I2CM_STATUS_TIMEOUT_ERR) {
		return -ETIMEDOUT;
	}
	if (!(status & ICM45686_BIT_I2CM_STATUS_DONE)
		|| (status
			& (ICM45686_BIT_I2CM_STATUS_SDA_ERR | ICM45686_BIT_I2CM_STATUS_SCL_ERR
			   | ICM45686_BIT_I2CM_STATUS_SRST_ERR))) {
		return -EIO;
	}

	uint8_t dev_status;
	err = icm45_bank_read_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_EXT_DEV_STATUS, &dev_status);
	if (err) {
		return err;
	}
	return (dev_status & 0x01) ? -ENXIO : 0;
}

/** Drain even an uncertain GO before changing its command; discarded data is never reused. */
static int icm45_ext_drain_read_ahead(void)
{
	ext_read_ahead_active = false;
	uint8_t status;
	return icm45_i2cm_wait_done(&status);
}

static int icm45_ext_set_prefetch(bool enabled)
{
	int err = icm45_ext_drain_read_ahead();
	if (!err) {
		ext_prefetch_enabled = enabled;
	}
	return err;
}

int icm45_ext_write(const uint8_t addr, const uint8_t *buf, uint32_t num_bytes)
{
	if (num_bytes == 0 || num_bytes > 6 || buf == NULL) {
		return -EINVAL;
	}

	bool had_read_ahead = ext_prefetch_enabled && ext_read_ahead_active;
	uint8_t saved_addr = ext_read_ahead_addr;
	uint8_t saved_reg = ext_read_ahead_reg;
	uint8_t saved_len = ext_read_ahead_len;

	int err = icm45_ext_drain_read_ahead();
	if (err) {
		return err;
	}

	uint8_t dev_profile[2] = {0x00, addr};
	err = icm45_bank_write(ICM45686_IPREG_TOP1, ICM45686_DEV_PROFILE_0, dev_profile, 2);
	if (err) {
		return err;
	}
	err = icm45_bank_write(ICM45686_IPREG_TOP1, ICM45686_I2CM_WR_DATA_0, buf, num_bytes);
	if (err) {
		return err;
	}
	err = icm45_bank_write_byte(
		ICM45686_IPREG_TOP1,
		ICM45686_I2CM_COMMAND_0,
		ICM45686_I2CM_CMD_ENDFLAG | ICM45686_I2CM_CMD_RW_WRITE | num_bytes
	);
	if (err) {
		return err;
	}
	err = icm45_bank_write_byte(
		ICM45686_IPREG_TOP1,
		ICM45686_I2CM_CONTROL,
		ICM45686_I2CM_CONTROL_RESTART_EN | ICM45686_I2CM_CONTROL_GO
	);
	if (err) {
		return err;
	}
	err = icm45_i2cm_complete();
	if (err) {
		return err; // Never replay a write whose side effects may already have happened.
	}

	// Restore one-ahead reads only for sensors that permit background acquisition.
	if (had_read_ahead && ext_prefetch_enabled) {
		uint8_t rd_profile[2] = {saved_reg, saved_addr};
		err = icm45_bank_write(ICM45686_IPREG_TOP1, ICM45686_DEV_PROFILE_0, rd_profile, 2);
		if (err) {
			return err;
		}
		err = icm45_bank_write_byte(
			ICM45686_IPREG_TOP1,
			ICM45686_I2CM_COMMAND_0,
			ICM45686_I2CM_CMD_ENDFLAG | ICM45686_I2CM_CMD_RW_READ_REG | saved_len
		);
		if (err) {
			return err;
		}
		err = icm45_bank_write_byte(
			ICM45686_IPREG_TOP1,
			ICM45686_I2CM_CONTROL,
			ICM45686_I2CM_CONTROL_RESTART_EN | ICM45686_I2CM_CONTROL_GO
		);
		if (err) {
			return err;
		}
		ext_read_ahead_active = true;
		ext_read_ahead_addr = saved_addr;
		ext_read_ahead_reg = saved_reg;
		ext_read_ahead_len = saved_len;
	}
	return 0;
}

int icm45_ext_write_read(const uint8_t addr, const void *write_buf, size_t num_write, void *read_buf, size_t num_read)
{
	if (num_write != 1 || num_read < 1 || num_read > 15 || write_buf == NULL || read_buf == NULL) {
		return -EINVAL;
	}

	uint8_t reg_addr = ((const uint8_t *)write_buf)[0];
	bool matching = ext_prefetch_enabled && ext_read_ahead_active && addr == ext_read_ahead_addr
				 && reg_addr == ext_read_ahead_reg && num_read == ext_read_ahead_len;
	// Consume cache ownership before any fallible I/O, including terminal status.
	ext_read_ahead_active = false;
	int err;
	if (!matching) {
		err = icm45_ext_drain_read_ahead();
		if (err) {
			return err;
		}
		uint8_t dev_profile[2] = {reg_addr, addr};
		err = icm45_bank_write(ICM45686_IPREG_TOP1, ICM45686_DEV_PROFILE_0, dev_profile, 2);
		if (err) {
			return err;
		}
		err = icm45_bank_write_byte(
			ICM45686_IPREG_TOP1,
			ICM45686_I2CM_COMMAND_0,
			ICM45686_I2CM_CMD_ENDFLAG | ICM45686_I2CM_CMD_RW_READ_REG | num_read
		);
		if (err) {
			return err;
		}
		err = icm45_bank_write_byte(
			ICM45686_IPREG_TOP1,
			ICM45686_I2CM_CONTROL,
			ICM45686_I2CM_CONTROL_RESTART_EN | ICM45686_I2CM_CONTROL_GO
		);
		if (err) {
			return err;
		}
	}

	err = icm45_i2cm_complete();
	if (err) {
		return err;
	}
	err = icm45_bank_read(ICM45686_IPREG_TOP1, ICM45686_I2CM_RD_DATA_0, read_buf, num_read);
	if (err) {
		return err;
	}

	if (!ext_scanning_mode && ext_prefetch_enabled) {
		err = icm45_bank_write_byte(
			ICM45686_IPREG_TOP1,
			ICM45686_I2CM_CONTROL,
			ICM45686_I2CM_CONTROL_RESTART_EN | ICM45686_I2CM_CONTROL_GO
		);
		// A failed GO may have reached hardware. Report it and drain on the next call.
		if (err) {
			return err;
		}
		ext_read_ahead_active = true;
		ext_read_ahead_addr = addr;
		ext_read_ahead_reg = reg_addr;
		ext_read_ahead_len = num_read;
	}
	return 0;
}

const sensor_ext_ssi_t sensor_ext_icm45686 = {
	.ext_write = icm45_ext_write,
	.ext_write_read = icm45_ext_write_read,
	.ext_burst = 15,
	.ext_set_prefetch = icm45_ext_set_prefetch,
};

int icm45_ext_setup(enum sensor_ext_mode mode)
{
	int err = icm45_ext_drain_read_ahead();
	if (err) {
		return err;
	}

	if (mode == SENSOR_EXT_MODE_OFF || mode == SENSOR_EXT_MODE_I2C_PASSTHROUGH) {
		err = ssi_reg_write_byte(
			SENSOR_INTERFACE_DEV_IMU,
			ICM45686_IOC_PAD_SCENARIO_AUX_OVRD,
			mode == SENSOR_EXT_MODE_I2C_PASSTHROUGH ? 0x18 : 0x00
		);
		if (err) {
			LOG_ERR("Communication error");
		}
		return err;
	}

	if (mode != SENSOR_EXT_MODE_I2CM_PROXY) {
		return -EINVAL;
	}

	// Select the gyro oscillator below. If gyro is off after WOM/shutdown,
	// use standby during discovery; retain the existing 1 ms startup delay.
	uint8_t pwr = 0;
	err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_PWR_MGMT0, &pwr);
	if (err) {
		return err;
	}
	uint8_t gyro_mode = (pwr >> 2) & 0x03;
	if (gyro_mode == GYRO_MODE_OFF) {
		pwr = (pwr & ~(0x03 << 2)) | (GYRO_MODE_STANDBY << 2);
		err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_PWR_MGMT0, pwr);
		last_accel_mode = 0xff;
		last_gyro_mode = 0xff;
		if (err) {
			return err;
		}
		k_msleep(1);
	}

	err = icm45_bank_write_byte(ICM45686_IPREG_BAR, ICM45686_IPREG_BAR_REG_60, ICM45686_BIT_AUX1_I2CM_MODE);
	if (err) {
		return err;
	}
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_IOC_PAD_SCENARIO_AUX_OVRD, 0x17);
	if (err) {
		return err;
	}
	err = ssi_reg_update_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_REG_MISC1, 0x0F, 0x02);
	if (err) {
		LOG_ERR("Communication error");
		return err;
	}

	ext_scanning_mode = true;
	ext_prefetch_enabled = true;
	sensor_interface_ext_configure(&sensor_ext_icm45686);
	return 0;
}

const sensor_imu_t sensor_imu_icm45686 = {
	icm45_init,
	icm45_shutdown,

	icm45_update_fs,
	icm45_update_odr,

	icm45_fifo_read,
	icm45_fifo_process,
	icm45_accel_read,
	icm45_gyro_read,
	icm45_temp_read,

	icm45_setup_DRDY,
	icm45_setup_WOM,

	icm45_ext_setup,
};
