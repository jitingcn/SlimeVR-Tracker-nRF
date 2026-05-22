#include <math.h>

#include <zephyr/logging/log.h>
#include <hal/nrf_gpio.h>

#include "ICM45686.h"
#include "sensor/sensor_none.h"

#define PACKET_SIZE 20

static const float accel_sensitivity = 16.0f / 32768.0f; // Always 16G
static const float gyro_sensitivity = 2000.0f / 32768.0f; // Always 2000dps

static const float accel_sensitivity_32 = 32.0f / ((uint32_t)2<<30); // 32G forced
static const float gyro_sensitivity_32 = 4000.0f / ((uint32_t)2<<30); // 4000dps forced

static const uint16_t times[] = {6400, 3200, 1600, 800, 400, 200, 100, 50, 25, 2, 0};
static const uint8_t odrs[] = {ACCEL_ODR_6_4kHz, ACCEL_ODR_3_2kHz, ACCEL_ODR_1_6kHz, ACCEL_ODR_800Hz, ACCEL_ODR_400Hz, ACCEL_ODR_200Hz, ACCEL_ODR_100Hz, ACCEL_ODR_50Hz, ACCEL_ODR_25Hz, ACCEL_ODR_12_5Hz};

static uint8_t last_accel_odr = 0xff;
static uint8_t last_gyro_odr = 0xff;
static const float clock_reference = 32000;
static float clock_scale = 1; // ODR is scaled by clock_rate/clock_reference

// I2CM pre-triggered continuous read mode state
// When active, icm45_ext_write_read() reads cached I2CM_RD_DATA directly
// instead of performing a full one-shot I2CM cycle (~500us)
static bool ext_continuous_active = false;
static uint8_t ext_cont_addr = 0;
static uint8_t ext_cont_sub = 0;
static uint8_t ext_cont_len = 0;
static bool ext_scanning_mode = true;
static void icm45_ext_stop_continuous(void);

// Cache the latest FIFO temperature so temperature reads can stay synchronized
// with the current accel/gyro batch when FIFO packets are available.
static float fifo_temp = 25.0f;
static bool fifo_temp_valid = false;

static void icm45_cache_fifo_temp(const uint8_t *data, uint16_t packets)
{
	fifo_temp_valid = false;
	int32_t raw_temp_sum = 0;
	uint16_t valid_packets = 0;

	for (uint16_t i = 0; i < packets; i++)
	{
		const uint8_t *packet = &data[i * PACKET_SIZE];
		if (packet[0] != 0x78 && packet[0] != 0x7A)
			continue;

		int16_t raw_temp = (int16_t)((((uint16_t)packet[13]) << 8) | packet[14]);
		raw_temp_sum += raw_temp;
		valid_packets++;
	}

	if (valid_packets == 0)
		return;

	fifo_temp = ((float)raw_temp_sum / (128.0f * valid_packets)) + 25.0f;
	fifo_temp_valid = true;
}

LOG_MODULE_REGISTER(ICM45686, LOG_LEVEL_DBG);

// IREG helpers for runtime verification.
// Note: ICM45686 host access requires writing IREG_ADDR_15_8/IREG_ADDR_7_0 then reading IREG_DATA.
// In this codebase we only expose ICM45686_IREG_ADDR_15_8 (0x7C) and ICM45686_IREG_DATA (0x7E).
// The low byte (IREG_ADDR_7_0) is auto-incremented and accessible as 0x7D.
#define ICM45686_IREG_ADDR_7_0 0x7D

static int icm45686_ireg_read8(uint16_t ireg_addr, uint8_t *out)
{
	uint8_t addr_hi = (uint8_t)(ireg_addr >> 8);
	uint8_t addr_lo = (uint8_t)(ireg_addr & 0xFF);
	int err = 0;

	// Read must also be in a single burst-write for address set to avoid unintended prefetch.
	// Write (IREG_ADDR_15_8, IREG_ADDR_7_0) in one burst, then read IREG_DATA.
	uint8_t addr_buf[2] = {addr_hi, addr_lo};
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, addr_buf, sizeof(addr_buf));
	k_busy_wait(5);

	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_DATA, out);
	k_busy_wait(5);
	return err;
}

static int icm45686_ireg_write8(uint16_t ireg_addr, uint8_t value)
{
	// IMPORTANT:
	// Datasheet note in [`ICM45686.h`](SlimeVR-Tracker-nRF/src/sensor/imu/ICM45686.h:69) says:
	// "The above programming steps must be performed in a single burst-write transaction"
	// to avoid unintended read prefetch.
	// So we write (IREG_ADDR_15_8, IREG_ADDR_7_0, IREG_DATA) in one SPI burst.
	uint8_t addr_hi = (uint8_t)(ireg_addr >> 8);
	uint8_t addr_lo = (uint8_t)(ireg_addr & 0xFF);
	uint8_t buf[3] = {addr_hi, addr_lo, value};
	int err = ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, buf, sizeof(buf));
	k_busy_wait(5);
	return err;
}

static void icm45686_set_src_ctrl(uint8_t gyro_sel, uint8_t accel_sel)
{
	// GYRO_SRC_CTRL: IPREG_SYS1_REG_166 (0x00A6) bits [6:5]
	// ACCEL_SRC_CTRL: IPREG_SYS2_REG_123 (0x007B) bits [1:0]
	// Encoding (per user guide):
	// 0: Interpolator and FIR filter off
	// 1: Interpolator off and FIR filter on
	// 2: Interpolator on and FIR filter on
	const uint16_t IREG_SYS1_GYRO_SRC_CTRL = 0xA400u + 0x00A6u;
	const uint16_t IREG_SYS2_ACCEL_SRC_CTRL = 0xA500u + 0x007Bu;

	gyro_sel &= 0x03;   // Limit to 2 bits
	accel_sel &= 0x03;  // Limit to 2 bits

	uint8_t gyro_reg = 0, accel_reg = 0;
	int err = 0;

	err |= icm45686_ireg_read8(IREG_SYS1_GYRO_SRC_CTRL, &gyro_reg);
	err |= icm45686_ireg_read8(IREG_SYS2_ACCEL_SRC_CTRL, &accel_reg);
	if (err)
	{
		LOG_WRN("SRC_CTRL pre-read failed (err=%d)", err);
		return;
	}

	uint8_t gyro_new = (uint8_t)((gyro_reg & ~(0x03u << 5)) | (uint8_t)(gyro_sel << 5));
	uint8_t accel_new = (uint8_t)((accel_reg & ~0x03u) | accel_sel);

	LOG_INF("SRC_CTRL set: GYRO 0x%02X->0x%02X (sel=%u), ACCEL 0x%02X->0x%02X (sel=%u)",
		gyro_reg, gyro_new, gyro_sel, accel_reg, accel_new, accel_sel);

	err |= icm45686_ireg_write8(IREG_SYS1_GYRO_SRC_CTRL, gyro_new);
	err |= icm45686_ireg_write8(IREG_SYS2_ACCEL_SRC_CTRL, accel_new);
	if (err)
		LOG_WRN("SRC_CTRL write failed (err=%d)", err);

	uint8_t gyro_rb = 0, accel_rb = 0;
	int rerr = 0;
	rerr |= icm45686_ireg_read8(IREG_SYS1_GYRO_SRC_CTRL, &gyro_rb);
	rerr |= icm45686_ireg_read8(IREG_SYS2_ACCEL_SRC_CTRL, &accel_rb);
	if (!rerr)
		LOG_INF("SRC_CTRL post-read: GYRO=0x%02X (sel=%u) ACCEL=0x%02X (sel=%u)",
			gyro_rb, (gyro_rb >> 5) & 0x03, accel_rb, accel_rb & 0x03);
	else
		LOG_WRN("SRC_CTRL post-read failed (err=%d)", rerr);
}

static void icm45686_set_ui_lpfbw_sel(uint8_t gyro_sel, uint8_t accel_sel)
{
	// Gyro UI LPF BW: IPREG_SYS1_REG_172 (0x00AC) bits [2:0]
	// Accel UI LPF BW: IPREG_SYS2_REG_131 (0x0083) bits [2:0]
	// 0=bypass, 1=ODR/4, 2=ODR/8, 3=ODR/16, 4=ODR/32, 5=ODR/64, 6=ODR/128, 7=ODR/128
	const uint16_t IREG_SYS1_GYRO_UI_LPFBW = 0xA400u + 0x00ACu;
	const uint16_t IREG_SYS2_ACCEL_UI_LPFBW = 0xA500u + 0x0083u;

	gyro_sel &= 0x07;
	accel_sel &= 0x07;

	uint8_t gyro_reg = 0, accel_reg = 0;
	int err = 0;
	err |= icm45686_ireg_read8(IREG_SYS1_GYRO_UI_LPFBW, &gyro_reg);
	err |= icm45686_ireg_read8(IREG_SYS2_ACCEL_UI_LPFBW, &accel_reg);
	if (err)
	{
		LOG_WRN("UI_LPFBW pre-read failed (err=%d)", err);
		return;
	}

	uint8_t gyro_new = (uint8_t)((gyro_reg & ~0x07u) | gyro_sel);
	uint8_t accel_new = (uint8_t)((accel_reg & ~0x07u) | accel_sel);

	LOG_INF("UI_LPFBW set: GYRO 0x%02X->0x%02X (sel=%u), ACCEL 0x%02X->0x%02X (sel=%u)",
		gyro_reg, gyro_new, gyro_sel, accel_reg, accel_new, accel_sel);

	err |= icm45686_ireg_write8(IREG_SYS1_GYRO_UI_LPFBW, gyro_new);
	err |= icm45686_ireg_write8(IREG_SYS2_ACCEL_UI_LPFBW, accel_new);
	if (err)
		LOG_WRN("UI_LPFBW write failed (err=%d)", err);

	uint8_t gyro_rb = 0, accel_rb = 0;
	int rerr = 0;
	rerr |= icm45686_ireg_read8(IREG_SYS1_GYRO_UI_LPFBW, &gyro_rb);
	rerr |= icm45686_ireg_read8(IREG_SYS2_ACCEL_UI_LPFBW, &accel_rb);
	if (!rerr)
		LOG_INF("UI_LPFBW post-read: GYRO=0x%02X (sel=%u) ACCEL=0x%02X (sel=%u)",
			gyro_rb, gyro_rb & 0x07, accel_rb, accel_rb & 0x07);
	else
		LOG_WRN("UI_LPFBW post-read failed (err=%d)", rerr);
}


// Bank read/write helpers for IREG-based register access with auto-increment
static int icm45_bank_write(uint8_t bank, uint8_t reg, const uint8_t *buf, uint32_t num_bytes)
{
	if (num_bytes == 0)
		return -1;
	int err = 0;
	uint8_t ireg_buf[3] = {bank, reg, buf[0]};
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, ireg_buf, 3);
	k_busy_wait(4);
	for (uint32_t i = 1; i < num_bytes; i++)
	{
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_DATA, buf[i]);
		k_busy_wait(4);
	}
	return err;
}

static int icm45_bank_write_byte(uint8_t bank, uint8_t reg, uint8_t value)
{
	return icm45_bank_write(bank, reg, &value, 1);
}

static int icm45_bank_read(uint8_t bank, uint8_t reg, uint8_t *buf, uint32_t num_bytes)
{
	if (num_bytes == 0)
		return -1;
	int err = 0;
	uint8_t ireg_buf[2] = {bank, reg};
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, ireg_buf, 2);
	k_busy_wait(4);
	for (uint32_t i = 0; i < num_bytes; i++)
	{
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_DATA, &buf[i]);
		k_busy_wait(4);
	}
	return err;
}

static int icm45_bank_read_byte(uint8_t bank, uint8_t reg, uint8_t *value)
{
	return icm45_bank_read(bank, reg, value, 1);
}

int icm45_init(float clock_rate, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	// After init, switch to operational mode for immediate continuous I2CM reads
	ext_scanning_mode = false;
	ext_continuous_active = false;
	fifo_temp = 25.0f;
	fifo_temp_valid = false;

	// setup interface for SPI
	sensor_interface_spi_configure(SENSOR_INTERFACE_DEV_IMU, MHZ(24), 0);

	int err = 0;

	// sensor_init() already issued shutdown/reset before calling init.
	// Continue from the post-reset state and rebuild runtime configuration below.

	// Read WHO_AM_I to verify communication
	uint8_t who_am_i = 0;
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, 0x72, &who_am_i); // WHO_AM_I register
	LOG_INF("WHO_AM_I = 0x%02X (expected 0xE9/0xE7)", who_am_i);
	if (who_am_i != 0xE9 && who_am_i != 0xE7)
	{
		LOG_ERR("Invalid WHO_AM_I value");
		return -1;
	}

	if (clock_rate > 0)
	{
		clock_scale = clock_rate / clock_reference;
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_IOC_PAD_SCENARIO_OVRD, 0x06); // override pin 9 to CLKIN
		err |= ssi_reg_update_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_RTC_CONFIG, 0x20, 0x20); // enable external CLKIN
//		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_RTC_CONFIG, 0x23); // enable external CLKIN (0x20, default register value is 0x03)
	}
	uint8_t ireg_buf[3];
	ireg_buf[0] = ICM45686_IPREG_BAR; // address is a word, icm is big endian
	ireg_buf[1] = ICM45686_IPREG_BAR_REG_58;
	ireg_buf[2] = 0xD9 & ~0x48; // disable internal pull resistors for AP pins (pin 13, 12)
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, ireg_buf, 3); // write buffer
	ireg_buf[1] = ICM45686_IPREG_BAR_REG_59;
	ireg_buf[2] = 0xB6 & ~0x92; // disable internal pull resistors for AP pins (pin 7, 1, 14)
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, ireg_buf, 3); // write buffer
	// Re-enable I2CM mode after the pre-init shutdown reset (if ext was configured during scan)
	ireg_buf[1] = ICM45686_IPREG_BAR_REG_60;
	ireg_buf[2] = ICM45686_BIT_AUX1_I2CM_MODE; // I2CM mode only, no internal pull-ups
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, ireg_buf, 3);
	// Enable AUX1 in I2CM Master mode
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_IOC_PAD_SCENARIO_AUX_OVRD, 0x17);
	// OSC_ID_OVRD must be 1 or 2 (if gyro is enabled) for I2CM operation
	err |= ssi_reg_update_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_REG_MISC1, 0x0F, 0x02);

	ireg_buf[0] = ICM45686_IPREG_TOP1; // address is a word, icm is big endian
	ireg_buf[1] = ICM45686_SREG_CTRL;
	ireg_buf[2] = 0x02; // set big endian
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, ireg_buf, 3); // write buffer


	// Power on sensors first (PWR_MGMT0 only)
	uint8_t pwr_mgmt = GYRO_MODE_LN << 2 | ACCEL_MODE_LN; // Both in Low Noise mode
	LOG_INF("PWR_MGMT0 write = 0x%02X (powering on sensors)", pwr_mgmt);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_PWR_MGMT0, pwr_mgmt);

	// Wait for gyro startup BEFORE configuring ODR
	LOG_INF("Waiting 50ms for gyroscope startup...");
	k_msleep(30); // datasheet minimum (30ms)

	// Check sensor status
	uint8_t status = 0;
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_INT1_STATUS0, &status);
	LOG_INF("INT1_STATUS0 after startup = 0x%02X", status);

	// Now configure ODR after sensors are fully powered on
	last_accel_odr = 0xff; // reset last odr
	last_gyro_odr = 0xff; // reset last odr
	err |= icm45_update_odr(accel_time, gyro_time, accel_actual_time, gyro_actual_time);

	// Configure SRC_CTRL: 0=off, 1=AAF only, 2=AAF+Interpolator
	icm45686_set_src_ctrl(1u, 1u);

	// Configure UI LPF bandwidth: 0=bypass, 1=ODR/4, 2=ODR/8, 3=ODR/16, etc.
	icm45686_set_ui_lpfbw_sel(1u, 1u);

	// Finally enable FIFO
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_CONFIG0, 0x40 | 0b000111); // set FIFO streaming mode (not stop-on-full), set FIFO depth to 2K bytes (see AN-000364)

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_CONFIG3, 0x0F); // begin FIFO stream, hires, a+g

	// Verify external CLKIN is actually working by checking FIFO output
	if (clock_rate > 0)
	{
		k_msleep(6); // wait for FIFO samples to accumulate
		uint8_t rawCount[2];
		ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_COUNT_0, rawCount, 2);
		uint16_t fifo_count = (uint16_t)(rawCount[0] << 8 | rawCount[1]);
		if (fifo_count == 0)
		{
			LOG_WRN("External CLKIN not working, falling back to internal clock");
			clock_scale = 1;
			// Disable CLKIN: revert pad scenario and RTC config
			err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_IOC_PAD_SCENARIO_OVRD, 0x00);
			err |= ssi_reg_update_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_RTC_CONFIG, 0x20, 0x00);
			// Recalculate ODR without clock scaling
			last_accel_odr = 0xff;
			last_gyro_odr = 0xff;
			err |= icm45_update_odr(accel_time, gyro_time, accel_actual_time, gyro_actual_time);
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

void icm45_shutdown(void)
{
	icm45_ext_stop_continuous();
	last_accel_odr = 0xff; // reset last odr
	last_gyro_odr = 0xff; // reset last odr
	fifo_temp = 25.0f;
	fifo_temp_valid = false;
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_REG_MISC2, 0x02); // Soft reset
	k_msleep(2); // Wait for reset to complete (datasheet: 1ms) - ensures clean state for init
	// TODO: not working
//	uint8_t ireg_buf[3];
//	ireg_buf[1] = ICM45686_IPREG_BAR_REG_60;
//	ireg_buf[2] = 0x6D & ~0x05; // set internal pull down resistors for AP pins (pin 10, 7)
//	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, ireg_buf, 3); // write buffer
//	ireg_buf[1] = ICM45686_IPREG_BAR_REG_61;
//	ireg_buf[2] = 0xBB & ~0x10; // set internal pull down resistors for AP pins (pin 11)
//	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, ireg_buf, 3); // write buffer
	if (err)
		LOG_ERR("Communication error");
}

void icm45_update_fs(float accel_range, float gyro_range, float *accel_actual_range, float *gyro_actual_range)
{
	// ICM45686 only supports 32G and 4000dps in high-res mode
	*accel_actual_range = 32; // always 32g in hires
	*gyro_actual_range = 4000; // always 4000dps in hires
}

int icm45_update_odr(float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	int ODR;
	// ICM45686 only supports 32G and 4000dps in high-res mode
	uint8_t ACCEL_UI_FS_SEL = ACCEL_UI_FS_SEL_32G;
	uint8_t GYRO_UI_FS_SEL = GYRO_UI_FS_SEL_4000DPS;
	uint8_t ACCEL_MODE;
	uint8_t GYRO_MODE;
	uint8_t ACCEL_ODR = 0;
	uint8_t GYRO_ODR = 0;

	// Calculate accel
	if (accel_time <= 0 || accel_time == INFINITY) // off, standby interpreted as off
	{
		ACCEL_MODE = ACCEL_MODE_OFF;
		accel_time = 0; // off
	}
	else
	{
		ACCEL_MODE = ACCEL_MODE_LN;
		ODR = 1 / accel_time;
		ODR /= clock_scale; // scale clock
		for (int i = 1; i < ARRAY_SIZE(times); i++)
		{
			if (ODR <= (i > 9 ? times[i] / 25.0 : times[i]))
				continue;
			ACCEL_ODR = odrs[i - 1];
			accel_time = i > 10 ? times[i - 1] / 25.0 : 1.0 / times[i - 1];
			break;
		}
	}
	accel_time /= clock_scale; // scale clock

	// Calculate gyro
	if (gyro_time <= 0) // off
	{
		GYRO_MODE = GYRO_MODE_OFF;
		gyro_time = 0; // off
	}
	else if (gyro_time == INFINITY) // standby
	{
		GYRO_MODE = GYRO_MODE_STANDBY;
		gyro_time = 0; // off
	}
	else
	{
		GYRO_MODE = GYRO_MODE_LN;
		ODR = 1 / gyro_time;
		ODR /= clock_scale; // scale clock
		for (int i = 1; i < ARRAY_SIZE(times); i++)
		{
			if (ODR <= (i > 9 ? times[i] / 25.0 : times[i]))
				continue;
			GYRO_ODR = odrs[i - 1];
			gyro_time = i > 10 ? times[i - 1] / 25.0 : 1.0 / times[i - 1];
			break;
		}
	}
	gyro_time /= clock_scale; // scale clock

	if (last_accel_odr == ACCEL_ODR && last_gyro_odr == GYRO_ODR) // if both were already configured
		return 1;

	int err = 0;
	// only if the power mode has changed
	if (last_accel_odr == 0xff || last_gyro_odr == 0xff || (last_accel_odr == 0 ? 0 : 1) != (ACCEL_ODR == 0 ? 0 : 1) || (last_gyro_odr == 0 ? 0 : 1) != (GYRO_ODR == 0 ? 0 : 1))
	{ // TODO: can't tell difference between gyro off and gyro standby
		uint8_t pwr_mgmt = GYRO_MODE << 2 | ACCEL_MODE;
		LOG_INF("PWR_MGMT0 write = 0x%02X (GYRO_MODE=%d, ACCEL_MODE=%d)", pwr_mgmt, GYRO_MODE, ACCEL_MODE);
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_PWR_MGMT0, pwr_mgmt); // set accel and gyro modes
		k_busy_wait(250); // wait >200us // TODO: is this needed?
		// Read back to verify
		uint8_t pwr_mgmt_readback = 0;
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_PWR_MGMT0, &pwr_mgmt_readback);
		LOG_INF("PWR_MGMT0 readback = 0x%02X", pwr_mgmt_readback);
	}
	last_accel_odr = ACCEL_ODR;
	last_gyro_odr = GYRO_ODR;

	uint8_t accel_config = ACCEL_UI_FS_SEL << 4 | ACCEL_ODR;
	uint8_t gyro_config = GYRO_UI_FS_SEL << 4 | GYRO_ODR;
	LOG_INF("ACCEL_CONFIG0 write = 0x%02X (ODR=%d), GYRO_CONFIG0 write = 0x%02X (ODR=%d)",
		accel_config, ACCEL_ODR, gyro_config, GYRO_ODR);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_ACCEL_CONFIG0, accel_config); // set accel ODR and FS
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_GYRO_CONFIG0, gyro_config); // set gyro ODR and FS
	// Read back to verify
	uint8_t accel_config_readback = 0, gyro_config_readback = 0;
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_ACCEL_CONFIG0, &accel_config_readback);
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_GYRO_CONFIG0, &gyro_config_readback);
	LOG_INF("ACCEL_CONFIG0 readback = 0x%02X, GYRO_CONFIG0 readback = 0x%02X",
		accel_config_readback, gyro_config_readback);
	if (err)
		LOG_ERR("Communication error");

	*accel_actual_time = accel_time;
	*gyro_actual_time = gyro_time;

	return 0;
}

uint16_t icm45_fifo_read(uint8_t *data, uint16_t len)
{
	int err = 0;
	uint8_t rawCount[2];
	err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_COUNT_0, &rawCount[0], 2);
	uint16_t packets = (uint16_t)(rawCount[0] << 8 | rawCount[1]); // Turn the 16 bits into a unsigned 16-bit value

	fifo_temp_valid = false;
	if (packets == 0)
		return 0;

	// Limit to buffer size - no phantom packets, no multipliers
	uint16_t limit = len / PACKET_SIZE;
	if (packets > limit)
	{
		LOG_WRN("FIFO read buffer limit reached, %d packets dropped", packets - limit);
		packets = limit;
	}

	// Read exactly what FIFO_COUNT said - single read, no loops
	uint16_t count = packets * PACKET_SIZE;
	err |= ssi_burst_read_interval(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_DATA, data, count, PACKET_SIZE);
	if (err)
		LOG_ERR("Communication error");
	else
		icm45_cache_fifo_temp(data, packets);

	return packets;
}

static const uint8_t invalid[6] = {0x80, 0x00, 0x80, 0x00, 0x80, 0x00};

int icm45_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3])
{
	index *= PACKET_SIZE;
	// Accept both 0x78 and 0x7A headers (0x7A has timestamp bit set)
	if (data[index] != 0x78 && data[index] != 0x7A)
	{
		LOG_WRN("Invalid FIFO header: 0x%02X (expected 0x78 or 0x7A)", data[index]);
		return 1; // Skip invalid header
	}

	// Debug: Log first packet gyro data bytes to see what we're getting
	static int debug_count = 0;
	if (debug_count < 3)
	{
		LOG_INF("FIFO packet[%d]: header=0x%02X, gyro bytes: %02X %02X %02X %02X %02X %02X",
			debug_count, data[index],
			data[index+7], data[index+8], data[index+9],
			data[index+10], data[index+11], data[index+12]);
		debug_count++;
	}
	// Empty packet is 7F filled
	// combine into 20 bit values in 32 bit int
	float a_raw[3] = {0};
	float g_raw[3] = {0};
	if (memcmp(&data[index + 1], invalid, sizeof(invalid))) // valid accel data
	{
		for (int i = 0; i < 3; i++) // accel x, y, z
			a_raw[i] = (int32_t)((((uint32_t)data[index + 1 + (i * 2)]) << 24) | (((uint32_t)data[index + 2 + (i * 2)]) << 16) | (((uint32_t)data[index + 17 + i] & 0xF0) << 8));
	}
	if (memcmp(&data[index + 7], invalid, sizeof(invalid))) // valid gyro data
	{
		for (int i = 0; i < 3; i++) // gyro x, y, z
			g_raw[i] = (int32_t)((((uint32_t)data[index + 7 + (i * 2)]) << 24) | (((uint32_t)data[index + 8 + (i * 2)]) << 16) | (((uint32_t)data[index + 17 + i] & 0x0F) << 12));
	}
	else
	{
		static int gyro_invalid_count = 0;
		if (gyro_invalid_count < 5)
		{
			LOG_WRN("Gyro data marked as invalid (0x80 pattern)");
			gyro_invalid_count++;
		}
		if (!memcmp(&data[index + 1], invalid, sizeof(invalid))) // Skip invalid data (both accel and gyro invalid)
		{
			return 1;
		}
	}
	for (int i = 0; i < 3; i++) // x, y, z
	{
		a_raw[i] *= accel_sensitivity_32;
		g_raw[i] *= gyro_sensitivity_32;
	}
	memcpy(a, a_raw, sizeof(a_raw));
	memcpy(g, g_raw, sizeof(g_raw));
	return 0;
}

void icm45_accel_read(float a[3])
{
	uint8_t rawAccel[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, ICM45686_ACCEL_DATA_X1_UI, &rawAccel[0], 6);
	if (err)
		LOG_ERR("Communication error");
	for (int i = 0; i < 3; i++) // x, y, z
	{
		a[i] = (int16_t)((((uint16_t)rawAccel[i * 2]) << 8) | rawAccel[1 + (i * 2)]);
		a[i] *= accel_sensitivity;
	}
}

void icm45_gyro_read(float g[3])
{
	uint8_t rawGyro[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, ICM45686_GYRO_DATA_X1_UI, &rawGyro[0], 6);
	if (err)
		LOG_ERR("Communication error");
	for (int i = 0; i < 3; i++) // x, y, z
	{
		g[i] = (int16_t)((((uint16_t)rawGyro[i * 2]) << 8) | rawGyro[1 + (i * 2)]);
		g[i] *= gyro_sensitivity;
	}
}

float icm45_temp_read(void)
{
	if (fifo_temp_valid)
		return fifo_temp;

	uint8_t rawTemp[2];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, ICM45686_TEMP_DATA1_UI, &rawTemp[0], 2);
	if (err)
		LOG_ERR("Communication error");
	// Temperature in Degrees Centigrade = (TEMP_DATA / 128) + 25
	float temp = (int16_t)((((uint16_t)rawTemp[0]) << 8) | rawTemp[1]);
	temp /= 128;
	temp += 25;
	return temp;
}

uint8_t icm45_setup_DRDY(uint16_t threshold)
{
	uint8_t buf[2];
	buf[0] = threshold & 0xFF;
	buf[1] = threshold >> 8;
	int err = ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_CONFIG1_0, buf, 2);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_INT1_CONFIG0, 0x02); // FIFO threshold interrupt
	if (err)
		LOG_ERR("Communication error");
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
}

uint8_t icm45_setup_WOM(void) // TODO: check if working
{
	// Disable FIFO streaming before entering WOM mode.
	// Prevents stale data accumulation and FIFO trigger storm after wake.
	ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_CONFIG3, 0x00); // stop FIFO streaming
	ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_CONFIG0, 0x00); // bypass mode flushes FIFO

	uint8_t interrupts;
	uint8_t ireg_buf[5];
	int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_INT1_STATUS0, &interrupts); // clear reset done int flag // TODO: is this needed
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_INT1_CONFIG0, 0x00); // disable default interrupt (RESET_DONE)
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_ACCEL_CONFIG0, ACCEL_UI_FS_SEL_8G << 4 | ACCEL_ODR_200Hz); // set accel ODR and FS
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_PWR_MGMT0, ACCEL_MODE_LP); // set accel and gyro modes
	ireg_buf[0] = ICM45686_IPREG_SYS2; // address is a word, icm is big endian
	ireg_buf[1] = ICM45686_IPREG_SYS2_REG_129;
	ireg_buf[2] = 0x00; // set ACCEL_LP_AVG_SEL to 1x
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, ireg_buf, 3); // write buffer
	// should already be defaulted to AULP
//	ireg_buf[0] = ICM45686_IPREG_TOP1;
//	ireg_buf[1] = ICM45686_SMC_CONTROL_0;
//	ireg_buf[2] = 0x60; // set ACCEL_LP_CLK_SEL to AULP
//	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, ireg_buf, 3); // write buffer
	ireg_buf[0] = ICM45686_IPREG_TOP1;
	ireg_buf[1] = ICM45686_ACCEL_WOM_X_THR;
	ireg_buf[2] = 0x07; // set wake thresholds // 7 x 3.9 mg is ~27.3 mg
	ireg_buf[3] = 0x07; // set wake thresholds
	ireg_buf[4] = 0x07; // set wake thresholds
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, ireg_buf, 5); // write buffer
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_TMST_WOM_CONFIG, 0x14); // enable WOM, enable WOM interrupt
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_INT1_CONFIG1, 0x0E); // route WOM interrupt
	if (err)
		LOG_ERR("Communication error");
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
}

/** Wait for I2CM to become idle */
static int icm45_i2cm_wait_done(void)
{
	uint8_t status = 0;
	int timeout = 1000;
	do {
		icm45_bank_read_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_STATUS, &status);
		if (--timeout <= 0)
			break;
	} while (status & ICM45686_BIT_I2CM_STATUS_BUSY);
	return (timeout > 0) ? 0 : -1;
}

/** Stop continuous I2CM read mode if active */
static void icm45_ext_stop_continuous(void)
{
	if (!ext_continuous_active)
		return;
	icm45_i2cm_wait_done();
	ext_continuous_active = false;
}

int icm45_ext_write(const uint8_t addr, const uint8_t *buf, uint32_t num_bytes)
{
	if (num_bytes > 6)
	{
		LOG_ERR("Unsupported write: %d bytes (max 6)", num_bytes);
		return -1;
	}

	// Save continuous state before stopping (we'll restore it after write)
	bool was_continuous = ext_continuous_active;
	uint8_t saved_addr = ext_cont_addr;
	uint8_t saved_sub = ext_cont_sub;
	uint8_t saved_len = ext_cont_len;

	// Stop continuous mode before writing (I2CM config will be changed)
	icm45_ext_stop_continuous();

	int retries = 2;
	int err;
	uint8_t dev_profile[2];
	uint8_t status;
	uint8_t dev_status;
	int timeout;

retry_write:
	err = 0;
	// DEV_PROFILE_0 = 0x00 (not used for write), DEV_PROFILE_1 = slave address
	dev_profile[0] = 0x00;
	dev_profile[1] = addr;
	err |= icm45_bank_write(ICM45686_IPREG_TOP1, ICM45686_DEV_PROFILE_0, dev_profile, 2);
	// Write data to WR_DATA registers
	err |= icm45_bank_write(ICM45686_IPREG_TOP1, ICM45686_I2CM_WR_DATA_0, buf, num_bytes);
	// Command: last transaction, write mode, num_bytes
	err |= icm45_bank_write_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_COMMAND_0,
								 ICM45686_I2CM_CMD_ENDFLAG | ICM45686_I2CM_CMD_RW_WRITE | num_bytes);
	// Trigger transaction
	err |= icm45_bank_write_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_CONTROL,
								 ICM45686_I2CM_CONTROL_RESTART_EN | ICM45686_I2CM_CONTROL_GO);

	// Wait for I2C transaction (Fast mode 400kHz: ~25us per byte + overhead)
	k_busy_wait(30 * (num_bytes + 2));

	status = 0;
	timeout = 1000;
	do {
		err |= icm45_bank_read_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_STATUS, &status);
		if (--timeout <= 0)
		{
			LOG_ERR("I2CM write timeout");
			return -1;
		}
	} while (status & ICM45686_BIT_I2CM_STATUS_BUSY);

	if (status & (ICM45686_BIT_I2CM_STATUS_SDA_ERR | ICM45686_BIT_I2CM_STATUS_SCL_ERR))
	{
		if (retries-- > 0)
		{
			LOG_WRN("I2CM bus error on write 0x%02X: 0x%02x, retrying", addr, status);
			k_msleep(5);
			goto retry_write;
		}
		LOG_ERR("I2CM bus error: 0x%02x", status);
		return -1;
	}

	if (!(status & ICM45686_BIT_I2CM_STATUS_DONE))
	{
		LOG_ERR("I2CM write failed: status 0x%02x", status);
		return -1;
	}

	err |= icm45_bank_read_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_EXT_DEV_STATUS, &dev_status);
	if (dev_status & 0x01)
	{
		LOG_DBG("I2CM NACK on write to addr 0x%02x", addr);
		return -1;
	}

	// Restore continuous I2CM read: re-setup DEV_PROFILE + COMMAND for the
	// previously cached read and trigger it. This way the next ext_write_read
	// finds fresh data ready without the full oneshot cycle.
	if (was_continuous && !err)
	{
		uint8_t rd_profile[2] = {saved_sub, saved_addr};
		icm45_bank_write(ICM45686_IPREG_TOP1, ICM45686_DEV_PROFILE_0, rd_profile, 2);
		icm45_bank_write_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_COMMAND_0,
							 ICM45686_I2CM_CMD_ENDFLAG | ICM45686_I2CM_CMD_RW_READ_REG | saved_len);
		icm45_bank_write_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_CONTROL,
							 ICM45686_I2CM_CONTROL_RESTART_EN | ICM45686_I2CM_CONTROL_GO);
		ext_continuous_active = true;
		ext_cont_addr = saved_addr;
		ext_cont_sub = saved_sub;
		ext_cont_len = saved_len;
	}

	return err;
}

int icm45_ext_write_read(const uint8_t addr, const void *write_buf, size_t num_write, void *read_buf, size_t num_read)
{
	if (num_write != 1 || num_read < 1 || num_read > 15)
	{
		LOG_ERR("Unsupported write_read: write=%d read=%d", num_write, num_read);
		return -1;
	}

	uint8_t sub_addr = ((const uint8_t *)write_buf)[0];

	// Fast path: if a pre-triggered I2CM read matches, just read cached RD_DATA.
	// The I2C transaction was triggered at the end of the previous call and has
	// completed in the background during FIFO processing (~6ms >> ~300us I2C).
	if (ext_continuous_active && addr == ext_cont_addr &&
	    sub_addr == ext_cont_sub && num_read == ext_cont_len)
	{
		// Wait only if I2CM is still busy (rare - only if called very quickly)
		uint8_t status = 0;
		icm45_bank_read_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_STATUS, &status);
		if (status & ICM45686_BIT_I2CM_STATUS_BUSY)
		{
			// Transaction still in progress - wait briefly
			int timeout = 200;
			do {
				icm45_bank_read_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_STATUS, &status);
				if (--timeout <= 0)
				{
					LOG_WRN("I2CM continuous read timeout, falling back");
					ext_continuous_active = false;
					goto oneshot_read;
				}
			} while (status & ICM45686_BIT_I2CM_STATUS_BUSY);
		}

		if (!(status & ICM45686_BIT_I2CM_STATUS_DONE) ||
		    (status & (ICM45686_BIT_I2CM_STATUS_SDA_ERR | ICM45686_BIT_I2CM_STATUS_SCL_ERR)))
		{
			LOG_WRN("I2CM continuous read error: 0x%02x, falling back", status);
			ext_continuous_active = false;
			goto oneshot_read;
		}

		// Read cached data from previous I2CM transaction
		int err = icm45_bank_read(ICM45686_IPREG_TOP1, ICM45686_I2CM_RD_DATA_0, read_buf, num_read);

		// Pre-trigger next I2CM read (DEV_PROFILE + COMMAND persist in IPREG_TOP1)
		icm45_bank_write_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_CONTROL,
							 ICM45686_I2CM_CONTROL_RESTART_EN | ICM45686_I2CM_CONTROL_GO);
		return err;
	}

	// If continuous was active with different params, stop it
	if (ext_continuous_active)
		icm45_ext_stop_continuous();

oneshot_read:;
	// Full one-shot I2CM read with retry for transient bus errors
	int retries = 2;
	int err;
	uint8_t dev_profile[2];
	uint8_t status;
	uint8_t dev_status;
	int timeout;

retry_read:
	err = 0;
	// DEV_PROFILE_0 = register address, DEV_PROFILE_1 = slave address
	dev_profile[0] = sub_addr;
	dev_profile[1] = addr;
	err |= icm45_bank_write(ICM45686_IPREG_TOP1, ICM45686_DEV_PROFILE_0, dev_profile, 2);
	// Command: last transaction, read-with-register mode, num_read bytes
	err |= icm45_bank_write_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_COMMAND_0,
								 ICM45686_I2CM_CMD_ENDFLAG | ICM45686_I2CM_CMD_RW_READ_REG | num_read);
	// Trigger transaction
	err |= icm45_bank_write_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_CONTROL,
								 ICM45686_I2CM_CONTROL_RESTART_EN | ICM45686_I2CM_CONTROL_GO);

	// Wait for I2C transaction (Fast mode 400kHz: ~25us per byte + overhead)
	k_busy_wait(25 * num_read + 80);

	status = 0;
	timeout = 1000;
	do {
		err |= icm45_bank_read_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_STATUS, &status);
		if (--timeout <= 0)
		{
			LOG_ERR("I2CM read timeout");
			return -1;
		}
	} while (status & ICM45686_BIT_I2CM_STATUS_BUSY);

	if (status & (ICM45686_BIT_I2CM_STATUS_SDA_ERR | ICM45686_BIT_I2CM_STATUS_SCL_ERR))
	{
		if (retries-- > 0)
		{
			LOG_WRN("I2CM bus error on read 0x%02X: 0x%02x, retrying", addr, status);
			k_msleep(5);
			goto retry_read;
		}
		LOG_ERR("I2CM bus error: 0x%02x", status);
		return -1;
	}

	if (!(status & ICM45686_BIT_I2CM_STATUS_DONE))
	{
		LOG_ERR("I2CM read failed: status 0x%02x", status);
		return -1;
	}

	err |= icm45_bank_read_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_EXT_DEV_STATUS, &dev_status);
	if (dev_status & 0x01)
	{
		LOG_DBG("I2CM NACK from addr 0x%02x reg 0x%02x", addr, sub_addr);
		return -1;
	}

	err |= icm45_bank_read(ICM45686_IPREG_TOP1, ICM45686_I2CM_RD_DATA_0, read_buf, num_read);

	// In operational mode, pre-trigger next read for fast subsequent reads.
	// DEV_PROFILE + COMMAND persist in IPREG_TOP1, so just writing GO is enough.
	// The I2CM transaction (~300us) completes in the background during FIFO
	// processing, so next read finds data ready without any wait.
	if (!err && !ext_scanning_mode)
	{
		icm45_bank_write_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_CONTROL,
							 ICM45686_I2CM_CONTROL_RESTART_EN | ICM45686_I2CM_CONTROL_GO);
		ext_continuous_active = true;
		ext_cont_addr = addr;
		ext_cont_sub = sub_addr;
		ext_cont_len = num_read;
	}

	return err;
}

const sensor_ext_ssi_t sensor_ext_icm45686 = {
	icm45_ext_write,
	icm45_ext_write_read,
	15
};

int icm45_ext_setup(void)
{
	int err = 0;

	// Stop continuous mode and enter scanning mode for device discovery
	icm45_ext_stop_continuous();
	ext_scanning_mode = true;

	// --- Ensure gyroscope oscillator is available for I2CM ---
	// I2CM with OSC_ID_OVRD=2 requires gyro oscillator. After WOM wake or
	// shutdown, gyro may be off. Power on gyro standby to provide I2CM clock.
	uint8_t pwr = 0;
	ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_PWR_MGMT0, &pwr);
	uint8_t gyro_mode = (pwr >> 2) & 0x03;
	if (gyro_mode == GYRO_MODE_OFF) {
		pwr = (pwr & ~(0x03 << 2)) | (GYRO_MODE_STANDBY << 2);
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_PWR_MGMT0, pwr);
		k_msleep(1); // Wait for gyro oscillator to stabilize
	}

	uint8_t ireg_buf[3];
	ireg_buf[0] = ICM45686_IPREG_BAR;

	// Configure I2CM mode (REG_60)
	ireg_buf[1] = ICM45686_IPREG_BAR_REG_60;
	ireg_buf[2] = ICM45686_BIT_AUX1_I2CM_MODE;
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, ireg_buf, 3);

	// Enable AUX1 in I2CM Master mode
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_IOC_PAD_SCENARIO_AUX_OVRD, 0x17);

	// OSC_ID_OVRD must be 1 or 2 (if gyro is enabled) for I2CM operation (AN-000478)
	err |= ssi_reg_update_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_REG_MISC1, 0x0F, 0x02);

	if (err)
		LOG_ERR("Communication error");

	sensor_interface_ext_configure(&sensor_ext_icm45686);
	return err;
}

int icm45_ext_passthrough(bool passthrough)
{
	icm45_ext_stop_continuous();
	int err = 0;
	if (passthrough)
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_IOC_PAD_SCENARIO_AUX_OVRD, 0x18); // AUX1 in I2CM Bypass
	else
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_IOC_PAD_SCENARIO_AUX_OVRD, 0x00); // disable overrides
	if (err)
		LOG_ERR("Communication error");
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
	icm45_ext_passthrough
};
