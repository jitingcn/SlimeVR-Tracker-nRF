#include <math.h>

#include <zephyr/logging/log.h>

#include "QMC6309.h"
#include "sensor/sensor_none.h"

// https://www.lcsc.com/datasheet/lcsc_datasheet_2410121623_QST-QMC6309_C5439871.pdf

#define QMC6309_OUTX_L_REG 0x01

// Status register {DRDY:1;OVFL:1;...}
#define QMC6309_STAT_REG 0x09

#define STAT_DATA_RDY_MASK 0b01
#define STAT_OVERFLOW_MASK 0b10

// Control register 1 {MD:2;:1;OSR:2;LPF:3;}
#define QMC6309_CTRL_REG_1 0x0A

#define MD_SUSPEND 0b00
#define MD_NORMAL 0b01
#define MD_SINGLE 0b10
#define MD_CONTINUOUS 0b11
#define MD_MASK 0b11

#define OSR_OFF 0b11
#define OSR_2 0b10
#define OSR_4 0b01
#define OSR_8 0b00
#define OSR_MASK(osr) ((osr) << 3)

#define LPF_OFF 0b000
#define LPF_2 0b001
#define LPF_4 0b010
#define LPF_8 0b011
#define LPF_16 0b100
#define LPF_MASK(lpf) ((lpf) << 5)

// Control register 2 {SET:2;RGN:2;ODR:3;SRT:1;}
#define QMC6309_CTRL_REG_2 0x0B

#define SET_RESET_ON 0b00
#define SET_ONLY 0b01
#define SET_RESET_OFF 0b11

#define RNG_32G 0b00
#define RNG_16G 0b01
#define RNG_8G 0b10
#define RNG_MASK(rng) ((rng) << 2)

#define CURRENT_RNG RNG_8G

#define ODR_1Hz 0b000
#define ODR_10Hz 0b001
#define ODR_50Hz 0b010
#define ODR_100Hz 0b011
#define ODR_200Hz 0b100
#define ODR_MASK(odr) ((odr) << 4)

#define SOFT_RESET_MASK 0x80
#define SOFT_RESET_CLEAR 0x00

// QMC6309 Sensitivity (LSB/Gauss)
// @ 8G range, sensitivity is typically 3000-4000 LSB/Gauss depending on specific QST chip version.
static const float sensitivity = 1.0f / 4000.0f;

static uint8_t last_state = 0xff;
static bool lastOvfl = false;
static int64_t oneshot_trigger_time = 0;

LOG_MODULE_REGISTER(QMC6309, LOG_LEVEL_INF);

int qmc_init(float time, float *actual_time)
{
	last_state = 0xff; // init state
	lastOvfl = false;
	oneshot_trigger_time = 0;
	int err = qmc_update_odr(time, actual_time);
	LOG_INF("QMC6309 init: err=%d, time=%.4f", err, (double)*actual_time);
	return (err < 0 ? err : 0);
}

void qmc_shutdown(void)
{
	// Soft Reset
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC6309_CTRL_REG_2, SOFT_RESET_MASK);
	k_msleep(1); // Give it a moment to reset
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC6309_CTRL_REG_2, SOFT_RESET_CLEAR);
	if (err) {
		LOG_ERR("Communication error");
	}
}

int qmc_update_odr(float time, float *actual_time)
{
	int ODR;
	uint8_t MODR;
	uint8_t MD;

	if (time <= 0 || time == INFINITY) // power down mode or single measurement mode
	{
		MD = MD_SUSPEND;
		ODR = 0;
	} else {
		MD = MD_CONTINUOUS;
		ODR = (int)(1.0f / time);
	}

	if (MD == MD_SUSPEND) {
		MODR = ODR_200Hz; // for oneshot
		time = 0;         // off
	} else if (ODR > 100) {
		MODR = ODR_200Hz;
		time = 1.f / 200;
	} else if (ODR > 50) {
		MODR = ODR_100Hz;
		time = 1.f / 100;
	} else if (ODR > 25) {
		MODR = ODR_50Hz;
		time = 1.f / 50;
	} else if (ODR > 5) {
		MODR = ODR_10Hz;
		time = 1.f / 10;
	} else {
		MODR = ODR_1Hz;
		time = 1.f;
	}

	uint8_t STAT = ODR_MASK(MODR) | MD;

	if (last_state == STAT) {
		*actual_time = time;
		return 0;
	}
	last_state = STAT;

	// Write Config
	// Use CURRENT_RNG macro ensuring consistency
	int err = ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_MAG,
		QMC6309_CTRL_REG_2,
		ODR_MASK(MODR) | RNG_MASK(CURRENT_RNG) | SET_RESET_ON
	);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC6309_CTRL_REG_1, LPF_MASK(LPF_2) | OSR_MASK(OSR_8) | MD);

	if (err) {
		LOG_ERR("Communication error");
	}

	oneshot_trigger_time = 0;
	*actual_time = time;
	return err;
}

void qmc_mag_oneshot(void)
{
	// Trigger a single measurement
	int err = ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_MAG,
		QMC6309_CTRL_REG_1,
		LPF_MASK(LPF_2) | OSR_MASK(OSR_8) | MD_SINGLE
	);
	oneshot_trigger_time = k_uptime_get();
	if (err) {
		LOG_ERR("Communication error");
	}
}

void qmc_mag_read(float m[3])
{
	int err = 0;
	uint8_t status = 0;

	// 1. Read Status
	err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, QMC6309_STAT_REG, &status);
	if (err) {
		LOG_ERR("Status read error: %d", err);
		return;
	}
	// LOG_INF("QMC Status: 0x%02X", status); // Uncomment if needed, spammy

	// 2. Check Data Ready
	if ((status & STAT_DATA_RDY_MASK) == 0) {
		if (oneshot_trigger_time > 0 && k_uptime_get() > oneshot_trigger_time + 50) {
			LOG_WRN("Magnetometer oneshot timeout");
			oneshot_trigger_time = 0;
		}
		return; // Data not ready
	}

	oneshot_trigger_time = 0; // Clear timeout check

	// 3. Check Overflow
	if (status & STAT_OVERFLOW_MASK) {
		if (!lastOvfl) {
			LOG_WRN("Magnetometer magnetic sensor overflow");
			lastOvfl = true;
		}
		// QST recommendation: Usually reading the data clears the logic even if overflowed
		// But the data might be clipped. Let's proceed to read to clear registers.
	} else {
		lastOvfl = false;
	}

	// 4. Read Data (6 bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H)
	uint8_t rawData[6];
	err = ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, QMC6309_OUTX_L_REG, rawData, 6);
	if (err) {
		return;
	}

	// Process data (convert to float)
	qmc_mag_process(rawData, m);
}

void qmc_mag_process(uint8_t *raw_m, float m[3])
{
	int16_t x = (int16_t)((raw_m[1] << 8) | raw_m[0]);
	int16_t y = (int16_t)((raw_m[3] << 8) | raw_m[2]);
	int16_t z = (int16_t)((raw_m[5] << 8) | raw_m[4]);

	// LOG_DBG("Mag raw: %d %d %d", x, y, z);

	// Apply sensitivity
	m[0] = x * sensitivity;
	m[1] = y * sensitivity;
	m[2] = z * sensitivity;
}

const sensor_mag_t sensor_mag_qmc6309
	= {.init = qmc_init,
	   .shutdown = qmc_shutdown,
	   .update_odr = qmc_update_odr,
	   .mag_oneshot = qmc_mag_oneshot,
	   .mag_read = qmc_mag_read,
	   .temp_read = mag_none_temp_read,
	   .mag_process = qmc_mag_process,
	   .ext_min_burst = 6,
	   .ext_burst = 6};
