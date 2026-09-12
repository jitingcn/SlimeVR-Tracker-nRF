#include <math.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include "QMC6309.h"
#include "sensor/sensor_none.h"

// QMC6309 Rev C: https://www.qstcorp.com/upload/pdf/202512/587AA782C1A94BF3B11CC72DB8A11E67.pdf
// QMC6309H Rev E: https://www.qstcorp.com/upload/pdf/202601/CF382A94E1424763B3DE87DC967757FC.pdf

#define QMC6309_OUTX_L_REG 0x01

// Status register {DRDY:1;OVFL:1;...}
#define QMC6309_STAT_REG 0x09

#define STAT_DATA_RDY_MASK 0b01
#define STAT_OVERFLOW_MASK 0b10

// Control register 1 {MD:2;:1;OSR:2;LPF:3;}
#define QMC6309_CTRL_REG_1 0x0A

#define MD_SUSPEND 0b00
#define MD_NORMAL  0b01
#define MD_SINGLE  0b10
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
#define ODR_1Hz  0b000 // QMC6309 only; unsupported on QMC6309H
#define ODR_10Hz  0b001 // QMC6309 only; unsupported on QMC6309H
#define ODR_50Hz  0b010
#define ODR_100Hz 0b011
#define ODR_200Hz 0b100
#define ODR_MASK(odr) ((odr) << 4)

#define SOFT_RESET_MASK 0x80
#define SOFT_RESET_CLEAR 0x00

static const float sensitivity = 1 / 4000.0f; // ~0.25 mgauss/LSB @ 8G range -> ~0.00025 G/LSB

static uint8_t last_state = 0xff;
static bool last_overflow = false;
static int64_t oneshot_trigger_ms = 0;
static bool oneshot_pending;
static bool oneshot_failed;
static bool qmc6309h;
// Normal-mode data registers retain the last sample (Rev. E, section 9.2.1).
// Byte equality is a duplicate heuristic, not proof of sample identity.
// The external-interface path omits the separate status read.
static uint8_t last_raw_sample[6];
static bool last_raw_sample_valid = false;
static int64_t last_mag_time_ms;
static int32_t mag_period_ms = 20; // default 50Hz

LOG_MODULE_REGISTER(QMC6309, LOG_LEVEL_INF);

void qmc_set_variant(bool is_qmc6309h)
{
	qmc6309h = is_qmc6309h;
	last_state = 0xff;
}

int qmc_init(float period_s, float *actual_period_s)
{
	last_state = 0xff; // init state
	last_overflow = false;
	oneshot_trigger_ms = 0;
	oneshot_pending = false;
	oneshot_failed = false;
	last_raw_sample_valid = false;
	last_mag_time_ms = 0;
	int err = qmc_update_odr(period_s, actual_period_s);
	return (err < 0 ? err : 0);
}

void qmc_shutdown(void)
{
	last_state = 0xff;
	oneshot_trigger_ms = 0;
	oneshot_pending = false;
	oneshot_failed = false;
	last_raw_sample_valid = false;
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC6309_CTRL_REG_2, SOFT_RESET_MASK);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC6309_CTRL_REG_2, SOFT_RESET_CLEAR);
	if (err)
		LOG_ERR("Communication error");
}

int qmc_update_odr(float period_s, float *actual_period_s)
{
	int requested_odr_hz; // Truncate fractional Hz before selecting a supported rate.
	uint8_t odr_code;
	uint8_t mode_code;

	if (period_s <= 0 || period_s == INFINITY) // power down mode or single measurement mode
	{
		mode_code = MD_SUSPEND; // oneshot will set SINGLE after
		requested_odr_hz = 0;
	} else {
		mode_code = MD_NORMAL;
		requested_odr_hz = 1 / period_s;
	}

	if (mode_code == MD_SUSPEND) {
		odr_code = ODR_200Hz; // for oneshot
		period_s = INFINITY;  // signal oneshot mode available
	} else if (requested_odr_hz > 100) {
		odr_code = ODR_200Hz;
		period_s = 1.f / 200;
	} else if (requested_odr_hz > 50) {
		odr_code = ODR_100Hz;
		period_s = 1.f / 100;
	} else if (qmc6309h || requested_odr_hz > 25) {
		/* QMC6309H codes 000/001 are unsupported; QMC6309 uses them for 1/10 Hz. */
		odr_code = ODR_50Hz;
		period_s = 1.f / 50;
	} else if (requested_odr_hz > 1) {
		odr_code = ODR_10Hz;
		period_s = 1.f / 10;
	} else {
		odr_code = ODR_1Hz;
		period_s = 1.f;
	}

	uint8_t config_code = ODR_MASK(odr_code) | mode_code;
	if (last_state == config_code) {
		*actual_period_s = period_s;
		return 0; /* already configured — success for err|= callers */
	}

	int err = ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_MAG,
		QMC6309_CTRL_REG_2,
		ODR_MASK(odr_code) | RNG_MASK(RNG_8G) | SET_RESET_ON
	);
	if (!err)
		err = ssi_reg_write_byte(
			SENSOR_INTERFACE_DEV_MAG,
			QMC6309_CTRL_REG_1,
			LPF_MASK(LPF_4) | OSR_MASK(OSR_8) | mode_code
		);
	if (err) {
		LOG_ERR("Communication error");
		last_state = 0xff;
		return err;
	}

	last_state = config_code;
	oneshot_trigger_ms = 0;
	oneshot_pending = false;
	oneshot_failed = false;
	last_raw_sample_valid = false;

	if (mode_code != MD_SUSPEND) {
		mag_period_ms = (int32_t)(period_s * 1000);
	}

	*actual_period_s = period_s;
	return 0;
}

void qmc_mag_oneshot(void)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC6309_CTRL_REG_1, LPF_MASK(LPF_4) | OSR_MASK(OSR_8) | MD_SINGLE);
	last_state = 0xff;
	oneshot_failed = err != 0;
	oneshot_pending = true;
	oneshot_trigger_ms = k_uptime_get();
	if (err)
		LOG_ERR("Communication error");
}

bool qmc_mag_read(float m[3])
{
	bool was_oneshot = oneshot_pending;
	if (oneshot_pending)
	{
		if (oneshot_failed) {
			oneshot_pending = false;
			oneshot_failed = false;
			return false;
		}

		// Oneshot mode: wait for DRDY with timeout
		uint8_t status = 0;
		int64_t deadline_ms = oneshot_trigger_ms + 10; // 10ms timeout
		while ((status & STAT_DATA_RDY_MASK) == 0)
		{
			int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, QMC6309_STAT_REG, &status);
			if (err) {
				LOG_ERR("Communication error");
				oneshot_pending = false;
				return false;
			}
			if (k_uptime_get() >= deadline_ms) {
				LOG_WRN("Data ready status timeout!");
				oneshot_pending = false;
				return false;
			}
		}
		oneshot_trigger_ms = 0;
		oneshot_pending = false;
		if (status & STAT_OVERFLOW_MASK) {
			if (!last_overflow) {
				LOG_INF("Magnetometer overflow");
			}
			last_overflow = true;
			return false;
		}
		last_overflow = false;
	}
	if (!was_oneshot && sensor_interface_get_spec(SENSOR_INTERFACE_DEV_MAG) != SENSOR_INTERFACE_SPEC_EXT) {
		uint8_t status = 0;
		int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, QMC6309_STAT_REG, &status);
		if (err) {
			LOG_ERR("Communication error");
			return false;
		}
		if (!(status & STAT_DATA_RDY_MASK) || (status & STAT_OVERFLOW_MASK))
			return false;
	}
	uint8_t rawData[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, QMC6309_OUTX_L_REG, rawData, 6);
	if (err)
	{
		LOG_ERR("Communication error");
		return false;
	}
	// Reject byte-identical readings until the cached period plus its integer
	// 10% margin expires. A new conversion can have identical values, so this
	// heuristic trades exact freshness detection for bounded duplicate suppression.
	int64_t now = k_uptime_get();
	if (last_raw_sample_valid && memcmp(rawData, last_raw_sample, 6) == 0) {
		if ((now - last_mag_time_ms) < (mag_period_ms + mag_period_ms / 10))
			return false;
		// Advance one cached period; resynchronize to now only when far behind.
		last_mag_time_ms += mag_period_ms;
		if (now - last_mag_time_ms > mag_period_ms * 2)
			last_mag_time_ms = now;
	} else {
		last_mag_time_ms = now;
	}
	memcpy(last_raw_sample, rawData, 6);
	last_raw_sample_valid = true;
	qmc_mag_process(rawData, m);
	return true;
}

void qmc_mag_process(uint8_t *raw_m, float m[3])
{
	for (int i = 0; i < 3; i++) // x, y, z
	{
		uint16_t raw = ((uint16_t)raw_m[i * 2 + 1] << 8) | raw_m[i * 2];
		m[i] = (int16_t)raw;
		m[i] *= sensitivity; // Gauss
	}
}

const sensor_mag_t sensor_mag_qmc6309 = {
	qmc_init,
	qmc_shutdown,

	qmc_update_odr,

	qmc_mag_oneshot,
	qmc_mag_read,
	mag_none_temp_read,

	qmc_mag_process,
	6, 6
};
