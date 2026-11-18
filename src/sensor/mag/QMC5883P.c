#include <math.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include "QMC5883P.h"
#include "sensor/sensor_none.h"

// QMC5883P Rev E (QST-PD-B002-22). Register plumbing follows QMC6309.c; the
// datasheet differences that drive this file are listed in QMC5883P.h.

#define QMC5883P_CHIPID_REG 0x00 // reads 0x80

#define QMC5883P_OUTX_L_REG 0x01

// Status register {DRDY:1;OVFL:1;...}
#define QMC5883P_STAT_REG 0x09

#define STAT_DATA_RDY_MASK 0b01
#define STAT_OVERFLOW_MASK 0b10

// Control register 1 {OSR2:2;OSR1:2;ODR:2;MODE:2}: the P keeps ODR here.
#define QMC5883P_CTRL_REG_1 0x0A

#define MODE_SUSPEND 0b00
#define MODE_NORMAL 0b01
#define MODE_SINGLE 0b10
#define MODE_CONTINUOUS 0b11
#define MODE_MASK 0b11

// OSR1 (bits [5:4]) oversampling: 8x = 0b00. Magnetic fusion runs on a
// seconds-scale time constant, so noise matters more than bandwidth.
#define OSR1_8X 0b00
// OSR2 (bits [7:6]) decimation: /8 = 0b11, the setting behind the datasheet's
// 0xCD example.
#define OSR2_DIV8 0b11
#define OSR_MASK (OSR2_DIV8 << 6 | OSR1_8X << 4)

// ODR (bits [3:2]), two bits: 10/50/100/200 Hz. The 100 Hz working point
// matches CONFIG_SENSOR_MAG_ODR=100; 200 Hz would only raise normal-mode
// current because the fusion filter cannot use the extra rate.
#define ODR_10Hz 0b00
#define ODR_50Hz 0b01
#define ODR_100Hz 0b10
#define ODR_200Hz 0b11
#define ODR_MASK(odr) ((odr) << 2)

#define CONFIG_UNKNOWN 0xff

// Control register 2 {SOFT_RST:1;SELF_TEST:1;-:1;-:1;INT_ENB:1;RNG:2;SET_RST:2}
#define QMC5883P_CTRL_REG_2 0x0B

// Periodic set/reset suppresses bridge offset drift; never turn it off to save
// power. 8 G keeps headroom for Earth's field plus worn hard-iron bias.
#define SET_RESET_ON 0b00
#define RNG_8G 0b10
#define RNG_MASK(rng) ((rng) << 2)
// Normal CTRL2 value = RNG_8G<<2 | SET_RESET_ON = 0x08. Never 0x41 or 0x01:
// those are QMC5883L values (self-test bit and 30 G range); see the header.
#define CTRL2_VALUE (RNG_MASK(RNG_8G) | SET_RESET_ON)

#define SOFT_RESET_MASK 0x80
#define SOFT_RESET_CLEAR 0x00
#define SELF_TEST_MASK 0x40 // factory self-test; never set during init

// The datasheet power-on sequence writes 0x29 = 0x06. Rewrite it during
// configuration to avoid a polarity flip after a mode change.
#define QMC5883P_SIGN_REG 0x29
#define SIGN_VALUE 0x06

// 8 G range: 3750 LSB/G (QMC6309 is 4000).
static const float sensitivity = 1 / 3750.0f;

// Overflow guard: |raw| >= 30000 counts as saturated, in addition to the
// STATUS OVFL bit. A saturated sample is dropped, never rescaled: a mid-run
// range change would invalidate the hard/soft-iron LS fit and the VQF field
// reference.
#define MAG_SATURATION_LSB 30000

// A confirmed SINGLE owns one synchronous read; UNKNOWN never permits reads.
static uint8_t last_state = CONFIG_UNKNOWN;
static bool last_overflow = false;
static int64_t oneshot_trigger_ms = 0;
// In normal mode the data registers retain the previous sample; byte equality
// is a duplicate heuristic, not proof of sample identity. Only the external
// interface path needs it because it has no status read (same as QMC6309).
static uint8_t last_raw_sample[6];
static bool last_raw_sample_valid = false;
static int64_t last_mag_time_ms;
static int32_t mag_period_ms = 10; // default 100Hz

LOG_MODULE_REGISTER(QMC5883P, LOG_LEVEL_INF);

int qmc5883p_init(float period_s, float *actual_period_s)
{
	last_state = CONFIG_UNKNOWN;
	last_overflow = false;
	last_raw_sample_valid = false;
	last_mag_time_ms = 0;
	// update_odr writes 0x29 = 0x06 -> 0x0B = 0x08 -> 0x0A config.
	int err = qmc5883p_update_odr(period_s, actual_period_s);
	return (err < 0 ? err : 0);
}

void qmc5883p_shutdown(void)
{
	last_state = CONFIG_UNKNOWN;
	last_raw_sample_valid = false;
	// Issue a soft reset, then explicitly write the cleared CTRL2 value.
	ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_CTRL_REG_2, SOFT_RESET_MASK);
	ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_CTRL_REG_2, SOFT_RESET_CLEAR);
}

int qmc5883p_update_odr(float period_s, float *actual_period_s)
{
	float requested_odr_hz;
	uint8_t odr_code;
	uint8_t mode_code;

	if (period_s <= 0 || period_s == INFINITY) // power down or single measurement mode
	{
		mode_code = MODE_SUSPEND; // oneshot will set SINGLE after
		requested_odr_hz = 0;
	} else {
		mode_code = MODE_NORMAL;
		requested_odr_hz = 1 / period_s;
	}

	if (mode_code == MODE_SUSPEND) {
		odr_code = ODR_200Hz; // for oneshot
		period_s = INFINITY;  // signal oneshot mode available
	} else if (requested_odr_hz > 100) {
		odr_code = ODR_200Hz;
		period_s = 1.f / 200;
	} else if (requested_odr_hz > 50) {
		odr_code = ODR_100Hz;
		period_s = 1.f / 100;
	} else if (requested_odr_hz > 10) {
		odr_code = ODR_50Hz;
		period_s = 1.f / 50;
	} else {
		odr_code = ODR_10Hz;
		period_s = 1.f / 10;
	}

	uint8_t config_code = ODR_MASK(odr_code) | mode_code;
	if (last_state == config_code) {
		*actual_period_s = period_s;
		return 0; /* already configured - success for err|= callers */
	}

	uint8_t previous_state = last_state;
	last_state = CONFIG_UNKNOWN;
	last_raw_sample_valid = false;

	// Enter a running mode through suspend, including when a previous
	// operation may have changed the mode despite returning an error.
	int err = 0;
	if (mode_code != MODE_SUSPEND && (previous_state & MODE_MASK) != MODE_SUSPEND) {
		err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_CTRL_REG_1,
					 OSR_MASK | ODR_MASK(odr_code) | MODE_SUSPEND);
	}

	if (!err) {
		// Rewrite the sign register before every (re)entry into normal mode.
		err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_SIGN_REG, SIGN_VALUE);
	}
	if (!err) {
		err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_CTRL_REG_2, CTRL2_VALUE);
	}
	if (!err) {
		// 100 Hz working point: OSR_MASK | ODR_100Hz<<2 | MODE_NORMAL =
		// 1100 1001b = 0xC9 (the datasheet's 0xCD example is the 200 Hz
		// code; tests assert both).
		err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_CTRL_REG_1, OSR_MASK | config_code);
	}
	if (err)
		return err;

	last_state = config_code;

	if (mode_code != MODE_SUSPEND) {
		mag_period_ms = (int32_t)(period_s * 1000);
	}

	*actual_period_s = period_s;
	return 0;
}

void qmc5883p_mag_oneshot(void)
{
	// An unconfigured or uncertain device requires explicit configuration.
	if (last_state == CONFIG_UNKNOWN)
		return;

	float actual_period_s;
	if (qmc5883p_update_odr(INFINITY, &actual_period_s))
		return;

	// STATUS acknowledges both flags (Rev E §9.2.2). Drain any previous
	// conversion in suspend before starting the next one, even on EXT.
	uint8_t stale_status;
	if (ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_STAT_REG, &stale_status))
		return;

	uint8_t config = ODR_MASK(ODR_200Hz) | MODE_SINGLE;
	last_state = CONFIG_UNKNOWN;
	if (ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_CTRL_REG_1, OSR_MASK | config))
		return;

	oneshot_trigger_ms = k_uptime_get();
	last_state = config;
}

bool qmc5883p_mag_read(float m[3])
{
	uint8_t mode = last_state & MODE_MASK;
	bool was_oneshot = mode == MODE_SINGLE;
	if (!was_oneshot && mode != MODE_NORMAL)
		return false;

	bool ext = !was_oneshot && sensor_interface_get_spec(SENSOR_INTERFACE_DEV_MAG) == SENSOR_INTERFACE_SPEC_EXT;
	if (was_oneshot) {
		// Revoke the shot before fallible I/O. Only a fresh DRDY proves that
		// the chip has completed the conversion and returned to suspend.
		uint8_t suspend_config = last_state & ~MODE_MASK;
		last_state = CONFIG_UNKNOWN;
		uint8_t status;
		int64_t deadline_ms = oneshot_trigger_ms + 10; // 10ms timeout
		for (;;) {
			if (ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_STAT_REG, &status))
				return false;
			if (status & STAT_DATA_RDY_MASK)
				break;
			if (k_uptime_get() >= deadline_ms) {
				LOG_WRN("Data ready status timeout!");
				return false;
			}
		}
		last_state = suspend_config;
		if (status & STAT_OVERFLOW_MASK) {
			if (!last_overflow) {
				LOG_INF("Magnetometer overflow");
			}
			last_overflow = true;
			return false;
		}
		last_overflow = false;
	}
	if (!was_oneshot && !ext) {
		uint8_t status = 0;
		int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_STAT_REG, &status);
		if (err) {
			return false;
		}
		if (!(status & STAT_DATA_RDY_MASK) || (status & STAT_OVERFLOW_MASK)) {
			return false;
		}
	}
	uint8_t rawData[6];
	int err = was_oneshot ? ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, QMC5883P_OUTX_L_REG, rawData, 6)
			      : ssi_burst_read_frame(SENSOR_INTERFACE_DEV_MAG, QMC5883P_OUTX_L_REG, 0, 2, rawData, 6);
	if (err) {
		return false;
	}
	// Software saturation guard, in addition to the STATUS OVFL bit: reject a
	// sample when any axis reaches |raw| >= 30000. Dropping it keeps the
	// completed hard/soft-iron calibration and the VQF field reference valid.
	for (int i = 0; i < 3; i++) {
		uint16_t raw = ((uint16_t)rawData[i * 2 + 1] << 8) | rawData[i * 2];
		if ((int16_t)raw >= MAG_SATURATION_LSB || (int16_t)raw <= -MAG_SATURATION_LSB) {
			if (!last_overflow) {
				LOG_INF("Magnetometer saturation (|raw| >= %d)", MAG_SATURATION_LSB);
			}
			last_overflow = true;
			return false;
		}
	}
	last_overflow = false;
	if (ext) {
		// External periodic reads lack status. Retain the duplicate heuristic:
		// reject byte-identical readings until the period plus 10% expires.
		int64_t now = k_uptime_get();
		if (last_raw_sample_valid && memcmp(rawData, last_raw_sample, 6) == 0) {
			// Round up: a 5 ms period still needs a nonzero margin.
			if (now - last_mag_time_ms < mag_period_ms + (mag_period_ms + 9) / 10)
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
	}
	qmc5883p_mag_process(rawData, m);
	return true;
}

void qmc5883p_mag_process(uint8_t *raw_m, float m[3])
{
	// Data 0x01-0x06 is little-endian XYZ (assembled explicitly, so endianness
	// of the host cannot leak in).
	for (int i = 0; i < 3; i++) // x, y, z
	{
		uint16_t raw = ((uint16_t)raw_m[i * 2 + 1] << 8) | raw_m[i * 2];
		m[i] = (int16_t)raw;
		m[i] *= sensitivity; // Gauss (8G range, 3750 LSB/G)
	}
}

const sensor_mag_t sensor_mag_qmc5883p
	= {qmc5883p_init,
	   qmc5883p_shutdown,

	   qmc5883p_update_odr,

	   qmc5883p_mag_oneshot,
	   qmc5883p_mag_read,
	   mag_none_temp_read, // no temperature register (the chip compensates internally)

	   qmc5883p_mag_process,
	   6,
	   6,
	   .cached_poll_safe = false};
