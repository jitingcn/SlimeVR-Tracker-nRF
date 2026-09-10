#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include "ICT153xx.h"

/* TDK DS-000548 rev. 1.0, sections 5, 7 and 10:
 * https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/9004/ICT-153xx.pdf
 * Both parts require 1.62--1.98 V VDD. Bus levels differ: ICT-15312 uses
 * 1.2 V, ICT-15318 uses 1.8 V. Never write the factory VDDIO setting at 0x21.
 * I2C (direct or IMU proxy) only; no native SPI, I3C or FIFO support.
 */
#define ICT153XX_SLEEP 0x00
#define ICT153XX_PULSED 0x01
#define ICT153XX_SINGLE 0x02
#define ICT153XX_MRM 0x03
#define ICT153XX_MODE_MASK 0x03
#define ICT153XX_ODR_MASK 0x70
#define ICT153XX_CONFIG_INVALID 0xFF

/* Single-shot maximum is 3.355 ms; allow transport/tick rounding slack, as
 * in IST8306. Mode transitions have no specified maximum: this is a bounded
 * host readiness timeout, not a claimed hardware timing specification.
 * A synchronous proxy transaction itself may exceed this polling budget.
 */
#define ICT153XX_READY_TIMEOUT_MS 5

static uint8_t configured_ctrl = ICT153XX_CONFIG_INVALID;
static bool mode_dirty;
static bool reset_needed;
static bool oneshot_pending;
static bool oneshot_failed;
static int64_t oneshot_deadline;
static float last_temperature = NAN;

LOG_MODULE_REGISTER(ICT153xx, LOG_LEVEL_DBG);

static int16_t ict153xx_decode(const uint8_t *raw)
{
	return (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
}

static int ict153xx_set_mode(uint8_t ctrl)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, ICT153XX_MODE_CTRL, ctrl);
	if (err)
		return err;

	int64_t deadline = k_uptime_get() + ICT153XX_READY_TIMEOUT_MS;
	while (true) {
		uint8_t status;
		err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, ICT153XX_MODE_STATUS, &status);
		if (err)
			return err;
		if ((status & ICT153XX_MODE_MASK) == (ctrl & ICT153XX_MODE_MASK))
			return 0;
		if (k_uptime_get() >= deadline)
			return -ETIMEDOUT;
	}
}

static int ict153xx_sleep_and_drain(uint8_t ctrl)
{
	int err = ict153xx_set_mode(ctrl & ICT153XX_ODR_MASK);
	if (err)
		return err;

	/* Flush previous DRDY and any data latch left by an interrupted transfer.
	 * Prefetch stays disabled for the entire ICT lifecycle. */
	uint8_t discard[8];
	return ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, ICT153XX_TEMP_DATA_LSB, discard, sizeof(discard));
}

static int ict153xx_apply_mode(uint8_t ctrl)
{
	int err = ict153xx_sleep_and_drain(ctrl);
	if (!err && (ctrl & ICT153XX_MODE_MASK) == ICT153XX_PULSED)
		err = ict153xx_set_mode(ctrl);
	return err;
}

static int ict153xx_update_odr(float time, float *actual_time)
{
	/* Classify IEEE-754 nonfinite inputs explicitly, including under the
	 * firmware's finite-math optimizations. A NaN is not an ODR request. */
	uint32_t time_bits;
	memcpy(&time_bits, &time, sizeof(time_bits));
	if ((time_bits & 0x7FFFFFFF) > 0x7F800000)
		return -EINVAL;

	uint8_t ctrl;
	if (time_bits == 0x7F800000) {
		ctrl = ICT153XX_SINGLE; // configure sleep; mag_oneshot starts conversion
	} else if (time <= 0) {
		ctrl = ICT153XX_SLEEP;
		time = 0;
	} else {
		/* Quantize periods directly: never truncate a requested fractional Hz.
		 * Select at least the requested nominal rate, capped at 320 Hz. */
		static const float periods[] = {
			1.0f / 5, 1.0f / 10, 1.0f / 20, 1.0f / 50,
			1.0f / 100, 1.0f / 200, 1.0f / 320
		};
		static const uint8_t odrs[] = {4, 3, 2, 1, 0, 6, 5};
		unsigned int i = 0;
		while (i < ARRAY_SIZE(periods) - 1 && time < periods[i])
			i++;
		ctrl = (odrs[i] << 4) | ICT153XX_PULSED;
		time = periods[i];
	}

	if (ctrl != configured_ctrl || mode_dirty) {
		oneshot_pending = false;
		oneshot_failed = false;
		int err = ict153xx_apply_mode(ctrl);
		if (err) {
			configured_ctrl = ICT153XX_CONFIG_INVALID;
			mode_dirty = true;
			last_temperature = NAN;
			LOG_ERR("Mode configuration failed: %d", err);
			return err;
		}
		configured_ctrl = ctrl;
		mode_dirty = false;
	}
	*actual_time = time;
	return 0;
}

static int ict153xx_init(float time, float *actual_time)
{
	configured_ctrl = ICT153XX_CONFIG_INVALID;
	mode_dirty = true;
	reset_needed = true;
	oneshot_pending = false;
	oneshot_failed = false;
	last_temperature = NAN;
	if (sensor_interface_get_spec(SENSOR_INTERFACE_DEV_MAG) == SENSOR_INTERFACE_SPEC_SPI)
		return -ENOTSUP;

	/* Reading data acknowledges DRDY: speculative proxy reads are destructive. */
	int err = sensor_interface_ext_set_prefetch(false);
	if (err)
		return err;

	k_msleep(2); // Table 5: maximum power-on boot time, including OTP download
	uint8_t id[2];
	err = ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, ICT153XX_MANU_ID, id, sizeof(id));
	if (err) // Section 10.4.1.1 permits NACK on the very first I2C transaction
		err = ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, ICT153XX_MANU_ID, id, sizeof(id));
	if (err)
		return err;
	if (id[0] != ICT153XX_MANU_ID_VALUE || id[1] != ICT153XX_CHIP_ID_VALUE)
		return -ENODEV;

	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, ICT153XX_GLOBAL_LOCK, 0xCA);
	if (err) {
		/* A failed transfer can still have unlocked the device. */
		ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, ICT153XX_GLOBAL_LOCK, 0x00);
		return err;
	}
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, ICT153XX_SEQUENCER_CTRL, 0x80);
	/* Also wait/lock on a failed reset transfer: the write may have reached
	 * the device. Section 10.5 gives no separate soft-reset boot maximum. */
	k_msleep(2);
	int lock_err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, ICT153XX_GLOBAL_LOCK, 0x00);
	if (err || lock_err)
		return err ? err : lock_err;
	return ict153xx_update_odr(time, actual_time);
}

static void ict153xx_shutdown(void)
{
	uint8_t ctrl = configured_ctrl == ICT153XX_CONFIG_INVALID ? 0 : configured_ctrl;
	configured_ctrl = ICT153XX_CONFIG_INVALID;
	mode_dirty = true;
	reset_needed = true;
	oneshot_pending = false;
	oneshot_failed = false;
	last_temperature = NAN;
	int err = sensor_interface_ext_set_prefetch(false);
	if (!err)
		err = ict153xx_set_mode(ctrl & ICT153XX_ODR_MASK);
	if (err)
		LOG_ERR("Sleep failed: %d", err);
}

static void ict153xx_mag_oneshot(void)
{
	oneshot_pending = false;
	oneshot_failed = false;
	if (configured_ctrl == ICT153XX_CONFIG_INVALID ||
	    (configured_ctrl & ICT153XX_MODE_MASK) != ICT153XX_SINGLE)
		return;

	int err = ict153xx_sleep_and_drain(configured_ctrl);
	if (!err)
		err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, ICT153XX_MODE_CTRL, configured_ctrl);
	oneshot_deadline = k_uptime_get() + ICT153XX_READY_TIMEOUT_MS;
	oneshot_pending = !err;
	oneshot_failed = err != 0;
	mode_dirty = err != 0;
	if (err) {
		last_temperature = NAN;
		LOG_ERR("Single-shot trigger failed: %d", err);
	}
}

static void ict153xx_mag_process(uint8_t *raw_m, float m[3])
{
	for (int i = 0; i < 3; i++)
		m[i] = ict153xx_decode(&raw_m[i * 2]) * 0.00075f; // 75 nT/LSB to gauss
}

static int ict153xx_magnetic_reset(void)
{
	int err = ict153xx_sleep_and_drain(configured_ctrl);
	if (err)
		return err;
	err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, ICT153XX_MODE_CTRL,
	                         (configured_ctrl & ICT153XX_ODR_MASK) | ICT153XX_MRM);
	/* Even on a bus error the command may have arrived: leave MRM only after
	 * the specified minimum and attempt to exit its high-current state. */
	k_busy_wait(10);
	int resume_err = ict153xx_apply_mode(configured_ctrl);
	return err ? err : resume_err;
}

static bool ict153xx_mag_read(float m[3])
{
	if (configured_ctrl == ICT153XX_CONFIG_INVALID ||
	    (configured_ctrl & ICT153XX_MODE_MASK) == ICT153XX_SLEEP)
		return false;
	if (oneshot_failed) {
		oneshot_failed = false;
		return false;
	}
	if (mode_dirty) {
		mode_dirty = ict153xx_apply_mode(configured_ctrl) != 0;
		return false;
	}
	if ((configured_ctrl & ICT153XX_MODE_MASK) == ICT153XX_SINGLE && !oneshot_pending)
		return false;

	/* Check DRDY before consuming the latched frame. Proxy prefetch is disabled,
	 * so neither status nor data can come from a speculative background read. */
	uint8_t status;
	while (true) {
		int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, ICT153XX_STATUS, &status);
		if (err)
			goto read_error;
		if (status & 0x01)
			break;
		if (!oneshot_pending)
			return false;
		if (k_uptime_get() >= oneshot_deadline) {
			LOG_ERR("Single-shot read timeout");
			goto read_error;
		}
	}

	uint8_t frame[8];
	if (ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, ICT153XX_TEMP_DATA_LSB, frame, sizeof(frame)))
		goto read_error;
	oneshot_pending = false;
	last_temperature = ict153xx_decode(frame) * 0.00625f + 25.0f;

	float sample[3];
	ict153xx_mag_process(&frame[2], sample);
	float xy_squared = sample[0] * sample[0] + sample[1] * sample[1];
	float norm_squared = xy_squared + sample[2] * sample[2];
	bool clipped = false;
	for (int i = 0; i < 3; i++) {
		int16_t raw = ict153xx_decode(&frame[2 + i * 2]);
		clipped |= raw == INT16_MIN || raw == INT16_MAX;
	}
	if (clipped || norm_squared >= 24.0f * 24.0f) {
		reset_needed = true;
		return false;
	}
	if (reset_needed) {
		/* Section 10.1.4: no reset in an unsafe external field. The initial
		 * sample establishes this precondition but is not valid fusion data.
		 * In single-ready mode the next caller trigger takes the post-MRM
		 * sample; staying ready rather than invalidating ODR avoids deadlock. */
		if (xy_squared < 12.0f * 12.0f) {
			int err = ict153xx_magnetic_reset();
			mode_dirty = err != 0;
			reset_needed = err != 0;
			if (err) {
				last_temperature = NAN;
				LOG_ERR("Magnetic reset failed: %d", err);
			}
		}
		return false;
	}
	for (int i = 0; i < 3; i++)
		m[i] = sample[i];
	return true;

read_error:
	oneshot_pending = false;
	last_temperature = NAN;
	mode_dirty = true; // do not publish a partially-read, latched or late frame
	return false;
}

static float ict153xx_temp_read(float bias[3])
{
	(void)bias;
	/* Reading only 0x08..0x09 would leave the entire frame latched. */
	return last_temperature;
}

const sensor_mag_t sensor_mag_ict153xx = {
	*ict153xx_init,
	*ict153xx_shutdown,
	*ict153xx_update_odr,
	*ict153xx_mag_oneshot,
	*ict153xx_mag_read,
	*ict153xx_temp_read,
	*ict153xx_mag_process,
	/* The logical eight-byte frame is latched until 0x0F is read. SSI splits
	 * it into 7+1 bytes on LSM6DSV without speculative reads between chunks. */
	1, 8
};
