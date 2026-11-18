#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sensor/interface_internal.h"
#include "sensor/mag/QMC5883P.h"

/* QST-PD-B002-22 Rev E register model, not an electrical simulation: bus
 * timing, analogue offsets and self-test settling are out of scope. */
enum {
	REG_CHIPID = 0x00,
	REG_DATA = 0x01,
	REG_STATUS = 0x09,
	REG_CTRL1 = 0x0A,
	REG_CTRL2 = 0x0B,
	REG_SIGN = 0x29,
};
enum { MODE_SUSPEND = 0, MODE_NORMAL = 1, MODE_SINGLE = 2, MODE_CONTINUOUS = 3 };
enum { STAT_DRDY = 0x01, STAT_OVFL = 0x02 };

static const sensor_mag_t *const mag = &sensor_mag_qmc5883p;
static unsigned failures;

#define CHECK(cond)                                                                                                    \
	do {                                                                                                           \
		if (!(cond)) {                                                                                         \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                        \
			failures++;                                                                                    \
		}                                                                                                      \
	} while (0)

static struct {
	uint8_t regs[256];
	uint8_t mode;
	int16_t field[3];
	int64_t now_ms;
	int64_t single_at_ms;
	uint32_t single_delay_ms;
	unsigned writes;
	unsigned transits;
	unsigned resets;
	unsigned singles;
	unsigned bursts;
	unsigned frame_reads;
	unsigned status_reads;
	unsigned signed_writes;
	bool ext;
	bool reset_pending;
	uint8_t fail_write_reg;
	uint8_t fail_write_value;
	bool apply_failed_write;
	bool fail_status_read;
	bool fail_burst;
} hw;

/* Publish magnetic data without changing the field for the next single shot. */
static void next_sample(int16_t x, int16_t y, int16_t z)
{
	hw.regs[REG_DATA + 0] = (uint16_t)x & 0xff;
	hw.regs[REG_DATA + 1] = (uint16_t)x >> 8;
	hw.regs[REG_DATA + 2] = (uint16_t)y & 0xff;
	hw.regs[REG_DATA + 3] = (uint16_t)y >> 8;
	hw.regs[REG_DATA + 4] = (uint16_t)z & 0xff;
	hw.regs[REG_DATA + 5] = (uint16_t)z >> 8;
	hw.regs[REG_STATUS] |= STAT_DRDY;
}

static void settle(void)
{
	if (hw.single_at_ms && hw.now_ms >= hw.single_at_ms) {
		next_sample(hw.field[0], hw.field[1], hw.field[2]);
		hw.mode = MODE_SUSPEND;
		hw.single_at_ms = 0;
	}
}

static void advance(int64_t ms)
{
	hw.now_ms += ms;
	settle();
}

static void reset_hw(void)
{
	memset(&hw, 0, sizeof(hw));
	hw.regs[REG_CHIPID] = 0x80;
	hw.mode = MODE_SUSPEND;
	hw.single_delay_ms = 1;
}

/* QMC5883P has no temperature register, so the driver reuses the "none" hook. */
float mag_none_temp_read(float bias[3])
{
	(void)bias;
	return 0.0f;
}

/* Host clock: the driver reads it for timeouts and duplicate suppression. */
int64_t host_k_uptime_get(void) { return hw.now_ms; }
void host_k_busy_wait(uint32_t us) { advance((int64_t)us / 1000 + 1); }
void host_k_msleep(int32_t ms) { assert(ms >= 0); advance(ms); }
void host_k_usleep(int32_t us) { assert(us >= 0); advance((int64_t)us / 1000 + 1); }

enum sensor_interface_spec sensor_interface_get_spec(enum sensor_interface_dev dev)
{
	assert(dev == SENSOR_INTERFACE_DEV_MAG);
	return hw.ext ? SENSOR_INTERFACE_SPEC_EXT : SENSOR_INTERFACE_SPEC_SPI;
}

static int write_register(uint8_t reg, uint8_t value)
{
	advance(1);
	hw.writes++;
	hw.regs[reg] = value;

	if (reg == REG_SIGN) {
		hw.signed_writes++;
		assert(value == 0x06); /* Only the documented power-on value. */
		return 0;
	}
	if (reg == REG_CTRL2) {
		if (value & 0x80) { /* soft reset */
			hw.resets++;
			hw.reset_pending = true;
			hw.mode = MODE_SUSPEND;
			hw.single_at_ms = 0;
			hw.regs[REG_STATUS] = 0;
			hw.regs[REG_CTRL1] = 0;
			return 0;
		}
		if (value == 0x00) {
			/* The reset bit self-clears; only honour the explicit
			 * write-back that follows a reset. */
			CHECK(hw.reset_pending);
			hw.reset_pending = false;
			return 0;
		}
		CHECK(value == 0x08); /* RNG_8G | SET_RESET_ON; never 0x41/0x01. */
		hw.reset_pending = false;
		return 0;
	}
	if (reg == REG_CTRL1) {
		uint8_t mode = value & 3;
		if (mode == MODE_SUSPEND) {
			if (hw.mode != MODE_SUSPEND)
				hw.transits++;
			hw.mode = MODE_SUSPEND;
			hw.single_at_ms = 0;
			return 0;
		}
		/* A running mode may only be entered from suspend, or stay in the
		 * same mode for an ODR-only change. */
		assert(hw.mode == MODE_SUSPEND || hw.mode == mode);
		hw.mode = mode;
		if (mode == MODE_SINGLE) {
			hw.singles++;
			hw.single_at_ms = hw.now_ms + hw.single_delay_ms;
		}
		return 0;
	}
	return 0;
}

int ssi_reg_write_byte(enum sensor_interface_dev dev, uint8_t reg, uint8_t value)
{
	assert(dev == SENSOR_INTERFACE_DEV_MAG);
	if (reg == hw.fail_write_reg && value == hw.fail_write_value) {
		hw.fail_write_reg = 0;
		if (hw.apply_failed_write)
			write_register(reg, value);
		return -EIO;
	}
	return write_register(reg, value);
}
int ssi_reg_write_byte_io(enum sensor_interface_dev dev, uint8_t reg, uint8_t value,
	enum ssi_reporting reporting)
{
	(void)reporting;
	return ssi_reg_write_byte(dev, reg, value);
}

static int read_frame(uint8_t reg, uint8_t *buf, uint32_t len)
{
	advance(1);
	assert(reg == REG_DATA && len == 6);
	memcpy(buf, &hw.regs[REG_DATA], 6);
	return 0;
}

int ssi_burst_read(enum sensor_interface_dev dev, uint8_t reg, uint8_t *buf, uint32_t len)
{
	assert(dev == SENSOR_INTERFACE_DEV_MAG);
	hw.bursts++;
	if (hw.fail_burst) {
		hw.fail_burst = false;
		return -EIO;
	}
	return read_frame(reg, buf, len);
}
int ssi_burst_read_io(enum sensor_interface_dev dev, uint8_t reg, uint8_t *buf, uint32_t len,
	enum ssi_reporting reporting)
{
	(void)reporting;
	return ssi_burst_read(dev, reg, buf, len);
}
int ssi_burst_read_frame(enum sensor_interface_dev dev, uint8_t reg, uint8_t dummy, uint8_t word,
	uint8_t *buf, uint32_t len)
{
	assert(dev == SENSOR_INTERFACE_DEV_MAG);
	assert(dummy == 0 && word == 2);
	hw.frame_reads++;
	return read_frame(reg, buf, len);
}
int ssi_reg_read_byte(enum sensor_interface_dev dev, uint8_t reg, uint8_t *value)
{
	advance(1);
	assert(dev == SENSOR_INTERFACE_DEV_MAG);
	*value = hw.regs[reg];
	/* Reading STATUS acknowledges both flags (Rev E §9.2.2). */
	if (reg == REG_STATUS) {
		hw.status_reads++;
		hw.regs[REG_STATUS] &= (uint8_t)~(STAT_DRDY | STAT_OVFL);
		if (hw.fail_status_read) {
			hw.fail_status_read = false;
			return -EIO;
		}
	}
	return 0;
}

static void expect_read_blocked(void)
{
	float m[3] = {1, 2, 3};
	unsigned status_reads = hw.status_reads;
	unsigned bursts = hw.bursts;
	unsigned frame_reads = hw.frame_reads;
	CHECK(!mag->mag_read(m));
	CHECK(!mag->mag_read(m));
	CHECK(hw.status_reads == status_reads);
	CHECK(hw.bursts == bursts);
	CHECK(hw.frame_reads == frame_reads);
	CHECK(m[0] == 1 && m[1] == 2 && m[2] == 3);
}

static void test_off(void)
{
	reset_hw();
	hw.ext = true;
	next_sample(375, 0, 0);
	expect_read_blocked(); /* Before initialization */
	mag->mag_oneshot();
	CHECK(hw.writes == 0);

	float actual = 0;
	CHECK(mag->init(0, &actual) == 0);
	CHECK(hw.mode == MODE_SUSPEND);
	expect_read_blocked(); /* Powered down, with retained XYZ */
}

static void test_init_sequence(void)
{
	reset_hw();
	float actual = 0;
	CHECK(mag->init(0.01f, &actual) == 0);
	CHECK(hw.signed_writes == 1);
	CHECK(hw.regs[REG_SIGN] == 0x06);
	CHECK(hw.regs[REG_CTRL2] == 0x08);
	CHECK(hw.regs[REG_CTRL1] == 0xC9); /* 100 Hz, OSR 8x//8, normal mode */
	CHECK(hw.mode == MODE_NORMAL);
	CHECK(fabsf(actual - 0.01f) < 1e-6f);
}

static void test_odr_ladder(void)
{
	reset_hw();
	float actual = 0;
	CHECK(mag->update_odr(0.005f, &actual) == 0);
	CHECK(hw.regs[REG_CTRL1] == 0xCD); /* 200 Hz, the datasheet example value */
	CHECK(mag->update_odr(0.02f, &actual) == 0);
	CHECK(hw.regs[REG_CTRL1] == 0xC5); /* 50 Hz */
	CHECK(mag->update_odr(0.2f, &actual) == 0);
	CHECK(hw.regs[REG_CTRL1] == 0xC1); /* 10 Hz */
	CHECK(fabsf(actual - 0.1f) < 1e-6f);
	/* 10 Hz -> 100 Hz changes the mode field, so a suspend transit is due. */
	unsigned before = hw.transits;
	CHECK(mag->update_odr(0.01f, &actual) == 0);
	CHECK(hw.transits == before + 1);
	CHECK(hw.regs[REG_CTRL1] == 0xC9);
}

static void test_oneshot(bool start_normal)
{
	reset_hw();
	hw.ext = true;
	hw.single_delay_ms = 5;
	float actual = 0;
	CHECK(mag->init(start_normal ? 0.01f : INFINITY, &actual) == 0);
	CHECK(hw.mode == (start_normal ? MODE_NORMAL : MODE_SUSPEND));

	next_sample(-7500, 0, 0);
	hw.regs[REG_STATUS] |= STAT_OVFL;
	if (!start_normal) {
		CHECK(isinf(actual));
		expect_read_blocked();
	}
	hw.field[0] = 3750;
	hw.field[1] = -3750;
	hw.field[2] = 375;
	mag->mag_oneshot();
	CHECK(hw.singles == 1);
	CHECK(hw.transits == (unsigned)start_normal);
	CHECK(hw.mode == MODE_SINGLE); /* Old XYZ still present */
	CHECK(hw.regs[REG_STATUS] == 0);
	CHECK(hw.status_reads == 1);
	float m[3] = {0};
	CHECK(mag->mag_read(m));
	CHECK(hw.regs[REG_CTRL1] == (0xC0 | 0x0C | 0x02)); /* OSR | 200 Hz | single */
	CHECK(fabsf(m[0] - 1.0f) < 1e-4f);
	CHECK(fabsf(m[1] + 1.0f) < 1e-4f);
	CHECK(fabsf(m[2] - 0.1f) < 1e-4f);
	CHECK(hw.mode == MODE_SUSPEND);
	CHECK(hw.status_reads > 2); /* Waited for this conversion's readiness */
	CHECK(hw.bursts == 1);
	advance(20);
	expect_read_blocked();
	CHECK(hw.frame_reads == 0);

	/* A fresh trigger is allowed after auto-suspend, not a periodic read. */
	hw.field[0] = 7500;
	mag->mag_oneshot();
	CHECK(hw.singles == 2);
	CHECK(mag->mag_read(m));
	CHECK(fabsf(m[0] - 2.0f) < 1e-4f);
	expect_read_blocked();
}

static void test_oneshot_failures(void)
{
	reset_hw();
	hw.ext = true;
	float actual = 0;
	CHECK(mag->init(INFINITY, &actual) == 0);
	next_sample(3750, 0, 0);
	float m[3] = {0};

	hw.fail_status_read = true; /* Pre-trigger acknowledgement failed. */
	mag->mag_oneshot();
	CHECK(hw.singles == 0);
	expect_read_blocked();

	mag->mag_oneshot();
	hw.fail_burst = true; /* DRDY consumed, but XYZ did not arrive. */
	CHECK(!mag->mag_read(m));
	CHECK(hw.bursts == 1);
	advance(20);
	expect_read_blocked();

	mag->mag_oneshot();
	hw.fail_status_read = true; /* Completion might have been acknowledged. */
	CHECK(!mag->mag_read(m));
	CHECK(hw.bursts == 1);
	expect_read_blocked();

	CHECK(mag->update_odr(INFINITY, &actual) == 0);
	hw.single_delay_ms = 30;
	mag->mag_oneshot();
	CHECK(!mag->mag_read(m)); /* Timeout leaves the eventual data unowned. */
	CHECK(hw.mode == MODE_SINGLE);
	advance(30);
	CHECK(hw.mode == MODE_SUSPEND);
	expect_read_blocked();
	CHECK(hw.frame_reads == 0);
}

static void test_ambiguous_trigger(void)
{
	reset_hw();
	hw.ext = true;
	hw.single_delay_ms = 30;
	float actual = 0;
	CHECK(mag->init(INFINITY, &actual) == 0);
	next_sample(375, 0, 0);

	hw.fail_write_reg = REG_CTRL1;
	hw.fail_write_value = 0xCE;
	hw.apply_failed_write = true;
	mag->mag_oneshot();
	CHECK(hw.mode == MODE_SINGLE); /* Register write reached the chip. */
	expect_read_blocked();
	mag->mag_oneshot();
	CHECK(hw.singles == 1); /* Must not retry an uncertain trigger. */

	CHECK(mag->update_odr(0.01f, &actual) == 0);
	CHECK(hw.transits == 1);
	CHECK(hw.mode == MODE_NORMAL);
}

static void test_failed_configuration(void)
{
	reset_hw();
	hw.ext = true;
	float actual = 0;
	CHECK(mag->init(0.01f, &actual) == 0);
	next_sample(3750, 0, 0);

	hw.fail_write_reg = REG_SIGN;
	hw.fail_write_value = 0x06;
	mag->mag_oneshot(); /* Failed normal-to-suspend preparation. */
	CHECK(hw.singles == 0);
	CHECK(hw.mode == MODE_NORMAL);
	expect_read_blocked();

	CHECK(mag->update_odr(0.01f, &actual) == 0);
	float m[3] = {0};
	CHECK(mag->mag_read(m));
}

static void test_scale(void)
{
	uint8_t raw[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	float m[3];
	mag->mag_process(raw, m);
	CHECK(m[0] == 0.0f && m[1] == 0.0f && m[2] == 0.0f);

	uint8_t one_g[6] = {0xA6, 0x0E, 0x5A, 0xF1, 0x64, 0x00}; /* 3750, -3750, 100 */
	mag->mag_process(one_g, m);
	CHECK(fabsf(m[0] - 1.0f) < 1e-4f);
	CHECK(fabsf(m[1] + 1.0f) < 1e-4f);
	CHECK(fabsf(m[2] - (100.0f / 3750.0f)) < 1e-6f);
}

static void test_status_overflow(void)
{
	reset_hw();
	float actual = 0;
	CHECK(mag->init(0.01f, &actual) == 0);
	next_sample(100, 200, 300);
	hw.regs[REG_STATUS] = STAT_DRDY | STAT_OVFL;
	float m[3] = {0};
	CHECK(!mag->mag_read(m));

	CHECK(hw.regs[REG_STATUS] == 0);
	/* Neither overflow nor not-ready should reach the periodic frame reader. */
	CHECK(!mag->mag_read(m));
	CHECK(hw.frame_reads == 0);
}

static void test_saturation_guard(void)
{
	reset_hw();
	float actual = 0;
	CHECK(mag->init(0.01f, &actual) == 0);
	float m[3] = {0};

	next_sample(30000, 0, 0);
	CHECK(!mag->mag_read(m));
	next_sample(0, -30000, 0);
	CHECK(!mag->mag_read(m));
	next_sample(29999, 0, 0);
	CHECK(mag->mag_read(m));
	CHECK(fabsf(m[0] - (29999.0f / 3750.0f)) < 1e-5f);
}

static void test_external_duplicate_suppression(void)
{
	reset_hw();
	hw.ext = true;
	float actual = 0;
	CHECK(mag->init(0.01f, &actual) == 0);
	next_sample(375, 0, 0);
	float m[3] = {0};
	CHECK(mag->mag_read(m));
	CHECK(mag->mag_read(m) == false);      /* identical frame inside the window */
	advance(20);
	CHECK(mag->mag_read(m));               /* same bytes, window expired */
	next_sample(376, 0, 0);
	CHECK(mag->mag_read(m));               /* changed frame is not suppressed */
	CHECK(hw.status_reads == 0);
	CHECK(hw.regs[REG_STATUS] == STAT_DRDY); /* XYZ reads don't acknowledge it. */
	CHECK(hw.frame_reads >= 4);

	/* At 200 Hz the 5.5 ms threshold rounds up on the millisecond clock. */
	CHECK(mag->update_odr(0.005f, &actual) == 0);
	CHECK(mag->mag_read(m));
	advance(4); /* Each modeled frame read takes 1 ms. */
	CHECK(!mag->mag_read(m)); /* 5 ms since acceptance */
	CHECK(mag->mag_read(m));  /* 6 ms since acceptance */
}

static void test_shutdown(void)
{
	reset_hw();
	hw.ext = true;
	float actual = 0;
	CHECK(mag->init(0.01f, &actual) == 0);
	next_sample(3750, 0, 0);
	mag->shutdown();
	CHECK(hw.resets == 1);
	CHECK(hw.regs[REG_CTRL2] == 0x00);
	CHECK(hw.mode == MODE_SUSPEND);
	expect_read_blocked();
}

int main(void)
{
	test_off();
	test_init_sequence();
	test_odr_ladder();
	test_oneshot(false);
	test_oneshot(true);
	test_oneshot_failures();
	test_ambiguous_trigger();
	test_failed_configuration();
	test_scale();
	test_status_overflow();
	test_saturation_guard();
	test_external_duplicate_suppression();
	test_shutdown();
	if (failures) {
		printf("%u failure(s)\n", failures);
		return 1;
	}
	printf("QMC5883P host test: ok\n");
	return 0;
}
