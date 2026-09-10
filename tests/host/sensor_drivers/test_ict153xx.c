#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sensor/mag/ICT153xx.h"

/* DS000548 register model, not an SSI call transcript. No analogue offsets,
 * electrical bus timing, or real IMU proxy implementation are simulated. */
enum { MODE = 0x04, MODE_STATUS = 0x05, STATUS = 0x06, DATA = 0x08,
	RESET = 0x7c, LOCK = 0x7f, SLEEP = 0, PULSED = 1, SINGLE = 2, MRM = 3 };
static const sensor_mag_t *const mag = &sensor_mag_ict153xx;
static struct {
	uint8_t regs[256], latched[8], mode, requested;
	int16_t field[3], temperature;
	uint64_t now, transition_at, single_at, boot_at, mrm_at;
	unsigned transactions, fail_at, failures, resets, mrms;
	bool unlocked, latch, drdy, proxy, prefetch;
	int prefetch_error;
	uint32_t single_write_delay;
	uint8_t cache[8], cache_reg;
	uint32_t cache_len;
	uint64_t cache_at, prefetch_at;
} hw;

static void put16(uint8_t *p, int16_t value)
{
	p[0] = (uint16_t)value & 0xff;
	p[1] = (uint16_t)value >> 8;
}

static void capture(void)
{
	put16(hw.regs + DATA, hw.temperature);
	for (unsigned i = 0; i < 3; i++)
		put16(hw.regs + DATA + 2 + i * 2, hw.field[i]);
	hw.drdy = true;
}

static void settle(void)
{
	if (hw.transition_at && hw.now >= hw.transition_at) {
		hw.mode = hw.requested;
		hw.transition_at = 0;
	}
	if (hw.single_at && hw.now >= hw.single_at) {
		capture();
		hw.mode = SLEEP;
		hw.single_at = 0;
	}
}

static void prefetch_frame(void);

static void advance(uint64_t us)
{
	hw.now += us;
	settle();
	if (hw.prefetch_at && hw.now >= hw.prefetch_at)
		prefetch_frame();
}

int64_t host_k_uptime_get(void)
{
	advance(1);
	return hw.now / 1000;
}
void host_k_busy_wait(uint32_t us) { advance(us); }
void host_k_msleep(int32_t ms) { assert(ms >= 0); advance((uint64_t)ms * 1000); }
void host_k_usleep(int32_t us) { assert(us >= 0); advance(us); }

static bool transaction(void)
{
	assert(++hw.transactions < 10000); /* A stuck polling loop is a model failure. */
	advance(1);
	if (hw.fail_at == hw.transactions) {
		hw.failures++;
		return false;
	}
	return true;
}

static bool safe_field(void)
{
	double x = hw.field[0] * 0.00075, y = hw.field[1] * 0.00075;
	double z = hw.field[2] * 0.00075;
	return x * x + y * y < 144 && x * x + y * y + z * z < 576;
}

static int write_register(uint8_t reg, uint8_t value)
{
	assert(reg != 0x21); /* Factory calibration is not a runtime setting. */
	if (reg == LOCK) {
		assert(value == 0xca || value == 0);
		hw.unlocked = value == 0xca;
	} else if (reg == RESET) {
		assert(hw.unlocked && value == 0x80);
		hw.resets++;
		hw.mode = hw.requested = SLEEP;
		hw.transition_at = hw.single_at = 0;
		hw.drdy = hw.latch = false;
		hw.boot_at = hw.now + 2000;
		hw.regs[MODE] = 0;
	} else if (reg == MODE) {
		assert(!hw.unlocked && hw.now >= hw.boot_at);
		uint8_t next = value & 3;
		assert((value >> 4) <= 6);
		if (next == MRM) {
			assert(hw.mode == SLEEP && !hw.transition_at && safe_field());
			hw.mrm_at = hw.now;
			hw.mrms++;
		} else if (hw.mode == MRM) {
			assert(next == SLEEP && hw.now - hw.mrm_at >= 10);
		}
		if (next != SLEEP && next != hw.mode)
			assert(hw.mode == SLEEP && !hw.transition_at);
		hw.requested = next;
		if (next == SLEEP && hw.mode != SLEEP) {
			hw.transition_at = hw.now + 100;
			hw.single_at = 0;
		} else {
			hw.mode = next;
			if (next == SINGLE)
				hw.single_at = hw.now + 3355;
		}
	}
	hw.regs[reg] = value;
	return 0;
}

int ssi_reg_write_byte(enum sensor_interface_dev dev, uint8_t reg, uint8_t value)
{
	assert(dev == SENSOR_INTERFACE_DEV_MAG);
	if (!transaction())
		return -EIO;
	uint32_t restore_len = hw.cache_len;
	hw.prefetch_at = 0;
	int err = write_register(reg, value);
	if (!err && reg == MODE && (value & 3) == SINGLE)
		advance(hw.single_write_delay);
	/* A proxy may restore its previous read after a write. This transaction
	 * occurs after the scheduler pause, even if conversion finished meanwhile. */
	if (!err && hw.proxy && hw.prefetch && restore_len) {
		hw.cache_len = restore_len;
		prefetch_frame();
	} else {
		hw.cache_len = 0;
	}
	return err;
}

static int read_device(uint8_t reg, uint8_t *data, uint32_t len)
{
	assert(len && len <= 8 && (unsigned)reg + len <= 256);
	for (uint32_t i = 0; i < len; i++) {
		unsigned address = reg + i;
		if (address == MODE_STATUS)
			data[i] = hw.mode;
		else if (address == STATUS)
			data[i] = hw.drdy ? 1 : 0;
		else if (address >= DATA && address < DATA + 8) {
			if (!hw.latch) {
				memcpy(hw.latched, hw.regs + DATA, 8);
				hw.latch = true;
			}
			data[i] = hw.latched[address - DATA];
			if (address == DATA + 7)
				hw.drdy = hw.latch = false;
		} else
			data[i] = hw.regs[address];
	}
	return 0;
}

static void prefetch_frame(void)
{
	read_device(hw.cache_reg, hw.cache, hw.cache_len);
	hw.cache_at = hw.now;
	hw.prefetch_at = hw.now + 1000;
}

static int read_registers(uint8_t reg, uint8_t *data, uint32_t len)
{
	if (!hw.proxy || !hw.prefetch)
		return read_device(reg, data, len);
	if (hw.cache_len == len && hw.cache_reg == reg
		&& hw.now - hw.cache_at < 1000) {
		memcpy(data, hw.cache, len);
		return 0;
	}
	/* Reconfiguring a status read discards any speculative data frame. */
	hw.cache_len = len;
	hw.cache_reg = reg;
	hw.cache_at = hw.now;
	prefetch_frame();
	memcpy(data, hw.cache, len);
	return 0;
}

int ssi_burst_read(enum sensor_interface_dev dev, uint8_t reg, uint8_t *data, uint32_t len)
{
	assert(dev == SENSOR_INTERFACE_DEV_MAG);
	if (!transaction()) {
		/* A failed transfer is allowed to have partially overwritten its buffer. */
		if (reg == DATA && len == 8)
			read_registers(reg, data, 7); /* Interrupted before read-clear: latch persists. */
		if (len)
			data[0] = 0xa5;
		return -EIO;
	}
	return read_registers(reg, data, len);
}
int ssi_reg_read_byte(enum sensor_interface_dev dev, uint8_t reg, uint8_t *value)
{
	return ssi_burst_read(dev, reg, value, 1);
}
enum sensor_interface_spec sensor_interface_get_spec(enum sensor_interface_dev dev)
{
	assert(dev == SENSOR_INTERFACE_DEV_MAG);
	return hw.proxy ? SENSOR_INTERFACE_SPEC_EXT : SENSOR_INTERFACE_SPEC_I2C;
}

int sensor_interface_ext_set_prefetch(bool enabled)
{
	if (!hw.proxy)
		return 0;
	if (hw.prefetch_error)
		return hw.prefetch_error;
	hw.prefetch = enabled;
	hw.cache_len = 0;
	hw.prefetch_at = 0;
	return 0;
}

static void reset_model(bool proxy)
{
	memset(&hw, 0, sizeof(hw));
	hw.now = 10000;
	hw.regs[0] = 0xe7;
	hw.regs[1] = 0x45;
	hw.proxy = proxy;
	hw.prefetch = true;
	hw.field[0] = 400;
	hw.field[1] = -800;
	hw.field[2] = 1200;
	hw.temperature = -160;
}

static void frame(int16_t x, int16_t y, int16_t z, int16_t temperature)
{
	advance(6000); /* Allow both conversion completion and proxy refresh. */
	hw.field[0] = x;
	hw.field[1] = y;
	hw.field[2] = z;
	hw.temperature = temperature;
	if (hw.mode == PULSED)
		capture();
}

static void no_sample(void)
{
	float value[3] = {123, 456, 789};
	assert(!mag->mag_read(value));
	assert(value[0] == 123 && value[1] == 456 && value[2] == 789);
}

static void good_sample(int16_t x, int16_t y, int16_t z, int16_t temperature)
{
	float value[3] = {123, 456, 789}, bias[3] = {7, 8, 9};
	assert(mag->mag_read(value));
	assert(fabsf(value[0] - x * 0.00075f) < 0.00001f);
	assert(fabsf(value[1] - y * 0.00075f) < 0.00001f);
	assert(fabsf(value[2] - z * 0.00075f) < 0.00001f);
	assert(fabsf(mag->temp_read(bias) - (temperature * 0.00625f + 25)) < 0.00001f);
}

static void initialize(bool proxy)
{
	reset_model(proxy);
	float actual = -123;
	assert(mag->init(1.0f / 200, &actual) == 0);
	assert(fabsf(actual - 1.0f / 200) < 1e-7f);
	assert(hw.resets > 0 && !hw.unlocked);
	assert(hw.mrms == 0); /* Startup cannot reset magnets before observing field. */
}

static void running(bool proxy)
{
	initialize(proxy);
	frame(400, -800, 1200, -160);
	no_sample();
	assert(hw.mrms == 1);
	frame(400, -800, 1200, -160);
	good_sample(400, -800, 1200, -160);
}

static void recover_to_sample(void);

static void test_startup_and_freshness(void)
{
	for (unsigned proxy = 0; proxy < 2; proxy++) {
		running(proxy);
		no_sample();
		no_sample(); /* Same-shape cached reads must not duplicate a delivered frame. */
		frame(-1200, 800, -400, 320);
		hw.fail_at = hw.transactions + 1;
		no_sample();
		hw.fail_at = 0;
		recover_to_sample();
		frame(800, 400, -1200, -320);
		hw.fail_at = hw.transactions + 2;
		no_sample();
		hw.fail_at = 0;
		recover_to_sample();
		no_sample();
	}
}

static void test_overrange_and_recovery(void)
{
	running(false);
	unsigned resets = hw.mrms;
	frame(0, 0, 32000, 0); /* Exactly 24 G: invalid, not merely >24 G. */
	no_sample();
	assert(hw.mrms == resets);
	frame(16000, 0, 0, 0); /* Exactly 12 G XY: reset remains unsafe. */
	no_sample();
	assert(hw.mrms == resets);
	frame(12000, 12000, 0, 0); /* Each axis <12, but XY norm >12. */
	no_sample();
	assert(hw.mrms == resets);
	frame(15999, 0, 0, 0);
	no_sample();
	assert(hw.mrms == resets + 1);
	frame(400, -800, 1200, -160);
	good_sample(400, -800, 1200, -160);
	frame(23000, 23000, 0, 0); /* Vector norm, not per-axis thresholds. */
	no_sample();
	assert(hw.mrms == resets + 1);
	/* Large Z alone is safe for MRM only if total norm remains below 24 G. */
	frame(0, 0, 32767, 0);
	no_sample();
	frame(0, 0, 31999, 0);
	no_sample();
	assert(hw.mrms == resets + 2);
	frame(0, 0, 31999, 0);
	good_sample(0, 0, 31999, 0);
}

static void expect_period(float requested, float hz, unsigned code)
{
	float actual = -123;
	assert(mag->update_odr(requested, &actual) >= 0);
	assert(fabsf(actual - 1.0f / hz) < 1e-7f);
	advance(1000);
	assert(hw.mode == PULSED && hw.regs[MODE] >> 4 == code);
}

static void test_odr_and_single(void)
{
	running(false);
	static const struct { float hz; unsigned code; } rates[] = {
		{5, 4}, {10, 3}, {20, 2}, {50, 1}, {100, 0}, {200, 6}, {320, 5},
	};
	for (unsigned i = 0; i < ARRAY_SIZE(rates); i++) {
		float period = 1.0f / rates[i].hz;
		expect_period(period, rates[i].hz, rates[i].code);
		expect_period(nextafterf(period, INFINITY), rates[i].hz, rates[i].code);
		unsigned next = i + 1 < ARRAY_SIZE(rates) ? i + 1 : i;
		expect_period(nextafterf(period, 0), rates[next].hz, rates[next].code);
	}
	expect_period(1, 5, 4);
	expect_period(0.0001f, 320, 5);
	expect_period(nextafterf(0, 1), 320, 5);
	float actual = -123;
	assert(mag->update_odr(NAN, &actual) < 0 && actual == -123);
	frame(400, -800, 1200, -160);
	good_sample(400, -800, 1200, -160);
	assert(mag->update_odr(0, &actual) >= 0 && actual == 0);
	advance(1000);
	assert(hw.mode == SLEEP);
	no_sample();
	assert(mag->update_odr(-1, &actual) >= 0 && actual == 0);
	assert(mag->update_odr(-INFINITY, &actual) >= 0 && actual == 0);
	assert(mag->update_odr(INFINITY, &actual) >= 0 && isinf(actual) && actual > 0);
	advance(1000);
	assert(hw.mode == SLEEP);
	hw.field[0] = 100;
	hw.field[1] = -200;
	hw.field[2] = 300;
	hw.temperature = 400;
	mag->mag_oneshot();
	good_sample(100, -200, 300, 400);
	no_sample();
	mag->mag_oneshot();
	advance(1000);
	good_sample(100, -200, 300, 400);
	mag->shutdown();
	advance(1000);
	assert(hw.mode == SLEEP);
	/* Startup MRM must preserve single-ready configuration for the next trigger. */
	reset_model(false);
	assert(mag->init(INFINITY, &actual) == 0 && isinf(actual));
	mag->mag_oneshot();
	no_sample();
	assert(hw.mrms == 1);
	mag->mag_oneshot();
	good_sample(400, -800, 1200, -160);
	no_sample();
}

static void test_single_survives_speculative_read(void)
{
	for (unsigned proxy = 0; proxy < 2; proxy++) {
		running(proxy);
		float actual = -123;
		assert(mag->update_odr(INFINITY, &actual) >= 0 && isinf(actual));
		hw.single_write_delay = 4000;
		hw.field[0] = 100;
		hw.field[1] = -200;
		hw.field[2] = 300;
		hw.temperature = 400;
		/* sleep-and-drain reads 08..0f immediately before SINGLE. With
		 * speculative reads enabled, restoring that frame after this pause
		 * consumes DRDY; the subsequent status read discards the cached frame. */
		mag->mag_oneshot();
		good_sample(100, -200, 300, 400);
		no_sample();
		hw.field[0] = -300;
		hw.temperature = -400;
		mag->mag_oneshot();
		good_sample(-300, -200, 300, -400);
		no_sample();
	}
}

static void test_prefetch_disable_failure(void)
{
	running(true); /* A failed re-init must invalidate a formerly valid driver. */
	uint8_t before[sizeof(hw.regs)];
	memcpy(before, hw.regs, sizeof(before));
	uint64_t before_time = hw.now;
	hw.prefetch_error = -EIO;
	float actual = -123;
	assert(mag->init(1.0f / 200, &actual) == -EIO);
	assert(actual == -123 && hw.now == before_time);
	assert(memcmp(before, hw.regs, sizeof(before)) == 0);
	frame(100, -200, 300, 400);
	no_sample();
	float bias[3] = {7, 8, 9};
	assert(isnan(mag->temp_read(bias)));
	hw.prefetch_error = 0;
	assert(mag->init(1.0f / 200, &actual) == 0);
	recover_to_sample();
}

static void recover_to_sample(void)
{
	for (unsigned attempt = 0; attempt < 8; attempt++) {
		frame(400, -800, 1200, -160);
		float value[3] = {123, 456, 789};
		if (mag->mag_read(value)) {
			assert(fabsf(value[0] - 0.3f) < 1e-6f);
			assert(fabsf(value[1] + 0.6f) < 1e-6f);
			assert(fabsf(value[2] - 0.9f) < 1e-6f);
			return;
		}
		assert(value[0] == 123 && value[1] == 456 && value[2] == 789);
	}
	assert(!"driver failed to recover after a transient transfer failure");
}

static void test_transfer_failures(void)
{
	initialize(false);
	unsigned init_transactions = hw.transactions;
	for (unsigned fail = 1; fail <= init_transactions; fail++) {
		reset_model(false);
		hw.fail_at = fail;
		float actual = -123;
		int result = mag->init(1.0f / 200, &actual);
		assert(hw.failures == 1);
		if (result < 0)
			assert(actual == -123);
		hw.fail_at = 0;
		assert(mag->init(1.0f / 200, &actual) == 0);
		recover_to_sample();
	}

	running(false);
	unsigned begin = hw.transactions;
	float actual;
	assert(mag->update_odr(1.0f / 50, &actual) >= 0);
	unsigned odr_transactions = hw.transactions - begin;
	for (unsigned fail = 1; fail <= odr_transactions; fail++) {
		running(false);
		hw.fail_at = hw.transactions + fail;
		actual = -123;
		int result = mag->update_odr(1.0f / 50, &actual);
		assert(hw.failures == 1);
		if (result < 0)
			assert(actual == -123);
		hw.fail_at = 0;
		expect_period(1.0f / 50, 50, 1);
		recover_to_sample();
	}

	initialize(false);
	frame(400, -800, 1200, -160);
	begin = hw.transactions;
	no_sample();
	unsigned recovery_transactions = hw.transactions - begin;
	for (unsigned fail = 1; fail <= recovery_transactions; fail++) {
		initialize(false);
		frame(400, -800, 1200, -160);
		hw.fail_at = hw.transactions + fail;
		no_sample();
		assert(hw.failures == 1);
		hw.fail_at = 0;
		recover_to_sample();
		assert(hw.mrms > 0);
	}
}

int main(void)
{
	test_startup_and_freshness();
	test_overrange_and_recovery();
	test_odr_and_single();
	test_single_survives_speculative_read();
	test_prefetch_disable_failure();
	test_transfer_failures();
	puts("ICT153xx behavior regressions passed");
	return 0;
}
