#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sensor/imu/LSM6DSV.h"

/* Link the complete production driver. DS13476 defines NACK bit 3, ENDOP
 * bit 0 and WR_ONCE_DONE bit 7. Timing/electrical behavior is not simulated.
 * Status clearing is NOT documented by the sources used for this audit:
 * sticky and read-clear scenarios below are robustness probes, not claims
 * about the silicon. Reset normally clears status/output in this model;
 * an uncleared-status scenario also checks the driver's runtime guard. */
enum { ENDOP = 0x01, NACK = 0x08, WRITE_DONE = 0x80, MASTER_ON = 0x04 };
enum operation { READ, WRITE };
static struct {
	uint8_t main[256], hub[256], external[256], status;
	bool bank, master, stalled, no_xlda, clear_status, wrote, reset_stuck;
	bool late_nack;
	uint8_t error;
	uint64_t now, next_op;
	uint32_t latency;
	unsigned starts, writes, data_reads, status_reads;
	const sensor_ext_ssi_t *ext;
	struct {
		bool armed, bank, any_bank, applied_write;
		enum operation op;
		uint8_t reg;
		int value, error;
		unsigned skip;
	} fault;
} hw;

static void advance(uint64_t us)
{
	hw.now += us;
	if (!hw.master || hw.stalled || hw.no_xlda || hw.now < hw.next_op)
		return;
	bool reading = hw.hub[LSM6DSV_SLV0_ADD] & 1;
	if (!reading && hw.wrote && (hw.hub[LSM6DSV_MASTER_CONFIG] & 0x40))
		return; /* WRITE_ONCE constrains writes, not reads. */
	hw.status = ENDOP | hw.error | (reading ? 0 : WRITE_DONE);
	if (!hw.error) {
		uint8_t sub = hw.hub[LSM6DSV_SLV0_SUBADD];
		if (reading) {
			unsigned len = hw.hub[LSM6DSV_SLV0_CONFIG] & 7;
			for (unsigned i = 0; i < len; i++)
				hw.hub[LSM6DSV_SENSOR_HUB_1 + i] = hw.external[(uint8_t)(sub + i)];
		} else {
			hw.external[sub] = hw.hub[LSM6DSV_DATAWRITE_SLV0];
			hw.writes++;
		}
	}
	hw.wrote = !reading;
	hw.next_op = hw.now + 4200;
}
int64_t host_k_uptime_get(void) { return hw.now / 1000; }
void host_k_busy_wait(uint32_t us) { advance(us); }
void host_k_msleep(int32_t ms) { assert(ms >= 0); advance((uint64_t)ms * 1000); }
void host_k_usleep(int32_t us) { assert(us >= 0); advance(us); }

static int transfer(enum operation op, uint8_t reg, int value)
{
	advance(hw.latency);
	if (hw.fault.armed && hw.fault.op == op && hw.fault.reg == reg &&
		(hw.fault.any_bank || hw.fault.bank == hw.bank) &&
		(hw.fault.value < 0 || hw.fault.value == value)) {
		if (hw.fault.skip) {
			hw.fault.skip--;
		} else {
			hw.fault.armed = false;
			return hw.fault.error;
		}
	}
	return 0;
}

int ssi_burst_write(enum sensor_interface_dev dev, uint8_t reg, const uint8_t *buf, uint32_t len)
{
	assert(dev == SENSOR_INTERFACE_DEV_IMU && len && (unsigned)reg + len <= 256);
	int err = transfer(WRITE, reg, buf[0]);
	if (err && !hw.fault.applied_write)
		return err;
	if (reg == LSM6DSV_FUNC_CFG_ACCESS) {
		assert(len == 1);
		hw.bank = (buf[0] & 0x40) != 0;
		return err;
	}
	/* SENSOR_HUB output registers are read-only, including CTRL3's alias. */
	assert(!hw.bank || reg < LSM6DSV_SENSOR_HUB_1 || reg >= LSM6DSV_MASTER_CONFIG);
	memcpy((hw.bank ? hw.hub : hw.main) + reg, buf, len);
	if (hw.bank && reg == LSM6DSV_MASTER_CONFIG) {
		if (buf[0] & 0x80) {
			if (!hw.reset_stuck)
				hw.status = 0;
			memset(hw.hub + LSM6DSV_SENSOR_HUB_1, 0, 18);
		}
		/* Error arriving after terminal polling but before the stop is
		 * complete must not be hidden by an earlier successful snapshot. */
		if (hw.master && !(buf[0] & MASTER_ON) && hw.late_nack)
			hw.status |= NACK;
		hw.master = (buf[0] & MASTER_ON) != 0;
		if (hw.master) {
			hw.starts++;
			hw.wrote = false;
			hw.next_op = hw.now + 4200;
		}
	} else if (!hw.bank && reg == LSM6DSV_CTRL3 && (buf[0] & 1)) {
		memset(hw.main, 0, sizeof(hw.main));
		memset(hw.hub, 0, sizeof(hw.hub));
		hw.main[LSM6DSV_WHO_AM_I] = 0x70;
		hw.master = false;
		hw.status = 0;
	}
	if (err)
		advance(5000); /* A committed transfer can report failure after preemption. */
	return err;
}
int ssi_reg_write_byte(enum sensor_interface_dev dev, uint8_t reg, uint8_t value)
{
	return ssi_burst_write(dev, reg, &value, 1);
}
int ssi_burst_read(enum sensor_interface_dev dev, uint8_t reg, uint8_t *buf, uint32_t len)
{
	assert(dev == SENSOR_INTERFACE_DEV_IMU && len && (unsigned)reg + len <= 256);
	int err = transfer(READ, reg, -1);
	if (err)
		return err;
	if ((hw.bank && reg == LSM6DSV_STATUS_MASTER) ||
		(!hw.bank && reg == LSM6DSV_STATUS_MASTER_MAINPAGE)) {
		assert(len == 1);
		hw.status_reads++;
		*buf = hw.status;
		if (hw.clear_status)
			hw.status = 0;
	} else if (!hw.bank && reg == LSM6DSV_STATUS_REG) {
		assert(len == 1);
		*buf = hw.no_xlda ? 0 : 1;
	} else {
		if (hw.bank && reg == LSM6DSV_SENSOR_HUB_1)
			hw.data_reads++;
		memcpy(buf, (hw.bank ? hw.hub : hw.main) + reg, len);
	}
	return 0;
}
int ssi_reg_read_byte(enum sensor_interface_dev dev, uint8_t reg, uint8_t *out)
{
	return ssi_burst_read(dev, reg, out, 1);
}
int ssi_reg_update_byte(enum sensor_interface_dev dev, uint8_t reg, uint8_t mask, uint8_t value)
{
	uint8_t old;
	int err = ssi_reg_read_byte(dev, reg, &old);
	return err ? err : ssi_reg_write_byte(dev, reg, (old & ~mask) | (value & mask));
}
int sensor_interface_spi_configure(enum sensor_interface_dev dev, uint32_t frequency, uint32_t dummy)
{
	assert(dev == SENSOR_INTERFACE_DEV_IMU);
	(void)frequency;
	(void)dummy;
	return 0;
}
void sensor_interface_ext_configure(const sensor_ext_ssi_t *ext) { hw.ext = ext; }
const sensor_ext_ssi_t *sensor_interface_ext_get(void) { return hw.ext; }

static void initialize(void)
{
	float accel, gyro;
	assert(lsm_init(0, 0.005f, 0.005f, &accel, &gyro) == 0);
}
static void reset_model(bool prefetch, bool clear_status)
{
	memset(&hw, 0, sizeof(hw));
	hw.latency = 100;
	hw.clear_status = clear_status;
	hw.main[LSM6DSV_WHO_AM_I] = 0x70;
	assert(lsm_ext_setup(SENSOR_EXT_MODE_I2CM_PROXY) == 0);
	assert(hw.ext->ext_set_prefetch(prefetch) == 0);
	initialize();
	for (unsigned i = 0; i < 7; i++)
		hw.external[0x08 + i] = 0x50 + i;
}
static int sample(uint8_t *out)
{
	uint8_t sub = 0x08;
	return hw.ext->ext_write_read(0x1e, &sub, 1, out, 7);
}
static void expect_sample(uint8_t *out)
{
	assert(sample(out) == 0);
	assert(memcmp(out, hw.external + 0x08, 7) == 0);
}
static void fail_on(enum operation op, bool bank, uint8_t reg, int value, unsigned skip)
{
	hw.fault.armed = true;
	hw.fault.bank = bank;
	hw.fault.applied_write = false;
	hw.fault.any_bank = reg == LSM6DSV_FUNC_CFG_ACCESS;
	hw.fault.op = op;
	hw.fault.reg = reg;
	hw.fault.value = value;
	hw.fault.skip = skip;
	hw.fault.error = -ENXIO;
}

static void test_nack(bool clear_status)
{
	uint8_t out[7], command[] = {0x20, 0x73};
	reset_model(false, clear_status);
	hw.error = NACK;
	assert(sample(out) == -EIO);
	assert(hw.data_reads == 0 && !hw.master && !hw.bank);
	assert(hw.ext->ext_write(0x1e, command, sizeof(command)) == -EIO);
	assert(hw.writes == 0 && !hw.master && !hw.bank);
}

static void test_configuration_failfast(void)
{
	uint8_t out[7], command[] = {0x20, 0x73};
	for (unsigned writing = 0; writing < 2; writing++) {
		reset_model(false, false);
		uint8_t controls[3];
		memcpy(controls, hw.main + LSM6DSV_CTRL6, sizeof(controls));
		/* First select belongs to stop, second to profile configuration. */
		fail_on(WRITE, false, LSM6DSV_FUNC_CFG_ACCESS, 0x40, 1);
		int err = writing ? hw.ext->ext_write(0x1e, command, 2) : sample(out);
		assert(err == -ENXIO && !hw.fault.armed);
		assert(memcmp(controls, hw.main + LSM6DSV_CTRL6, sizeof(controls)) == 0);
		assert(hw.starts == 0 && !hw.master && !hw.bank);

		reset_model(false, false);
		hw.hub[LSM6DSV_SLV0_ADD] = 0x39; /* Old slave must never start. */
		fail_on(WRITE, true, LSM6DSV_SLV0_ADD, -1, 0);
		err = writing ? hw.ext->ext_write(0x1e, command, 2) : sample(out);
		assert(err == -ENXIO && !hw.fault.armed);
		assert(hw.starts == 0 && !hw.master && !hw.bank);
	}
}

static void test_stop_restore_invalidates(void)
{
	uint8_t out[7];
	reset_model(true, false);
	expect_sample(out);
	fail_on(WRITE, true, LSM6DSV_FUNC_CFG_ACCESS, 0x00, 0);
	assert(hw.ext->ext_set_prefetch(false) == -ENXIO);
	assert(!hw.master);
	memset(hw.external + 0x08, 0x91, 7);
	expect_sample(out); /* Stopped hub's previous bytes cannot be a success. */
}

static void test_uncleared_epoch_refused(void)
{
	uint8_t out[7];
	reset_model(false, false);
	hw.status = ENDOP;
	hw.reset_stuck = true;
	assert(sample(out) < 0);
	assert(hw.starts == 0 && hw.data_reads == 0 && !hw.master && !hw.bank);
}

static void test_continuous_error_and_freshness(bool clear_status)
{
	uint8_t out[7];
	reset_model(true, clear_status);
	expect_sample(out);
	hw.error = NACK;
	advance(5000);
	unsigned reads = hw.data_reads;
	assert(sample(out) == -EIO);
	assert(hw.data_reads == reads && !hw.master && !hw.bank);

	reset_model(true, clear_status);
	expect_sample(out);
	hw.status = 0;
	hw.stalled = true;
	reads = hw.data_reads;
	assert(sample(out) == -ETIMEDOUT);
	assert(hw.data_reads == reads && !hw.master && !hw.bank);
}

static void test_error_during_stop(void)
{
	uint8_t out[7];
	reset_model(true, true);
	expect_sample(out);
	advance(5000);
	hw.late_nack = true;
	unsigned reads = hw.data_reads;
	assert(sample(out) == -EIO);
	assert(hw.data_reads == reads && !hw.master && !hw.bank);
}

static void test_pending_status_io_failure(void)
{
	uint8_t out[7];
	reset_model(true, true);
	expect_sample(out);
	fail_on(READ, true, LSM6DSV_STATUS_MASTER, -1, 0);
	assert(sample(out) == -ENXIO && !hw.fault.armed);
	assert(!hw.master && !hw.bank);
	memset(hw.external + 0x08, 0x24, 7);
	expect_sample(out);
}

static void test_failed_read_invalidates(void)
{
	uint8_t out[7];
	reset_model(true, false);
	expect_sample(out);
	advance(5000);
	fail_on(READ, true, LSM6DSV_SENSOR_HUB_1, -1, 0);
	assert(sample(out) == -ENXIO);
	assert(!hw.master && !hw.bank);
	memset(hw.external + 0x08, 0x37, 7);
	expect_sample(out);
}

static void test_deadlines_and_status_io(void)
{
	uint8_t out[7], command[] = {0x20, 0x73};
	for (unsigned writing = 0; writing < 2; writing++) {
		for (unsigned xlda = 0; xlda < 2; xlda++) {
			reset_model(false, false);
			hw.latency = 700; /* Deadlines must include host IO and scheduling. */
			hw.no_xlda = xlda;
			hw.stalled = true;
			uint64_t start = hw.now;
			int err = writing ? hw.ext->ext_write(0x1e, command, 2) : sample(out);
			assert(err == -ETIMEDOUT);
			assert(hw.now - start < 110000);
			assert(!hw.master && !hw.bank && hw.data_reads == 0);
		}
		reset_model(false, false);
		fail_on(READ, false, LSM6DSV_STATUS_REG, -1, 0);
		int err = writing ? hw.ext->ext_write(0x1e, command, 2) : sample(out);
		assert(err == -ENXIO && !hw.fault.armed);
		assert(!hw.master && !hw.bank && hw.data_reads == 0);
	}
}

static void test_prefetch_policy_lifecycle(void)
{
	uint8_t out[7], command[] = {0x20, 0x73};
	reset_model(false, false);
	for (unsigned stage = 0; stage < 3; stage++) {
		if (stage == 1) {
			lsm_shutdown();
			initialize();
		} else if (stage == 2) {
			assert(lsm_setup_WOM() != 0xff);
			initialize();
		}
		expect_sample(out);
		assert(!hw.master);
		assert(hw.ext->ext_write(0x1e, command, sizeof(command)) == 0);
		assert(!hw.master);
		unsigned writes = hw.writes;
		advance(30000);
		assert(hw.writes == writes); /* No replay/background master after return. */
	}
}

static void test_ambiguous_write_is_not_replayed(void)
{
	uint8_t out[7], command[] = {0x20, 0x73};
	reset_model(false, false);
	fail_on(WRITE, true, LSM6DSV_MASTER_CONFIG, MASTER_ON | 0x40, 0);
	hw.fault.applied_write = true;
	assert(hw.ext->ext_write(0x1e, command, sizeof(command)) == -ENXIO);
	assert(hw.writes == 1 && hw.external[0x20] == 0x73);
	assert(!hw.master && !hw.bank);
	expect_sample(out);
	advance(30000);
	assert(hw.writes == 1);
}

static void test_shutdown_after_stop_failure(void)
{
	uint8_t out[7];
	for (unsigned restore_failure = 0; restore_failure < 2; restore_failure++) {
		reset_model(true, false);
		expect_sample(out);
		fail_on(WRITE, true,
			restore_failure ? LSM6DSV_FUNC_CFG_ACCESS : LSM6DSV_MASTER_CONFIG, 0, 0);
		lsm_shutdown();
		assert(!hw.fault.armed && !hw.master && !hw.bank);
		initialize();
		memset(hw.external + 0x08, 0x62, 7);
		expect_sample(out);
	}
}

int main(void)
{
	test_nack(false);
	test_nack(true);
	test_configuration_failfast();
	test_stop_restore_invalidates();
	test_uncleared_epoch_refused();
	test_continuous_error_and_freshness(false);
	test_continuous_error_and_freshness(true);
	test_error_during_stop();
	test_pending_status_io_failure();
	test_failed_read_invalidates();
	test_deadlines_and_status_io();
	test_prefetch_policy_lifecycle();
	test_ambiguous_write_is_not_replayed();
	test_shutdown_after_stop_failure();
	puts("LSM6DSV full-driver proxy regressions passed");
	return 0;
}
