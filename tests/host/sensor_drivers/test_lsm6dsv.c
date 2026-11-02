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
enum { FIFO_PACKET_BYTES = 7, FIFO_QUEUE_CAPACITY = 512 };
enum operation { READ, WRITE };
static struct {
	uint8_t registers[256], hub[256], external[256], status;
	bool bank, master, stalled, no_trigger, clear_status, wrote, reset_stuck;
	bool late_nack;
	bool dummy_prefix, partial_read_error, partial_status_error;
	unsigned slot_reads; /* Slots of the most recent read epoch. */
	uint8_t slot_sub[4], slot_len[4], last_master;
	unsigned sleep_count;
	uint32_t sleeps_us[1024];
	uint8_t error;
	uint64_t now, next_op;
	uint32_t latency, status_delay_us;
	unsigned starts, writes, data_reads, status_reads;
	unsigned stops;
	struct {
		uint8_t records[FIFO_QUEUE_CAPACITY][FIFO_PACKET_BYTES];
		unsigned head, count, pops, partial_pops, overrun_after;
		unsigned fail_status_after_pops;
		bool fail_status;
		uint8_t flags;
	} fifo;
	unsigned transactions, fail_transaction;
	const sensor_ext_ssi_t *ext;
	struct {
		bool armed, bank, any_bank, applied_write;
		enum operation op;
		uint8_t reg;
		int value, error;
		unsigned skip;
	} fault;
} hw;
static unsigned gyro_discontinuities;
void sensor_gyro_report_discontinuity(void) { gyro_discontinuities++; }

static void fifo_push(uint8_t tag, uint8_t count, const uint8_t payload[6])
{
	assert(hw.fifo.count < FIFO_QUEUE_CAPACITY - 1);
	unsigned tail = (hw.fifo.head + hw.fifo.count++) % FIFO_QUEUE_CAPACITY;
	hw.fifo.records[tail][0] = (uint8_t)((tag << 3) | ((count & 3) << 1));
	memcpy(&hw.fifo.records[tail][1], payload, 6);
}

static void fifo_pop(uint8_t out[FIFO_PACKET_BYTES])
{
	assert(hw.fifo.count);
	memcpy(out, hw.fifo.records[hw.fifo.head], FIFO_PACKET_BYTES);
	hw.fifo.head = (hw.fifo.head + 1) % FIFO_QUEUE_CAPACITY;
	hw.fifo.count--;
	hw.fifo.pops++;
	if (hw.fifo.overrun_after && --hw.fifo.overrun_after == 0)
		hw.fifo.flags |= LSM6DSV_FIFO_OVR_LATCHED;
}

/* Emit one physical FIFO record from the currently programmed slave slot. */
static void fifo_emit_slot(unsigned slot, uint8_t count)
{
	assert(hw.master && slot < ((hw.hub[LSM6DSV_MASTER_CONFIG] & 3) + 1u));
	unsigned base = LSM6DSV_SLV0_ADD + 3 * slot;
	uint8_t config = hw.hub[base + 2], sub = hw.hub[base + 1];
	unsigned len = config & 7;
	assert((hw.hub[base] & 1) && (config & LSM6DSV_SHUB_BATCH_EXT_SENS));
	assert(len && len <= 6);
	uint8_t payload[6] = {0};
	for (unsigned i = 0; i < len; i++) {
		payload[i] = hw.dummy_prefix
			? (i < 2 ? 0xee : hw.external[(uint8_t)(sub + i - 2)])
			: hw.external[(uint8_t)(sub + i)];
	}
	fifo_push((uint8_t)(LSM6DSV_TAG_SENSORHUB_SLAVE0 + slot), count, payload);
}

static void fifo_emit_frame(uint8_t count)
{
	unsigned slots = (hw.hub[LSM6DSV_MASTER_CONFIG] & 3) + 1;
	for (unsigned slot = 0; slot < slots; slot++)
		fifo_emit_slot(slot, (uint8_t)(count + slot));
}

static void fifo_emit_nack(unsigned slot, uint8_t count)
{
	uint8_t payload[6] = {(uint8_t)(0xfc | slot)};
	assert(slot < 4);
	fifo_push(LSM6DSV_TAG_SENSORHUB_NACK, count, payload);
}

static void fifo_emit_inertial(uint8_t tag, uint8_t count, uint8_t seed)
{
	uint8_t payload[6];
	assert(tag == LSM6DSV_TAG_ACCEL_NC || tag == LSM6DSV_TAG_GYRO_NC);
	for (unsigned i = 0; i < sizeof(payload); i++)
		payload[i] = (uint8_t)(seed + i);
	fifo_push(tag, count, payload);
}

static void advance(uint64_t us)
{
	hw.now += us;
	if (!hw.master || hw.stalled || hw.no_trigger || hw.now < hw.next_op)
		return;
	bool reading = hw.hub[LSM6DSV_SLV0_ADD] & 1;
	if (!reading && hw.wrote && (hw.hub[LSM6DSV_MASTER_CONFIG] & 0x40))
		return; /* WRITE_ONCE constrains writes, not reads. */
	hw.status = ENDOP | hw.error | (reading ? 0 : WRITE_DONE);
	if (!hw.error) {
		uint8_t sub = hw.hub[LSM6DSV_SLV0_SUBADD];
		if (reading) {
			unsigned offset = 0;
			unsigned slots = (hw.hub[LSM6DSV_MASTER_CONFIG] & 3) + 1;
			hw.slot_reads = 0;
			for (unsigned slot = 0; slot < slots; slot++) {
				unsigned base = LSM6DSV_SLV0_ADD + 3 * slot;
				unsigned len = hw.hub[base + 2] & 7;
				sub = hw.hub[base + 1];
				assert(len && offset + len <= 18);
				hw.slot_sub[hw.slot_reads] = sub;
				hw.slot_len[hw.slot_reads++] = len;
				for (unsigned i = 0; i < len; i++) {
					uint8_t value = hw.dummy_prefix
						? (i < 2 ? 0xee : hw.external[(uint8_t)(sub + i - 2)])
						: hw.external[(uint8_t)(sub + i)];
					hw.hub[LSM6DSV_SENSOR_HUB_1 + offset++] = value;
				}
			}
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
void host_k_usleep(int32_t us)
{
	assert(us >= 0);
	if (hw.sleep_count < ARRAY_SIZE(hw.sleeps_us))
		hw.sleeps_us[hw.sleep_count++] = us;
	advance(us);
}

static int transfer(enum operation op, uint8_t reg, int value)
{
	if (op == READ && hw.bank && hw.master && reg == LSM6DSV_STATUS_MASTER && hw.status_delay_us) {
		uint32_t delay = hw.status_delay_us;
		hw.status_delay_us = 0;
		advance(delay); /* Model a poll whose bus return is delayed by preemption. */
	}
	advance(hw.latency);
	if (++hw.transactions == hw.fail_transaction)
		return -ENXIO;
	if (op == READ && !hw.bank && hw.fifo.fail_status &&
		(reg == LSM6DSV_FIFO_STATUS1 || reg == LSM6DSV_FIFO_STATUS2) &&
		hw.fifo.pops >= hw.fifo.fail_status_after_pops) {
		hw.fifo.fail_status = false;
		return -ENXIO;
	}
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
	memcpy((hw.bank ? hw.hub : hw.registers) + reg, buf, len);
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
		bool next_master = (buf[0] & MASTER_ON) != 0;
		if (hw.master && !next_master)
			hw.stops++;
		if (!hw.master && next_master) {
			hw.starts++;
			hw.wrote = false;
			hw.next_op = hw.now + 4200;
		}
		hw.master = next_master;
		if (hw.master)
			hw.last_master = buf[0];
	} else if (!hw.bank && reg == LSM6DSV_CTRL3 && (buf[0] & 1)) {
		memset(hw.registers, 0, sizeof(hw.registers));
		memset(hw.hub, 0, sizeof(hw.hub));
		hw.registers[LSM6DSV_WHO_AM_I] = 0x70;
		hw.master = false;
		hw.status = 0;
		hw.fifo.head = hw.fifo.count = 0;
		hw.fifo.flags = 0;
	} else if (!hw.bank && reg == LSM6DSV_FIFO_CTRL4 && (buf[0] & 7) == 0) {
		/* Bypass flushes queued records; the pop counter remains diagnostic. */
		hw.fifo.head = hw.fifo.count = 0;
		hw.fifo.flags = 0;
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
	assert(dev == SENSOR_INTERFACE_DEV_IMU && len &&
		((!hw.bank && reg == LSM6DSV_FIFO_DATA_OUT_TAG) || (unsigned)reg + len <= 256));
	int err = transfer(READ, reg, -1);
	if (err) {
		if (hw.partial_read_error && hw.bank && reg == LSM6DSV_SENSOR_HUB_1)
			buf[0] = hw.hub[reg];
		if (hw.partial_status_error && !hw.bank &&
			(reg == LSM6DSV_FIFO_STATUS1 || reg == LSM6DSV_FIFO_STATUS2))
			hw.fifo.flags &= (uint8_t)~LSM6DSV_FIFO_OVR_LATCHED;
		if (!hw.bank && reg == LSM6DSV_FIFO_DATA_OUT_TAG && hw.fifo.partial_pops) {
			uint8_t record[FIFO_PACKET_BYTES];
			unsigned n = hw.fifo.partial_pops;
			hw.fifo.partial_pops = 0;
			for (unsigned i = 0; i < n && (i + 1) * FIFO_PACKET_BYTES <= len; i++)
				fifo_pop(buf + i * FIFO_PACKET_BYTES);
			if (n * FIFO_PACKET_BYTES < len && hw.fifo.count) {
				fifo_pop(record);
				buf[n * FIFO_PACKET_BYTES] = record[0];
			}
		}
		return err;
	}
	if (!hw.bank && reg == LSM6DSV_FIFO_DATA_OUT_TAG) {
		assert(len % FIFO_PACKET_BYTES == 0 && len / FIFO_PACKET_BYTES <= hw.fifo.count);
		for (unsigned i = 0; i < len; i += FIFO_PACKET_BYTES)
			fifo_pop(buf + i);
	} else if (!hw.bank && reg == LSM6DSV_FIFO_STATUS1) {
		assert(len == 1 || len == 2);
		buf[0] = (uint8_t)hw.fifo.count;
		if (len == 2) {
			buf[1] = hw.fifo.flags | ((hw.fifo.count >> 8) & LSM6DSV_FIFO_DIFF_8);
			hw.fifo.flags &= (uint8_t)~LSM6DSV_FIFO_OVR_LATCHED;
		}
	} else if (!hw.bank && reg == LSM6DSV_FIFO_STATUS2) {
		assert(len == 1);
		buf[0] = hw.fifo.flags | ((hw.fifo.count >> 8) & LSM6DSV_FIFO_DIFF_8);
		hw.fifo.flags &= (uint8_t)~LSM6DSV_FIFO_OVR_LATCHED;
	} else if ((hw.bank && reg == LSM6DSV_STATUS_MASTER) ||
		(!hw.bank && reg == LSM6DSV_STATUS_MASTER_MAINPAGE)) {
		assert(len == 1);
		hw.status_reads++;
		*buf = hw.status;
		if (hw.clear_status)
			hw.status = 0;
	} else {
		if (hw.bank && reg == LSM6DSV_SENSOR_HUB_1)
			hw.data_reads++;
		memcpy(buf, (hw.bank ? hw.hub : hw.registers) + reg, len);
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
	hw.registers[LSM6DSV_WHO_AM_I] = 0x70;
	assert(lsm_ext_setup(SENSOR_EXT_MODE_I2CM_PROXY) == 0);
	assert(hw.ext->ext_set_prefetch(prefetch) == 0);
	initialize();
	gyro_discontinuities = 0;
	for (unsigned i = 0; i < 18; i++)
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
static int read_frame(uint8_t reg, uint8_t dummy, uint8_t word, uint8_t *out, uint8_t len)
{
	return hw.ext->ext_read_frame(0x14, reg, dummy, word, out, len);
}
static bool frame_ready_no_io(void)
{
	unsigned transactions = hw.transactions, starts = hw.starts, stops = hw.stops;
	unsigned pops = hw.fifo.pops, count = hw.fifo.count, sleeps = hw.sleep_count;
	uint64_t now = hw.now;
	assert(hw.ext && hw.ext->ext_frame_ready);
	bool ready = hw.ext->ext_frame_ready();
	assert(hw.transactions == transactions && hw.starts == starts && hw.stops == stops);
	assert(hw.fifo.pops == pops && hw.fifo.count == count && hw.sleep_count == sleeps && hw.now == now);
	return ready;
}
static void expect_fill(const uint8_t *buf, size_t len, uint8_t value)
{
	for (size_t i = 0; i < len; i++)
		assert(buf[i] == value);
}
static void arm_frame(uint8_t reg, uint8_t dummy, uint8_t word, uint8_t *out, uint8_t len)
{
	memset(out, 0xa5, len);
	assert(read_frame(reg, dummy, word, out, len) == -EAGAIN);
	expect_fill(out, len, 0xa5);
	assert(hw.master && !hw.bank);
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
		memcpy(controls, hw.registers + LSM6DSV_CTRL6, sizeof(controls));
		/* First select belongs to stop, second to profile configuration. */
		fail_on(WRITE, false, LSM6DSV_FUNC_CFG_ACCESS, 0x40, 1);
		int err = writing ? hw.ext->ext_write(0x1e, command, 2) : sample(out);
		assert(err == -ENXIO && !hw.fault.armed);
		assert(memcmp(controls, hw.registers + LSM6DSV_CTRL6, sizeof(controls)) == 0);
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
	arm_frame(0x08, 0, 1, out, sizeof(out));
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

static void test_checked_epoch_error_and_freshness(bool clear_status)
{
	uint8_t out[7];
	reset_model(true, clear_status);
	expect_sample(out);
	assert(!hw.master);
	hw.error = NACK;
	unsigned reads = hw.data_reads;
	memset(out, 0xa5, sizeof(out));
	assert(sample(out) == -EIO);
	expect_fill(out, sizeof(out), 0xa5);
	assert(hw.data_reads == reads && !hw.master && !hw.bank);

	reset_model(true, clear_status);
	expect_sample(out);
	hw.stalled = true;
	reads = hw.data_reads;
	assert(sample(out) == -ETIMEDOUT);
	assert(hw.data_reads == reads && !hw.master && !hw.bank);
}

static void test_error_during_stop(void)
{
	uint8_t out[7];
	reset_model(false, true);
	hw.late_nack = true;
	assert(sample(out) == -EIO);
	assert(hw.data_reads == 0 && !hw.master && !hw.bank);
}

static void test_pending_status_io_failure(void)
{
	uint8_t out[7];
	for (unsigned baseline = 0; baseline < 2; baseline++) {
		reset_model(false, true);
		/* Fail either the stale-status clear or the checked baseline. */
		fail_on(READ, true, LSM6DSV_STATUS_MASTER, -1, baseline);
		memset(out, 0xa5, sizeof(out));
		assert(sample(out) == -ENXIO && !hw.fault.armed);
		expect_fill(out, sizeof(out), 0xa5);
		assert(!hw.master && !hw.bank);
		memset(hw.external + 0x08, 0x24, 7);
		expect_sample(out);
	}
}

static void test_failed_read_invalidates(void)
{
	uint8_t out[7];
	reset_model(true, false);
	hw.partial_read_error = true;
	memset(out, 0xa5, sizeof(out));
	fail_on(READ, true, LSM6DSV_SENSOR_HUB_1, -1, 0);
	assert(sample(out) == -ENXIO);
	expect_fill(out, sizeof(out), 0xa5);
	assert(!hw.master && !hw.bank);
	memset(hw.external + 0x08, 0x37, 7);
	expect_sample(out);
}

static void test_deadlines_and_status_io(void)
{
	uint8_t out[7], command[] = {0x20, 0x73};
	static const uint32_t latencies[] = {100, 700, 2000};
	for (unsigned writing = 0; writing < 2; writing++) {
		for (unsigned latency = 0; latency < ARRAY_SIZE(latencies); latency++) {
			reset_model(false, false);
			hw.latency = latencies[latency]; /* API duration also includes cleanup. */
			hw.no_trigger = latency;
			hw.stalled = !latency;
			hw.status_reads = 0;
			uint64_t start = hw.now;
			int err = writing ? hw.ext->ext_write(0x1e, command, 2) : sample(out);
			assert(err == -ETIMEDOUT);
			assert(hw.now - start < 110000);
			/* Polls yield the CPU between status reads. */
			assert(hw.status_reads <= 100000 / (250 + hw.latency) + 8);
			assert(!hw.master && !hw.bank && hw.data_reads == 0);
		}
		reset_model(false, false);
		/* Skip the stale-status clear and baseline; fail the completion poll. */
		fail_on(READ, true, LSM6DSV_STATUS_MASTER, -1, 2);
		int err = writing ? hw.ext->ext_write(0x1e, command, 2) : sample(out);
		assert(err == -ENXIO && !hw.fault.armed);
		assert(!hw.master && !hw.bank && hw.data_reads == 0);
	}
	/* A poll begun before the deadline may return after it. Preserve a
	 * completed operation, but never hide a NACK behind terminal DONE. */
	for (unsigned writing = 0; writing < 2; writing++) {
		for (unsigned nack = 0; nack < 2; nack++) {
			reset_model(false, true);
			hw.status_delay_us = 110000;
			hw.error = nack ? NACK : 0;
			memset(out, 0xa5, sizeof(out));
			uint64_t start = hw.now;
			int err = writing ? hw.ext->ext_write(0x1e, command, 2) : sample(out);
			assert(hw.now - start > 100000 && hw.status_delay_us == 0);
			assert(err == (nack ? -EIO : 0));
			assert(!hw.master && !hw.bank);
			if (writing) {
				assert(hw.writes == (nack ? 0u : 1u));
				if (!nack)
					assert(hw.external[command[0]] == command[1]);
			} else if (nack) {
				expect_fill(out, sizeof(out), 0xa5);
			} else {
				assert(memcmp(out, hw.external + 0x08, sizeof(out)) == 0);
			}
		}
	}
}

static void test_frame_ready_lifecycle(void)
{
	uint8_t out[9], batch[2 * FIFO_PACKET_BYTES];
	reset_model(true, false);
	assert(!frame_ready_no_io());
	arm_frame(0x08, 0, 1, out, sizeof(out));
	assert(!frame_ready_no_io());
	/* An incomplete FIFO group is not a consumer-visible frame. */
	fifo_emit_slot(0, 0);
	assert(lsm_fifo_read(batch, sizeof(batch)) == 0);
	assert(!frame_ready_no_io());
	fifo_emit_slot(1, 1);
	assert(lsm_fifo_read(batch, sizeof(batch)) == 0);
	assert(frame_ready_no_io() && frame_ready_no_io());
	assert(read_frame(0x08, 0, 1, out, sizeof(out)) == 0);
	assert(memcmp(out, hw.external + 0x08, sizeof(out)) == 0);
	assert(!frame_ready_no_io());
	assert(read_frame(0x08, 0, 1, out, sizeof(out)) == -EAGAIN);

	/* A command invalidates an unread frame and status polling must not make
	 * the stopped stream appear ready or restart it. */
	fifo_emit_frame(2);
	assert(lsm_fifo_read(batch, sizeof(batch)) == 0 && frame_ready_no_io());
	uint8_t command[] = {0x1b, 0x01};
	assert(hw.ext->ext_write(0x14, command, sizeof(command)) == 0);
	assert(!hw.master && !frame_ready_no_io());
	uint8_t status_reg = 0x18, status;
	assert(hw.ext->ext_write_read(0x14, &status_reg, 1, &status, 1) == 0);
	assert(!hw.master && !frame_ready_no_io());

	arm_frame(0x08, 0, 1, out, sizeof(out));
	assert(!frame_ready_no_io());
	fifo_emit_frame(3);
	assert(lsm_fifo_read(batch, sizeof(batch)) == 0 && frame_ready_no_io());
	assert(hw.ext->ext_set_prefetch(false) == 0);
	assert(!frame_ready_no_io());
	assert(read_frame(0x08, 0, 1, out, sizeof(out)) == 0);
	assert(!frame_ready_no_io());

	assert(hw.ext->ext_set_prefetch(true) == 0);
	arm_frame(0x08, 0, 1, out, sizeof(out));
	fifo_emit_frame(0);
	assert(lsm_fifo_read(batch, sizeof(batch)) == 0 && frame_ready_no_io());
	assert(lsm_ext_setup(SENSOR_EXT_MODE_I2CM_PROXY) == 0);
	assert(!frame_ready_no_io());
	assert(read_frame(0x08, 0, 1, out, sizeof(out)) == 0);
	assert(!frame_ready_no_io());
}

static void test_frame_layout_and_fifo_epochs(void)
{
	uint8_t out[9], expected[9], invalid[15], batch[7 * FIFO_PACKET_BYTES];
	float accel[3] = {0}, gyro[3] = {0};
	const uint8_t gyro_record[7] = {(LSM6DSV_TAG_GYRO_NC << 3) | (1 << 1),
		0x21, 0x22, 0x23, 0x24, 0x25, 0x26};
	const uint8_t accel_record[7] = {(LSM6DSV_TAG_ACCEL_NC << 3) | (3 << 1),
		0x41, 0x42, 0x43, 0x44, 0x45, 0x46};
	const uint8_t timestamp[6] = {0x91, 0x92, 0x93, 0x94, 0x95, 0x96};
	const uint8_t timestamp_record[7] = {(LSM6DSV_TAG_TIMESTAMP << 3) | (2 << 1),
		0x91, 0x92, 0x93, 0x94, 0x95, 0x96};
	reset_model(true, false);
	assert(hw.ext->ext_free_running && hw.ext->ext_read_frame);
	hw.dummy_prefix = true;
	for (unsigned i = 0; i < 18; i++)
		hw.external[0x31 + i] = (uint8_t)(0x10 + i);

	/* Every batched slot repeats the dummy bytes and carries whole words. */
	arm_frame(0x31, 2, 3, out, sizeof(out));
	assert(hw.last_master == (0x44 | 2));
	for (unsigned slot = 0; slot < 3; slot++) {
		unsigned base = LSM6DSV_SLV0_ADD + slot * 3;
		assert(hw.hub[base] == ((0x14 << 1) | 1));
		assert(hw.hub[base + 1] == 0x31 + 3 * slot);
		assert(hw.hub[base + 2] ==
			((slot == 0 ? 0xa0 : slot == 1 ? 0x10 : 0) | LSM6DSV_SHUB_BATCH_EXT_SENS | 5));
	}
	unsigned starts = hw.starts, stops = hw.stops;
	assert(read_frame(0x31, 2, 3, out, sizeof(out)) == -EAGAIN);
	expect_fill(out, sizeof(out), 0xa5);

	/* TAG_CNT belongs to FIFO records, not the SHUB acquisition. It wraps
	 * and need not agree between the three slots or interleaved inertial data. */
	fifo_emit_slot(0, 3);
	fifo_emit_inertial(LSM6DSV_TAG_GYRO_NC, 1, 0x21);
	fifo_emit_slot(1, 0);
	fifo_push(LSM6DSV_TAG_TIMESTAMP, 2, timestamp);
	fifo_emit_inertial(LSM6DSV_TAG_ACCEL_NC, 3, 0x41);
	fifo_emit_slot(2, 2);
	memcpy(expected, hw.external + 0x31, sizeof(expected));
	hw.external[0x31] ^= 0xff; /* Values are captured on FIFO enqueue. */
	assert(lsm_fifo_read(batch, 3 * FIFO_PACKET_BYTES) == 1);
	assert(hw.fifo.pops == 3 && memcmp(batch, gyro_record, sizeof(gyro_record)) == 0);
	assert(lsm_fifo_process(0, batch, accel, gyro) == 0);
	assert(read_frame(0x31, 2, 3, out, sizeof(out)) == -EAGAIN);
	expect_fill(out, sizeof(out), 0xa5);
	assert(lsm_fifo_read(batch, sizeof(batch)) == 2);
	assert(hw.fifo.pops == 6);
	assert(memcmp(batch, timestamp_record, sizeof(timestamp_record)) == 0);
	assert(memcmp(batch + FIFO_PACKET_BYTES, accel_record, sizeof(accel_record)) == 0);
	assert(lsm_fifo_process(0, batch, accel, gyro) == 1);
	assert(lsm_fifo_process(1, batch, accel, gyro) == 0);
	assert(read_frame(0x31, 2, 3, out, sizeof(out)) == 0);
	assert(memcmp(out, expected, sizeof(out)) == 0);
	memset(out, 0xa5, sizeof(out));
	assert(read_frame(0x31, 2, 3, out, sizeof(out)) == -EAGAIN);
	expect_fill(out, sizeof(out), 0xa5);

	/* Several complete epochs before consumption yield only the latest once. */
	memset(hw.external + 0x31, 0x63, sizeof(out));
	fifo_emit_frame(2);
	memset(hw.external + 0x31, 0x94, sizeof(out));
	fifo_emit_frame(3);
	memcpy(expected, hw.external + 0x31, sizeof(expected));
	memset(hw.external + 0x31, 0xbc, sizeof(out));
	unsigned pops = hw.fifo.pops;
	assert(lsm_fifo_read(batch, sizeof(batch)) == 0);
	assert(hw.fifo.pops == pops + 6 && hw.fifo.count == 0);
	assert(read_frame(0x31, 2, 3, out, sizeof(out)) == 0);
	assert(memcmp(out, expected, sizeof(out)) == 0);
	memset(out, 0xa5, sizeof(out));
	assert(read_frame(0x31, 2, 3, out, sizeof(out)) == -EAGAIN);
	expect_fill(out, sizeof(out), 0xa5);

	/* A newer partial group cannot displace the latest completed frame. */
	memset(hw.external + 0x31, 0x12, sizeof(out));
	fifo_emit_frame(0);
	memset(hw.external + 0x31, 0x36, sizeof(out));
	fifo_emit_slot(0, 2);
	assert(lsm_fifo_read(batch, sizeof(batch)) == 0);
	assert(read_frame(0x31, 2, 3, out, sizeof(out)) == 0);
	expect_fill(out, sizeof(out), 0x12);
	memset(out, 0xa5, sizeof(out));
	assert(read_frame(0x31, 2, 3, out, sizeof(out)) == -EAGAIN);
	expect_fill(out, sizeof(out), 0xa5);
	fifo_emit_slot(1, 3);
	fifo_emit_slot(2, 0);
	assert(lsm_fifo_read(batch, sizeof(batch)) == 0);
	assert(read_frame(0x31, 2, 3, out, sizeof(out)) == 0);
	expect_fill(out, sizeof(out), 0x36);
	assert(hw.starts == starts && hw.stops == stops && hw.master && !hw.bank);
	assert(gyro_discontinuities == 0);

	/* Invalid shapes never alter the running acquisition or touch the bus. */
	unsigned transactions = hw.transactions;
	assert(read_frame(0x31, 2, 3, invalid, sizeof(invalid)) == -E2BIG);
	assert(read_frame(0x31, 0, 8, invalid, 8) == -E2BIG);
	assert(read_frame(0x31, 7, 1, out, 1) == -EINVAL);
	assert(read_frame(0x31, 0, 0, out, 1) == -EINVAL);
	assert(hw.ext->ext_read_frame(0x80, 0x31, 0, 1, out, 1) == -EINVAL);
	assert(hw.transactions == transactions && hw.master);
}

static void test_fifo_generation_fence(void)
{
	uint8_t out[9], batch[4 * FIFO_PACKET_BYTES];
	uint8_t old0[6] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11};
	uint8_t old1[6] = {0x22, 0x22, 0x22, 0, 0, 0};
	const uint8_t inertial[2 * FIFO_PACKET_BYTES] = {
		(LSM6DSV_TAG_GYRO_NC << 3) | (2 << 1), 0x31, 0x32, 0x33, 0x34, 0x35, 0x36,
		(LSM6DSV_TAG_ACCEL_NC << 3) | (3 << 1), 0x61, 0x62, 0x63, 0x64, 0x65, 0x66,
	};
	reset_model(true, false);
	/* Both non-SHUB records count toward the arm-time fence. */
	fifo_emit_inertial(LSM6DSV_TAG_GYRO_NC, 2, 0x31);
	fifo_emit_inertial(LSM6DSV_TAG_ACCEL_NC, 3, 0x61);
	fifo_push(LSM6DSV_TAG_SENSORHUB_SLAVE0, 0, old0);
	fifo_push(LSM6DSV_TAG_SENSORHUB_SLAVE1, 1, old1);
	arm_frame(0x08, 0, 1, out, sizeof(out));
	fifo_emit_frame(2);
	assert(lsm_fifo_read(batch, 2 * FIFO_PACKET_BYTES) == 2);
	assert(memcmp(batch, inertial, sizeof(inertial)) == 0 && hw.fifo.pops == 2);
	assert(read_frame(0x08, 0, 1, out, sizeof(out)) == -EAGAIN);
	assert(lsm_fifo_read(batch, 2 * FIFO_PACKET_BYTES) == 0);
	assert(hw.fifo.pops == 4); /* Stripped pre-arm slots still count toward the fence. */
	assert(read_frame(0x08, 0, 1, out, sizeof(out)) == -EAGAIN);
	expect_fill(out, sizeof(out), 0xa5);
	assert(lsm_fifo_read(batch, sizeof(batch)) == 0);
	assert(hw.fifo.pops == 6 && hw.fifo.count == 0);
	assert(read_frame(0x08, 0, 1, out, sizeof(out)) == 0);
	assert(memcmp(out, hw.external + 0x08, sizeof(out)) == 0);
	assert(gyro_discontinuities == 0);
}

static void test_fifo_compaction_without_and_with_frame(void)
{
	uint8_t frame[9], batch[5 * FIFO_PACKET_BYTES];
	const uint8_t payload[6] = {1, 2, 3, 4, 5, 6};
	const uint8_t inertial[2 * FIFO_PACKET_BYTES] = {
		(LSM6DSV_TAG_GYRO_NC << 3) | (1 << 1), 0x31, 0x32, 0x33, 0x34, 0x35, 0x36,
		(LSM6DSV_TAG_ACCEL_NC << 3) | (2 << 1), 0x51, 0x52, 0x53, 0x54, 0x55, 0x56,
	};
	/* Stray SHUB records are filtered even when no frame stream is armed. */
	reset_model(false, false);
	fifo_push(LSM6DSV_TAG_SENSORHUB_SLAVE0, 3, payload);
	fifo_emit_nack(2, 0);
	fifo_emit_inertial(LSM6DSV_TAG_GYRO_NC, 1, 0x31);
	fifo_push(LSM6DSV_TAG_SENSORHUB_SLAVE3, 0, payload);
	fifo_emit_inertial(LSM6DSV_TAG_ACCEL_NC, 2, 0x51);
	assert(lsm_fifo_read(batch, sizeof(batch)) == 2);
	assert(hw.fifo.pops == 5 && hw.fifo.count == 0);
	assert(memcmp(batch, inertial, sizeof(inertial)) == 0 && gyro_discontinuities == 0);

	/* An external-only drain returns no inertial packets yet publishes once. */
	reset_model(true, false);
	arm_frame(0x08, 0, 1, frame, sizeof(frame));
	unsigned starts = hw.starts, stops = hw.stops;
	fifo_emit_frame(3);
	assert(lsm_fifo_read(batch, sizeof(batch)) == 0);
	assert(hw.fifo.pops == 2 && hw.fifo.count == 0);
	assert(read_frame(0x08, 0, 1, frame, sizeof(frame)) == 0);
	assert(memcmp(frame, hw.external + 0x08, sizeof(frame)) == 0);
	memset(frame, 0xa5, sizeof(frame));
	assert(read_frame(0x08, 0, 1, frame, sizeof(frame)) == -EAGAIN);
	expect_fill(frame, sizeof(frame), 0xa5);
	assert(hw.starts == starts && hw.stops == stops && gyro_discontinuities == 0);
}

static void test_fifo_incomplete_misordered_and_nack(void)
{
	uint8_t out[16], expected[16], batch[8 * FIFO_PACKET_BYTES];
	for (unsigned bad_slot = 0; bad_slot < 4; bad_slot++) {
		reset_model(true, false);
		arm_frame(0x08, 0, 4, out, sizeof(out));
		assert(hw.last_master == (0x44 | 3));
		unsigned starts = hw.starts, stops = hw.stops;
		fifo_emit_frame(0);
		assert(lsm_fifo_read(batch, sizeof(batch)) == 0 && hw.fifo.pops == 4);
		memcpy(expected, hw.external + 0x08, sizeof(expected));
		memset(hw.external + 0x08, 0x7b, sizeof(out));
		for (unsigned slot = 0; slot < 4; slot++) {
			unsigned config = hw.hub[LSM6DSV_SLV0_ADD + 3 * slot + 2];
			assert((config & 7) == 4 && (config & LSM6DSV_SHUB_BATCH_EXT_SENS));
			if (slot == bad_slot)
				fifo_emit_nack(slot, (uint8_t)(3 + slot));
			else
				fifo_emit_slot(slot, (uint8_t)(3 + slot));
		}
		assert(lsm_fifo_read(batch, sizeof(batch)) == 0 && hw.fifo.pops == 8);
		/* The failed group must not replace the previous complete frame. */
		assert(read_frame(0x08, 0, 4, out, sizeof(out)) == 0);
		assert(memcmp(out, expected, sizeof(out)) == 0);
		memset(out, 0xa5, sizeof(out));
		assert(read_frame(0x08, 0, 4, out, sizeof(out)) == -EAGAIN);
		expect_fill(out, sizeof(out), 0xa5);
		memset(hw.external + 0x08, 0x3c, sizeof(out));
		fifo_emit_frame(2);
		assert(lsm_fifo_read(batch, sizeof(batch)) == 0 && hw.fifo.pops == 12);
		assert(read_frame(0x08, 0, 4, out, sizeof(out)) == 0);
		assert(memcmp(out, hw.external + 0x08, sizeof(out)) == 0);
		assert(hw.starts == starts && hw.stops == stops && gyro_discontinuities == 0);
	}

	reset_model(true, false);
	arm_frame(0x08, 0, 4, out, sizeof(out));
	fifo_emit_frame(0); /* Corruption also invalidates an unread older frame. */
	assert(lsm_fifo_read(batch, sizeof(batch)) == 0);
	fifo_emit_slot(0, 3);
	fifo_emit_slot(2, 0); /* Missing slot 1: do not splice across the gap. */
	fifo_emit_slot(1, 1);
	fifo_emit_slot(3, 2);
	assert(lsm_fifo_read(batch, sizeof(batch)) == 0);
	assert(read_frame(0x08, 0, 4, out, sizeof(out)) == -EAGAIN);
	expect_fill(out, sizeof(out), 0xa5);
	fifo_emit_slot(0, 0);
	fifo_emit_slot(1, 3);
	assert(lsm_fifo_read(batch, sizeof(batch)) == 0);
	assert(read_frame(0x08, 0, 4, out, sizeof(out)) == -EAGAIN);
	expect_fill(out, sizeof(out), 0xa5);
	fifo_emit_slot(2, 1);
	fifo_emit_slot(3, 2);
	assert(lsm_fifo_read(batch, sizeof(batch)) == 0);
	assert(read_frame(0x08, 0, 4, out, sizeof(out)) == 0);
	assert(memcmp(out, hw.external + 0x08, sizeof(out)) == 0);
}

static void test_frame_reconfiguration_and_incidental_operations(void)
{
	uint8_t frame[9], out[7], batch[8 * FIFO_PACKET_BYTES];
	for (unsigned operation = 0; operation < 3; operation++) {
		reset_model(true, false);
		arm_frame(0x08, 0, 1, frame, sizeof(frame));
		fifo_emit_frame(3);
		fifo_emit_inertial(LSM6DSV_TAG_GYRO_NC, 0, 0x71);
		if (operation == 0) {
			/* A different frame shape fences the previously queued records. */
			arm_frame(0x0a, 0, 1, frame, sizeof(frame));
		} else if (operation == 1) {
			expect_sample(out);
		} else {
			uint8_t command[] = {0x1b, 0x01}; /* MMC CONTROL_0: TAKE_MEAS_M */
			assert(hw.ext->ext_write(0x14, command, sizeof(command)) == 0);
			assert(hw.external[0x1b] == 0x01);
			/* A successful command remains quiescent until the next frame request. */
			assert(!hw.master && !hw.bank);
			/* A not-yet-DONE poll must not wake the old frame stream before
			 * the caller can poll again and observe completion. */
			uint8_t status_reg = 0x18, status;
			hw.external[status_reg] = 0;
			assert(hw.ext->ext_write_read(0x14, &status_reg, 1, &status, 1) == 0);
			assert(status == 0 && !hw.master && !hw.bank);
			hw.external[status_reg] = 0x40;
			assert(hw.ext->ext_write_read(0x14, &status_reg, 1, &status, 1) == 0);
			assert(status == 0x40 && !hw.master && !hw.bank);
			uint8_t xyz_reg = 0x00, xyz[9];
			for (unsigned i = 0; i < sizeof(xyz); i++)
				hw.external[xyz_reg + i] = (uint8_t)(0x80 + i);
			assert(hw.ext->ext_write_read(0x14, &xyz_reg, 1, xyz, sizeof(xyz)) == 0);
			assert(memcmp(xyz, hw.external + xyz_reg, sizeof(xyz)) == 0 && !hw.master && !hw.bank);
			unsigned starts = hw.starts, writes = hw.writes;
			advance(30000);
			assert(!hw.master && hw.starts == starts && hw.writes == writes);
		}
		uint8_t reg = operation == 0 ? 0x0a : 0x08;
		if (operation != 2)
			assert(hw.master && !hw.bank);
		memset(frame, 0xa5, sizeof(frame));
		assert(read_frame(reg, 0, 1, frame, sizeof(frame)) == -EAGAIN);
		assert(hw.master && !hw.bank);
		memset(hw.external + reg, 0x95, sizeof(frame));
		fifo_emit_frame(0);
		assert(lsm_fifo_read(batch, 3 * FIFO_PACKET_BYTES) == 1);
		const uint8_t gyro_record[7] = {LSM6DSV_TAG_GYRO_NC << 3,
			0x71, 0x72, 0x73, 0x74, 0x75, 0x76};
		assert(memcmp(batch, gyro_record, sizeof(gyro_record)) == 0 && hw.fifo.pops == 3);
		assert(read_frame(reg, 0, 1, frame, sizeof(frame)) == -EAGAIN);
		expect_fill(frame, sizeof(frame), 0xa5);
		assert(lsm_fifo_read(batch, sizeof(batch)) == 0 && hw.fifo.pops == 5);
		assert(read_frame(reg, 0, 1, frame, sizeof(frame)) == 0);
		assert(memcmp(frame, hw.external + reg, sizeof(frame)) == 0);
	}
}

static void test_fifo_transport_and_postdrain_loss(void)
{
	uint8_t frame[9], batch[4 * FIFO_PACKET_BYTES];
	for (unsigned failure = 0; failure < 3; failure++) {
		reset_model(true, false);
		arm_frame(0x08, 0, 1, frame, sizeof(frame));
		/* An unread good frame must also be invalidated by a later loss. */
		fifo_emit_frame(0);
		assert(lsm_fifo_read(batch, sizeof(batch)) == 0 && hw.fifo.pops == 2);
		fifo_emit_slot(0, 1);
		assert(lsm_fifo_read(batch, sizeof(batch)) == 0 && hw.fifo.pops == 3);
		fifo_emit_inertial(LSM6DSV_TAG_GYRO_NC, 2, 0x21);
		fifo_emit_slot(1, 3);
		if (failure == 0) {
			hw.fifo.overrun_after = 1; /* Latches during, not before, the drain. */
		} else if (failure == 1) {
			/* The count read succeeds; the first status read after a pop fails. */
			hw.fifo.fail_status = true;
			hw.fifo.fail_status_after_pops = hw.fifo.pops + 1;
		} else {
			hw.fifo.partial_pops = 1;
			fail_on(READ, false, LSM6DSV_FIFO_DATA_OUT_TAG, -1, 0);
		}
		assert(lsm_fifo_read(batch, sizeof(batch)) == 0);
		assert(gyro_discontinuities == 1 && hw.fifo.count == 0);
		if (failure == 1)
			assert(!hw.fifo.fail_status);
		else if (failure == 2)
			assert(!hw.fault.armed);
		memset(frame, 0xa5, sizeof(frame));
		assert(read_frame(0x08, 0, 1, frame, sizeof(frame)) == -EAGAIN);
		expect_fill(frame, sizeof(frame), 0xa5);
		assert(hw.master);
		memset(hw.external + 0x08, 0x39, sizeof(frame));
		fifo_emit_frame(3);
		assert(lsm_fifo_read(batch, sizeof(batch)) == 0);
		assert(read_frame(0x08, 0, 1, frame, sizeof(frame)) == 0);
		assert(memcmp(frame, hw.external + 0x08, sizeof(frame)) == 0);
	}
}

static void test_fifo_resync_failure_rearms(void)
{
	uint8_t out[9], batch[2 * FIFO_PACKET_BYTES];
	reset_model(true, false);
	arm_frame(0x08, 0, 1, out, sizeof(out));
	fifo_emit_frame(0);
	hw.fifo.overrun_after = 1;
	fail_on(WRITE, false, LSM6DSV_FIFO_CTRL4, 0, 0);
	assert(lsm_fifo_read(batch, sizeof(batch)) == 0);
	assert(!hw.fault.armed && gyro_discontinuities == 1 && !hw.master && !hw.bank);
	memset(out, 0xa5, sizeof(out));
	assert(read_frame(0x08, 0, 1, out, sizeof(out)) == -EAGAIN);
	expect_fill(out, sizeof(out), 0xa5);
	assert(hw.master);
	memset(hw.external + 0x08, 0x64, sizeof(out));
	fifo_emit_frame(2);
	assert(lsm_fifo_read(batch, sizeof(batch)) == 0);
	assert(read_frame(0x08, 0, 1, out, sizeof(out)) == 0);
	expect_fill(out, sizeof(out), 0x64);
}

static void test_frame_scanning_and_prefetch_disabled(void)
{
	uint8_t frame[9];
	for (unsigned scanning = 0; scanning < 2; scanning++) {
		reset_model(false, false);
		if (scanning) {
			assert(lsm_ext_setup(SENSOR_EXT_MODE_I2CM_PROXY) == 0);
			assert(hw.ext->ext_set_prefetch(true) == 0);
		}
		assert(read_frame(0x08, 0, 1, frame, sizeof(frame)) == 0);
		assert(memcmp(frame, hw.external + 0x08, sizeof(frame)) == 0);
		assert(!hw.master && !hw.bank && hw.fifo.count == 0);
		assert(!(hw.hub[LSM6DSV_SLV0_CONFIG] & LSM6DSV_SHUB_BATCH_EXT_SENS));
	}
}

static void test_frame_arm_status_failure(void)
{
	uint8_t out[9], batch[2 * FIFO_PACKET_BYTES];
	reset_model(true, false);
	fail_on(READ, false, LSM6DSV_FIFO_STATUS1, -1, 0);
	memset(out, 0xa5, sizeof(out));
	assert(read_frame(0x08, 0, 1, out, sizeof(out)) == -ENXIO);
	expect_fill(out, sizeof(out), 0xa5);
	assert(!hw.fault.armed && !hw.master && !hw.bank && gyro_discontinuities == 1);
	arm_frame(0x08, 0, 1, out, sizeof(out));
	fifo_emit_frame(0);
	assert(lsm_fifo_read(batch, sizeof(batch)) == 0);
	assert(read_frame(0x08, 0, 1, out, sizeof(out)) == 0);
	assert(memcmp(out, hw.external + 0x08, sizeof(out)) == 0);
}

static void test_frame_prefetch_toggle(void)
{
	uint8_t out[9], batch[4 * FIFO_PACKET_BYTES];
	uint8_t stale0[6] = {1, 1, 1, 1, 1, 1}, stale1[6] = {2, 2, 2};
	reset_model(true, false);
	arm_frame(0x08, 0, 1, out, sizeof(out));
	fifo_emit_frame(0);
	assert(lsm_fifo_read(batch, sizeof(batch)) == 0);
	assert(hw.ext->ext_set_prefetch(false) == 0);
	memset(hw.external + 0x08, 0x52, sizeof(out));
	assert(read_frame(0x08, 0, 1, out, sizeof(out)) == 0);
	expect_fill(out, sizeof(out), 0x52);
	assert(!hw.master);
	assert(hw.ext->ext_set_prefetch(true) == 0);
	fifo_emit_inertial(LSM6DSV_TAG_GYRO_NC, 1, 0x31);
	fifo_push(LSM6DSV_TAG_SENSORHUB_SLAVE0, 2, stale0);
	fifo_push(LSM6DSV_TAG_SENSORHUB_SLAVE1, 3, stale1);
	arm_frame(0x08, 0, 1, out, sizeof(out));
	fifo_emit_frame(0);
	assert(lsm_fifo_read(batch, 3 * FIFO_PACKET_BYTES) == 1);
	assert((batch[0] >> 3) == LSM6DSV_TAG_GYRO_NC && batch[1] == 0x31);
	assert(read_frame(0x08, 0, 1, out, sizeof(out)) == -EAGAIN);
	expect_fill(out, sizeof(out), 0xa5);
	assert(lsm_fifo_read(batch, sizeof(batch)) == 0);
	assert(read_frame(0x08, 0, 1, out, sizeof(out)) == 0);
	expect_fill(out, sizeof(out), 0x52);
}

static void test_fifo_status_count_and_packet_pops(void)
{
	uint8_t batch[4 * FIFO_PACKET_BYTES];
	reset_model(false, false);
	for (unsigned i = 0; i < 260; i++)
		fifo_emit_inertial(i & 1 ? LSM6DSV_TAG_ACCEL_NC : LSM6DSV_TAG_GYRO_NC,
			(uint8_t)i, (uint8_t)i);
	assert(lsm_fifo_read(batch, sizeof(batch)) == 4);
	assert(hw.fifo.count == 256);
	for (unsigned i = 0; i < 4; i++) {
		assert((batch[i * FIFO_PACKET_BYTES] >> 3) ==
			(i & 1 ? LSM6DSV_TAG_ACCEL_NC : LSM6DSV_TAG_GYRO_NC));
		assert(batch[i * FIFO_PACKET_BYTES + 1] == (uint8_t)i);
	}
	assert(lsm_fifo_read(batch, 3 * FIFO_PACKET_BYTES) == 3);
	assert(hw.fifo.count == 253 && batch[1] == 4 && batch[8] == 5 && batch[15] == 6);
	assert(gyro_discontinuities == 0);
}

static void test_stop_settles_only_running_slots(void)
{
	uint8_t out[9];
	reset_model(false, false);
	hw.sleep_count = 0;
	assert(hw.ext->ext_write_read(0x1e, &(uint8_t){0x08}, 1, out, 9) == 0);
	unsigned settles = 0;
	for (unsigned i = 0; i < hw.sleep_count; i++) {
		if (hw.sleeps_us[i] == 250)
			continue;
		assert(hw.sleeps_us[i] == 100 + 250 * 2);
		settles++;
	}
	assert(settles == 1);
	/* A stopped hub is not settled again. */
	hw.sleep_count = 0;
	assert(hw.ext->ext_set_prefetch(false) == 0);
	assert(hw.sleep_count == 0);
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
		arm_frame(0x08, 0, 1, out, sizeof(out));
		fifo_emit_frame(0);
		fail_on(WRITE, true,
			restore_failure ? LSM6DSV_FUNC_CFG_ACCESS : LSM6DSV_MASTER_CONFIG, 0, 0);
		lsm_shutdown();
		assert(!hw.fault.armed && !hw.master && !hw.bank);
		assert(hw.fifo.count == 0);
		initialize();
		memset(hw.external + 0x08, 0x62, 7);
		expect_sample(out);
		arm_frame(0x08, 0, 1, out, sizeof(out));
		fifo_emit_frame(2);
		uint8_t batch[2 * FIFO_PACKET_BYTES];
		assert(lsm_fifo_read(batch, sizeof(batch)) == 0);
		assert(read_frame(0x08, 0, 1, out, sizeof(out)) == 0);
		expect_fill(out, sizeof(out), 0x62);
	}
}

static void test_fifo_destructive_loss_notification(void)
{
	reset_model(false, false);
	uint8_t record[7];
	fifo_emit_inertial(LSM6DSV_TAG_GYRO_NC, 0, 0x20);
	hw.fifo.flags = LSM6DSV_FIFO_OVR_LATCHED;
	assert(lsm_fifo_read(record, sizeof(record)) == 0);
	assert(gyro_discontinuities == 1);
	/* A latched overrun is still loss if the current FIFO count is zero. */
	hw.fifo.flags = LSM6DSV_FIFO_OVR_LATCHED;
	assert(lsm_fifo_read(record, sizeof(record)) == 0);
	assert(gyro_discontinuities == 2);
	fifo_emit_inertial(LSM6DSV_TAG_GYRO_NC, 1, 0x30);
	fail_on(READ, false, LSM6DSV_FIFO_DATA_OUT_TAG, -1, 0);
	assert(lsm_fifo_read(record, sizeof(record)) == 0);
	assert(gyro_discontinuities == 3);
	fifo_emit_inertial(LSM6DSV_TAG_GYRO_NC, 2, 0x40);
	hw.fifo.flags = LSM6DSV_FIFO_OVR_LATCHED;
	hw.partial_status_error = true; /* Hardware consumed the latch before the error. */
	fail_on(READ, false, LSM6DSV_FIFO_STATUS1, -1, 0);
	assert(lsm_fifo_read(record, sizeof(record)) == 0);
	/* A failed status transfer may have cleared the overrun latch. */
	assert(gyro_discontinuities == 4 && hw.fifo.count == 0);
	fifo_emit_inertial(LSM6DSV_TAG_GYRO_NC, 3, 0x50);
	hw.fifo.flags = LSM6DSV_FIFO_FULL_IA;
	assert(lsm_fifo_read(record, sizeof(record)) == 1);
	assert(gyro_discontinuities == 4);
}

static void test_wom_bus_failures(void)
{
	reset_model(false, false);
	hw.transactions = 0;
	assert(lsm_setup_WOM() != 0xFF);
	unsigned transactions = hw.transactions;
	assert(transactions > 3);
	for (unsigned failure = 1; failure <= transactions; failure++) {
		reset_model(false, false);
		hw.transactions = 0;
		hw.fail_transaction = failure;
		assert(lsm_setup_WOM() == 0xFF);
	}
	printf("LSM6DSV WOM: %u individual bus failures rejected\n", transactions);
}

static void test_hp_minimum_rate(void)
{
	reset_model(false, false);
	float accel, gyro;
	assert(lsm_update_odr(1.0f, 1.0f, &accel, &gyro) == 0);
	assert((hw.registers[LSM6DSV_CTRL1] & 0x0f) == ODR_7_5Hz);
	assert((hw.registers[LSM6DSV_CTRL2] & 0x0f) == ODR_7_5Hz);
	assert(accel == 1.0f / 7.5f && gyro == 1.0f / 7.5f);
	assert(lsm_update_odr(0.0f, 0.0f, &accel, &gyro) == 0);
	assert((hw.registers[LSM6DSV_CTRL1] & 0x0f) == ODR_OFF);
	assert((hw.registers[LSM6DSV_CTRL2] & 0x0f) == ODR_OFF);
}

static void test_consecutive_unknown_tags(void)
{
	reset_model(false, false);
	uint8_t unknown[7] = {0xf8}, valid[7] = {LSM6DSV_TAG_ACCEL_NC << 3};
	float accel[3], gyro[3];
	for (unsigned i = 0; i < 6; i++) {
		assert(lsm_fifo_process(0, unknown, accel, gyro) == 1);
		assert(lsm_fifo_process(0, valid, accel, gyro) == 0);
	}
	assert(gyro_discontinuities == 0);
	for (unsigned i = 0; i < 4; i++) {
		fifo_push(0x1f, 0, &unknown[1]);
		uint8_t batch[7];
		assert(lsm_fifo_read(batch, sizeof(batch)) == 1);
		assert(lsm_fifo_process(0, batch, accel, gyro) == 1);
		assert(gyro_discontinuities == (i == 3 ? 1u : 0u));
	}
}

static void test_adaptive_group(void)
{
	static const uint8_t lengths[] = {1, 7, 8, 14, 15, 18};
	uint8_t sub = 8, out[19];
	for (unsigned n = 0; n < ARRAY_SIZE(lengths); n++) {
		unsigned len = lengths[n], slots = (len + 6) / 7;
		reset_model(true, true);
		assert(hw.ext->ext_write_read(0x1e, &sub, 1, out, len) == 0);
		assert(memcmp(out, hw.external + sub, len) == 0);
		assert(hw.last_master == (0x44 | (slots - 1)));
		for (unsigned slot = 0, offset = 0; slot < slots; slot++) {
			unsigned take = len - offset > 7 ? 7 : len - offset;
			unsigned base = LSM6DSV_SLV0_ADD + slot * 3;
			assert(hw.hub[base] == 0x3d);
			assert(hw.hub[base + 1] == sub + offset);
			assert(hw.hub[base + 2] == ((slot == 0 ? 0xa0 : slot == 1 ? 0x10 : 0) | take));
			offset += take;
		}
		advance(150000);
		assert(hw.ext->ext_write_read(0x1e, &sub, 1, out, len) == 0);
		assert(memcmp(out, hw.external + sub, len) == 0);
		for (unsigned slot = 0; slot < slots; slot++) {
			reset_model(false, true);
			hw.error = 8u << slot;
			memset(out, 0xa5, sizeof(out));
			assert(hw.ext->ext_write_read(0x1e, &sub, 1, out, len) == -EIO);
			for (unsigned i = 0; i < sizeof(out); i++)
				assert(out[i] == 0xa5);
		}
	}
	reset_model(true, true);
	unsigned transactions = hw.transactions;
	assert(hw.ext->ext_write_read(0x1e, &sub, 1, out, 19) == -EINVAL);
	assert(hw.transactions == transactions);
	assert(hw.ext->ext_write_read(0x1e, &sub, 1, out, 18) == 0);
	assert(hw.ext->ext_write_read(0x1e, &sub, 1, out, 1) == 0);
	assert(hw.last_master == 0x44);
	uint8_t command[] = {0x20, 0x73};
	assert(hw.ext->ext_write(0x1e, command, sizeof(command)) == 0);
	assert(hw.last_master == 0x44 && hw.writes == 1);
}

static void test_group_failure_output(void)
{
	uint8_t sub = 8, out[18];
	reset_model(true, false);
	hw.transactions = 0;
	assert(hw.ext->ext_write_read(0x1e, &sub, 1, out, sizeof(out)) == 0);
	unsigned total = hw.transactions;
	for (unsigned failure = 1; failure <= total; failure++) {
		reset_model(true, false);
		hw.transactions = 0;
		hw.fail_transaction = failure;
		hw.partial_read_error = true;
		memset(out, 0xa5, sizeof(out));
		assert(hw.ext->ext_write_read(0x1e, &sub, 1, out, sizeof(out)) != 0);
		for (unsigned i = 0; i < sizeof(out); i++)
			assert(out[i] == 0xa5);
		assert(!hw.master && !hw.bank);
	}
}

int main(void)
{
	test_adaptive_group();
	test_group_failure_output();
	test_consecutive_unknown_tags();
	test_hp_minimum_rate();
	test_nack(false);
	test_nack(true);
	test_configuration_failfast();
	test_stop_restore_invalidates();
	test_uncleared_epoch_refused();
	test_checked_epoch_error_and_freshness(false);
	test_checked_epoch_error_and_freshness(true);
	test_error_during_stop();
	test_pending_status_io_failure();
	test_failed_read_invalidates();
	test_deadlines_and_status_io();
	test_frame_ready_lifecycle();
	test_frame_layout_and_fifo_epochs();
	test_fifo_generation_fence();
	test_fifo_compaction_without_and_with_frame();
	test_fifo_incomplete_misordered_and_nack();
	test_frame_reconfiguration_and_incidental_operations();
	test_fifo_transport_and_postdrain_loss();
	test_fifo_resync_failure_rearms();
	test_frame_scanning_and_prefetch_disabled();
	test_frame_arm_status_failure();
	test_frame_prefetch_toggle();
	test_fifo_status_count_and_packet_pops();
	test_stop_settles_only_running_slots();
	test_prefetch_policy_lifecycle();
	test_ambiguous_write_is_not_replayed();
	test_shutdown_after_stop_failure();
	test_wom_bus_failures();
	test_fifo_destructive_loss_notification();
	puts("LSM6DSV full-driver proxy regressions passed");
	return 0;
}
