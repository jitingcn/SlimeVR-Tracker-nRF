#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sensor/imu/ICM45686.h"

/* DS-000577 host/IREG model. The complete production driver is linked, not
 * extracted or reimplemented. AUX completion captures the external registers;
 * STATUS.DONE and EXT_DEV_STATUS are read-clear, RD_DATA is cleared by GO.
 * No electrical behavior or actual sensor conversion accuracy is simulated. */
#define TOP(reg) ((uint16_t)(0xa200u | (reg)))
enum bus_kind { ADDRESS, READ, WRITE };
struct bus_record {
	enum bus_kind kind;
	uint16_t address;
	uint64_t at;
	int result;
};
static struct {
	uint8_t host[256], ireg[65536], external[256];
	uint16_t pointer;
	uint64_t now, last_ireg_end, complete_at, go_at;
	uint32_t latency, duration;
	bool accessed_ireg, busy, stuck, omit_done;
	uint8_t terminal_error, nack, pending_command, pending_sub;
	unsigned go_count, writes, data_reads, nack_reads, done_reads;
	struct bus_record records[32768];
	unsigned record_count, failure_index;
	struct {
		bool armed, apply_write;
		enum bus_kind kind;
		uint16_t address;
		unsigned skip;
	} fault;
	const sensor_ext_ssi_t *ext;
} hw;
static uint64_t minimum_ireg_gap = UINT64_MAX;

static void settle(void)
{
	if (!hw.busy || hw.stuck || hw.now < hw.complete_at)
		return;
	hw.busy = false;
	hw.ireg[TOP(ICM45686_I2CM_STATUS)] = hw.terminal_error |
		(hw.omit_done ? 0 : ICM45686_BIT_I2CM_STATUS_DONE);
	hw.ireg[TOP(ICM45686_I2CM_EXT_DEV_STATUS)] = hw.nack;
	if (!hw.nack && !hw.terminal_error) {
		unsigned len = hw.pending_command & 0x0f;
		if ((hw.pending_command & 0x30) == ICM45686_I2CM_CMD_RW_READ_REG) {
			for (unsigned i = 0; i < len; i++)
				hw.ireg[TOP(ICM45686_I2CM_RD_DATA_0) + i] =
					hw.external[(uint8_t)(hw.pending_sub + i)];
		} else {
			/* Count commands, not final register value: replaying a SINGLE
			 * trigger or FIFO pop cannot be made safe by writing the same byte. */
			hw.writes++;
			uint8_t sub = hw.ireg[TOP(ICM45686_I2CM_WR_DATA_0)];
			for (unsigned i = 1; i < len; i++)
				hw.external[(uint8_t)(sub + i - 1)] =
					hw.ireg[TOP(ICM45686_I2CM_WR_DATA_0) + i];
		}
	}
}

static void advance(uint64_t us)
{
	hw.now += us;
	settle();
}

int64_t host_k_uptime_get(void) { return hw.now / 1000; }
void host_k_busy_wait(uint32_t us) { advance(us); }
void host_k_msleep(int32_t ms) { assert(ms >= 0); advance((uint64_t)ms * 1000); }
void host_k_usleep(int32_t us) { assert(us >= 0); advance(us); }

static int transaction(enum bus_kind kind, uint16_t address, bool ireg)
{
	assert(hw.record_count < ARRAY_SIZE(hw.records));
	if (ireg) {
		assert(!hw.accessed_ireg || hw.now - hw.last_ireg_end >= 4);
		if (hw.accessed_ireg && hw.now - hw.last_ireg_end < minimum_ireg_gap)
			minimum_ireg_gap = hw.now - hw.last_ireg_end;
		hw.accessed_ireg = true;
	}
	struct bus_record *record = &hw.records[hw.record_count++];
	*record = (struct bus_record){.kind = kind, .address = address, .at = hw.now};
	advance(hw.latency);
	if (ireg)
		hw.last_ireg_end = hw.now;
	if (hw.fault.armed && hw.fault.kind == kind && hw.fault.address == address) {
		if (hw.fault.skip) {
			hw.fault.skip--;
		} else {
			hw.fault.armed = false;
			hw.failure_index = hw.record_count;
			record->result = -EIO;
			return -EIO;
		}
	}
	return 0;
}

static void write_ireg(uint16_t address, uint8_t value)
{
	hw.ireg[address] = value;
	if (address != TOP(ICM45686_I2CM_CONTROL) || !(value & ICM45686_I2CM_CONTROL_GO))
		return;
	assert(!hw.busy); /* Reprogramming/GO while an old transaction runs is unsafe. */
	assert(hw.ireg[TOP(ICM45686_DEV_PROFILE_1)] == 0x1e);
	hw.go_count++;
	hw.go_at = hw.now;
	hw.busy = true;
	hw.complete_at = hw.now + hw.duration;
	hw.pending_command = hw.ireg[TOP(ICM45686_I2CM_COMMAND_0)];
	hw.pending_sub = hw.ireg[TOP(ICM45686_DEV_PROFILE_0)];
	hw.ireg[TOP(ICM45686_I2CM_STATUS)] = ICM45686_BIT_I2CM_STATUS_BUSY;
	hw.ireg[TOP(ICM45686_I2CM_EXT_DEV_STATUS)] = 0x0f;
	memset(hw.ireg + TOP(ICM45686_I2CM_RD_DATA_0), 0, 21);
	settle();
}

int ssi_burst_write(enum sensor_interface_dev dev, uint8_t reg, const uint8_t *buf, uint32_t len)
{
	assert(dev == SENSOR_INTERFACE_DEV_IMU);
	if (reg == ICM45686_IREG_ADDR_15_8) {
		/* Only the address and first data byte fit in this host burst.
		 * More bytes hit host 0x7f/0x80, not auto-incremented IREG_DATA. */
		assert(len == 2 || len == 3);
		uint16_t address = (uint16_t)buf[0] << 8 | buf[1];
		int err = transaction(len == 2 ? ADDRESS : WRITE, address, true);
		if (!err || (hw.failure_index == hw.record_count && hw.fault.apply_write)) {
			hw.pointer = address;
			if (len == 3)
				write_ireg(hw.pointer++, buf[2]);
		}
		return err;
	}
	assert(reg != ICM45686_IREG_DATA && (unsigned)reg + len <= 256);
	int err = transaction(WRITE, reg, false);
	if (!err) {
		memcpy(hw.host + reg, buf, len);
		if (reg == ICM45686_REG_MISC2 && len == 1 && (buf[0] & 0x02)) {
			/* Global reset aborts a stuck AUX operation; it is not another GO. */
			hw.busy = false;
			memset(hw.ireg, 0, sizeof(hw.ireg));
			memset(hw.host, 0, sizeof(hw.host));
			hw.host[0x72] = 0xe9;
			hw.pointer = 0;
		}
	}
	return err;
}

int ssi_reg_write_byte(enum sensor_interface_dev dev, uint8_t reg, uint8_t value)
{
	assert(dev == SENSOR_INTERFACE_DEV_IMU);
	if (reg == ICM45686_IREG_DATA) {
		int err = transaction(WRITE, hw.pointer, true);
		if (!err)
			write_ireg(hw.pointer++, value);
		return err;
	}
	return ssi_burst_write(dev, reg, &value, 1);
}

int ssi_reg_read_byte(enum sensor_interface_dev dev, uint8_t reg, uint8_t *out)
{
	assert(dev == SENSOR_INTERFACE_DEV_IMU);
	bool ireg = reg == ICM45686_IREG_DATA;
	uint16_t address = ireg ? hw.pointer : reg;
	int err = transaction(READ, address, ireg);
	if (err) {
		/* Failed transfers may leave plausible garbage. Consumers must use
		 * the return value, not a fabricated DONE/no-NACK status byte. */
		*out = address == TOP(ICM45686_I2CM_STATUS) ? 0x02 : 0;
		return err;
	}
	*out = ireg ? hw.ireg[address] : hw.host[address];
	if (ireg) {
		hw.pointer++;
		if (address == TOP(ICM45686_I2CM_STATUS)) {
			if (*out & ICM45686_BIT_I2CM_STATUS_DONE)
				hw.done_reads++;
			hw.ireg[address] &= ~ICM45686_BIT_I2CM_STATUS_DONE;
		} else if (address == TOP(ICM45686_I2CM_EXT_DEV_STATUS)) {
			hw.nack_reads++;
			hw.ireg[address] = 0;
		} else if (address >= TOP(ICM45686_I2CM_RD_DATA_0) &&
			address < TOP(ICM45686_I2CM_RD_DATA_0) + 21) {
			hw.data_reads++;
		}
	}
	return 0;
}

int ssi_burst_read(enum sensor_interface_dev dev, uint8_t reg, uint8_t *buf, uint32_t len)
{
	assert(dev == SENSOR_INTERFACE_DEV_IMU && reg != ICM45686_IREG_DATA);
	assert((unsigned)reg + len <= 256);
	int err = transaction(READ, reg, false);
	if (!err)
		memcpy(buf, hw.host + reg, len);
	return err;
}

int ssi_reg_update_byte(enum sensor_interface_dev dev, uint8_t reg, uint8_t mask, uint8_t value)
{
	uint8_t old;
	int err = ssi_reg_read_byte(dev, reg, &old);
	return err ? err : ssi_reg_write_byte(dev, reg, (old & ~mask) | (value & mask));
}

int ssi_burst_read_interval(enum sensor_interface_dev dev, uint8_t reg, uint8_t *buf,
	uint32_t len, uint32_t interval)
{
	(void)interval;
	return ssi_burst_read(dev, reg, buf, len);
}

int sensor_interface_spi_configure(enum sensor_interface_dev dev, uint32_t frequency, uint32_t dummy_reads)
{
	(void)frequency;
	(void)dummy_reads;
	assert(dev == SENSOR_INTERFACE_DEV_IMU);
	return 0;
}
void sensor_interface_ext_configure(const sensor_ext_ssi_t *ext) { hw.ext = ext; }

static void initialize(void)
{
	float accel, gyro;
	assert(icm45_init(0, 0.005f, 0.005f, &accel, &gyro) == 0);
}

static void reset_model(bool prefetch)
{
	memset(&hw, 0, sizeof(hw));
	hw.now = 100000;
	hw.latency = 1;
	hw.duration = 80;
	hw.host[0x72] = 0xe9;
	assert(icm45_ext_setup(SENSOR_EXT_MODE_I2CM_PROXY) == 0);
	assert(hw.ext);
	assert(hw.ext->ext_set_prefetch(prefetch) == 0);
	initialize();
}

static int read_sample(uint8_t sub, uint8_t *out, size_t len)
{
	return hw.ext->ext_write_read(0x1e, &sub, 1, out, len);
}

static void set_sample(uint8_t value)
{
	for (unsigned i = 0; i < 15; i++)
		hw.external[0x08 + i] = value + i;
}

static void expect_sample(const uint8_t *out, uint8_t value, size_t len)
{
	for (size_t i = 0; i < len; i++)
		assert(out[i] == (uint8_t)(value + i));
}

static void fail_on(enum bus_kind kind, uint16_t address, unsigned skip, bool apply_write)
{
	hw.fault.armed = true;
	hw.fault.kind = kind;
	hw.fault.address = address;
	hw.fault.skip = skip;
	hw.fault.apply_write = apply_write;
	hw.failure_index = 0;
}

static void expect_failure_stopped(void)
{
	assert(!hw.fault.armed && hw.failure_index);
	assert(hw.record_count == hw.failure_index);
	assert(hw.records[hw.failure_index - 1].result == -EIO);
}

static void test_prefetch_snapshot(void)
{
	reset_model(true);
	uint8_t out[15];
	set_sample(0x20);
	assert(read_sample(0x08, out, sizeof(out)) == 0);
	expect_sample(out, 0x20, sizeof(out));
	assert(hw.go_count == 2);
	set_sample(0x40);
	advance(hw.duration);
	set_sample(0x60);
	unsigned go = hw.go_count, done = hw.done_reads;
	assert(read_sample(0x08, out, sizeof(out)) == 0);
	expect_sample(out, 0x40, sizeof(out));
	assert(hw.go_count == go + 1 && hw.done_reads == done + 1);
	advance(hw.duration);
	set_sample(0x80);
	assert(read_sample(0x08, out, sizeof(out)) == 0);
	expect_sample(out, 0x60, sizeof(out));
	assert(hw.go_count == go + 2);
	printf("prefetch: repeated_reads=2 additional_GO=%u\n",
		hw.go_count - go);
}

static void test_disabled_prefetch_lifecycle(void)
{
	reset_model(false);
	uint8_t out[8];
	set_sample(0x31);
	assert(read_sample(0x08, out, sizeof(out)) == 0);
	expect_sample(out, 0x31, sizeof(out));
	assert(hw.go_count == 1 && !hw.busy);
	advance(4000);
	set_sample(0x51);
	initialize(); /* IMU init must not override an ICT-style read-clear policy. */
	assert(read_sample(0x08, out, sizeof(out)) == 0);
	expect_sample(out, 0x51, sizeof(out));
	assert(hw.go_count == 2 && !hw.busy);
	uint8_t command[] = {0x04, 0x02};
	assert(hw.ext->ext_write(0x1e, command, sizeof(command)) == 0);
	assert(hw.go_count == 3 && hw.writes == 1 && !hw.busy);
	icm45_shutdown();
	initialize();
	set_sample(0x71);
	assert(read_sample(0x08, out, sizeof(out)) == 0);
	expect_sample(out, 0x71, sizeof(out));
	assert(hw.go_count == 4 && !hw.busy);
}

static void test_error_completions(void)
{
	const uint8_t errors[] = {0x04, 0x08, 0x10, 0x20};
	for (unsigned i = 0; i < ARRAY_SIZE(errors); i++) {
		for (unsigned prefetched = 0; prefetched < 2; prefetched++) {
			reset_model(prefetched);
			uint8_t out[2] = {0xa5, 0x5a};
			if (prefetched) {
				assert(read_sample(0x08, out, sizeof(out)) == 0);
				out[0] = 0xa5;
				out[1] = 0x5a;
			}
			hw.terminal_error = errors[i];
			unsigned go = hw.go_count, data = hw.data_reads;
			assert(read_sample(0x08, out, sizeof(out)) < 0);
			assert(hw.go_count == go + !prefetched);
			assert(hw.data_reads == data && out[0] == 0xa5 && out[1] == 0x5a);
			hw.terminal_error = 0;
			set_sample(0x60);
			go = hw.go_count;
			assert(read_sample(0x08, out, sizeof(out)) == 0);
			expect_sample(out, 0x60, sizeof(out));
			assert(hw.go_count == go + 1 + prefetched);
		}
		reset_model(false);
		hw.terminal_error = errors[i];
		uint8_t command[] = {0x04, 0x02};
		assert(hw.ext->ext_write(0x1e, command, sizeof(command)) < 0);
		assert(hw.go_count == 1); /* Never implicitly replay a write. */
	}
	for (unsigned missing_done = 0; missing_done < 2; missing_done++) {
		for (unsigned prefetched = 0; prefetched < 2; prefetched++) {
			reset_model(prefetched);
			uint8_t out;
			if (prefetched)
				assert(read_sample(0x08, &out, 1) == 0);
			hw.omit_done = missing_done;
			hw.nack = !missing_done;
			out = 0xa5;
			unsigned go = hw.go_count, data = hw.data_reads;
			assert(read_sample(0x08, &out, 1) < 0);
			assert(out == 0xa5 && hw.data_reads == data);
			assert(hw.go_count == go + !prefetched);
			hw.omit_done = false;
			hw.nack = 0;
			set_sample(0x63);
			go = hw.go_count;
			assert(read_sample(0x08, &out, 1) == 0);
			assert(out == 0x63 && hw.go_count == go + 1 + prefetched);
		}
	}
}

static void test_transfer_faults(void)
{
	const struct { enum bus_kind kind; uint16_t address; } faults[] = {
		{WRITE, TOP(ICM45686_DEV_PROFILE_0)},
		{WRITE, TOP(ICM45686_DEV_PROFILE_1)},
		{WRITE, TOP(ICM45686_I2CM_COMMAND_0)},
		{WRITE, TOP(ICM45686_I2CM_CONTROL)},
		{ADDRESS, TOP(ICM45686_I2CM_STATUS)},
		{READ, TOP(ICM45686_I2CM_STATUS)},
		{ADDRESS, TOP(ICM45686_I2CM_EXT_DEV_STATUS)},
		{READ, TOP(ICM45686_I2CM_EXT_DEV_STATUS)},
		{ADDRESS, TOP(ICM45686_I2CM_RD_DATA_0)},
		{READ, TOP(ICM45686_I2CM_RD_DATA_0)},
		{READ, TOP(ICM45686_I2CM_RD_DATA_0) + 1},
	};
	for (unsigned i = 0; i < ARRAY_SIZE(faults); i++) {
		reset_model(true);
		uint8_t out[3];
		set_sample(0x20);
		assert(read_sample(0x08, out, sizeof(out)) == 0);
		advance(hw.duration);
		unsigned go = hw.go_count;
		fail_on(faults[i].kind, faults[i].address, 0, false);
		/* Change shape so a pending cached command must be discarded before
		 * every profile/command/data stage can be faulted. */
		assert(read_sample(0x08, out, 2) == -EIO);
		expect_failure_stopped();
		if (i < 6)
			assert(hw.go_count == go);
		else
			assert(hw.go_count == go + 1);
		advance(hw.duration);
		set_sample(0x70);
		go = hw.go_count;
		assert(read_sample(0x08, out, sizeof(out)) == 0);
		expect_sample(out, 0x70, sizeof(out));
		assert(hw.go_count == go + 2);
	}
	/* Matching cached reads have no setup phase: errors must invalidate that
	 * path too, including a successor GO that may already have reached silicon. */
	for (unsigned i = 3; i < ARRAY_SIZE(faults); i++) {
		reset_model(true);
		uint8_t out[3];
		set_sample(0x20);
		assert(read_sample(0x08, out, sizeof(out)) == 0);
		advance(hw.duration);
		fail_on(faults[i].kind, faults[i].address, 0, i == 3);
		assert(read_sample(0x08, out, sizeof(out)) == -EIO);
		expect_failure_stopped();
		advance(hw.duration);
		set_sample(0x70);
		unsigned go = hw.go_count;
		assert(read_sample(0x08, out, sizeof(out)) == 0);
		expect_sample(out, 0x70, sizeof(out));
		assert(hw.go_count == go + 2);
	}
}

static void test_write_faults(void)
{
	const struct { enum bus_kind kind; uint16_t address; unsigned skip; } faults[] = {
		{WRITE, TOP(ICM45686_DEV_PROFILE_0), 0},
		{WRITE, TOP(ICM45686_DEV_PROFILE_1), 0},
		{WRITE, TOP(ICM45686_I2CM_WR_DATA_0), 0},
		{WRITE, TOP(ICM45686_I2CM_WR_DATA_0) + 1, 0},
		{WRITE, TOP(ICM45686_I2CM_COMMAND_0), 0},
		{WRITE, TOP(ICM45686_I2CM_CONTROL), 0},
		{ADDRESS, TOP(ICM45686_I2CM_STATUS), 1},
		{READ, TOP(ICM45686_I2CM_STATUS), 1},
		{ADDRESS, TOP(ICM45686_I2CM_EXT_DEV_STATUS), 0},
		{READ, TOP(ICM45686_I2CM_EXT_DEV_STATUS), 0},
	};
	for (unsigned i = 0; i < ARRAY_SIZE(faults); i++) {
		reset_model(false);
		uint8_t command[] = {0x04, 0x02, 0x55};
		fail_on(faults[i].kind, faults[i].address, faults[i].skip, i == 5);
		assert(hw.ext->ext_write(0x1e, command, sizeof(command)) == -EIO);
		expect_failure_stopped();
		assert(hw.go_count == (i >= 5));
		advance(hw.duration);
		assert(hw.writes == (i >= 5));
		uint8_t out;
		assert(read_sample(0x04, &out, 1) == 0);
		assert(out == (i >= 5 ? 0x02 : 0));
		assert(hw.writes == (i >= 5));
	}
}

static void test_setup_failures(void)
{
	const struct { enum bus_kind kind; uint16_t address; } faults[] = {
		{READ, ICM45686_PWR_MGMT0},
		{WRITE, ICM45686_PWR_MGMT0},
		{WRITE, 0xa03c},
		{WRITE, ICM45686_IOC_PAD_SCENARIO_AUX_OVRD},
		{READ, ICM45686_REG_MISC1},
		{WRITE, ICM45686_REG_MISC1},
	};
	for (unsigned i = 0; i < ARRAY_SIZE(faults); i++) {
		reset_model(true);
		uint8_t out;
		set_sample(0x20);
		assert(read_sample(0x08, &out, 1) == 0);
		advance(hw.duration);
		hw.host[ICM45686_PWR_MGMT0] = 0;
		fail_on(faults[i].kind, faults[i].address, 0, false);
		unsigned go = hw.go_count;
		assert(icm45_ext_setup(SENSOR_EXT_MODE_I2CM_PROXY) == -EIO);
		expect_failure_stopped();
		assert(hw.go_count == go);
		set_sample(0x60);
		assert(read_sample(0x08, &out, 1) == 0);
		assert(out == 0x60 && hw.go_count == go + 2);
	}
}

static void test_elapsed_deadline(void)
{
	const unsigned latencies[] = {1, 700};
	for (unsigned i = 0; i < ARRAY_SIZE(latencies); i++) {
		reset_model(false);
		hw.latency = latencies[i];
		hw.stuck = true;
		uint8_t out = 0xa5;
		unsigned first = hw.record_count;
		assert(read_sample(0x08, &out, 1) == -ETIMEDOUT);
		assert(out == 0xa5 && hw.go_count == 1 && hw.data_reads == 0);
		uint64_t poll_start = 0;
		for (unsigned j = first; j < hw.record_count; j++) {
			const struct bus_record *r = &hw.records[j];
			if (r->kind == ADDRESS && r->address == TOP(ICM45686_I2CM_STATUS) && r->at >= hw.go_at) {
				poll_start = r->at;
				break;
			}
		}
		assert(poll_start);
		assert(poll_start <= hw.go_at + 8); /* No nominal AUX pre-delay. */
		/* Millisecond uptime may round the start down by 999us. A physical
		 * STATUS read is two host transfers with two mandatory 4us gaps. */
		uint64_t elapsed = hw.now - poll_start;
		assert(elapsed >= 9000);
		assert(elapsed <= 10000 + 2 * hw.latency + 8);
		printf("timeout: host_transfer_us=%u elapsed_poll_us=%llu bound_us=%u\n",
			hw.latency, (unsigned long long)elapsed, 10000 + 2 * hw.latency + 8);
		hw.stuck = false;
		advance(hw.duration);
		set_sample(0x60);
		assert(read_sample(0x08, &out, 1) == 0);
		assert(out == 0x60 && hw.go_count == 2);
	}
	/* Slow but legitimate AUX completion must use actual BUSY, not a nominal
	 * pre-delay or a poll-count budget. */
	reset_model(false);
	hw.duration = 8500;
	set_sample(0x39);
	uint8_t out;
	assert(read_sample(0x08, &out, 1) == 0);
	assert(out == 0x39 && hw.go_count == 1);
}

static void test_shutdown_recovers_failed_drain(void)
{
	for (unsigned status_read_error = 0; status_read_error < 2; status_read_error++) {
		reset_model(true);
		uint8_t out;
		set_sample(0x20);
		assert(read_sample(0x08, &out, 1) == 0);
		hw.stuck = true; /* The successor GO is still outstanding. */
		if (status_read_error)
			fail_on(READ, TOP(ICM45686_I2CM_STATUS), 0, false);
		icm45_shutdown();
		assert(!hw.busy); /* Failed drain must not prevent global reset recovery. */
		hw.stuck = false;
		initialize();
		set_sample(0x70);
		assert(read_sample(0x08, &out, 1) == 0);
		assert(out == 0x70);
	}
}

static void test_wom_register_spacing(void)
{
	reset_model(false);
	assert(icm45_setup_WOM() != 0xff);
	assert(hw.ireg[TOP(ICM45686_ACCEL_WOM_X_THR)] == 7);
	assert(hw.ireg[TOP(ICM45686_ACCEL_WOM_Y_THR)] == 7);
	assert(hw.ireg[TOP(ICM45686_ACCEL_WOM_Z_THR)] == 7);
	initialize();
	set_sample(0x45);
	uint8_t out;
	assert(read_sample(0x08, &out, 1) == 0);
	assert(out == 0x45 && hw.go_count == 1 && !hw.busy);
}

int main(void)
{
	test_prefetch_snapshot();
	test_disabled_prefetch_lifecycle();
	test_error_completions();
	test_transfer_faults();
	test_write_faults();
	test_setup_failures();
	test_elapsed_deadline();
	test_shutdown_recovers_failed_drain();
	test_wom_register_spacing();
	printf("IREG: minimum_inter_access_gap_us=%llu\n", (unsigned long long)minimum_ireg_gap);
	puts("ICM45686 actual-driver register regressions passed");
	return 0;
}
