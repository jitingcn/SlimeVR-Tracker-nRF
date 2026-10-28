#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sensor/imu/LSM6DSO.h"
#include "sensor/mag/QMC6309.h"

struct transaction {
	bool reading;
	uint8_t addr, sub, config, data;
};
struct bus_call {
	uint8_t reg;
	bool bank, reading;
};

/* Banked SPI model; a failed transfer may already have reached the device. */

static struct {
	uint8_t main[256], hub[256], external[256], status;
	bool bank, master, stalled, fail_status_once, settling, no_xlda;
	uint64_t now;
	unsigned bus_calls, starts, stops, settle_waits, cleanup_errors;
	unsigned main_status_reads, banked_status_reads, payload_reads;
	unsigned unexpected_i2c, protocol_errors, nack_start;
	unsigned fail_call, fail_second_call;
	bool fail_applied;
	struct bus_call calls[256];
	struct transaction transactions[32];
} hw;
static struct spi_dt_spec imu_spi;

static const uint8_t register_data[15] = {
	0x91, 0x02, 0xe3, 0x44, 0x15, 0xa6, 0x77, 0x38,
	0xc9, 0x5a, 0x0b, 0xdc, 0x6d, 0x2e, 0xff,
};

#define CHECK(condition) do { \
	if (!(condition)) { \
		fprintf(stderr, "%s:%d: check failed: %s\n", __func__, __LINE__, #condition); \
		return false; \
	} \
} while (0)

int64_t host_k_uptime_get(void) { return (int64_t)(hw.now / 1000); }
void host_k_busy_wait(uint32_t us) { hw.now += us; }
void host_k_msleep(int32_t ms) { hw.now += (uint64_t)ms * 1000; }
void host_k_usleep(int32_t us)
{
	hw.now += (uint32_t)us;
	if (hw.settling && us >= 350) {
		hw.settle_waits++;
		hw.settling = false;
	}
}

static int protocol_error(void)
{
	hw.protocol_errors++;
	return -EINVAL;
}

static int start_transaction(void)
{
	if (hw.starts >= ARRAY_SIZE(hw.transactions))
		return protocol_error();
	struct transaction *op = &hw.transactions[hw.starts++];
	*op = (struct transaction){
		.reading = (hw.hub[0x15] & 1) != 0,
		.addr = hw.hub[0x15] >> 1,
		.sub = hw.hub[0x16],
		.config = hw.hub[0x17],
		.data = hw.hub[0x21],
	};
	hw.master = true;
	hw.status = 0;
	if (hw.stalled)
		return 0;
	hw.status = 0x01 | (op->reading ? 0 : 0x80);
	if (hw.nack_start == hw.starts)
		hw.status |= 0x08;
	if (hw.status & 0x08)
		return 0;
	if (op->reading) {
		unsigned length = op->config & 0x07;
		for (unsigned i = 0; i < length; i++)
			hw.hub[0x02 + i] = hw.external[(uint8_t)(op->sub + i)];
	} else {
		hw.external[op->sub] = op->data;
	}
	return 0;
}

static int write_registers(uint8_t reg, const uint8_t *buf, size_t length)
{
	if ((unsigned)reg + length > 256)
		return protocol_error();
	if (reg == 0x01) {
		if (length != 1)
			return protocol_error();
		if (!(buf[0] & 0x40) && hw.settling)
			hw.cleanup_errors++;
		hw.bank = (buf[0] & 0x40) != 0;
		return 0;
	}
	memcpy((hw.bank ? hw.hub : hw.main) + reg, buf, length);
	if (hw.bank && reg == 0x14) {
		if (length != 1)
			return protocol_error();
		if (buf[0] & 0x04)
			return start_transaction();
		if (hw.master) {
			hw.stops++;
			hw.settling = true;
		}
		hw.master = false;
	}
	return 0;
}

static int read_registers(uint8_t reg, uint8_t *buf, size_t length)
{
	if ((unsigned)reg + length > 256)
		return protocol_error();
	if (hw.bank && reg == 0x22) {
		if (length != 1)
			return protocol_error();
		hw.banked_status_reads++;
		if (hw.fail_status_once) {
			hw.fail_status_once = false;
			return -EIO;
		}
		*buf = hw.status;
	} else if (!hw.bank && reg == 0x39) {
		if (length != 1)
			return protocol_error();
		hw.main_status_reads++;
		*buf = hw.status;
	} else if (!hw.bank && reg == 0x1e) {
		if (length != 1)
			return protocol_error();
		*buf = hw.no_xlda ? 0 : 0x01;
	} else {
		if (hw.bank && reg == 0x02)
			hw.payload_reads++;
		memcpy(buf, (hw.bank ? hw.hub : hw.main) + reg, length);
	}
	return 0;
}

int spi_transceive_dt(const struct spi_dt_spec *spec,
	const struct spi_buf_set *tx, const struct spi_buf_set *rx)
{
	hw.bus_calls++;
	hw.now += 100;
	if (spec != &imu_spi || tx == NULL || tx->count == 0 ||
		tx->buffers[0].buf == NULL || tx->buffers[0].len == 0)
		return protocol_error();
	uint8_t command = ((const uint8_t *)tx->buffers[0].buf)[0];
	bool reading = rx != NULL;
	if (hw.bus_calls <= ARRAY_SIZE(hw.calls))
		hw.calls[hw.bus_calls - 1] = (struct bus_call){command & 0x7f, hw.bank, reading};
	bool fail = hw.bus_calls == hw.fail_call || hw.bus_calls == hw.fail_second_call;
	if (fail && (reading || !hw.fail_applied))
		return -EIO;
	if (rx != NULL) {
		if (tx->count != 1 || tx->buffers[0].len != 1 || !(command & 0x80) ||
			rx->count != 2 || rx->buffers[0].len != 1 || rx->buffers[1].buf == NULL)
			return protocol_error();
		if (rx->buffers[0].buf != NULL)
			memset(rx->buffers[0].buf, 0, rx->buffers[0].len);
		return read_registers(command & 0x7f, rx->buffers[1].buf, rx->buffers[1].len);
	}
	if (command & 0x80)
		return protocol_error();
	uint8_t bytes[32];
	size_t length = 0;
	for (size_t i = 0; i < tx->count; i++) {
		if (tx->buffers[i].buf == NULL || tx->buffers[i].len > sizeof(bytes) - length)
			return protocol_error();
		memcpy(bytes + length, tx->buffers[i].buf, tx->buffers[i].len);
		length += tx->buffers[i].len;
	}
	if (length < 2)
		return protocol_error();
	int err = write_registers(bytes[0], bytes + 1, length - 1);
	return fail ? -EIO : err;
}

int i2c_write_dt(const struct i2c_dt_spec *spec, const uint8_t *buf, size_t length)
{
	(void)spec; (void)buf; (void)length;
	hw.unexpected_i2c++;
	return -ENOTSUP;
}
int i2c_read_dt(const struct i2c_dt_spec *spec, uint8_t *buf, size_t length)
{
	(void)spec; (void)buf; (void)length;
	hw.unexpected_i2c++;
	return -ENOTSUP;
}
int i2c_write_read_dt(const struct i2c_dt_spec *spec, const void *write_buf,
	size_t num_write, void *read_buf, size_t num_read)
{
	(void)spec; (void)write_buf; (void)num_write; (void)read_buf; (void)num_read;
	hw.unexpected_i2c++;
	return -ENOTSUP;
}
int i2c_burst_write_dt(const struct i2c_dt_spec *spec, uint8_t reg,
	const uint8_t *buf, size_t length)
{
	(void)spec; (void)reg; (void)buf; (void)length;
	hw.unexpected_i2c++;
	return -ENOTSUP;
}

static bool reset_model(void)
{
	memset(&hw, 0, sizeof(hw));
	memset(&imu_spi, 0, sizeof(imu_spi));
	memset(hw.hub + 0x02, 0xee, 18);
	memcpy(hw.external + 0x30, register_data, sizeof(register_data));
	sensor_interface_register_sensor_imu_spi(&imu_spi);
	CHECK(sensor_interface_spi_configure(SENSOR_INTERFACE_DEV_IMU, 10000000, 0) == 0);
	sensor_interface_ext_configure(NULL);
	CHECK(lsm6dso_ext_setup(SENSOR_EXT_MODE_I2CM_PROXY) == 0);
	CHECK(sensor_interface_register_sensor_mag_ext(0x7c, 1, 8) == 0);
	CHECK(!hw.bank && !hw.master);
	return true;
}

static bool clean_transaction(void)
{
	CHECK(!hw.master && !hw.bank && !hw.settling);
	CHECK(hw.stops == hw.starts && hw.settle_waits == hw.starts);
	CHECK(hw.cleanup_errors == 0 && hw.protocol_errors == 0 && hw.unexpected_i2c == 0);
	return true;
}

static int direct_read(uint8_t *out, size_t length)
{
	uint8_t sub = 0x30;
	return sensor_ext_lsm6dso.ext_write_read(0x7c, &sub, 1, out, length);
}

static bool test_healthy_write(unsigned unused)
{
	CHECK(reset_model());
	(void)unused;
	const uint8_t command[] = {0x50, 0x73};
	CHECK(ssi_write(SENSOR_INTERFACE_DEV_MAG, command, sizeof(command)) == 0);
	CHECK(hw.external[0x50] == 0x73 && hw.starts == 1);
	CHECK(!hw.transactions[0].reading && hw.transactions[0].addr == 0x7c);
	CHECK(hw.transactions[0].sub == 0x50 && hw.transactions[0].data == 0x73);
	CHECK(hw.banked_status_reads != 0);
	return clean_transaction();
}

static bool test_healthy_read(unsigned length)
{
	CHECK(reset_model());
	uint8_t out[7] = {0};
	CHECK(direct_read(out, length) == 0);
	CHECK(memcmp(out, register_data, length) == 0);
	CHECK(hw.starts == 1 && hw.payload_reads == 1);
	CHECK(hw.transactions[0].reading && hw.transactions[0].addr == 0x7c);
	CHECK(hw.transactions[0].sub == 0x30 && hw.transactions[0].config == length);
	CHECK(hw.main_status_reads != 0 && hw.banked_status_reads == 1);
	return clean_transaction();
}


static bool test_direct_reject(unsigned length)
{
	CHECK(reset_model());
	uint8_t out[9];
	memset(out, 0xa5, sizeof(out));
	unsigned before = hw.bus_calls;
	CHECK(direct_read(out, length) == -1);
	CHECK(hw.bus_calls == before && hw.starts == 0 && out[0] == 0xa5);
	return clean_transaction();
}

static bool test_nack_write(unsigned unused)
{
	(void)unused;
	CHECK(reset_model());
	hw.nack_start = 1;
	const uint8_t command[] = {0x50, 0x73};
	CHECK(ssi_write(SENSOR_INTERFACE_DEV_MAG, command, sizeof(command)) == -1);
	CHECK(hw.external[0x50] == 0 && hw.starts == 1);
	return clean_transaction();
}

static bool test_read_failure(unsigned fault)
{
	CHECK(reset_model());
	if (fault == 0)
		hw.nack_start = 1;
	else if (fault == 1)
		hw.fail_status_once = true;
	else
		hw.stalled = true;
	uint8_t out[7];
	memset(out, 0xa5, sizeof(out));
	CHECK(direct_read(out, sizeof(out)) == (fault == 1 ? -EIO : -1));
	const uint8_t empty[7] = {0};
	CHECK(memcmp(out, empty, sizeof(out)) == 0 && hw.payload_reads == 0);
	CHECK(hw.main_status_reads != 0 && hw.banked_status_reads == (fault == 2 ? 0U : 1U));
	CHECK(!hw.fail_status_once && hw.starts == 1);
	return clean_transaction();
}

static bool test_write_timeout(unsigned unused)
{
	(void)unused;
	CHECK(reset_model());
	hw.stalled = true;
	const uint8_t command[] = {0x50, 0x73};
	CHECK(ssi_write(SENSOR_INTERFACE_DEV_MAG, command, sizeof(command)) == -1);
	CHECK(hw.external[0x50] == 0 && hw.banked_status_reads != 0);
	return clean_transaction();
}

static bool test_segmented_read(unsigned length)
{
	CHECK(reset_model());
	uint8_t out[15];
	memset(out, 0xa5, sizeof(out));
	uint8_t sub = 0x30;
	CHECK(ssi_write_read(SENSOR_INTERFACE_DEV_MAG, &sub, 1, out, length) == 0);
	CHECK(memcmp(out, register_data, length) == 0);
	if (length == 8) {
		CHECK(hw.starts == 2 && hw.payload_reads == 2);
		CHECK(hw.transactions[0].sub == 0x30 && hw.transactions[0].config == 7);
		CHECK(hw.transactions[1].sub == 0x37 && hw.transactions[1].config == 1);
	} else {
		CHECK(hw.starts == 3 && hw.payload_reads == 3);
		CHECK(hw.transactions[0].sub == 0x30 && hw.transactions[0].config == 7);
		CHECK(hw.transactions[1].sub == 0x37 && hw.transactions[1].config == 7);
		CHECK(hw.transactions[2].sub == 0x3e && hw.transactions[2].config == 1);
	}
	return clean_transaction();
}

static bool test_second_chunk_failure(unsigned unused)
{
	(void)unused;
	CHECK(reset_model());
	hw.nack_start = 2;
	uint8_t out[15];
	memset(out, 0xa5, sizeof(out));
	CHECK(ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, 0x30, out, sizeof(out)) == -1);
	CHECK(hw.starts == 2 && hw.payload_reads == 1);
	CHECK(hw.transactions[0].sub == 0x30 && hw.transactions[0].config == 7);
	CHECK(hw.transactions[1].sub == 0x37 && hw.transactions[1].config == 7);
	return clean_transaction();
}

static bool test_qmc_configuration_nack(unsigned failed_write)
{
	CHECK(reset_model());
	qmc_set_variant(false);
	hw.nack_start = failed_write;
	float actual = -123.0f;
	CHECK(qmc_init(0.02f, &actual) == -1);
	CHECK(actual == -123.0f && hw.starts == failed_write);
	hw.nack_start = 0;
	unsigned before = hw.starts;
	CHECK(qmc_update_odr(0.02f, &actual) == 0);
	CHECK(actual == 0.02f && hw.starts == before + 2);
	CHECK(!hw.transactions[before].reading && hw.transactions[before].sub == 0x0b);
	CHECK(!hw.transactions[before + 1].reading && hw.transactions[before + 1].sub == 0x0a);
	before = hw.starts;
	actual = -123.0f;
	CHECK(qmc_update_odr(0.02f, &actual) == 0);
	CHECK(actual == 0.02f && hw.starts == before);
	return clean_transaction();
}

static bool test_qmc_failed_sample(unsigned status_io_error)
{
	CHECK(reset_model());
	qmc_set_variant(false);
	float actual = -123.0f;
	CHECK(qmc_init(0.02f, &actual) == 0 && actual == 0.02f);
	const uint8_t sample[] = {0xa0, 0x0f, 0xc0, 0xe0, 0xd0, 0x07};
	memcpy(hw.external + 0x01, sample, sizeof(sample));
	if (status_io_error)
		hw.fail_status_once = true;
	else
		hw.nack_start = hw.starts + 1;
	float m[3] = {11.0f, 22.0f, 33.0f};
	CHECK(!qmc_mag_read(m));
	CHECK(m[0] == 11.0f && m[1] == 22.0f && m[2] == 33.0f);
	CHECK(hw.payload_reads == 0 && !hw.fail_status_once);
	hw.nack_start = 0;
	CHECK(qmc_mag_read(m));
	CHECK(fabsf(m[0] - 1.0f) < 0.000001f);
	CHECK(fabsf(m[1] + 2.0f) < 0.000001f);
	CHECK(fabsf(m[2] - 0.5f) < 0.000001f);
	CHECK(hw.payload_reads == 1);
	return clean_transaction();
}

static bool test_setup_page_failures(unsigned mode)
{
	const uint8_t sentinel[] = {0x91, 0x82, 0x73, 0x64};
	for (unsigned failure = 1; failure <= 3; failure++) {
		for (unsigned applied = 0; applied < 2; applied++) {
			CHECK(reset_model());
			memcpy(hw.main + LSM6DSO_CTRL5, sentinel, sizeof(sentinel));
			hw.main[LSM6DSO_CTRL1] = 0xa7;
			sensor_interface_ext_configure(NULL);
			hw.fail_call = hw.bus_calls + failure;
			hw.fail_applied = applied;
			CHECK(lsm6dso_ext_setup((enum sensor_ext_mode)mode) == -EIO);
			CHECK(memcmp(hw.main + LSM6DSO_CTRL5, sentinel, sizeof(sentinel)) == 0);
			CHECK(hw.main[LSM6DSO_CTRL1] == 0xa7);
			CHECK(sensor_interface_ext_get() == NULL);
		}
	}
	return true;
}

static bool test_transaction_page_failures(unsigned writing)
{
	const uint8_t sentinel[] = {0x91, 0x82, 0x73, 0x64};
	const uint8_t command[] = {0x50, 0x73};
	uint8_t out[7];
	CHECK(reset_model());
	unsigned first = hw.bus_calls;
	CHECK((writing ? ssi_write(SENSOR_INTERFACE_DEV_MAG, command, sizeof(command))
		       : direct_read(out, sizeof(out))) == 0);
	unsigned count = hw.bus_calls - first;
	CHECK(count > 5 && first + count <= ARRAY_SIZE(hw.calls));
	struct bus_call calls[32];
	CHECK(count <= ARRAY_SIZE(calls));
	memcpy(calls, hw.calls + first, count * sizeof(calls[0]));

	for (unsigned failure = 1; failure <= count; failure++) {
		for (unsigned applied = 0; applied < 2; applied++) {
			CHECK(reset_model());
			memcpy(hw.main + LSM6DSO_CTRL5, sentinel, sizeof(sentinel));
			memset(out, 0xa5, sizeof(out));
			hw.fail_call = hw.bus_calls + failure;
			hw.fail_applied = applied;
			int err = writing ? ssi_write(SENSOR_INTERFACE_DEV_MAG, command, sizeof(command))
					  : direct_read(out, sizeof(out));
			CHECK(err == -EIO);
			CHECK(memcmp(hw.main + LSM6DSO_CTRL5, sentinel, sizeof(sentinel)) == 0);
			if (!writing) {
				const uint8_t empty[7] = {0};
				CHECK(memcmp(out, empty, sizeof(out)) == 0);
			}
			struct bus_call failed = calls[failure - 1];
			if (failed.reading && (failed.reg == LSM6DSO_STATUS_REG ||
					       failed.reg == LSM6DSO_STATUS_MASTER_MAINPAGE ||
					       (failed.bank && failed.reg == LSM6DSO_STATUS_MASTER))) {
				for (unsigned j = hw.fail_call; j < hw.bus_calls && j < ARRAY_SIZE(hw.calls); j++)
					CHECK(!hw.calls[j].reading || hw.calls[j].reg != failed.reg ||
					      hw.calls[j].bank != failed.bank);
			}
		}
	}

	for (unsigned i = 0; i < count; i++) {
		if (calls[i].reading && !calls[i].bank && calls[i].reg == LSM6DSO_STATUS_REG) {
			for (unsigned applied = 0; applied < 2; applied++) {
				CHECK(reset_model());
				memcpy(hw.main + LSM6DSO_CTRL5, sentinel, sizeof(sentinel));
				hw.fail_call = hw.bus_calls + i + 1;
				hw.fail_second_call = hw.fail_call + 1;
				hw.fail_applied = applied;
				CHECK((writing ? ssi_write(SENSOR_INTERFACE_DEV_MAG, command, sizeof(command))
					       : direct_read(out, sizeof(out))) == -EIO);
				CHECK(memcmp(hw.main + LSM6DSO_CTRL5, sentinel, sizeof(sentinel)) == 0);
			}
			return true;
		}
	}
	return false;
}

static bool test_xlda_timeout(unsigned writing)
{
	const uint8_t command[] = {0x50, 0x73};
	uint8_t out[7];
	CHECK(reset_model());
	hw.no_xlda = true;
	uint64_t start = hw.now;
	int err = writing ? ssi_write(SENSOR_INTERFACE_DEV_MAG, command, sizeof(command))
			  : direct_read(out, sizeof(out));
	CHECK(err == -1);
	CHECK(hw.now - start >= 80000 && hw.now - start < 90000);
	CHECK(hw.main_status_reads == 0 && hw.banked_status_reads == 0);
	CHECK(hw.starts == 1);
	return clean_transaction();
}

int main(void)
{
	const struct {
		const char *name;
		bool (*run)(unsigned);
		unsigned argument;
	} cases[] = {
		{"setup off page failures", test_setup_page_failures, SENSOR_EXT_MODE_OFF},
		{"setup passthrough page failures", test_setup_page_failures, SENSOR_EXT_MODE_I2C_PASSTHROUGH},
		{"setup proxy page failures", test_setup_page_failures, SENSOR_EXT_MODE_I2CM_PROXY},
		{"read page and status failures", test_transaction_page_failures, 0},
		{"write page and status failures", test_transaction_page_failures, 1},
		{"read XLDA timeout cleanup", test_xlda_timeout, 0},
		{"write XLDA timeout cleanup", test_xlda_timeout, 1},
		{"healthy write", test_healthy_write, 0},
		{"healthy read 1", test_healthy_read, 1},
		{"healthy read 7", test_healthy_read, 7},
		{"direct read 0 rejected", test_direct_reject, 0},
		{"direct read 8 rejected", test_direct_reject, 8},
		{"write NACK bit 3", test_nack_write, 0},
		{"read NACK bit 3", test_read_failure, 0},
		{"banked status I/O error", test_read_failure, 1},
		{"read timeout cleanup", test_read_failure, 2},
		{"write timeout cleanup", test_write_timeout, 0},
		{"interface read 8 uses 7+1", test_segmented_read, 8},
		{"interface read 15 uses 7+7+1", test_segmented_read, 15},
		{"second chunk failure stops transfer", test_second_chunk_failure, 0},
		{"QMC first configuration write NACK", test_qmc_configuration_nack, 1},
		{"QMC second configuration write NACK", test_qmc_configuration_nack, 2},
		{"QMC sample NACK preserves vector", test_qmc_failed_sample, 0},
		{"QMC status I/O error preserves vector", test_qmc_failed_sample, 1},
	};
	unsigned failures = 0;
	for (size_t i = 0; i < ARRAY_SIZE(cases); i++) {
		bool passed = cases[i].run(cases[i].argument);
		printf("%s %s\n", passed ? "PASS" : "FAIL", cases[i].name);
		if (!passed)
			failures++;
	}
	printf("LSM6DSO: %zu cases, %u failures\n", ARRAY_SIZE(cases), failures);
	return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
