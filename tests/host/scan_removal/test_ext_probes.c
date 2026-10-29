#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sensor/interface_internal.h"
#include "sensor/mag/ICT153xx.h"
#include "sensor/scan_ext.h"
#include "sensor/sensors_enum.h"
#include <zephyr/logging/log.h>

#if defined(PROBE_DSO)
#include "sensor/imu/LSM6DSO.h"
#define main existing_driver_tests_main
#include "../sensor_drivers/test_lsm6dso.c"
#undef main
static unsigned probe_gyro_losses;
void sensor_gyro_report_discontinuity(void) { probe_gyro_losses++; }
#else
#if defined(PROBE_DSV)
#include "sensor/imu/LSM6DSV.h"
#else
#include "sensor/imu/ICM45686.h"
#endif

/* Reuse the existing sensor model as the SPI peripheral, not as the SSI layer. */
#define ssi_burst_read model_ssi_burst_read
#define ssi_burst_write model_ssi_burst_write
#define ssi_reg_read_byte model_ssi_reg_read_byte
#define ssi_reg_write_byte model_ssi_reg_write_byte
#define ssi_reg_update_byte model_ssi_reg_update_byte
#define ssi_burst_read_interval model_ssi_burst_read_interval
#define ssi_burst_read_io model_ssi_burst_read_io
#define ssi_burst_write_io model_ssi_burst_write_io
#define ssi_reg_read_byte_io model_ssi_reg_read_byte_io
#define ssi_reg_write_byte_io model_ssi_reg_write_byte_io
#define sensor_interface_spi_configure model_sensor_interface_spi_configure
#define sensor_interface_ext_configure model_sensor_interface_ext_configure
#define sensor_interface_ext_get model_sensor_interface_ext_get
#define main existing_driver_tests_main
#if defined(PROBE_DSV)
#include "../sensor_drivers/test_lsm6dsv.c"
#else
#include "../sensor_drivers/test_icm45686.c"
#endif
#undef main
#undef sensor_interface_ext_get
#undef sensor_interface_ext_configure
#undef sensor_interface_spi_configure
#undef ssi_reg_write_byte_io
#undef ssi_reg_read_byte_io
#undef ssi_burst_write_io
#undef ssi_burst_read_io
#undef ssi_burst_read_interval
#undef ssi_reg_update_byte
#undef ssi_reg_write_byte
#undef ssi_reg_read_byte
#undef ssi_burst_write
#undef ssi_burst_read

static struct spi_dt_spec imu_spi;

int spi_transceive_dt(const struct spi_dt_spec *spec, const struct spi_buf_set *tx,
	const struct spi_buf_set *rx)
{
	assert(spec == &imu_spi && tx && tx->count && tx->buffers[0].len);
	const uint8_t *first = tx->buffers[0].buf;
	uint8_t reg = first[0];
	if (rx) {
		assert(tx->count == 1 && tx->buffers[0].len == 1 && (reg & 0x80));
		assert(rx->count == 2 && rx->buffers[0].len == 1);
		uint8_t *out = rx->buffers[1].buf;
		size_t len = rx->buffers[1].len;
		reg &= 0x7f;
		return len == 1 ? model_ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, reg, out)
			: model_ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, reg, out, len);
	}
	assert(!(reg & 0x80));
	const uint8_t *data;
	size_t len;
	if (tx->count == 1) {
		assert(tx->buffers[0].len >= 2);
		data = first + 1;
		len = tx->buffers[0].len - 1;
	} else {
		assert(tx->count == 2 && tx->buffers[0].len == 1);
		data = tx->buffers[1].buf;
		len = tx->buffers[1].len;
	}
	return len == 1 ? model_ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, reg, data[0])
		: model_ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, reg, data, len);
}

int i2c_write_dt(const struct i2c_dt_spec *spec, const uint8_t *buf, size_t len)
{
	(void)spec; (void)buf; (void)len;
	assert(false);
	return -ENOTSUP;
}
int i2c_read_dt(const struct i2c_dt_spec *spec, uint8_t *buf, size_t len)
{
	(void)spec; (void)buf; (void)len;
	assert(false);
	return -ENOTSUP;
}
int i2c_write_read_dt(const struct i2c_dt_spec *spec, const void *wr, size_t nw,
	void *rd, size_t nr)
{
	(void)spec; (void)wr; (void)nw; (void)rd; (void)nr;
	assert(false);
	return -ENOTSUP;
}
int i2c_burst_write_dt(const struct i2c_dt_spec *spec, uint8_t reg,
	const uint8_t *buf, size_t len)
{
	(void)spec; (void)reg; (void)buf; (void)len;
	assert(false);
	return -ENOTSUP;
}
#endif

static const struct device spi_controller = {.name = "spi-test"};

struct probe_call {
	uint8_t addr, reg;
	int result;
};
static struct probe_call calls[8];
static unsigned call_count;
static const sensor_ext_ssi_t *real_ext;
static bool nack_first_probe;

static void set_nack(void);
static void clear_nack(void);

static int trace_read(uint8_t addr, const void *write_buf, size_t num_write,
	void *read_buf, size_t num_read)
{
	assert(call_count < sizeof(calls) / sizeof(calls[0]));
	assert(num_write == 1 && num_read == 1);
	struct probe_call *call = &calls[call_count++];
	call->addr = addr;
	call->reg = *(const uint8_t *)write_buf;
	bool first_nack = nack_first_probe && call_count == 1;
	if (first_nack)
		set_nack();
	call->result = real_ext->ext_write_read(addr, write_buf, num_write, read_buf, num_read);
	if (first_nack)
		clear_nack();
	return call->result;
}

static void prepare(void)
{
#if defined(PROBE_DSO)
	assert(reset_model());
	imu_spi.bus = &spi_controller;
	imu_spi.config.slave = 3;
	probe_gyro_losses = 0;
#else
	memset(&hw, 0, sizeof(hw));
	memset(&imu_spi, 0, sizeof(imu_spi));
	imu_spi.bus = &spi_controller;
	imu_spi.config.slave = 3;
	sensor_interface_register_sensor_imu_spi(&imu_spi);
#if defined(PROBE_DSV)
	hw.latency = 100;
	hw.registers[LSM6DSV_WHO_AM_I] = 0x70;
	assert(lsm_ext_setup(SENSOR_EXT_MODE_I2CM_PROXY) == 0);
#else
	hw.now = 100000;
	hw.latency = 1;
	hw.duration = 80;
	hw.host[0x72] = 0xe9;
	hw.host[0x20] = 0x20;
	hw.fifo_capacity = ARRAY_SIZE(hw.fifo);
	assert(icm45_ext_setup(SENSOR_EXT_MODE_I2CM_PROXY) == 0);
#endif
#endif
	assert(sensor_interface_register_sensor_mag_ext(ICT153XX_I2C_ADDR, 1, 1) == 0);
	real_ext = sensor_interface_ext_get();
	assert(real_ext && real_ext->ext_write_read);
	hw.external[ICT153XX_MANU_ID] = ICT153XX_MANU_ID_VALUE;
	hw.external[ICT153XX_CHIP_ID] = ICT153XX_CHIP_ID_VALUE;
	host_log_reset();
	call_count = 0;
	nack_first_probe = false;
}

static int scan(uint8_t reg, uint8_t expected, int identity, uint16_t *addr, uint8_t *selected_reg)
{
	const uint8_t addresses[] = {1, ICT153XX_I2C_ADDR};
	const uint8_t registers[] = {1, reg};
	const uint8_t identities[] = {1, expected};
	const int result_ids[] = {identity};
	sensor_ext_ssi_t traced = *real_ext;
	traced.ext_write_read = trace_read;
	*addr = 0;
	*selected_reg = 0xff;
	return sensor_scan_ext(&traced, addr, selected_reg, 1, addresses, registers,
		identities, result_ids);
}

static void inject_nested_failure(void)
{
#if defined(PROBE_DSO)
	hw.fail_call = hw.bus_calls + 1;
#elif defined(PROBE_DSV)
	hw.fail_transaction = hw.transactions + 1;
#else
	fail_on(WRITE, TOP(ICM45686_DEV_PROFILE_0), 0, false);
#endif
}

static unsigned starts(void)
{
#if defined(PROBE_ICM)
	return hw.go_count;
#else
	return hw.starts;
#endif
}

static void expect_quiescent(void)
{
#if defined(PROBE_DSO)
	assert(!hw.master && !hw.bank && !hw.settling);
	assert(hw.cleanup_errors == 0 && hw.protocol_errors == 0 && hw.unexpected_i2c == 0);
	assert(probe_gyro_losses == 0);
#elif defined(PROBE_DSV)
	assert(!hw.master && !hw.bank);
#else
	assert(!hw.busy);
#endif
}

static void set_nack(void)
{
#if defined(PROBE_DSO)
	hw.nack_start = hw.starts + 1;
#elif defined(PROBE_DSV)
	hw.error = NACK;
#else
	hw.nack = 1;
#endif
}

static void clear_nack(void)
{
#if defined(PROBE_DSO)
	hw.nack_start = 0;
#elif defined(PROBE_DSV)
	hw.error = 0;
#else
	hw.nack = 0;
#endif
}

static int nack_error(void)
{
#if defined(PROBE_DSO)
	return -1;
#elif defined(PROBE_DSV)
	return -EIO;
#else
	return -ENXIO;
#endif
}

static int nested_error(void)
{
#if defined(PROBE_DSV)
	return -ENXIO;
#else
	return -EIO;
#endif
}

static int timeout_error(void)
{
#if defined(PROBE_DSO)
	return -1;
#else
	return -ETIMEDOUT;
#endif
}

static void expect_one_runtime_error(int err)
{
	char expected[256];
	snprintf(expected, sizeof(expected),
		"SSI reg_read dev=MAG bus=EXT controller=spi-test addr=0x1e reg=0x00 err=%d", err);
	assert(host_log_error_count() == 1);
	const struct host_log_entry *entry = host_log_error(0);
	assert(entry && strcmp(entry->message, expected) == 0);
}

static void test_absent_and_nested_scan(void)
{
	uint16_t addr;
	uint8_t reg;
	prepare();
	set_nack();
	assert(scan(0x03, 0x42, MAG_LIS3MDL, &addr, &reg) == -1);
	assert(addr == 0xff && reg == 0xff && call_count == 1 && starts() == 1);
	assert(calls[0].addr == 0x1e && calls[0].reg == 0x03 && calls[0].result == nack_error());
	assert(host_log_error_count() == 0);
	expect_quiescent();

	prepare();
	inject_nested_failure();
	assert(scan(0x03, 0x42, MAG_LIS3MDL, &addr, &reg) == -1);
	assert(addr == 0xff && reg == 0xff && call_count == 1 && starts() == 0);
	assert(calls[0].result == nested_error() && host_log_error_count() == 0);
	expect_quiescent();
}

static void test_ict_probe_retry(void)
{
	uint16_t addr;
	uint8_t reg;
	prepare();
	assert(scan(ICT153XX_MANU_ID, ICT153XX_MANU_ID_VALUE, MAG_ICT153XX, &addr, &reg)
		== MAG_ICT153XX);
	assert(addr == 0x1e && reg == 0x00 && call_count == 3 && starts() == 3);
	assert(calls[0].reg == 0x00 && calls[1].reg == 0x01 && calls[2].reg == 0x00);
	assert(calls[0].result == 0 && calls[1].result == 0 && calls[2].result == 0);
	assert(host_log_error_count() == 0);
	expect_quiescent();

	prepare();
	nack_first_probe = true;
	assert(scan(ICT153XX_MANU_ID, ICT153XX_MANU_ID_VALUE, MAG_ICT153XX, &addr, &reg)
		== MAG_ICT153XX);
	assert(addr == 0x1e && reg == 0x00 && call_count == 4 && starts() == 4);
	assert(calls[0].reg == 0x00 && calls[0].result == nack_error());
	assert(calls[1].reg == 0x00 && calls[1].result == 0);
	assert(calls[2].reg == 0x01 && calls[2].result == 0);
	assert(calls[3].reg == 0x00 && calls[3].result == 0);
	assert(host_log_error_count() == 0);
	expect_quiescent();
}

static void test_runtime_failures(void)
{
	uint8_t id;
	prepare();
	inject_nested_failure();
	id = 0xa5;
	assert(ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, 0, &id) == nested_error());
	assert(starts() == 0);
	expect_one_runtime_error(nested_error());
	expect_quiescent();
	assert(ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, 0, &id) == 0 && id == 0xe7);
	assert(starts() == 1 && host_log_error_count() == 1);
	expect_quiescent();

	prepare();
	set_nack();
	id = 0xa5;
	assert(ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, 0, &id) == nack_error());
	assert(starts() == 1);
	expect_one_runtime_error(nack_error());
	expect_quiescent();
	clear_nack();
	assert(ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, 0, &id) == 0 && id == 0xe7);
	assert(starts() == 2 && host_log_error_count() == 1);
	expect_quiescent();

	prepare();
#if defined(PROBE_ICM)
	hw.stuck = true;
#else
	hw.stalled = true;
#endif
	id = 0xa5;
	assert(ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, 0, &id) == timeout_error());
	assert(starts() == 1);
	expect_one_runtime_error(timeout_error());
#if defined(PROBE_ICM)
	assert(hw.busy);
	hw.stuck = false;
	advance(hw.duration);
#else
	expect_quiescent();
	hw.stalled = false;
#endif
	assert(ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, 0, &id) == 0 && id == 0xe7);
	assert(starts() == 2 && host_log_error_count() == 1);
	expect_quiescent();
}

int main(void)
{
	test_absent_and_nested_scan();
	test_ict_probe_retry();
	test_runtime_failures();
	puts("external scanner and SSI hub diagnostics passed");
	return 0;
}
