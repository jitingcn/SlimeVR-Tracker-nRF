#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sensor/interface.h"
#include "sensor/mag/ICT153xx.h"
#include "zephyr/logging/log.h"

static const struct device controller = {.name = "mag-i2c"};
static struct i2c_dt_spec mag_bus = {.bus = &controller, .addr = 0x1e};
static struct {
	uint8_t regs[256];
	unsigned identity_reads, fail_identity_reads, writes, reset_writes;
	unsigned first_write_identity_reads, unexpected;
	int64_t now;
} hw;

#define CHECK(condition) do { \
	if (!(condition)) { \
		fprintf(stderr, "%s:%d: check failed: %s\n", __func__, __LINE__, #condition); \
		return 1; \
	} \
} while (0)

int64_t host_k_uptime_get(void) { return hw.now; }
void host_k_busy_wait(uint32_t us) { hw.now += us / 1000; }
void host_k_msleep(int32_t ms) { hw.now += ms; }
void host_k_usleep(int32_t us) { hw.now += us / 1000; }

int i2c_write_dt(const struct i2c_dt_spec *spec, const uint8_t *buf, size_t length)
{
	if (spec != &mag_bus || length != 2) {
		hw.unexpected++;
		return -EINVAL;
	}
	if (hw.writes++ == 0)
		hw.first_write_identity_reads = hw.identity_reads;
	hw.regs[buf[0]] = buf[1];
	if (buf[0] == 0x04)
		hw.regs[0x05] = buf[1] & 0x03;
	if (buf[0] == 0x7c && buf[1] == 0x80)
		hw.reset_writes++;
	return 0;
}

int i2c_write_read_dt(const struct i2c_dt_spec *spec, const void *write_buf,
	size_t num_write, void *read_buf, size_t num_read)
{
	if (spec != &mag_bus || num_write != 1) {
		hw.unexpected++;
		return -EINVAL;
	}
	uint8_t reg = *(const uint8_t *)write_buf;
	if ((size_t)reg + num_read > sizeof(hw.regs)) {
		hw.unexpected++;
		return -EINVAL;
	}
	if (reg == 0x00 && num_read == 2) {
		hw.identity_reads++;
		if (hw.identity_reads <= hw.fail_identity_reads)
			return hw.identity_reads == 1 ? -ENXIO : -EIO;
	}
	memcpy(read_buf, hw.regs + reg, num_read);
	return 0;
}

int i2c_read_dt(const struct i2c_dt_spec *spec, uint8_t *buf, size_t length)
{
	(void)spec; (void)buf; (void)length;
	hw.unexpected++;
	return -ENOTSUP;
}
int i2c_burst_write_dt(const struct i2c_dt_spec *spec, uint8_t reg,
	const uint8_t *buf, size_t length)
{
	(void)spec; (void)reg; (void)buf; (void)length;
	hw.unexpected++;
	return -ENOTSUP;
}
int spi_transceive_dt(const struct spi_dt_spec *spec, const struct spi_buf_set *tx,
	const struct spi_buf_set *rx)
{
	(void)spec; (void)tx; (void)rx;
	hw.unexpected++;
	return -ENOTSUP;
}

static void reset_model(unsigned failures)
{
	memset(&hw, 0, sizeof(hw));
	hw.regs[0] = 0xe7;
	hw.regs[1] = 0x45;
	hw.fail_identity_reads = failures;
	host_log_reset();
	sensor_interface_register_sensor_mag_i2c(&mag_bus);
}

static int test_identity(unsigned failures)
{
	reset_model(failures);
	float actual = -123.0f;
	int result = sensor_mag_ict153xx.init(1.0f / 200, &actual);
	CHECK(hw.identity_reads == (failures ? 2U : 1U));
	CHECK(hw.unexpected == 0);
	if (failures == 2) {
		CHECK(result == -EIO && actual == -123.0f);
		CHECK(hw.writes == 0 && hw.reset_writes == 0);
		CHECK(host_log_matches_error("burst_read", "dev=MAG", "bus=I2C",
			"controller=mag-i2c", "addr=0x1e", "reg=0x00", -EIO));
	} else {
		CHECK(result == 0 && fabsf(actual - 1.0f / 200) < 1e-7f);
		CHECK(hw.first_write_identity_reads == hw.identity_reads && hw.writes == 5);
		CHECK(hw.reset_writes == 1 && hw.regs[0x7f] == 0 && hw.regs[0x04] == 0x61);
		CHECK(host_log_error_count() == 0);
	}
	return 0;
}

int main(void)
{
	CHECK(test_identity(0) == 0);
	CHECK(test_identity(1) == 0);
	CHECK(test_identity(2) == 0);
	puts("ICT153xx production SSI identity regressions passed");
	return 0;
}
