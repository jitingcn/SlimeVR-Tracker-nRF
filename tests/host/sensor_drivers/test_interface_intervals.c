#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef HOST_SENSOR_DRIVER_TEST
#include "zephyr/drivers/i2c.h"
#include "zephyr/drivers/spi.h"
#include "sensor/interface.h"

#else
#include "sensor/interface.h"
#endif

static int i2c_write_calls;
static int i2c_read_calls;
static int i2c_write_read_calls;
static int fail_i2c_read_call;
static int fail_i2c_write_read_call;
static uint8_t ext_read_regs[16];
static size_t ext_read_lengths[16];
static int ext_read_calls;
static uint8_t ext_seen_dummy;

#define CHECK(condition) do { \
	if (!(condition)) { \
		fprintf(stderr, "%s:%d: check failed: %s\n", __func__, __LINE__, #condition); \
		return EXIT_FAILURE; \
	} \
} while (0)

int64_t host_k_uptime_get(void)
{
	return 0;
}

void host_k_busy_wait(uint32_t usec)
{
	(void)usec;
}

void host_k_msleep(int32_t msec)
{
	(void)msec;
}

#ifdef HOST_SENSOR_DRIVER_TEST
int spi_transceive_dt(
	const struct spi_dt_spec *spec,
	const struct spi_buf_set *tx,
	const struct spi_buf_set *rx
)
{
	(void)spec;
	(void)tx;
	(void)rx;
	return 0;
}

int i2c_write_dt(const struct i2c_dt_spec *spec, const uint8_t *buf, size_t num_bytes)
{
	(void)spec;
	(void)buf;
	(void)num_bytes;
	i2c_write_calls++;
	return 0;
}

int i2c_read_dt(const struct i2c_dt_spec *spec, uint8_t *buf, size_t num_bytes)
{
	(void)spec;
	i2c_read_calls++;
	memset(buf, 0, num_bytes);
	if (fail_i2c_read_call == i2c_read_calls)
		return -EIO;
	return 0;
}

int i2c_write_read_dt(
	const struct i2c_dt_spec *spec,
	const void *write_buf,
	size_t num_write,
	void *read_buf,
	size_t num_read
)
{
	(void)spec;
	(void)write_buf;
	(void)num_write;
	i2c_write_read_calls++;
	memset(read_buf, 0, num_read);
	if (fail_i2c_write_read_call == i2c_write_read_calls)
		return -EIO;
	return 0;
}

int i2c_burst_write_dt(
	const struct i2c_dt_spec *spec,
	uint8_t start_addr,
	const uint8_t *buf,
	size_t num_bytes
)
{
	(void)spec;
	(void)start_addr;
	(void)buf;
	(void)num_bytes;
	return 0;
}
#endif

static int ext_write(uint8_t addr, const uint8_t *buf, uint32_t num_bytes)
{
	(void)addr;
	(void)buf;
	(void)num_bytes;
	return 0;
}

static int ext_write_read(
	uint8_t addr,
	const void *write_buf,
	size_t num_write,
	void *read_buf,
	size_t num_read
)
{
	(void)addr;
	CHECK(num_write == 1);
	CHECK(ext_read_calls < (int)(sizeof(ext_read_regs) / sizeof(ext_read_regs[0])));
	uint8_t reg = *(const uint8_t *)write_buf;
	ext_read_regs[ext_read_calls] = reg;
	ext_read_lengths[ext_read_calls] = num_read;
	ext_read_calls++;
	/* Model the IMU I2CM: the first ext_seen_dummy bytes of any read are a
	 * dummy prefix the interface layer must strip; the remaining bytes are
	 * register data starting at the requested register. */
	uint8_t *out = read_buf;
	for (size_t i = 0; i < num_read; i++)
		out[i] = i < ext_seen_dummy ? 0xee : (uint8_t)(reg + (i - ext_seen_dummy));
	return 0;
}

static void reset_counts(void)
{
	i2c_write_calls = 0;
	i2c_read_calls = 0;
	i2c_write_read_calls = 0;
	fail_i2c_read_call = 0;
	fail_i2c_write_read_call = 0;
	ext_read_calls = 0;
	ext_seen_dummy = 0;
	memset(ext_read_regs, 0, sizeof(ext_read_regs));
	memset(ext_read_lengths, 0, sizeof(ext_read_lengths));
}

int main(void)
{
	struct i2c_dt_spec dev = {0};
	sensor_interface_register_sensor_imu_i2c(&dev);
	uint8_t buffer[4096];

	reset_counts();
	CHECK(ssi_reg_read_interval(SENSOR_INTERFACE_DEV_IMU, 0x10, buffer, 16, 0) == -EINVAL);
	CHECK(i2c_write_calls == 0 && i2c_read_calls == 0);

	reset_counts();
	CHECK(ssi_reg_read_interval(SENSOR_INTERFACE_DEV_IMU, 0x10, buffer, 16, 4096) == -EINVAL);
	CHECK(i2c_write_calls == 0 && i2c_read_calls == 0);

	reset_counts();
	fail_i2c_read_call = 2;
	CHECK(ssi_reg_read_interval(SENSOR_INTERFACE_DEV_IMU, 0x10, buffer, sizeof(buffer), 8) == -EIO);
	CHECK(i2c_write_calls == 1);
	CHECK(i2c_read_calls == 2);

	reset_counts();
	fail_i2c_write_read_call = 2;
	CHECK(ssi_burst_read_interval(SENSOR_INTERFACE_DEV_IMU, 0x10, buffer, 3000, 6) == -EIO);
	CHECK(i2c_write_read_calls == 2);


	sensor_interface_register_sensor_imu_spi(NULL);
	const sensor_ext_ssi_t ext = {
		.ext_write = ext_write,
		.ext_write_read = ext_write_read,
		.ext_burst = 8,
	};
	sensor_interface_ext_configure(&ext);
	CHECK(sensor_interface_register_sensor_mag_ext(0x14, 3, 8) == 0);

	/* Dummy-prefixed burst read segmented across the I2CM width (8): the
	 * interface must advance the sub-register by data bytes and strip the
	 * per-chunk dummy prefix, producing contiguous register data. */
	reset_counts();
	ext_seen_dummy = 2;
	uint8_t dummy_data[9] = {0};
	CHECK(ssi_burst_read_dummy(SENSOR_INTERFACE_DEV_MAG, 0x31, 2, dummy_data, sizeof(dummy_data)) == 0);
	CHECK(ext_read_calls == 2);
	CHECK(ext_read_regs[0] == 0x31 && ext_read_lengths[0] == 8);
	CHECK(ext_read_regs[1] == 0x37 && ext_read_lengths[1] == 5);
	for (size_t i = 0; i < sizeof(dummy_data); i++)
		CHECK(dummy_data[i] == (uint8_t)(0x31 + i));

	/* Plain register burst read longer than one I2CM transaction: segmented
	 * by register-data length with the sub-register advanced per data chunk. */
	reset_counts();
	uint8_t long_data[13] = {0};
	CHECK(ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, 0x10, long_data, sizeof(long_data)) == 0);
	CHECK(ext_read_calls == 2);
	CHECK(ext_read_regs[0] == 0x10 && ext_read_lengths[0] == 8);
	CHECK(ext_read_regs[1] == 0x18 && ext_read_lengths[1] == 5);
	for (size_t i = 0; i < sizeof(long_data); i++)
		CHECK(long_data[i] == (uint8_t)(0x10 + i));

	/* A read within one transaction must not be segmented. */
	reset_counts();
	uint8_t short_data[8] = {0};
	CHECK(ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, 0x20, short_data, sizeof(short_data)) == 0);
	CHECK(ext_read_calls == 1);
	CHECK(ext_read_regs[0] == 0x20 && ext_read_lengths[0] == 8);

	/* Dummy prefix wider than the read budget is unsupported. */
	reset_counts();
	uint8_t tiny[4] = {0};
	CHECK(ssi_burst_read_dummy(SENSOR_INTERFACE_DEV_MAG, 0x40, 8, tiny, sizeof(tiny)) != 0);
	CHECK(ext_read_calls == 0);

	printf("All interface interval tests passed\n");
	return EXIT_SUCCESS;
}
