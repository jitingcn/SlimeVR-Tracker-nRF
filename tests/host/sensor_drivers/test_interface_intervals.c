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
#include "sensor/interface_internal.h"
#include "zephyr/logging/log.h"

#else
#include "sensor/interface.h"
#endif

enum transfer_kind { I2C_WRITE, I2C_READ, I2C_WRITE_READ, I2C_BURST_WRITE };
struct transfer {
	enum transfer_kind kind;
	uint8_t reg, value;
	size_t length;
};

static struct transfer transfers[16];
static size_t transfer_count;
static int i2c_write_calls;
static int i2c_read_calls;
static int i2c_write_read_calls;
static int i2c_burst_write_calls;
static int fail_i2c_write_call;
static int fail_i2c_read_call;
static int fail_i2c_write_read_call;
static int fail_i2c_burst_write_call;
static uint8_t i2c_read_value;
static uint8_t ext_read_regs[16];
static size_t ext_read_lengths[16];
static int ext_read_calls, fail_ext_read_call, fail_ext_write;
static unsigned ext_write_calls;
static uint8_t ext_write_addr, ext_write_reg, ext_write_value;
static uint32_t ext_write_length;
static uint8_t ext_seen_dummy;
static uint8_t ext_registers[256], ext_cache[8], ext_cache_reg;
static size_t ext_capacity = 8, ext_cache_len;
static bool ext_prefetch;
static int ext_prefetch_error;

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
	if (transfer_count < sizeof(transfers) / sizeof(transfers[0]))
		transfers[transfer_count++] = (struct transfer){
			I2C_WRITE, num_bytes ? buf[0] : 0, num_bytes > 1 ? buf[1] : 0, num_bytes,
		};
	i2c_write_calls++;
	if (fail_i2c_write_call == i2c_write_calls)
		return -EIO;
	return 0;
}

int i2c_read_dt(const struct i2c_dt_spec *spec, uint8_t *buf, size_t num_bytes)
{
	(void)spec;
	i2c_read_calls++;
	if (transfer_count < sizeof(transfers) / sizeof(transfers[0]))
		transfers[transfer_count++] = (struct transfer){I2C_READ, 0, 0, num_bytes};
	if (fail_i2c_read_call == i2c_read_calls)
		return -EIO;
	memset(buf, i2c_read_value, num_bytes);
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
	i2c_write_read_calls++;
	if (transfer_count < sizeof(transfers) / sizeof(transfers[0]))
		transfers[transfer_count++] = (struct transfer){
			I2C_WRITE_READ, num_write ? *(const uint8_t *)write_buf : 0, 0, num_read,
		};
	if (fail_i2c_write_read_call == i2c_write_read_calls)
		return -EIO;
	memset(read_buf, i2c_read_value, num_read);
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
	if (transfer_count < sizeof(transfers) / sizeof(transfers[0]))
		transfers[transfer_count++] = (struct transfer){
			I2C_BURST_WRITE, start_addr, num_bytes ? buf[0] : 0, num_bytes,
		};
	i2c_burst_write_calls++;
	return fail_i2c_burst_write_call == i2c_burst_write_calls ? -EIO : 0;
}
#endif

static int ext_write(uint8_t addr, const uint8_t *buf, uint32_t num_bytes)
{
	ext_write_calls++;
	ext_write_addr = addr;
	ext_write_length = num_bytes;
	ext_write_reg = num_bytes ? buf[0] : 0;
	ext_write_value = num_bytes > 1 ? buf[1] : 0;
	return fail_ext_write ? -EIO : 0;
}

static int read_external(
	const void *write_buf,
	size_t num_write,
	void *read_buf,
	size_t num_read,
	bool cached
)
{
	CHECK(num_write == 1);
	if (num_read > ext_capacity)
		return -EMSGSIZE; /* Hardware transaction width, independent of SSI. */
	if (num_read == 0 || num_read > sizeof(ext_cache))
		return -EINVAL;
	CHECK(ext_read_calls < (int)(sizeof(ext_read_regs) / sizeof(ext_read_regs[0])));
	uint8_t reg = *(const uint8_t *)write_buf;
	ext_read_regs[ext_read_calls] = reg;
	ext_read_lengths[ext_read_calls] = num_read;
	ext_read_calls++;
	if (fail_ext_read_call == ext_read_calls)
		return -EIO;
	/* Model the IMU I2CM: the first ext_seen_dummy bytes of any read are a
	 * dummy prefix the interface layer must strip; the remaining bytes are
	 * register data starting at the requested register. */
	uint8_t *out = read_buf;
	if (cached && ext_cache_len == num_read && ext_cache_reg == reg) {
		memcpy(out, ext_cache, num_read);
		return 0;
	}
	for (size_t i = 0; i < num_read; i++)
		out[i] = i < ext_seen_dummy ? 0xee : ext_registers[(uint8_t)(reg + i - ext_seen_dummy)];
	if (cached) {
		memcpy(ext_cache, out, num_read);
		ext_cache_len = num_read;
		ext_cache_reg = reg;
	}
	return 0;
}

static int ext_write_read(uint8_t addr, const void *write_buf, size_t num_write,
	void *read_buf, size_t num_read)
{
	(void)addr;
	return read_external(write_buf, num_write, read_buf, num_read, ext_prefetch);
}

static int synchronous_write_read(uint8_t addr, const void *write_buf, size_t num_write,
	void *read_buf, size_t num_read)
{
	(void)addr;
	return read_external(write_buf, num_write, read_buf, num_read, false);
}

static int ext_set_prefetch(bool enabled)
{
	if (ext_prefetch_error)
		return ext_prefetch_error;
	ext_prefetch = enabled;
	ext_cache_len = 0;
	return 0;
}

static void reset_counts(void)
{
	i2c_write_calls = 0;
	i2c_read_calls = 0;
	i2c_write_read_calls = 0;
	i2c_burst_write_calls = 0;
	fail_i2c_write_call = 0;
	fail_i2c_read_call = 0;
	fail_i2c_write_read_call = 0;
	fail_i2c_burst_write_call = 0;
	i2c_read_value = 0;
	transfer_count = 0;
	ext_read_calls = 0;
	fail_ext_read_call = 0;
	ext_write_calls = 0;
	fail_ext_write = 0;
	ext_seen_dummy = 0;
	ext_capacity = 8;
	ext_cache_len = 0;
	ext_prefetch = false;
	ext_prefetch_error = 0;
	host_log_reset();
	for (unsigned i = 0; i < sizeof(ext_registers); i++)
		ext_registers[i] = (uint8_t)i;
	memset(ext_read_regs, 0, sizeof(ext_read_regs));
	memset(ext_read_lengths, 0, sizeof(ext_read_lengths));
}

static int test_seven_byte_frame_and_prefetch(void)
{
	const sensor_ext_ssi_t proxy = {
		.ext_write = ext_write,
		.ext_write_read = ext_write_read,
		.ext_burst = 7,
		.ext_set_prefetch = ext_set_prefetch,
	};
	const sensor_ext_ssi_t synchronous = {
		.ext_write = ext_write,
		.ext_write_read = synchronous_write_read,
		.ext_burst = 7,
		.ext_set_prefetch = NULL,
	};
	reset_counts();
	ext_capacity = 7;
	ext_prefetch = true;
	sensor_interface_ext_configure(&proxy);
	CHECK(sensor_interface_register_sensor_mag_ext(0x14, 3, 8) == 0);
	const uint8_t frame[] = {0x60, 0xff, 0x90, 0x01, 0xe0, 0xfc, 0xb0, 0x04};
	memcpy(ext_registers + 0x08, frame, sizeof(frame));
	uint8_t result[sizeof(frame)] = {0};
	CHECK(ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, 0x08, result, sizeof(result)) == 0);
	CHECK(memcmp(result, frame, sizeof(frame)) == 0);

	/* A same-shape cached read hides later register changes until disabled. */
	CHECK(ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, 0x08, result, 3) == 0);
	ext_registers[0x08] = 0x19;
	CHECK(ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, 0x08, result, 3) == 0);
	CHECK(result[0] == frame[0]);
	ext_prefetch_error = -EBUSY;
	CHECK(sensor_interface_ext_set_prefetch(false) == -EBUSY);
	CHECK(host_log_matches_error("prefetch", "dev=MAG", "bus=EXT", "controller=imu-spi",
		0x14, -1, -EBUSY));
	host_log_reset();
	CHECK(ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, 0x08, result, 3) == 0);
	CHECK(result[0] == frame[0]);
	ext_prefetch_error = 0;
	CHECK(sensor_interface_ext_set_prefetch(false) == 0);
	CHECK(ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, 0x08, result, 3) == 0);
	CHECK(result[0] == 0x19 && result[1] == frame[1] && result[2] == frame[2]);
	ext_registers[0x08] = 0x27;
	CHECK(ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, 0x08, result, 3) == 0);
	CHECK(result[0] == 0x27);

	CHECK(sensor_interface_ext_set_prefetch(true) == 0);
	CHECK(ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, 0x08, result, 3) == 0);
	ext_registers[0x08] = 0x35;
	/* Direct-bus policy changes must not touch the previously registered proxy. */
	struct i2c_dt_spec direct = {0};
	sensor_interface_register_sensor_mag_i2c(&direct);
	CHECK(sensor_interface_ext_set_prefetch(false) == 0);
	CHECK(sensor_interface_register_sensor_mag_ext(0x14, 3, 8) == 0);
	CHECK(ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, 0x08, result, 3) == 0);
	CHECK(result[0] == 0x27);
	sensor_interface_register_sensor_mag_spi(NULL);
	CHECK(sensor_interface_ext_set_prefetch(false) == 0);
	CHECK(sensor_interface_register_sensor_mag_ext(0x14, 3, 8) == 0);
	CHECK(ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, 0x08, result, 3) == 0);
	CHECK(result[0] == 0x27);

	/* Replacing the backend with a legacy synchronous one must neither retain
	 * the old cache nor dispatch the old callback to alter that backend. */
	sensor_interface_ext_configure(&synchronous);
	CHECK(sensor_interface_ext_set_prefetch(false) == 0);
	CHECK(ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, 0x08, result, 3) == 0);
	CHECK(result[0] == 0x35);
	sensor_interface_ext_configure(&proxy);
	CHECK(ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, 0x08, result, 3) == 0);
	CHECK(result[0] == 0x27);
	CHECK(sensor_interface_ext_set_prefetch(false) == 0);
	CHECK(ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, 0x08, result, sizeof(result)) == 0);
	CHECK(memcmp(result, ext_registers + 0x08, sizeof(result)) == 0);
	sensor_interface_ext_configure(NULL);
	return EXIT_SUCCESS;
}

static int test_i2c_diagnostics(void)
{
	uint8_t value;
	reset_counts();
	i2c_read_value = 0xa0;
	value = 0;
	CHECK(ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, 0x10, &value) == 0);
	CHECK(value == 0xa0 && transfer_count == 1);
	CHECK(transfers[0].kind == I2C_WRITE_READ && transfers[0].reg == 0x10);
	CHECK(ssi_reg_update_byte(SENSOR_INTERFACE_DEV_IMU, 0x10, 0x0f, 0) == 0);
	CHECK(transfer_count == 2 && i2c_write_calls == 0);
	CHECK(transfers[1].kind == I2C_WRITE_READ && transfers[1].reg == 0x10);
	CHECK(ssi_reg_update_byte(SENSOR_INTERFACE_DEV_IMU, 0x11, 0x0f, 0x05) == 0);
	CHECK(transfer_count == 4 && i2c_write_calls == 1);
	CHECK(transfers[2].kind == I2C_WRITE_READ && transfers[2].reg == 0x11);
	CHECK(transfers[3].kind == I2C_WRITE && transfers[3].reg == 0x11 &&
		transfers[3].value == 0xa5 && transfers[3].length == 2);
	CHECK(host_log_error_count() == 0);

	reset_counts();
	fail_i2c_write_read_call = 1;
	value = 0x7e;
	CHECK(ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, 0x10, &value) == -EIO);
	CHECK(value == 0x7e && transfer_count == 1 && transfers[0].reg == 0x10);
	CHECK(host_log_matches_error("reg_read", "dev=IMU", "bus=I2C", "controller=imu-i2c",
		0x28, 0x10, -EIO));

	reset_counts();
	fail_i2c_write_call = 1;
	CHECK(ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, 0x11, 0x42) == -EIO);
	CHECK(transfer_count == 1 && transfers[0].kind == I2C_WRITE &&
		transfers[0].reg == 0x11 && transfers[0].value == 0x42);
	CHECK(host_log_matches_error("reg_write", "dev=IMU", "bus=I2C", "controller=imu-i2c",
		0x28, 0x11, -EIO));

	reset_counts();
	fail_i2c_write_read_call = 1;
	CHECK(ssi_reg_update_byte(SENSOR_INTERFACE_DEV_IMU, 0x12, 0x0f, 0x05) == -EIO);
	CHECK(transfer_count == 1 && transfers[0].kind == I2C_WRITE_READ &&
		transfers[0].reg == 0x12 && i2c_write_calls == 0);
	CHECK(host_log_matches_error("reg_update", "dev=IMU", "bus=I2C", "controller=imu-i2c",
		0x28, 0x12, -EIO));

	reset_counts();
	i2c_read_value = 0xa0;
	fail_i2c_write_call = 1;
	CHECK(ssi_reg_update_byte(SENSOR_INTERFACE_DEV_IMU, 0x13, 0x0f, 0x05) == -EIO);
	CHECK(transfer_count == 2 && transfers[0].kind == I2C_WRITE_READ &&
		transfers[0].reg == 0x13 && transfers[1].kind == I2C_WRITE &&
		transfers[1].reg == 0x13 && transfers[1].value == 0xa5);
	CHECK(host_log_matches_error("reg_update", "dev=IMU", "bus=I2C", "controller=imu-i2c",
		0x28, 0x13, -EIO));

	reset_counts();
	const uint8_t bytes[] = {0x42, 0x43};
	fail_i2c_burst_write_call = 1;
	CHECK(ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, 0x15, bytes, sizeof(bytes)) == -EIO);
	CHECK(transfer_count == 1 && transfers[0].kind == I2C_BURST_WRITE &&
		transfers[0].reg == 0x15 && transfers[0].value == 0x42 && transfers[0].length == 2);
	CHECK(host_log_matches_error("burst_write", "dev=IMU", "bus=I2C", "controller=imu-i2c",
		0x28, 0x15, -EIO));

	reset_counts();
	fail_i2c_read_call = 1;
	value = 0x7e;
	CHECK(ssi_read(SENSOR_INTERFACE_DEV_IMU, &value, 1) == -EIO);
	CHECK(value == 0x7e && transfer_count == 1 && transfers[0].kind == I2C_READ);
	CHECK(host_log_matches_error("read", "dev=IMU", "bus=I2C", "controller=imu-i2c",
		0x28, -1, -EIO));

	reset_counts();
	fail_i2c_write_read_call = 1;
	value = 0x7e;
	CHECK(ssi_reg_read_byte_io(SENSOR_INTERFACE_DEV_IMU, 0x20, &value, SSI_UNREPORTED) == -EIO);
	CHECK(value == 0x7e && transfer_count == 1 && transfers[0].reg == 0x20);
	CHECK(host_log_error_count() == 0);

	reset_counts();
	fail_i2c_write_call = 1;
	CHECK(ssi_reg_write_byte_io(SENSOR_INTERFACE_DEV_IMU, 0x21, 0x42, SSI_UNREPORTED) == -EIO);
	CHECK(transfer_count == 1 && transfers[0].kind == I2C_WRITE &&
		transfers[0].reg == 0x21 && transfers[0].value == 0x42);
	CHECK(host_log_error_count() == 0);

	reset_counts();
	fail_i2c_write_read_call = 1;
	value = 0x7e;
	CHECK(ssi_burst_read_io(SENSOR_INTERFACE_DEV_IMU, 0x22, &value, 1, SSI_UNREPORTED) == -EIO);
	CHECK(value == 0x7e && transfer_count == 1 && transfers[0].reg == 0x22);
	CHECK(host_log_error_count() == 0);

	reset_counts();
	fail_i2c_burst_write_call = 1;
	CHECK(ssi_burst_write_io(SENSOR_INTERFACE_DEV_IMU, 0x23, bytes, sizeof(bytes),
		SSI_UNREPORTED) == -EIO);
	CHECK(transfer_count == 1 && transfers[0].kind == I2C_BURST_WRITE &&
		transfers[0].reg == 0x23 && transfers[0].value == 0x42);
	CHECK(host_log_error_count() == 0);

	reset_counts();
	fail_i2c_write_read_call = 1;
	CHECK(ssi_reg_read_byte_io(SENSOR_INTERFACE_DEV_IMU, 0x24, &value, SSI_REPORT) == -EIO);
	CHECK(transfer_count == 1 && transfers[0].reg == 0x24);
	CHECK(host_log_matches_error("reg_read", "dev=IMU", "bus=I2C", "controller=imu-i2c",
		0x28, 0x24, -EIO));
	return EXIT_SUCCESS;
}

static int test_diagnostic_addresses(void)
{
	const struct device i2c_controller = {.name = "physical-i2c"};
	const struct device spi_controller = {.name = "physical-spi"};
	struct i2c_dt_spec i2c = {.bus = &i2c_controller, .addr = 0};
	struct spi_dt_spec spi = {.bus = NULL, .config = {.slave = UINT16_MAX}};
	const sensor_ext_ssi_t ext = {.ext_burst = 8};
	uint8_t value = 0xa5;

	reset_counts();
	CHECK(ssi_reg_read_interval(SENSOR_INTERFACE_DEV_COUNT, 0, &value, 1, 0) == -EINVAL);
	CHECK(host_log_matches_error("reg_read_interval", "dev=unknown", "bus=unknown",
		"controller=unknown", -1, 0, -EINVAL));

	sensor_interface_register_sensor_imu_i2c(&i2c);
	reset_counts();
	CHECK(ssi_reg_read_interval(SENSOR_INTERFACE_DEV_IMU, 0, &value, 1, 0) == -EINVAL);
	CHECK(host_log_matches_error("reg_read_interval", "dev=IMU", "bus=I2C",
		"controller=physical-i2c", 0, 0, -EINVAL));

	i2c.addr = UINT16_MAX;
	i2c.bus = NULL;
	reset_counts();
	CHECK(ssi_reg_read_interval(SENSOR_INTERFACE_DEV_IMU, 0xff, &value, 1, 0) == -EINVAL);
	CHECK(host_log_matches_error("reg_read_interval", "dev=IMU", "bus=I2C",
		"controller=unknown", UINT16_MAX, 0xff, -EINVAL));

	sensor_interface_register_sensor_imu_i2c(NULL);
	reset_counts();
	CHECK(ssi_reg_read_interval(SENSOR_INTERFACE_DEV_IMU, 0, &value, 1, 0) == -EINVAL);
	CHECK(host_log_matches_error("reg_read_interval", "dev=IMU", "bus=I2C",
		"controller=unknown", -1, 0, -EINVAL));

	sensor_interface_register_sensor_imu_spi(&spi);
	reset_counts();
	CHECK(ssi_reg_read_interval(SENSOR_INTERFACE_DEV_IMU, 0xff, &value, 1, 0) == -EINVAL);
	CHECK(host_log_matches_error("reg_read_interval", "dev=IMU", "bus=SPI",
		"controller=unknown", UINT16_MAX, 0xff, -EINVAL));

	sensor_interface_register_sensor_imu_spi(NULL);
	reset_counts();
	CHECK(ssi_reg_read_interval(SENSOR_INTERFACE_DEV_IMU, 0, &value, 1, 0) == -EINVAL);
	CHECK(host_log_matches_error("reg_read_interval", "dev=IMU", "bus=SPI",
		"controller=unknown", -1, 0, -EINVAL));

	spi.bus = &spi_controller;
	sensor_interface_register_sensor_imu_spi(&spi);
	sensor_interface_ext_configure(&ext);
	CHECK(sensor_interface_register_sensor_mag_ext(0, 1, 8) == 0);
	reset_counts();
	CHECK(ssi_reg_read_interval(SENSOR_INTERFACE_DEV_MAG, 0, &value, 1, 0) == -EINVAL);
	CHECK(host_log_matches_error("reg_read_interval", "dev=MAG", "bus=EXT",
		"controller=physical-spi", 0, 0, -EINVAL));

	i2c.bus = &i2c_controller;
	sensor_interface_register_sensor_imu_i2c(&i2c);
	reset_counts();
	CHECK(ssi_reg_read_interval(SENSOR_INTERFACE_DEV_MAG, 0xff, &value, 1, 0) == -EINVAL);
	CHECK(host_log_matches_error("reg_read_interval", "dev=MAG", "bus=EXT",
		"controller=physical-i2c", 0, 0xff, -EINVAL));

	i2c.bus = NULL;
	reset_counts();
	CHECK(ssi_reg_read_interval(SENSOR_INTERFACE_DEV_MAG, 0, &value, 1, 0) == -EINVAL);
	CHECK(host_log_matches_error("reg_read_interval", "dev=MAG", "bus=EXT",
		"controller=unknown", 0, 0, -EINVAL));

	sensor_interface_register_sensor_imu_i2c(NULL);
	reset_counts();
	CHECK(ssi_reg_read_interval(SENSOR_INTERFACE_DEV_MAG, 0, &value, 1, 0) == -EINVAL);
	CHECK(host_log_matches_error("reg_read_interval", "dev=MAG", "bus=EXT",
		"controller=unknown", 0, 0, -EINVAL));

	sensor_interface_register_sensor_imu_spi(NULL);
	reset_counts();
	CHECK(ssi_reg_read_interval(SENSOR_INTERFACE_DEV_MAG, 0, &value, 1, 0) == -EINVAL);
	CHECK(host_log_matches_error("reg_read_interval", "dev=MAG", "bus=EXT",
		"controller=unknown", 0, 0, -EINVAL));

	sensor_interface_register_sensor_mag_i2c(NULL);
	sensor_interface_ext_configure(NULL);
	return EXIT_SUCCESS;
}

int main(void)
{
	const struct device i2c_controller = {.name = "imu-i2c"};
	const struct device spi_controller = {.name = "imu-spi"};
	struct i2c_dt_spec dev = {.bus = &i2c_controller, .addr = 0x28};
	struct spi_dt_spec spi = {.bus = &spi_controller, .config = {.slave = 5}};
	CHECK(test_diagnostic_addresses() == EXIT_SUCCESS);
	sensor_interface_register_sensor_imu_i2c(&dev);
	uint8_t buffer[4096];
	CHECK(test_i2c_diagnostics() == EXIT_SUCCESS);

	reset_counts();
	CHECK(ssi_reg_read_interval(SENSOR_INTERFACE_DEV_IMU, 0x10, buffer, 16, 0) == -EINVAL);
	CHECK(i2c_write_calls == 0 && i2c_read_calls == 0);
	CHECK(host_log_matches_error("reg_read_interval", "dev=IMU", "bus=I2C",
		"controller=imu-i2c", 0x28, 0x10, -EINVAL));

	reset_counts();
	CHECK(ssi_reg_read_interval(SENSOR_INTERFACE_DEV_IMU, 0x10, buffer, 16, 4096) == -EINVAL);
	CHECK(i2c_write_calls == 0 && i2c_read_calls == 0);
	CHECK(host_log_matches_error("reg_read_interval", "dev=IMU", "bus=I2C",
		"controller=imu-i2c", 0x28, 0x10, -EINVAL));

	reset_counts();
	fail_i2c_read_call = 2;
	CHECK(ssi_reg_read_interval(SENSOR_INTERFACE_DEV_IMU, 0x10, buffer, sizeof(buffer), 8) == -EIO);
	CHECK(i2c_write_calls == 1);
	CHECK(i2c_read_calls == 2);
	CHECK(transfer_count == 3 && transfers[0].kind == I2C_WRITE &&
		transfers[0].reg == 0x10 && transfers[1].kind == I2C_READ &&
		transfers[1].length == 2048 && transfers[2].kind == I2C_READ &&
		transfers[2].length == 2048);
	CHECK(host_log_matches_error("reg_read_interval", "dev=IMU", "bus=I2C",
		"controller=imu-i2c", 0x28, 0x10, -EIO));

	reset_counts();
	fail_i2c_write_read_call = 2;
	CHECK(ssi_burst_read_interval(SENSOR_INTERFACE_DEV_IMU, 0x10, buffer, 3000, 6) == -EIO);
	CHECK(i2c_write_read_calls == 2);
	CHECK(transfer_count == 2 && transfers[0].kind == I2C_WRITE_READ &&
		transfers[0].reg == 0x10 && transfers[0].length == 1020 &&
		transfers[1].kind == I2C_WRITE_READ && transfers[1].reg == 0x10 &&
		transfers[1].length == 1020);
	CHECK(host_log_matches_error("burst_read_interval", "dev=IMU", "bus=I2C",
		"controller=imu-i2c", 0x28, 0x10, -EIO));


	sensor_interface_register_sensor_imu_spi(&spi);
	const sensor_ext_ssi_t ext = {
		.ext_write = ext_write,
		.ext_write_read = ext_write_read,
		.ext_burst = 8,
		.ext_set_prefetch = NULL,
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
	CHECK(host_log_error_count() == 0);

	reset_counts();
	ext_seen_dummy = 2;
	fail_ext_read_call = 2;
	memset(dummy_data, 0xa5, sizeof(dummy_data));
	CHECK(ssi_burst_read_dummy(SENSOR_INTERFACE_DEV_MAG, 0x31, 2, dummy_data,
		sizeof(dummy_data)) == -EIO);
	CHECK(ext_read_calls == 2 && ext_read_regs[0] == 0x31 && ext_read_lengths[0] == 8 &&
		ext_read_regs[1] == 0x37 && ext_read_lengths[1] == 5);
	for (size_t i = 0; i < 6; i++)
		CHECK(dummy_data[i] == (uint8_t)(0x31 + i));
	for (size_t i = 6; i < sizeof(dummy_data); i++)
		CHECK(dummy_data[i] == 0xa5);
	CHECK(host_log_matches_error("burst_read_dummy", "dev=MAG", "bus=EXT",
		"controller=imu-spi", 0x14, 0x31, -EIO));

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
	CHECK(host_log_error_count() == 0);

	reset_counts();
	memset(long_data, 0xa5, sizeof(long_data));
	fail_ext_read_call = 2;
	CHECK(ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, 0x10, long_data, sizeof(long_data)) == -EIO);
	CHECK(ext_read_calls == 2 && ext_read_regs[0] == 0x10 && ext_read_lengths[0] == 8 &&
		ext_read_regs[1] == 0x18 && ext_read_lengths[1] == 5);
	for (size_t i = 0; i < 8; i++)
		CHECK(long_data[i] == (uint8_t)(0x10 + i));
	for (size_t i = 8; i < sizeof(long_data); i++)
		CHECK(long_data[i] == 0xa5);
	CHECK(host_log_matches_error("burst_read", "dev=MAG", "bus=EXT", "controller=imu-spi",
		0x14, 0x10, -EIO));

	reset_counts();
	memset(long_data, 0xa5, sizeof(long_data));
	fail_ext_read_call = 2;
	uint8_t requested_reg = 0x10;
	CHECK(ssi_write_read(SENSOR_INTERFACE_DEV_MAG, &requested_reg, 1, long_data,
		sizeof(long_data)) == -EIO);
	CHECK(ext_read_calls == 2 && ext_read_regs[0] == 0x10 && ext_read_regs[1] == 0x18);
	for (size_t i = 0; i < 8; i++)
		CHECK(long_data[i] == (uint8_t)(0x10 + i));
	for (size_t i = 8; i < sizeof(long_data); i++)
		CHECK(long_data[i] == 0xa5);
	CHECK(host_log_matches_error("write_read", "dev=MAG", "bus=EXT", "controller=imu-spi",
		0x14, 0x10, -EIO));

	reset_counts();
	fail_ext_write = 1;
	CHECK(ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, 0x22, 0x59) == -EIO);
	CHECK(ext_write_calls == 1 && ext_write_addr == 0x14 && ext_write_length == 2 &&
		ext_write_reg == 0x22 && ext_write_value == 0x59);
	CHECK(host_log_matches_error("reg_write", "dev=MAG", "bus=EXT", "controller=imu-spi",
		0x14, 0x22, -EIO));

	/* A read within one transaction must not be segmented. */
	reset_counts();
	uint8_t short_data[8] = {0};
	CHECK(ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, 0x20, short_data, sizeof(short_data)) == 0);
	CHECK(ext_read_calls == 1);
	CHECK(ext_read_regs[0] == 0x20 && ext_read_lengths[0] == 8);
	CHECK(host_log_error_count() == 0);

	/* Dummy prefix wider than the read budget is unsupported. */
	reset_counts();
	uint8_t tiny[4] = {0};
	CHECK(ssi_burst_read_dummy(SENSOR_INTERFACE_DEV_MAG, 0x40, 8, tiny, sizeof(tiny)) != 0);
	CHECK(ext_read_calls == 0);
	CHECK(host_log_matches_error("burst_read_dummy", "dev=MAG", "bus=EXT",
		"controller=imu-spi", 0x14, 0x40, -1));

	CHECK(test_seven_byte_frame_and_prefetch() == EXIT_SUCCESS);

	const sensor_ext_ssi_t word_proxy = {
		.ext_write = ext_write,
		.ext_write_read = synchronous_write_read,
		.ext_burst = 7,
	};
	reset_counts();
	ext_capacity = 7;
	sensor_interface_ext_configure(&word_proxy);
	CHECK(sensor_interface_register_sensor_mag_ext(0x13, 3, 8) == 0);
	uint8_t word_frame[8] = {0};
	const uint8_t expected_words[] = {0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49};
	CHECK(ssi_burst_read_frame(SENSOR_INTERFACE_DEV_MAG, 0x42, 0, 2, word_frame, sizeof(word_frame)) == 0);
	CHECK(memcmp(word_frame, expected_words, sizeof(word_frame)) == 0);
	CHECK(ext_read_calls == 2);
	CHECK(ext_read_regs[0] == 0x42 && ext_read_lengths[0] == 6);
	CHECK(ext_read_regs[1] == 0x48 && ext_read_lengths[1] == 2);
	CHECK(host_log_error_count() == 0);
	sensor_interface_ext_configure(NULL);

	printf("All interface interval tests passed\n");
	return EXIT_SUCCESS;
}
