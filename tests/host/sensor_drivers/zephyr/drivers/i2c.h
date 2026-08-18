#ifndef HOST_ZEPHYR_I2C_H
#define HOST_ZEPHYR_I2C_H

#include <stddef.h>
#include <stdint.h>

struct i2c_dt_spec {
	uint16_t addr;
};

int i2c_write_dt(const struct i2c_dt_spec *spec, const uint8_t *buf, size_t num_bytes);
int i2c_read_dt(const struct i2c_dt_spec *spec, uint8_t *buf, size_t num_bytes);
int i2c_write_read_dt(
	const struct i2c_dt_spec *spec,
	const void *write_buf,
	size_t num_write,
	void *read_buf,
	size_t num_read
);
int i2c_burst_write_dt(
	const struct i2c_dt_spec *spec,
	uint8_t start_addr,
	const uint8_t *buf,
	size_t num_bytes
);

#endif
