#ifndef HOST_SCAN_REMOVAL_I2C_H
#define HOST_SCAN_REMOVAL_I2C_H

#include <stddef.h>
#include <stdint.h>

struct device { int unused; };
struct i2c_dt_spec {
	const struct device *bus;
	uint16_t addr;
};

int i2c_reg_read_byte(const struct device *dev, uint16_t addr, uint8_t reg, uint8_t *value);
int i2c_reg_write_byte(const struct device *dev, uint16_t addr, uint8_t reg, uint8_t value);
int i2c_burst_read(const struct device *dev, uint16_t addr, uint8_t reg, uint8_t *buf, size_t len);

#endif
