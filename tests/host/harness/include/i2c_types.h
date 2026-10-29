#ifndef HOST_HARNESS_I2C_TYPES_H
#define HOST_HARNESS_I2C_TYPES_H

#include <stdint.h>
#include "device_types.h"

struct i2c_dt_spec {
	uint16_t addr;
	const struct device *bus;
};

#endif
