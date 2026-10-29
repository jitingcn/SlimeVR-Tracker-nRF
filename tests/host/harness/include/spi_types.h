#ifndef HOST_HARNESS_SPI_TYPES_H
#define HOST_HARNESS_SPI_TYPES_H

#include <stddef.h>
#include <stdint.h>
#include "device_types.h"

struct spi_config {
	uint32_t frequency;
	uint16_t slave;
};

struct spi_dt_spec {
	struct spi_config config;
	const struct device *bus;
};

struct spi_buf {
	void *buf;
	size_t len;
};

struct spi_buf_set {
	struct spi_buf *buffers;
	size_t count;
};

#endif
