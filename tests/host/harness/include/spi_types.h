#ifndef HOST_HARNESS_SPI_TYPES_H
#define HOST_HARNESS_SPI_TYPES_H

#include <stddef.h>
#include <stdint.h>

struct spi_config {
	uint32_t frequency;
};

struct spi_dt_spec {
	struct spi_config config;
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
