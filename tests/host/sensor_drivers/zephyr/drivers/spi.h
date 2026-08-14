#ifndef HOST_ZEPHYR_SPI_H
#define HOST_ZEPHYR_SPI_H

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

int spi_transceive_dt(
	const struct spi_dt_spec *spec,
	const struct spi_buf_set *tx,
	const struct spi_buf_set *rx
);

#endif
