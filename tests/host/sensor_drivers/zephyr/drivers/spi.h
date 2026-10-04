#ifndef HOST_ZEPHYR_SPI_H
#define HOST_ZEPHYR_SPI_H

#include "../../../harness/include/spi_types.h"

int spi_transceive_dt(
	const struct spi_dt_spec *spec,
	const struct spi_buf_set *tx,
	const struct spi_buf_set *rx
);

#endif
