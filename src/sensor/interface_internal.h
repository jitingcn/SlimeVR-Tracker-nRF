#ifndef SLIMENRF_SENSOR_INTERFACE_INTERNAL
#define SLIMENRF_SENSOR_INTERFACE_INTERNAL

#include "interface.h"

enum ssi_reporting {
	SSI_UNREPORTED,
	SSI_REPORT,
};

int ssi_reg_read_byte_io(enum sensor_interface_dev dev, uint8_t reg_addr, uint8_t *value, enum ssi_reporting reporting);
int ssi_reg_write_byte_io(enum sensor_interface_dev dev, uint8_t reg_addr, uint8_t value, enum ssi_reporting reporting);
int ssi_burst_read_io(enum sensor_interface_dev dev, uint8_t start_addr, uint8_t *buf, uint32_t num_bytes, enum ssi_reporting reporting);
int ssi_burst_write_io(enum sensor_interface_dev dev, uint8_t start_addr, const uint8_t *buf, uint32_t num_bytes, enum ssi_reporting reporting);

#endif
