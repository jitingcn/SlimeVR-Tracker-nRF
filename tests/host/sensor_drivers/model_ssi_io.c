#include <assert.h>

#include "sensor/interface_internal.h"

static void check_reporting(enum ssi_reporting reporting)
{
	assert(reporting == SSI_UNREPORTED || reporting == SSI_REPORT);
}

int ssi_reg_read_byte_io(enum sensor_interface_dev dev, uint8_t reg, uint8_t *value,
	enum ssi_reporting reporting)
{
	check_reporting(reporting);
	return ssi_reg_read_byte(dev, reg, value);
}

int ssi_reg_write_byte_io(enum sensor_interface_dev dev, uint8_t reg, uint8_t value,
	enum ssi_reporting reporting)
{
	check_reporting(reporting);
	return ssi_reg_write_byte(dev, reg, value);
}

int ssi_burst_read_io(enum sensor_interface_dev dev, uint8_t reg, uint8_t *buf,
	uint32_t len, enum ssi_reporting reporting)
{
	check_reporting(reporting);
	return ssi_burst_read(dev, reg, buf, len);
}

int ssi_burst_write_io(enum sensor_interface_dev dev, uint8_t reg, const uint8_t *buf,
	uint32_t len, enum ssi_reporting reporting)
{
	check_reporting(reporting);
	return ssi_burst_write(dev, reg, buf, len);
}
