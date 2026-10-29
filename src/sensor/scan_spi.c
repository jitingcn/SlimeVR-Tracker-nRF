/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include <zephyr/logging/log.h>
#include <zephyr/types.h>
#include <zephyr/drivers/spi.h>

#if defined(CONFIG_SENSOR_DRV_ICT153XX)
#include "mag/ICT153xx.h"
#include "sensors_enum.h"
#endif

LOG_MODULE_REGISTER(sensor_scan_spi, LOG_LEVEL_DBG);

static int sensor_scan_spi_match(uint8_t reg, uint8_t id, int groups,
	const uint8_t dev_reg[], const uint8_t dev_id[], const int dev_ids[])
{
	int ri = 0, ii = 0, fi = 0;
	for (int group = 0; group < groups; group++) {
		int registers = dev_reg[ri++];
		for (int entry = 0; entry < registers; entry++) {
			uint8_t candidate_reg = dev_reg[ri++];
			int count = dev_id[ii++];
			for (int item = 0; item < count; item++) {
				uint8_t candidate = dev_id[ii + item];
				if (candidate_reg != reg || candidate == 0xff)
					continue;
#if defined(CONFIG_SENSOR_DRV_ICT153XX)
				if (reg == ICT153XX_MANU_ID && candidate == ICT153XX_MANU_ID_VALUE
					&& dev_ids[fi + item] == MAG_ICT153XX)
					continue;
#endif
				if (candidate == id)
					return dev_ids[fi + item];
			}
			ii += count;
			fi += count;
		}
	}
	return -1;
}

int sensor_scan_spi(struct spi_dt_spec *bus, uint8_t *spi_dev_reg, int dev_addr_count, const uint8_t dev_reg[], const uint8_t dev_id[], const int dev_ids[])
{
	uint8_t buf[3] = {0};
	struct spi_buf tx_buf = {.len = 1};
	const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
	struct spi_buf rx_buf = {.buf = buf, .len = 3};
	const struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

	/* SPI has no address groups. Probe each eligible register once, then
	 * match its response against every group, including later-only IDs. */
	uint8_t seen[16] = {0};
	int reg_index = 0, id_index = 0;
	for (int group = 0; group < dev_addr_count; group++) {
		int count = dev_reg[reg_index++];
		for (int entry = 0; entry < count; entry++) {
			uint8_t reg = dev_reg[reg_index++];
			int ids = dev_id[id_index++];
			id_index += ids;
			if (!ids || (*spi_dev_reg != 0xff && (*spi_dev_reg & 0x7f) != reg))
				continue;
			uint8_t bit = 1u << (reg & 7);
			if (seen[(reg & 0x7f) >> 3] & bit)
				continue;
			seen[(reg & 0x7f) >> 3] |= bit;
			uint8_t command = reg | 0x80;
			tx_buf.buf = &command;
			int err = spi_transceive_dt(bus, &tx, &rx);
			if (err)
				continue;
			uint8_t id = buf[1] ? buf[1] : buf[2];
			int found = sensor_scan_spi_match(reg, id, dev_addr_count, dev_reg, dev_id, dev_ids);
			if (found >= 0) {
				*spi_dev_reg = command;
				LOG_INF("Valid device found using register: 0x%02X (value: 0x%02X)", command, id);
				return found;
			}
		}
	}

	if (*spi_dev_reg != 0xFF) // preferred register failed, try again with full scan
	{
		LOG_WRN("No device found using register: 0x%02X", *spi_dev_reg);
		*spi_dev_reg = 0xFF;
		return sensor_scan_spi(bus, spi_dev_reg, dev_addr_count, dev_reg, dev_id, dev_ids);
	}

	return -1;
}
