#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "sensor/interface.h"

struct host_write {
	uint8_t reg;
	uint8_t value;
};

static struct host_write writes[128];
static size_t write_count;
static uint8_t registers[256];
static size_t transaction_count;
static size_t fail_transaction;
static int fail_read_reg = -1;
static size_t interval_bytes, interval_reads;
static uint8_t fifo_count_reg;
static bool fifo_count_is_bytes;

void host_bus_fail_read(int reg) { fail_read_reg = reg; }
size_t host_bus_interval_bytes(void) { return interval_bytes; }
size_t host_bus_interval_reads(void) { return interval_reads; }
void host_bus_fifo(uint8_t reg, bool bytes) { fifo_count_reg = reg; fifo_count_is_bytes = bytes; }
uint8_t host_bus_register(uint8_t reg) { return registers[reg]; }

void host_bus_reset(void)
{
	write_count = 0;
	transaction_count = 0;
	fail_transaction = 0;
	fail_read_reg = -1;
	interval_bytes = 0;
	interval_reads = 0;
	memset(registers, 0, sizeof(registers));
}

size_t host_bus_write_count(void) { return write_count; }
uint8_t host_bus_write_reg(size_t index) { return writes[index].reg; }
uint8_t host_bus_write_value(size_t index) { return writes[index].value; }
void host_bus_set_register(uint8_t reg, uint8_t value) { registers[reg] = value; }
void host_bus_fail_transaction(size_t transaction) { fail_transaction = transaction; }

int sensor_interface_spi_configure(enum sensor_interface_dev dev, uint32_t frequency, uint32_t dummy_reads)
{
	(void)dev; (void)frequency; (void)dummy_reads;
	return 0;
}

void sensor_interface_ext_configure(const sensor_ext_ssi_t *ext)
{
	(void)ext;
}

int ssi_reg_write_byte(enum sensor_interface_dev dev, uint8_t reg, uint8_t value)
{
	(void)dev;
	transaction_count++;
	if (fail_transaction == transaction_count)
		return -1;
	registers[reg] = value;
	if (write_count < sizeof(writes) / sizeof(writes[0]))
		writes[write_count++] = (struct host_write){reg, value};
	return 0;
}

int ssi_reg_read_byte(enum sensor_interface_dev dev, uint8_t reg, uint8_t *value)
{
	(void)dev;
	transaction_count++;
	if (fail_transaction == transaction_count || reg == fail_read_reg)
		return -1;
	*value = registers[reg];
	return 0;
}

int ssi_reg_update_byte(enum sensor_interface_dev dev, uint8_t reg, uint8_t mask, uint8_t value)
{
	return ssi_reg_write_byte(dev, reg, (registers[reg] & (uint8_t)~mask) | (value & mask));
}

int ssi_burst_read(enum sensor_interface_dev dev, uint8_t reg, uint8_t *buf, uint32_t len)
{
	transaction_count++;
	if (fail_transaction == transaction_count || reg == fail_read_reg)
		return -1;
	(void)dev;
	for (uint32_t i = 0; i < len; i++) buf[i] = registers[(uint8_t)(reg + i)];
	return 0;
}

int ssi_burst_read_dummy(enum sensor_interface_dev dev, uint8_t reg, uint8_t dummy, uint8_t *buf, uint32_t len)
{
	(void)dummy;
	return ssi_burst_read(dev, reg, buf, len);
}

int ssi_burst_write(enum sensor_interface_dev dev, uint8_t reg, const uint8_t *buf, uint32_t len)
{
	(void)dev;
	for (uint32_t i = 0; i < len; ++i) registers[(uint8_t)(reg + i)] = buf[i];
	return 0;
}

int ssi_reg_read_interval(enum sensor_interface_dev dev, uint8_t reg, uint8_t *buf, uint32_t len, uint32_t interval)
{
	(void)interval;
	return ssi_burst_read(dev, reg, buf, len);
}

int ssi_burst_read_interval(enum sensor_interface_dev dev, uint8_t reg, uint8_t *buf, uint32_t len, uint32_t interval)
{
	(void)dev; (void)reg;
	interval_reads++;
	interval_bytes += len;
	/* Deliberately write every requested byte: canaries expose oversized transfers. */
	memset(buf, 0xA5, len);
	uint16_t count = fifo_count_is_bytes
		? registers[fifo_count_reg] | (registers[(uint8_t)(fifo_count_reg + 1)] << 8)
		: (registers[fifo_count_reg] << 8) | registers[(uint8_t)(fifo_count_reg + 1)];
	uint16_t consumed = fifo_count_is_bytes ? len : len / interval;
	count = count > consumed ? count - consumed : 0;
	registers[fifo_count_reg] = fifo_count_is_bytes ? count : count >> 8;
	registers[(uint8_t)(fifo_count_reg + 1)] = fifo_count_is_bytes ? count >> 8 : count;
	return 0;
}
