#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "sensor/interface.h"
#include "sensor/interface_internal.h"
#include "sensor/sensor.h"

struct host_write {
	uint8_t reg;
	uint8_t value;
	uint8_t bank;
};

static struct host_write writes[128];
static size_t write_count;
static uint8_t registers[256];
static size_t transaction_count;
static size_t fail_transaction;
static bool icm_banked, fail_write_applied;
static uint8_t icm_bank;
static int fail_read_reg = -1;
static size_t interval_bytes;
static uint8_t fifo_count_reg;
static bool fifo_count_is_bytes;
static unsigned gyro_discontinuities;

/* Opt-in LSM6DSM word FIFO. Other driver fixtures retain register-only reads. */
static bool dsm_fifo_enabled;
static uint16_t dsm_words[2048];
static size_t dsm_head, dsm_count;
static uint16_t dsm_pattern, dsm_cycle_words;
static uint8_t dsm_status_flags;

static void dsm_fifo_status(void)
{
	registers[0x3a] = dsm_count & 0xff;
	registers[0x3b] = ((dsm_count >> 8) & 7) | dsm_status_flags;
	registers[0x3c] = dsm_pattern & 0xff;
	registers[0x3d] = (dsm_pattern >> 8) & 3;
}

void host_bus_dsm_fifo_load(const uint16_t *words, size_t count,
	uint16_t pattern, uint16_t cycle_words, uint8_t flags)
{
	assert(count <= sizeof(dsm_words) / sizeof(dsm_words[0]));
	assert(cycle_words != 0 && pattern < cycle_words);
	memcpy(dsm_words, words, count * sizeof(*words));
	dsm_fifo_enabled = true;
	dsm_head = 0;
	dsm_count = count;
	dsm_pattern = pattern;
	dsm_cycle_words = cycle_words;
	dsm_status_flags = flags;
	dsm_fifo_status();
}

void sensor_gyro_report_discontinuity(void) { gyro_discontinuities++; }
unsigned host_bus_gyro_discontinuities(void) { return gyro_discontinuities; }

void host_bus_fail_read(int reg) { fail_read_reg = reg; }
size_t host_bus_interval_bytes(void) { return interval_bytes; }
void host_bus_fifo(uint8_t reg, bool bytes) { fifo_count_reg = reg; fifo_count_is_bytes = bytes; }
uint8_t host_bus_register(uint8_t reg) { return registers[reg]; }

void host_bus_reset(void)
{
	write_count = 0;
	transaction_count = 0;
	fail_transaction = 0;
	icm_banked = fail_write_applied = false;
	icm_bank = 0;
	fail_read_reg = -1;
	interval_bytes = 0;
	gyro_discontinuities = 0;
	dsm_fifo_enabled = false;
	dsm_head = dsm_count = 0;
	memset(registers, 0, sizeof(registers));
}

size_t host_bus_write_count(void) { return write_count; }
uint8_t host_bus_write_reg(size_t index) { return writes[index].reg; }
uint8_t host_bus_write_bank(size_t index) { return writes[index].bank; }
uint8_t host_bus_write_value(size_t index) { return writes[index].value; }
void host_bus_set_register(uint8_t reg, uint8_t value) { registers[reg] = value; }
void host_bus_fail_transaction(size_t transaction) { fail_transaction = transaction; }
size_t host_bus_transaction_count(void) { return transaction_count; }
void host_bus_icm_banked(bool applied_on_failure)
{
	icm_banked = true;
	fail_write_applied = applied_on_failure;
}
uint8_t host_bus_current_bank(void) { return icm_bank; }

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
	bool failed = fail_transaction == transaction_count;
	if (failed && (!icm_banked || !fail_write_applied))
		return -1;
	registers[reg] = value;
	if (write_count < sizeof(writes) / sizeof(writes[0]))
		writes[write_count++] = (struct host_write){reg, value, icm_bank};
	if (icm_banked && reg == 0x76)
		icm_bank = value;
	if (failed)
		return -1;
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
	if (dsm_fifo_enabled && reg == 0x3e) {
		assert(len % 2 == 0 && len / 2 <= dsm_count);
		for (uint32_t i = 0; i < len / 2; i++) {
			uint16_t word = dsm_words[dsm_head++];
			buf[i * 2] = word & 0xff;
			buf[i * 2 + 1] = word >> 8;
		}
		dsm_count -= len / 2;
		dsm_pattern = (dsm_pattern + len / 2) % dsm_cycle_words;
		dsm_fifo_status();
		return 0;
	}
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

int ssi_reg_read_byte_io(enum sensor_interface_dev dev, uint8_t reg, uint8_t *value,
	enum ssi_reporting reporting)
{
	(void)reporting;
	return ssi_reg_read_byte(dev, reg, value);
}

int ssi_reg_write_byte_io(enum sensor_interface_dev dev, uint8_t reg, uint8_t value,
	enum ssi_reporting reporting)
{
	(void)reporting;
	return ssi_reg_write_byte(dev, reg, value);
}

int ssi_burst_read_io(enum sensor_interface_dev dev, uint8_t reg, uint8_t *buf,
	uint32_t len, enum ssi_reporting reporting)
{
	(void)reporting;
	return ssi_burst_read(dev, reg, buf, len);
}

int ssi_burst_write_io(enum sensor_interface_dev dev, uint8_t reg, const uint8_t *buf,
	uint32_t len, enum ssi_reporting reporting)
{
	(void)reporting;
	return ssi_burst_write(dev, reg, buf, len);
}

int ssi_reg_read_interval(enum sensor_interface_dev dev, uint8_t reg, uint8_t *buf, uint32_t len, uint32_t interval)
{
	(void)interval;
	return ssi_burst_read(dev, reg, buf, len);
}

int ssi_burst_read_interval(enum sensor_interface_dev dev, uint8_t reg, uint8_t *buf, uint32_t len, uint32_t interval)
{
	(void)dev; (void)reg;
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
