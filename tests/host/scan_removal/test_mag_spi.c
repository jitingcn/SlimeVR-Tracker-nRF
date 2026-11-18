#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sensor/sensors.h"

/* Driver symbols for the production tables. Scanning does not call them. */
const sensor_mag_t sensor_mag_qmc6309;
const sensor_mag_t sensor_mag_qmc5883p;
const sensor_mag_t sensor_mag_mmc5983ma;

static struct spi_dt_spec spi_bus;
static const struct device i2c_bus;
static uint8_t spi_regs[128];
static bool spi_probed[128];
static uint8_t i2c_addr;
static uint8_t i2c_id;

int spi_transceive_dt(const struct spi_dt_spec *spec, const struct spi_buf_set *tx, const struct spi_buf_set *rx)
{
	assert(spec == &spi_bus);
	assert(tx->count == 1 && tx->buffers[0].len == 1);
	assert(rx->count == 1 && rx->buffers[0].len == 3);
	uint8_t command = *(const uint8_t *)tx->buffers[0].buf;
	assert(command & 0x80);
	uint8_t reg = command & 0x7f;
	spi_probed[reg] = true;
	uint8_t *data = rx->buffers[0].buf;
	data[0] = 0;
	data[1] = spi_regs[reg];
	data[2] = spi_regs[(reg + 1) & 0x7f];
	return 0;
}

int i2c_reg_read_byte(const struct device *dev, uint16_t addr, uint8_t reg, uint8_t *value)
{
	assert(dev == &i2c_bus);
	if (addr != i2c_addr)
		return -ENXIO;
	*value = reg == 0x00 ? i2c_id : 0;
	return 0;
}

int i2c_reg_write_byte(const struct device *dev, uint16_t addr, uint8_t reg, uint8_t value)
{
	assert(false); /* These scan rows do not require a write. */
	return -EIO;
}

int i2c_burst_read(const struct device *dev, uint16_t addr, uint8_t reg, uint8_t *buf, size_t len)
{
	assert(false); /* These scan rows do not require dummy-byte reads. */
	return -EIO;
}

static void check_mmc(uint8_t hint, uint8_t product_id, int expected)
{
	memset(spi_regs, 0, sizeof(spi_regs));
	memset(spi_probed, 0, sizeof(spi_probed));
	/* An MMC5983MA can report a midpoint X sample whose high byte matches the
	 * QMC5883P/QMC6310 I2C ID. Only 0x2f is its product ID register. */
	spi_regs[MMC5983MA_XOUT_0] = 0x80;
	spi_regs[0x2f] = product_id;

	uint8_t reg = hint;
	assert(sensor_scan_mag_spi(&spi_bus, &reg) == expected);
	assert(reg == (expected == MAG_MMC5983MA ? 0xaf : 0xff));
	assert(spi_probed[MMC5983MA_XOUT_0] == (hint != 0xaf));
	assert(spi_probed[0x2f]);
}

static void check_i2c(uint8_t addr, uint8_t id, int expected)
{
	i2c_addr = addr;
	i2c_id = id;
	struct i2c_dt_spec bus = {.bus = &i2c_bus};
	uint8_t reg = 0xff;
	assert(sensor_scan_mag(&bus, &reg) == expected);
	assert(bus.addr == (expected >= 0 ? addr : 0xff));
	assert(reg == (expected >= 0 ? 0x00 : 0xff));
}

static void check_imu(void)
{
	/* The MPU6050/MPU6000 row shares MAG_QMC6310's enum value, not its ID. */
	_Static_assert((int)IMU_MPU6050 == (int)MAG_QMC6310, "expected overlapping enum values");
	memset(spi_regs, 0, sizeof(spi_regs));
	spi_regs[0x75] = 0x68;
	uint8_t reg = 0xff;
	assert(sensor_scan_imu_spi(&spi_bus, &reg) == IMU_MPU6050);
	assert(reg == 0xf5);
}

int main(void)
{
	check_mmc(0xff, 0x30, MAG_MMC5983MA); /* Full search. */
	check_mmc(0x80, 0x30, MAG_MMC5983MA); /* Stale QMC register hint. */
	check_mmc(0xaf, 0x30, MAG_MMC5983MA); /* Retained MMC5983MA hint. */
	check_mmc(0xff, 0x00, -1); /* XOUT0 alone never identifies a chip. */

	check_i2c(0x2c, 0x80, MAG_QMC5883P);
	check_i2c(0x3c, 0x80, IS_ENABLED(CONFIG_SENSOR_MAG_DRIVERS_MINIMAL) ? -1 : MAG_QMC6310);
	check_i2c(0x0c, 0x90, MAG_QMC6309);
	check_i2c(0x7c, 0x90, MAG_QMC6309);
	check_imu();
	puts("direct SPI magnetometer scan and I2C/IMU regressions passed");
	return 0;
}
