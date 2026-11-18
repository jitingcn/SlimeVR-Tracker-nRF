#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sensor/sensors.h"

/* Driver symbols for the production tables. Scanning does not call them. */
const sensor_mag_t sensor_mag_ak09940;
const sensor_mag_t sensor_mag_bmm150;
const sensor_mag_t sensor_mag_bmm350;
const sensor_mag_t sensor_mag_ict153xx;
const sensor_mag_t sensor_mag_ist8306;
const sensor_mag_t sensor_mag_ist8308;
const sensor_mag_t sensor_mag_lis2mdl;
const sensor_mag_t sensor_mag_lis3mdl;
const sensor_mag_t sensor_mag_mmc5603nj;
const sensor_mag_t sensor_mag_mmc5983ma;
const sensor_mag_t sensor_mag_qmc5883l;
const sensor_mag_t sensor_mag_qmc6309;
const sensor_mag_t sensor_mag_qmc5883p;

static struct spi_dt_spec spi_bus;
static const struct device i2c_bus;
static uint8_t spi_regs[128];
static bool spi_probed[128];
static bool spi_failed[128];
static uint8_t spi_dummy;
static uint8_t i2c_addr;
static uint8_t i2c_id;

int spi_transceive_dt(const struct spi_dt_spec *spec, const struct spi_buf_set *tx, const struct spi_buf_set *rx)
{
	assert(spec == &spi_bus);
	assert(tx->count == 1 && tx->buffers[0].len == 1);
	assert(rx->count == 1 && (rx->buffers[0].len == 2 || rx->buffers[0].len == 3));
	uint8_t command = *(const uint8_t *)tx->buffers[0].buf;
	assert(command & 0x80);
	uint8_t reg = command & 0x7f;
	spi_probed[reg] = true;
	uint8_t *data = rx->buffers[0].buf;
	data[0] = spi_dummy;
	int err = 0;
	for (size_t i = 1; i < rx->buffers[0].len; i++) {
		uint8_t payload_reg = (reg + i - 1) & 0x7f;
		data[i] = spi_regs[payload_reg];
		/* A failed transaction may still have populated the receive buffer. */
		if (spi_failed[payload_reg])
			err = -EIO;
	}
	return err;
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
	assert(dev == &i2c_bus && addr != i2c_addr);
	return -ENXIO;
}

int i2c_burst_read(const struct device *dev, uint16_t addr, uint8_t reg, uint8_t *buf, size_t len)
{
	assert(dev == &i2c_bus && addr != i2c_addr);
	return -ENXIO;
}

static const struct {
	int mag;
	uint8_t reg;
	uint8_t value;
	bool enabled;
} native_ids[] = {
	{MAG_MMC5983MA, 0x2f, 0x30, IS_ENABLED(CONFIG_SENSOR_DRV_MMC5983MA)},
	{MAG_LIS3MDL,   0x0f, 0x3d, IS_ENABLED(CONFIG_SENSOR_DRV_LIS3MDL)},
	{MAG_LIS2MDL,   0x4f, 0x40, IS_ENABLED(CONFIG_SENSOR_DRV_LIS2MDL)},
	{MAG_BMM150,    0x40, 0x32, IS_ENABLED(CONFIG_SENSOR_DRV_BMM150)},
	{MAG_AK09940,   0x00, 0x48, IS_ENABLED(CONFIG_SENSOR_DRV_AK09940)},
};

static void reset_spi(void)
{
	memset(spi_regs, 0, sizeof(spi_regs));
	memset(spi_failed, 0, sizeof(spi_failed));
	spi_dummy = 0;
}

static void check_mag_spi(uint8_t hint, int expected)
{
	bool allowed[128] = {0};
	uint8_t expected_hint = hint < 0x80 ? hint : 0xff;
	for (size_t i = 0; i < ARRAY_SIZE(native_ids); i++) {
		if (!native_ids[i].enabled)
			continue;
		allowed[native_ids[i].reg] = true;
		if (native_ids[i].mag == MAG_AK09940)
			allowed[0x01] = true;
		if (native_ids[i].mag == expected)
			expected_hint = native_ids[i].reg | 0x80;
	}

	memset(spi_probed, 0, sizeof(spi_probed));
	uint8_t reg = hint;
	assert(sensor_scan_mag_spi(&spi_bus, &reg) == expected);
	assert(reg == expected_hint || (expected == MAG_AK09940 && reg == 0x81));
	/* No order assumption; I2C-only and no-driver configs allow no transfers. */
	for (size_t i = 0; i < ARRAY_SIZE(spi_probed); i++)
		assert(!spi_probed[i] || allowed[i]);
}

static void check_native_ids(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(native_ids); i++) {
		uint8_t reg = native_ids[i].reg;
		uint8_t value = native_ids[i].value;
		int mag = native_ids[i].mag;
		reset_spi();
		/* A BMM150 fixture represents an already-awake device. */
		spi_regs[reg] = value;
		if (mag == MAG_AK09940)
			spi_regs[0x01] = 0xa3;
		check_mag_spi(0xff, native_ids[i].enabled ? mag : -1);

		if (!native_ids[i].enabled)
			continue;
		spi_failed[reg] = true;
		check_mag_spi(reg | 0x80, -1);
		if (mag == MAG_AK09940)
			continue;

		reset_spi();
		spi_dummy = value;
		check_mag_spi(0xff, -1);
		spi_dummy = 0;
		spi_regs[reg + 1] = value;
		check_mag_spi(0xff, -1);
	}
}

static void check_ak_signature(void)
{
	reset_spi();
	spi_regs[0x00] = 0x48;
	spi_regs[0x01] = 0xa2;
	spi_regs[0x02] = 0xa3;
	check_mag_spi(0x81, -1);
	spi_regs[0x01] = 0x00;
	check_mag_spi(0xff, -1); /* Neither the next register nor a zero is a valid WIA2. */
	spi_dummy = 0xa3;
	check_mag_spi(0x01, -1); /* The command-phase byte is not WIA2. */
	spi_dummy = 0;

	spi_regs[0x01] = 0xa3;
	spi_failed[0x01] = true;
	check_mag_spi(0x81, -1);
	spi_failed[0x01] = false;
	spi_regs[0x00] = 0x00;
	spi_dummy = 0x48;
	check_mag_spi(0xff, -1);
	spi_dummy = 0;
	spi_regs[0x01] = 0x48;
	check_mag_spi(0xff, -1); /* An AK-like pair starts one byte too late. */
}

static void check_priority(void)
{
	reset_spi();
	/* Both MMC XOUT bytes can look like the complete AK signature. */
	spi_regs[0x00] = 0x48;
	spi_regs[0x01] = 0xa3;
	spi_regs[0x2f] = 0x30;
	int ak_or_none = IS_ENABLED(CONFIG_SENSOR_DRV_AK09940) ? MAG_AK09940 : -1;
	int expected = IS_ENABLED(CONFIG_SENSOR_DRV_MMC5983MA) ? MAG_MMC5983MA : ak_or_none;
	check_mag_spi(0xff, expected);
	check_mag_spi(0x00, expected);
	check_mag_spi(0x81, expected);
	check_mag_spi(0xaf, expected);

	spi_failed[0x2f] = true;
	check_mag_spi(0xaf, ak_or_none);
}

static void check_i2c_only_ids(void)
{
	/* These IDs exist in I2C tables; none is a native SPI identity. */
	static const struct {
		uint8_t reg;
		uint8_t value;
	} ids[] = {
		{0x00, 0x06}, {0x00, 0x08}, {0x00, 0x20}, {0x00, 0x21},
		{0x00, 0x33}, {0x00, 0x48}, {0x00, 0x80}, {0x00, 0x90},
		{0x01, 0x09}, {0x0a, 0x48}, {0x0d, 0xff}, {0x20, 0x06},
		{0x2f, 0x0a}, {0x39, 0x10}, {0x39, 0x11},
	};
	for (size_t i = 0; i < ARRAY_SIZE(ids); i++) {
		reset_spi();
		spi_regs[ids[i].reg] = ids[i].value;
		check_mag_spi(0xff, -1);
		check_mag_spi(ids[i].reg | 0x80, -1);
	}

	reset_spi();
	spi_regs[0x00] = 0xe7; /* ICT153xx manufacturer and chip IDs. */
	spi_regs[0x01] = 0x45;
	check_mag_spi(0x80, -1);
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
	reset_spi();
	spi_regs[0x75] = 0x68;
	uint8_t reg = 0xff;
	assert(sensor_scan_imu_spi(&spi_bus, &reg) == IMU_MPU6050);
	assert(reg == 0xf5);
}

int main(void)
{
	check_native_ids();
	check_ak_signature();
	check_priority();
	check_i2c_only_ids();

	if (IS_ENABLED(CONFIG_SENSOR_DRV_QMC6309)) {
		check_i2c(0x2c, 0x80, MAG_QMC5883P);
		check_i2c(0x3c, 0x80, IS_ENABLED(CONFIG_SENSOR_MAG_DRIVERS_MINIMAL) ? -1 : MAG_QMC6310);
		check_i2c(0x0c, 0x90, MAG_QMC6309);
		check_i2c(0x7c, 0x90, MAG_QMC6309);
	}
	check_imu();
	puts("direct SPI magnetometer scan and I2C/IMU regressions passed");
	return 0;
}
