#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sensor/sensors.h"
#include "sensor/scan_spi.h"
#include "build_defines.h"
#include ".generated/server_imu_id.inc"

/* Only the bus and supported drivers are modeled. The scanners, the device
 * tables, the none driver, and the server-ID conversion are production code. */
static int modeled_init(float clock, float accel, float gyro, float *actual_accel, float *actual_gyro)
{
	(void)clock;
	(void)accel;
	(void)gyro;
	(void)actual_accel;
	(void)actual_gyro;
	return 0;
}

const sensor_imu_t sensor_imu_icm42688 = {.init = modeled_init};
const sensor_imu_t sensor_imu_icm42686 = {.init = modeled_init};
const sensor_imu_t sensor_imu_icm45686 = {.init = modeled_init};
const sensor_imu_t sensor_imu_lsm6dsm = {.init = modeled_init};
const sensor_imu_t sensor_imu_lsm6dso = {.init = modeled_init};
const sensor_imu_t sensor_imu_lsm6dsv = {.init = modeled_init};


static const char *current_case = "identity";
static unsigned current_addr;
static unsigned current_reg;
static unsigned current_chip_id;
static const char *current_hint = "";

static void check_failed(const char *expression, int line)
{
	fprintf(stderr, "%s:%d: %s: %s (addr=0x%02x reg=0x%02x id=0x%02x, %s)\n",
		__FILE__, line, current_case, expression, current_addr, current_reg,
		current_chip_id, current_hint);
	exit(EXIT_FAILURE);
}

#define CHECK(expression) do { if (!(expression)) check_failed(#expression, __LINE__); } while (0)
#define DRIVER_IF(option, name) (IS_ENABLED(option) ? &(name) : &sensor_imu_none)

struct identity {
	int enum_id;
	int raw_slot;
	int server_id;
	const sensor_imu_t *driver;
};

/* Explicit raw and server numbers protect the historical wire/storage IDs. */
static const struct identity identities[] = {
	{IMU_BMI160,    0,  8, &sensor_imu_none},
	{IMU_BMI270,    1, 11, &sensor_imu_none},
	{IMU_BMI323,    2,  0, &sensor_imu_none},
	{IMU_MPU6050,   3,  6, &sensor_imu_none},
	{IMU_MPU6500,   4,  2, &sensor_imu_none},
	{IMU_MPU9250,   5,  1, &sensor_imu_none},
	{IMU_ICM20948,  6,  9, &sensor_imu_none},
	{IMU_ICM42688,  7, 10, DRIVER_IF(CONFIG_SENSOR_DRV_ICM42688, sensor_imu_icm42688)},
	{IMU_ICM42686,  8, 10, DRIVER_IF(CONFIG_SENSOR_DRV_ICM42686, sensor_imu_icm42686)},
	{IMU_ICM45686,  9, 16, DRIVER_IF(CONFIG_SENSOR_DRV_ICM45686, sensor_imu_icm45686)},
	{IMU_ICM45688, 10, 16, DRIVER_IF(CONFIG_SENSOR_DRV_ICM45686, sensor_imu_icm45686)},
	{IMU_ISM330IS, 11,  0, &sensor_imu_none},
	{IMU_LSM6DS3,  12,  0, DRIVER_IF(CONFIG_SENSOR_DRV_LSM6DSM, sensor_imu_lsm6dsm)},
	{IMU_LSM6DSM,  13, 12, DRIVER_IF(CONFIG_SENSOR_DRV_LSM6DSM, sensor_imu_lsm6dsm)},
	{IMU_LSM6DSR,  14, 15, DRIVER_IF(CONFIG_SENSOR_DRV_LSM6DSO, sensor_imu_lsm6dso)},
	{IMU_LSM6DSO,  15, 14, DRIVER_IF(CONFIG_SENSOR_DRV_LSM6DSO, sensor_imu_lsm6dso)},
	{IMU_LSM6DST,  16,  0, &sensor_imu_none},
	{IMU_LSM6DSV,  17, 13, DRIVER_IF(CONFIG_SENSOR_DRV_LSM6DSV, sensor_imu_lsm6dsv)},
	{IMU_ISM330BX, 18, 13, DRIVER_IF(CONFIG_SENSOR_DRV_LSM6DSV, sensor_imu_lsm6dsv)},
};

#define KEEP_UNIMPLEMENTED (!IS_ENABLED(CONFIG_SENSOR_IMU_DRIVERS_MINIMAL))

struct scan_row {
	const char *name;
	uint8_t first_addr;
	uint8_t reg;
	uint8_t chip_id;
	int enum_id;
	bool detectable;
};

/* Include disabled and legacy rows too: they must not alias another entry.
 * In a full configuration the historically recognized, unimplemented rows
 * still scan to their original enum but select sensor_imu_none. */
static const struct scan_row rows[] = {
	{"removed BMI270", 0x68, 0x00, 0x24, IMU_BMI270, false},
	{"ICM20948",       0x68, 0x00, 0xEA, IMU_ICM20948, KEEP_UNIMPLEMENTED},
	{"BMI160",         0x68, 0x00, 0xD1, IMU_BMI160, KEEP_UNIMPLEMENTED},
	{"BMI323",         0x68, 0x00, 0x43, IMU_BMI323, KEEP_UNIMPLEMENTED},
	{"ICM45686",       0x68, 0x72, 0xE9, IMU_ICM45686, IS_ENABLED(CONFIG_SENSOR_DRV_ICM45686)},
	{"ICM45688",       0x68, 0x72, 0xE7, IMU_ICM45688, IS_ENABLED(CONFIG_SENSOR_DRV_ICM45686)},
	{"MPU6050",        0x68, 0x75, 0x68, IMU_MPU6050, KEEP_UNIMPLEMENTED},
	{"MPU6500",        0x68, 0x75, 0x70, IMU_MPU6500, KEEP_UNIMPLEMENTED},
	{"MPU9250",        0x68, 0x75, 0x71, IMU_MPU9250, KEEP_UNIMPLEMENTED},
	{"ICM42686",       0x68, 0x75, 0x44, IMU_ICM42686, IS_ENABLED(CONFIG_SENSOR_DRV_ICM42686)},
	{"ICM42688-P",     0x68, 0x75, 0x47, IMU_ICM42688, IS_ENABLED(CONFIG_SENSOR_DRV_ICM42688)},
	{"ICM42688-V",     0x68, 0x75, 0xDB, IMU_ICM42688, IS_ENABLED(CONFIG_SENSOR_DRV_ICM42688)},
	{"ISM330IS",       0x6A, 0x0F, 0x22, IMU_ISM330IS, KEEP_UNIMPLEMENTED},
	{"LSM6DS3",        0x6A, 0x0F, 0x69, IMU_LSM6DS3, IS_ENABLED(CONFIG_SENSOR_DRV_LSM6DSM)},
	{"LSM6DSM",        0x6A, 0x0F, 0x6A, IMU_LSM6DSM, IS_ENABLED(CONFIG_SENSOR_DRV_LSM6DSM)},
	{"LSM6DSR",        0x6A, 0x0F, 0x6B, IMU_LSM6DSR, IS_ENABLED(CONFIG_SENSOR_DRV_LSM6DSO)},
	{"LSM6DSO",        0x6A, 0x0F, 0x6C, IMU_LSM6DSO, IS_ENABLED(CONFIG_SENSOR_DRV_LSM6DSO)},
	{"LSM6DST",        0x6A, 0x0F, 0x6D, IMU_LSM6DST, KEEP_UNIMPLEMENTED},
	{"LSM6DSV",        0x6A, 0x0F, 0x70, IMU_LSM6DSV, IS_ENABLED(CONFIG_SENSOR_DRV_LSM6DSV)},
	{"ISM330BX",       0x6A, 0x0F, 0x71, IMU_ISM330BX, IS_ENABLED(CONFIG_SENSOR_DRV_LSM6DSV)},
};

static const struct device host_i2c_bus;
static struct spi_dt_spec host_spi_bus;
static struct {
	uint16_t addr;
	uint8_t reg;
	uint8_t id;
	unsigned spi_id_offset;
	unsigned target_reads;
	unsigned spi_calls;
	bool fail_spi;
} chip;

int i2c_reg_read_byte(const struct device *dev, uint16_t addr, uint8_t reg, uint8_t *value)
{
	CHECK(dev == &host_i2c_bus);
	if (addr != chip.addr)
		return -ENXIO;
	if (reg == chip.reg) {
		chip.target_reads++;
		*value = chip.id;
	} else {
		*value = 0;
	}
	return 0;
}

int i2c_reg_write_byte(const struct device *dev, uint16_t addr, uint8_t reg, uint8_t value)
{
	(void)dev;
	(void)addr;
	(void)reg;
	(void)value;
	CHECK(false); /* No IMU scan row requires the magnetometer power-up write. */
	return -EIO;
}

int i2c_burst_read(const struct device *dev, uint16_t addr, uint8_t reg, uint8_t *buf, size_t len)
{
	(void)dev;
	(void)addr;
	(void)reg;
	(void)buf;
	(void)len;
	CHECK(false); /* No IMU scan row uses the BMM350 burst workaround. */
	return -EIO;
}

int spi_transceive_dt(const struct spi_dt_spec *spec, const struct spi_buf_set *tx, const struct spi_buf_set *rx)
{
	CHECK(spec == &host_spi_bus);
	CHECK(tx->count == 1 && tx->buffers[0].len == 1);
	CHECK(rx->count == 1 && rx->buffers[0].len == 3);
	chip.spi_calls++;
	uint8_t requested = *(const uint8_t *)tx->buffers[0].buf;
	CHECK((requested & 0x80) != 0);
	uint8_t *response = rx->buffers[0].buf;
	memset(response, 0, 3);
	if ((requested & 0x7F) == chip.reg) {
		chip.target_reads++;
		if (chip.fail_spi)
			return -EIO;
		response[chip.spi_id_offset] = chip.id;
	}
	return 0;
}

static void check_identities(void)
{
	CHECK(ARRAY_SIZE(identities) == 19);
	CHECK(SENSOR_DEV_IMU_COUNT == 19);
	for (size_t i = 0; i < ARRAY_SIZE(identities); ++i) {
		const struct identity *identity = &identities[i];
		CHECK(identity->enum_id == identity->raw_slot);
		CHECK(identity->raw_slot == (int)i);
		CHECK(get_server_constant_imu_id(identity->enum_id) == identity->server_id);
		CHECK(sensor_imus[identity->raw_slot] == identity->driver);
		CHECK(sensor_imus[identity->raw_slot]->init != NULL);
		float accel = -1.0f, gyro = -1.0f;
		int rc = sensor_imus[identity->raw_slot]->init(0, 0, 0, &accel, &gyro);
		CHECK(identity->driver == &sensor_imu_none ? rc < 0 : rc == 0);
	}
	CHECK(IMU_BMI270 == 1);
	CHECK(get_server_constant_imu_id(1) == 11);
	CHECK(strcmp(dev_imu_names[1], "BMI270") == 0);
	CHECK(sensor_imus[1] == &sensor_imu_none);
}

static void set_chip(const struct scan_row *row, uint16_t addr, unsigned spi_id_offset)
{
	chip.addr = addr;
	chip.reg = row->reg;
	chip.id = row->chip_id;
	chip.spi_id_offset = spi_id_offset;
	chip.target_reads = 0;
	chip.spi_calls = 0;
	chip.fail_spi = false;
	current_case = row->name;
	current_addr = addr;
	current_reg = row->reg;
	current_chip_id = row->chip_id;
}

static void check_selected(const struct scan_row *row, int result)
{
	CHECK(result == (row->detectable ? row->enum_id : -1));
	if (result >= 0) {
		CHECK(result < (int)ARRAY_SIZE(identities));
		CHECK(sensor_imus[result] == identities[result].driver);
		CHECK(get_server_constant_imu_id(result) == identities[result].server_id);
	}
}

static void scan_i2c_case(const struct scan_row *row, uint16_t addr, uint16_t preferred_addr,
	uint8_t preferred_reg, const char *hint)
{
	set_chip(row, addr, 1);
	current_hint = hint;
	struct i2c_dt_spec bus = {.bus = &host_i2c_bus, .addr = preferred_addr};
	uint8_t reg = preferred_reg;
	int result = sensor_scan_imu(&bus, &reg);
	check_selected(row, result);
	if (row->detectable) {
		CHECK(bus.addr == addr);
		CHECK(reg == row->reg);
		CHECK(chip.target_reads > 0);
	} else {
		CHECK(bus.addr == 0xFF);
		CHECK(reg == 0xFF);
	}
	/* In the full table other 0x00 rows remain, so this proves the BMI270
	 * chip ID was actually read rather than merely skipping that register. */
	if (row->enum_id == IMU_BMI270 && !IS_ENABLED(CONFIG_SENSOR_IMU_DRIVERS_MINIMAL))
		CHECK(chip.target_reads > 0);
}

static void scan_spi_case(const struct scan_row *row, unsigned id_offset, uint8_t preferred_reg, const char *hint)
{
	set_chip(row, row->first_addr, id_offset);
	current_hint = hint;
	uint8_t reg = preferred_reg;
	int result = sensor_scan_imu_spi(&host_spi_bus, &reg);
	check_selected(row, result);
	if (row->detectable) {
		CHECK(reg == (uint8_t)(row->reg | 0x80));
		CHECK(chip.target_reads > 0);
	} else {
		CHECK(reg == 0xFF);
	}
	if (row->enum_id == IMU_BMI270 && !IS_ENABLED(CONFIG_SENSOR_IMU_DRIVERS_MINIMAL))
		CHECK(chip.target_reads > 0);
}

static void check_scans(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(rows); ++i) {
		const struct scan_row *row = &rows[i];
		for (unsigned offset = 0; offset < 2; ++offset) {
			uint16_t addr = row->first_addr + offset;
			uint16_t wrong_addr = addr == 0x68 ? 0x69 : 0x68;
			scan_i2c_case(row, addr, 0, 0xFF, "I2C full scan");
			scan_i2c_case(row, addr, addr, row->reg, "I2C retained hint");
			scan_i2c_case(row, addr, wrong_addr, 0x00, "I2C stale hint and fallback");
		}
		/* Exercise both the first-byte and the dummy-byte SPI responses. */
		for (unsigned id_offset = 1; id_offset <= 2; ++id_offset) {
			scan_spi_case(row, id_offset, 0xFF, "SPI full scan");
			scan_spi_case(row, id_offset, row->reg | 0x80, "SPI retained hint");
			scan_spi_case(row, id_offset, 0x81, "SPI stale hint and fallback");
		}
	}
}

static void check_spi_group_dedup(void)
{
	const uint8_t registers[] = {2, 0x00, 0x0f, 1, 0x00, 1, 0x0f, 1, 0x4f};
	const uint8_t ids[] = {2, 0x08, 0x90, 1, 0x3d, 1, 0xa3, 1, 0x42, 0};
	const int devices[] = {101, 102, 103, 104, 105};
	const struct scan_row cases[] = {
		{"first shared ID", 0, 0, 0x08, 101, true},
		{"second shared ID", 0, 0, 0x90, 102, true},
		{"later register ID", 0, 0x0f, 0x3d, 103, true},
		{"later-only ID", 0, 0, 0xa3, 104, true},
		{"later duplicate-register ID", 0, 0x0f, 0x42, 105, true},
		{"unmatched duplicate register", 0, 0, 0x77, -1, false},
	};
	for (unsigned offset = 1; offset <= 2; offset++) {
		for (unsigned i = 0; i < ARRAY_SIZE(cases); i++) {
			set_chip(&cases[i], 0, offset);
			uint8_t reg = 0xff;
			CHECK(sensor_scan_spi(&host_spi_bus, &reg, 4, registers, ids, devices) == cases[i].enum_id);
			CHECK(chip.target_reads == 1);
			CHECK(chip.spi_calls <= 2);
			if (cases[i].detectable)
				CHECK(reg == (cases[i].reg | 0x80));
		}
	}
	set_chip(&cases[0], 0, 1);
	chip.fail_spi = true;
	uint8_t reg = 0xff;
	CHECK(sensor_scan_spi(&host_spi_bus, &reg, 4, registers, ids, devices) == -1);
	CHECK(chip.target_reads == 1 && chip.spi_calls == 2);
}

int main(void)
{
	check_spi_group_dedup();
	check_identities();
	check_scans();
	printf("scan removal: %zu identities, %zu chip rows, I2C and SPI passed\n",
		ARRAY_SIZE(identities), ARRAY_SIZE(rows));
	return 0;
}
