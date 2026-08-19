#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#include "sensor/mag/AK09940.h"
#include "sensor/mag/BMM150.h"
#include "sensor/mag/BMM350.h"
#include "sensor/mag/IST8306.h"
#include "sensor/mag/IST8308.h"
#include "sensor/mag/LIS2MDL.h"
#include "sensor/mag/LIS3MDL.h"
#include "sensor/mag/MMC5603NJ.h"
#include "sensor/mag/MMC5983MA.h"
#include "sensor/mag/QMC5883L.h"
#include "sensor/mag/QMC6309.h"

enum fake_op_type {
	FAKE_REG_WRITE,
	FAKE_REG_READ,
	FAKE_BURST_READ,
	FAKE_BURST_READ_DUMMY,
	FAKE_BURST_WRITE,
	FAKE_REG_UPDATE,
};

struct fake_op {
	enum fake_op_type type;
	uint8_t reg;
	uint8_t value;
	uint8_t mask;
	uint32_t length;
};

#define MAX_FAKE_OPS 512

struct fake_bus {
	struct fake_op ops[MAX_FAKE_OPS];
	size_t op_count;
	size_t transaction_count;
	size_t fail_transaction;
	uint8_t registers[256];
	int64_t now_ms;
	uint16_t bmm350_otp[32];
};

static struct fake_bus bus;
static enum sensor_interface_spec fake_spec = SENSOR_INTERFACE_SPEC_I2C;

#define CHECK(condition) do { \
	if (!(condition)) { \
		fprintf(stderr, "%s:%d: check failed: %s\n", __func__, __LINE__, #condition); \
		return 1; \
	} \
} while (0)

static void fake_bus_reset(void)
{
	memset(&bus, 0, sizeof(bus));
	bus.now_ms = 1;
	fake_spec = SENSOR_INTERFACE_SPEC_I2C;
}

static void fake_bus_clear_transactions(void)
{
	bus.op_count = 0;
	bus.transaction_count = 0;
	bus.fail_transaction = 0;
}

static bool fake_bus_record(
	enum fake_op_type type,
	uint8_t reg,
	uint8_t value,
	uint8_t mask,
	uint32_t length
)
{
	if (bus.op_count >= MAX_FAKE_OPS) {
		fprintf(stderr, "fake bus operation log overflow\n");
		abort();
	}
	bus.ops[bus.op_count++] = (struct fake_op) {
		.type = type,
		.reg = reg,
		.value = value,
		.mask = mask,
		.length = length,
	};
	bus.transaction_count++;
	return bus.fail_transaction == bus.transaction_count;
}

int64_t host_k_uptime_get(void)
{
	return bus.now_ms++;
}

void host_k_busy_wait(uint32_t usec)
{
	bus.now_ms += (usec + 999U) / 1000U;
}

void host_k_msleep(int32_t msec)
{
	bus.now_ms += msec;
}

void host_k_usleep(int32_t usec)
{
	bus.now_ms += (usec + 999) / 1000;
}

int ssi_reg_write_byte(enum sensor_interface_dev dev, uint8_t reg_addr, uint8_t value)
{
	(void)dev;
	if (fake_bus_record(FAKE_REG_WRITE, reg_addr, value, 0, 1))
		return -EIO;
	/* Model the self-clearing reset bit needed by LIS2MDL shutdown/init. */
	if (reg_addr == BMM350_PMU_CMD)
		bus.registers[BMM350_PMU_CMD_STATUS_0] = (uint8_t)(value << 5);
	if (reg_addr == BMM350_OTP_CMD_REG && (value & 0xe0) == BMM350_OTP_CMD_DIR_READ) {
		uint16_t word = bus.bmm350_otp[value & 0x1f];
		bus.registers[BMM350_OTP_DATA_MSB_REG] = (uint8_t)(word >> 8);
		bus.registers[BMM350_OTP_DATA_LSB_REG] = (uint8_t)word;
		bus.registers[BMM350_OTP_STATUS_REG] = BMM350_OTP_STATUS_CMD_DONE;
	}
	bus.registers[reg_addr] =
		(reg_addr == LIS2MDL_CFG_REG_A && value == CFG_A_SOFT_RST) ? 0 : value;
	return 0;
}

int ssi_reg_read_byte(enum sensor_interface_dev dev, uint8_t reg_addr, uint8_t *value)
{
	(void)dev;
	if (fake_bus_record(FAKE_REG_READ, reg_addr, 0, 0, 1))
		return -EIO;
	*value = bus.registers[reg_addr];
	return 0;
}

int ssi_burst_read(
	enum sensor_interface_dev dev,
	uint8_t start_addr,
	uint8_t *buf,
	uint32_t num_bytes
)
{
	(void)dev;
	if (fake_bus_record(FAKE_BURST_READ, start_addr, 0, 0, num_bytes))
		return -EIO;
	for (uint32_t i = 0; i < num_bytes; i++)
		buf[i] = bus.registers[(uint8_t)(start_addr + i)];
	return 0;
}

int ssi_burst_read_dummy(
	enum sensor_interface_dev dev,
	uint8_t start_addr,
	uint8_t dummy_bytes,
	uint8_t *buf,
	uint32_t num_bytes
)
{
	(void)dev;
	if (fake_bus_record(FAKE_BURST_READ_DUMMY, start_addr, bus.registers[start_addr], dummy_bytes, num_bytes))
		return -EIO;
	for (uint32_t i = 0; i < num_bytes; i++)
		buf[i] = bus.registers[(uint8_t)(start_addr + i)];
	return 0;
}

int ssi_burst_write(
	enum sensor_interface_dev dev,
	uint8_t start_addr,
	const uint8_t *buf,
	uint32_t num_bytes
)
{
	(void)dev;
	uint8_t first = num_bytes ? buf[0] : 0;
	if (fake_bus_record(FAKE_BURST_WRITE, start_addr, first, 0, num_bytes))
		return -EIO;
	for (uint32_t i = 0; i < num_bytes; i++)
		bus.registers[(uint8_t)(start_addr + i)] = buf[i];
	return 0;
}

int ssi_reg_update_byte(
	enum sensor_interface_dev dev,
	uint8_t reg_addr,
	uint8_t mask,
	uint8_t value
)
{
	(void)dev;
	if (fake_bus_record(FAKE_REG_UPDATE, reg_addr, value, mask, 1))
		return -EIO;
	bus.registers[reg_addr] = (bus.registers[reg_addr] & ~mask) | (value & mask);
	return 0;
}

enum sensor_interface_spec sensor_interface_get_spec(enum sensor_interface_dev dev)
{
	(void)dev;
	return fake_spec;
}

float mag_none_temp_read(float bias[3])
{
	(void)bias;
	return NAN;
}

struct expected_write {
	uint8_t reg;
	uint8_t value;
	enum fake_op_type type;
};
#define EXPECT_WRITE(reg_, value_) { .reg = (reg_), .value = (value_), .type = FAKE_REG_WRITE }
#define EXPECT_READ(reg_, value_) { .reg = (reg_), .value = (value_), .type = FAKE_BURST_READ_DUMMY }

struct odr_case {
	const char *name;
	int (*update_odr)(float, float *);
	void (*oneshot)(void);
	void (*shutdown)(void);
	float requested_time;
	float actual_time;
	const struct expected_write *finite_writes;
	size_t finite_write_count;
	bool oneshot_writes;
	bool oneshot_invalidates;
};

static const struct expected_write ak_writes[] = {
	EXPECT_WRITE(AK09940_CNTL3, (MT_LND2 << 5) | MODE_CMM4_100Hz),
};
static const struct expected_write bmm150_writes[] = {
	EXPECT_WRITE(BMM150_OP_CTRL, (DR_ODR_25Hz << 2 | OPMODE_NORMAL) << 1),
	EXPECT_WRITE(BMM150_REP_XY, 4),
	EXPECT_WRITE(BMM150_REP_Z, 14),
};
static const struct expected_write bmm350_writes[] = {
	EXPECT_WRITE(BMM350_PMU_CMD_AGGR_SET, (AGGR_AVG_4 << 4) | AGGR_ODR_100Hz),
	EXPECT_WRITE(BMM350_PMU_CMD, PMU_CMD_UPD_OAE),
	EXPECT_READ(BMM350_PMU_CMD_STATUS_0, PMU_CMD_UPD_OAE << 5),
	EXPECT_WRITE(BMM350_PMU_CMD, PMU_CMD_SUS),
	EXPECT_READ(BMM350_PMU_CMD_STATUS_0, PMU_CMD_SUS << 5),
	EXPECT_WRITE(BMM350_PMU_CMD, PMU_CMD_NM),
	EXPECT_READ(BMM350_PMU_CMD_STATUS_0, PMU_CMD_NM << 5),
};
static const struct expected_write ist_writes[] = {
	EXPECT_WRITE(IST8306_CNTL1, NSF_Low << 5),
	EXPECT_WRITE(IST8306_CNTL2, MODE_CMM_100Hz),
	EXPECT_WRITE(IST8306_OSRCNTL, OSR_16),
};
static const struct expected_write lis2_writes[] = {
	EXPECT_WRITE(LIS2MDL_CFG_REG_C, CFG_C_BDU),
	EXPECT_WRITE(LIS2MDL_CFG_REG_A, CFG_A_COMP_TEMP_EN | (ODR_100Hz << 2) | MD_CONTINUOUS),
};
static const struct expected_write lis3_writes[] = {
	EXPECT_WRITE(LIS3MDL_CTRL_REG1, 0x80 | (OM_UHP << 5) | (1 << 1)),
	EXPECT_WRITE(LIS3MDL_CTRL_REG3, MD_CONTINUOUS_CONV),
	EXPECT_WRITE(LIS3MDL_CTRL_REG4, OM_UHP << 2),
	EXPECT_WRITE(LIS3MDL_CTRL_REG5, LIS3MDL_CTRL_REG5_BDU),
};
static const struct expected_write mmc5603_writes[] = {
	EXPECT_WRITE(MMC5603NJ_CONTROL_1, MBW_3_5ms),
	EXPECT_WRITE(MMC5603NJ_ODR, 150),
	EXPECT_WRITE(MMC5603NJ_CONTROL_0, MCTRL0_AUTO_SR_EN | MCTRL0_CMM_FREQ_EN),
	EXPECT_WRITE(MMC5603NJ_CONTROL_2, MCTRL2_CMM_EN | MCTRL2_EN_PRD_SET | MSET_2000),
};
static const struct expected_write mmc5983_writes[] = {
	EXPECT_WRITE(MMC5983MA_CONTROL_1, MBW_400Hz),
	EXPECT_WRITE(MMC5983MA_CONTROL_2, 0x80 | (MSET_2000 << 4) | 0x08 | MODR_100Hz),
};
static const struct expected_write qmc5883_writes[] = {
	EXPECT_WRITE(0x09, 0x19),
};
static const struct expected_write qmc6309_writes[] = {
	EXPECT_WRITE(0x0B, 0x38),
	EXPECT_WRITE(0x0A, 0x41),
};

static const struct odr_case odr_cases[] = {
	{"AK09940", ak_update_odr, ak_mag_oneshot, ak_shutdown, 0.012f, 1.0f / 100, ak_writes, ARRAY_SIZE(ak_writes), true, true},
	{"BMM150", bmm1_update_odr, bmm1_mag_oneshot, bmm1_shutdown, 0.04f, 1.0f / 25, bmm150_writes, ARRAY_SIZE(bmm150_writes), true, true},
	{"BMM350", bmm3_update_odr, bmm3_mag_oneshot, bmm3_shutdown, 0.012f, 1.0f / 100, bmm350_writes, ARRAY_SIZE(bmm350_writes), true, true},
	{"IST8306", ist8306_update_odr, ist8306_mag_oneshot, ist8306_shutdown, 0.012f, 1.0f / 100, ist_writes, ARRAY_SIZE(ist_writes), true, true},
	{"IST8308", ist8308_update_odr, ist8308_mag_oneshot, ist8308_shutdown, 0.012f, 1.0f / 100, ist_writes, ARRAY_SIZE(ist_writes), true, true},
	{"LIS2MDL", lis2_update_odr, lis2_mag_oneshot, lis2_shutdown, 0.012f, 1.0f / 100, lis2_writes, ARRAY_SIZE(lis2_writes), false, false},
	{"LIS3MDL", lis3_update_odr, lis3_mag_oneshot, lis3_shutdown, 0.012f, 1.0f / 155, lis3_writes, ARRAY_SIZE(lis3_writes), true, true},
	{"MMC5603NJ", mmc5603_update_odr, mmc5603_mag_oneshot, mmc5603_shutdown, 0.012f, 1.0f / 150, mmc5603_writes, ARRAY_SIZE(mmc5603_writes), true, true},
	{"MMC5983MA", mmc_update_odr, mmc_mag_oneshot, mmc_shutdown, 0.012f, 1.0f / 100, mmc5983_writes, ARRAY_SIZE(mmc5983_writes), true, true},
	{"QMC5883L", qmc5883l_update_odr, qmc5883l_mag_oneshot, qmc5883l_shutdown, 0.012f, 1.0f / 100, qmc5883_writes, ARRAY_SIZE(qmc5883_writes), true, true},
	{"QMC6309", qmc_update_odr, qmc_mag_oneshot, qmc_shutdown, 0.012f, 1.0f / 100, qmc6309_writes, ARRAY_SIZE(qmc6309_writes), true, true},
};

static bool float_equal(float a, float b)
{
	return fabsf(a - b) < 0.000001f;
}

static int check_write_sequence(const struct expected_write *writes, size_t count)
{
	CHECK(bus.op_count == count);
	for (size_t i = 0; i < count; i++) {
		if (bus.ops[i].type != writes[i].type || bus.ops[i].reg != writes[i].reg || bus.ops[i].value != writes[i].value)
			fprintf(stderr, "op[%zu] got type=%d reg=%02x value=%02x expected type=%d reg=%02x value=%02x\n",
				i, bus.ops[i].type, bus.ops[i].reg, bus.ops[i].value,
				writes[i].type, writes[i].reg, writes[i].value);
		CHECK(bus.ops[i].type == writes[i].type);
		CHECK(bus.ops[i].reg == writes[i].reg);
		CHECK(bus.ops[i].value == writes[i].value);
		CHECK(bus.ops[i].length == 1);
	}
	return 0;
}

static void reset_driver(const struct odr_case *test)
{
	/* shutdown invalidates static driver cache; do not retain its bus side effects. */
	fake_bus_reset();
	test->shutdown();
	fake_bus_reset();
}

static int test_fixed_odr_case(const struct odr_case *test)
{
	reset_driver(test);
	float actual = -1.0f;
	CHECK(test->update_odr(test->requested_time, &actual) == 0);
	CHECK(float_equal(actual, test->actual_time));
	CHECK(check_write_sequence(test->finite_writes, test->finite_write_count) == 0);

	fake_bus_clear_transactions();
	actual = -2.0f;
	CHECK(test->update_odr(test->requested_time, &actual) == 0);
	CHECK(float_equal(actual, test->actual_time));
	CHECK(bus.transaction_count == 0);

	fake_bus_clear_transactions();
	CHECK(test->update_odr(0.0f, &actual) == 0);
	CHECK(bus.transaction_count > 0);
	fake_bus_clear_transactions();
	CHECK(test->update_odr(test->requested_time, &actual) == 0);
	CHECK(float_equal(actual, test->actual_time));
	CHECK(check_write_sequence(test->finite_writes, test->finite_write_count) == 0);

	fake_bus_clear_transactions();
	test->oneshot();
	if (test->oneshot_writes)
		CHECK(bus.transaction_count == 1);
	else
		CHECK(bus.transaction_count == 0);
	if (strcmp(test->name, "BMM150") == 0) {
		CHECK(bus.ops[0].type == FAKE_REG_UPDATE);
		CHECK(bus.ops[0].reg == BMM150_OP_CTRL);
		CHECK(bus.ops[0].mask == 0x06);
		CHECK(bus.ops[0].value == (OPMODE_FORCED << 1));
	} else if (test->oneshot_writes) {
		CHECK(bus.ops[0].type == FAKE_REG_WRITE);
		CHECK(bus.ops[0].length == 1);
	}
	fake_bus_clear_transactions();
	CHECK(test->update_odr(test->requested_time, &actual) == 0);
	CHECK(float_equal(actual, test->actual_time));
	if (test->oneshot_invalidates)
		CHECK(check_write_sequence(test->finite_writes, test->finite_write_count) == 0);
	else
		CHECK(bus.transaction_count == 0);

	for (size_t fail_at = 1; fail_at <= test->finite_write_count; fail_at++) {
		reset_driver(test);
		/* Establish another valid cached configuration before the failed change. */
		CHECK(test->update_odr(0.1f, &actual) == 0);
		fake_bus_clear_transactions();
		actual = -123.0f;
		bus.fail_transaction = fail_at;
		CHECK(test->update_odr(test->requested_time, &actual) < 0);
		CHECK(bus.transaction_count == fail_at);
		CHECK(actual == -123.0f);

		/* Failure invalidates even the previously valid cache entry. */
		fake_bus_clear_transactions();
		CHECK(test->update_odr(0.1f, &actual) == 0);
		CHECK(bus.transaction_count == test->finite_write_count);

		fake_bus_clear_transactions();
		CHECK(test->update_odr(test->requested_time, &actual) == 0);
		CHECK(float_equal(actual, test->actual_time));
		CHECK(check_write_sequence(test->finite_writes, test->finite_write_count) == 0);
	}
	return 0;
}

static int test_all_fixed_odr_transactions(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(odr_cases); i++) {
		if (test_fixed_odr_case(&odr_cases[i])) {
			fprintf(stderr, "fixed-ODR matrix failed for %s\n", odr_cases[i].name);
			return 1;
		}
	}
	return 0;
}

static size_t count_ops(enum fake_op_type type)
{
	size_t count = 0;
	for (size_t i = 0; i < bus.op_count; i++)
		count += bus.ops[i].type == type;
	return count;
}

static int test_lis3_oneshot_timeout_stops_before_data(void)
{
	reset_driver(&odr_cases[6]);
	float actual_time;
	CHECK(lis3_update_odr(INFINITY, &actual_time) == 0);
	lis3_mag_oneshot();
	bus.registers[LIS3MDL_CTRL_REG3] = MD_SINGLE_CONV;
	fake_bus_clear_transactions();
	float m[3] = {1.0f, 2.0f, 3.0f};
	CHECK(!lis3_mag_read(m));
	CHECK(count_ops(FAKE_BURST_READ) == 0);
	CHECK(m[0] == 1.0f && m[1] == 2.0f && m[2] == 3.0f);
	return 0;
}

static int test_lis_temperatures_fail_loudly_and_apply_offset(void)
{
	fake_bus_reset();
	float bias[3] = {0};
	bus.fail_transaction = 1;
	CHECK(isnan(lis2_temp_read(bias)));

	fake_bus_reset();
	bus.fail_transaction = 1;
	CHECK(isnan(lis3_temp_read(bias)));

	fake_bus_reset();
	bus.registers[LIS2MDL_TEMP_OUT_L_REG] = 0x08;
	CHECK(lis2_temp_read(bias) == 26.0f);

	fake_bus_reset();
	bus.registers[LIS3MDL_TEMP_OUT_L] = 0xf8;
	bus.registers[LIS3MDL_TEMP_OUT_L + 1] = 0xff;
	CHECK(lis3_temp_read(bias) == 24.0f);
	return 0;
}

static int test_lis3_combines_status_and_data(void)
{
	float actual;
	float m[3] = {1.0f, 2.0f, 3.0f};
	reset_driver(&odr_cases[6]);
	CHECK(lis3_update_odr(0.012f, &actual) == 0);
	fake_bus_clear_transactions();
	CHECK(!lis3_mag_read(m));
	CHECK(bus.op_count == 1 && bus.ops[0].type == FAKE_BURST_READ);
	CHECK(bus.ops[0].reg == LIS3MDL_STATUS_REG && bus.ops[0].length == 7);
	CHECK(m[0] == 1.0f && m[1] == 2.0f && m[2] == 3.0f);
	bus.registers[LIS3MDL_STATUS_REG] = LIS3MDL_STATUS_ZYXDA;
	bus.registers[LIS3MDL_OUT_X_L] = 1;
	fake_bus_clear_transactions();
	CHECK(lis3_mag_read(m));
	CHECK(bus.op_count == 1 && bus.ops[0].type == FAKE_BURST_READ);
	CHECK(fabsf(m[0] - 1.0f / 3421.0f) < 0.000001f);
	fake_spec = SENSOR_INTERFACE_SPEC_EXT;
	bus.registers[LIS3MDL_OUT_X_L] = 2;
	fake_bus_clear_transactions();
	CHECK(lis3_mag_read(m));
	CHECK(bus.op_count == 1 && bus.ops[0].reg == LIS3MDL_STATUS_REG && bus.ops[0].length == 7);
	return 0;
}

static int test_status_plus_data_drivers_publish_only_ready(void)
{
	float actual;
	float m[3] = {1.0f, 2.0f, 3.0f};
	struct {
		const struct odr_case *odr;
		bool (*read_mag)(float[3]);
		uint8_t status_reg;
		uint8_t data_reg;
	} cases[] = {
		{&odr_cases[3], ist8306_mag_read, IST8306_STAT, IST8306_DATAXL},
		{&odr_cases[4], ist8308_mag_read, IST8306_STAT, IST8306_DATAXL},
		{&odr_cases[5], lis2_mag_read, LIS2MDL_STATUS_REG, LIS2MDL_OUTX_L_REG},
	};
	for (size_t i = 0; i < ARRAY_SIZE(cases); i++) {
		reset_driver(cases[i].odr);
		CHECK(cases[i].odr->update_odr(cases[i].odr->requested_time, &actual) == 0);
		fake_bus_clear_transactions();
		CHECK(!cases[i].read_mag(m));
		CHECK(bus.op_count == 1 && bus.ops[0].type == FAKE_BURST_READ);
		CHECK(bus.ops[0].reg == cases[i].status_reg && bus.ops[0].length == 7);
		bus.registers[cases[i].status_reg] = 0x09;
		bus.registers[cases[i].data_reg] = (uint8_t)(i + 1);
		fake_bus_clear_transactions();
		CHECK(cases[i].read_mag(m));
		CHECK(bus.op_count == 1 && bus.ops[0].type == FAKE_BURST_READ);
		fake_spec = SENSOR_INTERFACE_SPEC_EXT;
		bus.registers[cases[i].data_reg]++;
		fake_bus_clear_transactions();
		CHECK(cases[i].read_mag(m));
		CHECK(bus.op_count == 1 && bus.ops[0].reg == cases[i].status_reg && bus.ops[0].length == 7);
	}
	return 0;
}

static int test_bmm150_direct_status_gate(void)
{
	float actual;
	float m[3] = {1.0f, 2.0f, 3.0f};
	reset_driver(&odr_cases[1]);
	CHECK(bmm1_update_odr(0.04f, &actual) == 0);
	fake_bus_clear_transactions();
	CHECK(!bmm1_mag_read(m));
	CHECK(bus.op_count == 1 && bus.ops[0].type == FAKE_REG_READ);
	CHECK(count_ops(FAKE_BURST_READ) == 0);
	bus.registers[BMM150_DATA_READY_STATUS] = BMM150_DATA_READY_MASK;
	CHECK(bmm1_mag_read(m));
	CHECK(count_ops(FAKE_BURST_READ) == 1);
	fake_spec = SENSOR_INTERFACE_SPEC_EXT;
	bus.registers[BMM150_DATAX_LSB]++;
	fake_bus_clear_transactions();
	CHECK(bmm1_mag_read(m));
	CHECK(bus.op_count == 1 && bus.ops[0].type == FAKE_BURST_READ);
	return 0;
}

static bool contains_write(uint8_t reg, uint8_t value);
static int test_bmm350_init_and_compensation(void)
{
	fake_bus_reset();
	bus.registers[BMM350_CHIP_ID_REG] = BMM350_CHIP_ID;
	/* Deterministic OTP vector exercises signed offset/sensitivity/cross-axis parsing. */
	bus.bmm350_otp[0x0d] = 0x0205;
	bus.bmm350_otp[0x0e] = 0xf064;
	bus.bmm350_otp[0x0f] = 0x0320;
	bus.bmm350_otp[0x10] = 0x0104;
	bus.bmm350_otp[0x11] = 0xfe02;
	bus.bmm350_otp[0x12] = 0x0102;
	bus.bmm350_otp[0x13] = 0xfffe;
	bus.bmm350_otp[0x14] = 0x0201;
	bus.bmm350_otp[0x15] = 0x01ff;
	bus.bmm350_otp[0x16] = 0xff01;
	bus.bmm350_otp[0x18] = 0x0200;
	float actual = -1.0f;
	CHECK(bmm3_init(0.01f, &actual) == 0);
	CHECK(float_equal(actual, 0.01f));
	CHECK(bus.ops[0].type == FAKE_REG_WRITE && bus.ops[0].reg == BMM350_CMD);
	CHECK(bus.ops[0].value == BMM350_CMD_SOFTRESET);
	CHECK(contains_write(BMM350_OTP_CMD_REG, BMM350_OTP_CMD_PWR_OFF));
	CHECK(contains_write(BMM350_PMU_CMD_AXIS_EN, BMM350_AXIS_EN_XYZ));
	CHECK(contains_write(BMM350_INT_CTRL, 0x86));
	CHECK(contains_write(BMM350_PMU_CMD, PMU_CMD_UPD_OAE));
	CHECK(contains_write(BMM350_PMU_CMD, PMU_CMD_NM));

	bus.registers[BMM350_INT_STATUS] = BMM350_DRDY_DATA_REG;
	bus.registers[BMM350_MAG_X_XLSB] = 1;
	bus.registers[BMM350_TEMP_XLSB] = 0;
	float m[3] = {0};
	CHECK(bmm3_mag_read(m));
	CHECK(isfinite(m[0]) && isfinite(m[1]) && isfinite(m[2]));
	CHECK(fabsf(m[0]) > 0.1f);
	CHECK(fabsf(m[0] - 0.9698315f) < 0.00001f);
	CHECK(fabsf(m[1] + 2.2043101f) < 0.00001f);
	CHECK(fabsf(m[2] - 7.7091193f) < 0.00001f);
	CHECK(fabsf(bmm3_temp_read(NULL) + 24.5895703f) < 0.00001f);
	fake_bus_reset();
	bus.registers[BMM350_CHIP_ID_REG] = BMM350_CHIP_ID;
	bus.fail_transaction = 1;
	actual = -123.0f;
	CHECK(bmm3_init(0.01f, &actual) < 0);
	CHECK(actual == -123.0f);
	CHECK(!bmm3_mag_read(m));
	return 0;
}

static int test_mmc_timeouts_stop_before_data(void)
{
	float actual_time;
	float m[3] = {1.0f, 2.0f, 3.0f};

	reset_driver(&odr_cases[7]);
	CHECK(mmc5603_update_odr(INFINITY, &actual_time) == 0);
	mmc5603_mag_oneshot();
	fake_bus_clear_transactions();
	CHECK(!mmc5603_mag_read(m));
	CHECK(count_ops(FAKE_BURST_READ) == 0);
	CHECK(m[0] == 1.0f && m[1] == 2.0f && m[2] == 3.0f);

	reset_driver(&odr_cases[8]);
	CHECK(mmc_update_odr(INFINITY, &actual_time) == 0);
	mmc_mag_oneshot();
	fake_bus_clear_transactions();
	CHECK(!mmc_mag_read(m));
	CHECK(count_ops(FAKE_BURST_READ) == 0);
	CHECK(m[0] == 1.0f && m[1] == 2.0f && m[2] == 3.0f);
	return 0;
}

static int test_mmc_temperature_failures_do_not_publish_bias(void)
{
	float bias[3] = {4.0f, 5.0f, 6.0f};
	fake_bus_reset();
	bus.fail_transaction = 1;
	CHECK(isnan(mmc5603_temp_read(bias)));
	CHECK(bias[0] == 4.0f && bias[1] == 5.0f && bias[2] == 6.0f);

	fake_bus_reset();
	bus.fail_transaction = 1;
	CHECK(isnan(mmc_temp_read(bias)));
	CHECK(bias[0] == 4.0f && bias[1] == 5.0f && bias[2] == 6.0f);
	return 0;
}

static int test_qmc_timeouts_stop_before_data(void)
{
	float actual_time;
	float m[3] = {1.0f, 2.0f, 3.0f};

	reset_driver(&odr_cases[9]);
	CHECK(qmc5883l_update_odr(INFINITY, &actual_time) == 0);
	qmc5883l_mag_oneshot();
	fake_bus_clear_transactions();
	CHECK(!qmc5883l_mag_read(m));
	CHECK(count_ops(FAKE_BURST_READ) == 0);
	CHECK(m[0] == 1.0f && m[1] == 2.0f && m[2] == 3.0f);

	reset_driver(&odr_cases[10]);
	CHECK(qmc_update_odr(INFINITY, &actual_time) == 0);
	qmc_mag_oneshot();
	fake_bus_clear_transactions();
	CHECK(!qmc_mag_read(m));
	CHECK(count_ops(FAKE_BURST_READ) == 0);
	CHECK(m[0] == 1.0f && m[1] == 2.0f && m[2] == 3.0f);
	return 0;
}

static int test_qmc_trigger_and_burst_failures_are_not_published(void)
{
	float actual_time;
	float m[3] = {1.0f, 2.0f, 3.0f};
	reset_driver(&odr_cases[9]);
	CHECK(qmc5883l_update_odr(INFINITY, &actual_time) == 0);
	fake_bus_clear_transactions();
	bus.fail_transaction = 1;
	qmc5883l_mag_oneshot();
	bus.registers[0x06] = 0x01;
	CHECK(!qmc5883l_mag_read(m));
	CHECK(count_ops(FAKE_BURST_READ) == 0);

	reset_driver(&odr_cases[9]);
	CHECK(qmc5883l_update_odr(0.02f, &actual_time) == 0);
	fake_bus_clear_transactions();
	bus.fail_transaction = 1;
	CHECK(!qmc5883l_mag_read(m));
	CHECK(m[0] == 1.0f && m[1] == 2.0f && m[2] == 3.0f);
	return 0;
}

static int test_qmc_oneshot_overflow_is_not_published(void)
{
	float actual_time;
	float m[3] = {1.0f, 2.0f, 3.0f};
	reset_driver(&odr_cases[9]);
	CHECK(qmc5883l_update_odr(INFINITY, &actual_time) == 0);
	qmc5883l_mag_oneshot();
	bus.registers[0x06] = 0x03;
	CHECK(!qmc5883l_mag_read(m));
	CHECK(count_ops(FAKE_BURST_READ) == 0);

	reset_driver(&odr_cases[10]);
	CHECK(qmc_update_odr(INFINITY, &actual_time) == 0);
	qmc_mag_oneshot();
	bus.registers[0x09] = 0x03;
	CHECK(!qmc_mag_read(m));
	CHECK(count_ops(FAKE_BURST_READ) == 0);
	return 0;
}

static int test_ak09940_rejects_invalid_frames(void)
{
	float actual_time;
	float m[3] = {1.0f, 2.0f, 3.0f};
	reset_driver(&odr_cases[0]);
	CHECK(ak_update_odr(0.01f, &actual_time) == 0);

	fake_bus_clear_transactions();
	CHECK(!ak_mag_read(m));
	CHECK(bus.op_count == 1 && bus.ops[0].type == FAKE_BURST_READ);
	CHECK(bus.ops[0].reg == AK09940_ST1 && bus.ops[0].length == 12);
	CHECK(m[0] == 1.0f && m[1] == 2.0f && m[2] == 3.0f);

	bus.registers[AK09940_ST1] = AK09940_ST1_DRDY;
	bus.registers[AK09940_ST2] = AK09940_ST2_INV;
	CHECK(!ak_mag_read(m));
	bus.registers[AK09940_ST2] = AK09940_ST2_DOR;
	CHECK(!ak_mag_read(m));

	bus.registers[AK09940_ST2] = 0;
	bus.registers[AK09940_HXL] = 0xff;
	bus.registers[AK09940_HXL + 1] = 0xff;
	bus.registers[AK09940_HXL + 2] = 0x01;
	CHECK(!ak_mag_read(m));

	uint8_t negative_18bit[9] = {0x00, 0x00, 0x02};
	ak_mag_process(negative_18bit, m);
	CHECK(fabsf(m[0] + 13.1072f) < 0.0001f);
	uint8_t minus_one[9] = {0xff, 0xff, 0x03};
	ak_mag_process(minus_one, m);
	CHECK(fabsf(m[0] + 0.0001f) < 0.000001f);
	return 0;
}

static int test_qmc_direct_status_gate_and_variant_odr(void)
{
	float actual = -1.0f;
	float m[3] = {1.0f, 2.0f, 3.0f};

	reset_driver(&odr_cases[10]);
	qmc_set_variant(false);
	CHECK(qmc_update_odr(0.25f, &actual) == 0);
	CHECK(float_equal(actual, 0.1f));
	CHECK(bus.ops[0].type == FAKE_REG_WRITE && bus.ops[0].reg == 0x0B && bus.ops[0].value == 0x18);

	fake_bus_clear_transactions();
	qmc_set_variant(true);
	CHECK(qmc_update_odr(0.25f, &actual) == 0);
	CHECK(float_equal(actual, 1.0f / 50));
	CHECK(bus.ops[0].type == FAKE_REG_WRITE && bus.ops[0].reg == 0x0B && bus.ops[0].value == 0x28);
	fake_bus_clear_transactions();
	CHECK(!qmc_mag_read(m));
	CHECK(bus.op_count == 1 && bus.ops[0].type == FAKE_REG_READ);
	CHECK(count_ops(FAKE_BURST_READ) == 0);
	bus.registers[0x09] = 0x02;
	CHECK(!qmc_mag_read(m));
	CHECK(count_ops(FAKE_BURST_READ) == 0);
	bus.registers[0x09] = 0x01;
	CHECK(qmc_mag_read(m));
	CHECK(count_ops(FAKE_BURST_READ) == 1);

	bus.registers[0x01] = 1;
	fake_spec = SENSOR_INTERFACE_SPEC_EXT;
	fake_bus_clear_transactions();
	CHECK(qmc_mag_read(m));
	CHECK(bus.op_count == 1 && bus.ops[0].type == FAKE_BURST_READ);

	reset_driver(&odr_cases[9]);
	CHECK(qmc5883l_update_odr(0.02f, &actual) == 0);
	fake_bus_clear_transactions();
	CHECK(!qmc5883l_mag_read(m));
	CHECK(count_ops(FAKE_BURST_READ) == 0);
	bus.registers[0x06] = 0x02;
	CHECK(!qmc5883l_mag_read(m));
	CHECK(count_ops(FAKE_BURST_READ) == 0);
	bus.registers[0x06] = 0x01;
	CHECK(qmc5883l_mag_read(m));
	CHECK(count_ops(FAKE_BURST_READ) == 1);

	bus.registers[0x00] = 2;
	fake_spec = SENSOR_INTERFACE_SPEC_EXT;
	fake_bus_clear_transactions();
	CHECK(qmc5883l_mag_read(m));
	CHECK(bus.op_count == 1 && bus.ops[0].type == FAKE_BURST_READ);
	return 0;
}


static int test_qmc_raw_decode_allows_unaligned_input(void)
{
	_Alignas(2) uint8_t raw[7] = {0, 0x34, 0x12, 0xcc, 0xff, 0x00, 0x80};
	float m[3];
	qmc5883l_mag_process(&raw[1], m);
	CHECK(fabsf(m[0] - (4660.0f / 3000.0f)) < 0.00001f);
	CHECK(fabsf(m[1] - (-52.0f / 3000.0f)) < 0.00001f);
	CHECK(fabsf(m[2] - (-32768.0f / 3000.0f)) < 0.00001f);
	qmc_mag_process(&raw[1], m);
	CHECK(fabsf(m[0] - (4660.0f / 4000.0f)) < 0.00001f);
	CHECK(fabsf(m[1] - (-52.0f / 4000.0f)) < 0.00001f);
	CHECK(fabsf(m[2] - (-32768.0f / 4000.0f)) < 0.00001f);
	return 0;
}

static bool contains_write(uint8_t reg, uint8_t value)
{
	for (size_t i = 0; i < bus.op_count; i++) {
		if (bus.ops[i].type == FAKE_REG_WRITE &&
			bus.ops[i].reg == reg && bus.ops[i].value == value)
			return true;
	}
	return false;
}

static int test_mmc5603_calibration_restores_control(void)
{
	const struct odr_case *test = &odr_cases[7];
	reset_driver(test);
	float actual;
	CHECK(mmc5603_update_odr(0.1f, &actual) == 0);
	bus.registers[MMC5603NJ_TOUT] = 42;
	bus.registers[MMC5603NJ_STATUS] = MSTAT_MEAS_M_DONE;
	fake_bus_clear_transactions();
	float bias[3] = {1.0f, 2.0f, 3.0f};
	CHECK(!isnan(mmc5603_temp_read(bias)));
	CHECK(contains_write(MMC5603NJ_ODR, 10));
	CHECK(contains_write(MMC5603NJ_CONTROL_0, MCTRL0_AUTO_SR_EN | MCTRL0_CMM_FREQ_EN));
	CHECK(contains_write(MMC5603NJ_CONTROL_2,
		MCTRL2_CMM_EN | MCTRL2_EN_PRD_SET | MSET_2000));
	CHECK(bus.ops[bus.op_count - 1].type == FAKE_REG_WRITE);
	CHECK(bus.ops[bus.op_count - 1].reg == MMC5603NJ_CONTROL_0);
	CHECK(bus.ops[bus.op_count - 1].value == (MCTRL0_AUTO_SR_EN | MCTRL0_TAKE_MEAS_T));
	fake_bus_clear_transactions();
	CHECK(mmc5603_update_odr(0.1f, &actual) == 0);
	CHECK(bus.transaction_count == 0);
	return 0;
}

static int test_mmc5983_calibration_restores_control(void)
{
	const struct odr_case *test = &odr_cases[8];
	reset_driver(test);
	float actual;
	CHECK(mmc_update_odr(0.1f, &actual) == 0);
	bus.registers[MMC5983MA_TOUT] = 42;
	bus.registers[MMC5983MA_STATUS] = 0x01;
	fake_bus_clear_transactions();
	float bias[3] = {1.0f, 2.0f, 3.0f};
	CHECK(!isnan(mmc_temp_read(bias)));
	CHECK(contains_write(MMC5983MA_CONTROL_1, MBW_100Hz));
	CHECK(contains_write(MMC5983MA_CONTROL_2,
		0x80 | (MSET_2000 << 4) | 0x08 | MODR_10Hz));
	CHECK(bus.ops[bus.op_count - 1].type == FAKE_REG_WRITE);
	CHECK(bus.ops[bus.op_count - 1].reg == MMC5983MA_CONTROL_0);
	CHECK(bus.ops[bus.op_count - 1].value == 0x22);
	fake_bus_clear_transactions();
	CHECK(mmc_update_odr(0.1f, &actual) == 0);
	CHECK(bus.transaction_count == 0);
	return 0;
}

static int test_ist8308_init_programs_drive_rate_first(void)
{
	fake_bus_reset();
	ist8308_shutdown();
	fake_bus_reset();
	float actual = -1.0f;
	CHECK(ist8308_init(0.012f, &actual) == 0);
	CHECK(float_equal(actual, 1.0f / 100));
	CHECK(bus.op_count == 4);
	CHECK(bus.ops[0].type == FAKE_REG_WRITE);
	CHECK(bus.ops[0].reg == 0x34);
	CHECK(bus.ops[0].value == DR_200);
	CHECK(bus.ops[1].reg == IST8306_CNTL1);
	CHECK(bus.ops[2].reg == IST8306_CNTL2);
	CHECK(bus.ops[3].reg == IST8306_OSRCNTL);
	return 0;
}

struct test_case {
	const char *name;
	int (*run)(void);
};

static int run_isolated(const struct test_case *test)
{
	pid_t child = fork();
	if (child < 0) {
		perror("fork");
		return 1;
	}
	if (child == 0) {
		alarm(2);
		int result = test->run();
		fflush(NULL);
		_exit(result == 0 ? EXIT_SUCCESS : EXIT_FAILURE);
	}

	int status;
	if (waitpid(child, &status, 0) < 0) {
		perror("waitpid");
		return 1;
	}
	if (WIFEXITED(status) && WEXITSTATUS(status) == EXIT_SUCCESS) {
		printf("PASS %s\n", test->name);
		return 0;
	}
	if (WIFSIGNALED(status))
		fprintf(stderr, "FAIL %s (signal %d)\n", test->name, WTERMSIG(status));
	else
		fprintf(stderr, "FAIL %s\n", test->name);
	return 1;
}

int main(void)
{
	static const struct test_case tests[] = {
		{"BMM350 init and compensation", test_bmm350_init_and_compensation},
		{"11-driver fixed-ODR transaction matrix", test_all_fixed_odr_transactions},
		{"IST8308 init drive-rate write", test_ist8308_init_programs_drive_rate_first},
		{"LIS3 oneshot timeout", test_lis3_oneshot_timeout_stops_before_data},
		{"LIS temperatures", test_lis_temperatures_fail_loudly_and_apply_offset},
		{"AK09940 frame validity", test_ak09940_rejects_invalid_frames},
		{"LIS3 combined status/data", test_lis3_combines_status_and_data},
		{"Combined status/data drivers", test_status_plus_data_drivers_publish_only_ready},
		{"BMM150 direct status", test_bmm150_direct_status_gate},
		{"MMC oneshot timeouts", test_mmc_timeouts_stop_before_data},
		{"MMC temperature read failures", test_mmc_temperature_failures_do_not_publish_bias},
		{"QMC oneshot timeouts", test_qmc_timeouts_stop_before_data},
		{"QMC direct status and variant ODR", test_qmc_direct_status_gate_and_variant_odr},
		{"QMC trigger and burst failures", test_qmc_trigger_and_burst_failures_are_not_published},
		{"QMC oneshot overflow", test_qmc_oneshot_overflow_is_not_published},
		{"QMC unaligned decode", test_qmc_raw_decode_allows_unaligned_input},
		{"MMC5603 calibration control restore", test_mmc5603_calibration_restores_control},
		{"MMC5983 calibration control restore", test_mmc5983_calibration_restores_control},
	};

	int failed = 0;
	for (size_t i = 0; i < ARRAY_SIZE(tests); i++)
		failed += run_isolated(&tests[i]);
	if (failed) {
		fprintf(stderr, "%d sensor driver tests failed\n", failed);
		return EXIT_FAILURE;
	}
	printf("All %zu sensor driver tests passed\n", ARRAY_SIZE(tests));
	return EXIT_SUCCESS;
}
