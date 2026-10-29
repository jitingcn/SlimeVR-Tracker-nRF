#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sensor/imu/LSM6DSM.h"

void host_bus_reset(void);
void host_bus_set_register(uint8_t, uint8_t);
uint8_t host_bus_register(uint8_t);
void host_bus_fail_transaction(size_t);
size_t host_bus_transaction_count(void);
void host_bus_fail_read(int);
unsigned host_bus_gyro_discontinuities(void);
void host_bus_dsm_fifo_load(const uint16_t *, size_t, uint16_t, uint16_t, uint8_t);

/* Distinct signed XYZ payloads expose repeated or skipped FIFO words. */
static void load_samples(unsigned start, unsigned count, unsigned cycle, uint8_t flags)
{
	uint16_t words[128 * 3];
	assert(count <= 128);
	for (unsigned i = 0; i < count; i++) {
		words[i * 3] = 0x4000;
		words[i * 3 + 1] = (uint16_t)(int16_t)(-100 - (int)i);
		words[i * 3 + 2] = 200 + i;
	}
	host_bus_dsm_fifo_load(words, count * 3, start * 3, cycle * 3, flags);
}

static void check_records(uint8_t *data, unsigned first, unsigned count,
	unsigned start, const char *order)
{
	for (unsigned i = 0; i < count; i++) {
		uint8_t *record = data + i * 7;
		bool gyro = order[(start + i) % strlen(order)] == 'G';
		assert(record[0] == (gyro ? 1 : 2));
		assert(record[1] == 0 && record[2] == 0x40);
		assert((int16_t)(record[3] | record[4] << 8) == -100 - (int)(first + i));
		assert((uint16_t)(record[5] | record[6] << 8) == 200 + first + i);
		float a[3] = {-99, -99, -99}, g[3] = {-99, -99, -99};
		assert(sensor_imu_lsm6dsm.fifo_process(i, data, a, g) == 0);
		float *decoded = gyro ? g : a, *untouched = gyro ? a : g;
		float scale = gyro ? .07f : 1.0f / 2048;
		assert(fabsf(decoded[0] - 16384 * scale) < .001f);
		assert(fabsf(decoded[1] - (-100 - (int)(first + i)) * scale) < .001f);
		assert(fabsf(decoded[2] - (200 + first + i) * scale) < .001f);
		for (unsigned axis = 0; axis < 3; axis++)
			assert(untouched[axis] == -99);
	}
}

static void configure(float a, float g)
{
	float actual_a = -1, actual_g = -1;
	host_bus_reset();
	assert(lsm6dsm_init(0, a, g, &actual_a, &actual_g) == 0);
	assert((g == INFINITY) == (actual_g == INFINITY));
	float ar, gr;
	lsm6dsm_update_fs(16, 2000, &ar, &gr);
}

static void watermark(uint16_t samples, uint16_t words)
{
	lsm6dsm_setup_DRDY(samples);
	assert(host_bus_register(LSM6DSM_FIFO_CTRL1) == (words & 255));
	assert(host_bus_register(LSM6DSM_FIFO_CTRL1 + 1) == (words >> 8));
}

static void read_pattern(unsigned start, unsigned count, const char *order)
{
	uint8_t data[128 * 7 + 7];
	memset(data, 0xcc, sizeof(data));
	load_samples(start, count, strlen(order), 0);
	assert(lsm6dsm_fifo_read(data, count * 7 + 6) == count);
	check_records(data, 0, count, start, order);
	for (unsigned i = count * 7; i < sizeof(data); i++)
		assert(data[i] == 0xcc);
	assert(host_bus_register(LSM6DSM_FIFO_STATUS1) == 0);
	assert((host_bus_register(LSM6DSM_FIFO_STATUS1 + 1) & 7) == 0);
	unsigned next = ((start + count) % strlen(order)) * 3;
	assert(host_bus_register(LSM6DSM_FIFO_STATUS1 + 2) == next);
}

static void test_streams(void)
{
	static const char gyro[] = "G", accel[] = "A";
	configure(0, .005f);
	assert(host_bus_register(LSM6DSM_FIFO_CTRL3) == 8);
	read_pattern(0, 3, gyro);
	watermark(5, 15);
	configure(.005f, 0);
	assert(host_bus_register(LSM6DSM_FIFO_CTRL3) == 1);
	read_pattern(0, 3, accel);
	watermark(5, 15);
	watermark(0, 0);
	watermark(UINT16_MAX, 2047);
	/* L6. Sleeping gyro retains ODR, but is not a batched stream. */
	float a, g;
	configure(.005f, .005f);
	uint8_t gyro_odr = host_bus_register(LSM6DSM_CTRL2);
	assert(lsm6dsm_update_odr(.005f, INFINITY, &a, &g) == 0 && isinf(g));
	assert(host_bus_register(LSM6DSM_CTRL2) == gyro_odr);
	assert(host_bus_register(LSM6DSM_FIFO_CTRL3) == 1);
	read_pattern(0, 3, accel);
	watermark(5, 15);
	assert(lsm6dsm_update_odr(.005f, .005f, &a, &g) == 0 && isfinite(g));
	assert(host_bus_register(LSM6DSM_FIFO_CTRL3) == 9);
	assert(lsm6dsm_update_odr(0, INFINITY, &a, &g) == 0 && isinf(g));
	assert(host_bus_register(LSM6DSM_FIFO_CTRL3) == 0);
	uint8_t data[7];
	assert(lsm6dsm_fifo_read(data, sizeof(data)) == 0);
	watermark(5, 0);
	configure(0, 0);
	assert(host_bus_register(LSM6DSM_FIFO_CTRL3) == 0);
	assert(lsm6dsm_fifo_read(data, sizeof(data)) == 0);
	watermark(5, 0);
}

static void test_ratios(void)
{
	/* Literal datasheet schedules: gyro precedes accel at their common tick.
	 * No generated sensor-kind decision duplicates the production decoder. */
	static const struct {
		float slow;
		uint8_t accel_fast_ctrl3, gyro_fast_ctrl3;
		unsigned watermark_words;
		const char *accel_fast, *gyro_fast;
	} cases[] = {
		{.002f, 0x09, 0x09, 30, "GA", "GA"},
		{.004f, 0x11, 0x0a, 24, "GAA", "GAG"},
		{.008f, 0x21, 0x0c, 21, "GAAAA", "GAGGG"},
		{.016f, 0x29, 0x0d, 18, "GAAAAAAAA", "GAGGGGGGG"},
		{.032f, 0x31, 0x0e, 18, "GAAAAAAAAAAAAAAAA", "GAGGGGGGGGGGGGGGG"},
		{.064f, 0x39, 0x0f, 18,
			"GAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA",
			"GAGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG"},
	};
	for (unsigned c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
		for (unsigned gyro_fast = 0; gyro_fast < 2; gyro_fast++) {
			configure(gyro_fast ? cases[c].slow : .002f,
				gyro_fast ? .002f : cases[c].slow);
			assert(host_bus_register(LSM6DSM_FIFO_CTRL3)
				== (gyro_fast ? cases[c].gyro_fast_ctrl3 : cases[c].accel_fast_ctrl3));
			assert(host_bus_register(LSM6DSM_FIFO_CTRL5) == 0x3e);
			const char *order = gyro_fast ? cases[c].gyro_fast : cases[c].accel_fast;
			for (unsigned start = 0; start < strlen(order); start++)
				read_pattern(start, strlen(order) + 3, order);
			/* Cross the 24-record transfer and 32-record status batch boundaries
			 * repeatedly; ratio 32 includes its actual 33rd record. */
			read_pattern(0, 70, order);
			watermark(5, cases[c].watermark_words);
			watermark(UINT16_MAX, 2047);

			uint8_t first[25 * 7 + 6], rest[45 * 7 + 6];
			memset(first, 0xcc, sizeof(first));
			memset(rest, 0xcc, sizeof(rest));
			load_samples(0, 70, strlen(order), 0);
			assert(lsm6dsm_fifo_read(first, sizeof(first)) == 25);
			check_records(first, 0, 25, 0, order);
			assert(host_bus_register(LSM6DSM_FIFO_STATUS1) == 135);
			unsigned pattern = (25 % strlen(order)) * 3;
			assert(host_bus_register(LSM6DSM_FIFO_STATUS1 + 2) == pattern);
			assert(lsm6dsm_fifo_read(rest, 6) == 0);
			assert(host_bus_register(LSM6DSM_FIFO_STATUS1) == 135);
			assert(lsm6dsm_fifo_read(rest, sizeof(rest)) == 45);
			check_records(rest, 25, 45, 25, order);
			for (unsigned i = 25 * 7; i < sizeof(first); i++) assert(first[i] == 0xcc);
			for (unsigned i = 45 * 7; i < sizeof(rest); i++) assert(rest[i] == 0xcc);
			assert(host_bus_register(LSM6DSM_FIFO_STATUS1) == 0);
		}
	}
	/* Existing unsupported ratio policy excludes the slow stream. */
	configure(.0001f, .08f);
	assert(host_bus_register(LSM6DSM_FIFO_CTRL3) == 1);
	read_pattern(0, 3, "A");
}

static void test_failures(void)
{
	configure(0, .005f);
	float a = -7, g = -8;
	host_bus_fail_transaction(host_bus_transaction_count() + 1);
	assert(lsm6dsm_update_odr(.005f, 0, &a, &g) < 0);
	assert(a == -7 && g == -8);
	static const char gyro[] = "G";
	read_pattern(0, 3, gyro);
	watermark(5, 15);
	uint8_t data[7];
	memset(data, 0xcc, sizeof(data));
	load_samples(0, 1, 1, 0);
	host_bus_fail_read(LSM6DSM_FIFO_STATUS1);
	assert(lsm6dsm_fifo_read(data, sizeof(data)) == 0 && data[0] == 0xcc);
	host_bus_fail_read(LSM6DSM_FIFO_DATA_OUT_L);
	load_samples(0, 1, 1, 0);
	assert(lsm6dsm_fifo_read(data, sizeof(data)) == 0 && data[0] == 0xcc);
	/* Failure of the last config write may leave hardware partly changed.
	 * Only the published output/cache/shape contract is asserted here. */
	for (unsigned failure = 6; failure <= 7; failure++) {
		configure(0, .005f);
		a = -7; g = -8;
		host_bus_fail_transaction(host_bus_transaction_count() + failure);
		assert(lsm6dsm_update_odr(.002f, .008f, &a, &g) < 0);
		assert(a == -7 && g == -8);
		read_pattern(0, 3, "G");
		watermark(5, 15);
		size_t before_retry = host_bus_transaction_count();
		assert(lsm6dsm_update_odr(.002f, .008f, &a, &g) == 0);
		assert(host_bus_transaction_count() == before_retry + 7);
		assert(fabsf(a - .0012f) < .000001f && fabsf(g - .0048f) < .000001f);
		assert(host_bus_register(LSM6DSM_FIFO_CTRL3) == 0x21);
		assert(host_bus_register(LSM6DSM_FIFO_CTRL5) == 0x3e);
		read_pattern(0, 35, "GAAAA");
		watermark(5, 21);
	}
}


static void test_alignment_and_stable_tags(void)
{
	configure(.005f, .005f);
	const uint16_t partial[] = {0xaaaa, 0xbbbb, 0x4000, (uint16_t)-100, 200};
	host_bus_dsm_fifo_load(partial, 5, 1, 6, 0);
	uint8_t data[7];
	assert(lsm6dsm_fifo_read(data, sizeof(data)) == 1);
	check_records(data, 0, 1, 1, "GA");
	/* Both kinds already returned to the owner survive a configuration change. */
	uint8_t mixed[14];
	load_samples(0, 2, 2, 0);
	assert(lsm6dsm_fifo_read(mixed, sizeof(mixed)) == 2);
	float a, g;
	assert(lsm6dsm_update_odr(0, .005f, &a, &g) == 0);
	check_records(data, 0, 1, 1, "GA");
	check_records(mixed, 0, 2, 0, "GA");
	assert(lsm6dsm_update_odr(.005f, 0, &a, &g) == 0);
	check_records(mixed, 0, 2, 0, "GA");
	float av[3] = {-99, -99, -99}, gv[3] = {-99, -99, -99};
	data[0] = 0xff;
	av[0] = 77;
	assert(sensor_imu_lsm6dsm.fifo_process(0, data, av, gv) == 1);
	assert(av[0] == 77 && gv[0] == -99);
	configure(.005f, .005f);
	host_bus_dsm_fifo_load(partial, 5, 1, 6, 0);
	host_bus_fail_read(LSM6DSM_FIFO_DATA_OUT_L);
	memset(data, 0xcc, sizeof(data));
	assert(lsm6dsm_fifo_read(data, sizeof(data)) == 0 && data[0] == 0xcc);
}

int main(void)
{
	test_streams();
	test_ratios();
	test_failures();
	test_alignment_and_stable_tags();
	puts("DSM actual-driver FIFO stream tests passed");
	return 0;
}
