#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sensor/imu/ICM42686.h"
#include "sensor/imu/ICM42688.h"

/* Link both complete production translation units. Section GC discards the
 * unused hardware paths; the decoder path has no mocked I/O or copied code. */
struct decoder {
	int (*process)(uint16_t index, uint8_t *data, float a[3], float g[3]);
	float accel_range;
	float gyro_range;
};

static const struct decoder decoders[] = {
	{icm42686_fifo_process, 32.0f, 4000.0f},
	{icm_fifo_process, 16.0f, 2000.0f},
};

/* Accel: +524287, -524288, -1. Gyro: +1, -2, +74565.
 * Different low nibbles catch cross-sensor/axis mixing. Temperature and
 * timestamp bytes are deliberately nonzero and must not enter either vector. */
static const uint8_t sample[20] = {
	0x78,
	0x7f, 0xff, 0x80, 0x00, 0xff, 0xff,
	0x00, 0x00, 0xff, 0xff, 0x12, 0x34,
	0x9a, 0xbc, 0xde, 0xf0,
	0xf1, 0x0e, 0xf5,
};
static const int32_t accel_counts[3] = {524287, -524288, -1};
static const int32_t gyro_counts[3] = {1, -2, 74565};
static const uint8_t invalid[6] = {0x80, 0x00, 0x80, 0x00, 0x80, 0x00};

static void expect_vector(const float actual[3], const int32_t counts[3], float range)
{
	for (unsigned i = 0; i < 3; i++) {
		/* Physical units from signed, right-aligned 20-bit counts. */
		assert(actual[i] == (float)counts[i] * (range / 524288.0f));
	}
}

static void expect_zero(const float actual[3])
{
	for (unsigned i = 0; i < 3; i++) {
		assert(actual[i] == 0.0f);
	}
}

static void expect_skipped(const struct decoder *decoder, uint8_t packet[20])
{
	float a[3] = {11.0f, 12.0f, 13.0f};
	float g[3] = {21.0f, 22.0f, 23.0f};
	assert(decoder->process(0, packet, a, g) == 1);
	for (unsigned i = 0; i < 3; i++) {
		assert(a[i] == 11.0f + i);
		assert(g[i] == 21.0f + i);
	}
}

static void test_signed_scaling_and_index(const struct decoder *decoder)
{
	uint8_t packets[40];
	memset(packets, 0xff, 20);
	memcpy(packets + 20, sample, 20);
	float a[3], g[3];
	assert(decoder->process(1, packets, a, g) == 0);
	expect_vector(a, accel_counts, decoder->accel_range);
	expect_vector(g, gyro_counts, decoder->gyro_range);
}

static void test_invalid_vectors(const struct decoder *decoder)
{
	uint8_t packet[20];
	float a[3] = {11.0f, 12.0f, 13.0f};
	float g[3] = {21.0f, 22.0f, 23.0f};
	memcpy(packet, sample, sizeof(packet));
	memcpy(packet + 1, invalid, sizeof(invalid));
	/* Extension nibbles remain nonzero: the historical sentinel uses only
	 * the three upper-16-bit values, not the full 20-bit values. */
	assert(decoder->process(0, packet, a, g) == 0);
	expect_zero(a);
	expect_vector(g, gyro_counts, decoder->gyro_range);

	memcpy(packet, sample, sizeof(packet));
	memcpy(packet + 7, invalid, sizeof(invalid));
	assert(decoder->process(0, packet, a, g) == 0);
	expect_vector(a, accel_counts, decoder->accel_range);
	expect_zero(g);

	memcpy(packet + 1, invalid, sizeof(invalid));
	expect_skipped(decoder, packet);
}

static void test_empty_and_zero(const struct decoder *decoder)
{
	uint8_t packet[20];
	memcpy(packet, sample, sizeof(packet));
	const uint8_t empty_headers[] = {0x80, 0x7f, 0xff};
	for (unsigned i = 0; i < sizeof(empty_headers); i++) {
		packet[0] = empty_headers[i];
		expect_skipped(decoder, packet);
	}

	/* Preserve valid-zero ambiguity: success and zero vectors, no new
	 * sample-validity flags. Header presence bits are not newly enforced. */
	const uint8_t zero_headers[] = {0x78, 0x00};
	for (unsigned i = 0; i < sizeof(zero_headers); i++) {
		memset(packet, 0, sizeof(packet));
		packet[0] = zero_headers[i];
		float a[3] = {11.0f, 12.0f, 13.0f};
		float g[3] = {21.0f, 22.0f, 23.0f};
		assert(decoder->process(0, packet, a, g) == 0);
		expect_zero(a);
		expect_zero(g);
	}
}

int main(void)
{
	for (unsigned i = 0; i < sizeof(decoders) / sizeof(decoders[0]); i++) {
		test_signed_scaling_and_index(&decoders[i]);
		test_invalid_vectors(&decoders[i]);
		test_empty_and_zero(&decoders[i]);
	}
	puts("ICM42686/ICM42688 hires decoder tests passed");
	return 0;
}
