#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/sys/crc.h>
#include "customer_info_skt0.h"
#include "fixture.h"

static void put16(uint8_t *p, uint16_t value)
{
	p[0] = value;
	p[1] = value >> 8;
}

static void put32(uint8_t *p, uint32_t value)
{
	for (unsigned i = 0; i < 4; ++i) {
		p[i] = value >> (8 * i);
	}
}

static void seal(uint8_t *record)
{
	put32(record + 60, crc32_ieee(record, 60));
}

static struct customer_info_result expect(const uint8_t *record, size_t length,
					 enum customer_info_status status)
{
	struct customer_info_result result;
	memset(&result, 0xa5, sizeof(result));
	customer_info_skt0_decode(record, length, &result);
	assert(result.status == status);
	assert(result.source == CUSTOMER_INFO_SOURCE_NONE);
	assert(result.identity == CUSTOMER_INFO_IDENTITY_NOT_CHECKED);
	if (status == CUSTOMER_INFO_VALID) {
		assert(result.format == CUSTOMER_INFO_FORMAT_SKT0_V1);
		assert(result.info.manufacturer[0] == '\0');
		assert(customer_info_check_identity(&result, "SK", result.info.product_id,
			result.info.hardware_revision) == CUSTOMER_INFO_IDENTITY_NOT_CHECKED);
	} else {
		const struct customer_info zero = {0};
		assert(memcmp(&result.info, &zero, sizeof(zero)) == 0);
		if (status != CUSTOMER_INFO_INVALID) {
			assert(result.format == CUSTOMER_INFO_FORMAT_NONE);
		}
	}
	return result;
}

static void test_upstream_vectors(void)
{
	assert(crc32_ieee((const uint8_t *)"123456789", 9) == UINT32_C(0xcbf43926));
	const uint8_t *fixtures[] = {customer_info_p00_golden, customer_info_p10_golden};
	const uint32_t crc[] = {UINT32_C(0xa07be55b), UINT32_C(0xced5fc0d)};
	const uint16_t hardware[] = {0, 10};
	const char *batch[] = {"P00-EXAMPLE-01", "P10-EXAMPLE-01"};
	for (unsigned i = 0; i < 2; ++i) {
		assert(crc32_ieee(fixtures[i], 60) == crc[i]);
		struct customer_info_result result = expect(fixtures[i], 64, CUSTOMER_INFO_VALID);
		assert(result.info.product_id == 1);
		assert(result.info.hardware_revision == hardware[i]);
		assert(result.info.skt0_generation == 1);
		assert(strcmp(result.info.batch, batch[i]) == 0);
		assert(result.info.production_date == 20260819);
		assert(strcmp(result.info.skt0_region, "CN") == 0);
		assert(memcmp(result.info.factory_app_version, fixtures[i] + 36, 4) == 0);
		assert(memcmp(result.info.provenance_sha256_prefix, fixtures[i] + 40, 16) == 0);
		uint8_t unaligned[65];
		memcpy(unaligned + 1, fixtures[i], 64);
		expect(unaligned + 1, 64, CUSTOMER_INFO_VALID);
	}
}

static void test_bounds_schema_and_crc(void)
{
	uint8_t record[65];
	memset(record, 0xff, sizeof(record));
	expect(record, 64, CUSTOMER_INFO_ABSENT);
	memset(record, 0, 64);
	expect(record, 64, CUSTOMER_INFO_UNRECOGNIZED);
	expect(golden, 64, CUSTOMER_INFO_UNRECOGNIZED);
	for (size_t length = 0; length < 64; ++length) {
		expect(customer_info_p00_golden, length, CUSTOMER_INFO_INVALID);
	}
	expect(NULL, 0, CUSTOMER_INFO_INVALID);
	expect(NULL, 64, CUSTOMER_INFO_INVALID);
	memcpy(record, customer_info_p00_golden, 64);
	expect(record, 65, CUSTOMER_INFO_INVALID);
	const uint16_t schemas[] = {0, 2, 0x100, 0xffff};
	for (unsigned i = 0; i < sizeof(schemas) / sizeof(schemas[0]); ++i) {
		memcpy(record, customer_info_p00_golden, 64);
		put16(record + 4, schemas[i]);
		/* Unknown schema classification must not assume schema-1 CRC semantics. */
		record[16] = 0xff;
		expect(record, 64, CUSTOMER_INFO_UNSUPPORTED);
	}
	const uint16_t lengths[] = {0, 1, 63, 65, 0x4000, 0xffff};
	for (unsigned i = 0; i < sizeof(lengths) / sizeof(lengths[0]); ++i) {
		memcpy(record, customer_info_p00_golden, 64);
		put16(record + 6, lengths[i]);
		seal(record);
		expect(record, 64, CUSTOMER_INFO_INVALID);
	}
	const uint32_t generations[] = {0, 2, 0x100, UINT32_MAX};
	for (unsigned i = 0; i < sizeof(generations) / sizeof(generations[0]); ++i) {
		memcpy(record, customer_info_p00_golden, 64);
		put32(record + 12, generations[i]);
		seal(record);
		expect(record, 64, CUSTOMER_INFO_INVALID);
	}
	for (unsigned byte = 8; byte < 64; ++byte) {
		for (unsigned bit = 0; bit < 8; ++bit) {
			memcpy(record, customer_info_p00_golden, 64);
			record[byte] ^= 1U << bit;
			expect(record, 64, CUSTOMER_INFO_INVALID);
		}
	}
}

static void test_batch_region_and_reserved(void)
{
	uint8_t record[64];
	const char *bad[] = {"", "a", "A B", "A_B", "-A", "A-", "A--B", "A/B", "A.B", "A\x7f", "A\x80"};
	for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i) {
		memcpy(record, customer_info_p00_golden, 64);
		memset(record + 16, 0, 16);
		memcpy(record + 16, bad[i], strlen(bad[i]));
		seal(record);
		expect(record, 64, CUSTOMER_INFO_INVALID);
	}
	memcpy(record, customer_info_p00_golden, 64);
	memset(record + 16, 'A', 16);
	seal(record);
	expect(record, 64, CUSTOMER_INFO_INVALID);
	record[31] = 0;
	seal(record);
	expect(record, 64, CUSTOMER_INFO_VALID);
	for (unsigned padding = 2; padding < 16; ++padding) {
		memset(record + 16, 0, 16);
		record[16] = 'A';
		record[16 + padding] = 'X';
		seal(record);
		expect(record, 64, CUSTOMER_INFO_INVALID);
	}
	const char *valid[] = {"Z", "0", "A-B-C", "UNKNOWN"};
	for (unsigned i = 0; i < sizeof(valid) / sizeof(valid[0]); ++i) {
		memcpy(record, customer_info_p00_golden, 64);
		memset(record + 16, 0, 16);
		memcpy(record + 16, valid[i], strlen(valid[i]));
		seal(record);
		expect(record, 64, CUSTOMER_INFO_VALID);
	}
	const uint8_t bad_region[] = {0, 'a', 'z', '0', '-', '@', '[', 0x7f, 0xff};
	for (unsigned offset = 56; offset <= 57; ++offset) {
		for (unsigned i = 0; i < sizeof(bad_region); ++i) {
			memcpy(record, customer_info_p00_golden, 64);
			record[offset] = bad_region[i];
			seal(record);
			expect(record, 64, CUSTOMER_INFO_INVALID);
		}
	}
	memcpy(record, customer_info_p00_golden, 64);
	memcpy(record + 56, "ZZ", 2);
	seal(record);
	struct customer_info_result result = expect(record, 64, CUSTOMER_INFO_VALID);
	assert(strcmp(result.info.skt0_region, "ZZ") == 0);
	for (unsigned offset = 58; offset <= 59; ++offset) {
		for (unsigned value = 0; value < 255; ++value) {
			memcpy(record, customer_info_p00_golden, 64);
			record[offset] = value;
			seal(record);
			expect(record, 64, CUSTOMER_INFO_INVALID);
		}
	}
}

static void test_dates_ids_and_opaque(void)
{
	const uint32_t valid[] = {10000101, 99991231, 20000229, 20240229, 19000228, 21000301};
	const uint32_t invalid[] = {0, 9991231, 100000101, 20240001, 20241301, 20240100,
		20240132, 20240431, 20230229, 19000229, 21000229, 20240230, UINT32_MAX};
	uint8_t record[64];
	for (unsigned i = 0; i < sizeof(valid) / sizeof(valid[0]); ++i) {
		memcpy(record, customer_info_p00_golden, 64);
		put32(record + 32, valid[i]);
		seal(record);
		struct customer_info_result result = expect(record, 64, CUSTOMER_INFO_VALID);
		assert(result.info.production_date == valid[i]);
	}
	for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
		memcpy(record, customer_info_p00_golden, 64);
		put32(record + 32, invalid[i]);
		seal(record);
		expect(record, 64, CUSTOMER_INFO_INVALID);
	}
	/* Unknown numeric identities are valid data, not an invented vendor mapping. */
	const uint16_t ids[] = {0, 0x1234, UINT16_MAX};
	for (unsigned i = 0; i < sizeof(ids) / sizeof(ids[0]); ++i) {
		memcpy(record, customer_info_p00_golden, 64);
		put16(record + 8, ids[i]);
		put16(record + 10, ids[i]);
		seal(record);
		struct customer_info_result result = expect(record, 64, CUSTOMER_INFO_VALID);
		assert(result.info.product_id == ids[i]);
		assert(result.info.hardware_revision == ids[i]);
	}
	for (unsigned value = 0; value < 2; ++value) {
		memcpy(record, customer_info_p00_golden, 64);
		memset(record + 36, value ? 0xff : 0, 20);
		seal(record);
		struct customer_info_result result = expect(record, 64, CUSTOMER_INFO_VALID);
		assert(memcmp(result.info.factory_app_version, record + 36, 4) == 0);
		assert(memcmp(result.info.provenance_sha256_prefix, record + 40, 16) == 0);
	}
}

int main(void)
{
	test_upstream_vectors();
	test_bounds_schema_and_crc();
	test_batch_region_and_reserved();
	test_dates_ids_and_opaque();
	puts("customer_info: SKT0 pinned P00/P10 vectors, boundaries and semantics passed");
	return 0;
}
