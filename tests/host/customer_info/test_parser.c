#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/sys/crc.h>
#include "customer_info_parser.h"
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
	customer_info_parse(record, length, &result);
	assert(result.status == status);
	assert(result.source == CUSTOMER_INFO_SOURCE_NONE);
	assert(result.identity == CUSTOMER_INFO_IDENTITY_NOT_CHECKED);
	if (status != CUSTOMER_INFO_VALID) {
		const struct customer_info zero = {0};
		assert(memcmp(&result.info, &zero, sizeof(zero)) == 0);
		assert(customer_info_check_identity(&result, "ACME-7", 0x1234, 0x5678) ==
		       CUSTOMER_INFO_IDENTITY_NOT_CHECKED);
	}
	return result;
}

static void test_golden(void)
{
	assert(crc32_ieee((const uint8_t *)"123456789", 9) == UINT32_C(0xcbf43926));
	assert(crc32_ieee(golden, 60) == UINT32_C(0xee79a8a8));
	struct customer_info_result result = expect(golden, 64, CUSTOMER_INFO_VALID);
	assert(strcmp(result.info.manufacturer, "ACME-7") == 0);
	assert(result.format == CUSTOMER_INFO_FORMAT_SNCI_V1);
	assert(result.info.skt0_generation == 0);
	assert(result.info.skt0_region[0] == '\0');
	assert(result.info.product_id == 0x1234);
	assert(result.info.hardware_revision == 0x5678);
	assert(strcmp(result.info.batch, "LOT-2026-09") == 0);
	assert(result.info.production_date == 20240229);
	assert(memcmp(result.info.factory_app_version, golden + 40, 4) == 0);
	assert(memcmp(result.info.provenance_sha256_prefix, golden + 44, 16) == 0);
	/* Parse an unaligned buffer too: the wire ABI is not a native C struct. */
	uint8_t unaligned[65];
	memcpy(unaligned + 1, golden, 64);
	expect(unaligned + 1, 64, CUSTOMER_INFO_VALID);
}

static void test_status_and_bounds(void)
{
	uint8_t record[65];
	memset(record, 0xff, sizeof(record));
	expect(record, 64, CUSTOMER_INFO_ABSENT);
	memset(record, 0, 64);
	expect(record, 64, CUSTOMER_INFO_UNRECOGNIZED);
	memcpy(record, golden, 64);
	memcpy(record, "SKT0", 4);
	expect(record, 64, CUSTOMER_INFO_UNRECOGNIZED);
	memcpy(record, golden, 64);
	put16(record + 4, 2);
	/* Version classification must not assume version-1 CRC or field rules. */
	record[8] = 0xff;
	expect(record, 64, CUSTOMER_INFO_UNSUPPORTED);
	for (size_t length = 0; length < 64; ++length) {
		expect(golden, length, CUSTOMER_INFO_INVALID);
	}
	expect(NULL, 64, CUSTOMER_INFO_INVALID);
	expect(NULL, 0, CUSTOMER_INFO_INVALID);
	memcpy(record, golden, 64);
	expect(record, 65, CUSTOMER_INFO_INVALID);
	const uint16_t lengths[] = {0, 1, 63, 65, 0x4000, 0xffff};
	for (size_t i = 0; i < sizeof(lengths) / sizeof(lengths[0]); ++i) {
		memcpy(record, golden, 64);
		put16(record + 6, lengths[i]);
		seal(record);
		expect(record, 64, CUSTOMER_INFO_INVALID);
	}
	/* Every payload/CRC bit is protected, including opaque optional bytes. */
	for (unsigned byte = 8; byte < 64; ++byte) {
		for (unsigned bit = 0; bit < 8; ++bit) {
			memcpy(record, golden, 64);
			record[byte] ^= 1U << bit;
			expect(record, 64, CUSTOMER_INFO_INVALID);
		}
	}
}

static void test_tokens(void)
{
	const char *bad[] = {"", "a", "A B", "A_B", "-A", "A-", "A--B", "A/B", "A.B", "A\x7f", "A\x80"};
	uint8_t record[64];
	const size_t offsets[] = {8, 20};
	const size_t widths[] = {8, 16};
	for (unsigned field = 0; field < 2; ++field) {
		size_t offset = offsets[field], width = widths[field];
		for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i) {
			memcpy(record, golden, 64);
			memset(record + offset, 0, width);
			memcpy(record + offset, bad[i], strlen(bad[i]));
			seal(record);
			expect(record, 64, CUSTOMER_INFO_INVALID);
		}
		memcpy(record, golden, 64);
		memset(record + offset, 'A', width); /* missing terminator */
		seal(record);
		expect(record, 64, CUSTOMER_INFO_INVALID);
		for (size_t padding = 2; padding < width; ++padding) {
			memset(record + offset, 0, width);
			record[offset] = 'A';
			record[offset + padding] = 'X';
			seal(record);
			expect(record, 64, CUSTOMER_INFO_INVALID);
		}
		const char *valid[] = {"Z", "0", "A-B-C", "UNKNOWN"};
		for (size_t i = 0; i < sizeof(valid) / sizeof(valid[0]); ++i) {
			memcpy(record, golden, 64);
			memset(record + offset, 0, width);
			memcpy(record + offset, valid[i], strlen(valid[i]));
			seal(record);
			expect(record, 64, CUSTOMER_INFO_VALID);
		}
		memset(record + offset, '9', width - 1);
		record[offset + width - 1] = 0;
		seal(record);
		expect(record, 64, CUSTOMER_INFO_VALID);
	}
}

static void test_dates_and_optional(void)
{
	const uint32_t valid[] = {0, 10000101, 99991231, 20000229, 20240229, 19000228, 21000301};
	const uint32_t invalid[] = {9991231, 100000101, 20240001, 20241301, 20240100,
		20240132, 20240431, 20230229, 19000229, 21000229, 20240230, UINT32_MAX};
	uint8_t record[64];
	for (size_t i = 0; i < sizeof(valid) / sizeof(valid[0]); ++i) {
		memcpy(record, golden, 64);
		put32(record + 36, valid[i]);
		seal(record);
		struct customer_info_result result = expect(record, 64, CUSTOMER_INFO_VALID);
		assert(result.info.production_date == valid[i]);
	}
	for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
		memcpy(record, golden, 64);
		put32(record + 36, invalid[i]);
		seal(record);
		expect(record, 64, CUSTOMER_INFO_INVALID);
	}
	memcpy(record, golden, 64);
	put32(record + 36, 0);
	memset(record + 40, 0xff, 20);
	seal(record);
	struct customer_info_result result = expect(record, 64, CUSTOMER_INFO_VALID);
	assert(result.info.production_date == 0);
	assert(memcmp(result.info.factory_app_version, record + 40, 4) == 0);
	assert(memcmp(result.info.provenance_sha256_prefix, record + 44, 16) == 0);
	/* Partial FF and all-zero values are recorded opaque bytes, not invalid. */
	record[40] = 0;
	record[44] = 0;
	seal(record);
	result = expect(record, 64, CUSTOMER_INFO_VALID);
	assert(memcmp(result.info.factory_app_version, record + 40, 4) == 0);
	assert(memcmp(result.info.provenance_sha256_prefix, record + 44, 16) == 0);
	memset(record + 40, 0, 20);
	seal(record);
	expect(record, 64, CUSTOMER_INFO_VALID);
}

static void test_identity(void)
{
	struct customer_info_result result = expect(golden, 64, CUSTOMER_INFO_VALID);
	assert(customer_info_check_identity(&result, NULL, 0, 0) == CUSTOMER_INFO_IDENTITY_NOT_CHECKED);
	assert(customer_info_check_identity(&result, "", 0, 0) == CUSTOMER_INFO_IDENTITY_NOT_CHECKED);
	assert(customer_info_check_identity(&result, "ACME-7", 0x1234, 0x5678) == CUSTOMER_INFO_IDENTITY_MATCH);
	assert(customer_info_check_identity(&result, "OTHER", 0x1234, 0x5678) == CUSTOMER_INFO_IDENTITY_MISMATCH);
	assert(customer_info_check_identity(&result, "ACME-7", 0, 0x5678) == CUSTOMER_INFO_IDENTITY_MISMATCH);
	assert(customer_info_check_identity(&result, "ACME-7", 0x1234, 0) == CUSTOMER_INFO_IDENTITY_MISMATCH);
	uint8_t record[64];
	memcpy(record, golden, 64);
	memset(record + 16, 0, 4);
	seal(record);
	result = expect(record, 64, CUSTOMER_INFO_VALID);
	assert(result.info.product_id == 0 && result.info.hardware_revision == 0);
	assert(customer_info_check_identity(&result, "ACME-7", 0, 0) == CUSTOMER_INFO_IDENTITY_MATCH);
	assert(customer_info_check_identity(&result, "ACME-7", 1, 0) == CUSTOMER_INFO_IDENTITY_MISMATCH);
	assert(customer_info_check_identity(&result, "ACME-7", 0, 1) == CUSTOMER_INFO_IDENTITY_MISMATCH);
}

int main(void)
{
	test_golden();
	test_status_and_bounds();
	test_tokens();
	test_dates_and_optional();
	test_identity();
	puts("customer_info: parser, independent CRC fixture, bounds and identity passed");
	return 0;
}
