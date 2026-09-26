#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <hal/nrf_uicr.h>
#include <zephyr/sys/crc.h>
#include "customer_info.h"
#include "fixture.h"

NRF_UICR_Type host_uicr;
static char output[4096];
static size_t output_length;
static bool smoke_output;

int host_print(const char *format, ...)
{
	va_list args;
	va_start(args, format);
	int count = vsnprintf(output + output_length, sizeof(output) - output_length, format, args);
	va_end(args);
	assert(count >= 0 && (size_t)count < sizeof(output) - output_length);
	output_length += (size_t)count;
	if (smoke_output) {
		fputs(output + output_length - (size_t)count, stdout);
	}
	return count;
}

static void load(unsigned first_word, const uint8_t *bytes)
{
	for (unsigned i = 0; i < 16; ++i) {
		const uint8_t *p = bytes + 4 * i;
		host_uicr.CUSTOMER[first_word + i] = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
			((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
	}
}

static void seal(uint8_t *record)
{
	uint32_t crc = crc32_ieee(record, 60);
	for (unsigned i = 0; i < 4; ++i) {
		record[60 + i] = crc >> (8 * i);
	}
}

static void report(enum customer_info_report_mode mode)
{
	NRF_UICR_Type before;
	memcpy(&before, &host_uicr, sizeof(before));
	output_length = 0;
	output[0] = 0;
	customer_info_report(mode);
	assert(memcmp(&before, &host_uicr, sizeof(before)) == 0);
}

static struct customer_info_result expect_read(enum customer_info_status status,
					       enum customer_info_source source,
					       enum customer_info_format format)
{
	NRF_UICR_Type before;
	memcpy(&before, &host_uicr, sizeof(before));
	struct customer_info_result result;
	memset(&result, 0xa5, sizeof(result));
	customer_info_read(&result);
	assert(memcmp(&before, &host_uicr, sizeof(before)) == 0);
	assert(result.status == status);
	assert(result.source == source);
	assert(result.format == format);
	if (status != CUSTOMER_INFO_VALID) {
		const struct customer_info zero = {0};
		assert(memcmp(&result.info, &zero, sizeof(zero)) == 0);
		assert(result.identity == CUSTOMER_INFO_IDENTITY_NOT_CHECKED);
	}
	return result;
}

static void expect_snci(void)
{
	struct customer_info_result result = expect_read(CUSTOMER_INFO_VALID,
		CUSTOMER_INFO_SOURCE_SLOT_B, CUSTOMER_INFO_FORMAT_SNCI_V1);
	assert(strcmp(result.info.manufacturer, "ACME-7") == 0);
	assert(result.info.product_id == 0x1234);
	assert(result.info.hardware_revision == 0x5678);
	assert(strcmp(result.info.batch, "LOT-2026-09") == 0);
	assert(result.info.production_date == 20240229);
	assert(result.info.skt0_generation == 0);
	assert(result.info.skt0_region[0] == 0);
	assert(memcmp(result.info.factory_app_version, golden + 40, 4) == 0);
	assert(memcmp(result.info.provenance_sha256_prefix, golden + 44, 16) == 0);
	enum customer_info_identity expected = CUSTOMER_INFO_IDENTITY_NOT_CHECKED;
#if CONFIG_CUSTOMER_INFO_CHECK_IDENTITY && CONFIG_CUSTOMER_INFO_EXPECTED_PRODUCT_ID >= 0 && \
	CONFIG_CUSTOMER_INFO_EXPECTED_HARDWARE_REVISION >= 0
	if (CONFIG_CUSTOMER_INFO_EXPECTED_MANUFACTURER[0] != '\0') {
		expected = strcmp(CONFIG_CUSTOMER_INFO_EXPECTED_MANUFACTURER, "ACME-7") == 0 &&
			CONFIG_CUSTOMER_INFO_EXPECTED_PRODUCT_ID == 0x1234 &&
			CONFIG_CUSTOMER_INFO_EXPECTED_HARDWARE_REVISION == 0x5678 ?
			CUSTOMER_INFO_IDENTITY_MATCH : CUSTOMER_INFO_IDENTITY_MISMATCH;
	}
#endif
	assert(result.identity == expected);
}

#if CONFIG_CUSTOMER_INFO_SKT0
static void expect_skt0(const uint8_t *fixture, uint16_t hardware, const char *batch)
{
	struct customer_info_result result = expect_read(CUSTOMER_INFO_VALID,
		CUSTOMER_INFO_SOURCE_SLOT_A, CUSTOMER_INFO_FORMAT_SKT0_V1);
	assert(result.info.manufacturer[0] == '\0');
	assert(result.info.product_id == 1);
	assert(result.info.hardware_revision == hardware);
	assert(strcmp(result.info.batch, batch) == 0);
	assert(result.info.production_date == 20260819);
	assert(result.info.skt0_generation == 1);
	assert(strcmp(result.info.skt0_region, "CN") == 0);
	assert(memcmp(result.info.factory_app_version, fixture + 36, 4) == 0);
	assert(memcmp(result.info.provenance_sha256_prefix, fixture + 40, 16) == 0);
	enum customer_info_identity expected = CUSTOMER_INFO_IDENTITY_NOT_CHECKED;
#if CONFIG_CUSTOMER_INFO_SKT0_PRODUCT_ID >= 0 && CONFIG_CUSTOMER_INFO_SKT0_HARDWARE_REVISION >= 0
	expected = CONFIG_CUSTOMER_INFO_SKT0_PRODUCT_ID == 1 &&
		CONFIG_CUSTOMER_INFO_SKT0_HARDWARE_REVISION == hardware ?
		CUSTOMER_INFO_IDENTITY_MATCH : CUSTOMER_INFO_IDENTITY_MISMATCH;
#endif
	assert(result.identity == expected);
	assert(customer_info_check_identity(&result, "ACME-7", 1, hardware) ==
		CUSTOMER_INFO_IDENTITY_NOT_CHECKED);
}
#endif

static void expect_no_provenance(void)
{
	/* Values, not wording: invalid records must not expose a prior snapshot. */
	assert(strstr(output, "ACME-7") == NULL);
	assert(strstr(output, "LOT-2026-09") == NULL);
	assert(strstr(output, "P00-EXAMPLE-01") == NULL);
	assert(strstr(output, "P10-EXAMPLE-01") == NULL);
	assert(strstr(output, "000102030405060708090a0b0c0d0e0f") == NULL);
}

static void test_routing(void)
{
	uint8_t erased[64], record[64];
	memset(erased, 0xff, sizeof(erased));
	memset(&host_uicr, 0xff, sizeof(host_uicr));
	expect_read(CUSTOMER_INFO_ABSENT, CUSTOMER_INFO_SOURCE_NONE, CUSTOMER_INFO_FORMAT_NONE);
	load(0, customer_info_p00_golden);
#if CONFIG_CUSTOMER_INFO_SKT0
	expect_skt0(customer_info_p00_golden, 0, "P00-EXAMPLE-01");
	load(0, customer_info_p10_golden);
	expect_skt0(customer_info_p10_golden, 10, "P10-EXAMPLE-01");
#else
	expect_read(CUSTOMER_INFO_ABSENT, CUSTOMER_INFO_SOURCE_NONE, CUSTOMER_INFO_FORMAT_NONE);
#endif
	/* A cannot affect populated B: valid SNCI wins even over valid SKT0. */
	load(16, golden);
	expect_snci();
	for (unsigned pattern = 0; pattern < 4; ++pattern) {
		for (unsigned i = 0; i < 16; ++i) {
			host_uicr.CUSTOMER[i] = pattern == 0 ? 0 : pattern == 1 ? UINT32_MAX :
				pattern == 2 ? UINT32_C(0xdeadbeef) : i;
		}
		expect_snci();
	}
	load(0, customer_info_p00_golden);
	for (unsigned failure = 0; failure < 4; ++failure) {
		memcpy(record, golden, 64);
		enum customer_info_status status;
		enum customer_info_format format = CUSTOMER_INFO_FORMAT_NONE;
		if (failure == 0) {
			record[63] ^= 1;
			status = CUSTOMER_INFO_INVALID;
			format = CUSTOMER_INFO_FORMAT_SNCI_V1;
		} else if (failure == 1) {
			memset(record, 0xff, 64);
			record[63] = 0xfe; /* Any non-FF byte forbids fallback. */
			status = CUSTOMER_INFO_UNRECOGNIZED;
		} else if (failure == 2) {
			record[4] = 2;
			status = CUSTOMER_INFO_UNSUPPORTED;
		} else {
			memcpy(record, customer_info_p00_golden, 64);
			status = CUSTOMER_INFO_UNRECOGNIZED; /* SKT0 never decoded in B. */
		}
		load(16, record);
		expect_read(status, CUSTOMER_INFO_SOURCE_SLOT_B, format);
		report(CUSTOMER_INFO_REPORT_CONSOLE_DETAILS);
		expect_no_provenance();
	}
	load(16, erased);
#if CONFIG_CUSTOMER_INFO_SKT0
	/* A refreshes on every read; invalid A cannot leak prior decoded fields. */
	expect_skt0(customer_info_p00_golden, 0, "P00-EXAMPLE-01");
	memcpy(record, customer_info_p00_golden, 64);
	record[60] ^= 1;
	load(0, record);
	expect_read(CUSTOMER_INFO_INVALID, CUSTOMER_INFO_SOURCE_SLOT_A, CUSTOMER_INFO_FORMAT_SKT0_V1);
	report(CUSTOMER_INFO_REPORT_LOG_SUMMARY);
	expect_no_provenance();
	load(0, golden);
	expect_read(CUSTOMER_INFO_UNRECOGNIZED, CUSTOMER_INFO_SOURCE_SLOT_A, CUSTOMER_INFO_FORMAT_NONE);
#endif
	load(0, erased);
	expect_read(CUSTOMER_INFO_ABSENT, CUSTOMER_INFO_SOURCE_NONE, CUSTOMER_INFO_FORMAT_NONE);
	report(CUSTOMER_INFO_REPORT_CONSOLE_DETAILS);
	expect_no_provenance();
	load(16, golden);
	expect_snci();
}

static void test_opaque_policy(void)
{
	uint8_t record[64], erased[64];
	memset(erased, 0xff, 64);
	for (unsigned value = 0; value < 2; ++value) {
		memcpy(record, golden, 64);
		memset(record + 40, value ? 0xff : 0, 20);
		seal(record);
		load(16, record);
		struct customer_info_result result = expect_read(CUSTOMER_INFO_VALID,
			CUSTOMER_INFO_SOURCE_SLOT_B, CUSTOMER_INFO_FORMAT_SNCI_V1);
		assert(memcmp(result.info.factory_app_version, record + 40, 4) == 0);
		assert(memcmp(result.info.provenance_sha256_prefix, record + 44, 16) == 0);
		report(CUSTOMER_INFO_REPORT_CONSOLE_DETAILS);
		if (value) {
			assert(strstr(output, "255.255.255.255") == NULL);
			assert(strstr(output, "ffffffffffffffffffffffffffffffff") == NULL);
		} else {
			assert(strstr(output, "0.0.0.0") != NULL);
			assert(strstr(output, "00000000000000000000000000000000") != NULL);
		}
#if CONFIG_CUSTOMER_INFO_SKT0
		load(16, erased);
		memcpy(record, customer_info_p00_golden, 64);
		memset(record + 36, value ? 0xff : 0, 20);
		seal(record);
		load(0, record);
		result = expect_read(CUSTOMER_INFO_VALID, CUSTOMER_INFO_SOURCE_SLOT_A, CUSTOMER_INFO_FORMAT_SKT0_V1);
		assert(memcmp(result.info.factory_app_version, record + 36, 4) == 0);
		assert(memcmp(result.info.provenance_sha256_prefix, record + 40, 16) == 0);
		assert(result.info.manufacturer[0] == '\0');
		report(CUSTOMER_INFO_REPORT_CONSOLE_DETAILS);
		assert(strstr(output, value ? "255.255.255.255" : "0.0.0.0") != NULL);
		assert(strstr(output, value ? "ffffffffffffffffffffffffffffffff" :
			"00000000000000000000000000000000") != NULL);
#endif
	}
}

static void smoke(void)
{
	smoke_output = true;
	memset(&host_uicr, 0xff, sizeof(host_uicr));
	load(0, customer_info_p00_golden);
	puts("--- SKT0 P00 in A, erased B ---");
	report(CUSTOMER_INFO_REPORT_CONSOLE_DETAILS);
	load(0, customer_info_p10_golden);
	puts("--- SKT0 P10 in A, erased B ---");
	report(CUSTOMER_INFO_REPORT_CONSOLE_DETAILS);
	load(16, golden);
	puts("--- valid SNCI B wins over valid SKT0 A ---");
	report(CUSTOMER_INFO_REPORT_CONSOLE_DETAILS);
	host_uicr.CUSTOMER[31] ^= 1;
	puts("--- corrupt SNCI B blocks valid SKT0 A ---");
	report(CUSTOMER_INFO_REPORT_CONSOLE_DETAILS);
	host_uicr.CUSTOMER[17] = UINT32_C(0x00400002);
	puts("--- future SNCI B blocks valid SKT0 A ---");
	report(CUSTOMER_INFO_REPORT_LOG_SUMMARY);
	memset(&host_uicr, 0xff, sizeof(host_uicr));
	puts("--- both slots erased ---");
	report(CUSTOMER_INFO_REPORT_CONSOLE_DETAILS);
}

int main(int argc, char **argv)
{
	assert(argc == 2);
	if (strcmp(argv[1], "--smoke") == 0) {
		smoke();
		return 0;
	}
	test_routing();
	test_opaque_policy();
	printf("customer_info: production MMIO/report (%s) passed\n", argv[1]);
	return 0;
}
