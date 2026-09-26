#include "customer_info.h"

#include <hal/nrf_uicr.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(customer_info, LOG_LEVEL_INF);

BUILD_ASSERT(offsetof(NRF_UICR_Type, CUSTOMER) == 0x80, "Unexpected CUSTOMER register offset");
BUILD_ASSERT(sizeof(NRF_UICR->CUSTOMER[0]) == sizeof(uint32_t), "Unexpected CUSTOMER word size");
BUILD_ASSERT(sizeof(NRF_UICR->CUSTOMER) >= CUSTOMER_INFO_CUSTOMER_OFFSET + CUSTOMER_INFO_RECORD_SIZE,
	"CUSTOMER register array does not contain the selected record");
BUILD_ASSERT(CUSTOMER_INFO_CUSTOMER_OFFSET % sizeof(uint32_t) == 0, "Record must be word aligned");

void customer_info_read(struct customer_info_result *out)
{
	uint8_t record[CUSTOMER_INFO_RECORD_SIZE];
	const volatile uint32_t *words = &NRF_UICR->CUSTOMER[CUSTOMER_INFO_CUSTOMER_OFFSET / sizeof(uint32_t)];
	/* Snapshot only slot B first. Nonempty B always wins, including invalid data. */
	for (size_t i = 0; i < CUSTOMER_INFO_RECORD_SIZE / sizeof(uint32_t); i++) {
		sys_put_le32(words[i], record + i * sizeof(uint32_t));
	}
	customer_info_parse(record, sizeof(record), out);
	if (out->status == CUSTOMER_INFO_ABSENT) {
#if CONFIG_CUSTOMER_INFO_SKT0
		customer_info_read_board_variant(out);
#endif
		return;
	}
	out->source = CUSTOMER_INFO_SOURCE_SLOT_B;
#if CONFIG_CUSTOMER_INFO_CHECK_IDENTITY && CONFIG_CUSTOMER_INFO_EXPECTED_PRODUCT_ID >= 0 && \
	CONFIG_CUSTOMER_INFO_EXPECTED_HARDWARE_REVISION >= 0
	out->identity = customer_info_check_identity(out, CONFIG_CUSTOMER_INFO_EXPECTED_MANUFACTURER,
		CONFIG_CUSTOMER_INFO_EXPECTED_PRODUCT_ID, CONFIG_CUSTOMER_INFO_EXPECTED_HARDWARE_REVISION);
#endif
}

static bool not_recorded(const uint8_t *bytes, size_t size)
{
	for (size_t i = 0; i < size; i++) {
		if (bytes[i] != 0xFF) {
			return false;
		}
	}
	return true;
}

static const char *source_name(enum customer_info_source source)
{
	switch (source) {
	case CUSTOMER_INFO_SOURCE_SLOT_B:
		return "UICR CUSTOMER +64 bytes (slot B, words 16..31, 0x100010C0)";
#if CONFIG_CUSTOMER_INFO_SKT0
	case CUSTOMER_INFO_SOURCE_SLOT_A:
		return "UICR CUSTOMER +0 bytes (slot A, words 0..15, 0x10001080)";
#endif
	default:
#if CONFIG_CUSTOMER_INFO_SKT0
		return "none (selected slots B and board compatibility A are erased)";
#else
		return "none (selected slot B is erased; other CUSTOMER bytes not inspected)";
#endif
	}
}

static const char *format_name(enum customer_info_format format)
{
	switch (format) {
	case CUSTOMER_INFO_FORMAT_SNCI_V1: return "SNCI v1";
#if CONFIG_CUSTOMER_INFO_SKT0
	case CUSTOMER_INFO_FORMAT_SKT0_V1: return "SKT0 v1";
#endif
	default: return "not recognized";
	}
}

/* Console lines share the same snapshot and identity decision as the startup summary. */
#define REPORT(format, ...) printk(format "\n", ##__VA_ARGS__)

void customer_info_report(enum customer_info_report_mode mode)
{
	struct customer_info_result result;
	customer_info_read(&result);
	enum customer_info_identity identity = result.identity;
	const char *identity_name = identity == CUSTOMER_INFO_IDENTITY_MATCH ? "match" :
		identity == CUSTOMER_INFO_IDENTITY_MISMATCH ? "mismatch" : "not_checked";
	if (mode == CUSTOMER_INFO_REPORT_LOG_SUMMARY) {
		if (result.status != CUSTOMER_INFO_VALID) {
			LOG_INF("CUSTOMER %s: %s (%s); identity=%s", source_name(result.source),
				customer_info_status_str(result.status), result.reason, identity_name);
		} else if (result.format == CUSTOMER_INFO_FORMAT_SNCI_V1) {
			LOG_INF("CUSTOMER %s: %s valid; identity=%s; %s product=%u hw=%u",
				source_name(result.source), format_name(result.format), identity_name,
				result.info.manufacturer, (unsigned)result.info.product_id,
				(unsigned)result.info.hardware_revision);
#if CONFIG_CUSTOMER_INFO_SKT0
		} else {
			LOG_INF("CUSTOMER %s: %s valid; identity=%s; product=%u hw=%u gen=%u region=%s",
				source_name(result.source), format_name(result.format), identity_name,
				(unsigned)result.info.product_id, (unsigned)result.info.hardware_revision,
				(unsigned)result.info.skt0_generation, result.info.skt0_region);
#endif
		}
		return;
	}
	if (mode == CUSTOMER_INFO_REPORT_CONSOLE_SUMMARY) {
		REPORT("customer info: %s; format=%s; identity=%s%s%s",
			customer_info_status_str(result.status), format_name(result.format), identity_name,
			result.status == CUSTOMER_INFO_VALID ? "" : "; reason=",
			result.status == CUSTOMER_INFO_VALID ? "" : result.reason);
		return;
	}
	REPORT("CUSTOMER provenance:");
	REPORT("  Source: %s", source_name(result.source));
	REPORT("  Status: %s (%s)", customer_info_status_str(result.status), result.reason);
	REPORT("  Identity: %s", identity_name);
	if (result.status != CUSTOMER_INFO_VALID) {
		return;
	}
	const struct customer_info *info = &result.info;
	REPORT("  Format: %s", format_name(result.format));
	if (result.format == CUSTOMER_INFO_FORMAT_SNCI_V1) {
		REPORT("  Manufacturer namespace: %s", info->manufacturer);
#if CONFIG_CUSTOMER_INFO_SKT0
	} else if (result.format == CUSTOMER_INFO_FORMAT_SKT0_V1) {
		REPORT("  SKT0 generation: %u; region: %s", (unsigned)info->skt0_generation,
			info->skt0_region);
#endif
	}
	REPORT("  Product ID: %u; hardware revision: %u", (unsigned)info->product_id,
		(unsigned)info->hardware_revision);
	REPORT("  Batch: %s", info->batch);
	if (info->production_date == 0) {
		REPORT("  Production date: not recorded");
	} else {
		REPORT("  Production date: %08u (YYYYMMDD)", (unsigned)info->production_date);
	}
	if (result.format == CUSTOMER_INFO_FORMAT_SNCI_V1 &&
		not_recorded(info->factory_app_version, sizeof(info->factory_app_version))) {
		REPORT("  Factory app version: not recorded (not current firmware)");
	} else {
		REPORT("  Factory app version: %u.%u.%u.%u (not current firmware)",
			(unsigned)info->factory_app_version[0], (unsigned)info->factory_app_version[1],
			(unsigned)info->factory_app_version[2], (unsigned)info->factory_app_version[3]);
	}
	if (result.format == CUSTOMER_INFO_FORMAT_SNCI_V1 &&
		not_recorded(info->provenance_sha256_prefix, sizeof(info->provenance_sha256_prefix))) {
		REPORT("  Provenance SHA-256 prefix: not recorded");
	} else {
		static const char hex[] = "0123456789abcdef";
		char prefix[33];
		for (size_t i = 0; i < sizeof(info->provenance_sha256_prefix); i++) {
			prefix[i * 2] = hex[info->provenance_sha256_prefix[i] >> 4];
			prefix[i * 2 + 1] = hex[info->provenance_sha256_prefix[i] & 0x0F];
		}
		prefix[32] = '\0';
		REPORT("  Provenance SHA-256 prefix: %s", prefix);
	}
}
