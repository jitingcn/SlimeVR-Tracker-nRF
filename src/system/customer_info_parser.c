#include "customer_info_parser.h"

#include <stdbool.h>
#include <string.h>
#include <zephyr/sys/crc.h>

static uint16_t read_le16(const uint8_t *bytes)
{
	return (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
}

static uint32_t read_le32(const uint8_t *bytes)
{
	return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
		((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
}

static bool valid_identifier(const uint8_t *bytes, size_t size)
{
	size_t length = 0;
	while (length < size && bytes[length] != 0) {
		uint8_t ch = bytes[length];
		if (!((ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9') ||
			(ch == '-' && length != 0 && bytes[length - 1] != '-'))) {
			return false;
		}
		length++;
	}
	if (length == 0 || length == size || bytes[length - 1] == '-') {
		return false;
	}
	for (size_t i = length; i < size; i++) {
		if (bytes[i] != 0) {
			return false;
		}
	}
	return true;
}

static bool valid_date(uint32_t date)
{
	if (date == 0) {
		return true;
	}
	uint32_t year = date / 10000;
	uint32_t month = (date / 100) % 100;
	uint32_t day = date % 100;
	static const uint8_t month_days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	if (year < 1000 || year > 9999 || month < 1 || month > 12) {
		return false;
	}
	uint8_t days = month_days[month - 1];
	if (month == 2 && year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)) {
		days++;
	}
	return day >= 1 && day <= days;
}

void customer_info_parse(const uint8_t *record, size_t length, struct customer_info_result *out)
{
	memset(out, 0, sizeof(*out));
	out->status = CUSTOMER_INFO_INVALID;
	out->reason = "record size must be 64 bytes";
	if (record == NULL || length != CUSTOMER_INFO_RECORD_SIZE) {
		return;
	}
	bool erased = true;
	for (size_t i = 0; i < CUSTOMER_INFO_RECORD_SIZE; i++) {
		if (record[i] != 0xFF) {
			erased = false;
			break;
		}
	}
	if (erased) {
		out->status = CUSTOMER_INFO_ABSENT;
		out->reason = "selected record is erased";
		return;
	}
	if (memcmp(record, "SNCI", 4) != 0) {
		out->status = CUSTOMER_INFO_UNRECOGNIZED;
		out->reason = "unknown magic";
		return;
	}
	if (read_le16(record + 4) != 1) {
		out->status = CUSTOMER_INFO_UNSUPPORTED;
		out->reason = "unsupported schema";
		return;
	}
	out->format = CUSTOMER_INFO_FORMAT_SNCI_V1;
	if (read_le16(record + 6) != CUSTOMER_INFO_RECORD_SIZE) {
		out->reason = "invalid encoded length";
		return;
	}
	if (crc32_ieee(record, 60) != read_le32(record + 60)) {
		out->reason = "CRC32 mismatch";
		return;
	}
	if (!valid_identifier(record + 8, 8)) {
		out->reason = "invalid manufacturer namespace";
		return;
	}
	if (!valid_identifier(record + 20, 16)) {
		out->reason = "invalid batch";
		return;
	}
	uint32_t date = read_le32(record + 36);
	if (!valid_date(date)) {
		out->reason = "invalid production date";
		return;
	}
	memcpy(out->info.manufacturer, record + 8, sizeof(out->info.manufacturer));
	out->info.product_id = read_le16(record + 16);
	out->info.hardware_revision = read_le16(record + 18);
	memcpy(out->info.batch, record + 20, sizeof(out->info.batch));
	out->info.production_date = date;
	memcpy(out->info.factory_app_version, record + 40, sizeof(out->info.factory_app_version));
	memcpy(out->info.provenance_sha256_prefix, record + 44, sizeof(out->info.provenance_sha256_prefix));
	out->status = CUSTOMER_INFO_VALID;
	out->reason = "schema and CRC32 valid";
}

const char *customer_info_status_str(enum customer_info_status status)
{
	switch (status) {
	case CUSTOMER_INFO_ABSENT: return "absent";
	case CUSTOMER_INFO_UNRECOGNIZED: return "unrecognized";
	case CUSTOMER_INFO_UNSUPPORTED: return "unsupported";
	case CUSTOMER_INFO_INVALID: return "invalid";
	case CUSTOMER_INFO_VALID: return "valid";
	default: return "unknown";
	}
}

enum customer_info_identity customer_info_check_identity(
	const struct customer_info_result *result, const char *manufacturer, uint16_t product, uint16_t hardware)
{
	if (result->status != CUSTOMER_INFO_VALID || result->format != CUSTOMER_INFO_FORMAT_SNCI_V1 ||
		manufacturer == NULL || manufacturer[0] == '\0') {
		return CUSTOMER_INFO_IDENTITY_NOT_CHECKED;
	}
	return strcmp(result->info.manufacturer, manufacturer) == 0 &&
		result->info.product_id == product && result->info.hardware_revision == hardware
		? CUSTOMER_INFO_IDENTITY_MATCH : CUSTOMER_INFO_IDENTITY_MISMATCH;
}
