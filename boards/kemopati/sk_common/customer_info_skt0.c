/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "customer_info_skt0.h"

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

static bool batch_id_is_valid(const uint8_t *field)
{
	size_t length = 0;
	bool previous_was_hyphen = false;

	while (length < 16 && field[length] != '\0') {
		uint8_t value = field[length];

		if (value == '-') {
			if (length == 0 || previous_was_hyphen) {
				return false;
			}
			previous_was_hyphen = true;
		} else if ((value >= 'A' && value <= 'Z') || (value >= '0' && value <= '9')) {
			previous_was_hyphen = false;
		} else {
			return false;
		}
		length++;
	}

	if (length == 0 || length == 16 || previous_was_hyphen) {
		return false;
	}
	for (size_t i = length; i < 16; i++) {
		if (field[i] != '\0') {
			return false;
		}
	}
	return true;
}

static bool production_date_is_valid(uint32_t value)
{
	static const uint8_t days_per_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	uint32_t year = value / 10000U;
	uint32_t month = (value / 100U) % 100U;
	uint32_t day = value % 100U;

	if (year < 1000U || year > 9999U || month < 1U || month > 12U) {
		return false;
	}
	uint32_t maximum_day = days_per_month[month - 1U];
	if (month == 2U && ((year % 4U == 0U && year % 100U != 0U) || year % 400U == 0U)) {
		maximum_day++;
	}
	return day >= 1U && day <= maximum_day;
}

void customer_info_skt0_decode(const uint8_t *record, size_t length, struct customer_info_result *out)
{
	if (out == NULL) {
		return;
	}
	memset(out, 0, sizeof(*out));
	out->status = CUSTOMER_INFO_INVALID;
	out->reason = "record size must be 64 bytes";
	if (record == NULL || length != CUSTOMER_INFO_RECORD_SIZE) {
		return;
	}
	bool erased = true;
	for (size_t i = 0; i < CUSTOMER_INFO_RECORD_SIZE; i++) {
		if (record[i] != 0xFFU) {
			erased = false;
			break;
		}
	}
	if (erased) {
		out->status = CUSTOMER_INFO_ABSENT;
		out->reason = "selected record is erased";
		return;
	}
	if (memcmp(record, "SKT0", 4) != 0) {
		out->status = CUSTOMER_INFO_UNRECOGNIZED;
		out->reason = "unknown magic";
		return;
	}
	if (read_le16(record + 4) != 1U) {
		out->status = CUSTOMER_INFO_UNSUPPORTED;
		out->reason = "unsupported schema";
		return;
	}
	out->format = CUSTOMER_INFO_FORMAT_SKT0_V1;
	if (read_le16(record + 6) != CUSTOMER_INFO_RECORD_SIZE) {
		out->reason = "invalid encoded length";
		return;
	}
	if (crc32_ieee(record, 60) != read_le32(record + 60)) {
		out->reason = "CRC32 mismatch";
		return;
	}
	if (read_le32(record + 12) != 1U) {
		out->reason = "invalid generation";
		return;
	}
	if (!batch_id_is_valid(record + 16)) {
		out->reason = "invalid batch";
		return;
	}
	uint32_t date = read_le32(record + 32);
	if (!production_date_is_valid(date)) {
		out->reason = "invalid production date";
		return;
	}
	if (record[56] < 'A' || record[56] > 'Z' || record[57] < 'A' || record[57] > 'Z') {
		out->reason = "invalid region";
		return;
	}
	if (record[58] != 0xFFU || record[59] != 0xFFU) {
		out->reason = "invalid reserved bytes";
		return;
	}
	out->info.product_id = read_le16(record + 8);
	out->info.hardware_revision = read_le16(record + 10);
	out->info.skt0_generation = read_le32(record + 12);
	memcpy(out->info.batch, record + 16, sizeof(out->info.batch));
	out->info.production_date = date;
	memcpy(out->info.factory_app_version, record + 36, sizeof(out->info.factory_app_version));
	memcpy(out->info.provenance_sha256_prefix, record + 40, sizeof(out->info.provenance_sha256_prefix));
	out->info.skt0_region[0] = (char)record[56];
	out->info.skt0_region[1] = (char)record[57];
	out->status = CUSTOMER_INFO_VALID;
	out->reason = "schema and CRC32 valid";
}
