#ifndef CUSTOMER_INFO_PARSER_H
#define CUSTOMER_INFO_PARSER_H

#include <stddef.h>
#include <stdint.h>

#define CUSTOMER_INFO_RECORD_SIZE 64U

enum customer_info_status {
	CUSTOMER_INFO_ABSENT,
	CUSTOMER_INFO_UNRECOGNIZED,
	CUSTOMER_INFO_UNSUPPORTED,
	CUSTOMER_INFO_INVALID,
	CUSTOMER_INFO_VALID,
};

enum customer_info_format {
	CUSTOMER_INFO_FORMAT_NONE,
	CUSTOMER_INFO_FORMAT_SNCI_V1,
	CUSTOMER_INFO_FORMAT_SKT0_V1,
};

enum customer_info_source {
	CUSTOMER_INFO_SOURCE_NONE,
	CUSTOMER_INFO_SOURCE_SLOT_B,
	CUSTOMER_INFO_SOURCE_SLOT_A,
};

enum customer_info_identity {
	CUSTOMER_INFO_IDENTITY_NOT_CHECKED,
	CUSTOMER_INFO_IDENTITY_MATCH,
	CUSTOMER_INFO_IDENTITY_MISMATCH,
};

struct customer_info {
	char manufacturer[8];
	uint16_t product_id;
	uint16_t hardware_revision;
	char batch[16];
	uint32_t production_date;
	uint8_t factory_app_version[4];
	uint8_t provenance_sha256_prefix[16];
	/* Legacy SKT0 fields only; manufacturer is empty for this format. */
	uint32_t skt0_generation;
	char skt0_region[3];
};

struct customer_info_result {
	enum customer_info_status status;
	const char *reason;
	enum customer_info_format format;
	enum customer_info_source source;
	enum customer_info_identity identity;
	struct customer_info info;
};

/* Parse SNCI only; info is zeroed unless valid. Source and identity are unset. */
void customer_info_parse(const uint8_t *record, size_t length, struct customer_info_result *out);
const char *customer_info_status_str(enum customer_info_status status);
enum customer_info_identity customer_info_check_identity(
	const struct customer_info_result *result, const char *manufacturer, uint16_t product, uint16_t hardware);

#endif
