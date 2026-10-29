#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "zephyr/logging/log.h"

#define HOST_LOG_CAPACITY 128

static struct host_log_entry entries[HOST_LOG_CAPACITY];
static size_t entry_count;
static size_t error_count;

void host_log_reset(void)
{
	entry_count = 0;
	error_count = 0;
}

void host_log(enum host_log_level level, const char *format, ...)
{
	if (level == HOST_LOG_ERROR)
		error_count++;
	if (entry_count == HOST_LOG_CAPACITY)
		return;

	struct host_log_entry *entry = &entries[entry_count++];
	entry->level = level;
	va_list args;
	va_start(args, format);
	vsnprintf(entry->message, sizeof(entry->message), format, args);
	va_end(args);
}

size_t host_log_error_count(void)
{
	return error_count;
}

const struct host_log_entry *host_log_error(size_t index)
{
	for (size_t i = 0; i < entry_count; i++) {
		if (entries[i].level == HOST_LOG_ERROR && index-- == 0)
			return &entries[i];
	}
	return NULL;
}

bool host_log_matches_error(const char *operation, const char *dev, const char *bus,
	const char *controller, const char *address, const char *reg, int error)
{
	if (error_count != 1)
		return false;
	const struct host_log_entry *entry = host_log_error(0);
	if (!entry || entry->level != HOST_LOG_ERROR)
		return false;
	char expected_operation[64], expected_error[32];
	snprintf(expected_operation, sizeof(expected_operation), "SSI %s dev=", operation ? operation : "");
	snprintf(expected_error, sizeof(expected_error), "err=%d", error);
	const char *error_field = strstr(entry->message, expected_error);
	return strstr(entry->message, "SSI ") &&
		(!operation || strstr(entry->message, expected_operation)) &&
		strstr(entry->message, dev) && strstr(entry->message, bus) &&
		strstr(entry->message, controller) && strstr(entry->message, address) &&
		strstr(entry->message, reg) && error_field &&
		error_field[strlen(expected_error)] == '\0';
}
