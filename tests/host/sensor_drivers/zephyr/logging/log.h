#ifndef HOST_ZEPHYR_LOG_H
#define HOST_ZEPHYR_LOG_H

#include <stddef.h>
#include <stdbool.h>

enum host_log_level {
	HOST_LOG_DEBUG,
	HOST_LOG_INFO,
	HOST_LOG_WARNING,
	HOST_LOG_ERROR,
};

struct host_log_entry {
	enum host_log_level level;
	char message[256];
};

void host_log_reset(void);
void host_log(enum host_log_level level, const char *format, ...);
size_t host_log_error_count(void);
const struct host_log_entry *host_log_error(size_t index);
bool host_log_matches_error(const char *operation, const char *dev, const char *bus,
	const char *controller, const char *address, const char *reg, int error);

#define LOG_LEVEL_DBG 0
#define LOG_LEVEL_INF 0
#define LOG_MODULE_REGISTER(...)
#define LOG_DBG(...) host_log(HOST_LOG_DEBUG, __VA_ARGS__)
#define LOG_INF(...) host_log(HOST_LOG_INFO, __VA_ARGS__)
#define LOG_WRN(...) host_log(HOST_LOG_WARNING, __VA_ARGS__)
#define LOG_ERR(...) host_log(HOST_LOG_ERROR, __VA_ARGS__)
#define LOG_HEXDUMP_DBG(...) do { } while (0)

#endif
