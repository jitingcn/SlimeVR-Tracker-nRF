#ifndef HOST_ZEPHYR_LOG_H
#define HOST_ZEPHYR_LOG_H

/* Keep log arguments type-checked/referenced without evaluating them. */
static inline void host_log(const char *format, ...) { (void)format; }

#define LOG_LEVEL_DBG 0
#define LOG_LEVEL_INF 0
#define LOG_MODULE_REGISTER(...)
#define LOG_DBG(...) do { if (0) host_log(__VA_ARGS__); } while (0)
#define LOG_INF(...) do { if (0) host_log(__VA_ARGS__); } while (0)
#define LOG_WRN(...) do { if (0) host_log(__VA_ARGS__); } while (0)
#define LOG_ERR(...) do { if (0) host_log(__VA_ARGS__); } while (0)
#define LOG_HEXDUMP_DBG(...) do { } while (0)

#endif
