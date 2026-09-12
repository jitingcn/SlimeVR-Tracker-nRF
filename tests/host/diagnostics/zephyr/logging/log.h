#ifndef TEST_DIAGNOSTICS_LOG_H
#define TEST_DIAGNOSTICS_LOG_H

#define LOG_MODULE_DECLARE(...)
#define LOG_INF(...) test_log(__VA_ARGS__)
void test_log(const char *format, ...);

#endif
