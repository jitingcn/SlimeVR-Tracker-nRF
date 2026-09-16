#ifndef ONLINE_TEST_GLOBALS_H
#define ONLINE_TEST_GLOBALS_H
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#define LOG_MODULE_REGISTER(...)
void fixture_log(const char *format, ...);
#define LOG_INF(...) fixture_log(__VA_ARGS__)
#define LOG_WRN(...) fixture_log(__VA_ARGS__)
#define LOG_LEVEL_INF 0
#define CLAMP(x, a, b) ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))
#define MAG_ONLINE_CALIBRATION_ENABLED 1
#define MAG_ONLINE_CALIBRATION_DISABLED 2
#define MAG_ONLINE_CALIBRATION_ID 1
#define MAIN_MAG_BIAS_ID 2
struct retained_fixture {
	float magBAinv[4][3];
	struct {
		uint8_t update_count;
		float last_buf_avg_norm;
	} onlineMagState;
	uint8_t mag_online_calibration_mode;
};
extern struct retained_fixture *retained;
#endif
