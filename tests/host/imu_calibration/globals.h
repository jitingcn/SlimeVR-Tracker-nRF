#ifndef TEST_IMU_CALIBRATION_GLOBALS_H
#define TEST_IMU_CALIBRATION_GLOBALS_H

#include <stdbool.h>
#include <stdint.h>

struct host_retained {
	float accelBias[3];
	float gyroBias[3];
	float accBAinv[4][3];
	struct {
		bool doffset_valid;
		float doffset[3];
	} bootCalState;
	uint8_t fusion_id;
	float gyroSensScale[3];
	uint64_t build_timestamp;
};

extern struct host_retained *retained;
void retained_update(void);
void host_log_error(const char *format, ...);
void host_log_warning(const char *format, ...);
#define LOG_LEVEL_INF 3
#define LOG_MODULE_REGISTER(name, level)
#define LOG_ERR(...) host_log_error(__VA_ARGS__)
#define LOG_WRN(...) host_log_warning(__VA_ARGS__)

#endif
