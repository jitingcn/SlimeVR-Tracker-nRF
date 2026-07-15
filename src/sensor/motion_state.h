#ifndef SLIMENRF_MOTION_STATE_H
#define SLIMENRF_MOTION_STATE_H

#include <stdbool.h>
#include <stdint.h>

struct sensor_activity_score {
	int32_t value_ms;
	int64_t last_update_ms;
};

bool sensor_motion_is_quiet(
	float gyro_speed_dps,
	float lin_accel_ms2,
	float quat_delta_rad,
	bool fusion_rest_detected,
	const float fusion_rest_deviations[2]);
bool sensor_motion_is_active(
	float gyro_speed_dps,
	float lin_accel_ms2,
	float quat_delta_rad,
	const float fusion_rest_deviations[2]);
bool sensor_activity_score_update(
	struct sensor_activity_score *state,
	float gyro_speed_dps,
	float lin_accel_ms2,
	int64_t now_ms,
	uint32_t startup_guard_ms,
	uint32_t meaningful_score_ms);

#endif
