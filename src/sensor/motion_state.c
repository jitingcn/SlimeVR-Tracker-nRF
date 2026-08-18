#include "motion_state.h"

#define REST_ENTER_GYRO_DPS 1.2f
#define REST_EXIT_GYRO_DPS 3.0f
#define REST_ENTER_LIN_ACCEL_MS2 0.5f
#define REST_EXIT_LIN_ACCEL_MS2 1.2f
#define REST_ENTER_QUAT_RAD 0.015f
#define REST_EXIT_QUAT_RAD 0.025f
#define REST_ENTER_FUSION_DEV 1.5f
#define REST_EXIT_FUSION_DEV 2.0f

bool sensor_motion_is_quiet(
	float gyro_speed_dps,
	float lin_accel_ms2,
	float quat_delta_rad,
	bool fusion_rest_detected,
	const float fusion_rest_deviations[2])
{
	bool local_quiet = gyro_speed_dps < REST_ENTER_GYRO_DPS
		&& lin_accel_ms2 < REST_ENTER_LIN_ACCEL_MS2;
	bool fusion_quiet = fusion_rest_deviations
		&& fusion_rest_deviations[0] < REST_ENTER_FUSION_DEV
		&& fusion_rest_deviations[1] < REST_ENTER_FUSION_DEV;
	bool strong_motion = gyro_speed_dps > REST_EXIT_GYRO_DPS
		|| lin_accel_ms2 > REST_EXIT_LIN_ACCEL_MS2;
	return !strong_motion && quat_delta_rad < REST_ENTER_QUAT_RAD
		&& (local_quiet || fusion_rest_detected || fusion_quiet);
}

bool sensor_motion_is_active(
	float gyro_speed_dps,
	float lin_accel_ms2,
	float quat_delta_rad,
	const float fusion_rest_deviations[2])
{
	bool local_hint = gyro_speed_dps > REST_ENTER_GYRO_DPS
		|| lin_accel_ms2 > REST_ENTER_LIN_ACCEL_MS2
		|| quat_delta_rad > REST_ENTER_QUAT_RAD;
	bool fusion_motion = fusion_rest_deviations
		&& (fusion_rest_deviations[0] > REST_EXIT_FUSION_DEV
			|| fusion_rest_deviations[1] > REST_EXIT_FUSION_DEV);
	return gyro_speed_dps > REST_EXIT_GYRO_DPS
		|| lin_accel_ms2 > REST_EXIT_LIN_ACCEL_MS2
		|| quat_delta_rad > REST_EXIT_QUAT_RAD
		|| (fusion_motion && local_hint);
}

bool sensor_activity_score_update(
	struct sensor_activity_score *state,
	float gyro_speed_dps,
	float lin_accel_ms2,
	int64_t now_ms,
	uint32_t startup_guard_ms,
	uint32_t meaningful_score_ms)
{
	if (state->last_update_ms < 0) {
		state->last_update_ms = now_ms;
		state->session_start_ms = now_ms;
		return false;
	}
	int32_t elapsed_ms = (int32_t)(now_ms - state->last_update_ms);
	state->last_update_ms = now_ms;
	if (elapsed_ms <= 0)
		return state->value_ms >= (int32_t)meaningful_score_ms;
	if (elapsed_ms > 250)
		elapsed_ms = 250;
	if ((uint64_t)(now_ms - state->session_start_ms) <= startup_guard_ms) {
		state->value_ms = 0;
		return false;
	}
	int32_t weight = -1;
	if (gyro_speed_dps >= 10.0f || lin_accel_ms2 >= 2.0f)
		weight = 3;
	else if (gyro_speed_dps >= 2.5f || lin_accel_ms2 >= 0.8f)
		weight = 1;
	state->value_ms += elapsed_ms * weight;
	if (state->value_ms < 0)
		state->value_ms = 0;
	if (state->value_ms > (int32_t)meaningful_score_ms)
		state->value_ms = meaningful_score_ms;
	return state->value_ms >= (int32_t)meaningful_score_ms;
}
