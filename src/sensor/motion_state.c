#include "motion_state.h"
#include "util.h"

#include <math.h>
#include <string.h>

/* Upper motion policy owns this time constant, independently of fusion. */
#define REST_REFERENCE_TAU_S 1.0f

void sensor_rest_detector_reset(struct sensor_rest_detector *state)
{
	memset(state, 0, sizeof(*state));
}

static void rest_observation_invalidate(struct sensor_rest_observation *state)
{
	memset(state, 0, sizeof(*state));
	state->frame_invalid = true;
}

static void rest_observation_update(struct sensor_rest_observation *state, float squared)
{
	if (!v_finite(&squared, 1)) {
		rest_observation_invalidate(state);
		return;
	}
	state->last_squared = squared;
	if (!state->frame_seen || squared > state->frame_peak_squared) {
		state->frame_peak_squared = squared;
	}
	state->valid = true;
	state->frame_seen = true;
}

static bool rest_period_valid(float dt)
{
	return v_finite(&dt, 1) && dt > 0.0f && dt <= 2.0f * REST_REFERENCE_TAU_S;
}


void sensor_rest_detector_update_accel(struct sensor_rest_detector *state, const float accel[3], float dt)
{
	float input[3] = {
		accel[0] * CONST_EARTH_GRAVITY,
		accel[1] * CONST_EARTH_GRAVITY,
		accel[2] * CONST_EARTH_GRAVITY,
	};
	/* Bit checks remain effective with fast-math/CMSIS, including overflow. */
	if (!v_finite(input, 3) || !rest_period_valid(dt)
		|| (input[0] == 0.0f && input[1] == 0.0f && input[2] == 0.0f)) {
		rest_observation_invalidate(&state->accel);
		return;
	}
	if (state->accel_period_s != dt) {
		state->accel_alpha = -expm1f(-dt / REST_REFERENCE_TAU_S);
		state->accel_period_s = dt;
	}
	/* The first usable sample seeds the reference; entry dwell provides
	 * confidence instead of a separate filter warmup state machine. */
	if (!state->accel.valid) {
		memcpy(state->accel_reference_ms2, input, sizeof(input));
	}
	float squared_deviation = 0.0f;
	for (int i = 0; i < 3; i++) {
		state->accel_reference_ms2[i] += state->accel_alpha * (input[i] - state->accel_reference_ms2[i]);
		float residual = input[i] - state->accel_reference_ms2[i];
		squared_deviation += residual * residual;
	}
	rest_observation_update(&state->accel, squared_deviation);
}

static float rest_observation_take(struct sensor_rest_observation *state, bool *valid)
{
	*valid = state->valid && !state->frame_invalid;
	float squared = state->frame_seen ? state->frame_peak_squared : state->last_squared;
	state->frame_seen = false;
	state->frame_invalid = false;
	return squared;
}

void sensor_rest_detector_take(struct sensor_rest_detector *state, struct sensor_rest_evidence *evidence)
{
	evidence->accel_deviation_squared_m2_s4 = rest_observation_take(&state->accel, &evidence->accel_valid);
}

#define REST_ENTER_LIN_ACCEL_MS2 0.5f
#define REST_EXIT_LIN_ACCEL_MS2 1.2f
#define REST_ENTER_QUAT_RAD (0.60f * (3.14159265358979323846f / 180.0f))
#define REST_EXIT_QUAT_RAD (1.20f * (3.14159265358979323846f / 180.0f))
#define REST_ENTER_ACCEL_DEV_MS2 0.10f
#define REST_EXIT_ACCEL_DEV_MS2 0.25f

bool sensor_motion_is_quiet(
	float lin_accel_ms2,
	float quat_delta_rad,
	const struct sensor_rest_evidence *evidence)
{
	return evidence->accel_valid
		&& evidence->accel_deviation_squared_m2_s4 < REST_ENTER_ACCEL_DEV_MS2 * REST_ENTER_ACCEL_DEV_MS2
		&& lin_accel_ms2 < REST_ENTER_LIN_ACCEL_MS2
		&& quat_delta_rad < REST_ENTER_QUAT_RAD;
}

bool sensor_motion_is_active(
	float lin_accel_ms2,
	float quat_delta_rad,
	const struct sensor_rest_evidence *evidence)
{
	return (evidence->accel_valid
			&& evidence->accel_deviation_squared_m2_s4 > REST_EXIT_ACCEL_DEV_MS2 * REST_EXIT_ACCEL_DEV_MS2)
		|| lin_accel_ms2 > REST_EXIT_LIN_ACCEL_MS2
		|| quat_delta_rad > REST_EXIT_QUAT_RAD;
}

bool sensor_activity_score_update(
	struct sensor_activity_score *state,
	float angular_speed_dps,
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
	if (angular_speed_dps >= 10.0f || lin_accel_ms2 >= 2.0f)
		weight = 3;
	else if (angular_speed_dps >= 2.5f || lin_accel_ms2 >= 0.8f)
		weight = 1;
	state->value_ms += elapsed_ms * weight;
	if (state->value_ms < 0)
		state->value_ms = 0;
	if (state->value_ms > (int32_t)meaningful_score_ms)
		state->value_ms = meaningful_score_ms;
	return state->value_ms >= (int32_t)meaningful_score_ms;
}
