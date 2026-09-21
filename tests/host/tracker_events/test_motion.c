#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "sensor/motion_state.h"
#include "util.h"

static const float stationary_accel[3] = {0.0f, 0.0f, 1.0f};

static void settle(struct sensor_rest_detector *state)
{
	sensor_rest_detector_reset(state);
	sensor_rest_detector_update_accel(state, stationary_accel, 0.01f);
	struct sensor_rest_evidence evidence;
	sensor_rest_detector_take(state, &evidence);
	assert(sensor_motion_is_quiet(0.0f, 0.0f, &evidence));
}

static void accel_gates_and_fifo_peak(void)
{
	struct sensor_rest_detector state;
	settle(&state);
	struct sensor_rest_evidence evidence;
	float pulse[3] = {0.15f / CONST_EARTH_GRAVITY, 0.0f, 1.0f};
	sensor_rest_detector_update_accel(&state, pulse, 0.01f);
	sensor_rest_detector_update_accel(&state, stationary_accel, 0.01f);
	sensor_rest_detector_take(&state, &evidence);
	assert(!sensor_motion_is_quiet(0.0f, 0.0f, &evidence));
	assert(!sensor_motion_is_active(0.0f, 0.0f, &evidence));
	pulse[0] = 0.30f / CONST_EARTH_GRAVITY;
	sensor_rest_detector_update_accel(&state, pulse, 0.01f);
	sensor_rest_detector_update_accel(&state, stationary_accel, 0.01f);
	sensor_rest_detector_take(&state, &evidence);
	assert(sensor_motion_is_active(0.0f, 0.0f, &evidence));
	/* Held data is not refiltered, even across many empty acquisitions. */
	sensor_rest_detector_take(&state, &evidence);
	float held = evidence.accel_deviation_squared_m2_s4;
	assert(held > 0.0f);
	for (int i = 0; i < 20; i++) {
		sensor_rest_detector_take(&state, &evidence);
		assert(evidence.accel_deviation_squared_m2_s4 == held);
	}
}

static void first_order_periods_and_hysteresis(void)
{
	struct sensor_rest_detector state;
	settle(&state);
	struct sensor_rest_evidence evidence;
	const float step[3] = {0.30f / CONST_EARTH_GRAVITY, 0.0f, 1.0f};
	float elapsed = 0.0f;
	int active_samples = 0;
	for (int i = 0; i < 100; i++) {
		float dt = i < 50 ? 0.01f : 0.02f;
		elapsed += dt;
		sensor_rest_detector_update_accel(&state, step, dt);
		sensor_rest_detector_take(&state, &evidence);
		assert(fabsf(sqrtf(evidence.accel_deviation_squared_m2_s4) - 0.30f * expf(-elapsed)) < 0.00001f);
		if (sensor_motion_is_active(0.0f, 0.0f, &evidence)) active_samples++;
		if (elapsed > 0.20f && elapsed < 1.0f) {
			assert(!sensor_motion_is_active(0.0f, 0.0f, &evidence));
			assert(!sensor_motion_is_quiet(0.0f, 0.0f, &evidence));
		}
	}
	assert(active_samples == 18); /* .3 step exceeds .25 for .1823s, not 250ms. */
	assert(sensor_motion_is_quiet(0.0f, 0.0f, &evidence));
}

static void invalid_and_strict_gates(void)
{
	struct sensor_rest_detector state;
	struct sensor_rest_evidence evidence;
	sensor_rest_detector_reset(&state);
	sensor_rest_detector_take(&state, &evidence);
	assert(!sensor_motion_is_quiet(0, 0, &evidence));
	const float invalid[][3] = {{NAN,0,1}, {0,INFINITY,1}, {0,0,0}};
	for (unsigned i = 0; i < 3; i++) {
		settle(&state);
		sensor_rest_detector_update_accel(&state, invalid[i], .01f);
		sensor_rest_detector_update_accel(&state, stationary_accel, .01f);
		sensor_rest_detector_take(&state, &evidence);
		assert(!sensor_motion_is_quiet(0, 0, &evidence));
		sensor_rest_detector_take(&state, &evidence);
		assert(sensor_motion_is_quiet(0, 0, &evidence));
	}
	sensor_rest_detector_update_accel(&state, stationary_accel, 0);
	sensor_rest_detector_take(&state, &evidence);
	assert(!sensor_motion_is_quiet(0, 0, &evidence));
	settle(&state);
	sensor_rest_detector_take(&state, &evidence);
	evidence.accel_deviation_squared_m2_s4 = .10f * .10f;
	assert(!sensor_motion_is_quiet(0, 0, &evidence));
	evidence.accel_deviation_squared_m2_s4 = .25f * .25f;
	assert(!sensor_motion_is_active(0, 0, &evidence));
	evidence.accel_deviation_squared_m2_s4 = 0;
	const float rad = 3.14159265358979323846f / 180.0f;
	assert(!sensor_motion_is_quiet(.5f, 0, &evidence));
	/* Upper rest policy: entry below .60deg, exit above1.0deg. */
	assert(!sensor_motion_is_quiet(0, .60f * rad, &evidence));
	assert(sensor_motion_is_quiet(0, .599f * rad, &evidence));
	assert(!sensor_motion_is_active(1.2f, 1.0f * rad, &evidence));
	assert(sensor_motion_is_active(1.21f, 0, &evidence));
	assert(sensor_motion_is_active(0, 1.001f * rad, &evidence));
}

int main(void)
{
	accel_gates_and_fifo_peak();
	first_order_periods_and_hysteresis();
	invalid_and_strict_gates();
	puts("motion: accel EMA/FIFO peaks, held evidence, invalid input and strict physical gates passed");
	return 0;
}
