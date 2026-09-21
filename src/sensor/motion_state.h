#ifndef SLIMENRF_MOTION_STATE_H
#define SLIMENRF_MOTION_STATE_H

#include <stdbool.h>
#include <stdint.h>

struct sensor_activity_score {
	int32_t value_ms;
	/* last_update_ms < 0 means "not started" */
	int64_t last_update_ms;
	/* Reference time of this scoring session, so the startup guard does not
	 * depend on uptime being zero at boot. */
	int64_t session_start_ms;
};

/* Per-channel FIFO peak plus the last actual observation for slower ODRs. */
struct sensor_rest_observation {
	float last_squared;
	float frame_peak_squared;
	bool valid;
	bool frame_seen;
	bool frame_invalid;
};

struct sensor_rest_detector {
	float accel_reference_ms2[3];
	float accel_period_s;
	float accel_alpha;
	struct sensor_rest_observation accel;
};

struct sensor_rest_evidence {
	float accel_deviation_squared_m2_s4;
	bool accel_valid;
};

void sensor_rest_detector_reset(struct sensor_rest_detector *state);
/* Fresh calibrated accel samples in g; dt in seconds. */
void sensor_rest_detector_update_accel(struct sensor_rest_detector *state, const float accel[3], float dt);
/* Consume FIFO extrema, retaining last observations without refiltering.
 * Invalid evidence blocks entry but is not itself an exit condition. */
void sensor_rest_detector_take(struct sensor_rest_detector *state, struct sensor_rest_evidence *evidence);

bool sensor_motion_is_quiet(
	float lin_accel_ms2,
	float quat_delta_rad,
	const struct sensor_rest_evidence *evidence);
bool sensor_motion_is_active(
	float lin_accel_ms2,
	float quat_delta_rad,
	const struct sensor_rest_evidence *evidence);
bool sensor_activity_score_update(
	struct sensor_activity_score *state,
	float angular_speed_dps,
	float lin_accel_ms2,
	int64_t now_ms,
	uint32_t startup_guard_ms,
	uint32_t meaningful_score_ms);

#endif
