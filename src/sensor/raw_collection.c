#include "raw_collection.h"

#include "connection/connection.h"
#include "util.h"

#include <math.h>
#include <string.h>
#include <zephyr/sys/atomic.h>

/* Same N as fusion unless batch collection requests a lower rate. */
struct raw_rate_plan {
	uint8_t n_raw;
	uint8_t emit_count;
	float chip_hz;
	float send_hz;
	float fusion_hz;
	bool batch_active;
	float batch_accum;
	float batch_interval;
};

static struct raw_rate_plan raw_rate_plan = {.n_raw = 1};
static atomic_t batch_collect_active;
static atomic_t batch_collect_rate_hz;
static atomic_t batch_collect_generation;
static bool last_data_collection_state;
static atomic_val_t last_batch_collect_generation;
static float gyro_actual_time;
/* Integrate uncorrected gyro so offline fusion can re-estimate bias. */
static float raw_gyr_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float raw_collect_a[3];

void sensor_set_batch_collect(bool active, float emit_hz)
{
	uint32_t rate_hz = emit_hz > 0.0f ? (uint32_t)emit_hz : 0;
	atomic_set(&batch_collect_rate_hz, (atomic_val_t)rate_hz);
	atomic_set(&batch_collect_active, active ? 1 : 0);
	atomic_inc(&batch_collect_generation);
}

static void raw_rate_plan_refresh(const struct sensor_raw_collection_config *config)
{
	gyro_actual_time = config->gyro_period;
	float chip_hz = 1.0f / gyro_actual_time;
	uint8_t n_raw = config->gyro_oversample_n > 0 ? config->gyro_oversample_n : 1;
	bool batch_active = atomic_get(&batch_collect_active) != 0;
	float requested_hz = (float)atomic_get(&batch_collect_rate_hz);
	float accel_hz = 1.0f / config->accel_period;
	float emit_hz = requested_hz > 0.0f ? requested_hz : accel_hz;
	if (emit_hz > chip_hz) {
		emit_hz = chip_hz;
	}
	if (emit_hz <= 0.0f) {
		emit_hz = chip_hz;
	}

	raw_rate_plan.n_raw = n_raw;
	raw_rate_plan.chip_hz = chip_hz;
	raw_rate_plan.fusion_hz = chip_hz / (float)n_raw;
	raw_rate_plan.batch_active = batch_active;
	raw_rate_plan.batch_interval = batch_active ? chip_hz / emit_hz : 0.0f;
	raw_rate_plan.send_hz = batch_active ? emit_hz : raw_rate_plan.fusion_hz;
}

void sensor_raw_collection_send_metadata(const struct sensor_raw_collection_config *config)
{
	raw_rate_plan_refresh(config);
	connection_send_raw_metadata(
		config->gyro_range,
		config->accel_range,
		raw_rate_plan.send_hz,
		1.0f / config->accel_period,
		config->mag_active ? 1.0f / config->mag_period : 0.0f,
		config->imu_id,
		config->mag_id,
		raw_rate_plan.chip_hz,
		raw_rate_plan.fusion_hz
	);
}

bool sensor_raw_collection_begin_frame(bool *dc_active)
{
	/* A generation change catches an in-place rate update or batch->single cutover. */
	*dc_active = connection_get_data_collection() || connection_get_data_collection_batch();
	atomic_val_t generation = atomic_get(&batch_collect_generation);
	bool config_changed = *dc_active && generation != last_batch_collect_generation;
	bool restart = *dc_active && (!last_data_collection_state || config_changed);
	if (restart) {
		raw_gyr_quat[0] = 1.0f;
		raw_gyr_quat[1] = 0.0f;
		raw_gyr_quat[2] = 0.0f;
		raw_gyr_quat[3] = 0.0f;
		raw_rate_plan.batch_accum = 0.0f;
	}
	/* Metadata/calibration are session-stable; no periodic resend. */
	last_batch_collect_generation = generation;
	last_data_collection_state = *dc_active;
	return restart;
}

void sensor_raw_collection_on_sample(const float raw_a[3], const float raw_g[3], float temp_c, bool active)
{
	/* Pair the latest accel tag with the next emitted gyro tag once. */
	if (raw_a[0] != 0 || raw_a[1] != 0 || raw_a[2] != 0) {
		memcpy(raw_collect_a, raw_a, sizeof(raw_collect_a));
	}

	/* Zero gyro vectors are absent tags, including while inactive. */
	if (raw_g[0] != 0 || raw_g[1] != 0 || raw_g[2] != 0) {
		if (active) {
			/* Integrate every chip sample; queue only at the raw emit rate. */
			float g_rad[3] = {raw_g[0] * DEG_TO_RAD, raw_g[1] * DEG_TO_RAD, raw_g[2] * DEG_TO_RAD};
			float gyr_norm = sqrtf(g_rad[0] * g_rad[0] + g_rad[1] * g_rad[1] + g_rad[2] * g_rad[2]);
			if (gyr_norm > 1e-6f) {
				float angle = gyr_norm * gyro_actual_time;
				float ha = angle * 0.5f;
				float s = sinf(ha) / gyr_norm;
				float step[4] = {cosf(ha), s * g_rad[0], s * g_rad[1], s * g_rad[2]};
				/* q_new = q_old * step */
				float q0 = raw_gyr_quat[0] * step[0] - raw_gyr_quat[1] * step[1] - raw_gyr_quat[2] * step[2]
						 - raw_gyr_quat[3] * step[3];
				float q1 = raw_gyr_quat[0] * step[1] + raw_gyr_quat[1] * step[0] + raw_gyr_quat[2] * step[3]
						 - raw_gyr_quat[3] * step[2];
				float q2 = raw_gyr_quat[0] * step[2] - raw_gyr_quat[1] * step[3] + raw_gyr_quat[2] * step[0]
						 + raw_gyr_quat[3] * step[1];
				float q3 = raw_gyr_quat[0] * step[3] + raw_gyr_quat[1] * step[2] - raw_gyr_quat[2] * step[1]
						 + raw_gyr_quat[3] * step[0];
				float inv_norm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
				raw_gyr_quat[0] = q0 * inv_norm;
				raw_gyr_quat[1] = q1 * inv_norm;
				raw_gyr_quat[2] = q2 * inv_norm;
				raw_gyr_quat[3] = q3 * inv_norm;
			}

			bool emit_raw = false;
			if (raw_rate_plan.batch_active) {
				raw_rate_plan.batch_accum += 1.0f;
				if (raw_rate_plan.batch_accum >= raw_rate_plan.batch_interval) {
					raw_rate_plan.batch_accum -= raw_rate_plan.batch_interval;
					emit_raw = true;
				}
			} else {
				raw_rate_plan.emit_count++;
				if (raw_rate_plan.emit_count >= raw_rate_plan.n_raw) {
					raw_rate_plan.emit_count = 0;
					emit_raw = true;
				}
			}
			if (emit_raw) {
				struct raw_imu_sample raw_sample;
				memcpy(raw_sample.gyr_quat, raw_gyr_quat, sizeof(raw_sample.gyr_quat));
				memcpy(raw_sample.accel, raw_collect_a, sizeof(raw_sample.accel));
				raw_sample.temp_c = temp_c;
				connection_queue_raw_sample(&raw_sample);
				/* Clear only on emit so mid-window accel tags stay latched. */
				memset(raw_collect_a, 0, sizeof(raw_collect_a));
			}
		} else {
			memset(raw_collect_a, 0, sizeof(raw_collect_a));
		}
	}
}
