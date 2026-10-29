#ifndef SLIMENRF_RAW_COLLECTION_H
#define SLIMENRF_RAW_COLLECTION_H

#include <stdbool.h>
#include <stdint.h>

/* Driver configuration is supplied only when publishing a metadata snapshot. */
struct sensor_raw_collection_config {
	float gyro_period;
	float accel_period;
	float mag_period;
	float gyro_range;
	float accel_range;
	uint8_t gyro_oversample_n;
	uint8_t imu_id;
	uint8_t mag_id;
	bool mag_active;
};

/* Connection control publication; remaining hooks belong to the sensor owner
 * or its exclusive suspended-owner hardware-restart path. */
void sensor_set_batch_collect(bool active, float emit_hz);

/* Set the frame's activity snapshot. True means the caller must resume the
 * hardware, send metadata, then acquire samples. Activation, a new transport
 * session or active batch configuration change reset the quaternion and
 * fractional accumulator; single emit phase and latest acceleration persist. */
bool sensor_raw_collection_begin_frame(bool *dc_active);
void sensor_raw_collection_send_metadata(const struct sensor_raw_collection_config *config);

/* Call before calibration mutates the FIFO vectors, including while inactive.
 * Zero vectors represent absent FIFO tags. No input vector is modified. */
void sensor_raw_collection_on_sample(const float raw_a[3], const float raw_g[3], float temp_c, bool active);

#endif
