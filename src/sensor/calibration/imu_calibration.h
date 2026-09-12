#ifndef SLIMENRF_IMU_CALIBRATION_INTERNAL_H
#define SLIMENRF_IMU_CALIBRATION_INTERNAL_H

#include "calibration.h"

enum sensor_calibration_effect {
	SENSOR_CALIBRATION_UNCHANGED,
	SENSOR_CALIBRATION_SAVE_FUSION,
	SENSOR_CALIBRATION_COEFFICIENTS_CHANGED,
	SENSOR_CALIBRATION_BIAS_CHANGED,
	SENSOR_CALIBRATION_FRAME_CHANGED,
};

/* Owner-only: startup before sampling, then one application per frame. */
void sensor_calibration_imu_load(void);
bool sensor_calibration_imu_ready(void);
/* Sensor lifecycle, separate from one-time coefficient loading. */
void sensor_calibration_set_consumer_ready(bool ready);
enum sensor_calibration_effect sensor_calibration_apply_pending(void);
void sensor_calibration_persist_pending(void);
bool sensor_calibration_fusion_stale(void);
void sensor_calibration_fusion_applied(void);
void sensor_calibration_apply_accel(float a[3]);
void sensor_calibration_subtract_gyro_bias(float g[3]);
void sensor_calibration_gyro_bias(float out[3]);

#endif
