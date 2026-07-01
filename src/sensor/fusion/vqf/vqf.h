/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#ifndef SLIMENRF_VQF
#define SLIMENRF_VQF

#include <stdint.h>

#include "sensor/sensor.h"

void vqf_update_sensor_ids(int imu);

void vqf_init(float g_time, float a_time, float m_time);
void vqf_load(const void *data);
void vqf_save(void *data);

void vqf_update_gyro(float *g, float time);
void vqf_update_accel(float *a, float time);
void vqf_update_mag(float *m, float time);
void vqf_update(float *g, float *a, float *m, float time);

void vqf_update_gyro_ts(float *g, uint64_t timestamp_us);
void vqf_update_accel_ts(float *a, uint64_t timestamp_us);
void vqf_update_mag_ts(float *m, uint64_t timestamp_us);

void vqf_get_gyro_bias(float *g_off);
void vqf_set_gyro_bias(float *g_off);

void vqf_update_gyro_sanity(float *g, float *m);
int vqf_get_gyro_sanity(void);

void vqf_get_lin_a(float *lin_a);
void vqf_get_quat(float *q);

bool vqf_get_rest_detected(void);
bool vqf_get_mag_dist_detected(void);
void vqf_reset_mag_ref(void);
void vqf_set_mag_ref(float norm, float dip);
float vqf_get_mag_ref_norm(void);
void vqf_get_mag_ref(float *norm, float *dip);
float vqf_get_delta(void);
void vqf_set_delta(float delta);
void vqf_get_relative_rest_deviations(float *out);

// Debug information structure
//
// Units:
// - bias, bias_sigma: °/s
// - delta, mag_*_dip, mag_*_dis_angle: degrees
// - mag_*_corr_rate: °/s
// - mag_*_t: seconds
// - mag_*_norm: same unit as magnetometer input (depends on driver calibration)
typedef struct {
    bool rest_detected;
    float rest_deviations[2];  // [gyr, acc]
    float bias[3];             // °/s
    float bias_sigma;          // °/s

    // Heading correction state
    float delta;               // degrees

    // Magnetic disturbance / reference
    bool mag_dist_detected;
    float mag_ref_norm;
    float mag_ref_dip;         // degrees

    // Current magnetic field (after optional magCurrentTau LPF)
    float mag_norm;
    float mag_dip;             // degrees

    // Heading correction diagnostics (from last magnetometer update)
    float mag_dis_angle;       // degrees (lastMagDisAngle)
    float mag_corr_rate;       // °/s (lastMagCorrAngularRate)

    // Disturbance rejection timers
    float mag_undisturbed_t;   // seconds
    float mag_reject_t;        // seconds

    // Candidate field tracking
    float mag_candidate_norm;
    float mag_candidate_dip;   // degrees
    float mag_candidate_t;     // seconds

    // Filter gains (useful to understand how strong mag correction is)
    float mag_k;               // dimensionless (kMag)
    float mag_k_init;          // dimensionless (kMagInit)
#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
    // Adaptive tauAcc state
    float tau_acc;             // current tauAcc value (seconds)
    float motion_intensity;    // motion intensity estimate [0, 1]
#endif
    // Rest detection diagnostics (cumulative since boot/init)
    uint32_t rest_enter_count;     // number of transitions to rest
    uint32_t rest_exit_count;      // number of transitions from rest
    float rest_total_s;            // cumulative time in rest state (seconds)
    float rest_last_duration_s;    // duration of the most recent rest period (seconds)
    float uptime_s;                // total uptime since init (seconds)

    // Recent rest events log (circular, most recent last)
    #define VQF_REST_EVENT_LOG_SIZE 5
    struct {
        float time_s;  // uptime when event occurred
        bool entered;   // true = entered rest, false = left rest
    } rest_events[VQF_REST_EVENT_LOG_SIZE];
    uint8_t rest_event_count;      // total events recorded (wraps at log size)

    // Kalman filter state (P diagonal, internal units: (0.01rad/s)^2)
    float biasP[3];                // P[0,0], P[1,1], P[2,2]
} vqf_debug_info_t;

void vqf_get_debug_info(vqf_debug_info_t *info);
#if defined(CONFIG_VQF_BENCH)
void vqf_run_benchmark(uint32_t iterations);
#endif

extern const sensor_fusion_t sensor_fusion_vqf;

#endif
