/*
 * ABC-EqF n=0: Attitude-Bias Equivariant Filter
 *
 * Based on: Fornasier et al., "Overcoming Bias: Equivariant Filter Design
 * for Biased Attitude Estimation with Online Calibration", RA-L 2022.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef SLIMENRF_EQF
#define SLIMENRF_EQF

#include <stdint.h>
#include "sensor/sensor.h"

void eqf_init(float g_time, float a_time, float m_time);
void eqf_load(const void *data);
void eqf_save(void *data);

void eqf_update_gyro(float *g, float time);
void eqf_update_accel(float *a, float time);
void eqf_update_mag(float *m, float time);
void eqf_update(float *g, float *a, float *m, float time);

void eqf_get_gyro_bias(float *g_off);
void eqf_set_gyro_bias(float *g_off);

void eqf_update_gyro_sanity(float *g, float *m);
int eqf_get_gyro_sanity(void);

void eqf_get_lin_a(float *lin_a);
void eqf_get_quat(float *q);

extern const sensor_fusion_t sensor_fusion_eqf;

#endif
