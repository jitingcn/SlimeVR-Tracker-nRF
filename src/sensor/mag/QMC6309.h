#ifndef QMC6309_h
#define QMC6309_h

#include "sensor/sensor.h"

int qmc_init(float time, float *actual_time);
void qmc_shutdown(void);

int qmc_update_odr(float time, float *actual_time);

void qmc_mag_oneshot(void);
bool qmc_mag_read(float m[3]);

void qmc_mag_process(uint8_t *raw_m, float m[3]);

extern const sensor_mag_t sensor_mag_qmc6309;

#endif
