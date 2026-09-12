#ifndef QMC5883L_h
#define QMC5883L_h

#include "sensor/sensor.h"

int qmc5883l_init(float period_s, float *actual_period_s);
void qmc5883l_shutdown(void);

int qmc5883l_update_odr(float period_s, float *actual_period_s);

void qmc5883l_mag_oneshot(void);
bool qmc5883l_mag_read(float m[3]);

void qmc5883l_mag_process(uint8_t *raw_m, float m[3]);

extern const sensor_mag_t sensor_mag_qmc5883l;

#endif
