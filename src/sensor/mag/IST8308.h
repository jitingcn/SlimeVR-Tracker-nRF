#ifndef IST8308_h
#define IST8308_h

#include "sensor/sensor.h"

#define IST8308_CNTL4   0x33

#define DR_500 0b00
#define DR_200 0b1

int ist8308_init(float time, float *actual_time);
void ist8308_shutdown(void);

int ist8308_update_odr(float time, float *actual_time);

void ist8308_mag_oneshot(void);
bool ist8308_mag_read(float m[3]);

void ist8308_mag_process(uint8_t *raw_m, float m[3]);

extern const sensor_mag_t sensor_mag_ist8308;

#endif
