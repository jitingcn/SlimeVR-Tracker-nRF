#ifndef LIS2MDL_h
#define LIS2MDL_h

#include "sensor/sensor.h"

// https://www.st.com/resource/en/datasheet/lis2mdl.pdf
#define LIS2MDL_CFG_REG_A 0x60

#define LIS2MDL_OUTX_L_REG 0x68

#define LIS2MDL_TEMP_OUT_L_REG 0x6E

#define ODR_10Hz  0x00
#define ODR_20Hz  0x01
#define ODR_50Hz  0x02
#define ODR_100Hz 0x03

#define MD_CONTINUOUS 0x00
#define MD_SINGLE     0x01 // Performs oneshot, then switches to idle
#define MD_IDLE       0x03

int lis2_init(float time, float *actual_time);
void lis2_shutdown(void);

int lis2_update_odr(float time, float *actual_time);

void lis2_mag_oneshot(void);
bool lis2_mag_read(float m[3]);
float lis2_temp_read(float bias[3]);

void lis2_mag_process(uint8_t *raw_m, float m[3]);

extern const sensor_mag_t sensor_mag_lis2mdl;

#endif
