#ifndef LIS2MDL_h
#define LIS2MDL_h

#include "sensor/sensor.h"

// https://www.st.com/resource/en/datasheet/lis2mdl.pdf
#define LIS2MDL_CFG_REG_A 0x60
#define LIS2MDL_CFG_REG_B 0x61
#define LIS2MDL_CFG_REG_C 0x62
#define LIS2MDL_STATUS_REG 0x67
#define LIS2MDL_OUTX_L_REG 0x68
#define LIS2MDL_TEMP_OUT_L_REG 0x6E

#define CFG_A_COMP_TEMP_EN 0x80
#define CFG_A_SOFT_RST     0x20

#define CFG_B_LPF      0x01 // low-pass filter (BW ODR/4 instead of ODR/2)
#define CFG_B_OFF_CANC 0x02 // offset cancellation (internal set/reset bias cancel)

#define CFG_C_I2C_DIS 0x20 // disable I2C interface (SPI only)
#define CFG_C_BDU     0x10
#define CFG_C_4WSPI   0x04 // enable 4-wire SPI (SDO on pin 7)

#define STATUS_ZYXDA 0x08

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
