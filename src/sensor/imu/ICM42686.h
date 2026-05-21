/* 01/14/2022 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (ladybug default), respectively,
  and it uses the Ladybug STM32L432 Breakout Board.
  The ICM42686-P is a combo sensor with embedded accel and gyro,
  here used as 6 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#ifndef ICM42686_h
#define ICM42686_h

#include "sensor/sensor.h"

// ICM-42686-P Datasheet DS-000348
// User Bank 0
#define ICM42686_DEVICE_CONFIG             0x11
#define ICM42686_FIFO_CONFIG               0x16

#define ICM42686_INT_CONFIG                0x14

#define ICM42686_TEMP_DATA1                0x1D
#define ICM42686_ACCEL_DATA_X1             0x1F
#define ICM42686_GYRO_DATA_X1              0x25

#define ICM42686_INT_STATUS                0x2D

#define ICM42686_FIFO_COUNTH               0x2E
#define ICM42686_FIFO_DATA                 0x30

#define ICM42686_INTF_CONFIG0              0x4C
#define ICM42686_INTF_CONFIG1              0x4D

#define ICM42686_PWR_MGMT0                 0x4E

#define ICM42686_GYRO_CONFIG0              0x4F
#define ICM42686_ACCEL_CONFIG0             0x50
#define ICM42686_GYRO_ACCEL_CONFIG0        0x52

#define ICM42686_SMD_CONFIG                0x57

#define ICM42686_FIFO_CONFIG1              0x5F
#define ICM42686_FIFO_CONFIG2              0x60

#define ICM42686_INT_CONFIG0               0x63
#define ICM42686_INT_CONFIG1               0x64

#define ICM42686_INT_SOURCE0               0x65
#define ICM42686_INT_SOURCE1               0x66

#define ICM42686_REG_BANK_SEL              0x76

// User Bank 1
#define ICM42686_INTF_CONFIG5              0x7B

// User Bank 4
#define ICM42686_ACCEL_WOM_X_THR           0x4A
#define ICM42686_ACCEL_WOM_Y_THR           0x4B
#define ICM42686_ACCEL_WOM_Z_THR           0x4C

// Accelerometer full scale selection
#define ICM42686_AFS_32G                   0x00
#define ICM42686_AFS_16G                   0x01
#define ICM42686_AFS_8G                    0x02
#define ICM42686_AFS_4G                    0x03
#define ICM42686_AFS_2G                    0x04

// Gyroscope full scale selection
#define ICM42686_GFS_4000DPS               0x00
#define ICM42686_GFS_2000DPS               0x01
#define ICM42686_GFS_1000DPS               0x02
#define ICM42686_GFS_500DPS                0x03
#define ICM42686_GFS_250DPS                0x04
#define ICM42686_GFS_125DPS                0x05
#define ICM42686_GFS_62_50DPS              0x06
#define ICM42686_GFS_31_25DPS              0x07

// Accelerometer ODR selection
// Low Noise mode
#define ICM42686_AODR_32kHz                0x01
#define ICM42686_AODR_16kHz                0x02
#define ICM42686_AODR_8kHz                 0x03
#define ICM42686_AODR_4kHz                 0x04
#define ICM42686_AODR_2kHz                 0x05
#define ICM42686_AODR_1kHz                 0x06

// Low Noise or Low Power modes
#define ICM42686_AODR_500Hz                0x0F
#define ICM42686_AODR_200Hz                0x07
#define ICM42686_AODR_100Hz                0x08
#define ICM42686_AODR_50Hz                 0x09
#define ICM42686_AODR_25Hz                 0x0A
#define ICM42686_AODR_12_5Hz               0x0B

// Low Power mode
#define ICM42686_AODR_6_25Hz               0x0C
#define ICM42686_AODR_3_125Hz              0x0D
#define ICM42686_AODR_1_5625Hz             0x0E

// Gyroscope ODR selection
#define ICM42686_GODR_32kHz                0x01
#define ICM42686_GODR_16kHz                0x02
#define ICM42686_GODR_8kHz                 0x03
#define ICM42686_GODR_4kHz                 0x04
#define ICM42686_GODR_2kHz                 0x05
#define ICM42686_GODR_1kHz                 0x06
#define ICM42686_GODR_500Hz                0x0F
#define ICM42686_GODR_200Hz                0x07
#define ICM42686_GODR_100Hz                0x08
#define ICM42686_GODR_50Hz                 0x09
#define ICM42686_GODR_25Hz                 0x0A
#define ICM42686_GODR_12_5Hz               0x0B

// Power modes
#define ICM42686_aMode_OFF                 0x01
#define ICM42686_aMode_LP                  0x02
#define ICM42686_aMode_LN                  0x03

#define ICM42686_gMode_OFF                 0x00
#define ICM42686_gMode_SBY                 0x01
#define ICM42686_gMode_LN                  0x03

int icm42686_init(float clock_rate, float accel_time, float gyro_time,
                  float *accel_actual_time, float *gyro_actual_time);
void icm42686_shutdown(void);

void icm42686_update_fs(float accel_range, float gyro_range,
                        float *accel_actual_range, float *gyro_actual_range);
int icm42686_update_odr(float accel_time, float gyro_time,
                        float *accel_actual_time, float *gyro_actual_time);

uint16_t icm42686_fifo_read(uint8_t *data, uint16_t len);
int icm42686_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3]);
void icm42686_accel_read(float a[3]);
void icm42686_gyro_read(float g[3]);
float icm42686_temp_read(void);

uint8_t icm42686_setup_DRDY(uint16_t threshold);
uint8_t icm42686_setup_WOM(void);

extern const sensor_imu_t sensor_imu_icm42686;

#endif