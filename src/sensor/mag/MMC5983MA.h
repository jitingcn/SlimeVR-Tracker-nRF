/* 06/14/2020 Copyright Tlera Corporation

	Created by Kris Winer

  The MMC5983MA is a low power magnetometer, here used as 3 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#ifndef MMC5983MA_h
#define MMC5983MA_h

#include "sensor/sensor.h"

// https://www.memsic.com/Public/Uploads/uploadfile/files/20220119/MMC5983MADatasheetRevA.pdf
#define MMC5983MA_XOUT_0        0x00
#define MMC5983MA_TOUT          0x07

#define MMC5983MA_STATUS        0x08
#define MMC5983MA_CONTROL_0     0x09
#define MMC5983MA_CONTROL_1     0x0A
#define MMC5983MA_CONTROL_2     0x0B

// Status and command bits (Rev. A, Internal Control 0/1/2).
#define MMC5983MA_STATUS_MEAS_M_DONE 0x01
#define MMC5983MA_CTRL0_TAKE_MEAS_M 0x01
#define MMC5983MA_CTRL0_TAKE_MEAS_T 0x02
#define MMC5983MA_CTRL0_SET 0x08
#define MMC5983MA_CTRL0_RESET 0x10
#define MMC5983MA_CTRL0_AUTO_SR_EN 0x20
#define MMC5983MA_CTRL1_SW_RESET 0x80
#define MMC5983MA_CTRL2_CMM_EN 0x08
#define MMC5983MA_CTRL2_EN_PRD_SET 0x80

// Sample rates
#define MODR_ONESHOT   0x00
#define MODR_1Hz       0x01
#define MODR_10Hz      0x02
#define MODR_20Hz      0x03
#define MODR_50Hz      0x04
#define MODR_100Hz     0x05
#define MODR_200Hz     0x06 // BW = 0x01 only
#define MODR_1000Hz    0x07 // BW = 0x11 only

//Bandwidths
#define MBW_100Hz 0x00  // 8 ms measurement time
#define MBW_200Hz 0x01  // 4 ms
#define MBW_400Hz 0x02  // 2 ms
#define MBW_800Hz 0x03  // 0.5 ms

// Periodic SET interval in measurements (not periodic RESET).
#define MSET_1 0x00     // SET each data measurement
#define MSET_25    0x01 // each 25 data measurements
#define MSET_75    0x02
#define MSET_100   0x03
#define MSET_250   0x04
#define MSET_500   0x05
#define MSET_1000  0x06
#define MSET_2000  0x07

int mmc_init(float period_s, float *actual_period_s);
void mmc_shutdown(void);

int mmc_update_odr(float period_s, float *actual_period_s);

void mmc_mag_oneshot(void);
bool mmc_mag_read(float m[3]);
float mmc_temp_read(float bias[3]);

void mmc_mag_process(uint8_t *raw_m, float m[3]);

extern const sensor_mag_t sensor_mag_mmc5983ma;

#endif
