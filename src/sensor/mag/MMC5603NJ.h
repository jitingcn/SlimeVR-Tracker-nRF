#ifndef MMC5603NJ_h
#define MMC5603NJ_h

#include "sensor/sensor.h"

// https://www.memsic.com/Public/Uploads/uploadfile/files/20220119/MMC5603NJDatasheetRev.B.pdf
#define MMC5603NJ_XOUT_0     0x00
#define MMC5603NJ_TOUT       0x09

#define MMC5603NJ_STATUS     0x18
#define MMC5603NJ_ODR        0x1A
#define MMC5603NJ_CONTROL_0  0x1B
#define MMC5603NJ_CONTROL_1  0x1C
#define MMC5603NJ_CONTROL_2  0x1D
#define MMC5603NJ_PRODUCT_ID 0x39

// Status1 bits
#define MSTAT_MEAS_M_DONE 0x40
#define MSTAT_MEAS_T_DONE 0x80

// Control 0 bits
#define MCTRL0_TAKE_MEAS_M  0x01
#define MCTRL0_TAKE_MEAS_T  0x02
#define MCTRL0_DO_SET       0x08
#define MCTRL0_DO_RESET     0x10
#define MCTRL0_AUTO_SR_EN   0x20
#define MCTRL0_CMM_FREQ_EN  0x80

// Control 1 bits
#define MCTRL1_SW_RESET 0x80

// Bandwidths (measurement time)
#define MBW_6_6ms 0x00 // 75Hz max ODR with auto SET/RESET
#define MBW_3_5ms 0x01 // 150Hz
#define MBW_2_0ms 0x02 // 255Hz
#define MBW_1_2ms 0x03 // 255Hz, 1000Hz with hpower

// Control 2 bits
#define MCTRL2_EN_PRD_SET 0x08
#define MCTRL2_CMM_EN     0x10
#define MCTRL2_HPOWER     0x80

// Set/Reset as a function of measurements
#define MSET_1     0x00 // Set/Reset each data measurement
#define MSET_25    0x01 // each 25 data measurements
#define MSET_75    0x02
#define MSET_100   0x03
#define MSET_250   0x04
#define MSET_500   0x05
#define MSET_1000  0x06
#define MSET_2000  0x07

int mmc5603_init(float time, float *actual_time);
void mmc5603_shutdown(void);

int mmc5603_update_odr(float time, float *actual_time);

void mmc5603_mag_oneshot(void);
bool mmc5603_mag_read(float m[3]);
float mmc5603_temp_read(float bias[3]);

void mmc5603_mag_process(uint8_t *raw_m, float m[3]);

extern const sensor_mag_t sensor_mag_mmc5603nj;

#endif
