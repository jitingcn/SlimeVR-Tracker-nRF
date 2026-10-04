#ifndef TEST_IMU_CALIBRATION_SYSTEM_H
#define TEST_IMU_CALIBRATION_SYSTEM_H

#include <stddef.h>
#include <stdint.h>

#define MAIN_ACCEL_BIAS_ID 3
#define MAIN_GYRO_BIAS_ID 4
#define MAIN_ACC_6_BIAS_ID 7

int sys_write(uint16_t id, void *ptr, const void *data, size_t len);
void sys_warm_feedback_arm(uint32_t identity);
void sys_warm_transaction_begin(void);
void sys_warm_transaction_mark(uint16_t id, const void *data, size_t size);
void sys_warm_transaction_end(bool schedule);
int sys_flush_warm(void);

#endif
