#ifndef ICM45686_h
#define ICM45686_h

#include "sensor/sensor.h"

// https://www.invensense.tdk.com/en-us/download-resource/ds-000577-icm-45686-datasheet-0
// https://www.invensense.tdk.com/en-us/download-resource/000478-icm-45605-icm-45686-user-guide
// Register fields also cross-checked against TDK icm456xx inv_imu_regmap_le.h
// and inv_imu_defs.h in the NCS hal/tdk module.

// User Bank 0
#define ICM45686_ACCEL_DATA_X1_UI 0x00
#define ICM45686_GYRO_DATA_X1_UI 0x06

#define ICM45686_TEMP_DATA1_UI 0x0C

#define ICM45686_PWR_MGMT0 0x10

#define ICM45686_FIFO_COUNT_0 0x12
#define ICM45686_FIFO_DATA 0x14

#define ICM45686_INT1_CONFIG0 0x16
#define ICM45686_INT1_CONFIG1 0x17

#define ICM45686_INT1_STATUS0 0x19

#define ICM45686_ACCEL_CONFIG0 0x1B
#define ICM45686_GYRO_CONFIG0 0x1C

#define ICM45686_FIFO_CONFIG0 0x1D
#define ICM45686_FIFO_CONFIG1_0 0x1E
#define ICM45686_FIFO_CONFIG3 0x21

// FIFO_CONFIG0 fields, already shifted into register position.
#define ICM45686_FIFO_MODE_STREAM (0x01 << 6)
#define ICM45686_FIFO_DEPTH_2K 0x07
// FIFO_CONFIG3: external-sensor enables [5:4] remain clear in this driver.
#define ICM45686_FIFO_IF_EN (1 << 0)
#define ICM45686_FIFO_ACCEL_EN (1 << 1)
#define ICM45686_FIFO_GYRO_EN (1 << 2)
#define ICM45686_FIFO_HIRES_EN (1 << 3)

// Unshifted selector codes shared by the gyro and accel filter fields.
#define ICM45686_SRC_INTERPOLATOR_OFF_FIR_ON 0x01u
#define ICM45686_UI_LPF_ODR_DIV_4 0x01u

#define ICM45686_TMST_WOM_CONFIG 0x23

#define ICM45686_RTC_CONFIG 0x26

// DMP_EXT_SEN_ODR_CFG (User Bank 0, 0x27); not enabled by the host-triggered proxy.
#define ICM45686_DMP_EXT_SEN_ODR_CFG 0x27
#define ICM45686_EXT_SENSOR_EN (1 << 6)
// EXT_ODR[2:0] bits [5:3]: 000=3.125Hz, 001=6.25Hz, 010=12.5Hz, 011=25Hz,
//                           100=50Hz, 101=100Hz, 110=200Hz, 111=400Hz
#define ICM45686_EXT_ODR_3_125Hz (0x00 << 3)
#define ICM45686_EXT_ODR_6_25Hz (0x01 << 3)
#define ICM45686_EXT_ODR_12_5Hz (0x02 << 3)
#define ICM45686_EXT_ODR_25Hz (0x03 << 3)
#define ICM45686_EXT_ODR_50Hz (0x04 << 3)
#define ICM45686_EXT_ODR_100Hz (0x05 << 3)
#define ICM45686_EXT_ODR_200Hz (0x06 << 3)
#define ICM45686_EXT_ODR_400Hz (0x07 << 3)

#define ICM45686_IOC_PAD_SCENARIO_AUX_OVRD 0x30
#define ICM45686_IOC_PAD_SCENARIO_OVRD 0x31 // see application note

#define ICM45686_REG_MISC1 0x35

#define ICM45686_IREG_ADDR_15_8 0x7C
#define ICM45686_IREG_DATA 0x7E

#define ICM45686_REG_MISC2 0x7F

// User Bank IPREG_BAR
#define ICM45686_IPREG_BAR 0xA0 // MSB

#define ICM45686_IPREG_BAR_REG_58 0x3A
#define ICM45686_IPREG_BAR_REG_59 0x3B
#define ICM45686_IPREG_BAR_REG_60 0x3C

// IPREG_BAR_REG_60 bit definitions (AUX1_SCLK, AUX1_CS)
#define ICM45686_BIT_AUX1_SCLK_PULL_EN (1 << 6)
#define ICM45686_BIT_AUX1_SCLK_PULL_UP (1 << 5)
#define ICM45686_BIT_AUX1_I2CM_MODE (1 << 4) // Must be 1 for I2C Master Mode
#define ICM45686_BIT_AUX1_CS_PULL_EN (1 << 3)
#define ICM45686_BIT_AUX1_CS_PULL_UP (1 << 2)

#define ICM45686_IPREG_BAR_REG_61 0x3D

// IPREG_BAR_REG_61 bit definitions (AUX1_SDO, AUX1_SDI)
#define ICM45686_BIT_AUX1_SDO_PULL_UP (1 << 4)
#define ICM45686_BIT_AUX1_SDO_PULL_EN (1 << 3)
#define ICM45686_BIT_AUX1_SDI_PULL_EN (1 << 1)
#define ICM45686_BIT_AUX1_SDI_PULL_UP (1 << 0)

// User Bank IPREG_TOP1
#define ICM45686_IPREG_TOP1 0xA2 // MSB

// I2CM registers (IPREG_TOP1 bank)
#define ICM45686_I2CM_COMMAND_0 0x06
#define ICM45686_DEV_PROFILE_0 0x0E
#define ICM45686_DEV_PROFILE_1 0x0F
#define ICM45686_I2CM_CONTROL 0x16
#define ICM45686_I2CM_STATUS 0x18
#define ICM45686_I2CM_EXT_DEV_STATUS 0x1A
#define ICM45686_I2CM_RD_DATA_0 0x1B // 21 read buffers (0x1B~0x2F)
#define ICM45686_I2CM_WR_DATA_0 0x33 // 6 write buffers (0x33~0x38)

// I2CM_COMMAND_0 bit fields
// Bit 7: ENDFLAG (1=last I2C transaction)
// Bit 6: CH_SEL (channel select)
// Bits 5:4: R_W (0=Write, 1=Read with register addr, 2=Read without register addr)
// Bits 3:0: BURSTLEN (burst length)
#define ICM45686_I2CM_CMD_ENDFLAG (1 << 7)
#define ICM45686_I2CM_CMD_RW_WRITE (0x00 << 4)
#define ICM45686_I2CM_CMD_RW_READ_REG (0x01 << 4)
#define ICM45686_I2CM_CMD_RW_READ_NOREG (0x02 << 4)

// I2CM_CONTROL bits
#define ICM45686_I2CM_CONTROL_GO 0x01
#define ICM45686_I2CM_CONTROL_I2CM_SPEED (1 << 3) // 0=Fast Mode 400kHz, 1=Standard 100kHz
#define ICM45686_I2CM_CONTROL_RESTART_EN 0x40

// I2CM_STATUS bits
#define ICM45686_BIT_I2CM_STATUS_SDA_ERR (1 << 5)
#define ICM45686_BIT_I2CM_STATUS_SCL_ERR (1 << 4)
#define ICM45686_BIT_I2CM_STATUS_SRST_ERR (1 << 3)
#define ICM45686_BIT_I2CM_STATUS_TIMEOUT_ERR (1 << 2)
#define ICM45686_BIT_I2CM_STATUS_DONE (1 << 1)
#define ICM45686_BIT_I2CM_STATUS_BUSY (1 << 0)

#define ICM45686_SMC_CONTROL_0 0x58

// INT_I2CM_SOURCE (IPREG_TOP1, 0x74); autonomous triggers are not configured here.
#define ICM45686_INT_I2CM_SOURCE 0x74
#define ICM45686_INT_I2CM_SMC_EXT_ODR_EN (1 << 1)
#define ICM45686_INT_I2CM_IOC_EXT_TRIG_EN (1 << 0)

#define ICM45686_SREG_CTRL 0x67

#define ICM45686_ACCEL_WOM_X_THR 0x7E
#define ICM45686_ACCEL_WOM_Y_THR 0x7F
#define ICM45686_ACCEL_WOM_Z_THR 0x80

// User Bank IPREG_SYS2
#define ICM45686_IPREG_SYS2 0xA5 // MSB

#define ICM45686_IPREG_SYS2_REG_129 0x81

/*
 * IREG transport uses a 16-bit address: bank byte followed by register offset.
 * The bank helpers burst the two address bytes (and first DATA byte for writes),
 * then access IREG_DATA one byte at a time with 4 us gaps. Subsequent DATA
 * accesses use the auto-incremented address, not a burst of multiple IREGs.
 * TDK inv_imu_transport.c read_mreg/write_mreg uses this address/DATA pattern.
 */

#define GYRO_MODE_OFF 0x00
#define GYRO_MODE_STANDBY 0x01
#define GYRO_MODE_LP 0x02
#define GYRO_MODE_LN 0x03

#define ACCEL_MODE_OFF 0x01
#define ACCEL_MODE_LP 0x02
#define ACCEL_MODE_LN 0x03

#define ACCEL_UI_FS_SEL_32G 0x00
#define ACCEL_UI_FS_SEL_16G 0x01
#define ACCEL_UI_FS_SEL_8G 0x02
#define ACCEL_UI_FS_SEL_4G 0x03
#define ACCEL_UI_FS_SEL_2G 0x04

// Low Noise mode
#define ACCEL_ODR_6_4kHz 0x03
#define ACCEL_ODR_3_2kHz 0x04
#define ACCEL_ODR_1_6kHz 0x05
#define ACCEL_ODR_800Hz 0x06
// Low Noise or Low Power modes
#define ACCEL_ODR_400Hz 0x07
#define ACCEL_ODR_200Hz 0x08
#define ACCEL_ODR_100Hz 0x09
#define ACCEL_ODR_50Hz 0x0A
#define ACCEL_ODR_25Hz 0x0B
#define ACCEL_ODR_12_5Hz 0x0C
// Low Power mode
#define ACCEL_ODR_6_25Hz 0x0D
#define ACCEL_ODR_3_125Hz 0x0E
#define ACCEL_ODR_1_5625Hz 0x0F

#define GYRO_UI_FS_SEL_4000DPS 0x00
#define GYRO_UI_FS_SEL_2000DPS 0x01
#define GYRO_UI_FS_SEL_1000DPS 0x02
#define GYRO_UI_FS_SEL_500DPS 0x03
#define GYRO_UI_FS_SEL_250DPS 0x04
#define GYRO_UI_FS_SEL_125DPS 0x05
#define GYRO_UI_FS_SEL_62_5DPS 0x06
#define GYRO_UI_FS_SEL_31_25DPS 0x07
#define GYRO_UI_FS_SEL_15_625DPS 0x08

// Low Noise mode
#define GYRO_ODR_6_4kHz 0x03
#define GYRO_ODR_3_2kHz 0x04
#define GYRO_ODR_1_6kHz 0x05
#define GYRO_ODR_800Hz 0x06
// Low Noise or Low Power modes
#define GYRO_ODR_400Hz 0x07
#define GYRO_ODR_200Hz 0x08
#define GYRO_ODR_100Hz 0x09
#define GYRO_ODR_50Hz 0x0A
#define GYRO_ODR_25Hz 0x0B
#define GYRO_ODR_12_5Hz 0x0C
// Low Power mode
#define GYRO_ODR_6_25Hz 0x0D
#define GYRO_ODR_3_125Hz 0x0E
#define GYRO_ODR_1_5625Hz 0x0F

int icm45_init(
	float clock_rate_hz,
	float accel_period_s,
	float gyro_period_s,
	float *accel_actual_period_s,
	float *gyro_actual_period_s
);
void icm45_shutdown(void);

void icm45_update_fs(float accel_range, float gyro_range, float *accel_actual_range, float *gyro_actual_range);
int icm45_update_odr(
	float accel_period_s,
	float gyro_period_s,
	float *accel_actual_period_s,
	float *gyro_actual_period_s
);

uint16_t icm45_fifo_read(uint8_t *data, uint16_t len);
int icm45_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3]);
void icm45_accel_read(float a[3]);
void icm45_gyro_read(float g[3]);
float icm45_temp_read(void);

uint8_t icm45_setup_DRDY(uint16_t threshold);
uint8_t icm45_setup_WOM(void);

int icm45_ext_setup(enum sensor_ext_mode mode);

int icm45_ext_write(const uint8_t addr, const uint8_t *buf, uint32_t num_bytes);
int icm45_ext_write_read(const uint8_t addr, const void *write_buf, size_t num_write, void *read_buf, size_t num_read);

extern const sensor_imu_t sensor_imu_icm45686;
extern const sensor_ext_ssi_t sensor_ext_icm45686;

#endif
