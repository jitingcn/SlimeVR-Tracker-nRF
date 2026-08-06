/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include <zephyr/logging/log.h>

#include "sensors.h"

// Reuse the log module registered in scan.c so single-driver fast-path logs
// share the "sensor_scan" prefix.
LOG_MODULE_DECLARE(sensor_scan, LOG_LEVEL_INF);

// Compile-time count of enabled IMU / mag driver translation units (one per
// SENSOR_DRV_* Kconfig, not per WHO_AM_I enum alias). Task 6 uses these to pick
// the single-driver fast path; the scan tables below use them to trim rows.
#define SLIME_DRV1(cfg) IS_ENABLED(cfg)

#define SENSOR_IMU_DRV_COUNT                                                                                           \
	(SLIME_DRV1(CONFIG_SENSOR_DRV_BMI270) + SLIME_DRV1(CONFIG_SENSOR_DRV_ICM42688)                                     \
	 + SLIME_DRV1(CONFIG_SENSOR_DRV_ICM42686) + SLIME_DRV1(CONFIG_SENSOR_DRV_ICM45686)                                 \
	 + SLIME_DRV1(CONFIG_SENSOR_DRV_LSM6DSM) + SLIME_DRV1(CONFIG_SENSOR_DRV_LSM6DSO)                                   \
	 + SLIME_DRV1(CONFIG_SENSOR_DRV_LSM6DSV))

#define SENSOR_MAG_DRV_COUNT                                                                                           \
	(SLIME_DRV1(CONFIG_SENSOR_DRV_AK09940) + SLIME_DRV1(CONFIG_SENSOR_DRV_BMM150)                                      \
	 + SLIME_DRV1(CONFIG_SENSOR_DRV_BMM350) + SLIME_DRV1(CONFIG_SENSOR_DRV_IST8306)                                    \
	 + SLIME_DRV1(CONFIG_SENSOR_DRV_IST8308) + SLIME_DRV1(CONFIG_SENSOR_DRV_LIS2MDL)                                   \
	 + SLIME_DRV1(CONFIG_SENSOR_DRV_LIS3MDL) + SLIME_DRV1(CONFIG_SENSOR_DRV_MMC5603NJ)                                 \
	 + SLIME_DRV1(CONFIG_SENSOR_DRV_MMC5983MA)                                                                         \
	 + SLIME_DRV1(CONFIG_SENSOR_DRV_QMC5883L) + SLIME_DRV1(CONFIG_SENSOR_DRV_QMC6309))

// Unimplemented WHO_AM_I rows (chips that map to sensor_*_none) are kept in full
// builds for probe-behavior parity, and dropped when that axis opts into MINIMAL.
#define SLIME_IMU_KEEP_UNIMPL (!IS_ENABLED(CONFIG_SENSOR_IMU_DRIVERS_MINIMAL))
#define SLIME_MAG_KEEP_UNIMPL (!IS_ENABLED(CONFIG_SENSOR_MAG_DRIVERS_MINIMAL))

// A packed address group is emitted only if at least one of its WHO_AM_I rows
// survives gating. *_addr_count must equal the number of emitted groups.
#define SLIME_IMU_GROUP_A                                                                                              \
	(IS_ENABLED(CONFIG_SENSOR_DRV_BMI270) || IS_ENABLED(CONFIG_SENSOR_DRV_ICM45686)                                    \
	 || IS_ENABLED(CONFIG_SENSOR_DRV_ICM42686) || IS_ENABLED(CONFIG_SENSOR_DRV_ICM42688) || SLIME_IMU_KEEP_UNIMPL)
#define SLIME_IMU_GROUP_B                                                                                              \
	(IS_ENABLED(CONFIG_SENSOR_DRV_LSM6DSM) || IS_ENABLED(CONFIG_SENSOR_DRV_LSM6DSO)                                    \
	 || IS_ENABLED(CONFIG_SENSOR_DRV_LSM6DSV) || SLIME_IMU_KEEP_UNIMPL)

#define SLIME_MAG_G0                                                                                                   \
	(SLIME_MAG_KEEP_UNIMPL || IS_ENABLED(CONFIG_SENSOR_DRV_AK09940) || IS_ENABLED(CONFIG_SENSOR_DRV_IST8308))
#define SLIME_MAG_G1                                                                                                   \
	(SLIME_MAG_KEEP_UNIMPL || IS_ENABLED(CONFIG_SENSOR_DRV_AK09940) || IS_ENABLED(CONFIG_SENSOR_DRV_IST8308)          \
	 || IS_ENABLED(CONFIG_SENSOR_DRV_QMC5883L))
#define SLIME_MAG_G2                                                                                                   \
	(SLIME_MAG_KEEP_UNIMPL || IS_ENABLED(CONFIG_SENSOR_DRV_AK09940) || IS_ENABLED(CONFIG_SENSOR_DRV_IST8308))
#define SLIME_MAG_G3 (IS_ENABLED(CONFIG_SENSOR_DRV_BMM150))
#define SLIME_MAG_G4 (IS_ENABLED(CONFIG_SENSOR_DRV_BMM350))
#define SLIME_MAG_G5 (SLIME_MAG_KEEP_UNIMPL || IS_ENABLED(CONFIG_SENSOR_DRV_IST8306))
#define SLIME_MAG_G6 (SLIME_MAG_KEEP_UNIMPL || IS_ENABLED(CONFIG_SENSOR_DRV_LIS3MDL))
#define SLIME_MAG_G7                                                                                                   \
	(SLIME_MAG_KEEP_UNIMPL || IS_ENABLED(CONFIG_SENSOR_DRV_LIS3MDL) || IS_ENABLED(CONFIG_SENSOR_DRV_LIS2MDL))
#define SLIME_MAG_G8                                                                                                   \
	(SLIME_MAG_KEEP_UNIMPL || IS_ENABLED(CONFIG_SENSOR_DRV_MMC5603NJ) || IS_ENABLED(CONFIG_SENSOR_DRV_MMC5983MA))
#define SLIME_MAG_G9 (SLIME_MAG_KEEP_UNIMPL)
#define SLIME_MAG_G10 (IS_ENABLED(CONFIG_SENSOR_DRV_QMC6309))

const char *dev_imu_names[SENSOR_DEV_IMU_COUNT]
	= {"BMI160",
	   "BMI270",
	   "BMI323",
	   "MPU-6000/MPU-6050",
	   "MPU-6500",
	   "MPU-9250",
	   "ICM-20948",
	   "ICM-42688-P/ICM-42688-V",
	   "ICM-42686-P",
	   "ICM-45686",
	   "ICM-45688-P",
	   "LSM6DSO16IS/ISM330IS",
	   "LSM6DS3",
	   "LSM6DS3TR-C/LSM6DSL/LSM6DSM/ISM330DLC",
	   "LSM6DSR/ISM330DHCX",
	   "LSM6DSO",
	   "LSM6DST",
	   "LSM6DSV",
	   "LSM6DSV16B/ISM330BX"};
const sensor_imu_t *sensor_imus[SENSOR_DEV_IMU_COUNT] = {
	&sensor_imu_none, // will not implement, too low quality
#if IS_ENABLED(CONFIG_SENSOR_DRV_BMI270)
	&sensor_imu_bmi270,
#else
	&sensor_imu_none,
#endif
	&sensor_imu_none,
	&sensor_imu_none, // cardinal sin
	&sensor_imu_none, // cardinal sin
	&sensor_imu_none, // cardinal sin
	&sensor_imu_none,
#if IS_ENABLED(CONFIG_SENSOR_DRV_ICM42688)
	&sensor_imu_icm42688,
#else
	&sensor_imu_none,
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_ICM42686)
	&sensor_imu_icm42686,
#else
	&sensor_imu_none,
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_ICM45686)
	&sensor_imu_icm45686,
#else
	&sensor_imu_none,
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_ICM45686)
	&sensor_imu_icm45686, // compatible with driver
#else
	&sensor_imu_none,
#endif
	&sensor_imu_none, // will not implement, does not have FIFO
#if IS_ENABLED(CONFIG_SENSOR_DRV_LSM6DSM)
	&sensor_imu_lsm6dsm, // compatible with driver (unfortunately)
#else
	&sensor_imu_none,
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_LSM6DSM)
	&sensor_imu_lsm6dsm,
#else
	&sensor_imu_none,
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_LSM6DSO)
	&sensor_imu_lsm6dso, // compatible with driver
#else
	&sensor_imu_none,
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_LSM6DSO)
	&sensor_imu_lsm6dso,
#else
	&sensor_imu_none,
#endif
	&sensor_imu_none,
#if IS_ENABLED(CONFIG_SENSOR_DRV_LSM6DSV)
	&sensor_imu_lsm6dsv,
#else
	&sensor_imu_none,
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_LSM6DSV)
	&sensor_imu_lsm6dsv // compatible with driver
#else
	&sensor_imu_none
#endif
};
// Scan tables use the packed format read by sensor_scan_i2c/_spi/_ext:
//   addr[] : per group { count, addr0, addr1, ... }
//   reg[]  : per group { reg_count, reg0, reg1, ... }
//   id[]   : per group, per register { id_count, id0, id1, ... }
//   dev[]  : one enum per id value, in the same order as id[]
// Rows are gated per driver; a whole group is dropped when none of its rows
// survive. Registers within a kept group are retained even if all their IDs are
// gated out (they emit an empty { 0 } id sublist) to keep reg_count in sync.
static const int i2c_dev_imu_addr_count = SLIME_IMU_GROUP_A + SLIME_IMU_GROUP_B;
static const uint8_t i2c_dev_imu_addr[] = {
#if SLIME_IMU_GROUP_A
	2,
	0x68,
	0x69,
#endif
#if SLIME_IMU_GROUP_B
	2,
	0x6A,
	0x6B,
#endif
};
static const uint8_t i2c_dev_imu_reg[] = {
#if SLIME_IMU_GROUP_A
	3,
	0x00,
	0x72,
	0x75,
#endif
#if SLIME_IMU_GROUP_B
	1,
	0x0F,
#endif
};
static const uint8_t i2c_dev_imu_id[] = {
#if SLIME_IMU_GROUP_A
	// reg 0x00
	(SLIME_IMU_KEEP_UNIMPL * 3 + IS_ENABLED(CONFIG_SENSOR_DRV_BMI270)),
#if SLIME_IMU_KEEP_UNIMPL
	0xEA,
	0xD1, // ICM20948, BMI160
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_BMI270)
	0x24, // BMI270
#endif
#if SLIME_IMU_KEEP_UNIMPL
	0x43, // BMI323
#endif
	// reg 0x72
	(IS_ENABLED(CONFIG_SENSOR_DRV_ICM45686) * 2),
#if IS_ENABLED(CONFIG_SENSOR_DRV_ICM45686)
	0xE9,
	0xE7, // ICM45686, ICM45688
#endif
	// reg 0x75
	(SLIME_IMU_KEEP_UNIMPL * 3 + IS_ENABLED(CONFIG_SENSOR_DRV_ICM42686) + IS_ENABLED(CONFIG_SENSOR_DRV_ICM42688) * 2),
#if SLIME_IMU_KEEP_UNIMPL
	0x68,
	0x70,
	0x71, // MPU6050, MPU6500, MPU9250
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_ICM42686)
	0x44, // ICM42686
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_ICM42688)
	0x47,
	0xDB, // ICM42688-P, ICM42688-V
#endif
#endif // SLIME_IMU_GROUP_A
#if SLIME_IMU_GROUP_B
	// reg 0x0F
	(SLIME_IMU_KEEP_UNIMPL * 2 + IS_ENABLED(CONFIG_SENSOR_DRV_LSM6DSM) * 2 + IS_ENABLED(CONFIG_SENSOR_DRV_LSM6DSO) * 2
	 + IS_ENABLED(CONFIG_SENSOR_DRV_LSM6DSV) * 2),
#if SLIME_IMU_KEEP_UNIMPL
	0x22, // ISM330IS
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_LSM6DSM)
	0x69,
	0x6A, // LSM6DS3, LSM6DSM
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_LSM6DSO)
	0x6B,
	0x6C, // LSM6DSR, LSM6DSO
#endif
#if SLIME_IMU_KEEP_UNIMPL
	0x6D, // LSM6DST
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_LSM6DSV)
	0x70,
	0x71, // LSM6DSV, ISM330BX
#endif
#endif // SLIME_IMU_GROUP_B
};
static const int i2c_dev_imu[] = {
#if SLIME_IMU_GROUP_A
// reg 0x00
#if SLIME_IMU_KEEP_UNIMPL
	IMU_ICM20948, IMU_BMI160,
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_BMI270)
	IMU_BMI270,
#endif
#if SLIME_IMU_KEEP_UNIMPL
	IMU_BMI323,
#endif
// reg 0x72
#if IS_ENABLED(CONFIG_SENSOR_DRV_ICM45686)
	IMU_ICM45686, IMU_ICM45688,
#endif
// reg 0x75
#if SLIME_IMU_KEEP_UNIMPL
	IMU_MPU6050,  IMU_MPU6500,  IMU_MPU9250,
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_ICM42686)
	IMU_ICM42686,
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_ICM42688)
	IMU_ICM42688, IMU_ICM42688, // ICM-42688-P, ICM-42688-V
#endif
#endif // SLIME_IMU_GROUP_A
#if SLIME_IMU_GROUP_B
// reg 0x0F
#if SLIME_IMU_KEEP_UNIMPL
	IMU_ISM330IS,
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_LSM6DSM)
	IMU_LSM6DS3,  IMU_LSM6DSM,
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_LSM6DSO)
	IMU_LSM6DSR,  IMU_LSM6DSO,
#endif
#if SLIME_IMU_KEEP_UNIMPL
	IMU_LSM6DST,
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_LSM6DSV)
	IMU_LSM6DSV,  IMU_ISM330BX,
#endif
#endif // SLIME_IMU_GROUP_B
};

const char *dev_mag_names[SENSOR_DEV_MAG_COUNT]
	= {"HMC5883L",  "QMC5883L",        "QMC6309", "QMC6310",    "AK8963",    "AK09916",
	   "AK09940",   "BMM150",          "BMM350",  "IST8306",    "IST8308",   "IST8320",
	   "IST8321",   "IIS2MDC/LIS2MDL", "LIS3MDL", "MMC34160PJ", "MMC3630KJ", "MMC5603NJ/MMC5633NJL",
	   "MMC5616WA", "MMC5983MA"};
const sensor_mag_t *sensor_mags[SENSOR_DEV_MAG_COUNT] = {
	&sensor_mag_none, // HMC5883 will not implement, too low quality
#if IS_ENABLED(CONFIG_SENSOR_DRV_QMC5883L)
	&sensor_mag_qmc5883l,
#else
	&sensor_mag_none, // QMC5883 not implemented
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_QMC6309)
	&sensor_mag_qmc6309,
#else
	&sensor_mag_none,
#endif
	&sensor_mag_none, // QMC6310
	&sensor_mag_none, // AK8963
	&sensor_mag_none, // AK09916
#if IS_ENABLED(CONFIG_SENSOR_DRV_AK09940)
	&sensor_mag_ak09940,
#else
	&sensor_mag_none,
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_BMM150)
	&sensor_mag_bmm150,
#else
	&sensor_mag_none,
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_BMM350)
	&sensor_mag_bmm350,
#else
	&sensor_mag_none,
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_IST8306)
	&sensor_mag_ist8306,
#else
	&sensor_mag_none,
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_IST8308)
	&sensor_mag_ist8308,
#else
	&sensor_mag_none,
#endif
	&sensor_mag_none, // IST8320
	&sensor_mag_none, // IST8321
#if IS_ENABLED(CONFIG_SENSOR_DRV_LIS2MDL)
	&sensor_mag_lis2mdl,
#else
	&sensor_mag_none,
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_LIS3MDL)
	&sensor_mag_lis3mdl,
#else
	&sensor_mag_none,
#endif
	&sensor_mag_none, // MMC34160
	&sensor_mag_none, // MMC3630
#if IS_ENABLED(CONFIG_SENSOR_DRV_MMC5603NJ)
	&sensor_mag_mmc5603nj, // MMC5603NJ/MMC5633NJL
#else
	&sensor_mag_none, // MMC5603/MMC5633
#endif
	&sensor_mag_none, // MMC5616
#if IS_ENABLED(CONFIG_SENSOR_DRV_MMC5983MA)
	&sensor_mag_mmc5983ma
#else
	&sensor_mag_none
#endif
};
// When no mag driver is enabled every SLIME_MAG_Gx gates to 0, which would
// leave these initializers empty and trip -Werror. Guard the whole table and
// stub sensor_scan_mag* to return -1 instead (see below).
#if SENSOR_MAG_DRV_COUNT > 0
static const int i2c_dev_mag_addr_count = SLIME_MAG_G0 + SLIME_MAG_G1 + SLIME_MAG_G2 + SLIME_MAG_G3 + SLIME_MAG_G4
										+ SLIME_MAG_G5 + SLIME_MAG_G6 + SLIME_MAG_G7 + SLIME_MAG_G8 + SLIME_MAG_G9
										+ SLIME_MAG_G10;
static const uint8_t i2c_dev_mag_addr[] = {
#if SLIME_MAG_G0
	1, 0x0C,
#endif
#if SLIME_MAG_G1
	1, 0x0D,
#endif
#if SLIME_MAG_G2
	2, 0x0E, 0x0F,
#endif
#if SLIME_MAG_G3
	4, 0x10, 0x11, 0x12, 0x13, // why bosch
#endif
#if SLIME_MAG_G4
	4, 0x14, 0x15, 0x16, 0x17,
#endif
#if SLIME_MAG_G5
	1, 0x19,
#endif
#if SLIME_MAG_G6
	1, 0x1C,
#endif
#if SLIME_MAG_G7
	1, 0x1E,
#endif
#if SLIME_MAG_G8
	1, 0x30,
#endif
#if SLIME_MAG_G9
	1, 0x3C,
#endif
#if SLIME_MAG_G10
	1, 0x7C,
#endif
};
static const uint8_t i2c_dev_mag_reg[] = {
#if SLIME_MAG_G0
	2,    0x01, // AK09916/AK09940 first
	0x00,
#endif
#if SLIME_MAG_G1
	3,    0x01, // AK09940 first
	0x00, 0x0D,
#endif
#if SLIME_MAG_G2
	2,    0x01, // AK09940 first
	0x00,
#endif
#if SLIME_MAG_G3
	1,    0x40,
#endif
#if SLIME_MAG_G4
	1,    0x00,
#endif
#if SLIME_MAG_G5
	1,    0x00,
#endif
#if SLIME_MAG_G6
	2,    0x00, 0x0F,
#endif
#if SLIME_MAG_G7
	3,    0x0A, 0x0F, 0x4F,
#endif
#if SLIME_MAG_G8
	3,    0x20, 0x39, 0x2F,
#endif
#if SLIME_MAG_G9
	1,    0x00,
#endif
#if SLIME_MAG_G10
	1,    0x00,
#endif
};
static const uint8_t i2c_dev_mag_id[] = {
#if SLIME_MAG_G0
	// 0x0C reg 0x01
	(SLIME_MAG_KEEP_UNIMPL + IS_ENABLED(CONFIG_SENSOR_DRV_AK09940)),
#if SLIME_MAG_KEEP_UNIMPL
	0x09, // AK09916
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_AK09940)
	0xA3, // AK09940
#endif
	// 0x0C reg 0x00
	(IS_ENABLED(CONFIG_SENSOR_DRV_IST8308) + SLIME_MAG_KEEP_UNIMPL),
#if IS_ENABLED(CONFIG_SENSOR_DRV_IST8308)
	0x08, // IST8308
#endif
#if SLIME_MAG_KEEP_UNIMPL
	0x48, // AK8963
#endif
#endif // SLIME_MAG_G0
#if SLIME_MAG_G1
	// 0x0D reg 0x01
	(IS_ENABLED(CONFIG_SENSOR_DRV_AK09940)),
#if IS_ENABLED(CONFIG_SENSOR_DRV_AK09940)
	0xA3, // AK09940
#endif
	// 0x0D reg 0x00
	(IS_ENABLED(CONFIG_SENSOR_DRV_IST8308) + SLIME_MAG_KEEP_UNIMPL),
#if IS_ENABLED(CONFIG_SENSOR_DRV_IST8308)
	0x08, // IST8308
#endif
#if SLIME_MAG_KEEP_UNIMPL
	0x48, // AK8963
#endif
	// 0x0D reg 0x0D
	(SLIME_MAG_KEEP_UNIMPL || IS_ENABLED(CONFIG_SENSOR_DRV_QMC5883L)),
#if SLIME_MAG_KEEP_UNIMPL || IS_ENABLED(CONFIG_SENSOR_DRV_QMC5883L)
	0xFF, // QMC5883L
#endif
#endif // SLIME_MAG_G1
#if SLIME_MAG_G2
	// 0x0E/0x0F reg 0x01
	(IS_ENABLED(CONFIG_SENSOR_DRV_AK09940)),
#if IS_ENABLED(CONFIG_SENSOR_DRV_AK09940)
	0xA3, // AK09940
#endif
	// 0x0E/0x0F reg 0x00
	(IS_ENABLED(CONFIG_SENSOR_DRV_IST8308) + SLIME_MAG_KEEP_UNIMPL),
#if IS_ENABLED(CONFIG_SENSOR_DRV_IST8308)
	0x08, // IST8308
#endif
#if SLIME_MAG_KEEP_UNIMPL
	0x48, // AK8963
#endif
#endif // SLIME_MAG_G2
#if SLIME_MAG_G3
	// 0x10-0x13 reg 0x40
	(IS_ENABLED(CONFIG_SENSOR_DRV_BMM150)),
#if IS_ENABLED(CONFIG_SENSOR_DRV_BMM150)
	0x32, // BMM150
#endif
#endif // SLIME_MAG_G3
#if SLIME_MAG_G4
	// 0x14-0x17 reg 0x00
	(IS_ENABLED(CONFIG_SENSOR_DRV_BMM350)),
#if IS_ENABLED(CONFIG_SENSOR_DRV_BMM350)
	0x33, // BMM350
#endif
#endif // SLIME_MAG_G4
#if SLIME_MAG_G5
	// 0x19 reg 0x00
	(IS_ENABLED(CONFIG_SENSOR_DRV_IST8306) + SLIME_MAG_KEEP_UNIMPL * 2),
#if IS_ENABLED(CONFIG_SENSOR_DRV_IST8306)
	0x06, // IST8306
#endif
#if SLIME_MAG_KEEP_UNIMPL
	0x20,
	0x21, // IST8320, IST8321
#endif
#endif // SLIME_MAG_G5
#if SLIME_MAG_G6
	// 0x1C reg 0x00
	(SLIME_MAG_KEEP_UNIMPL),
#if SLIME_MAG_KEEP_UNIMPL
	0x80, // QMC6310
#endif
	// 0x1C reg 0x0F
	(IS_ENABLED(CONFIG_SENSOR_DRV_LIS3MDL)),
#if IS_ENABLED(CONFIG_SENSOR_DRV_LIS3MDL)
	0x3D, // LIS3MDL
#endif
#endif // SLIME_MAG_G6
#if SLIME_MAG_G7
	// 0x1E reg 0x0A
	(SLIME_MAG_KEEP_UNIMPL),
#if SLIME_MAG_KEEP_UNIMPL
	0x48, // HMC5883L
#endif
	// 0x1E reg 0x0F
	(IS_ENABLED(CONFIG_SENSOR_DRV_LIS3MDL)),
#if IS_ENABLED(CONFIG_SENSOR_DRV_LIS3MDL)
	0x3D, // LIS3MDL
#endif
	// 0x1E reg 0x4F
	(IS_ENABLED(CONFIG_SENSOR_DRV_LIS2MDL)),
#if IS_ENABLED(CONFIG_SENSOR_DRV_LIS2MDL)
	0x40, // LIS2MDL
#endif
#endif // SLIME_MAG_G7
#if SLIME_MAG_G8
	// 0x30 reg 0x20
	(SLIME_MAG_KEEP_UNIMPL),
#if SLIME_MAG_KEEP_UNIMPL
	0x06, // MMC34160PJ
#endif
	// 0x30 reg 0x39 (checked before 0x2F: MMC5603NJ reads 0x0A at undocumented 0x2F, colliding with MMC3630KJ)
	(SLIME_MAG_KEEP_UNIMPL * 2 + (1 - SLIME_MAG_KEEP_UNIMPL) * IS_ENABLED(CONFIG_SENSOR_DRV_MMC5603NJ)),
#if SLIME_MAG_KEEP_UNIMPL || IS_ENABLED(CONFIG_SENSOR_DRV_MMC5603NJ)
	0x10, // MMC5603NJ/MMC5633NJL
#endif
#if SLIME_MAG_KEEP_UNIMPL
	0x11, // MMC5616WA
#endif
	// 0x30 reg 0x2F
	(SLIME_MAG_KEEP_UNIMPL + IS_ENABLED(CONFIG_SENSOR_DRV_MMC5983MA)),
#if SLIME_MAG_KEEP_UNIMPL
	0x0A, // MMC3630KJ
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_MMC5983MA)
	0x30, // MMC5983MA
#endif
#endif // SLIME_MAG_G8
#if SLIME_MAG_G9
	// 0x3C reg 0x00
	(SLIME_MAG_KEEP_UNIMPL),
#if SLIME_MAG_KEEP_UNIMPL
	0x80, // QMC6310
#endif
#endif // SLIME_MAG_G9
#if SLIME_MAG_G10
	// 0x7C reg 0x00
	(IS_ENABLED(CONFIG_SENSOR_DRV_QMC6309)),
#if IS_ENABLED(CONFIG_SENSOR_DRV_QMC6309)
	0x90, // QMC6309
#endif
#endif // SLIME_MAG_G10
};
static const int i2c_dev_mag[] = {
#if SLIME_MAG_G0
#if SLIME_MAG_KEEP_UNIMPL
	MAG_AK09916,
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_AK09940)
	MAG_AK09940,
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_IST8308)
	MAG_IST8308,
#endif
#if SLIME_MAG_KEEP_UNIMPL
	MAG_AK8963,
#endif
#endif // SLIME_MAG_G0
#if SLIME_MAG_G1
#if IS_ENABLED(CONFIG_SENSOR_DRV_AK09940)
	MAG_AK09940,
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_IST8308)
	MAG_IST8308,
#endif
#if SLIME_MAG_KEEP_UNIMPL
	MAG_AK8963,
#endif
#if SLIME_MAG_KEEP_UNIMPL || IS_ENABLED(CONFIG_SENSOR_DRV_QMC5883L)
	MAG_QMC5883L,
#endif
#endif // SLIME_MAG_G1
#if SLIME_MAG_G2
#if IS_ENABLED(CONFIG_SENSOR_DRV_AK09940)
	MAG_AK09940,
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_IST8308)
	MAG_IST8308,
#endif
#if SLIME_MAG_KEEP_UNIMPL
	MAG_AK8963,
#endif
#endif // SLIME_MAG_G2
#if SLIME_MAG_G3
#if IS_ENABLED(CONFIG_SENSOR_DRV_BMM150)
	MAG_BMM150,
#endif
#endif // SLIME_MAG_G3
#if SLIME_MAG_G4
#if IS_ENABLED(CONFIG_SENSOR_DRV_BMM350)
	MAG_BMM350,
#endif
#endif // SLIME_MAG_G4
#if SLIME_MAG_G5
#if IS_ENABLED(CONFIG_SENSOR_DRV_IST8306)
	MAG_IST8306,
#endif
#if SLIME_MAG_KEEP_UNIMPL
	MAG_IST8320,    MAG_IST8321,
#endif
#endif // SLIME_MAG_G5
#if SLIME_MAG_G6
#if SLIME_MAG_KEEP_UNIMPL
	MAG_QMC6310,
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_LIS3MDL)
	MAG_LIS3MDL,
#endif
#endif // SLIME_MAG_G6
#if SLIME_MAG_G7
#if SLIME_MAG_KEEP_UNIMPL
	MAG_HMC5883L,
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_LIS3MDL)
	MAG_LIS3MDL,
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_LIS2MDL)
	MAG_LIS2MDL,
#endif
#endif // SLIME_MAG_G7
#if SLIME_MAG_G8
#if SLIME_MAG_KEEP_UNIMPL
	MAG_MMC34160PJ,
#endif
#if SLIME_MAG_KEEP_UNIMPL || IS_ENABLED(CONFIG_SENSOR_DRV_MMC5603NJ)
	MAG_MMC5633NJL,
#endif
#if SLIME_MAG_KEEP_UNIMPL
	MAG_MMC5616WA, MAG_MMC3630KJ,
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_MMC5983MA)
	MAG_MMC5983MA,
#endif
#endif // SLIME_MAG_G8
#if SLIME_MAG_G9
#if SLIME_MAG_KEEP_UNIMPL
	MAG_QMC6310,
#endif
#endif // SLIME_MAG_G9
#if SLIME_MAG_G10
#if IS_ENABLED(CONFIG_SENSOR_DRV_QMC6309)
	MAG_QMC6309,
#endif
#endif // SLIME_MAG_G10
};
#endif // SENSOR_MAG_DRV_COUNT > 0

int sensor_scan_imu(struct i2c_dt_spec *i2c_dev, uint8_t *i2c_dev_reg)
{
#if SENSOR_IMU_DRV_COUNT == 1
	// Only one IMU driver is compiled in, so the trimmed table holds a single
	// driver's WHO_AM_I rows; the shared walker below already skips every other
	// chip. This log just makes the fast path visible.
	LOG_INF("IMU single-driver scan (%d groups)", i2c_dev_imu_addr_count);
#endif
	return sensor_scan_i2c(
		i2c_dev,
		i2c_dev_reg,
		i2c_dev_imu_addr_count,
		i2c_dev_imu_addr,
		i2c_dev_imu_reg,
		i2c_dev_imu_id,
		i2c_dev_imu
	);
}

int sensor_scan_mag(struct i2c_dt_spec *i2c_dev, uint8_t *i2c_dev_reg)
{
#if SENSOR_MAG_DRV_COUNT == 0
	ARG_UNUSED(i2c_dev);
	ARG_UNUSED(i2c_dev_reg);
	return -1;
#else
#if SENSOR_MAG_DRV_COUNT == 1
	LOG_INF("Magnetometer single-driver scan (%d groups)", i2c_dev_mag_addr_count);
#endif
	return sensor_scan_i2c(
		i2c_dev,
		i2c_dev_reg,
		i2c_dev_mag_addr_count,
		i2c_dev_mag_addr,
		i2c_dev_mag_reg,
		i2c_dev_mag_id,
		i2c_dev_mag
	);
#endif
}

int sensor_scan_imu_spi(struct spi_dt_spec *bus, uint8_t *spi_dev_reg)
{
#if SENSOR_IMU_DRV_COUNT == 1
	LOG_INF("IMU single-driver SPI scan (%d groups)", i2c_dev_imu_addr_count);
#endif
	return sensor_scan_spi(bus, spi_dev_reg, i2c_dev_imu_addr_count, i2c_dev_imu_reg, i2c_dev_imu_id, i2c_dev_imu);
}

int sensor_scan_mag_spi(struct spi_dt_spec *bus, uint8_t *spi_dev_reg)
{
#if SENSOR_MAG_DRV_COUNT == 0
	ARG_UNUSED(bus);
	ARG_UNUSED(spi_dev_reg);
	return -1;
#else
#if SENSOR_MAG_DRV_COUNT == 1
	LOG_INF("Magnetometer single-driver SPI scan (%d groups)", i2c_dev_mag_addr_count);
#endif
	return sensor_scan_spi(bus, spi_dev_reg, i2c_dev_mag_addr_count, i2c_dev_mag_reg, i2c_dev_mag_id, i2c_dev_mag);
#endif
}

int sensor_scan_mag_ext(const sensor_ext_ssi_t *ext_ssi, uint16_t *ext_dev_addr, uint8_t *ext_dev_reg)
{
#if SENSOR_MAG_DRV_COUNT == 0
	ARG_UNUSED(ext_ssi);
	ARG_UNUSED(ext_dev_addr);
	ARG_UNUSED(ext_dev_reg);
	return -1;
#else
#if SENSOR_MAG_DRV_COUNT == 1
	LOG_INF("Magnetometer single-driver external scan (%d groups)", i2c_dev_mag_addr_count);
#endif
	return sensor_scan_ext(
		ext_ssi,
		ext_dev_addr,
		ext_dev_reg,
		i2c_dev_mag_addr_count,
		i2c_dev_mag_addr,
		i2c_dev_mag_reg,
		i2c_dev_mag_id,
		i2c_dev_mag
	);
#endif
}
