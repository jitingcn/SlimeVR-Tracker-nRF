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
#ifndef SLIMENRF_SENSOR_SENSORS
#define SLIMENRF_SENSOR_SENSORS

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>

#include "sensor_none.h"

#if IS_ENABLED(CONFIG_SENSOR_DRV_BMI270)
#include "imu/BMI270.h"
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_ICM42688)
#include "imu/ICM42688.h"
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_ICM42686)
#include "imu/ICM42686.h"
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_ICM45686)
#include "imu/ICM45686.h"
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_LSM6DSM)
#include "imu/LSM6DSM.h"
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_LSM6DSO)
#include "imu/LSM6DSO.h"
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_LSM6DSV)
#include "imu/LSM6DSV.h"
#endif

#if IS_ENABLED(CONFIG_SENSOR_DRV_AK09940)
#include "mag/AK09940.h"
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_BMM150)
#include "mag/BMM150.h"
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_BMM350)
#include "mag/BMM350.h"
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_IST8306)
#include "mag/IST8306.h"
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_IST8308)
#include "mag/IST8308.h"
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_LIS2MDL)
#include "mag/LIS2MDL.h"
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_LIS3MDL)
#include "mag/LIS3MDL.h"
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_MMC5983MA)
#include "mag/MMC5983MA.h"
#endif
#if IS_ENABLED(CONFIG_SENSOR_DRV_QMC6309)
#include "mag/QMC6309.h"
#endif

#include "scan.h"
#include "scan_spi.h"
#include "scan_ext.h"
#include "sensor.h"
#include "sensors_enum.h"

/* Sized so ARRAY_SIZE() works for callers; keep in sync with sensors_tables.c. */
#define SENSOR_DEV_IMU_COUNT (IMU_ISM330BX + 1)
#define SENSOR_DEV_MAG_COUNT (MAG_MMC5983MA + 1)

extern const char *dev_imu_names[SENSOR_DEV_IMU_COUNT];
extern const sensor_imu_t *sensor_imus[SENSOR_DEV_IMU_COUNT];
extern const char *dev_mag_names[SENSOR_DEV_MAG_COUNT];
extern const sensor_mag_t *sensor_mags[SENSOR_DEV_MAG_COUNT];

int sensor_scan_imu(struct i2c_dt_spec *i2c_dev, uint8_t *i2c_dev_reg);
int sensor_scan_mag(struct i2c_dt_spec *i2c_dev, uint8_t *i2c_dev_reg);
int sensor_scan_imu_spi(struct spi_dt_spec *bus, uint8_t *spi_dev_reg);
int sensor_scan_mag_spi(struct spi_dt_spec *bus, uint8_t *spi_dev_reg);
int sensor_scan_mag_ext(const sensor_ext_ssi_t *ext_ssi, uint16_t *ext_dev_addr, uint8_t *ext_dev_reg);

#endif
