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
#ifndef SLIMENRF_SENSOR_INTERFACE
#define SLIMENRF_SENSOR_INTERFACE

#include <stdbool.h>

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>

enum sensor_interface_dev
{
	SENSOR_INTERFACE_DEV_IMU,
#if CONFIG_SENSOR_USE_MAGNETOMETER
	SENSOR_INTERFACE_DEV_MAG,
#endif
	SENSOR_INTERFACE_DEV_COUNT
};

enum sensor_interface_spec
{
	SENSOR_INTERFACE_SPEC_SPI,
	SENSOR_INTERFACE_SPEC_I2C,
	SENSOR_INTERFACE_SPEC_EXT
};

typedef struct sensor_ext_ssi {
	/* Transfers are unreported. The caller owns any operation-level diagnostic. */
	int (*ext_write)(const uint8_t addr, const uint8_t *buf, uint32_t num_bytes);
	/* One logical register read, possibly using several physical I2CM
	 * transactions. num_read includes any device dummy prefixes. */
	int (*ext_write_read)(const uint8_t addr, const void *write_buf, size_t num_write, void *read_buf, size_t num_read);
	/* Aggregate read capacity. Longer requests advance by register-data bytes. */
	uint8_t ext_burst;
	/* Physical transaction capacity for per-transaction dummy prefixes.
	 * Zero uses ext_burst. */
	uint8_t ext_single_burst;
	/* Optional unreported policy hook. Disabling must quiesce pending
	 * background reads before success. NULL means a synchronous-only backend. */
	int (*ext_set_prefetch)(bool enabled);
	/* Optional periodic magnetometer frame read in one acquisition. Each
	 * physical transaction repeats the dummy prefix and carries whole words of
	 * register data. Returns -EAGAIN instead of waiting for a background
	 * acquisition, and -E2BIG when the frame cannot fit one acquisition. */
	int (*ext_read_frame)(const uint8_t addr, uint8_t reg, uint8_t dummy_bytes, uint8_t word_bytes, uint8_t *buf, uint8_t num_bytes);
	/* Reads repeat in the background without the driver observing them, so a
	 * read-clear status bit cannot mark new data. */
	bool ext_free_running;
} sensor_ext_ssi_t;

void sensor_interface_register_sensor_imu_spi(struct spi_dt_spec *dev);
void sensor_interface_register_sensor_imu_i2c(struct i2c_dt_spec *dev);

#if CONFIG_SENSOR_USE_MAGNETOMETER
void sensor_interface_register_sensor_mag_spi(struct spi_dt_spec *dev);
void sensor_interface_register_sensor_mag_i2c(struct i2c_dt_spec *dev);
int sensor_interface_register_sensor_mag_ext(uint8_t addr, uint8_t min_burst, uint8_t burst);
#endif

bool sensor_interface_imu_is_i2c(void);

int sensor_interface_spi_configure(enum sensor_interface_dev dev, uint32_t frequency, uint32_t dummy_reads);
#if CONFIG_SENSOR_USE_MAGNETOMETER
void sensor_interface_ext_configure(const sensor_ext_ssi_t *ext);
const sensor_ext_ssi_t *sensor_interface_ext_get(void);
/* Direct I2C/SPI is a no-op; EXT requires a configured backend. */
int sensor_interface_ext_set_prefetch(bool enabled);
/* True when magnetometer reads may repeat unobserved, consuming read-clear status. */
bool sensor_interface_mag_free_running(void);
#endif
enum sensor_interface_spec sensor_interface_get_spec(enum sensor_interface_dev dev);

int ssi_write(enum sensor_interface_dev dev, const uint8_t *buf, uint32_t num_bytes);
int ssi_read(enum sensor_interface_dev dev, uint8_t *buf, uint32_t num_bytes);
int ssi_write_read(enum sensor_interface_dev dev, const void *write_buf, size_t num_write, void *read_buf, size_t num_read);

int ssi_burst_read(enum sensor_interface_dev dev, uint8_t start_addr, uint8_t *buf, uint32_t num_bytes);
int ssi_burst_read_dummy(enum sensor_interface_dev dev, uint8_t start_addr, uint8_t dummy_bytes, uint8_t *buf, uint32_t num_bytes);
/* Periodic data frame: segments only on word boundaries. -EAGAIN is unreported
 * and means no new acquisition is available yet. */
int ssi_burst_read_frame(enum sensor_interface_dev dev, uint8_t start_addr, uint8_t dummy_bytes, uint8_t word_bytes, uint8_t *buf, uint32_t num_bytes);
int ssi_burst_write(enum sensor_interface_dev dev, uint8_t start_addr, const uint8_t *buf, uint32_t num_bytes);
int ssi_reg_read_byte(enum sensor_interface_dev dev, uint8_t reg_addr, uint8_t *value);
int ssi_reg_write_byte(enum sensor_interface_dev dev, uint8_t reg_addr, uint8_t value);
int ssi_reg_update_byte(enum sensor_interface_dev dev, uint8_t reg_addr, uint8_t mask, uint8_t value);

int ssi_reg_read_interval(enum sensor_interface_dev dev, uint8_t start_addr, uint8_t *buf, uint32_t num_bytes, uint32_t interval);
int ssi_burst_read_interval(enum sensor_interface_dev dev, uint8_t start_addr, uint8_t *buf, uint32_t num_bytes, uint32_t interval);

#endif
