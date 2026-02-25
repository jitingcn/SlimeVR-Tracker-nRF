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
#include "globals.h"
#include "system/system.h"
#include "system/watchdog.h"
#include "util.h"
#include "connection/connection.h"
#include "calibration.h"

#include <math.h>
#include <hal/nrf_gpio.h>

#if CONFIG_CMSIS_DSP
#include <arm_math.h>
#endif

#include "fusion/fusions.h"
#include "sensors.h"

#include "sensor.h"

#define SPI_OP SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8)

// Debug mode state
typedef struct {
	bool enabled;
	int64_t start_time;
	int64_t duration_ms;
	uint32_t accel_count;         // Count accel samples since last output
	uint32_t output_every_n;      // Output every N accel samples
	uint32_t output_count;        // Total output count
} sensor_debug_state_t;

static sensor_debug_state_t debug_state = {
	.enabled = false,
	.output_every_n = 2  // Default: output every 2 accel samples
};

#if CONFIG_SENSOR_RANGE_STATS
// Sensor range tracking state - records min/max values during runtime (not persisted)
static sensor_range_stats_t range_stats = {
	.gyro_max = {-INFINITY, -INFINITY, -INFINITY},
	.gyro_min = {INFINITY, INFINITY, INFINITY},
	.accel_max = {-INFINITY, -INFINITY, -INFINITY},
	.accel_min = {INFINITY, INFINITY, INFINITY},
	.sample_count = 0,
	.initialized = false
};
#endif // CONFIG_SENSOR_RANGE_STATS

#if DT_NODE_HAS_STATUS(DT_NODELABEL(imu_spi), okay)
#define SENSOR_IMU_SPI_EXISTS true
#define SENSOR_IMU_SPI_NODE DT_NODELABEL(imu_spi)
static struct spi_dt_spec sensor_imu_spi_dev = SPI_DT_SPEC_GET(SENSOR_IMU_SPI_NODE, SPI_OP, 0);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(imu), okay)
#define SENSOR_IMU_EXISTS true
#define SENSOR_IMU_NODE DT_NODELABEL(imu)
static struct i2c_dt_spec sensor_imu_dev = I2C_DT_SPEC_GET(SENSOR_IMU_NODE);
#else
static struct i2c_dt_spec sensor_imu_dev = {0};
#endif
#if !SENSOR_IMU_SPI_EXISTS && !SENSOR_IMU_EXISTS
#error "IMU node does not exist"
#endif
static uint8_t sensor_imu_dev_reg = 0xFF;

#if DT_NODE_HAS_STATUS(DT_NODELABEL(mag_spi), okay)
#define SENSOR_MAG_SPI_EXISTS true
#define SENSOR_MAG_SPI_NODE DT_NODELABEL(mag_spi)
static struct spi_dt_spec sensor_mag_spi_dev = SPI_DT_SPEC_GET(SENSOR_MAG_SPI_NODE, SPI_OP, 0);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(mag), okay)
#define SENSOR_MAG_EXISTS true
#define SENSOR_MAG_NODE DT_NODELABEL(mag)
static struct i2c_dt_spec sensor_mag_dev = I2C_DT_SPEC_GET(SENSOR_MAG_NODE);
#else
static struct i2c_dt_spec sensor_mag_dev = {0};
#endif
#if SENSOR_IMU_SPI_EXISTS // might exist
#define SENSOR_MAG_EXT_EXISTS true
#endif
#if !SENSOR_MAG_SPI_EXISTS && !SENSOR_MAG_EXISTS && !SENSOR_MAG_EXT_EXISTS
#warning "Magnetometer node does not exist"
#endif
static uint8_t sensor_mag_dev_reg = 0xFF;

static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // vector to hold quaternion
static float last_q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // vector to hold quaternion

static float q3[4] = {SENSOR_QUATERNION_CORRECTION}; // correction quaternion

static float last_lin_a[3] = {0}; // vector to hold last linear accelerometer

static float temp; // sensor temperature
static int64_t last_temp_time = -1000;

static int64_t last_suspend_attempt_time = 0;
static int64_t last_data_time;
static int64_t last_sensor_send_time = 0;
static int64_t last_retained_save_time = 0;

// Periodic retained save interval (ms) for crash recovery
#define RETAINED_SAVE_INTERVAL_MS 5000

static float max_gyro_speed_square;
static bool mag_use_oneshot;
static bool mag_skip_oneshot;

static float accel_actual_time;
static float gyro_actual_time;
static float mag_actual_time;

#if CONFIG_SENSOR_GYRO_OVERSAMPLING > 1
// Gyroscope oversampling state for noise reduction
// Accumulates gyro samples and averages them before fusion
static float gyro_oversample_sum[3] = {0};
static int gyro_oversample_count = 0;
static float gyro_effective_time; // Effective time step for fusion after oversampling
#endif

#if CONFIG_SENSOR_ACCEL_OVERSAMPLING > 1
// Accelerometer oversampling state for noise reduction
// Accumulates accel samples and averages them before fusion
static float accel_oversample_sum[3] = {0};
static int accel_oversample_count = 0;
static float accel_effective_time; // Effective time step for fusion after oversampling
#endif

static float accel_actual_range;  // Actual accelerometer full scale range (g)
static float gyro_actual_range;   // Actual gyroscope full scale range (deg/s)

static float sensor_actual_time;
static int16_t sensor_fifo_threshold;
static int64_t sensor_data_time; // ticks

static bool sensor_fusion_init;
static bool sensor_sensor_init;

static bool sensor_sensor_scanning;

static bool main_suspended;

static bool mag_available;
#if MAG_ENABLED
static bool mag_enabled = true; // TODO: toggle from server
#else
static bool mag_enabled = false;
#endif

#if CONFIG_SENSOR_USE_XIOFUSION
static const sensor_fusion_t *sensor_fusion = &sensor_fusion_fusion; // TODO: change from server
int fusion_id = FUSION_FUSION;
#elif CONFIG_SENSOR_USE_VQF
static const sensor_fusion_t *sensor_fusion = &sensor_fusion_vqf; // TODO: change from server
int fusion_id = FUSION_VQF;
#endif

static int sensor_imu_id = -1;
static int sensor_mag_id = -1;
static const sensor_imu_t *sensor_imu = &sensor_imu_none;
static const sensor_mag_t *sensor_mag = &sensor_mag_none;

#if CONFIG_SENSOR_USE_TCAL
static float sensor_tcal_temp = 25.0f; // Default to 25C safety
#endif

//#define DEBUG true

#if DEBUG
LOG_MODULE_REGISTER(sensor, LOG_LEVEL_DBG);
#else
LOG_MODULE_REGISTER(sensor, LOG_LEVEL_INF);
#endif

static int sensor_scan(void);
static int sensor_init(void);
static void sensor_loop(void);
#if CONFIG_SENSOR_RANGE_STATS
static void sensor_update_range_stats_gyro(float g[3]);
static void sensor_update_range_stats_accel(float a[3]);
#endif // CONFIG_SENSOR_RANGE_STATS
static struct k_thread sensor_thread_id;
static K_THREAD_STACK_DEFINE(sensor_thread_id_stack, 2048);

K_THREAD_DEFINE(sensor_init_thread_id, 256, sensor_request_scan, true, NULL, NULL, 7, 0, 0);
//crashing on nrf54l at 256

/* init thread handles starting scanner on the main thread, and then switches to the loop, before returning
   afterwards, other calls to start scanner will stop the loop on their thread and start the scanner on its own; it will also wait for the scanner to finish
   if the loop needs to handle power off, it should start another thread or otherwise offload the call so it does not try to kill itself
   in this case, it is appropriate to queue the request to power thread
*/

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, int0_gpios)
#define IMU_INT_EXISTS true
static const struct gpio_dt_spec int0 = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, int0_gpios);
#endif

const char *sensor_get_sensor_imu_name(void)
{
	if (sensor_imu_id < 0)
		return "None";
	return dev_imu_names[sensor_imu_id];
}

bool sensor_is_initialized(void)
{
	return sensor_sensor_init;
}

const char *sensor_get_sensor_mag_name(void)
{
	if (sensor_mag_id < 0)
		return "None";
	return dev_mag_names[sensor_mag_id];
}

const char *sensor_get_sensor_fusion_name(void)
{
	if (fusion_id < 0)
		return "None";
	return fusion_names[fusion_id];
}

int sensor_get_sensor_temperature(float *ptr)
{
	if (sensor_imu == &sensor_imu_none || (k_uptime_get() - last_temp_time > 1000))
	{
		if (get_status(SYS_STATUS_SENSOR_ERROR))
			return -2; // no imu!
		else
			return -1; // imu probably not scanned yet or temp not read yet or last valid temp is old
	}
	*ptr = temp;
	return 0;
}

void sensor_scan_thread(void)
{
	int err;
	sys_interface_resume(); // make sure interfaces are enabled
	err = sensor_scan(); // IMUs discovery
	if (err)
	{
		k_msleep(5);
		LOG_INF("Retrying sensor detection");

		// Reset address before retrying sensor detection
		sensor_imu_dev.addr = 0x00;

		err = sensor_scan(); // on POR, the sensor may not be ready yet
	}
	sys_interface_suspend();
//	if (err)
//		return err;
}

int sensor_scan(void)
{
	while (sensor_sensor_scanning)
		k_usleep(1); // already scanning
	if (sensor_sensor_init)
		return 0; // already initialized
	sensor_sensor_scanning = true;

	sensor_scan_read();
	// Enable external clock for IMU (required for ICM45686 gyroscope)
	float clock_actual_rate = 0;
#if CONFIG_USE_SENSOR_CLOCK
	set_sensor_clock(true, 32768, &clock_actual_rate);
	if (clock_actual_rate != 0)
	{
		LOG_INF("Sensor clock enabled: %.2fHz", (double)clock_actual_rate);
	}
#endif

	// Wait for sensors to power up and stabilize
	k_msleep(50);

	int imu_id = -1;
#if SENSOR_IMU_SPI_EXISTS
	// for SPI scan, set frequency of 10MHz, it will be set later by the driver initialization if needed
	sensor_imu_spi_dev.config.frequency = MHZ(10);
	LOG_INF("Scanning SPI bus for IMU");
	imu_id = sensor_scan_imu_spi(&sensor_imu_spi_dev, &sensor_imu_dev_reg);
	if (imu_id >= 0)
		sensor_interface_register_sensor_imu_spi(&sensor_imu_spi_dev);
#endif
#if SENSOR_IMU_EXISTS
	if (imu_id < 0)
	{
		LOG_INF("Scanning I2C bus for IMU");
		imu_id = sensor_scan_imu(&sensor_imu_dev, &sensor_imu_dev_reg);
		if (imu_id >= 0)
			sensor_interface_register_sensor_imu_i2c(&sensor_imu_dev);
	}
#endif
#if !SENSOR_IMU_SPI_EXISTS && !SENSOR_IMU_EXISTS
	LOG_ERR("IMU node does not exist");
#endif
	if (imu_id >= (int)ARRAY_SIZE(dev_imu_names))
		LOG_WRN("Found unknown device");
	else if (imu_id < 0)
		LOG_ERR("No IMU detected");
	else
		LOG_INF("Found %s", dev_imu_names[imu_id]);
	if (imu_id >= 0)
	{
		if (imu_id >= (int)ARRAY_SIZE(sensor_imus) || sensor_imus[imu_id] == NULL || sensor_imus[imu_id] == &sensor_imu_none)
		{
			sensor_scan_clear(); // clear invalid sensor data
			sensor_imu = &sensor_imu_none;
			sensor_sensor_scanning = false; // done
			LOG_ERR("IMU not supported");
			set_status(SYS_STATUS_SENSOR_ERROR, true);
			return -1; // an IMU was detected but not supported
		}
		else
		{
			sensor_imu = sensor_imus[imu_id];
		}
	}
	else
	{
		sensor_scan_clear(); // clear invalid sensor data
		sensor_imu = &sensor_imu_none;
		sensor_sensor_scanning = false; // done
		set_status(SYS_STATUS_SENSOR_ERROR, true);
		return -1; // no IMU detected! something is very wrong
	}

	int mag_id = -1;
#if SENSOR_MAG_SPI_EXISTS
	// for SPI scan, set frequency of 10MHz, it will be set later by the driver initialization if needed
	sensor_mag_spi_dev.config.frequency = MHZ(10);
	LOG_INF("Scanning SPI bus for magnetometer");
	mag_id = sensor_scan_mag_spi(&sensor_mag_spi_dev, &sensor_mag_dev_reg);
	if (mag_id >= 0)
		sensor_interface_register_sensor_mag_spi(&sensor_mag_spi_dev);
#endif
#if SENSOR_MAG_EXISTS
	if (mag_id < 0)
	{
		LOG_INF("Scanning bus for magnetometer");
		mag_id = sensor_scan_mag(&sensor_mag_dev, &sensor_mag_dev_reg);
		if (mag_id >= 0)
			sensor_interface_register_sensor_mag_i2c(&sensor_mag_dev);
	}
	if (mag_id < 0 && !(sensor_imu_dev_reg & 0x80)) // I2C IMU
	{
		// IMU may support passthrough mode if the magnetometer is connected through the IMU
		int err = sensor_imu->ext_passthrough(true); // no need to disable, the imu will be reset later
		if (!err)
		{
			LOG_INF("Scanning bus for magnetometer through IMU passthrough");
			if (sensor_mag_dev.addr > 0x80) // marked as external
			{
				sensor_mag_dev.addr &= 0x7F;
			}
			else
			{
				sensor_mag_dev.addr = 0x00; // reset magnetometer data
				sensor_mag_dev_reg = 0xFF;
			}
			mag_id = sensor_scan_mag(&sensor_mag_dev, &sensor_mag_dev_reg);
			if (mag_id >= 0)
			{
				sensor_mag_dev.addr |= 0x80; // mark as external
				sensor_interface_register_sensor_mag_i2c(&sensor_mag_dev); // can register as i2c
			}
		}
	}
#endif
#if SENSOR_MAG_EXT_EXISTS
	if (mag_id < 0 && (sensor_imu_dev_reg & 0x80)) // SPI IMU
	{
		// IMU may support I2CM if the magnetometer is connected through the IMU
		int err = sensor_imu->ext_setup();
		if (!err)
		{
			LOG_INF("Scanning bus for magnetometer through IMU I2CM");
			if (sensor_mag_dev.addr > 0x80) // marked as external
			{
				sensor_mag_dev.addr &= 0x7F;
			}
			else
			{
				sensor_mag_dev.addr = 0x00; // reset magnetometer data
				sensor_mag_dev_reg = 0xFF;
			}
			mag_id = sensor_scan_mag_ext(sensor_interface_ext_get(), &sensor_mag_dev.addr, &sensor_mag_dev_reg);
			if (mag_id >= 0 && mag_id < (int)ARRAY_SIZE(sensor_mags) && sensor_mags[mag_id] != NULL && sensor_mags[mag_id] != &sensor_mag_none)
			{
				err = sensor_interface_register_sensor_mag_ext(sensor_mag_dev.addr, sensor_mags[mag_id]->ext_min_burst, sensor_mags[mag_id]->ext_burst);
				sensor_mag_dev.addr |= 0x80; // mark as external
				if (err)
				{
					mag_id = -1;
					LOG_ERR("Failed to register magnetometer external interface");
				}
			}
		}
	}
#endif
#if !SENSOR_MAG_SPI_EXISTS && !SENSOR_MAG_EXISTS && !SENSOR_MAG_EXT_EXISTS
	LOG_WRN("Magnetometer node does not exist");
#endif
	if (mag_id >= (int)ARRAY_SIZE(dev_mag_names))
		LOG_WRN("Found unknown device");
	else if (mag_id < 0)
		LOG_WRN("No magnetometer detected");
	else
		LOG_INF("Found %s", dev_mag_names[mag_id]);
	if (mag_id >= 0) // if there is no magnetometer we do not care as much
	{
		if (mag_id >= (int)ARRAY_SIZE(sensor_mags) || sensor_mags[mag_id] == NULL || sensor_mags[mag_id] == &sensor_mag_none)
		{
			sensor_mag = &sensor_mag_none;
			mag_available = false;
			LOG_ERR("Magnetometer not supported");
		}
		else
		{
			sensor_mag = sensor_mags[mag_id];
			mag_available = true;
		}
	}
	else
	{
		sensor_mag = &sensor_mag_none;
		mag_available = false; // marked as not available
	}

	sensor_scan_write();
	connection_update_sensor_ids(imu_id, mag_id);
	sensor_imu_id = imu_id;
	sensor_mag_id = mag_id;

	sensor_sensor_init = true; // successfully initialized
	sensor_sensor_scanning = false; // done
	set_status(SYS_STATUS_SENSOR_ERROR, false); // clear error
	return 0;
}

static bool main_running = false;

int sensor_request_scan(bool force)
{
	if (sensor_sensor_init && !force)
		return 0; // already initialized
	main_imu_suspend();

	/* Pause watchdog before aborting thread to prevent timeout */
	watchdog_pause(WDT_CHANNEL_SENSOR);

	k_thread_abort(&sensor_thread_id); // stop the sensor thread // TODO: may need to handle fusion state
	LOG_INF("Aborted sensor thread");
	main_suspended = false;
	sensor_sensor_init = false;
	if (force)
	{
		sensor_imu_dev.addr = 0x00;
		sensor_mag_dev.addr = 0x00;
		sensor_imu_dev_reg = 0xFF;
		sensor_mag_dev_reg = 0xFF;
		LOG_INF("Requested sensor scan");
	}
	k_thread_create(&sensor_thread_id, sensor_thread_id_stack, K_THREAD_STACK_SIZEOF(sensor_thread_id_stack), (k_thread_entry_t)sensor_scan_thread, NULL, NULL, NULL, 7, 0, K_NO_WAIT);
	k_thread_join(&sensor_thread_id, K_FOREVER); // wait for the thread to finish
	if (sensor_sensor_init && force)
	{
		k_thread_create(&sensor_thread_id, sensor_thread_id_stack, K_THREAD_STACK_SIZEOF(sensor_thread_id_stack), (k_thread_entry_t)sensor_loop, NULL, NULL, NULL, 7, 0, K_NO_WAIT);
		LOG_INF("Started sensor loop");
	}
	return !sensor_sensor_init;
}

void sensor_scan_read(void) // TODO: move some of this to sys?
{
	if (retained->imu_addr != 0)
	{
		sensor_imu_dev.addr = retained->imu_addr;
		sensor_imu_dev_reg = retained->imu_reg;
	}
	if (retained->mag_addr != 0)
	{
		sensor_mag_dev.addr = retained->mag_addr;
		sensor_mag_dev_reg = retained->mag_reg;
	}
	LOG_INF("IMU address: 0x%02X, register: 0x%02X", sensor_imu_dev.addr, sensor_imu_dev_reg);
	LOG_INF("Magnetometer address: 0x%02X, register: 0x%02X", sensor_mag_dev.addr, sensor_mag_dev_reg);
}

void sensor_scan_write(void) // TODO: move some of this to sys?
{
	retained->imu_addr = sensor_imu_dev.addr;
	retained->mag_addr = sensor_mag_dev.addr;
	retained->imu_reg = sensor_imu_dev_reg;
	retained->mag_reg = sensor_mag_dev_reg;
	retained_update();
}

void sensor_scan_clear(void) // TODO: move some of this to sys?
{
	retained->imu_addr = 0x00;
	retained->mag_addr = 0x00;
	retained->imu_reg = 0xFF;
	retained->mag_reg = 0xFF;
	retained_update();
}

void sensor_retained_read(void) // TODO: move some of this to sys? or move to calibration?
{
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	LOG_INF("Accelerometer matrix:");
	for (int i = 0; i < 3; i++)
		LOG_INF("%.5f %.5f %.5f %.5f", (double)retained->accBAinv[0][i], (double)retained->accBAinv[1][i], (double)retained->accBAinv[2][i], (double)retained->accBAinv[3][i]);
#else
	LOG_INF("Accelerometer bias: %.5f %.5f %.5f", (double)retained->accelBias[0], (double)retained->accelBias[1], (double)retained->accelBias[2]);
#endif
	LOG_INF("Gyroscope bias: %.5f %.5f %.5f", (double)retained->gyroBias[0], (double)retained->gyroBias[1], (double)retained->gyroBias[2]);
	if (mag_available && mag_enabled)
	{
//		LOG_INF("Magnetometer bridge offset: %.5f %.5f %.5f", (double)retained->magBias[0], (double)retained->magBias[1], (double)retained->magBias[2]);
		LOG_INF("Magnetometer matrix:");
		for (int i = 0; i < 3; i++)
			LOG_INF("%.5f %.5f %.5f %.5f", (double)retained->magBAinv[0][i], (double)retained->magBAinv[1][i], (double)retained->magBAinv[2][i], (double)retained->magBAinv[3][i]);
	}
	if (retained->fusion_id)
		LOG_INF("Fusion data recovered");
}

void sensor_retained_write(void) // TODO: move to sys?
{
	if (!sensor_fusion_init)
		return;
//	memcpy(retained->magBias, sensor_calibration_get_magBias(), sizeof(retained->magBias));
	sensor_fusion->save(retained->fusion_data);
	retained->fusion_id = fusion_id;
	retained_update();
}

void sensor_shutdown(void) // Communicate all imus to shut down
{
	int err = sensor_request_scan(false); // try initialization if possible
	if (mag_available || !err)
	{
		sys_interface_resume();
		if (mag_available) // try to shutdown magnetometer first (in case of passthrough)
			sensor_mag->shutdown();
		if (!err)
			sensor_imu->shutdown();
		sys_interface_suspend();
	}
	else
	{
		LOG_ERR("Failed to shutdown sensors");
	}
}

uint8_t sensor_setup_WOM(void)
{
	int err = sensor_request_scan(false); // try initialization if possible
	if (!err)
	{
		sys_interface_resume();
		err = sensor_imu->setup_WOM();
		sys_interface_suspend();
		return err;
	}
	else
	{
		LOG_ERR("Failed to configure IMU wake up");
		return 0;
	}
}

void sensor_fusion_invalidate(void)
{
	main_imu_restart(); // reinitialize fusion (resets quaternion to identity)
	if (sensor_fusion_init)
	{ // clear fusion gyro offset
		sensor_fusion_update_bias(NULL);
		sensor_retained_write();
	}
	else
	{ // TODO: always clearing the fusion?
		retained->fusion_id = 0; // Invalidate retained fusion data
		retained_update();
	}
}

void sensor_fusion_update_bias(float *g_off)
{
	// Lightweight bias update that preserves quaternion orientation
	// Use this after calibration changes that only affect bias/offset values
	// Pass NULL or a float[3] with the new bias values
	if (sensor_fusion_init)
	{
		float bias[3] = {0};
		if (g_off != NULL)
		{
			// Use provided bias values
			bias[0] = g_off[0];
			bias[1] = g_off[1];
			bias[2] = g_off[2];
		}
		sensor_fusion->set_gyro_bias(bias);
		sensor_retained_write();
		LOG_INF("Fusion bias updated: [%.3f, %.3f, %.3f]",
			(double)bias[0], (double)bias[1], (double)bias[2]);
	}
	else
	{
		// If fusion is not initialized yet, just invalidate retained data
		retained->fusion_id = 0;
		retained_update();
	}
}

int sensor_update_time_ms = 6;

// TODO: get rid of it.. ?
static void set_update_time_ms(int time_ms)
{
	// TODO: maybe not get rid of it? it is now repurposed to also change FIFO threshold
	// TODO: return pin_config and replace call in sensor_init
#if IMU_INT_EXISTS
	float fifo_threshold = time_ms / 1000.0f / sensor_actual_time; // target loop rate
	sensor_fifo_threshold = fifo_threshold;
	LOG_INF("FIFO THS/WM/WTM: %.2f -> %d", (double)fifo_threshold, sensor_fifo_threshold);
	sensor_imu->setup_DRDY(sensor_fifo_threshold); // do not need to reset pin config
#endif
	sensor_update_time_ms = time_ms; // TODO: terrible naming
}

bool main_wfi = false;

static void sensor_interrupt_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	// wake up sensor thread
	if (main_wfi)
	{
		// Use to time latency
		sensor_data_time = k_uptime_ticks();
		main_wfi = false;
		k_wakeup(&sensor_thread_id);
	}
	else
	{
		// need to signal to catch up thread
		main_wfi = true;
	}
}

static struct gpio_callback sensor_cb_data;

enum sensor_sensor_mode {
//	SENSOR_SENSOR_MODE_OFF,
	SENSOR_SENSOR_MODE_LOW_NOISE,
	SENSOR_SENSOR_MODE_LOW_POWER,
	SENSOR_SENSOR_MODE_LOW_POWER_2
};

static enum sensor_sensor_mode sensor_mode = SENSOR_SENSOR_MODE_LOW_NOISE;
static enum sensor_sensor_mode last_sensor_mode = SENSOR_SENSOR_MODE_LOW_NOISE;

enum sensor_sensor_timeout {
	SENSOR_SENSOR_TIMEOUT_IMU,
	SENSOR_SENSOR_TIMEOUT_IMU_ELAPSED,
	SENSOR_SENSOR_TIMEOUT_ACTIVITY,
	SENSOR_SENSOR_TIMEOUT_ACTIVITY_ELAPSED,
};

static enum sensor_sensor_timeout sensor_timeout = SENSOR_SENSOR_TIMEOUT_IMU;

// Check the IMU gyroscope // TODO: gyro sanity not used
 // TODO: timeouts and power management should be outside sensor! (ie. sleeping/shutdown even if the imu completely errored out)
 // all this really means is that this should be called in sensor loop while the sensor is in an error state
static void sensor_update_sensor_state(void)
{
	bool calibrating = get_status(SYS_STATUS_CALIBRATION_RUNNING);
	bool resting = sensor_fusion->get_gyro_sanity() == 0 ? q_epsilon(q, last_q, 0.005) : q_epsilon(q, last_q, 0.05); // TODO: Probably okay to use the constantly updating last_q?
	if (!calibrating && resting)
	{
		int64_t last_data_delta = k_uptime_get() - last_data_time;
		if (sensor_mode < SENSOR_SENSOR_MODE_LOW_POWER && last_data_delta > CONFIG_SENSOR_LP_TIMEOUT) // No motion in lp timeout
		{
			LOG_INF("No motion from sensors in %dms", CONFIG_SENSOR_LP_TIMEOUT);
			sensor_mode = SENSOR_SENSOR_MODE_LOW_POWER;
		}
#if CONFIG_SENSOR_USE_LOW_POWER_2 || CONFIG_USE_IMU_TIMEOUT
		int64_t imu_timeout = CLAMP(last_data_time - last_suspend_attempt_time, CONFIG_IMU_TIMEOUT_RAMP_MIN, CONFIG_IMU_TIMEOUT_RAMP_MAX); // Ramp timeout from last_data_time
#endif
#if CONFIG_SENSOR_USE_LOW_POWER_2
		if (sensor_mode < SENSOR_SENSOR_MODE_LOW_POWER_2 && last_data_delta > imu_timeout) // No motion in ramp time
			sensor_mode = SENSOR_SENSOR_MODE_LOW_POWER_2;
#endif
#if CONFIG_USE_ACTIVE_TIMEOUT
		if (sensor_timeout < SENSOR_SENSOR_TIMEOUT_ACTIVITY && last_data_delta > CONFIG_ACTIVE_TIMEOUT_THRESHOLD) // higher priority than IMU timeout
		{
			LOG_INF("Switching to activity timeout");
			sensor_timeout = SENSOR_SENSOR_TIMEOUT_ACTIVITY;
		}
		if (sensor_timeout == SENSOR_SENSOR_TIMEOUT_ACTIVITY && last_data_delta > CONFIG_ACTIVE_TIMEOUT_DELAY)
		{
			LOG_INF("No motion from sensors in %dm", CONFIG_ACTIVE_TIMEOUT_DELAY / 60000);
#if CONFIG_SLEEP_ON_ACTIVE_TIMEOUT && CONFIG_USE_IMU_WAKE_UP
			// Queue power state request, it is possible for the request to be overridden so the thread may continue unaware
			sys_request_WOM(true, false);
#elif CONFIG_SHUTDOWN_ON_ACTIVE_TIMEOUT && CONFIG_USER_SHUTDOWN
			// Queue power state request, thread will be suspended when entering system_off
			sys_request_system_off(false);
#endif
			sensor_timeout = SENSOR_SENSOR_TIMEOUT_ACTIVITY_ELAPSED; // only try to suspend once
		}
#endif
#if CONFIG_USE_IMU_TIMEOUT && CONFIG_USE_IMU_WAKE_UP
		if (sensor_timeout == SENSOR_SENSOR_TIMEOUT_IMU && last_data_delta > imu_timeout) // No motion in ramp time
		{
			LOG_INF("No motion from sensors in %llds", imu_timeout / 1000);
			// Queue power state request
			sys_request_WOM(false, false);
			sensor_timeout = SENSOR_SENSOR_TIMEOUT_IMU_ELAPSED; // only try to suspend once
		}
#endif
	}
	else
	{
		if (sensor_mode == SENSOR_SENSOR_MODE_LOW_POWER_2 || sensor_timeout == SENSOR_SENSOR_TIMEOUT_IMU_ELAPSED)
			last_suspend_attempt_time = k_uptime_get();
		// last_data_time now updated when sending data to improve responsiveness
		if (sensor_timeout == SENSOR_SENSOR_TIMEOUT_IMU_ELAPSED) // Resetting IMU timeout
			sensor_timeout = SENSOR_SENSOR_TIMEOUT_IMU;
		sensor_mode = SENSOR_SENSOR_MODE_LOW_NOISE;
	}
}

int sensor_init(void)
{
	int err;
	// TODO: on any errors set main_ok false and skip (make functions return nonzero)
	if (mag_available) // shutdown magnetometer first (in case of passthrough)
		sensor_mag->shutdown(); // TODO: is this needed?
	sensor_imu->shutdown(); // TODO: is this needed?

	// Clock already enabled during sensor scan, just ensure it's still on
	float clock_actual_rate = 0;
#if CONFIG_USE_SENSOR_CLOCK
	set_sensor_clock(true, 32768, &clock_actual_rate); // ensure clock source is still enabled
#endif

	// wait for sensor register reset // TODO: is this needed?
	k_usleep(250);

	// set FS/range
	float accel_range = CONFIG_SENSOR_ACCEL_FS;
	float gyro_range = CONFIG_SENSOR_GYRO_FS;
	sensor_imu->update_fs(accel_range, gyro_range, &accel_actual_range, &gyro_actual_range);
	LOG_INF("Accelerometer range: %.2fg", (double)accel_actual_range);
	LOG_INF("Gyroscope range: %.2fdps", (double)gyro_actual_range);

	// setup sensor, set ODR
	float accel_initial_time = 1.0 / CONFIG_SENSOR_ACCEL_ODR; // configure with ~1000Hz ODR
	float gyro_initial_time = 1.0 / CONFIG_SENSOR_GYRO_ODR; // configure with ~1000Hz ODR
	float mag_initial_time = sensor_update_time_ms / 1000.0; // configure with ~200Hz ODR
	err = sensor_imu->init(clock_actual_rate, accel_initial_time, gyro_initial_time, &accel_actual_time, &gyro_actual_time);
	sensor_actual_time = MIN(accel_actual_time, gyro_actual_time);
#if SENSOR_IMU_SPI_EXISTS
	LOG_INF("Requested SPI frequency: %.2fMHz", (double)sensor_imu_spi_dev.config.frequency / 1000000.0);
#endif
	LOG_INF("Accelerometer initial rate: %.2fHz", 1.0 / (double)accel_actual_time);
	LOG_INF("Gyrometer initial rate: %.2fHz", 1.0 / (double)gyro_actual_time);
	if (err < 0)
		return err;
// 55-66ms to wait, get chip ids, and setup icm (50ms spent waiting for accel and gyro to start)
	if (mag_available && mag_enabled)
	{
		// Only enable passthrough for I2C IMU with external magnetometer
		// SPI IMU with external magnetometer uses I2CM (EXT interface), not passthrough
		if ((sensor_mag_dev.addr & 0x80) && !(sensor_imu_dev_reg & 0x80))
		{
			sensor_imu->ext_passthrough(true); // reenable passthrough for I2C IMU
		}
		err = sensor_mag->init(mag_initial_time, &mag_actual_time); // configure with ~200Hz ODR
#if SENSOR_MAG_SPI_EXISTS
		LOG_INF("Requested SPI frequency: %.2fMHz", (double)sensor_mag_spi_dev.config.frequency / 1000000.0);
#endif
		LOG_INF("Magnetometer initial rate: %.2fHz", 1.0 / (double)mag_actual_time);
		if (err < 0)
			return err;
// 0-1ms to setup mmc
	}
	LOG_INF("Initialized sensors");

#if CONFIG_SENSOR_GYRO_OVERSAMPLING > 1
	// Initialize gyro oversampling state
	gyro_oversample_count = 0;
	for (int i = 0; i < 3; i++)
		gyro_oversample_sum[i] = 0;
	// Calculate effective time step for fusion after oversampling
	gyro_effective_time = gyro_actual_time * CONFIG_SENSOR_GYRO_OVERSAMPLING;
	LOG_INF("Gyro oversampling: %dx, effective rate: %.2fHz",
		CONFIG_SENSOR_GYRO_OVERSAMPLING, 1.0 / (double)gyro_effective_time);
#endif

#if CONFIG_SENSOR_ACCEL_OVERSAMPLING > 1
	// Initialize accel oversampling state
	accel_oversample_count = 0;
	for (int i = 0; i < 3; i++)
		accel_oversample_sum[i] = 0;
	// Calculate effective time step for fusion after oversampling
	accel_effective_time = accel_actual_time * CONFIG_SENSOR_ACCEL_OVERSAMPLING;
	LOG_INF("Accel oversampling: %dx, effective rate: %.2fHz",
		CONFIG_SENSOR_ACCEL_OVERSAMPLING, 1.0 / (double)accel_effective_time);
#endif

	// Setup fusion
	sensor_retained_read(); // TODO: useless
	if (fusion_id == FUSION_VQF)
		vqf_update_sensor_ids(sensor_imu_id);
	if (retained->fusion_id == fusion_id) // Check if the retained fusion data is valid and matches the selected fusion
	{ // Load state if the data is valid (fusion was initialized before)
		sensor_fusion->load(retained->fusion_data);
		retained->fusion_id = 0; // Invalidate retained fusion data
		retained_update();
	}
	else
	{
		// Determine effective gyro time step for fusion
#if CONFIG_SENSOR_GYRO_OVERSAMPLING > 1
		float fusion_gyro_time = gyro_effective_time;
#else
		float fusion_gyro_time = gyro_actual_time;
#endif
		// Determine effective accel time step for fusion
#if CONFIG_SENSOR_ACCEL_OVERSAMPLING > 1
		float fusion_accel_time = accel_effective_time;
#else
		float fusion_accel_time = accel_actual_time;
#endif
		sensor_fusion->init(fusion_gyro_time, fusion_accel_time, mag_initial_time); // TODO: using initial time since mag are not polled at the actual rate
	}

	sensor_calibration_update_sensor_ids(sensor_imu_id);
	if (sensor_imu == &sensor_imu_bmi270) // bmi270 specific
	{
		LOG_INF("Applying gyroscope gain");
		bmi_gain_apply(sensor_calibration_get_sensor_data());
	}

#if IMU_INT_EXISTS
	// Setup interrupt
	float fifo_threshold = sensor_update_time_ms / 1000.0f / sensor_actual_time; // target loop rate
	sensor_fifo_threshold = fifo_threshold;
	LOG_INF("FIFO THS/WM/WTM: %.2f -> %d", (double)fifo_threshold, sensor_fifo_threshold);
	uint8_t pin_config = sensor_imu->setup_DRDY(sensor_fifo_threshold);
	if (pin_config == 0)
		return -1;
	uint32_t int0_gpios = NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, int0_gpios);
	LOG_INF("FIFO THS/WM/WTM GPIO pin: %u, config: %u", int0_gpios, pin_config);
	uint32_t pull_flags = ((pin_config >> 4) == NRF_GPIO_PIN_PULLDOWN ? GPIO_PULL_DOWN : 0) | ((pin_config >> 4) == NRF_GPIO_PIN_PULLUP ? GPIO_PULL_UP : 0);
	gpio_pin_configure_dt(&int0, GPIO_INPUT | pull_flags);
	uint32_t int_flags = ((pin_config & 0xF) == NRF_GPIO_PIN_SENSE_LOW ? GPIO_INT_EDGE_FALLING : 0) | ((pin_config & 0xF) == NRF_GPIO_PIN_SENSE_HIGH ? GPIO_INT_EDGE_RISING : 0);
	gpio_pin_interrupt_configure_dt(&int0, int_flags);
	gpio_init_callback(&sensor_cb_data, sensor_interrupt_handler, BIT(int0.pin));
	gpio_add_callback(int0.port, &sensor_cb_data);
#else
	LOG_WRN("IMU FIFO THS/WM/WTM GPIO does not exist");
	LOG_WRN("IMU FIFO THS/WM/WTM not available");
#endif

	LOG_INF("Using %s", fusion_names[fusion_id]);
	LOG_INF("Initialized fusion");
	sensor_fusion_init = true;
	return 0;
}

static bool main_ok = false;

static int packet_errors = 0;

#define ACQUISITION_START_MS 1000
#define STATUS_INTERVAL_MS 5000

static int64_t last_status_time = 0;
static int64_t max_loop_time = 0;

#if DEBUG
static int64_t last_acquisition_time = INT64_MAX;
static uint64_t total_acquisition_time = 0;
static uint64_t total_read_packets = 0;
static uint64_t total_processed_packets = 0;
static uint64_t total_gyro_samples = 0;
static uint64_t total_accel_samples = 0;
static uint64_t total_loop_time = 0;
static uint64_t total_loop_iterations = 0;
#endif

void sensor_loop(void)
{
	if (!sensor_sensor_init)
		return;
	main_running = true;
	sys_interface_resume(); // make sure interfaces are enabled

	/* Register sensor thread with watchdog */
	watchdog_register_thread(WDT_CHANNEL_SENSOR, 0);

	int err = sensor_init(); // Initialize IMUs and Fusion // TODO: run as thread before loop
	// TODO: handle imu init error, maybe restart device?
	// TODO: on failure to init, disable sensor interface
	if (err)
		set_status(SYS_STATUS_SENSOR_ERROR, true); // TODO: only handles general init error
	else
		main_ok = true;
	while (1)
	{
		int64_t time_begin = k_uptime_get();
		if (main_ok)
		{
#if DEBUG
			int64_t loop_begin = k_uptime_ticks();
#endif
			// Resume devices
			sys_interface_resume();

			// Trigger reconfig on sensor mode change
			bool reconfig = last_sensor_mode != sensor_mode;
			last_sensor_mode = sensor_mode;

			// Reading IMUs will take between 2.5ms (~7 samples, low noise) - 7ms (~33 samples, low power)
			// Magneto sample will take ~400us
			// Fusing data will take between 100us (~7 samples, low noise) - 500us (~33 samples, low power) for xiofusion
			// TODO: on any errors set main_ok false and skip (make functions return nonzero)

			// At high speed, use oneshot mode to have synced magnetometer data
			// Call before FIFO and get the data after
			if (mag_available && mag_enabled && mag_use_oneshot)
				sensor_mag->mag_oneshot();

#if CONFIG_SENSOR_USE_TCAL
			// Read IMU temperature
			temp = sensor_imu->temp_read();
			// Only update if the value looks like a valid temperature (-10 to 60).
			if (temp != 0.0f && temp > -10.0f && temp < 60.0f)
			{
				last_temp_time = k_uptime_get();
				sensor_tcal_temp = temp; // Update the static cache
				connection_update_sensor_temp(temp);
			}
#else
			// Read IMU temperature
			temp = sensor_imu->temp_read(); // TODO: use as calibration data
			last_temp_time = k_uptime_get();
			connection_update_sensor_temp(temp);
#endif

			// Read gyroscope (FIFO)
			// Buffer size calculation:
			// - Worst case is ICM 20 byte packet
			// - At 1600Hz gyro ODR with 6ms update interval: 1600 * 0.006 = ~10 packets
			// - At 1000Hz ODR with 33ms low power update: 1000 * 0.033 = ~33 packets
			// - At 1000Hz ODR with 100ms low power 2 update: 1000 * 0.100 = ~100 packets
			// - With 4x oversampling at 1600Hz: effectively same as 400Hz but with 4x raw packets
#if CONFIG_SENSOR_USE_LOW_POWER_2
			uint8_t* rawData = (uint8_t*)k_malloc(2048);  // Increased for oversampling: worst case ~100 packets * 20 bytes = 2000 bytes
			if (rawData == NULL)
			{
				LOG_ERR("Failed to allocate memory for FIFO buffer");
				set_status(SYS_STATUS_SENSOR_ERROR, true);
				main_ok = false;
			}
			uint16_t packets = sensor_imu->fifo_read(rawData, 2048);
#elif CONFIG_SENSOR_GYRO_OVERSAMPLING > 1
			// With oversampling, we read more raw gyro samples per update interval
			// E.g., 1600Hz * 6ms = ~10 packets, but need margin for timing jitter
			uint8_t* rawData = (uint8_t*)k_malloc(1536);  // ~75 packets * 20 bytes, enough for 4x oversampling
			if (rawData == NULL)
			{
				LOG_ERR("Failed to allocate memory for FIFO buffer");
				set_status(SYS_STATUS_SENSOR_ERROR, true);
				main_ok = false;
			}
			uint16_t packets = sensor_imu->fifo_read(rawData, 1536);
#else
			uint8_t* rawData = (uint8_t*)k_malloc(1024);  // Standard: ~50 packets * 20 bytes
			if (rawData == NULL)
			{
				LOG_ERR("Failed to allocate memory for FIFO buffer");
				set_status(SYS_STATUS_SENSOR_ERROR, true);
				main_ok = false;
			}
			uint16_t packets = sensor_imu->fifo_read(rawData, 1024);
#endif

			// Debug info
#if DEBUG
			int64_t acquisition_time = k_uptime_ticks();
			bool valid_acquisition = k_uptime_get() > ACQUISITION_START_MS && last_acquisition_time < acquisition_time; // wait before beginning profiling
			if (valid_acquisition)
			{
				total_acquisition_time += acquisition_time - last_acquisition_time;
				total_read_packets += packets;
			}
			last_acquisition_time = acquisition_time;
#endif

			// Read magnetometer
			float raw_m[3];
			if (mag_available && mag_enabled)
				sensor_mag->mag_read(raw_m); // reading mag last, and it will be processed last

			if (reconfig) // TODO: get rid of reconfig?
			{
				// Changing FIFO threshold here should be fine since FIFO is empty now
				// TODO: causing warnings since packet processing and loop timing still expects previous update_time
				switch (sensor_mode)
				{
				case SENSOR_SENSOR_MODE_LOW_NOISE:
					set_update_time_ms(6);
					LOG_INF("Switching sensors to low noise");
					break;
				case SENSOR_SENSOR_MODE_LOW_POWER:
					set_update_time_ms(33);
					LOG_INF("Switching sensors to low power");
					break;
				case SENSOR_SENSOR_MODE_LOW_POWER_2:
					set_update_time_ms(100);
					LOG_INF("Switching sensors to low power 2");
					break;
				};
			}

			// Suspend devices
			sys_interface_suspend();

			// Fuse all data
			int g_count = 0;
			float a_sum[3] = {0};
			int a_count = 0;
			max_gyro_speed_square = 0;
			int processed_packets = 0;

			// For debug: accumulate raw and calibrated data
			float debug_raw_g_sum[3] = {0};
			float debug_raw_a_sum[3] = {0};
			float debug_cal_g_sum[3] = {0};
			int debug_g_samples = 0;
			int debug_a_samples = 0;
			for (uint16_t i = 0; i < packets; i++)
			{
				float raw_a[3] = {0};
				float raw_g[3] = {0};
				if (sensor_imu->fifo_process(i, rawData, raw_a, raw_g))
					continue; // skip on error

				// Debug: Log gyro values to see if they're all zero
				static int gyro_log_count = 0;
				if (gyro_log_count < 10)
				{
					LOG_INF("Gyro raw: %.3f, %.3f, %.3f", (double)raw_g[0], (double)raw_g[1], (double)raw_g[2]);
					gyro_log_count++;
				}

				// TODO: split into separate functions
				if (raw_g[0] != 0 || raw_g[1] != 0 || raw_g[2] != 0)
				{
#if DEBUG
					if (valid_acquisition)
						total_gyro_samples++;
#endif
					// Accumulate raw gyro for debug
					if (sensor_debug_is_active()) {
						for (int j = 0; j < 3; j++)
							debug_raw_g_sum[j] += raw_g[j];
						debug_g_samples++;
					}

#if CONFIG_SENSOR_GYRO_OVERSAMPLING > 1
					// Gyroscope oversampling: accumulate raw samples BEFORE calibration
					// This is the optimal position because:
					// 1. Noise reduction is most effective on raw data before any processing
					// 2. Calibration (bias/sensitivity) are linear operations, so order doesn't affect result mathematically
					// 3. More efficient: calibration operations run once per averaged sample instead of per raw sample
#if CONFIG_CMSIS_DSP
					// CMSIS-DSP optimized vector accumulation
					arm_add_f32(gyro_oversample_sum, raw_g, gyro_oversample_sum, 3);
#else
					for (int j = 0; j < 3; j++)
						gyro_oversample_sum[j] += raw_g[j];
#endif
					gyro_oversample_count++;

					// When we have enough samples, compute average and then apply calibration
					if (gyro_oversample_count >= CONFIG_SENSOR_GYRO_OVERSAMPLING)
					{
						float g_avg[3];
#if CONFIG_CMSIS_DSP
						// CMSIS-DSP optimized vector scaling (averaging) and reset
						float scale = 1.0f / CONFIG_SENSOR_GYRO_OVERSAMPLING;
						arm_scale_f32(gyro_oversample_sum, scale, g_avg, 3);
						arm_fill_f32(0.0f, gyro_oversample_sum, 3);
#else
						for (int j = 0; j < 3; j++)
						{
							g_avg[j] = gyro_oversample_sum[j] / CONFIG_SENSOR_GYRO_OVERSAMPLING;
							gyro_oversample_sum[j] = 0; // Reset accumulator
						}
#endif
						gyro_oversample_count = 0;

						// Now apply calibration to the averaged data
						sensor_calibration_process_gyro(g_avg);

#if CONFIG_SENSOR_USE_SENS_CALIBRATION
						// Apply sensitivity scaling
						if (retained) {
							g_avg[0] *= retained->gyroSensScale[0];
							g_avg[1] *= retained->gyroSensScale[1];
							g_avg[2] *= retained->gyroSensScale[2];
						}
#endif

						// Accumulate calibrated gyro for debug (after zero bias and sensitivity calibration)
						if (sensor_debug_is_active()) {
							for (int j = 0; j < 3; j++)
								debug_cal_g_sum[j] += g_avg[j];
						}

#if CONFIG_SENSOR_RANGE_STATS
						// Update range statistics with calibrated gyro data
						sensor_update_range_stats_gyro(g_avg);
#endif // CONFIG_SENSOR_RANGE_STATS

						// Process fusion with averaged and calibrated gyro data
						sensor_fusion->update_gyro(g_avg, gyro_effective_time);
						g_count++;

						if (mag_available && mag_enabled)
						{
							// Get fusion's corrected gyro data (or get gyro bias from fusion) and use it here
							float g_off[3] = {};
							sensor_fusion->get_gyro_bias(g_off);
							for (int j = 0; j < 3; j++)
								g_off[j] = g_avg[j] - g_off[j];

							// Get the highest gyro speed
							float gyro_speed_square = g_off[0] * g_off[0] + g_off[1] * g_off[1] + g_off[2] * g_off[2];
							if (gyro_speed_square > max_gyro_speed_square)
								max_gyro_speed_square = gyro_speed_square;
						}
					}
#else
					// No oversampling: apply calibration directly
					sensor_calibration_process_gyro(raw_g);
					float gx = raw_g[0];
					float gy = raw_g[1];
					float gz = raw_g[2];
					float g[] = {gx, gy, gz};

#if CONFIG_SENSOR_USE_SENS_CALIBRATION
					// Apply sensitivity scaling
					if (retained) {
						g[0] *= retained->gyroSensScale[0];
						g[1] *= retained->gyroSensScale[1];
						g[2] *= retained->gyroSensScale[2];
					}
#endif

					// Accumulate calibrated gyro for debug (after zero bias and sensitivity calibration)
					if (sensor_debug_is_active()) {
						for (int j = 0; j < 3; j++)
							debug_cal_g_sum[j] += g[j];
					}

#if CONFIG_SENSOR_RANGE_STATS
					// Update range statistics with calibrated gyro data
					sensor_update_range_stats_gyro(g);
#endif // CONFIG_SENSOR_RANGE_STATS

					// Process fusion directly
					sensor_fusion->update_gyro(g, gyro_actual_time);

					g_count++;

					if (mag_available && mag_enabled)
					{
						// Get fusion's corrected gyro data (or get gyro bias from fusion) and use it here
						float g_off[3] = {};
						sensor_fusion->get_gyro_bias(g_off);
						for (int j = 0; j < 3; j++)
							g_off[j] = g[j] - g_off[j];

						// Get the highest gyro speed
						float gyro_speed_square = g_off[0] * g_off[0] + g_off[1] * g_off[1] + g_off[2] * g_off[2];
						if (gyro_speed_square > max_gyro_speed_square)
							max_gyro_speed_square = gyro_speed_square;
					}
#endif
				}

				if (raw_a[0] != 0 || raw_a[1] != 0 || raw_a[2] != 0)
				{
#if DEBUG
					if (valid_acquisition)
						total_accel_samples++;
#endif
					// Accumulate raw accel for debug
					if (sensor_debug_is_active()) {
						for (int i = 0; i < 3; i++)
							debug_raw_a_sum[i] += raw_a[i];
						debug_a_samples++;
					}

#if CONFIG_SENSOR_ACCEL_OVERSAMPLING > 1
					// Accelerometer oversampling: accumulate raw samples BEFORE calibration
					// This is the optimal position because:
					// 1. Noise reduction is most effective on raw data before any processing
					// 2. Calibration (bias/scale) are linear operations, so order doesn't affect result mathematically
					// 3. More efficient: calibration operations run once per averaged sample instead of per raw sample
#if CONFIG_CMSIS_DSP
					// CMSIS-DSP optimized vector accumulation
					arm_add_f32(accel_oversample_sum, raw_a, accel_oversample_sum, 3);
#else
					for (int j = 0; j < 3; j++)
						accel_oversample_sum[j] += raw_a[j];
#endif
					accel_oversample_count++;

					// When we have enough samples, compute average and then apply calibration
					if (accel_oversample_count >= CONFIG_SENSOR_ACCEL_OVERSAMPLING)
					{
						float a_avg[3];
#if CONFIG_CMSIS_DSP
						// CMSIS-DSP optimized vector scaling (averaging) and reset
						float scale = 1.0f / CONFIG_SENSOR_ACCEL_OVERSAMPLING;
						arm_scale_f32(accel_oversample_sum, scale, a_avg, 3);
						arm_fill_f32(0.0f, accel_oversample_sum, 3);
#else
						for (int j = 0; j < 3; j++)
						{
							a_avg[j] = accel_oversample_sum[j] / CONFIG_SENSOR_ACCEL_OVERSAMPLING;
							accel_oversample_sum[j] = 0; // Reset accumulator
						}
#endif
						accel_oversample_count = 0;

						// Now apply calibration to the averaged data
						// Always call sensor_calibration_process_accel to update sample buffers
						// This is needed for calibration routines (e.g., wait_for_motion) even without 6-side calibration
						sensor_calibration_process_accel(a_avg);

						float ax = a_avg[0];
						float ay = a_avg[1];
						float az = a_avg[2];
						float a[] = {ax, ay, az};

#if CONFIG_SENSOR_RANGE_STATS
						// Update range statistics with calibrated accel data
						sensor_update_range_stats_accel(a);
#endif // CONFIG_SENSOR_RANGE_STATS

						// Process fusion with averaged and calibrated accel data
						sensor_fusion->update_accel(a, accel_effective_time);

						for (int i = 0; i < 3; i++)
							a_sum[i] += a[i];
						a_count++;
					}
#else
					// No oversampling: apply calibration directly
					// Always call sensor_calibration_process_accel to update sample buffers
					// This is needed for calibration routines (e.g., wait_for_motion) even without 6-side calibration
					sensor_calibration_process_accel(raw_a);

					float ax = raw_a[0];
					float ay = raw_a[1];
					float az = raw_a[2];
					float a[] = {ax, ay, az};

#if CONFIG_SENSOR_RANGE_STATS
					// Update range statistics with calibrated accel data
					sensor_update_range_stats_accel(a);
#endif // CONFIG_SENSOR_RANGE_STATS

					// Process fusion
					sensor_fusion->update_accel(a, accel_actual_time);

					for (int i = 0; i < 3; i++)
						a_sum[i] += a[i];
					a_count++;
#endif
				}

				processed_packets++;
			}

			// Free the FIFO buffer
			k_free(rawData);

#if DEBUG
			if (valid_acquisition)
				total_processed_packets += processed_packets;
#endif

			if (mag_available && mag_enabled)
			{
				bool mag_calibrated = true;
				float uncalibrated_m[3] = {0};
				memcpy(uncalibrated_m, raw_m, sizeof(uncalibrated_m)); // copy raw magnetometer data
				sensor_calibration_process_mag(raw_m);
				float zero_m[3] = {0};
				if (v_epsilon(raw_m, zero_m, 1e-6)) // if the magnetometer is not calibrated, skip and send raw data
				{
					memcpy(raw_m, uncalibrated_m, sizeof(uncalibrated_m));
					mag_calibrated = false;
				}
				float mx = raw_m[0];
				float my = raw_m[1];
				float mz = raw_m[2];
				float m[] = {SENSOR_MAGNETOMETER_AXES_ALIGNMENT};

				// Process fusion
				if (mag_calibrated)
					sensor_fusion->update_mag(m, sensor_update_time_ms / 1000.0); // TODO: use actual time?

				v_rotate(m, q3, m); // magnetic field in local device frame, no other transformation will be done
				connection_update_sensor_mag(m);
			}

			// Copy average acceleration for this frame
			static float a[3] = {0}; // keep persistent
			if (a_count > 0)
			{
				for (int i = 0; i < 3; i++)
					a[i] = a_sum[i] / a_count;
			}

			// Check packet processing
			if ((packets != 0 || k_uptime_get() > 100) && processed_packets == 0)
			{
				if (packets)
					LOG_WRN("No packets processed");
				else
					LOG_WRN("No packets in buffer");
				if (++packet_errors == 10)
				{
					LOG_ERR("Packet error threshold exceeded");
					set_status(SYS_STATUS_SENSOR_ERROR, true);
					if (packets)
					{
						sensor_retained_write(); // keep the fusion state
						sys_request_system_reboot(false);
					}
				}
			}
			else if (processed_packets == packets && packets > 0)
			{
				packet_errors = 0;
			}

			// Check if expected number of timesteps when using FIFO threshold
			// When accel and gyro have different ODRs, check them separately based on their expected rates
			// The FIFO threshold is calculated based on the faster sensor, which determines interrupt timing
			if (sensor_fifo_threshold && (g_count || a_count))
			{
				// Calculate expected samples based on actual sensor rates
				// sensor_fifo_threshold is based on sensor_actual_time = MIN(accel_actual_time, gyro_actual_time)
				// So we need to calculate expected samples for each sensor independently
				float expected_gyro_samples = sensor_update_time_ms / 1000.0f / gyro_actual_time;
				float expected_accel_samples = sensor_update_time_ms / 1000.0f / accel_actual_time;

#if CONFIG_SENSOR_GYRO_OVERSAMPLING > 1
				// With gyro oversampling, expected fusion timesteps is reduced by oversampling factor
				float expected_gyro_timesteps_f = expected_gyro_samples / CONFIG_SENSOR_GYRO_OVERSAMPLING;
				// Only warn if actual count is significantly off (more than ±50% or at least ±1)
				// This handles fractional expected values better
				if (g_count) {
					int min_expected = (int)expected_gyro_timesteps_f; // floor
					int max_expected = (int)(expected_gyro_timesteps_f + 0.99f); // ceiling
					if (g_count < min_expected - 1 || g_count > max_expected + 1)
						LOG_WRN("Expected ~%.1f gyro timestep%s (oversampling %dx), got %d",
							(double)expected_gyro_timesteps_f,
							expected_gyro_timesteps_f == 1.0f ? "" : "s",
							CONFIG_SENSOR_GYRO_OVERSAMPLING, g_count);
				}
#else
				// Check gyro samples: allow reasonable tolerance for timing variations
				// Since FIFO threshold uses floor(), actual samples can range from floor to floor+1
				if (g_count) {
					int min_expected = (int)expected_gyro_samples; // floor
					int max_expected = (int)(expected_gyro_samples + 0.99f); // ceiling
					if (g_count < min_expected - 1 || g_count > max_expected + 1)
						LOG_WRN("Expected ~%.1f gyro sample%s, got %d",
							(double)expected_gyro_samples,
							expected_gyro_samples == 1.0f ? "" : "s", g_count);
				}
#endif

#if CONFIG_SENSOR_ACCEL_OVERSAMPLING > 1
				// With accel oversampling, expected fusion timesteps is reduced by oversampling factor
				float expected_accel_timesteps_f = expected_accel_samples / CONFIG_SENSOR_ACCEL_OVERSAMPLING;
				// Only warn if actual count is significantly off
				if (a_count) {
					int min_expected = (int)expected_accel_timesteps_f; // floor
					int max_expected = (int)(expected_accel_timesteps_f + 0.99f); // ceiling
					if (a_count < min_expected - 1 || a_count > max_expected + 1)
						LOG_WRN("Expected ~%.1f accel timestep%s (oversampling %dx), got %d",
							(double)expected_accel_timesteps_f,
							expected_accel_timesteps_f == 1.0f ? "" : "s",
							CONFIG_SENSOR_ACCEL_OVERSAMPLING, a_count);
				}
#else
				// Check accel samples: allow reasonable tolerance for timing variations
				if (a_count) {
					int min_expected = (int)expected_accel_samples; // floor
					int max_expected = (int)(expected_accel_samples + 0.99f); // ceiling
					if (a_count < min_expected - 1 || a_count > max_expected + 1)
						LOG_WRN("Expected ~%.1f accel sample%s, got %d",
							(double)expected_accel_samples,
							expected_accel_samples == 1.0f ? "" : "s", a_count);
				}
#endif
			}

			// Update fusion gyro sanity? // TODO: use to detect drift and correct or suspend tracking
//			sensor_fusion->update_gyro_sanity(g, m);

			// Get updated quaternion from fusion
			sensor_fusion->get_quat(q);
			q_normalize(q, q); // safe to use self as output

			// Get linear acceleration
			float lin_a[3] = {0};
			if (v_diff_mag(a, lin_a) != 0) // lin_a as zero vector
				a_to_lin_a(q, a, lin_a);

			sensor_update_sensor_state();

			// Update magnetometer mode
			if (mag_available && mag_enabled)
			{
				// TODO: magnetometer might be better to limit to a lower (fixed) rate
				float gyro_speed = sqrtf(max_gyro_speed_square);
				float mag_target_time = 1.0f / (4 * gyro_speed); // target mag ODR for ~0.25 deg error
				if (mag_target_time < 0.005f && mag_skip_oneshot) // only use continuous modes if oneshot is not available
					mag_target_time = 0.005f;
				if (mag_target_time > 0.1f) // limit to 0.1 (minimum 10Hz)
					mag_target_time = 0.1f;
				sys_interface_resume();
				if (mag_target_time < 0.005f) // cap at 0.005 (200Hz), above this the sensor will use oneshot mode instead
				{
					int err = sensor_mag->update_odr(INFINITY, &mag_actual_time);
					if (mag_actual_time == INFINITY)
					{
						if (!err)
							LOG_DBG("Switching magnetometer to oneshot");
						mag_use_oneshot = true;
					}
					else // magnetometer did not have a oneshot mode, try 200Hz
					{
						if (!err)
							mag_skip_oneshot = true;
						mag_target_time = 0.005f;
					}
				}
				if (mag_target_time >= 0.005f || mag_actual_time != INFINITY) // under 200Hz or magnetometer did not have a oneshot mode
				{
					int err = sensor_mag->update_odr(mag_target_time, &mag_actual_time);
					if (!err)
						LOG_DBG("Switching magnetometer ODR to %.2fHz", 1.0 / (double)mag_actual_time);
					mag_use_oneshot = false;
				}
				sys_interface_suspend();
			}

			// Debug mode output - based on accel sample count, not time interval
			if (sensor_debug_is_active() && debug_a_samples > 0) {
				debug_state.accel_count += debug_a_samples;
				if (debug_state.accel_count >= debug_state.output_every_n) {
					debug_state.accel_count = 0;
					debug_state.output_count++;

					int64_t current_time = k_uptime_get();
					float elapsed_sec = (float)(current_time - debug_state.start_time) / 1000.0f;

					// Calculate average raw and calibrated data
					float avg_raw_g[3] = {0};
					float avg_raw_a[3] = {0};
					float avg_cal_g[3] = {0};
					if (debug_g_samples > 0) {
						for (int i = 0; i < 3; i++) {
							avg_raw_g[i] = debug_raw_g_sum[i] / debug_g_samples;
							avg_cal_g[i] = debug_cal_g_sum[i] / debug_g_samples;
						}
					}
					if (debug_a_samples > 0) {
						for (int i = 0; i < 3; i++)
							avg_raw_a[i] = debug_raw_a_sum[i] / debug_a_samples;
					}

					// Get VQF debug info
#if CONFIG_SENSOR_USE_VQF
					vqf_debug_info_t vqf_info;
					vqf_get_debug_info(&vqf_info);
#endif

					// Compact output format with raw, calibrated, and fused data
					printk("[%.2fs] RAW: A[%.3f,%.3f,%.3f] G[%.2f,%.2f,%.2f] T:%.2fC\n",
						(double)elapsed_sec,
						(double)avg_raw_a[0], (double)avg_raw_a[1], (double)avg_raw_a[2],
						(double)avg_raw_g[0], (double)avg_raw_g[1], (double)avg_raw_g[2],
						(double)temp);

					printk("     CAL: A[%.3f,%.3f,%.3f] G[%.2f,%.2f,%.2f]\n",
						(double)a[0], (double)a[1], (double)a[2],
						(double)avg_cal_g[0], (double)avg_cal_g[1], (double)avg_cal_g[2]);

#if CONFIG_SENSOR_USE_VQF
					printk("     VQF: Q[%.3f,%.3f,%.3f,%.3f] LinA[%.2f,%.2f,%.2f]\n",
						(double)q[0], (double)q[1], (double)q[2], (double)q[3],
						(double)lin_a[0], (double)lin_a[1], (double)lin_a[2]);
					printk("     Rest:%c RestDev[G:%.3f,A:%.3f] Bias[%.3f,%.3f,%.3f]°/s Sigma:%.3f°/s Delta:%.2f°\n",
						vqf_info.rest_detected ? 'Y' : 'N',
						(double)vqf_info.rest_deviations[0], (double)vqf_info.rest_deviations[1],
						(double)vqf_info.bias[0], (double)vqf_info.bias[1], (double)vqf_info.bias[2],
						(double)vqf_info.bias_sigma, (double)vqf_info.delta);
					printk("     MagDist:%c MagRefNorm:%.3f MagRefDip:%.2f°\n",
						vqf_info.mag_dist_detected ? 'Y' : 'N',
						(double)vqf_info.mag_ref_norm, (double)vqf_info.mag_ref_dip);
#else
					printk("     Q[%.3f,%.3f,%.3f,%.3f] LinA[%.2f,%.2f,%.2f]\n",
						(double)q[0], (double)q[1], (double)q[2], (double)q[3],
						(double)lin_a[0], (double)lin_a[1], (double)lin_a[2]);
#endif
				}
			}

			// Update orientation
			bool send_quat_data = !q_epsilon(q, last_q, 0.001f);
			bool send_lin_accel_data = !v_epsilon(lin_a, last_lin_a, 0.03f);

			// Check if we need to force send based on time to maintain minimum packet rate
			int64_t now = k_uptime_get();
			bool resting = sensor_fusion->get_gyro_sanity() == 0 ? q_epsilon(q, last_q, 0.005f) : q_epsilon(q, last_q, 0.05f);
			int64_t min_interval = 1000;
			bool force_send_by_time = (now - last_sensor_send_time) >= min_interval;

			if (send_quat_data || send_lin_accel_data || force_send_by_time)
			{
				memcpy(last_q, q, sizeof(q));
				memcpy(last_lin_a, lin_a, sizeof(lin_a));
				float q_offset[4];
				q_multiply(q, q3, q_offset); // quaternion in device orientation, connection will change format from wxyz to xyzw
				v_rotate(lin_a, q3, lin_a); // linear acceleration in local device frame, no other transformation will be done

				if (!send_quat_data && !send_lin_accel_data) {
					memset(lin_a, 0, sizeof(lin_a)); // zero out linear acceleration when no motion detected
				}

				connection_update_sensor_data(q_offset, lin_a, sensor_data_time);
				last_sensor_send_time = now;

				if (!resting) {
					last_data_time = now;
				}
			}

#if CONFIG_SENSOR_USE_TCAL
			// Check for boot calibration (higher priority than auto calibration)
			sensor_tcal_boot_calibration_check();

			// Check for runtime periodic calibration (when device is resting for extended period)
			// This helps maintain accuracy during long usage sessions by updating D_offset
			sensor_runtime_calibration_check(resting);

			// Check for automatic temperature calibration (only when device is resting)
			if (resting) {
				float current_temp = temp;
				if (!isnan(current_temp)) {
					sensor_tcal_check_auto_calibration(current_temp);
					// If auto-calibration is enabled, reset last_data_time to prevent sleep
					if (sensor_tcal_get_auto_calibration()) {
						last_data_time = now;
					}
				}
			}
#endif
			// Handle magnetometer calibration
			if (mag_available && mag_enabled && last_sensor_mode == SENSOR_SENSOR_MODE_LOW_POWER && sensor_mode == SENSOR_SENSOR_MODE_LOW_POWER)
				sensor_request_calibration_mag();

			// Periodic retained save for crash recovery
			if (now - last_retained_save_time >= RETAINED_SAVE_INTERVAL_MS)
			{
				sensor_retained_write();
				last_retained_save_time = now;
				LOG_DBG("Periodic retained save completed");
			}


#if DEBUG
			if (valid_acquisition)
			{
				total_loop_time += k_uptime_ticks() - loop_begin;
				total_loop_iterations++;
			}
#endif
		}

		/* Feed watchdog at end of each loop iteration */
		watchdog_feed(WDT_CHANNEL_SENSOR);

		main_running = false;
		int64_t time_delta = k_uptime_get() - time_begin;

		if (time_delta > sensor_update_time_ms && time_delta > max_loop_time)
			max_loop_time = time_delta;

		if (k_uptime_get() - last_status_time > STATUS_INTERVAL_MS)
		{
			last_status_time = k_uptime_get();
			if (max_loop_time > 0)
			{
				LOG_WRN("Last update steps took up to %lld ms", time_delta);
				max_loop_time = 0;
			}
#if DEBUG
			LOG_DBG("loop iterations: %llu, packets read: %llu, processed: %llu, gyro samples: %llu, accel samples: %llu, total acquisition time: %lld us, total loop time: %lld us", total_loop_iterations, total_read_packets, total_processed_packets, total_gyro_samples, total_accel_samples, k_ticks_to_us_near64(total_acquisition_time), k_ticks_to_us_near64(total_loop_time));
			LOG_DBG("sensor loop rate: %.2fHz, processing time: %.2f/%.2f us -> %.2f%%", (double)total_loop_iterations / (double)k_ticks_to_us_near64(total_acquisition_time) * 1000000.0, (double)k_ticks_to_us_near64(total_loop_time) / (double)total_loop_iterations, (double)k_ticks_to_us_near64(total_acquisition_time) / (double)total_loop_iterations, (double)total_loop_time / (double)total_acquisition_time * 100.0);
			LOG_DBG("reported gyro rate: %.2fHz, actual: %.2fHz, reported accel rate: %.2fHz, actual: %.2fHz", 1.0 / (double)gyro_actual_time, (double)total_gyro_samples / (double)k_ticks_to_us_near64(total_acquisition_time) * 1000000.0, 1.0 / (double)accel_actual_time, (double)total_accel_samples / (double)k_ticks_to_us_near64(total_acquisition_time) * 1000000.0);
#endif
		}

#if IMU_INT_EXISTS
		sensor_data_time = 0; // reset data time
		if (!main_wfi)
		{
			main_wfi = true; // TODO: this is terrible
			k_msleep(sensor_update_time_ms + 10); // will be resumed by interrupt // TODO: dont use hard timeout
			if (main_wfi) // timeout
			{
				LOG_DBG("Sensor interrupt timeout");
				main_wfi = false;
			}
		}
		else // if signal was sent during processing, loop immediately to catch up
		{
			LOG_INF("FIFO THS/WM/WTM triggered during loop");
			k_yield();
			main_wfi = false;
		}
#else
		// TODO: old behavior
//		led_clock_offset += time_delta;
		if (time_delta > sensor_update_time_ms)
			k_yield();
		else
			k_msleep(sensor_update_time_ms - time_delta);
#endif

		if (main_suspended) // TODO:
			k_thread_suspend(&sensor_thread_id);

		main_running = true;
	}
}

void wait_for_threads(void) // TODO: add timeout
{
	while (main_running)
		k_usleep(1); // bane of my existence. don't use k_yield()!!!!!!
}

void main_imu_suspend(void) // TODO: add timeout
{
	main_suspended = true;
	if (!main_running) // don't suspend if already stopped (TODO: may be called from sensor thread)
		return;
	while (sensor_sensor_scanning)
		k_usleep(1); // try not to interrupt scanning
	while (main_running) // TODO: change to detect if i2c is busy
		k_usleep(1); // try not to interrupt anything actually
	k_thread_suspend(&sensor_thread_id);
	LOG_INF("Suspended sensor thread");
}

void main_imu_resume(void)
{
	if (!main_suspended) // not suspended
		return;
	k_thread_resume(&sensor_thread_id);
	LOG_INF("Resumed sensor thread");
}

void main_imu_wakeup(void)
{
	if (!main_suspended) // don't wake up if pending suspension
		k_wakeup(&sensor_thread_id);
}

void main_imu_restart(void)
{
	if (main_ok) // only restart fusion if initialized
	{
		// Determine effective gyro time step for fusion (must match sensor_init logic)
#if CONFIG_SENSOR_GYRO_OVERSAMPLING > 1
		float fusion_gyro_time = gyro_effective_time;
#else
		float fusion_gyro_time = gyro_actual_time;
#endif
		// Determine effective accel time step for fusion
#if CONFIG_SENSOR_ACCEL_OVERSAMPLING > 1
		float fusion_accel_time = accel_effective_time;
#else
		float fusion_accel_time = accel_actual_time;
#endif
		sensor_fusion->init(fusion_gyro_time, fusion_accel_time, 6 / 1000.0f); // TODO: using default initial time
	}
}

#if CONFIG_SENSOR_USE_TCAL
// Public function to get the current IMU temperature
float sensor_get_current_imu_temperature(void)
{
	return sensor_tcal_temp;
}
#endif

// Get actual accelerometer ODR in Hz
float sensor_get_accel_odr(void)
{
	if (accel_actual_time > 0.0f) {
		return 1.0f / accel_actual_time;
	}
	return (float)CONFIG_SENSOR_ACCEL_ODR; // Fallback to config value
}

// Get actual gyroscope ODR in Hz
float sensor_get_gyro_odr(void)
{
	if (gyro_actual_time > 0.0f) {
		return 1.0f / gyro_actual_time;
	}
	return (float)CONFIG_SENSOR_GYRO_ODR; // Fallback to config value
}

// Debug mode control functions
void sensor_debug_start(uint32_t duration_sec)
{
	if (duration_sec == 0 || duration_sec > 30) {
		duration_sec = 10; // Default to 10 seconds
	}

	debug_state.enabled = true;
	debug_state.start_time = k_uptime_get();
	debug_state.duration_ms = duration_sec * 1000;
	debug_state.accel_count = 0;
	debug_state.output_count = 0;
	// output_every_n is already set to 5 by default

	float accel_odr = sensor_get_accel_odr();
	LOG_INF("Debug mode started for %u seconds (accel ODR: %.1fHz, output every %u samples)",
		duration_sec, (double)accel_odr, debug_state.output_every_n);
}

void sensor_debug_stop(void)
{
	if (debug_state.enabled) {
		debug_state.enabled = false;
		LOG_INF("Debug mode stopped. %u outputs generated", debug_state.output_count);
	}
}

bool sensor_debug_is_active(void)
{
	if (debug_state.enabled) {
		int64_t elapsed = k_uptime_get() - debug_state.start_time;
		if (elapsed >= debug_state.duration_ms) {
			sensor_debug_stop();
			return false;
		}
		return true;
	}
	return false;
}

#if CONFIG_SENSOR_RANGE_STATS
// Sensor range tracking functions
const sensor_range_stats_t* sensor_get_range_stats(void)
{
	return &range_stats;
}

void sensor_reset_range_stats(void)
{
	for (int i = 0; i < 3; i++) {
		range_stats.gyro_max[i] = -INFINITY;
		range_stats.gyro_min[i] = INFINITY;
		range_stats.accel_max[i] = -INFINITY;
		range_stats.accel_min[i] = INFINITY;
	}
	range_stats.sample_count = 0;
	range_stats.initialized = false;
	LOG_INF("Range statistics reset");
}

// Internal function to update range statistics with new gyro data
static void sensor_update_range_stats_gyro(float g[3])
{
	if (!range_stats.initialized) {
		range_stats.initialized = true;
	}
	for (int i = 0; i < 3; i++) {
		if (g[i] > range_stats.gyro_max[i]) {
			range_stats.gyro_max[i] = g[i];
		}
		if (g[i] < range_stats.gyro_min[i]) {
			range_stats.gyro_min[i] = g[i];
		}
	}
}

// Internal function to update range statistics with new accel data
static void sensor_update_range_stats_accel(float a[3])
{
	if (!range_stats.initialized) {
		range_stats.initialized = true;
	}
	for (int i = 0; i < 3; i++) {
		if (a[i] > range_stats.accel_max[i]) {
			range_stats.accel_max[i] = a[i];
		}
		if (a[i] < range_stats.accel_min[i]) {
			range_stats.accel_min[i] = a[i];
		}
	}
	range_stats.sample_count++;
}

void sensor_print_range_stats(void)
{
	if (!range_stats.initialized) {
		printk("Range statistics not initialized (no data collected yet)\n");
		return;
	}

	printk("\n=== Sensor Range Statistics ===\n");
	printk("Total samples: %llu\n", range_stats.sample_count);

	printk("\nGyroscope (deg/s):\n");
	printk("  X: min=%.2f, max=%.2f, peak=%.2f\n",
		(double)range_stats.gyro_min[0],
		(double)range_stats.gyro_max[0],
		(double)fmaxf(fabsf(range_stats.gyro_min[0]), fabsf(range_stats.gyro_max[0])));
	printk("  Y: min=%.2f, max=%.2f, peak=%.2f\n",
		(double)range_stats.gyro_min[1],
		(double)range_stats.gyro_max[1],
		(double)fmaxf(fabsf(range_stats.gyro_min[1]), fabsf(range_stats.gyro_max[1])));
	printk("  Z: min=%.2f, max=%.2f, peak=%.2f\n",
		(double)range_stats.gyro_min[2],
		(double)range_stats.gyro_max[2],
		(double)fmaxf(fabsf(range_stats.gyro_min[2]), fabsf(range_stats.gyro_max[2])));

	// Calculate overall peak gyro value
	float gyro_peak = 0;
	for (int i = 0; i < 3; i++) {
		float axis_peak = fmaxf(fabsf(range_stats.gyro_min[i]), fabsf(range_stats.gyro_max[i]));
		if (axis_peak > gyro_peak) {
			gyro_peak = axis_peak;
		}
	}
	// Use actual range if available, otherwise fall back to config value
	float gyro_fs = (gyro_actual_range > 0) ? gyro_actual_range : (float)CONFIG_SENSOR_GYRO_FS;
	printk("  Overall peak: %.2f deg/s (FS=%.0f)\n", (double)gyro_peak, (double)gyro_fs);
	if (gyro_peak > gyro_fs * 0.9f) {
		printk("  WARNING: Peak value exceeds 90%% of full scale!\n");
	}

	printk("\nAccelerometer (g):\n");
	printk("  X: min=%.3f, max=%.3f, peak=%.3f\n",
		(double)range_stats.accel_min[0],
		(double)range_stats.accel_max[0],
		(double)fmaxf(fabsf(range_stats.accel_min[0]), fabsf(range_stats.accel_max[0])));
	printk("  Y: min=%.3f, max=%.3f, peak=%.3f\n",
		(double)range_stats.accel_min[1],
		(double)range_stats.accel_max[1],
		(double)fmaxf(fabsf(range_stats.accel_min[1]), fabsf(range_stats.accel_max[1])));
	printk("  Z: min=%.3f, max=%.3f, peak=%.3f\n",
		(double)range_stats.accel_min[2],
		(double)range_stats.accel_max[2],
		(double)fmaxf(fabsf(range_stats.accel_min[2]), fabsf(range_stats.accel_max[2])));

	// Calculate overall peak accel value
	float accel_peak = 0;
	for (int i = 0; i < 3; i++) {
		float axis_peak = fmaxf(fabsf(range_stats.accel_min[i]), fabsf(range_stats.accel_max[i]));
		if (axis_peak > accel_peak) {
			accel_peak = axis_peak;
		}
	}
	// Use actual range if available, otherwise fall back to config value
	float accel_fs = (accel_actual_range > 0) ? accel_actual_range : (float)CONFIG_SENSOR_ACCEL_FS;
	printk("  Overall peak: %.3f g (FS=%.0f)\n", (double)accel_peak, (double)accel_fs);
	if (accel_peak > accel_fs * 0.9f) {
		printk("  WARNING: Peak value exceeds 90%% of full scale!\n");
	}

	printk("================================\n");
}
#endif // CONFIG_SENSOR_RANGE_STATS
