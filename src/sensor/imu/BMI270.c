#include <math.h>

#include <zephyr/logging/log.h>
#include <hal/nrf_gpio.h>

#include "BMI270.h"
#include "BMI270_firmware.h"
#include "sensor/sensor_none.h"

#define PACKET_SIZE 12 // Headerless FIFO: gyro XYZ, then accel XYZ; little-endian 16-bit axes.
#define BMI270_PWR_CTRL_GYRO_EN 0x02
#define BMI270_PWR_CTRL_ACCEL_EN 0x04
#define BMI270_PWR_CTRL_TEMP_EN 0x08

static float accel_sensitivity = 16.0f / 32768.0f;  // default 16g
static float gyro_sensitivity = 2000.0f / 32768.0f; // default 2000dps

static uint8_t accel_fs = RANGE_16G;
static uint8_t gyro_fs = RANGE_2000;

static const float accel_odr_hz[]
	= {1600.0f, 800.0f, 400.0f, 200.0f, 100.0f, 50.0f, 25.0f, 12.5f, 6.25f, 3.125f, 1.5625f, 0.78125f};
static const uint8_t accel_odrs[]
	= {ODR_1k6, ODR_800, ODR_400, ODR_200, ODR_100, ODR_50, ODR_25, ODR_12p5, ODR_6p25, ODR_3p1, ODR_1p5, ODR_0p78};
static const float gyro_odr_hz[] = {3200.0f, 1600.0f, 800.0f, 400.0f, 200.0f, 100.0f, 50.0f, 25.0f};
static const uint8_t gyro_odrs[] = {ODR_3k2, ODR_1k6, ODR_800, ODR_400, ODR_200, ODR_100, ODR_50, ODR_25};

static uint8_t last_accel_odr = 0xff;
static uint8_t last_gyro_odr = 0xff;
static float factor_zx;

LOG_MODULE_REGISTER(BMI270, LOG_LEVEL_DBG);

static int asic_init(void);
static int upload_config_file(void);
static float factor_zx_read(void);

int bmi_init(
	float clock_rate,
	float accel_period_s,
	float gyro_period_s,
	float *accel_actual_period_s,
	float *gyro_actual_period_s
)
{
	ARG_UNUSED(clock_rate);
	// setup interface for SPI
	sensor_interface_spi_configure(SENSOR_INTERFACE_DEV_IMU, MHZ(10), 1);
	last_accel_odr = 0xff; // reset before bus work so fail paths never lie
	last_gyro_odr = 0xff;
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_PWR_CONF, 0x00); // disable adv_power_save
	k_usleep(450);
	if (asic_init()) {
		return -1;
	}
	factor_zx = factor_zx_read();
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_ACC_RANGE, accel_fs);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_GYR_RANGE, gyro_fs);
	err |= bmi_update_odr(accel_period_s, gyro_period_s, accel_actual_period_s, gyro_actual_period_s);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_FIFO_CONFIG_0, 0x00); // do not return sensortime frame
	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		BMI270_FIFO_CONFIG_1,
		0xC0
	); // enable a+g data in FIFO, don't store header
	if (err) {
		LOG_ERR("Communication error");
	}
	return (err < 0 ? err : 0);
}

void bmi_shutdown(void) // this does not reset the device, to avoid clearing the config
{
	last_accel_odr = 0xff;                                                         // reset last odr
	last_gyro_odr = 0xff;                                                          // reset last odr
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_PWR_CTRL, 0x00); // disable all sensors
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_PWR_CONF, 0x01);    // enable adv_power_save (suspend)
	if (err) {
		LOG_ERR("Communication error");
	}
}

void bmi_update_fs(float accel_range, float gyro_range, float *accel_actual_range, float *gyro_actual_range)
{
	if (accel_range > 8) {
		accel_fs = RANGE_16G;
		accel_range = 16;
	} else if (accel_range > 4) {
		accel_fs = RANGE_8G;
		accel_range = 8;
	} else if (accel_range > 2) {
		accel_fs = RANGE_4G;
		accel_range = 4;
	} else {
		accel_fs = RANGE_2G;
		accel_range = 2;
	}

	if (gyro_range > 1000) {
		gyro_fs = RANGE_2000;
		gyro_range = 2000;
	} else if (gyro_range > 500) {
		gyro_fs = RANGE_1000;
		gyro_range = 1000;
	} else if (gyro_range > 250) {
		gyro_fs = RANGE_500;
		gyro_range = 500;
	} else {
		gyro_fs = RANGE_250;
		gyro_range = 250;
	}

	accel_sensitivity = accel_range / 32768.0f;
	gyro_sensitivity = gyro_range / 32768.0f;

	*accel_actual_range = accel_range;
	*gyro_actual_range = gyro_range;
}

int bmi_update_odr(float accel_period_s, float gyro_period_s, float *accel_actual_period_s, float *gyro_actual_period_s)
{
	float requested_odr_hz;
	uint8_t acc_odr = 0;
	uint8_t gyr_odr = 0;

	// Calculate accel
	if (accel_period_s <= 0 || accel_period_s == INFINITY) // off, standby interpreted as off
	{
		accel_period_s = 0; // off
	} else {
		requested_odr_hz = 1.0f / accel_period_s;
		size_t selected = 0;
		for (size_t i = 1; i < ARRAY_SIZE(accel_odr_hz); i++) {
			if (requested_odr_hz > accel_odr_hz[i]) {
				break;
			}
			selected = i;
		}
		acc_odr = accel_odrs[selected];
		accel_period_s = 1.0f / accel_odr_hz[selected];
	}

	// Calculate gyro
	if (gyro_period_s <= 0 || gyro_period_s == INFINITY) // off, standby interpreted as off
	{
		gyro_period_s = 0; // off
	} else {
		requested_odr_hz = 1.0f / gyro_period_s;
		size_t selected = 0;
		for (size_t i = 1; i < ARRAY_SIZE(gyro_odr_hz); i++) {
			if (requested_odr_hz > gyro_odr_hz[i]) {
				break;
			}
			selected = i;
		}
		gyr_odr = gyro_odrs[selected];
		gyro_period_s = 1.0f / gyro_odr_hz[selected];
	}

	if (last_accel_odr == acc_odr && last_gyro_odr == gyr_odr) {
		*accel_actual_period_s = accel_period_s;
		*gyro_actual_period_s = gyro_period_s;
		return 0; /* already configured — success for err|= callers */
	}

	int err = 0;
	if (acc_odr != 0) {
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_ACC_CONF, 0xA0 | acc_odr);
	}
	if (gyr_odr != 0) {
		err |= ssi_reg_write_byte(
			SENSOR_INTERFACE_DEV_IMU,
			BMI270_GYR_CONF,
			0xE0 | gyr_odr
		); // set performance opt. noise performance
	}

	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		BMI270_PWR_CTRL,
		BMI270_PWR_CTRL_TEMP_EN | (acc_odr != 0 ? BMI270_PWR_CTRL_ACCEL_EN : 0)
			| (gyr_odr != 0 ? BMI270_PWR_CTRL_GYRO_EN : 0)
	); // enable temp, set accel and gyro power
	if (err) {
		last_accel_odr = 0xff;
		last_gyro_odr = 0xff;
		LOG_ERR("Communication error");
		return err;
	}

	last_accel_odr = acc_odr;
	last_gyro_odr = gyr_odr;
	*accel_actual_period_s = accel_period_s;
	*gyro_actual_period_s = gyro_period_s;

	return 0;
}

// Historical gyro/accel latency difference has no verified cause here.
uint16_t bmi_fifo_read(uint8_t *data, uint16_t capacity_bytes)
{
	uint16_t total_packets = 0;
	uint16_t packet_count = UINT16_MAX;
	while (packet_count > 0 && capacity_bytes >= PACKET_SIZE) {
		uint8_t raw_count[2];
		int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, BMI270_FIFO_LENGTH_0, &raw_count[0], 2);
		if (err) {
			LOG_ERR("Failed to read FIFO count");
			return total_packets;
		}
		uint16_t byte_count = (uint16_t)((raw_count[1] & 0x3F) << 8 | raw_count[0]); // FIFO byte count.
		if (!byte_count) {                                                           // nothing to do
			break;
		}
		packet_count = byte_count / PACKET_SIZE;
		uint16_t packet_capacity = capacity_bytes / PACKET_SIZE;
		if (packet_count > packet_capacity) {
			LOG_WRN("FIFO read buffer limit reached, %d packets dropped", packet_count - packet_capacity);
			packet_count = packet_capacity;
			byte_count = packet_count * PACKET_SIZE;
		}
		err = ssi_burst_read_interval(SENSOR_INTERFACE_DEV_IMU, BMI270_FIFO_DATA, data, byte_count, PACKET_SIZE);
		if (err) {
			LOG_ERR("Communication error");
			return total_packets;
		}
		data += packet_count * PACKET_SIZE;
		capacity_bytes -= packet_count * PACKET_SIZE;
		total_packets += packet_count;
	}
	return total_packets;
}

static const uint8_t overread[2] = {0x00, 0x80};
static const uint8_t invalid_accel[6] = {0x01, 0x7F, 0x00, 0x80, 0x00, 0x80};
static const uint8_t invalid_gyro[6] = {0x02, 0x7F, 0x00, 0x80, 0x00, 0x80};

int bmi_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3])
{
	const uint16_t packet_offset = index * PACKET_SIZE;
	const uint8_t *packet = &data[packet_offset];
	if (!memcmp(packet, overread, sizeof(overread))) {
		return 1; // Skip overread packets
	}
	float a_bmi[3];
	float g_bmi[3];
	for (int i = 0; i < 3; i++) // x, y, z
	{
		a_bmi[i] = (int16_t)((((uint16_t)packet[7 + (i * 2)]) << 8) | packet[6 + (i * 2)]);
		a_bmi[i] *= accel_sensitivity;
		g_bmi[i] = (int16_t)((((uint16_t)packet[1 + (i * 2)]) << 8) | packet[i * 2]);
		g_bmi[i] *= gyro_sensitivity;
	}
	if (memcmp(&packet[6], invalid_accel, sizeof(invalid_accel))) // Invalid channels leave their output unchanged.
	{
		a[0] = -a_bmi[1];
		a[1] = a_bmi[0];
		a[2] = a_bmi[2];
	}
	if (memcmp(packet, invalid_gyro, sizeof(invalid_gyro))) // valid gyro data
	{
		// factor_zx already includes the GYR_CAS coefficient's 1/512 scale.
		g_bmi[0] -= g_bmi[2] * factor_zx;
		g[0] = -g_bmi[1];
		g[1] = g_bmi[0];
		g[2] = g_bmi[2];
	}
	return 0;
}

void bmi_accel_read(float a[3])
{
	uint8_t raw_accel[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, BMI270_DATA_8, &raw_accel[0], 6);
	if (err) {
		LOG_ERR("Communication error");
		memset(a, 0, 3 * sizeof(*a));
		return;
	}
	float a_bmi[3];
	for (int i = 0; i < 3; i++) // x, y, z
	{
		a_bmi[i] = (int16_t)((((uint16_t)raw_accel[1 + (i * 2)]) << 8) | raw_accel[i * 2]);
		a_bmi[i] *= accel_sensitivity;
	}
	a[0] = -a_bmi[1];
	a[1] = a_bmi[0];
	a[2] = a_bmi[2];
}

void bmi_gyro_read(float g[3])
{
	uint8_t raw_gyro[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, BMI270_DATA_14, &raw_gyro[0], 6);
	if (err) {
		LOG_ERR("Communication error");
		memset(g, 0, 3 * sizeof(*g));
		return;
	}
	float g_bmi[3];
	for (int i = 0; i < 3; i++) // x, y, z
	{
		g_bmi[i] = (int16_t)((((uint16_t)raw_gyro[1 + (i * 2)]) << 8) | raw_gyro[i * 2]);
		g_bmi[i] *= gyro_sensitivity;
	}
	// Ratex = DATA_15<<8+DATA_14 - GYR_CAS.factor_zx * (DATA_19<<8+DATA_18) / 2^9
	g_bmi[0] -= g_bmi[2] * factor_zx;
	g[0] = -g_bmi[1];
	g[1] = g_bmi[0];
	g[2] = g_bmi[2];
}

float bmi_temp_read(void)
{
	uint8_t raw_temp[2];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, BMI270_TEMPERATURE_0, &raw_temp[0], 2);
	if (err) {
		LOG_ERR("Communication error");
		return NAN;
	}
	if (raw_temp[0] == 0x00 && raw_temp[1] == 0x80) {
		return NAN;
	}
	// 0x0000 -> 23°C
	// The resolution is 1/2^9 K/LSB
	float temp = (int16_t)((((uint16_t)raw_temp[1]) << 8) | raw_temp[0]);
	temp /= 512;
	temp += 23;
	return temp;
}

uint8_t bmi_setup_DRDY(uint16_t threshold)
{
	uint8_t buf[2];
	threshold *= PACKET_SIZE; // byte threshold
	buf[0] = threshold & 0xFF;
	buf[1] = (threshold >> 8) & 0x1F;
	int err = ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, BMI270_FIFO_WTM_0, buf, 2);
	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		BMI270_INT1_MAP_FEAT,
		0x00
	); // disable any_motion_out (interrupt)
	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		BMI270_INT1_IO_CTRL,
		0x0C
	); // set INT1 active low, open-drain, output enabled
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_INT_MAP_DATA, 0x02); // FIFO threshold interrupt
	if (err) {
		LOG_ERR("Communication error");
	}
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
}

uint8_t bmi_setup_WOM(void) // Historical sensitivity/compatibility concerns remain unverified.
{
	uint8_t config[4] = {0};
	uint16_t *ptr = (uint16_t *)config;     // bmi is little endian
	ptr[0] = 0x7 << 13 | 0x000;             // enable all axes, set detection duration to 0
	ptr[1] = 0x1 << 15 | 0x7 << 11 | 0x040; // enable any_motion, set out_conf to bit 6, set threshold (1LSB equals to
											// 0.488mg, 64 * 0.488mg is ~31.25mg)
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_PWR_CONF, 0x00); // disable adv_power_save
	k_usleep(450);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_ACC_CONF, ODR_200); // disable filters, set accel ODR
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_PWR_CTRL, 0x04);    // enable accel
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_FEAT_PAGE, 0x01);   // go to page 1
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, BMI270_ANYMO_1, config, sizeof(config)); // Start write buffer
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_INT_MAP_DATA, 0x00); // disable FIFO threshold interrupt
	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		BMI270_INT1_IO_CTRL,
		0x0C
	);            // set INT1 active low, open-drain, output enabled
	k_msleep(55); // wait for sensor to settle
	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		BMI270_INT1_MAP_FEAT,
		0x40
	);                                                                          // enable any_motion_out (interrupt)
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_PWR_CONF, 0x01); // enable adv_power_save (suspend)
	if (err) {
		LOG_ERR("Communication error");
	}
	// LOG_DBG("WOM setup complete");
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
}

static int asic_init(void)
{
	uint8_t status = 0;
	int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_INTERNAL_STATUS, &status);
	if (err) {
		LOG_ERR("Communication error");
		return err;
	}
	if ((status & 0x7) != 0x1) // ASIC is not initialized
	{
		LOG_DBG("ASIC not initialized, uploading config file");
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_CMD, 0xB6); // softreset
		k_msleep(2);
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_PWR_CONF, 0x00); // disable adv_power_save
		k_usleep(450);
		err |= upload_config_file();
		if (err) {
			LOG_ERR("Communication error");
			return err;
		}
		int retry_count = 0;
		while ((status & 0x7) != 0x1) {
			if (retry_count > 100) {
				LOG_ERR("Failed to initialize ASIC");
				return -1;
			};
			k_msleep(1);
			err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_INTERNAL_STATUS, &status);
			// LOG_DBG("Status: 0x%02X", status);
			retry_count++;
		}
		if (err) {
			LOG_ERR("Communication error");
			return err;
		}
		LOG_DBG("ASIC initialized");
	}
	return 0;
}

/* Config upload adapted from Zephyr BMI270 driver. */
static int upload_config_file(void)
{
	uint16_t byte_count = sizeof(bmi270_config_file) / sizeof(bmi270_config_file[0]);
	uint8_t init_addr[2] = {0};
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_INIT_CTRL, 0x00); // prepare config load
	for (int i = 0; i < byte_count; i += 64) {
		init_addr[0] = (i / 2) & 0xF;
		init_addr[1] = (i / 2) >> 4;
		err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, BMI270_INIT_ADDR_0, init_addr, 2);
		err |= ssi_burst_write(
			SENSOR_INTERFACE_DEV_IMU,
			BMI270_INIT_DATA,
			&bmi270_config_file[i],
			64
		); // Preserve 64-byte chunks; the historical larger-transfer limitation is unverified.
	}
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_INIT_CTRL, 0x01); // complete config load
	if (err) {
		LOG_ERR("Communication error");
		return err;
	}
	// LOG_DBG("Completed config load");
	return 0;
}

static float factor_zx_read(void)
{
	uint8_t data;
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_FEAT_PAGE, 0x00); // go to page 0
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_GYR_CAS, &data);
	if (err) {
		LOG_ERR("Communication error");
		return 0.0f;
	}
	// GYR_CAS.factor_zx is a 7-bit two-complement encoded signed value
	data <<= 1; // shift to 8-bit
	// Ratex = DATA_15<<8+DATA_14 - GYR_CAS.factor_zx * (DATA_19<<8+DATA_18) / 2^9
	return (int8_t)data / 2 / 512.0f;
}

// from https://github.com/SlimeVR/SlimeVR-Tracker-ESP/blob/main/src/sensors/softfusion/drivers/bmi270.h
int bmi_crt(uint8_t *data)
{
	uint8_t status;
	uint8_t acc_odr = last_accel_odr; // store last odr
	uint8_t gyr_odr = last_gyro_odr;  // store last odr
	uint8_t config[2] = {0};
	uint16_t *ptr = (uint16_t *)config;                                       // bmi is little endian
	*ptr = 0x0100;                                                            // CRT will be executed
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_CMD, 0xB6); // softreset
	k_msleep(2);
	// in case of SPI, where CS pin must trigger rising edge for BMI to enable interface
	uint8_t tmp;
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, 0x00, &tmp);
	k_usleep(200);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_PWR_CONF, 0x00); // disable adv_power_save
	k_usleep(450);
	if (asic_init()) {
		/* softreset already ran; do not claim prior ODR still applied */
		last_accel_odr = 0xff;
		last_gyro_odr = 0xff;
		return -1;
	}

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_OFFSET_6, 0x80); // gyr_gain_en
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_PWR_CTRL, 0x04); // enable accel
	k_msleep(2);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_GYR_CRT_CONF, 0x04);            // set CRT running
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_FEAT_PAGE, 0x01);               // go to page 1
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, BMI270_G_TRIG_1, config, sizeof(config)); // Start write buffer
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_CMD, 0x02);                     // g_trigger
	do {
		k_msleep(200);
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_GYR_CRT_CONF, &status);
	} while (status == 0x04);

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_FEAT_PAGE, 0x00); // go to page 0
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_GYR_GAIN_STATUS, &status);
	if (status) {
		LOG_INF("Execution status: 0x%02X", status);
		if (status == 0x03) {
			LOG_INF("Motion detected during CRT");
		}
		data[0] = 0; // invalid
	} else {
		err |= ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, BMI270_GYR_USR_GAIN, data + 1, 3);
		LOG_INF("Gain: 0x%02X, 0x%02X, 0x%02X", data[1], data[2], data[3]);
		data[0] = 1; // flag for valid gain
	}

	// CRT seems to leave some state behind which isn't persisted after
	// restart. If we continue without restarting, the gyroscope will behave
	// differently on this run compared to subsequent restarts.
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_CMD, 0xB6); // softreset
	k_msleep(2);
	// in case of SPI, where CS pin must trigger rising edge for BMI to enable interface
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, 0x00, &tmp);
	k_usleep(200);
	float accel_actual_period_s, gyro_actual_period_s;
	err |= bmi_init(0, 0, 0, &accel_actual_period_s, &gyro_actual_period_s);
	if (acc_odr != 0) {
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_ACC_CONF, 0xA0 | acc_odr);
	}
	if (gyr_odr != 0) {
		err |= ssi_reg_write_byte(
			SENSOR_INTERFACE_DEV_IMU,
			BMI270_GYR_CONF,
			0xE0 | gyr_odr
		); // set performance opt. noise performance
	}
	err |= ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_IMU,
		BMI270_PWR_CTRL,
		0x08 | (acc_odr != 0 ? 0x04 : 0) | (gyr_odr != 0 ? 0x02 : 0)
	); // enable temp, set accel and gyro power
	if (err) {
		LOG_ERR("Communication error");
		last_accel_odr = 0xff;
		last_gyro_odr = 0xff;
		return err;
	} else {
		last_accel_odr = acc_odr;
		last_gyro_odr = gyr_odr;
	}

	if (data[0] == 0) {
		return 1;
	}
	return 0;
}

void bmi_gain_apply(uint8_t *data)
{
	if (data[0] == 1) // flag for valid gain
	{
		int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_OFFSET_6, 0x80); // gyr_gain_en
		err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, BMI270_GYR_USR_GAIN, data + 1, 3);
		if (err) {
			LOG_ERR("Communication error");
		}
	}
}

const sensor_imu_t sensor_imu_bmi270
	= {*bmi_init,
	   *bmi_shutdown,

	   *bmi_update_fs,
	   *bmi_update_odr,

	   *bmi_fifo_read,
	   *bmi_fifo_process,
	   *bmi_accel_read,
	   *bmi_gyro_read,
	   *bmi_temp_read,

	   *bmi_setup_DRDY,
	   *bmi_setup_WOM,

	   *imu_none_ext_setup};
