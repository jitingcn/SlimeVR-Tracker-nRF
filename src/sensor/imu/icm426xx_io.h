/* Private ICM42686/ICM42688 bank-0 I/O algorithms. Chip policy stays in the drivers. */
#ifndef ICM426XX_IO_H
#define ICM426XX_IO_H

#include "icm426xx_hires.h"

static inline uint16_t icm426xx_fifo_read(
	uint8_t count_reg, uint8_t data_reg, float multiplier,
	uint8_t *data, uint16_t capacity_bytes, float *temperature, bool *temperature_valid)
{
	*temperature_valid = false;
	uint16_t total_packets = 0;
	while (capacity_bytes >= ICM426XX_HIRES_PACKET_SIZE) {
		uint8_t raw_count[2];
		int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, count_reg, raw_count, 2);
		if (err) {
			*temperature_valid = false;
			LOG_ERR("Failed to read FIFO count");
			return total_packets;
		}
		uint16_t packet_count = (uint16_t)(raw_count[0] << 8 | raw_count[1]);
		if (!packet_count) {
			break;
		}
		uint16_t packet_capacity = capacity_bytes / ICM426XX_HIRES_PACKET_SIZE;
		float requested_packets = packet_count + packet_count * multiplier;
		if (requested_packets > packet_capacity) {
			LOG_WRN("FIFO read buffer limit reached");
			packet_count = packet_capacity;
		} else {
			packet_count = (uint16_t)requested_packets;
		}
		uint16_t byte_count = packet_count * ICM426XX_HIRES_PACKET_SIZE;
		err = ssi_burst_read_interval(SENSOR_INTERFACE_DEV_IMU, data_reg, data, byte_count,
			ICM426XX_HIRES_PACKET_SIZE);
		if (err) {
			*temperature_valid = false;
			LOG_ERR("Communication error");
			return total_packets;
		}
		if (!icm426xx_hires_temperature(data, packet_count, temperature)) {
			*temperature_valid = true;
		}
		data += byte_count;
		capacity_bytes -= byte_count;
		total_packets += packet_count;
	}
	return total_packets;
}

static inline void icm426xx_vector_read(uint8_t reg, float sensitivity, float out[3])
{
	uint8_t raw[6];
	if (ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, reg, raw, sizeof(raw))) {
		LOG_ERR("Communication error");
		memset(out, 0, 3 * sizeof(*out));
		return;
	}
	for (int i = 0; i < 3; i++) {
		out[i] = (int16_t)((uint16_t)raw[2 * i] << 8 | raw[2 * i + 1]);
		out[i] *= sensitivity;
	}
}

static inline float icm426xx_temp_read(uint8_t reg, float temperature, bool temperature_valid)
{
	if (temperature_valid) {
		return temperature;
	}
	uint8_t raw[2];
	if (ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, reg, raw, sizeof(raw))) {
		LOG_ERR("Communication error");
		return NAN;
	}
	return (int16_t)((uint16_t)raw[0] << 8 | raw[1]) / 132.48f + 25;
}

static inline uint8_t icm426xx_setup_DRDY(uint8_t watermark_reg, uint8_t source_reg, uint16_t threshold)
{
	uint8_t buf[2] = {threshold & 0xFF, (threshold >> 8) & 0x0F};
	int err = ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, watermark_reg, buf, sizeof(buf));
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, source_reg, 0x04);
	if (err) {
		LOG_ERR("Communication error");
	}
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW;
}

#endif
