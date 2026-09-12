/* FIFO decoding derived from the ICM42686/ICM42688 drivers.
 * 01/14/2022 Copyright Tlera Corporation; created by Kris Winer.
 * Library may be used freely and without limit with attribution.
 */
#ifndef SLIMENRF_ICM426XX_HIRES_H
#define SLIMENRF_ICM426XX_HIRES_H

#include <stdint.h>
#include <string.h>

#define ICM426XX_HIRES_PACKET_SIZE 20

/* Family-private, no-I/O decoder. Scales apply to signed 20-bit samples
 * left-aligned in int32_t, not to right-aligned 20-bit counts.
 * Preserve the vtable contract: skipped packets leave outputs untouched;
 * an invalid vector in an otherwise usable packet becomes zero. The legacy
 * invalid sentinel compares only the upper 16 bits of all three axes.
 */
static inline int icm426xx_hires_decode(
	const uint8_t *packet, float accel_scale, float gyro_scale, float a[3], float g[3]
)
{
	static const uint8_t invalid[6] = {0x80, 0x00, 0x80, 0x00, 0x80, 0x00};

	if ((packet[0] & 0x80) == 0x80 || (packet[0] & 0x7F) == 0x7F) {
		return 1;
	}

	float a_raw[3] = {0};
	float g_raw[3] = {0};

	if (memcmp(&packet[1], invalid, sizeof(invalid))) {
		for (int i = 0; i < 3; i++) {
			a_raw[i] = (int32_t)((((uint32_t)packet[1 + (i * 2)]) << 24)
							 | (((uint32_t)packet[2 + (i * 2)]) << 16)
							 | (((uint32_t)packet[17 + i] & 0xF0) << 8));
		}
	}

	if (memcmp(&packet[7], invalid, sizeof(invalid))) {
		for (int i = 0; i < 3; i++) {
			g_raw[i] = (int32_t)((((uint32_t)packet[7 + (i * 2)]) << 24)
							 | (((uint32_t)packet[8 + (i * 2)]) << 16)
							 | (((uint32_t)packet[17 + i] & 0x0F) << 12));
		}
	} else if (!memcmp(&packet[1], invalid, sizeof(invalid))) {
		return 1;
	}

	for (int i = 0; i < 3; i++) {
		a_raw[i] *= accel_scale;
		g_raw[i] *= gyro_scale;
	}

	memcpy(a, a_raw, sizeof(a_raw));
	memcpy(g, g_raw, sizeof(g_raw));
	return 0;
}

#endif
