#ifndef SLIMENRF_ICM426XX_ODR_H
#define SLIMENRF_ICM426XX_ODR_H

#include <stddef.h>
#include <stdint.h>

/* Both models use these ODR encodings. Power modes and clock scaling stay local. */
static inline uint8_t icm426xx_select_odr(float requested_hz, float *period_s)
{
	static const struct { float hz; uint8_t code; } rates[] = {
		{32000, 0x01}, {16000, 0x02}, {8000, 0x03}, {4000, 0x04},
		{2000, 0x05}, {1000, 0x06}, {500, 0x0f}, {200, 0x07},
		{100, 0x08}, {50, 0x09}, {25, 0x0a}, {12.5f, 0x0b},
	};
	size_t selected = 0;
	for (size_t i = 1; i < sizeof(rates) / sizeof(rates[0]); i++) {
		if (requested_hz > rates[i].hz) {
			break;
		}
		selected = i;
	}
	*period_s = 1.0f / rates[selected].hz;
	return rates[selected].code;
}

#endif
