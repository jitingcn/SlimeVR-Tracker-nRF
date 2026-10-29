#ifndef SLIMENRF_LSM6DSX_SELECT_H
#define SLIMENRF_LSM6DSX_SELECT_H

#include <stdbool.h>
#include <stdint.h>

/* DSM and DSO share these range encodings and physical sensitivities. */
static inline uint8_t lsm6dsx_accel_range(float *range)
{
	if (*range > 8) { *range = 16; return 0x04; }
	if (*range > 4) { *range = 8; return 0x0c; }
	if (*range > 2) { *range = 4; return 0x08; }
	*range = 2;
	return 0;
}

static inline uint8_t lsm6dsx_gyro_range(float *range)
{
	if (*range > 1000) { *range = 2000; return 0x0c; }
	if (*range > 500) { *range = 1000; return 0x08; }
	if (*range > 250) { *range = 500; return 0x04; }
	*range = 250;
	return 0;
}

/* Only finite, active rates enter here. The caller owns frequency trim, integer
 * requested-Hz rounding, off/sleep policy and the fastest gyro period rounding. */
static inline uint8_t lsm6dsx_select_odr(float *period, int requested_hz)
{
	if (*period < 0.3f / 1000) { *period = 0.15 / 1000; return 0xa0; }
	if (*period < 0.6f / 1000) { *period = 0.3 / 1000; return 0x90; }
	if (*period < 1.2f / 1000) { *period = 0.6 / 1000; return 0x80; }
	if (*period < 2.4f / 1000) { *period = 1.2 / 1000; return 0x70; }
	if (*period < 4.8f / 1000) { *period = 2.4 / 1000; return 0x60; }
	if (*period < 9.6f / 1000) { *period = 4.8 / 1000; return 0x50; }
	if (*period < 19.2f / 1000) { *period = 9.6 / 1000; return 0x40; }
	if (*period < 38.4f / 1000) { *period = 19.2 / 1000; return 0x30; }
	if (requested_hz > 12.5) { *period = 38.4 / 1000; return 0x20; }
	*period = 1.0 / 12.5;
	return 0x10;
}

#endif
