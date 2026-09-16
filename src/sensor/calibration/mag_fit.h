/* SPDX-License-Identifier: MIT */
#ifndef SLIMENRF_MAG_FIT_H
#define SLIMENRF_MAG_FIT_H
#include <stdbool.h>
#include <stdint.h>

typedef bool (*mag_fit_read_fn)(void *ctx, unsigned index, float out[3]);
typedef bool (*mag_fit_poll_fn)(void *ctx);
struct mag_fit_result {
	float field_norm;
	float condition;
	float rms;
};

/* Exclusively owned by the calibration thread. Manual accumulation and online
 * fitting are disjoint phases; a cancelled fit leaves the workspace unspecified. */
union mag_cal_workspace {
	double ata[100];
	struct {
		double information[81];
		double step[9]; /* gradient during accumulation, solution after solve */
		uint16_t cells[24];
	} fit;
};
_Static_assert(sizeof(union mag_cal_workspace) <= 800, "calibration scratch budget");
extern union mag_cal_workspace mag_cal_workspace;

/* read=false skips a slot; poll=false cancels. Output/result unchanged on error.
 * Successful matrices always target radius 0.5 (result.field_norm), including
 * bootstrap with previous=NULL and field_norm=0. Otherwise field_norm is the
 * previous model's positive target radius; its matrix is rescaled before fitting.
 * Only the calibration thread may call this function. */
int magneto_robust_fit(
	unsigned slots,
	mag_fit_read_fn read_sample,
	mag_fit_poll_fn poll,
	void *ctx,
	const float previous[4][3],
	float field_norm,
	float output[4][3],
	struct mag_fit_result *result
);
#endif
