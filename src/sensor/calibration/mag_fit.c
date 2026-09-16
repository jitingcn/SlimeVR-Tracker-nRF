/* SPDX-License-Identifier: MIT */
#include "mag_fit.h"
#include "sensor/magneto/magneto1_4.h"
#include <errno.h>
#include <float.h>
#include <math.h>
#include <string.h>

union mag_cal_workspace mag_cal_workspace;
#define W (mag_cal_workspace.fit)
#define HUBER 0.06f

static bool finite3(const float x[3])
{
	return isfinite(x[0]) && isfinite(x[1]) && isfinite(x[2]);
}
static float norm3(const float x[3])
{
	return sqrtf(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
}
static bool polling(mag_fit_poll_fn poll, void *ctx, unsigned i)
{
	return (i % 16) != 0 || !poll || poll(ctx);
}
/* Symmetric Jacobi: bounded cyclic sweeps, eigenvectors are not required. */
static bool eigen(double *a, unsigned n, mag_fit_poll_fn poll, void *ctx, double *lo, double *hi)
{
	for (unsigned sweep = 0; sweep < 8; sweep++) {
		if (poll && !poll(ctx)) {
			return false;
		}
		for (unsigned p = 0; p < n; p++) {
			for (unsigned q = p + 1; q < n; q++) {
				double apq = a[p * n + q];
				if (fabs(apq) < 1e-12) {
					continue;
				}
				double tau = (a[q * n + q] - a[p * n + p]) / (2 * apq);
				double t = copysign(1.0, tau) / (fabs(tau) + hypot(1, tau));
				double c = 1 / sqrt(1 + t * t), s = t * c;
				a[p * n + p] -= t * apq;
				a[q * n + q] += t * apq;
				a[p * n + q] = a[q * n + p] = 0;
				for (unsigned k = 0; k < n; k++) {
					if (k != p && k != q) {
						double x = a[k * n + p], y = a[k * n + q];
						a[k * n + p] = a[p * n + k] = c * x - s * y;
						a[k * n + q] = a[q * n + k] = s * x + c * y;
					}
				}
			}
		}
	}
	*lo = DBL_MAX;
	*hi = 0;
	for (unsigned i = 0; i < n; i++) {
		/* Gershgorin residual bounds make incomplete convergence conservative. */
		double radius = 0;
		for (unsigned j = 0; j < n; j++) {
			if (j != i) {
				radius += fabs(a[i * n + j]);
			}
		}
		*lo = fmin(*lo, a[i * n + i] - radius);
		*hi = fmax(*hi, a[i * n + i] + radius);
	}
	return isfinite(*lo) && isfinite(*hi);
}
static void matrix(const float p[9], float a[9])
{
	a[0] = p[3];
	a[4] = p[4];
	a[8] = p[5];
	a[1] = a[3] = p[6];
	a[2] = a[6] = p[7];
	a[5] = a[7] = p[8];
}
static bool constrained(const float p[9], const float *initial)
{
	for (unsigned i = 0; i < 9; i++) {
		if (!isfinite(p[i])) {
			return false;
		}
	}
	if (norm3(p) > 4) {
		return false;
	}
	double a[9], lo, hi;
	a[0] = p[3];
	a[4] = p[4];
	a[8] = p[5];
	a[1] = a[3] = p[6];
	a[2] = a[6] = p[7];
	a[5] = a[7] = p[8];
	if (!eigen(a, 3, NULL, NULL, &lo, &hi) || lo < .25f || hi > 4 || hi / lo > 8) {
		return false;
	}
	if (initial) {
		float db = 0, da = 0, base = 0;
		for (unsigned i = 0; i < 9; i++) {
			float d = p[i] - initial[i], weight = i >= 6 ? 2 : 1;
			if (i < 3) {
				db += d * d;
			} else {
				da += weight * d * d;
				base += weight * initial[i] * initial[i];
			}
		}
		if (db > .25f * .25f || da > .35f * .35f * base) {
			return false;
		}
	}
	return true;
}
static unsigned cell(const float y[3])
{
	unsigned axis = 0;
	if (fabsf(y[1]) > fabsf(y[axis])) {
		axis = 1;
	}
	if (fabsf(y[2]) > fabsf(y[axis])) {
		axis = 2;
	}
	return axis * 8 + (y[0] >= 0 ? 1 : 0) + (y[1] >= 0 ? 2 : 0) + (y[2] >= 0 ? 4 : 0);
}
static bool residual(const float raw[3], float h, const float p[9], float *r, float j[9], unsigned *bin)
{
	float x[3], y[3];
	for (unsigned i = 0; i < 3; i++) {
		x[i] = raw[i] / h - p[i];
	}
	y[0] = p[3] * x[0] + p[6] * x[1] + p[7] * x[2];
	y[1] = p[6] * x[0] + p[4] * x[1] + p[8] * x[2];
	y[2] = p[7] * x[0] + p[8] * x[1] + p[5] * x[2];
	float n = norm3(y);
	if (!isfinite(n) || n < 1e-6f) {
		return false;
	}
	*bin = cell(y);
	*r = n - 1;
	for (unsigned i = 0; i < 3; i++) {
		y[i] /= n;
	}
	j[0] = -(p[3] * y[0] + p[6] * y[1] + p[7] * y[2]);
	j[1] = -(p[6] * y[0] + p[4] * y[1] + p[8] * y[2]);
	j[2] = -(p[7] * y[0] + p[8] * y[1] + p[5] * y[2]);
	j[3] = y[0] * x[0];
	j[4] = y[1] * x[1];
	j[5] = y[2] * x[2];
	j[6] = y[0] * x[1] + y[1] * x[0];
	j[7] = y[0] * x[2] + y[2] * x[0];
	j[8] = y[1] * x[2] + y[2] * x[1];
	return true;
}
/* Cell membership/counts are anchored to the iteration's initial model. This
 * keeps line-search objectives comparable rather than moving their weights. */
static int accumulate(
	unsigned slots,
	mag_fit_read_fn read,
	mag_fit_poll_fn poll,
	void *ctx,
	float h,
	const float p[9],
	const float anchor[9],
	bool normal,
	float *loss,
	float *rms,
	float *inliers
)
{
	double total = 0, cost = 0, squares = 0, good = 0;
	if (normal) {
		memset(W.information, 0, sizeof(W.information));
		memset(W.step, 0, sizeof(W.step));
	}
	for (unsigned i = 0; i < slots; i++) {
		if (!polling(poll, ctx, i)) {
			return -ECANCELED;
		}
		float raw[3], r, j[9], ar, aj[9];
		unsigned bin, ab;
		if (!read(ctx, i, raw)) {
			continue;
		}
		if (!finite3(raw) || !residual(raw, h, p, &r, j, &bin) || !residual(raw, h, anchor, &ar, aj, &ab)) {
			return -EDOM;
		}
		/* Huber alone does not bound gain-parameter leverage (J contains raw
		 * coordinates). Anchor this radial leverage cap during line search. */
		float leverage = fmaxf(1.0f, (ar + 1) * (ar + 1));
		double w = 1.0 / (W.cells[ab] * (double)leverage);
		float e = fabsf(r);
		total += w;
		cost += w * (e <= HUBER ? .5f * r * r : HUBER * (e - .5f * HUBER));
		float clipped = fminf(e, .18f);
		squares += w * clipped * clipped;
		if (e <= .18f) {
			good += w;
		}
		if (!normal) {
			continue;
		}
		w *= e > HUBER ? HUBER / e : 1;
		for (unsigned a = 0; a < 9; a++) {
			W.step[a] += w * j[a] * r;
			for (unsigned b = 0; b < 9; b++) {
				W.information[a * 9 + b] += w * j[a] * j[b];
			}
		}
	}
	if (!(total > 0) || !isfinite(cost)) {
		return -EDOM;
	}
	*loss = cost / total;
	*rms = sqrt(squares / total);
	*inliers = good / total;
	if (normal) {
		for (unsigned a = 0; a < 9; a++) {
			W.step[a] /= total;
			for (unsigned b = 0; b < 9; b++) {
				W.information[a * 9 + b] /= total;
			}
		}
	}
	return 0;
}
static int directions(unsigned slots, mag_fit_read_fn read, mag_fit_poll_fn poll, void *ctx, float h, const float p[9])
{
	memset(W.cells, 0, sizeof(W.cells));
	unsigned count = 0, occupied = 0;
	for (unsigned i = 0; i < slots; i++) {
		if (!polling(poll, ctx, i)) {
			return -ECANCELED;
		}
		float raw[3], r, j[9];
		unsigned bin;
		if (!read(ctx, i, raw)) {
			continue;
		}
		if (!finite3(raw) || !residual(raw, h, p, &r, j, &bin)) {
			return -EDOM;
		}
		if (!W.cells[bin]++) {
			occupied++;
		}
		count++;
	}
	return count >= 48 && occupied >= 12 ? 0 : -EDOM;
}
static bool solve(void)
{
	/* Rank was checked first. Rebuilt information is factored in place;
	 * the gradient is overwritten by forward/back substitution. */
	for (unsigned i = 0; i < 9; i++) {
		W.information[i * 9 + i] += 1e-5;
		for (unsigned j = 0; j <= i; j++) {
			double s = W.information[i * 9 + j];
			for (unsigned k = 0; k < j; k++) {
				s -= W.information[i * 9 + k] * W.information[j * 9 + k];
			}
			if (i == j) {
				if (!(s > 0)) {
					return false;
				}
				W.information[i * 9 + j] = sqrt(s);
			} else {
				W.information[i * 9 + j] = s / W.information[j * 9 + j];
			}
		}
		double s = -W.step[i];
		for (unsigned j = 0; j < i; j++) {
			s -= W.information[i * 9 + j] * W.step[j];
		}
		W.step[i] = s / W.information[i * 9 + i];
	}
	for (int i = 8; i >= 0; i--) {
		double s = W.step[i];
		for (unsigned j = i + 1; j < 9; j++) {
			s -= W.information[j * 9 + i] * W.step[j];
		}
		W.step[i] = s / W.information[i * 9 + i];
	}
	return true;
}
static int bootstrap(unsigned slots, mag_fit_read_fn read, mag_fit_poll_fn poll, void *ctx, float out[4][3])
{
	/* Median raw radius by bounded bisection: no sample copy and no sort buffer.
	 * Only gross radial outliers are removed from the algebraic initializer. */
	float low = FLT_MAX, high = 0;
	unsigned count = 0;
	for (unsigned i = 0; i < slots; i++) {
		if (!polling(poll, ctx, i)) {
			return -ECANCELED;
		}
		float x[3];
		if (!read(ctx, i, x)) {
			continue;
		}
		float n = norm3(x);
		if (!finite3(x) || !isfinite(n) || n <= 0) {
			return -EDOM;
		}
		low = fminf(low, n);
		high = fmaxf(high, n);
		count++;
	}
	if (count < 48) {
		return -EDOM;
	}
	for (unsigned pass = 0; pass < 16; pass++) {
		float mid = low + (high - low) * .5f;
		unsigned below = 0;
		for (unsigned i = 0; i < slots; i++) {
			if (!polling(poll, ctx, i)) {
				return -ECANCELED;
			}
			float x[3];
			if (read(ctx, i, x) && norm3(x) <= mid) {
				below++;
			}
		}
		if (below * 2 < count) {
			low = mid;
		} else {
			high = mid;
		}
	}
	float scale = high;
	if (!(scale > 1e-6f) || scale > 1e6f) {
		return -ERANGE;
	}
	memset(mag_cal_workspace.ata, 0, sizeof(mag_cal_workspace.ata));
	double sum = 0, used = 0;
	for (unsigned i = 0; i < slots; i++) {
		if (!polling(poll, ctx, i)) {
			return -ECANCELED;
		}
		float x[3];
		if (!read(ctx, i, x)) {
			continue;
		}
		float n = norm3(x) / scale;
		if (n < .15f || n > 3) {
			continue;
		}
		magneto_sample(x[0] / scale, x[1] / scale, x[2] / scale, mag_cal_workspace.ata, &sum, &used);
	}
	if (used < 48 || used < .8 * count) {
		return -EDOM;
	}
	if (poll && !poll(ctx)) {
		return -ECANCELED;
	}
	int err = magneto_current_calibration(out, mag_cal_workspace.ata, sum, used);
	if (err) {
		return err;
	}
	if (poll && !poll(ctx)) {
		return -ECANCELED;
	}
	/* Magneto targets the mean raw radius, not the centered field radius.
	 * Undo both that target and the input scaling before fixed-radius fitting. */
	for (unsigned i = 0; i < 3; i++) {
		out[0][i] *= scale;
		for (unsigned j = 0; j < 3; j++) {
			out[i + 1][j] *= (float)(.5 / ((sum / used) * scale));
		}
	}
	return 0;
}
int magneto_robust_fit(
	unsigned slots,
	mag_fit_read_fn read,
	mag_fit_poll_fn poll,
	void *ctx,
	const float previous[4][3],
	float field_norm,
	float output[4][3],
	struct mag_fit_result *result
)
{
	if (!read || !output || !result || slots < 48 || slots > 65535) {
		return -EINVAL;
	}
	float model[4][3], h = .5f;
	if (previous) {
		if (!isfinite(field_norm) || field_norm <= 1e-6f || field_norm > 1e6f) {
			return -ERANGE;
		}
		memcpy(model, previous, sizeof(model));
		for (unsigned i = 1; i < 4; i++) {
			for (unsigned j = 0; j < 3; j++) {
				model[i][j] *= h / field_norm;
			}
		}
	} else {
		if (field_norm != 0) {
			return -EINVAL;
		}
		int err = bootstrap(slots, read, poll, ctx, model);
		if (err) {
			return err;
		}
	}
	float p[9], initial[9], trial[9];
	for (unsigned i = 0; i < 3; i++) {
		if (!finite3(model[i + 1]) || !isfinite(model[0][i])) {
			return -EDOM;
		}
		p[i] = model[0][i] / h;
		p[3 + i] = model[i + 1][i];
	}
	if (fabsf(model[1][1] - model[2][0]) > 1e-4f || fabsf(model[1][2] - model[3][0]) > 1e-4f
		|| fabsf(model[2][2] - model[3][1]) > 1e-4f) {
		return -EDOM;
	}
	p[6] = .5f * (model[1][1] + model[2][0]);
	p[7] = .5f * (model[1][2] + model[3][0]);
	p[8] = .5f * (model[2][2] + model[3][1]);
	memcpy(initial, p, sizeof(p));
	if (!constrained(p, NULL)) {
		return -ERANGE;
	}
	float condition = 0, rms = 0, good = 0, loss = 0;
	for (unsigned iteration = 0; iteration <= 4; iteration++) {
		int err = directions(slots, read, poll, ctx, h, p);
		if (err) {
			return err;
		}
		err = accumulate(slots, read, poll, ctx, h, p, p, true, &loss, &rms, &good);
		if (err) {
			return err;
		}
		double lo, hi;
		if (!eigen(W.information, 9, poll, ctx, &lo, &hi)) {
			return -ECANCELED;
		}
		condition = hi / lo;
		if (lo < .002f || !isfinite(condition) || condition > 1000) {
			return -EDOM;
		}
		if (iteration == 4) {
			break;
		}
		/* Jacobi destroys H. Stream the frozen pool again, avoiding a second
		 * 648-byte matrix while retaining double accumulation and solving. */
		err = accumulate(slots, read, poll, ctx, h, p, p, true, &loss, &rms, &good);
		if (err) {
			return err;
		}
		if (!solve()) {
			return -EDOM;
		}
		bool accepted = false;
		for (unsigned backtrack = 0; backtrack < 4; backtrack++) {
			float factor = 1.0f / (1u << backtrack);
			for (unsigned i = 0; i < 9; i++) {
				trial[i] = p[i] + factor * W.step[i];
			}
			if (!constrained(trial, p) || (previous && !constrained(trial, initial))) {
				continue;
			}
			float cost, trms, tgood;
			err = accumulate(slots, read, poll, ctx, h, trial, p, false, &cost, &trms, &tgood);
			if (err) {
				return err;
			}
			if (cost <= loss + 1e-9f) {
				memcpy(p, trial, sizeof(p));
				accepted = true;
				break;
			}
		}
		if (!accepted) {
			double step2 = 0;
			for (unsigned i = 0; i < 9; i++) {
				step2 += W.step[i] * W.step[i];
			}
			if (step2 > 1e-8f) {
				return -EAGAIN;
			}
			break;
		}
	}
	if (good < .8f || rms > .08f) {
		return -EDOM;
	}
	if (poll && !poll(ctx)) {
		return -ECANCELED;
	}
	float a[9];
	matrix(p, a);
	for (unsigned i = 0; i < 3; i++) {
		model[0][i] = p[i] * h;
		memcpy(model[i + 1], a + i * 3, 3 * sizeof(float));
	}
	memcpy(output, model, sizeof(model));
	*result = (struct mag_fit_result){h, condition, rms};
	return 0;
}
