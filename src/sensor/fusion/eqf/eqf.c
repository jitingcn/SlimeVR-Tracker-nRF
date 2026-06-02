/*
 * ABC-EqF n=0: Attitude-Bias Equivariant Filter
 *
 * Implements the n=0 (no online calibration) variant of the Equivariant
 * Filter which tracks attitude R ∈ SO(3) and gyro bias b ∈ R³ using a
 * Kalman filter on the Lie group SO(3) × R³.
 *
 * State representation:
 *   Group element X = (A, a_vec) where A ∈ SO(3), a_vec ∈ R³
 *   Attitude estimate: R_hat = A  (body → earth rotation)
 *   Gyro bias estimate: b_hat = -A^T · a_vec
 *   Error covariance: P ∈ R^{6×6}
 *
 * Reference: Fornasier et al., "Overcoming Bias: Equivariant Filter Design
 * for Biased Attitude Estimation with Online Calibration", RA-L 2022.
 *
 * SPDX-License-Identifier: MIT
 */

#include <math.h>
#include <string.h>

#include "eqf.h"
#include "retained.h"
#include "util.h"

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.01745329251994329577f  /* (float)(M_PI / 180.0) */
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.29577951308232087680f  /* (float)(180.0 / M_PI) */
#endif

/* ── EqF tuning parameters (optimised from Python sweep) ───────────── */
#define EQF_SIGMA_W      0.003f    /* gyro noise PSD      (rad/s/√Hz) */
#define EQF_SIGMA_B      0.0003f   /* bias random walk PSD (rad/s²/√Hz) */
#define EQF_SIGMA_ACC    0.01f     /* accel meas. noise (unit-vector)  */
#define EQF_SIGMA_MAG    0.03f     /* mag meas. noise (unit-vector)    */
#define EQF_ACC_ADAPT_K  100.0f    /* accel adaptive noise gain        */

#define EQF_INIT_SAMPLES   50     /* TRIAD init accumulation count    */
#define EQF_ORTHO_INTERVAL 100    /* re-orthonormalise every N gyro   */

#define EQF_P_INIT_ATT   0.1f    /* initial attitude variance (rad²) */
#define EQF_P_INIT_BIAS  0.01f   /* initial bias variance (rad/s)²   */

#define EQF_MAGIC         0x45714630u  /* "EqF0" */

/* ── Rest detection parameters (mirrors VQF defaults) ──────────────── */
#define EQF_REST_TAU      0.5f    /* LP filter time constant (s)      */
#define EQF_REST_TH_GYR   2.0f    /* gyro deviation threshold (deg/s) */
#define EQF_REST_TH_ACC   0.5f    /* accel deviation threshold (g)    */
#define EQF_REST_MIN_T    1.5f    /* min rest duration to trigger (s) */
#define EQF_REST_SIGMA    0.03f   /* bias meas. noise during rest     */
#define EQF_REST_MAX_BIAS 10.0f   /* max plausible bias (deg/s)       */
#define EQF_REST_ACC_NORM_TH  0.1f /* accel-norm deviation from 1g    */

#if defined(EQF_DISABLE_MAG_DIST_REJECTION)
#define EQF_MAG_DIST_REJECTION_ENABLED 0
#else
#define EQF_MAG_DIST_REJECTION_ENABLED 1
#endif

/* ── Magnetic disturbance handling (mirrors VQF strategy) ─────────── */
#define EQF_MAG_CURRENT_TAU          0.50f    /* current norm/dip LP (s)      */
#define EQF_MAG_REF_TAU             15.0f     /* reference adaptation LP (s)  */
#define EQF_MAG_NORM_TH              0.25f    /* relative norm threshold      */
#define EQF_MAG_DIP_TH              12.0f     /* dip threshold (deg)          */
#define EQF_MAG_NEW_TIME            12.0f     /* accept new field after (s)   */
#define EQF_MAG_NEW_FIRST_TIME       5.5f     /* first accept with no ref (s) */
#define EQF_MAG_NEW_MIN_GYR         16.0f     /* motion needed for accept     */
#define EQF_MAG_MIN_UNDISTURBED_T    0.6f     /* stable time to re-enable     */
#define EQF_MAG_MAX_REJECTION_TIME 3200.0f    /* full rejection duration (s)  */
#define EQF_MAG_REJECTION_FACTOR  1150.0f     /* weak trust after long disturb */
#define EQF_MAG_REJECTION_SIGMA_SCALE 34.0f   /* short-disturbance downweight  */

/* ── Internal state ────────────────────────────────────────────────── */

typedef enum { EQF_INIT, EQF_RUNNING } eqf_mode_t;

/* Persistent state – saved / loaded via retained memory */
typedef struct {
	float A[9];          /* SO(3) rotation matrix, row-major        */
	float a_vec[3];      /* bias Lie-algebra vector                 */
	float P[36];         /* 6×6 error covariance, row-major         */
	float d_mag[3];      /* mag reference direction (earth frame)   */
	float mag_ref_norm;  /* reference mag field norm                */
	uint32_t magic;      /* validation word                         */
} eqf_saved_t;

static eqf_saved_t st;
static eqf_mode_t  mode;
static float dt_gyr, dt_acc, dt_mag;

/* TRIAD init accumulators */
static float acc_sum[3], mag_sum[3];
static float mag_norm_sum;
static int   init_count;
static int   acc_init_count;
static int   mag_init_count;
static int   acc_only_init_count;
static bool  have_acc, have_mag;

/* Last accel in m/s² for linear-acceleration query */
static float last_a_ms2[3];

static int ortho_counter;

/* ── Rest detection state (volatile, not saved to retained) ────────── */
static float rest_gyr_lp[3];    /* LP filtered gyro (rad/s)          */
static float rest_acc_lp[3];    /* LP filtered accel (g)             */
static float rest_gyr_dev;      /* last gyro squared deviation       */
static float rest_t;            /* accumulated rest time (s)         */
static bool  rest_detected;
static bool  rest_gyr_lp_init;  /* gyro LP initialized              */
static bool  rest_acc_lp_init;  /* accel LP initialized             */

/* ── Magnetic heading-trust state (volatile, not saved) ───────────── */
static bool  mag_active;           /* true when heading can trust mag    */
static bool  mag_ref_valid;        /* current reference came from real mag */
static bool  mag_dist_detected;    /* disturbance gate active            */
static bool  mag_norm_dip_lp_init; /* current norm/dip LP initialized    */
static float mag_norm_lp;          /* LP current field norm              */
static float mag_dip_lp;           /* LP current field dip               */
static float mag_candidate_norm;   /* alternative field candidate norm   */
static float mag_candidate_dip;    /* alternative field candidate dip    */
static float mag_candidate_t;      /* time spent in candidate field      */
static float mag_undisturbed_t;    /* stable time in current field       */
static float mag_reject_t;         /* accumulated rejection time         */

/* ── 3×3 matrix helpers (row-major float[9]) ───────────────────────── */

static inline void m3_eye(float *M)
{
	memset(M, 0, 9 * sizeof(float));
	M[0] = M[4] = M[8] = 1.0f;
}

static inline void m3_copy(float *dst, const float *src)
{
	memcpy(dst, src, 9 * sizeof(float));
}

/* C = A · B */
static void m3_mul(const float *A, const float *B, float *C)
{
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			C[i * 3 + j] = A[i * 3 + 0] * B[0 * 3 + j]
				      + A[i * 3 + 1] * B[1 * 3 + j]
				      + A[i * 3 + 2] * B[2 * 3 + j];
}

/* out = A^T  (safe if out == A) */
static void m3_transpose(const float *A, float *out)
{
	float t[9];
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			t[i * 3 + j] = A[j * 3 + i];
	memcpy(out, t, sizeof(t));
}

/* out = A · v */
static inline void m3v(const float *A, const float *v, float *out)
{
	out[0] = A[0] * v[0] + A[1] * v[1] + A[2] * v[2];
	out[1] = A[3] * v[0] + A[4] * v[1] + A[5] * v[2];
	out[2] = A[6] * v[0] + A[7] * v[1] + A[8] * v[2];
}

/* out = A^T · v */
static inline void m3Tv(const float *A, const float *v, float *out)
{
	out[0] = A[0] * v[0] + A[3] * v[1] + A[6] * v[2];
	out[1] = A[1] * v[0] + A[4] * v[1] + A[7] * v[2];
	out[2] = A[2] * v[0] + A[5] * v[1] + A[8] * v[2];
}

/* C = A · B^T  (avoids explicit transpose) */
static void m3_mulBT(const float *A, const float *B, float *C)
{
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			C[i * 3 + j] = A[i * 3 + 0] * B[j * 3 + 0]
				      + A[i * 3 + 1] * B[j * 3 + 1]
				      + A[i * 3 + 2] * B[j * 3 + 2];
}

/* skew-symmetric matrix  [0 -z y; z 0 -x; -y x 0] */
static inline void skew3(const float *v, float *M)
{
	M[0] =  0;     M[1] = -v[2]; M[2] =  v[1];
	M[3] =  v[2];  M[4] =  0;    M[5] = -v[0];
	M[6] = -v[1];  M[7] =  v[0]; M[8] =  0;
}

/* skew(d)² = d·d^T − I  (for unit d) */
static inline void skew3_sq(const float *d, float *M)
{
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			M[i * 3 + j] = d[i] * d[j] - (i == j ? 1.0f : 0.0f);
}

static inline float v3_dot(const float *a, const float *b)
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static inline float v3_norm(const float *v)
{
	return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static inline float eqf_resolve_dt(float time, float fallback_dt)
{
	return (isfinite(time) && time > 0.0f && time < 1.0f) ? time : fallback_dt;
}

static inline float eqf_alpha_from_tau(float tau, float dt)
{
	if (!(tau > 0.0f) || !(dt > 0.0f)) {
		return 1.0f;
	}
	float alpha = dt / tau;
	if (alpha > 1.0f) {
		alpha = 1.0f;
	}
	return alpha;
}

static inline float eqf_clampf(float x, float lo, float hi)
{
	if (x < lo) {
		return lo;
	}
	if (x > hi) {
		return hi;
	}
	return x;
}

static inline float eqf_get_mag_ref_dip(void)
{
	return atan2f(-st.d_mag[2], st.d_mag[0]);
}

static void eqf_set_mag_ref(float norm, float dip)
{
	float c = cosf(dip);
	float s = sinf(dip);

	st.mag_ref_norm = norm;
	st.d_mag[0] = c;
	st.d_mag[1] = 0.0f;
	st.d_mag[2] = -s;
}

static void eqf_reset_mag_runtime_state(bool trust_ref_now)
{
	float ref_dip = eqf_get_mag_ref_dip();

	mag_norm_dip_lp_init = false;
	mag_norm_lp = st.mag_ref_norm;
	mag_dip_lp = ref_dip;
	mag_candidate_norm = trust_ref_now && mag_ref_valid ? st.mag_ref_norm : 0.0f;
	mag_candidate_dip = trust_ref_now && mag_ref_valid ? ref_dip : 0.0f;
	mag_candidate_t = 0.0f;
	mag_reject_t = 0.0f;
	mag_undisturbed_t = trust_ref_now && mag_ref_valid ? EQF_MAG_MIN_UNDISTURBED_T : 0.0f;
	mag_dist_detected = !mag_ref_valid;
	mag_active = trust_ref_now && mag_ref_valid;
}

static void eqf_get_mag_norm_dip(const float *m, float *norm_out, float *dip_out)
{
	float mn = v3_norm(m);
	if (mn < 1e-10f) {
		*norm_out = 0.0f;
		*dip_out = 0.0f;
		return;
	}

	float unit_mag[3] = { m[0] / mn, m[1] / mn, m[2] / mn };
	float mag_earth[3];
	m3v(st.A, unit_mag, mag_earth);
	float sin_dip = eqf_clampf(-mag_earth[2], -1.0f, 1.0f);

	*norm_out = mn;
	*dip_out = asinf(sin_dip);
}

static inline void v3_normalise(float *v)
{
	float n = v3_norm(v);
	if (n > 1e-10f) {
		float inv = 1.0f / n;
		v[0] *= inv;
		v[1] *= inv;
		v[2] *= inv;
	}
}

static inline void v3_cross(const float *a, const float *b, float *out)
{
	out[0] = a[1] * b[2] - a[2] * b[1];
	out[1] = a[2] * b[0] - a[0] * b[2];
	out[2] = a[0] * b[1] - a[1] * b[0];
}

/* 3×3 matrix inverse via cofactors (scale-aware threshold) */
static bool m3_inv(const float *M, float *out)
{
	float det = M[0] * (M[4] * M[8] - M[5] * M[7])
		  - M[1] * (M[3] * M[8] - M[5] * M[6])
		  + M[2] * (M[3] * M[7] - M[4] * M[6]);
	/* scale-aware: require |det| > eps * max(|diag|)^3 */
	float dmax = fabsf(M[0]);
	if (fabsf(M[4]) > dmax) dmax = fabsf(M[4]);
	if (fabsf(M[8]) > dmax) dmax = fabsf(M[8]);
	float thr = 1e-8f * dmax * dmax * dmax;
	if (thr < 1e-20f) thr = 1e-20f;
	if (fabsf(det) < thr)
		return false;
	float id = 1.0f / det;
	out[0] = (M[4] * M[8] - M[5] * M[7]) * id;
	out[1] = (M[2] * M[7] - M[1] * M[8]) * id;
	out[2] = (M[1] * M[5] - M[2] * M[4]) * id;
	out[3] = (M[5] * M[6] - M[3] * M[8]) * id;
	out[4] = (M[0] * M[8] - M[2] * M[6]) * id;
	out[5] = (M[2] * M[3] - M[0] * M[5]) * id;
	out[6] = (M[3] * M[7] - M[4] * M[6]) * id;
	out[7] = (M[1] * M[6] - M[0] * M[7]) * id;
	out[8] = (M[0] * M[4] - M[1] * M[3]) * id;
	return true;
}

/* Rodrigues formula: R = exp(v), v = angle × axis */
static void rodrigues(const float *v, float *R)
{
	float angle = v3_norm(v);
	if (angle < 1e-8f) {
		/* first-order: R ≈ I + [v]× */
		m3_eye(R);
		R[1] -= v[2]; R[2] += v[1];
		R[3] += v[2]; R[5] -= v[0];
		R[6] -= v[1]; R[7] += v[0];
		return;
	}
	float c  = cosf(angle), s = sinf(angle), t = 1.0f - c;
	float ia = 1.0f / angle;
	float x  = v[0] * ia, y = v[1] * ia, z = v[2] * ia;

	R[0] = c + t * x * x;     R[1] = t * x * y - s * z; R[2] = t * x * z + s * y;
	R[3] = t * y * x + s * z; R[4] = c + t * y * y;     R[5] = t * y * z - s * x;
	R[6] = t * z * x - s * y; R[7] = t * z * y + s * x; R[8] = c + t * z * z;
}

/*
 * SO(3) left Jacobian.
 * J_L(v) = sinθ/θ I + (1 − sinθ/θ)(a⊗a) + (1−cosθ)/θ [a]×
 * For small |v|: J ≈ I + ½[v]×
 */
static void so3_jl(const float *v, float *J)
{
	float angle = v3_norm(v);
	if (angle < 1e-6f) {
		m3_eye(J);
		float h = 0.5f;
		J[1] -= h * v[2]; J[2] += h * v[1];
		J[3] += h * v[2]; J[5] -= h * v[0];
		J[6] -= h * v[1]; J[7] += h * v[0];
		return;
	}
	float s  = sinf(angle), c = cosf(angle), ia = 1.0f / angle;
	float x  = v[0] * ia, y = v[1] * ia, z = v[2] * ia;
	float sa = s * ia;              /* sinθ/θ   */
	float ms = 1.0f - sa;          /* 1−sinθ/θ */
	float mc = (1.0f - c) * ia;    /* (1−cosθ)/θ */

	J[0] = sa + ms * x * x;       J[1] = ms * x * y - mc * z; J[2] = ms * x * z + mc * y;
	J[3] = ms * y * x + mc * z;   J[4] = sa + ms * y * y;     J[5] = ms * y * z - mc * x;
	J[6] = ms * z * x - mc * y;   J[7] = ms * z * y + mc * x; J[8] = sa + ms * z * z;
}

/* Symmetrise a 3×3 matrix in-place: M = (M + M^T)/2 */
static inline void m3_sym(float *M)
{
	float a;
	a = 0.5f * (M[1] + M[3]); M[1] = M[3] = a;
	a = 0.5f * (M[2] + M[6]); M[2] = M[6] = a;
	a = 0.5f * (M[5] + M[7]); M[5] = M[7] = a;
}

/* Gram-Schmidt re-orthonormalisation (row-major) */
static void reortho(float *A)
{
	float r0[3] = { A[0], A[1], A[2] };
	float r1[3] = { A[3], A[4], A[5] };
	float r2[3];

	v3_normalise(r0);
	float d = v3_dot(r1, r0);
	r1[0] -= d * r0[0]; r1[1] -= d * r0[1]; r1[2] -= d * r0[2];
	v3_normalise(r1);
	v3_cross(r0, r1, r2);

	A[0] = r0[0]; A[1] = r0[1]; A[2] = r0[2];
	A[3] = r1[0]; A[4] = r1[1]; A[5] = r1[2];
	A[6] = r2[0]; A[7] = r2[1]; A[8] = r2[2];
}

/* Rotation matrix → quaternion [w, x, y, z] (Shepperd's method) */
static void mat_to_quat(const float *R, float *q)
{
	float tr = R[0] + R[4] + R[8];
	if (tr > 0) {
		float s = 0.5f / sqrtf(tr + 1.0f);
		q[0] = 0.25f / s;
		q[1] = (R[7] - R[5]) * s;
		q[2] = (R[2] - R[6]) * s;
		q[3] = (R[3] - R[1]) * s;
	} else if (R[0] > R[4] && R[0] > R[8]) {
		float s = 2.0f * sqrtf(1.0f + R[0] - R[4] - R[8]);
		q[0] = (R[7] - R[5]) / s;
		q[1] = 0.25f * s;
		q[2] = (R[1] + R[3]) / s;
		q[3] = (R[2] + R[6]) / s;
	} else if (R[4] > R[8]) {
		float s = 2.0f * sqrtf(1.0f + R[4] - R[0] - R[8]);
		q[0] = (R[2] - R[6]) / s;
		q[1] = (R[1] + R[3]) / s;
		q[2] = 0.25f * s;
		q[3] = (R[5] + R[7]) / s;
	} else {
		float s = 2.0f * sqrtf(1.0f + R[8] - R[0] - R[4]);
		q[0] = (R[3] - R[1]) / s;
		q[1] = (R[2] + R[6]) / s;
		q[2] = (R[5] + R[7]) / s;
		q[3] = 0.25f * s;
	}
	/* canonical: positive w */
	if (q[0] < 0) {
		q[0] = -q[0]; q[1] = -q[1]; q[2] = -q[2]; q[3] = -q[3];
	}
}

/* ── 6×6 block helpers ─────────────────────────────────────────────── */
/* P is stored as float[36], row-major.  Block indices bi,bj ∈ {0,1}. */

static void p_get(const float *P, int bi, int bj, float *B)
{
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			B[i * 3 + j] = P[(bi * 3 + i) * 6 + bj * 3 + j];
}

static void p_set(float *P, int bi, int bj, const float *B)
{
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			P[(bi * 3 + i) * 6 + bj * 3 + j] = B[i * 3 + j];
}

/* ── TRIAD initialisation ──────────────────────────────────────────── */
/*
 * Build an initial R from averaged accel (gravity) and mag readings.
 * Earth frame: X along horizontal mag component, Y east, Z up.
 */
static void triad_init(const float *acc_avg, const float *mag_avg, float mag_norm_avg)
{
	/* ---- body triad ---- */
	float b1[3] = { acc_avg[0], acc_avg[1], acc_avg[2] };
	v3_normalise(b1);

	float mn[3] = { mag_avg[0], mag_avg[1], mag_avg[2] };
	v3_normalise(mn);

	float b2[3];
	v3_cross(b1, mn, b2);
	if (v3_norm(b2) < 1e-6f) {
		/* mag ∥ gravity – can't determine heading; pick arbitrary */
		float tmp[3] = { 1, 0, 0 };
		v3_cross(b1, tmp, b2);
		if (v3_norm(b2) < 1e-6f) {
			float tmp2[3] = { 0, 1, 0 };
			v3_cross(b1, tmp2, b2);
		}
	}
	v3_normalise(b2);

	float b3[3];
	v3_cross(b1, b2, b3);

	/* ---- mag reference direction from dip angle ---- */
	float sin_dip = -v3_dot(mn, b1); /* vertical component of unit mag */
	float cos_dip = sqrtf(fmaxf(0.0f, 1.0f - sin_dip * sin_dip));
	if (cos_dip < 0.01f) cos_dip = 0.01f;

	st.d_mag[0] =  cos_dip;
	st.d_mag[1] =  0.0f;
	st.d_mag[2] = -sin_dip;

	/*
	 * Earth triad (constant for our convention):
	 *   e1 = [0,0,1],  e2 = [0,1,0],  e3 = [-1,0,0]
	 *
	 * R = E · B^T  where columns of E = [e1|e2|e3], B = [b1|b2|b3].
	 * Expanding:
	 *   row 0 of R = −b3
	 *   row 1 of R =  b2
	 *   row 2 of R =  b1
	 */
	st.A[0] = -b3[0]; st.A[1] = -b3[1]; st.A[2] = -b3[2];
	st.A[3] =  b2[0]; st.A[4] =  b2[1]; st.A[5] =  b2[2];
	st.A[6] =  b1[0]; st.A[7] =  b1[1]; st.A[8] =  b1[2];

	st.mag_ref_norm = mag_norm_avg > 0.01f ? mag_norm_avg : v3_norm(mag_avg);
	mag_ref_valid = true;

	/* bias = 0 */
	memset(st.a_vec, 0, sizeof(st.a_vec));

	/* diagonal covariance */
	memset(st.P, 0, sizeof(st.P));
	for (int i = 0; i < 3; i++)
		st.P[i * 6 + i] = EQF_P_INIT_ATT;
	for (int i = 3; i < 6; i++)
		st.P[i * 6 + i] = EQF_P_INIT_BIAS;

	st.magic = EQF_MAGIC;
	eqf_reset_mag_runtime_state(true);
}

/*
 * gravity_init – accel-only initialisation when mag is unavailable.
 * Determines pitch/roll from averaged accel, sets yaw to 0.
 * Gives larger initial yaw covariance since heading is unknown.
 */
static void gravity_init(const float *acc_avg)
{
	/* normalise gravity vector → body z-axis (up) */
	float gn = v3_norm(acc_avg);
	if (gn < 1e-6f) {
		/* degenerate – stay at identity */
		m3_eye(st.A);
		goto done;
	}
	float inv = 1.0f / gn;
	float b1[3] = { acc_avg[0] * inv, acc_avg[1] * inv, acc_avg[2] * inv };

	/* choose an arbitrary east direction perpendicular to gravity */
	float ref[3] = { 1.0f, 0.0f, 0.0f };
	if (fabsf(v3_dot(b1, ref)) > 0.9f) {
		ref[0] = 0.0f; ref[1] = 1.0f; /* fallback if gravity ≈ x-axis */
	}
	float b2[3];
	v3_cross(b1, ref, b2);
	float b2n = v3_norm(b2);
	if (b2n < 1e-6f) { m3_eye(st.A); goto done; }
	inv = 1.0f / b2n;
	b2[0] *= inv; b2[1] *= inv; b2[2] *= inv;

	float b3[3];
	v3_cross(b1, b2, b3);

	/* R = E · B^T  (same convention as TRIAD) */
	st.A[0] = -b3[0]; st.A[1] = -b3[1]; st.A[2] = -b3[2];
	st.A[3] =  b2[0]; st.A[4] =  b2[1]; st.A[5] =  b2[2];
	st.A[6] =  b1[0]; st.A[7] =  b1[1]; st.A[8] =  b1[2];

done:
	/* default mag reference (horizontal north) */
	st.d_mag[0] = 1.0f; st.d_mag[1] = 0.0f; st.d_mag[2] = 0.0f;
	st.mag_ref_norm = 1.0f;
	mag_ref_valid = false;

	memset(st.a_vec, 0, sizeof(st.a_vec));
	memset(st.P, 0, sizeof(st.P));
	/* large yaw variance since heading is unknown;
	 * in left-trivialized error, Δ_rot[2] is earth-z rotation (yaw) */
	st.P[0]  = EQF_P_INIT_ATT; /* earth-x rotation */
	st.P[7]  = EQF_P_INIT_ATT; /* earth-y rotation */
	st.P[14] = 3.14f * 3.14f;  /* earth-z = yaw ≈ π² */
	for (int i = 3; i < 6; i++)
		st.P[i * 6 + i] = EQF_P_INIT_BIAS;

	st.magic = EQF_MAGIC;
	eqf_reset_mag_runtime_state(false);
}

/* ── Core EqF propagation (one gyro sample) ────────────────────────── */

static void eqf_propagate(const float *w, float dt)
{
	/* b̂ = −A^T · a_vec */
	float bh[3];
	m3Tv(st.A, st.a_vec, bh);
	bh[0] = -bh[0]; bh[1] = -bh[1]; bh[2] = -bh[2];

	/* Lift: ω = w − b̂ ,  L_bias = b̂ × w */
	float om[3] = { w[0] - bh[0], w[1] - bh[1], w[2] - bh[2] };
	float lb[3];
	v3_cross(bh, w, lb);

	float lr_dt[3] = { om[0] * dt, om[1] * dt, om[2] * dt };
	float lb_dt[3] = { lb[0] * dt, lb[1] * dt, lb[2] * dt };

	/* ---- state update: X_new = X · exp(L·dt) ---- */
	float dA[9];
	rodrigues(lr_dt, dA);

	float J[9];
	so3_jl(lr_dt, J);
	float da[3];
	m3v(J, lb_dt, da);

	float A_new[9];
	m3_mul(st.A, dA, A_new);

	float A_da[3];
	m3v(st.A, da, A_da);
	st.a_vec[0] += A_da[0];
	st.a_vec[1] += A_da[1];
	st.a_vec[2] += A_da[2];
	m3_copy(st.A, A_new);

	/* ---- covariance propagation ---- */
	/* W0 = [A·ω]×  */
	float Aw[3];
	m3v(st.A, om, Aw);
	float W0[9];
	skew3(Aw, W0);

	float W0sq[9];
	m3_mul(W0, W0, W0sq);

	float dt2 = dt * dt;

	/* Phi12 = −dt·(I + dt/2·W0 + dt²/6·W0²) */
	float F[9];
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++) {
			int k = i * 3 + j;
			float eye = (i == j) ? 1.0f : 0.0f;
			F[k] = -dt * (eye + 0.5f * dt * W0[k]
				     + (dt2 / 6.0f) * W0sq[k]);
		}

	/* Phi22 = I + dt·W0 + dt²/2·W0² */
	float G[9];
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++) {
			int k = i * 3 + j;
			float eye = (i == j) ? 1.0f : 0.0f;
			G[k] = eye + dt * W0[k] + 0.5f * dt2 * W0sq[k];
		}

	/* P blocks */
	float P11[9], P12[9], P21[9], P22[9];
	p_get(st.P, 0, 0, P11);
	p_get(st.P, 0, 1, P12);
	p_get(st.P, 1, 0, P21);
	p_get(st.P, 1, 1, P22);

	/*
	 * Phi = [[I, F], [0, G]]
	 *
	 * new_P11 = P11 + F·P21 + (F·P21)^T + F·P22·F^T
	 * new_P12 = (P12 + F·P22) · G^T
	 * new_P21 = new_P12^T              (symmetry)
	 * new_P22 = G·P22·G^T
	 */
	float T1[9], T2[9];
	m3_mul(F, P21, T1);         /* F·P21  */
	m3_mul(F, P22, T2);         /* F·P22  */

	float FT[9];
	m3_transpose(F, FT);
	float T2FT[9];
	m3_mul(T2, FT, T2FT);      /* F·P22·F^T */

	float nP11[9];
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++) {
			int k = i * 3 + j;
			nP11[k] = P11[k] + T1[k] + T1[j * 3 + i] + T2FT[k];
		}

	float mid[9];               /* P12 + F·P22 */
	for (int i = 0; i < 9; i++)
		mid[i] = P12[i] + T2[i];
	float GT[9];
	m3_transpose(G, GT);
	float nP12[9];
	m3_mul(mid, GT, nP12);

	float nP21[9];
	m3_transpose(nP12, nP21);

	float GP22[9];
	m3_mul(G, P22, GP22);
	float nP22[9];
	m3_mul(GP22, GT, nP22);

	/* process noise M = diag(σ_w², …, σ_b², …) · dt */
	float sw2dt = EQF_SIGMA_W * EQF_SIGMA_W * dt;
	float sb2dt = EQF_SIGMA_B * EQF_SIGMA_B * dt;
	nP11[0] += sw2dt; nP11[4] += sw2dt; nP11[8] += sw2dt;
	nP22[0] += sb2dt; nP22[4] += sb2dt; nP22[8] += sb2dt;

	/* symmetrise to prevent float32 drift (propagation runs at 473 Hz) */
	m3_sym(nP11);
	m3_sym(nP22);

	p_set(st.P, 0, 0, nP11);
	p_set(st.P, 0, 1, nP12);
	p_set(st.P, 1, 0, nP21);
	p_set(st.P, 1, 1, nP22);

	/* periodic re-orthonormalisation */
	if (++ortho_counter >= EQF_ORTHO_INTERVAL) {
		reortho(st.A);
		ortho_counter = 0;
	}
}

/* ── Core EqF direction-measurement update ─────────────────────────── */
/*
 * Shared logic for accel and mag updates.
 * y_raw : raw body-frame measurement (need not be unit)
 * d     : known unit-direction in earth frame
 * sigma : measurement noise std-dev
 */
static void eqf_dir_update(const float *y_raw, const float *d, float sigma,
			   bool suppress_yaw)
{
	/* normalise measurement */
	float y[3] = { y_raw[0], y_raw[1], y_raw[2] };
	float n = v3_norm(y);
	if (n < 1e-10f)
		return;
	float inv = 1.0f / n;
	y[0] *= inv; y[1] *= inv; y[2] *= inv;

	/* Cd = skew(d)² = d·d^T − I   (3×3) */
	float Cd[9];
	skew3_sq(d, Cd);

	/* innovation δ = [d]× · (A · y) */
	float Ay[3];
	m3v(st.A, y, Ay);
	float sk_d[9];
	skew3(d, sk_d);
	float delta[3];
	m3v(sk_d, Ay, delta);

	/* S = Cd · P11 · Cd^T + σ² I */
	float P11[9];
	p_get(st.P, 0, 0, P11);
	float CdP[9];
	m3_mul(Cd, P11, CdP);
	float CdT[9];
	m3_transpose(Cd, CdT);
	float S[9];
	m3_mul(CdP, CdT, S);
	float s2 = sigma * sigma;
	S[0] += s2; S[4] += s2; S[8] += s2;

	float Si[9];
	if (!m3_inv(S, Si))
		return;

	/* K = P · C^T · S⁻¹  where C = [Cd | 0₃ₓ₃]
	 * K_upper = P11·Cd^T·Si,  K_lower = P21·Cd^T·Si */
	float P21[9];
	p_get(st.P, 1, 0, P21);

	float P11Ct[9], P21Ct[9];
	m3_mul(P11, CdT, P11Ct);
	m3_mul(P21, CdT, P21Ct);

	float Ku[9], Kl[9];
	m3_mul(P11Ct, Si, Ku);
	m3_mul(P21Ct, Si, Kl);

	/*
	 * 6-axis yaw suppression: prevent accel from affecting
	 * earth-z rotation (Ku row 2) and gyro bias (Kl).
	 * Bias is then ONLY updated through rest detection.
	 */
	if (suppress_yaw) {
		Ku[6] = 0.0f; Ku[7] = 0.0f; Ku[8] = 0.0f;
		memset(Kl, 0, 9 * sizeof(float));
	}

	/* InnovationLift  diag(1,1,1,−1,−1,−1) */
	float Delta[6];
	m3v(Ku, delta, &Delta[0]);
	m3v(Kl, delta, &Delta[3]);
	Delta[3] = -Delta[3]; Delta[4] = -Delta[4]; Delta[5] = -Delta[5];

	/* ---- state correction: X_new = exp(Δ) · X_old  (left-multiply) ---- */
	float dA[9];
	rodrigues(&Delta[0], dA);

	float J[9];
	so3_jl(&Delta[0], J);
	float da[3];
	m3v(J, &Delta[3], da);

	float A_new[9];
	m3_mul(dA, st.A, A_new);

	float dAa[3];
	m3v(dA, st.a_vec, dAa);
	st.a_vec[0] = da[0] + dAa[0];
	st.a_vec[1] = da[1] + dAa[1];
	st.a_vec[2] = da[2] + dAa[2];
	m3_copy(st.A, A_new);

	/* ---- covariance: Joseph form P = M·P·M^T + K·R·K^T ---- */
	/*
	 * M = I − KC = [[ A_,  0 ],   where A_ = I − Ku·Cd
	 *               [ B_,  I ]]          B_ = −Kl·Cd
	 *
	 * Joseph form guarantees positive semi-definiteness under float32,
	 * unlike the simplified P = (I−KC)·P which accumulates cancellation.
	 */
	float KuCd[9], KlCd[9];
	m3_mul(Ku, Cd, KuCd);
	m3_mul(Kl, Cd, KlCd);

	/* A_ = I − Ku·Cd */
	float A_[9];
	m3_eye(A_);
	for (int i = 0; i < 9; i++)
		A_[i] -= KuCd[i];

	/* B_ = −Kl·Cd */
	float B_[9];
	for (int i = 0; i < 9; i++)
		B_[i] = -KlCd[i];

	float P12[9], P22[9];
	p_get(st.P, 0, 1, P12);
	p_get(st.P, 1, 1, P22);

	/* reusable intermediates */
	float AP11[9], BP11[9];
	m3_mul(A_, P11, AP11);   /* A_·P11 */
	m3_mul(B_, P11, BP11);   /* B_·P11 */

	/* nP11 = A_·P11·A_^T + σ²·Ku·Ku^T */
	float nP11[9], T[9];
	m3_mulBT(AP11, A_, nP11);
	m3_mulBT(Ku, Ku, T);
	for (int i = 0; i < 9; i++)
		nP11[i] += s2 * T[i];

	/* nP12 = A_·P11·B_^T + A_·P12 + σ²·Ku·Kl^T */
	float nP12[9], AP12[9];
	m3_mulBT(AP11, B_, nP12);
	m3_mul(A_, P12, AP12);
	m3_mulBT(Ku, Kl, T);
	for (int i = 0; i < 9; i++)
		nP12[i] += AP12[i] + s2 * T[i];

	/* nP21 = nP12^T  (enforce symmetry) */
	float nP21[9];
	m3_transpose(nP12, nP21);

	/* nP22 = B_·P11·B_^T + B_·P12 + P21·B_^T + P22 + σ²·Kl·Kl^T */
	float nP22[9], BP12[9], P21BT[9];
	m3_mulBT(BP11, B_, nP22);
	m3_mul(B_, P12, BP12);
	m3_mulBT(P21, B_, P21BT);
	m3_mulBT(Kl, Kl, T);
	for (int i = 0; i < 9; i++)
		nP22[i] += BP12[i] + P21BT[i] + P22[i] + s2 * T[i];

	/* symmetrise diagonal blocks */
	m3_sym(nP11);
	m3_sym(nP22);

	p_set(st.P, 0, 0, nP11);
	p_set(st.P, 0, 1, nP12);
	p_set(st.P, 1, 0, nP21);
	p_set(st.P, 1, 1, nP22);

	/* clamp negative diagonals (extra safety) */
	for (int i = 0; i < 6; i++)
		if (st.P[i * 6 + i] < 1e-12f)
			st.P[i * 6 + i] = 1e-10f;
}

/*
 * eqf_rest_bias_update – Kalman measurement update on gyro bias during rest.
 *
 * Observation model:  h(X) = −A^T · a_vec  (= gyro bias)
 * Measurement:        z = gyr_lp           (LP filtered gyro ≈ true bias)
 * Innovation:         e = z − b̂ = gyr_lp + A^T · a_vec
 * Jacobian:           C = [0₃, −A^T]  (left-trivialized, rotation terms cancel)
 * InnovationLift:     Identity (no sign flip — unlike direction measurements)
 *
 * Block structure of M = I − KC:
 *   M = [[ I,  F_ ],  where F_ = Ku·A^T,  E_ = I + Kl·A^T
 *        [ 0,  E_ ]]
 */
static void eqf_rest_bias_update(void)
{
	/* b̂ = −A^T · a_vec */
	float bh[3];
	m3Tv(st.A, st.a_vec, bh);
	bh[0] = -bh[0]; bh[1] = -bh[1]; bh[2] = -bh[2];

	/* innovation: e = gyr_lp − b̂ */
	float e[3] = { rest_gyr_lp[0] - bh[0],
		       rest_gyr_lp[1] - bh[1],
		       rest_gyr_lp[2] - bh[2] };

	/* C = [0₃, Cb] where Cb = −A^T
	 * S = Cb · P22 · Cb^T + σ²I = A^T · P22 · A + σ²I */
	float P22[9];
	p_get(st.P, 1, 1, P22);

	float AT[9];
	m3_transpose(st.A, AT);

	float ATP22[9], S[9];
	m3_mul(AT, P22, ATP22);
	m3_mul(ATP22, st.A, S);
	float s2 = EQF_REST_SIGMA * EQF_REST_SIGMA;
	S[0] += s2; S[4] += s2; S[8] += s2;

	float Si[9];
	if (!m3_inv(S, Si))
		return;

	/* K = P · C^T · S⁻¹  where C^T = [[0₃], [Cb^T]] = [[0₃], [−A]]
	 * Ku = P12 · (−A) · S⁻¹,   Kl = P22 · (−A) · S⁻¹ */
	float P12[9];
	p_get(st.P, 0, 1, P12);

	float mA[9]; /* −A */
	for (int i = 0; i < 9; i++)
		mA[i] = -st.A[i];

	float P12mA[9], P22mA[9];
	m3_mul(P12, mA, P12mA);
	m3_mul(P22, mA, P22mA);

	float Ku[9], Kl[9];
	m3_mul(P12mA, Si, Ku);
	m3_mul(P22mA, Si, Kl);

	/* Correction: Δ = K · e  (identity lift — no sign flip) */
	float Delta[6];
	m3v(Ku, e, &Delta[0]);
	m3v(Kl, e, &Delta[3]);

	/* ---- state correction: X_new = exp(Δ) · X_old ---- */
	float dA[9];
	rodrigues(&Delta[0], dA);

	float J[9];
	so3_jl(&Delta[0], J);
	float da[3];
	m3v(J, &Delta[3], da);

	float A_new[9];
	m3_mul(dA, st.A, A_new);

	float dAa[3];
	m3v(dA, st.a_vec, dAa);
	st.a_vec[0] = da[0] + dAa[0];
	st.a_vec[1] = da[1] + dAa[1];
	st.a_vec[2] = da[2] + dAa[2];
	m3_copy(st.A, A_new);

	/* ---- covariance: Joseph form with M = [[I, F_], [0, E_]] ----
	 * F_ = −Ku · Cb = Ku · A^T,   E_ = I − Kl · Cb = I + Kl · A^T */
	float F_[9], E_[9];
	m3_mul(Ku, AT, F_);

	m3_mul(Kl, AT, E_);
	E_[0] += 1.0f; E_[4] += 1.0f; E_[8] += 1.0f;

	float P11[9], P21[9];
	p_get(st.P, 0, 0, P11);
	p_get(st.P, 1, 0, P21);

	/* nP11 = P11 + P12·F_^T + F_·P21 + F_·P22·F_^T + σ²·Ku·Ku^T */
	float nP11[9], T[9], T2[9];
	m3_mulBT(P12, F_, nP11);        /* P12·F_^T */
	m3_mul(F_, P21, T);             /* F_·P21 */
	for (int i = 0; i < 9; i++)
		nP11[i] += P11[i] + T[i];
	m3_mul(F_, P22, T);             /* F_·P22 */
	m3_mulBT(T, F_, T2);           /* F_·P22·F_^T */
	m3_mulBT(Ku, Ku, T);           /* Ku·Ku^T */
	for (int i = 0; i < 9; i++)
		nP11[i] += T2[i] + s2 * T[i];

	/* nP12 = (P12 + F_·P22) · E_^T + σ²·Ku·Kl^T */
	float nP12[9], FP22[9], sum12[9];
	m3_mul(F_, P22, FP22);
	for (int i = 0; i < 9; i++)
		sum12[i] = P12[i] + FP22[i];
	m3_mulBT(sum12, E_, nP12);
	m3_mulBT(Ku, Kl, T);
	for (int i = 0; i < 9; i++)
		nP12[i] += s2 * T[i];

	/* nP21 = nP12^T */
	float nP21[9];
	m3_transpose(nP12, nP21);

	/* nP22 = E_·P22·E_^T + σ²·Kl·Kl^T */
	float nP22[9], EP22[9];
	m3_mul(E_, P22, EP22);
	m3_mulBT(EP22, E_, nP22);
	m3_mulBT(Kl, Kl, T);
	for (int i = 0; i < 9; i++)
		nP22[i] += s2 * T[i];

	/* symmetrise diagonal blocks */
	m3_sym(nP11);
	m3_sym(nP22);

	p_set(st.P, 0, 0, nP11);
	p_set(st.P, 0, 1, nP12);
	p_set(st.P, 1, 0, nP21);
	p_set(st.P, 1, 1, nP22);

	/* clamp negative diagonals */
	for (int i = 0; i < 6; i++)
		if (st.P[i * 6 + i] < 1e-12f)
			st.P[i * 6 + i] = 1e-10f;
}

/* ── Public API ────────────────────────────────────────────────────── */

void eqf_init(float g_time, float a_time, float m_time)
{
	dt_gyr = g_time;
	dt_acc = a_time;
	dt_mag = m_time;

	mode = EQF_INIT;
	memset(acc_sum, 0, sizeof(acc_sum));
	memset(mag_sum, 0, sizeof(mag_sum));
	mag_norm_sum = 0.0f;
	init_count = 0;
	acc_init_count = 0;
	mag_init_count = 0;
	acc_only_init_count = 0;
	have_acc = false;
	have_mag = false;
	ortho_counter = 0;
	memset(last_a_ms2, 0, sizeof(last_a_ms2));

	/* reset rest detection */
	memset(rest_gyr_lp, 0, sizeof(rest_gyr_lp));
	memset(rest_acc_lp, 0, sizeof(rest_acc_lp));
	rest_gyr_dev = 0.0f;
	rest_t = 0.0f;
	rest_detected = false;
	rest_gyr_lp_init = false;
	rest_acc_lp_init = false;

	/* pre-fill with identity until TRIAD completes */
	m3_eye(st.A);
	memset(st.a_vec, 0, sizeof(st.a_vec));
	memset(st.P, 0, sizeof(st.P));
	for (int i = 0; i < 6; i++)
		st.P[i * 6 + i] = 0.1f;
	st.d_mag[0] = 1; st.d_mag[1] = 0; st.d_mag[2] = 0;
	st.mag_ref_norm = 1.0f;
	st.magic = 0;

	/* reset magnetic state */
	mag_ref_valid = false;
	eqf_reset_mag_runtime_state(false);
}

void eqf_load(const void *data)
{
	BUILD_ASSERT(sizeof(eqf_saved_t) <= sizeof(((struct retained_data *)0)->fusion_data),
		     "EqF state exceeds fusion_data buffer");

	memcpy(&st, data, sizeof(st));
	if (st.magic != EQF_MAGIC) {
		/* corrupt / first-boot – will re-init on next eqf_init */
		mode = EQF_INIT;
		return;
	}
	mode = EQF_RUNNING;
	ortho_counter = 0;
	memset(last_a_ms2, 0, sizeof(last_a_ms2));
	/* rest detection starts fresh each boot */
	memset(rest_gyr_lp, 0, sizeof(rest_gyr_lp));
	memset(rest_acc_lp, 0, sizeof(rest_acc_lp));
	rest_gyr_dev = 0.0f;
	rest_t = 0.0f;
	rest_detected = false;
	rest_gyr_lp_init = false;
	rest_acc_lp_init = false;
	mag_ref_valid = isfinite(st.mag_ref_norm) && st.mag_ref_norm > 0.01f;
	if (!mag_ref_valid) {
		st.d_mag[0] = 1.0f;
		st.d_mag[1] = 0.0f;
		st.d_mag[2] = 0.0f;
		st.mag_ref_norm = 1.0f;
	}
	eqf_reset_mag_runtime_state(mag_ref_valid);
}

void eqf_save(void *data)
{
	BUILD_ASSERT(sizeof(eqf_saved_t) <= sizeof(((struct retained_data *)0)->fusion_data),
		     "EqF state exceeds fusion_data buffer");
	if (mode == EQF_RUNNING)
		st.magic = EQF_MAGIC;
	else
		st.magic = 0;
	memcpy(data, &st, sizeof(st));
}

/* ── Sensor update callbacks (matching sensor_fusion_t interface) ── */

void eqf_update_gyro(float *g, float time)
{
	if (mode != EQF_RUNNING)
		return;
	float dt = eqf_resolve_dt(time, dt_gyr);

	/* convert deg/s → rad/s */
	float w[3] = { g[0] * DEG_TO_RAD, g[1] * DEG_TO_RAD, g[2] * DEG_TO_RAD };

	eqf_propagate(w, dt);

	/* ── rest detection: gyro LP + deviation ────────────────────── */
	float alpha = eqf_alpha_from_tau(EQF_REST_TAU, dt);

	if (!rest_gyr_lp_init) {
		rest_gyr_lp[0] = w[0]; rest_gyr_lp[1] = w[1]; rest_gyr_lp[2] = w[2];
		rest_gyr_lp_init = true;
	} else {
		rest_gyr_lp[0] += alpha * (w[0] - rest_gyr_lp[0]);
		rest_gyr_lp[1] += alpha * (w[1] - rest_gyr_lp[1]);
		rest_gyr_lp[2] += alpha * (w[2] - rest_gyr_lp[2]);
	}

	float dg0 = w[0] - rest_gyr_lp[0];
	float dg1 = w[1] - rest_gyr_lp[1];
	float dg2 = w[2] - rest_gyr_lp[2];
	rest_gyr_dev = dg0 * dg0 + dg1 * dg1 + dg2 * dg2;
}

void eqf_update_accel(float *a, float time)
{
	float dt = eqf_resolve_dt(time, dt_acc);

	/* store in m/s² for get_lin_a */
	last_a_ms2[0] = a[0] * CONST_EARTH_GRAVITY;
	last_a_ms2[1] = a[1] * CONST_EARTH_GRAVITY;
	last_a_ms2[2] = a[2] * CONST_EARTH_GRAVITY;

	if (mode == EQF_INIT) {
		acc_sum[0] += a[0]; acc_sum[1] += a[1]; acc_sum[2] += a[2];
		acc_init_count++;
		have_acc = true;
		if (have_mag)
			init_count++;
		if (init_count >= EQF_INIT_SAMPLES && acc_init_count > 0 && mag_init_count > 0) {
			float acc_inv = 1.0f / (float)acc_init_count;
			float mag_inv = 1.0f / (float)mag_init_count;
			float aa[3] = { acc_sum[0] * acc_inv, acc_sum[1] * acc_inv, acc_sum[2] * acc_inv };
			float mm[3] = { mag_sum[0] * mag_inv, mag_sum[1] * mag_inv, mag_sum[2] * mag_inv };
			triad_init(aa, mm, mag_norm_sum * mag_inv);
			mode = EQF_RUNNING;
		}
		/* fallback: accel-only init if mag never appears */
		if (mode == EQF_INIT) {
			acc_only_init_count++;
			if (!have_mag && acc_only_init_count >= EQF_INIT_SAMPLES * 2) {
				float inv = 1.0f / (float)acc_only_init_count;
				float aa[3] = { acc_sum[0] * inv, acc_sum[1] * inv, acc_sum[2] * inv };
				gravity_init(aa);
				mode = EQF_RUNNING;
			}
		} else {
			acc_only_init_count = 0;
		}
		return;
	}

	/* adaptive accel noise:  σ × (1 + k·(|a|−1)²) */
	float anorm = v3_norm(a);
	float dev = anorm - 1.0f;
	float base_sigma = EQF_SIGMA_ACC;
	/* In 6-axis, inflate σ_acc to weaken accel influence overall */
	if (!mag_active)
		base_sigma *= 50.0f;
	float sigma = base_sigma * (1.0f + EQF_ACC_ADAPT_K * dev * dev);

	/* skip wildly-anomalous readings */
	if (anorm < 0.1f || anorm > 5.0f)
		return;

	static const float d_acc[3] = { 0.0f, 0.0f, 1.0f };
	eqf_dir_update(a, d_acc, sigma, !mag_active);

	/* ── rest detection: accel LP + combined threshold check ───── */
	float alpha = eqf_alpha_from_tau(EQF_REST_TAU, dt);

	if (!rest_acc_lp_init) {
		rest_acc_lp[0] = a[0]; rest_acc_lp[1] = a[1]; rest_acc_lp[2] = a[2];
		rest_acc_lp_init = true;
	} else {
		rest_acc_lp[0] += alpha * (a[0] - rest_acc_lp[0]);
		rest_acc_lp[1] += alpha * (a[1] - rest_acc_lp[1]);
		rest_acc_lp[2] += alpha * (a[2] - rest_acc_lp[2]);
	}

	float da0 = a[0] - rest_acc_lp[0];
	float da1 = a[1] - rest_acc_lp[1];
	float da2 = a[2] - rest_acc_lp[2];
	float acc_dev_sq = da0 * da0 + da1 * da1 + da2 * da2;

	float gyr_th_rad = EQF_REST_TH_GYR * DEG_TO_RAD;
	float max_bias_rad = EQF_REST_MAX_BIAS * DEG_TO_RAD;

	/* reject rest if: gyro deviation too high, accel deviation too high,
	 * accel norm not near 1g, or LP'd gyro exceeds max plausible bias */
	if (rest_gyr_dev >= gyr_th_rad * gyr_th_rad
	    || acc_dev_sq >= EQF_REST_TH_ACC * EQF_REST_TH_ACC
	    || fabsf(anorm - 1.0f) > EQF_REST_ACC_NORM_TH
	    || fabsf(rest_gyr_lp[0]) > max_bias_rad
	    || fabsf(rest_gyr_lp[1]) > max_bias_rad
	    || fabsf(rest_gyr_lp[2]) > max_bias_rad) {
		rest_t = 0.0f;
		rest_detected = false;
	} else {
		rest_t += dt;
		if (rest_t >= EQF_REST_MIN_T)
			rest_detected = true;
	}

	/* apply bias measurement update during rest */
	if (rest_detected && rest_gyr_lp_init)
		eqf_rest_bias_update();
}

void eqf_update_mag(float *m, float time)
{
	float dt = eqf_resolve_dt(time, dt_mag);
	float mn = v3_norm(m);
	if (mn < 1e-10f)
		return;

	if (mode == EQF_INIT) {
		mag_sum[0] += m[0]; mag_sum[1] += m[1]; mag_sum[2] += m[2];
		mag_norm_sum += mn;
		mag_init_count++;
		have_mag = true;
		return;
	}

#if !EQF_MAG_DIST_REJECTION_ENABLED
	mag_ref_valid = true;
	mag_dist_detected = false;
	mag_active = true;
	eqf_dir_update(m, st.d_mag, EQF_SIGMA_MAG, false);
	return;
#else
	float curr_norm, curr_dip;
	eqf_get_mag_norm_dip(m, &curr_norm, &curr_dip);

	if (!mag_norm_dip_lp_init) {
		mag_norm_lp = curr_norm;
		mag_dip_lp = curr_dip;
		mag_norm_dip_lp_init = true;
	} else if (EQF_MAG_CURRENT_TAU > 0.0f) {
		float alpha = eqf_alpha_from_tau(EQF_MAG_CURRENT_TAU, dt);
		mag_norm_lp += alpha * (curr_norm - mag_norm_lp);
		mag_dip_lp += alpha * (curr_dip - mag_dip_lp);
	}
	curr_norm = mag_norm_lp;
	curr_dip = mag_dip_lp;

	if (!mag_ref_valid) {
		eqf_set_mag_ref(curr_norm, curr_dip);
		mag_ref_valid = true;
		eqf_reset_mag_runtime_state(true);
		eqf_dir_update(m, st.d_mag, EQF_SIGMA_MAG, false);
		return;
	}

	if (mag_ref_valid) {
		float ref_norm_th = EQF_MAG_NORM_TH * fmaxf(st.mag_ref_norm, 0.01f);

		if (fabsf(curr_norm - st.mag_ref_norm) < ref_norm_th) {
			mag_undisturbed_t += dt;
			if (mag_undisturbed_t >= EQF_MAG_MIN_UNDISTURBED_T) {
				float alpha = eqf_alpha_from_tau(EQF_MAG_REF_TAU, dt);
				st.mag_ref_norm += alpha * (curr_norm - st.mag_ref_norm);
				mag_dist_detected = false;
			}
		} else {
			mag_undisturbed_t = 0.0f;
			mag_dist_detected = true;
		}
	} else {
		mag_undisturbed_t = 0.0f;
		mag_dist_detected = true;
	}

	float sigma_mag = EQF_SIGMA_MAG;
	bool allow_update = mag_ref_valid;

	if (mag_ref_valid && mag_dist_detected) {
		mag_reject_t = fminf(mag_reject_t + dt, EQF_MAG_MAX_REJECTION_TIME);
		sigma_mag *= EQF_MAG_REJECTION_SIGMA_SCALE;
	} else if (mag_ref_valid) {
		mag_reject_t = fmaxf(mag_reject_t - EQF_MAG_REJECTION_FACTOR * dt, 0.0f);
	}

	mag_active = mag_ref_valid && !mag_dist_detected;
	if (allow_update) {
		eqf_dir_update(m, st.d_mag, sigma_mag, false);
	}
#endif
}

void eqf_update(float *g, float *a, float *m, float time)
{
	if (g[0] != 0 || g[1] != 0 || g[2] != 0)
		eqf_update_gyro(g, time);
	eqf_update_accel(a, time);
	eqf_update_mag(m, time);
}

/* ── State queries ─────────────────────────────────────────────────── */

void eqf_get_quat(float *q)
{
	mat_to_quat(st.A, q);
	/* normalize for safety */
	float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	if (n > 0.0f) {
		float inv = 1.0f / n;
		q[0] *= inv; q[1] *= inv; q[2] *= inv; q[3] *= inv;
	}
}

void eqf_get_gyro_bias(float *g_off)
{
	/* b̂ = −A^T · a_vec  (internal: rad/s → interface: deg/s) */
	float bh[3];
	m3Tv(st.A, st.a_vec, bh);
	g_off[0] = -bh[0] * RAD_TO_DEG;
	g_off[1] = -bh[1] * RAD_TO_DEG;
	g_off[2] = -bh[2] * RAD_TO_DEG;
}

void eqf_set_gyro_bias(float *g_off)
{
	/* given b in deg/s → set a_vec = −A · b_rad */
	float b_rad[3] = {
		g_off[0] * DEG_TO_RAD,
		g_off[1] * DEG_TO_RAD,
		g_off[2] * DEG_TO_RAD
	};
	float Ab[3];
	m3v(st.A, b_rad, Ab);
	st.a_vec[0] = -Ab[0];
	st.a_vec[1] = -Ab[1];
	st.a_vec[2] = -Ab[2];
}

void eqf_get_lin_a(float *lin_a)
{
	/* gravity direction in body frame = A^T · [0,0,1] = row 2 of A */
	float grav[3] = { st.A[6], st.A[7], st.A[8] };

	lin_a[0] = last_a_ms2[0] - grav[0] * CONST_EARTH_GRAVITY;
	lin_a[1] = last_a_ms2[1] - grav[1] * CONST_EARTH_GRAVITY;
	lin_a[2] = last_a_ms2[2] - grav[2] * CONST_EARTH_GRAVITY;
}

void eqf_update_gyro_sanity(float *g, float *m)
{
	(void)g; (void)m;
}

int eqf_get_gyro_sanity(void)
{
	return 0;
}


/* ── Fusion vtable ─────────────────────────────────────────────────── */

const sensor_fusion_t sensor_fusion_eqf = {
	.init             = eqf_init,
	.load             = eqf_load,
	.save             = eqf_save,
	.update_gyro      = eqf_update_gyro,
	.update_accel     = eqf_update_accel,
	.update_mag       = eqf_update_mag,
	.update           = eqf_update,
	.get_gyro_bias    = eqf_get_gyro_bias,
	.set_gyro_bias    = eqf_set_gyro_bias,
	.update_gyro_sanity = eqf_update_gyro_sanity,
	.get_gyro_sanity  = eqf_get_gyro_sanity,
	.get_lin_a        = eqf_get_lin_a,
	.get_quat         = eqf_get_quat,
};
