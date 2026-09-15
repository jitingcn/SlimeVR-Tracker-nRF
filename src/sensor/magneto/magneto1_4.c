// magneto 1.4 magnetometer/accelerometer calibration code
// from http://sailboatinstruments.blogspot.com/2011/08/improved-magnetometer-calibration.html

#include <errno.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <zephyr/kernel.h>

#include "mymathlib_matrix.h"

#include "magneto1_4.h"

/* Each call owns one heap allocation. Only S22a, SS and v1 cross phases:
 * reduction -> eigenproblem -> reconstruction. Union members are never live
 * across a phase transition, so speculative/manual fits do not share scratch.
 */
struct magneto_workspace {
    double S22a[24], SS[36], v1[6];
    int perm[6];
    union {
        struct {
            double S11[36], S12[24], S12t[24], S22[16], S22b[36];
        } reduction;
        struct {
            double C[36], E[36], SSS[36], eigen_real[6], eigen_imag[6];
        } eigen;
        struct {
            double v2[4], U[3], Q[9], v[10], B[3], Q_1[9], QB[3];
            double SSSS[9], Dz[9], eigen_real3[3], eigen_imag3[3];
            double SQ[9], vdz[9], A_1[9];
        } reconstruction;
    } phase;
};

static bool finite_values(const double *values, size_t count)
{
    for (size_t i = 0; i < count; i++) {
        if (!isfinite(values[i]))
            return false;
    }
    return true;
}

void magneto_sample(double x, double y, double z, double *ata, double *norm_sum, double *sample_count)
{
    *sample_count += 1.0;
    *norm_sum += sqrt(x * x + y * y + z * z);

    double D[10] = {
        x * x,
        y * y,
        z * z,
        2.0 * y * z,
        2.0 * x * z,
        2.0 * x * y,
        2.0 * x,
        2.0 * y,
        2.0 * z,
        1.0};

    Multiply_Self_Transpose(ata, D, 10, 1);
}

int magneto_current_calibration(float BAinv[4][3], double *ata, double norm_sum, double sample_count)
{
    if (BAinv == NULL || ata == NULL || !isfinite(norm_sum) || norm_sum <= 0.0
        || !isfinite(sample_count) || sample_count <= 0.0 || !finite_values(ata, 100))
        return -EINVAL;

    double hm = norm_sum / sample_count;
    if (!isfinite(hm) || hm <= 0.0)
        return -ERANGE;

    struct magneto_workspace *workspace = k_malloc(sizeof(*workspace));
    if (workspace == NULL)
        return -ENOMEM;
    int err = -EDOM;

    double *S11 = workspace->phase.reduction.S11;
    Get_Submatrix(S11, 6, 6, ata, 10, 0, 0);
    double *S12 = workspace->phase.reduction.S12;
    Get_Submatrix(S12, 6, 4, ata, 10, 0, 6);
    double *S12t = workspace->phase.reduction.S12t;
    Get_Submatrix(S12t, 4, 6, ata, 10, 6, 0);
    double *S22 = workspace->phase.reduction.S22;
    Get_Submatrix(S22, 4, 4, ata, 10, 6, 6);


    if (Choleski_LU_Decomposition(S22, 4) < 0 || !finite_values(S22, 16)
        || Choleski_LU_Inverse(S22, 4) < 0 || !finite_values(S22, 16))
        goto cleanup;

    // Calculate S22a = S22 * S12t   4*6 = 4x4 * 4x6   C = AB
    double *S22a = workspace->S22a;
    Multiply_Matrices(S22a, S22, 4, 4, S12t, 6);

    // Then calculate S22b = S12 * S22a      ( 6x6 = 6x4 * 4x6)
    double *S22b = workspace->phase.reduction.S22b;
    Multiply_Matrices(S22b, S12, 6, 4, S22a, 6);

    // Calculate SS = S11 - S22b
    double *SS = workspace->SS;
    for (int i = 0; i < 36; i++)
        SS[i] = S11[i] - S22b[i];
    if (!finite_values(S22a, 24) || !finite_values(SS, 36))
        goto cleanup;

    // Create pre-inverted constraint matrix C
    double *C = workspace->phase.eigen.C;
    C[0] = 0.0;
    C[1] = 0.5;
    C[2] = 0.5;
    C[3] = 0.0;
    C[4] = 0.0;
    C[5] = 0.0;
    C[6] = 0.5;
    C[7] = 0.0;
    C[8] = 0.5;
    C[9] = 0.0;
    C[10] = 0.0;
    C[11] = 0.0;
    C[12] = 0.5;
    C[13] = 0.5;
    C[14] = 0.0;
    C[15] = 0.0;
    C[16] = 0.0;
    C[17] = 0.0;
    C[18] = 0.0;
    C[19] = 0.0;
    C[20] = 0.0;
    C[21] = -0.25;
    C[22] = 0.0;
    C[23] = 0.0;
    C[24] = 0.0;
    C[25] = 0.0;
    C[26] = 0.0;
    C[27] = 0.0;
    C[28] = -0.25;
    C[29] = 0.0;
    C[30] = 0.0;
    C[31] = 0.0;
    C[32] = 0.0;
    C[33] = 0.0;
    C[34] = 0.0;
    C[35] = -0.25;
    double *E = workspace->phase.eigen.E;
    Multiply_Matrices(E, C, 6, 6, SS, 6);

    double *SSS = workspace->phase.eigen.SSS;
    if (!finite_values(E, 36)
        || Hessenberg_Form_Elementary(E, SSS, 6, workspace->perm) < 0
        || !finite_values(E, 36) || !finite_values(SSS, 36))
        goto cleanup;

    int index = 0;
    {
        double *eigen_real = workspace->phase.eigen.eigen_real;
        double *eigen_imag = workspace->phase.eigen.eigen_imag;

        if (QR_Hessenberg_Matrix(E, SSS, eigen_real, eigen_imag, 6, 100) < 0) {
            err = -EAGAIN;
            goto cleanup;
        }
        if (!finite_values(eigen_real, 6) || !finite_values(eigen_imag, 6)
            || !finite_values(SSS, 36))
            goto cleanup;

        double maxval = eigen_real[0];
        for (int i = 1; i < 6; i++)
        {
            if (eigen_real[i] > maxval)
            {
                maxval = eigen_real[i];
                index = i;
            }
        }
        if (eigen_imag[index] != 0.0)
            goto cleanup;
    }

    double *v1 = workspace->v1;
    v1[0] = SSS[index];
    v1[1] = SSS[index + 6];
    v1[2] = SSS[index + 12];
    v1[3] = SSS[index + 18];
    v1[4] = SSS[index + 24];
    v1[5] = SSS[index + 30];

    // normalize v1
    {
        double norm = sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2] + v1[3] * v1[3] + v1[4] * v1[4] + v1[5] * v1[5]);
        if (!isfinite(norm) || norm <= 0.0)
            goto cleanup;
        v1[0] /= norm;
        v1[1] /= norm;
        v1[2] /= norm;
        v1[3] /= norm;
        v1[4] /= norm;
        v1[5] /= norm;
    }

    if (v1[0] < 0.0)
    {
        v1[0] = -v1[0];
        v1[1] = -v1[1];
        v1[2] = -v1[2];
        v1[3] = -v1[3];
        v1[4] = -v1[4];
        v1[5] = -v1[5];
    }

    // Calculate v2 = S22a * v1      ( 4x1 = 4x6 * 6x1)
    double *v2 = workspace->phase.reconstruction.v2;
    Multiply_Matrices(v2, S22a, 4, 6, v1, 1);
    if (!finite_values(v2, 4))
        goto cleanup;

    double *U = workspace->phase.reconstruction.U;
    double *Q = workspace->phase.reconstruction.Q;
    double J;
    {
        double *v = workspace->phase.reconstruction.v;
        v[0] = v1[0];
        v[1] = v1[1];
        v[2] = v1[2];
        v[3] = v1[3];
        v[4] = v1[4];
        v[5] = v1[5];
        v[6] = -v2[0];
        v[7] = -v2[1];
        v[8] = -v2[2];
        v[9] = -v2[3];

        Q[0] = v[0];
        Q[1] = v[5];
        Q[2] = v[4];
        Q[3] = v[5];
        Q[4] = v[1];
        Q[5] = v[3];
        Q[6] = v[4];
        Q[7] = v[3];
        Q[8] = v[2];

        U[0] = v[6];
        U[1] = v[7];
        U[2] = v[8];

        J = v[9];
    }

    double *B = workspace->phase.reconstruction.B;
    {
        double *Q_1 = workspace->phase.reconstruction.Q_1;
        for (int i = 0; i < 9; i++)
            Q_1[i] = Q[i];
        if (Choleski_LU_Decomposition(Q_1, 3) < 0 || !finite_values(Q_1, 9)
            || Choleski_LU_Inverse(Q_1, 3) < 0 || !finite_values(Q_1, 9))
            goto cleanup;

        // Calculate B = Q-1 * U   ( 3x1 = 3x3 * 3x1)
        Multiply_Matrices(B, Q_1, 3, 3, U, 1);
        B[0] = -B[0]; // x-axis combined bias
        B[1] = -B[1]; // y-axis combined bias
        B[2] = -B[2]; // z-axis combined bias
    }

    // First calculate QB = Q * B   ( 3x1 = 3x3 * 3x1)
    double btqb;
    {
        double *QB = workspace->phase.reconstruction.QB;
        Multiply_Matrices(QB, Q, 3, 3, B, 1);

        // Then calculate btqb = BT * QB    ( 1x1 = 1x3 * 3x1)
        Multiply_Matrices(&btqb, B, 1, 3, QB, 1);
    }

    // Calculate SQ, the square root of matrix Q
    double *SSSS = workspace->phase.reconstruction.SSSS;
    if (!finite_values(B, 3) || !isfinite(btqb) || !isfinite(btqb - J) || btqb - J <= 0.0
        || Hessenberg_Form_Elementary(Q, SSSS, 3, workspace->perm) < 0
        || !finite_values(Q, 9) || !finite_values(SSSS, 9))
        goto cleanup;

    double *Dz = workspace->phase.reconstruction.Dz;
    for (int i = 0; i < 9; i++)
    {
        Dz[i] = 0;
    }
    {
        double *eigen_real3 = workspace->phase.reconstruction.eigen_real3;
        double *eigen_imag3 = workspace->phase.reconstruction.eigen_imag3;
        if (QR_Hessenberg_Matrix(Q, SSSS, eigen_real3, eigen_imag3, 3, 100) < 0) {
            err = -EAGAIN;
            goto cleanup;
        }
        if (!finite_values(eigen_real3, 3) || !finite_values(eigen_imag3, 3)
            || !finite_values(SSSS, 9))
            goto cleanup;
        for (int i = 0; i < 3; i++) {
            if (eigen_real3[i] <= 0.0 || eigen_imag3[i] != 0.0)
                goto cleanup;
        }

        Dz[0] = sqrt(eigen_real3[0]);
        Dz[4] = sqrt(eigen_real3[1]);
        Dz[8] = sqrt(eigen_real3[2]);
    }

    {
        // normalize eigenvectors
        double norm = sqrt(SSSS[0] * SSSS[0] + SSSS[3] * SSSS[3] + SSSS[6] * SSSS[6]);
        if (!isfinite(norm) || norm <= 0.0)
            goto cleanup;
        SSSS[0] /= norm;
        SSSS[3] /= norm;
        SSSS[6] /= norm;
        norm = sqrt(SSSS[1] * SSSS[1] + SSSS[4] * SSSS[4] + SSSS[7] * SSSS[7]);
        if (!isfinite(norm) || norm <= 0.0)
            goto cleanup;
        SSSS[1] /= norm;
        SSSS[4] /= norm;
        SSSS[7] /= norm;
        norm = sqrt(SSSS[2] * SSSS[2] + SSSS[5] * SSSS[5] + SSSS[8] * SSSS[8]);
        if (!isfinite(norm) || norm <= 0.0)
            goto cleanup;
        SSSS[2] /= norm;
        SSSS[5] /= norm;
        SSSS[8] /= norm;
    }

    double *SQ = workspace->phase.reconstruction.SQ;
    {
        double *vdz = workspace->phase.reconstruction.vdz;
        Multiply_Matrices(vdz, SSSS, 3, 3, Dz, 3);
        Transpose_Square_Matrix(SSSS, 3);
        Multiply_Matrices(SQ, vdz, 3, 3, SSSS, 3);
    }

    double *A_1 = workspace->phase.reconstruction.A_1;
    // Calculate hmb = sqrt(btqb - J).
    double hmb = sqrt(btqb - J);

    for (int i = 0; i < 9; i++)
        A_1[i] = SQ[i] * hm / hmb;

    // Check the entire candidate before the first caller-visible write. A
    // finite double can still overflow or lose its positive diagonal in float.
    for (int i = 0; i < 3; i++) {
        if (!isfinite(B[i]) || fabs(B[i]) > FLT_MAX) {
            err = -ERANGE;
            goto cleanup;
        }
    }
    for (int i = 0; i < 9; i++) {
        if (!isfinite(A_1[i]) || fabs(A_1[i]) > FLT_MAX) {
            err = -ERANGE;
            goto cleanup;
        }
    }
    for (int i = 0; i < 3; i++) {
        if ((float)A_1[i * 3 + i] <= 0.0f) {
            err = -ERANGE;
            goto cleanup;
        }
    }

    for (int i = 0; i < 3; i++)
        BAinv[0][i] = B[i];

    for (int i = 0; i < 3; i++)
    {
        BAinv[i + 1][0] = A_1[i * 3];
        BAinv[i + 1][1] = A_1[i * 3 + 1];
        BAinv[i + 1][2] = A_1[i * 3 + 2];
    }
    err = 0;

cleanup:
    k_free(workspace);
    return err;
}
