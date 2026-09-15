void magneto_sample(double x, double y, double z, double* ata, double* norm_sum, double* sample_count);
/*
 * Returns 0 on success; negative errno on invalid input (-EINVAL), allocation
 * failure (-ENOMEM), degenerate/non-finite fit (-EDOM), QR nonconvergence
 * (-EAGAIN), or unrepresentable scale/output (-ERANGE).
 * BAinv is unchanged on every failure. ATA is never modified.
 */
int magneto_current_calibration(float BAinv[4][3], double* ata, double norm_sum, double sample_count);