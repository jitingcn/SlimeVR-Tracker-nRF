#include "sensor/raw_collection.h"
#include "connection/connection.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

static bool single_active;
static bool batch_active;
static struct raw_imu_sample samples[32];
static unsigned sample_count;
static unsigned metadata_count;
static float metadata_send_hz;
static float metadata_chip_hz;
static float metadata_fusion_hz;
static float metadata_mag_hz;
static const float zero[3];
static const float gyro_z[3] = {0.0f, 0.0f, 90.0f};

bool connection_get_data_collection(void)
{
	return single_active;
}

bool connection_get_data_collection_batch(void)
{
	return batch_active;
}

void connection_queue_raw_sample(const struct raw_imu_sample *sample)
{
	assert(metadata_count != 0);
	assert(sample_count < sizeof(samples) / sizeof(samples[0]));
	samples[sample_count++] = *sample;
}

void connection_send_raw_metadata(float gyro_range, float accel_range,
	float gyro_odr, float accel_odr, float mag_odr, uint8_t imu_id, uint8_t mag_id,
	float chip_gyro_hz, float fusion_gyro_hz)
{
	(void)gyro_range;
	(void)accel_range;
	(void)accel_odr;
	(void)imu_id;
	(void)mag_id;
	metadata_count++;
	metadata_send_hz = gyro_odr;
	metadata_chip_hz = chip_gyro_hz;
	metadata_fusion_hz = fusion_gyro_hz;
	metadata_mag_hz = mag_odr;
}

static void near(float actual, float expected)
{
	assert(fabsf(actual - expected) < 0.00001f);
}

static struct sensor_raw_collection_config setup(void)
{
	single_active = false;
	batch_active = false;
	sensor_set_batch_collect(false, 0.0f);
	sample_count = 0;
	metadata_count = 0;
	memset(samples, 0, sizeof(samples));
	return (struct sensor_raw_collection_config){
		.gyro_period = 0.1f,
		.accel_period = 0.2f,
		.mag_period = 0.5f,
		.gyro_range = 2000.0f,
		.accel_range = 16.0f,
		.gyro_oversample_n = 1,
		.imu_id = 7,
		.mag_id = 2,
		.mag_active = true,
	};
}

static bool begin(const struct sensor_raw_collection_config *config, bool expected_restart)
{
	bool active;
	bool restart = sensor_raw_collection_begin_frame(&active);
	assert(restart == expected_restart);
	if (restart) {
		/* Production caller resumes hardware here, before publishing metadata. */
		sensor_raw_collection_send_metadata(config);
	}
	return active;
}

static void test_quaternion_order_and_accel_latch(void)
{
	struct sensor_raw_collection_config config = setup();
	config.gyro_period = 1.0f;
	single_active = true;
	assert(begin(&config, true));
	const float accel[3] = {1.0f, 2.0f, 3.0f};
	const float gyro_x[3] = {90.0f, 0.0f, 0.0f};
	const float gyro_y[3] = {0.0f, 90.0f, 0.0f};
	sensor_raw_collection_on_sample(accel, zero, NAN, true);
	assert(sample_count == 0);
	sensor_raw_collection_on_sample(zero, gyro_x, 27.0f, true);
	assert(sample_count == 1);
	near(samples[0].gyr_quat[0], sqrtf(0.5f));
	near(samples[0].gyr_quat[1], sqrtf(0.5f));
	near(samples[0].accel[0], 1.0f);
	near(samples[0].accel[1], 2.0f);
	near(samples[0].accel[2], 3.0f);
	near(samples[0].temp_c, 27.0f);
	sensor_raw_collection_on_sample(zero, gyro_y, NAN, true);
	assert(sample_count == 2);
	/* qx*qy, not qy*qx: the z component must be positive. */
	for (unsigned i = 0; i < 4; i++) {
		near(samples[1].gyr_quat[i], 0.5f);
	}
	for (unsigned i = 0; i < 3; i++) {
		near(samples[1].accel[i], 0.0f);
	}
	assert(isnan(samples[1].temp_c));
}

static void test_fractional_emit_and_in_place_rate_change(void)
{
	struct sensor_raw_collection_config config = setup();
	config.gyro_oversample_n = 2;
	batch_active = true;
	sensor_set_batch_collect(true, 4.9f); /* Existing integer-Hz publication. */
	assert(begin(&config, true));
	near(metadata_send_hz, 4.0f);
	near(metadata_chip_hz, 10.0f);
	near(metadata_fusion_hz, 5.0f);
	const unsigned emitted_at[] = {3, 5, 8, 10};
	unsigned expected_count = 0;
	for (unsigned tick = 1; tick <= 10; tick++) {
		sensor_raw_collection_on_sample(zero, gyro_z, 20.0f, true);
		if (expected_count < 4 && tick == emitted_at[expected_count]) {
			expected_count++;
		}
		assert(sample_count == expected_count);
	}
	near(samples[3].gyr_quat[0], sqrtf(0.5f));
	near(samples[3].gyr_quat[3], sqrtf(0.5f));
	assert(begin(&config, false));
	assert(metadata_count == 1);
	/* Leave two chip samples in the old fractional window before reconfiguring. */
	sensor_raw_collection_on_sample(zero, gyro_z, 20.0f, true);
	sensor_raw_collection_on_sample(zero, gyro_z, 20.0f, true);
	assert(sample_count == 4);
	sensor_set_batch_collect(true, 5.0f);
	assert(begin(&config, true));
	sensor_raw_collection_on_sample(zero, gyro_z, 20.0f, true);
	assert(sample_count == 4);
	sensor_raw_collection_on_sample(zero, gyro_z, 20.0f, true);
	assert(sample_count == 5);
	/* Configuration change restarts quaternion integration as well as phase. */
	near(samples[4].gyr_quat[0], cosf(0.15707963267948966f));
	near(samples[4].gyr_quat[3], sinf(0.15707963267948966f));
}

static void test_session_preserves_single_emit_phase_and_accel(void)
{
	struct sensor_raw_collection_config config = setup();
	config.gyro_oversample_n = 3;
	single_active = true;
	assert(begin(&config, true));
	const float accel[3] = {0.0f, 0.0f, 1.0f};
	sensor_raw_collection_on_sample(accel, gyro_z, 20.0f, true);
	assert(sample_count == 0);
	single_active = false;
	assert(!begin(&config, false));
	/* No inactive gyro tag: the latest accel remains latched across activation. */
	single_active = true;
	assert(begin(&config, true));
	sensor_raw_collection_on_sample(zero, gyro_z, 20.0f, true);
	assert(sample_count == 0);
	sensor_raw_collection_on_sample(zero, gyro_z, 20.0f, true);
	assert(sample_count == 1);
	near(samples[0].accel[2], 1.0f);
	near(samples[0].gyr_quat[0], cosf(0.15707963267948966f));
	near(samples[0].gyr_quat[3], sinf(0.15707963267948966f));
}

static void test_inactive_tags_and_metadata_rate_limits(void)
{
	struct sensor_raw_collection_config config = setup();
	const float accel[3] = {1.0f, 0.0f, 0.0f};
	sensor_raw_collection_on_sample(accel, zero, NAN, false);
	sensor_raw_collection_on_sample(zero, gyro_z, NAN, false);
	assert(sample_count == 0);
	batch_active = true;
	sensor_set_batch_collect(true, 100.0f);
	assert(begin(&config, true));
	near(metadata_send_hz, 10.0f);
	near(metadata_mag_hz, 2.0f);
	sensor_raw_collection_on_sample(zero, gyro_z, 20.0f, true);
	assert(sample_count == 1);
	near(samples[0].accel[0], 0.0f);
	sensor_set_batch_collect(true, 0.0f);
	config.mag_active = false;
	assert(begin(&config, true));
	near(metadata_send_hz, 5.0f);
	near(metadata_mag_hz, 0.0f);
	sensor_raw_collection_on_sample(zero, gyro_z, 20.0f, true);
	assert(sample_count == 1);
	sensor_raw_collection_on_sample(zero, gyro_z, 20.0f, true);
	assert(sample_count == 2);
	/* Batch->single cutover is a generation change even though still active. */
	batch_active = false;
	single_active = true;
	sensor_set_batch_collect(false, 0.0f);
	config.gyro_oversample_n = 0;
	assert(begin(&config, true));
	near(metadata_send_hz, 10.0f);
	sensor_raw_collection_on_sample(zero, gyro_z, 20.0f, true);
	assert(sample_count == 3);
	near(samples[2].gyr_quat[0], cosf(0.07853981633974483f));
	near(samples[2].gyr_quat[3], sinf(0.07853981633974483f));
}

static void test_metadata_refresh_preserves_active_integration(void)
{
	struct sensor_raw_collection_config config = setup();
	batch_active = true;
	sensor_set_batch_collect(true, 2.0f);
	assert(begin(&config, true));
	const float accel[3] = {1.0f, 2.0f, 3.0f};
	for (unsigned i = 0; i < 4; i++) {
		sensor_raw_collection_on_sample(accel, gyro_z, 20.0f, true);
	}
	assert(sample_count == 0);
	/* Hardware reinitialization refreshes metadata without a new transport session. */
	sensor_raw_collection_send_metadata(&config);
	assert(begin(&config, false));
	near(metadata_send_hz, 2.0f);
	sensor_raw_collection_on_sample(zero, gyro_z, 20.0f, true);
	assert(sample_count == 1);
	near(samples[0].gyr_quat[0], cosf(0.39269908169872415f));
	near(samples[0].gyr_quat[3], sinf(0.39269908169872415f));
	near(samples[0].accel[0], 1.0f);
	near(samples[0].accel[1], 2.0f);
	near(samples[0].accel[2], 3.0f);
}

static void run_isolated(void (*test)(void))
{
	pid_t child = fork();
	assert(child >= 0);
	if (child == 0) {
		test();
		_exit(0);
	}
	int status;
	assert(waitpid(child, &status, 0) == child);
	assert(WIFEXITED(status) && WEXITSTATUS(status) == 0);
}

int main(void)
{
	run_isolated(test_quaternion_order_and_accel_latch);
	run_isolated(test_fractional_emit_and_in_place_rate_change);
	run_isolated(test_session_preserves_single_emit_phase_and_accel);
	run_isolated(test_inactive_tags_and_metadata_rate_limits);
	run_isolated(test_metadata_refresh_preserves_active_integration);
	puts("raw collection host tests passed");
	return 0;
}
