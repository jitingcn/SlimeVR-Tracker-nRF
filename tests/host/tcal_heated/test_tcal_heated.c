/* Included after complete production tcal_heated.c and exact runtime accumulator
 * definitions. Only hardware, time, storage and shared-dispatcher boundaries are
 * adapted. imu_calibration.c is separately linked for real exclusivity. */
#include "sensor/calibration/imu_calibration.h"
#include "connection/tracker_events.h"
#include <assert.h>
#include <stdlib.h>

static struct retained_data storage;
struct retained_data *retained = &storage;
static int64_t now_ms = 10000;
static struct sensor_temperature_observation sample = {25.0f, 25.0f, 10000, 1};
static bool external_power = true, power_ready = true, hw_available = true;
static bool accel_available = true, hw_expiry, ota_active, ota_suppressed;
static int64_t accel_age;
static float accel[3] = {0, 0, 1};
static float gyro_odr = 1000.0f;
static int sample_error, arm_error, write_error, off_error;
static unsigned gate_depth, model_depth, storage_depth;
static bool imu_reserved, reset_requested;
static uint32_t model_generation = 17, armed_epoch;
static uint16_t output_duty;
static int64_t lease_deadline;
static unsigned writes, arms, offs, feeds, resets, partial_finishes, refreshes, dirty_marks, scheduled;
static void (*on_storage_begin)(void);
static bool expect_publication;
static float publication_tolerance;
static bool maintenance_active;
static struct TempCalPoint expected_points[TCAL_BUFFER_SIZE];
static struct TempCalPoint old_points[TCAL_BUFFER_SIZE];
static const float bias[3] = {0.01f, -0.02f, 0.03f};
static const float zero[3];

static bool expected_model_matches(void)
{
	if (!publication_tolerance)
		return !memcmp(retained->tempCalPoints, expected_points, sizeof(expected_points));
	for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
		if (fabsf(retained->tempCalPoints[i].temp - expected_points[i].temp) >= 0.0001f)
			return false;
		for (int axis = 0; axis < 3; axis++)
			if (fabsf(retained->tempCalPoints[i].bias[axis] - expected_points[i].bias[axis]) >=
			    publication_tolerance)
				return false;
	}
	return true;
}

int64_t k_uptime_get(void) { return now_ms; }

/* Semantic LED boundary. Submission is observed per semantic; hardware
 * ownership is asserted where the owner publishes protection facts. */
static unsigned led_results[LED_SEMANTIC_COUNT];
static unsigned led_states[LED_SEMANTIC_COUNT];
static unsigned led_requests[LED_SEMANTIC_COUNT];
static unsigned led_faults, led_operations;
static bool led_fault_safety;
static uint32_t led_identity;
static uint32_t warm_armed;
static unsigned warm_arms;

uint32_t led_request_id(void) { return ++led_identity; }
uint32_t led_event_id(void) { return ++led_identity; }
struct led_token led_begin(enum led_owner owner, uint32_t request_id)
{
	return (struct led_token){ .owner = owner, .session = ++led_identity, .request_id = request_id };
}
enum led_admission led_state(struct led_token token, uint32_t revision, enum led_semantic semantic)
{
	(void)token;
	(void)revision;
	led_states[semantic]++;
	return LED_ADMITTED;
}
enum led_admission led_result(struct led_token token, uint32_t event_id, enum led_semantic semantic)
{
	(void)event_id;
	if (token.session) {
		led_results[semantic]++;
	}
	return LED_ADMITTED;
}
enum led_admission led_request_event(enum led_owner owner, uint32_t request_id, uint32_t event_id,
				     enum led_semantic semantic)
{
	(void)owner;
	(void)request_id;
	(void)event_id;
	led_requests[semantic]++;
	return LED_ADMITTED;
}
void led_fault_publish(enum led_owner owner, enum led_fault_kind fault, uint32_t protection_id)
{
	(void)owner;
	bool safety = fault == LED_FAULT_SAFETY;
	(void)protection_id;
	/* A non-safety protection record is only legitimate once the heater
	 * hardware is actually off; a failed off must be published as safety. */
	assert(safety || output_duty == 0);
	led_faults++;
	led_fault_safety = safety;
}
void led_operation_publish(enum led_owner owner, bool ota_active, bool heated_active)
{
	(void)owner;
	(void)ota_active;
	(void)heated_active;
	led_operations++;
}
/* Warm storage owner hands the immutable receipt identity back to the owner. */
void sys_warm_feedback_arm(uint32_t identity)
{
	assert(gate_depth == 1 && identity != 0);
	warm_armed = identity;
	warm_arms++;
}

/* Request-generation guard from the request owner (calibration.c, not built
 * here); the linked IMU owner validates every queued candidate against it. */
static uint32_t tcal_generation = 1;

uint32_t sensor_calibration_current_generation(void) { return tcal_generation; }
bool sensor_calibration_generation_valid(uint32_t generation)
{
	return generation == 0 || generation == tcal_generation;
}
void sensor_calibration_invalidate_requests(void) { tcal_generation++; }
/* Accepted reset admission clears one request kind atomically. */
void sensor_calibration_invalidate_kind(int kind) { (void)kind; }

bool heater_external_power_present(void) { return external_power; }
bool heater_power_ready(void) { return power_ready; }
bool esb_ota_is_active(void) { return ota_active; }
bool connection_get_ota_suppressed(void) { return ota_suppressed; }
bool heater_hw_available(void) { return hw_available; }
bool heater_hw_expired(void) { return hw_expiry; }

int sensor_get_imu_temperature_observation(struct sensor_temperature_observation *out, int64_t max_age)
{
	if (sample_error) return sample_error;
	if (sample.sampled_at_ms > now_ms || now_ms - sample.sampled_at_ms > max_age) return -ENODATA;
	/* Intentionally pass nonfinite values through to test controller defense. */
	*out = sample;
	return 0;
}

bool sensor_peek_accel(float out[3])
{
	memcpy(out, accel, sizeof(accel));
	return accel_available;
}

bool sensor_peek_accel_fresh(float out[3], int64_t max_age)
{
	return sensor_peek_accel(out) && accel_age >= 0 && accel_age <= max_age && v_finite(out, 3);
}
float sensor_get_gyro_odr(void) { return gyro_odr; }

int heater_hw_arm(uint32_t generation)
{
	assert(gate_depth == 1 && !output_duty && generation != 0);
	arms++;
	armed_epoch = generation;
	lease_deadline = now_ms + HEAT_FRESH_MS;
	return arm_error;
}

int heater_hw_write(uint32_t generation, uint32_t sequence, int64_t sampled_at,
		    float raw, uint16_t duty)
{
	assert(gate_depth == 1 && !model_depth && !storage_depth);
	assert(generation == armed_epoch);
	assert(sequence == sample.sequence && sampled_at == sample.sampled_at_ms);
	assert(raw == sample.raw_c && duty <= CONFIG_SYSTEM_IMU_HEATER_MAX_DUTY_PPTT);
	writes++;
	output_duty = duty;
	lease_deadline = sampled_at + HEAT_FRESH_MS;
	return write_error;
}

int heater_hw_force_off(void)
{
	assert(gate_depth == 1 && !model_depth);
	offs++;
	if (!off_error) output_duty = 0;
	return off_error;
}

void heater_hw_get_status(struct heater_hw_status *out)
{
	*out = (struct heater_hw_status){.available = hw_available, .armed = imu_reserved,
		.expired = hw_expiry, .generation = armed_epoch, .duty_pptt = output_duty,
		.deadline_ms = lease_deadline};
}

void sensor_tcal_heated_lock(void) { assert(gate_depth++ == 0); }
void sensor_tcal_heated_unlock(void) { assert(gate_depth == 1); gate_depth--; }

/* The shared dispatcher is not reimplemented. Only the actual coefficient
 * owner's reservation is connected to the controller at this adapter. */
int sensor_calibration_heated_reserve_locked(void)
{
	assert(gate_depth == 1);
	int err = sensor_calibration_imu_reserve_heated();
	if (!err) imu_reserved = true;
	return err;
}

bool sensor_calibration_maintenance_active_locked(void)
{
	assert(gate_depth);
	return maintenance_active;
}

void sensor_calibration_heated_release_locked(void)
{
	assert(gate_depth == 1 && !output_duty);
	imu_reserved = false;
	sensor_calibration_imu_release_heated();
}

void sensor_tcal_lock(void) { assert(gate_depth == 1 && model_depth++ == 0); }
void sensor_tcal_unlock(void)
{
	assert(model_depth == 1);
	if (expect_publication) {
		/* An observer at the model unlock sees the complete old or new model,
		 * never an intermediate stage merge. */
		assert(!memcmp(retained->tempCalPoints, old_points, sizeof(old_points)) ||
		       expected_model_matches());
	}
	model_depth--;
}
uint32_t sensor_tcal_model_generation(void) { assert(model_depth); return model_generation; }
void sensor_tcal_refresh_model(void)
{
	assert(storage_depth == 1 && gate_depth == 1 && model_depth == 1);
	assert(!output_duty && expect_publication);
	assert(expected_model_matches());
	refreshes++;
	model_generation++;
}
void sensor_tcal_clear_doffset(void) { retained->bootCalState.doffset_valid = false; }
void tcal_accum_reset(void)
{
	assert(gate_depth);
	resets++;
	production_tcal_accum_reset();
}
void tcal_accum_request_reset(void)
{
	assert(gate_depth);
	reset_requested = true;
	production_tcal_accum_request_reset();
}
void tcal_accum_apply_reset(void)
{
	if (reset_requested) { resets++; reset_requested = false; }
	production_tcal_accum_apply_reset();
}
void sensor_tcal_heated_accum_feed(const float g[3], float temp)
{
	assert(gate_depth && heat.sampling);
	feeds++;
	production_sensor_tcal_heated_accum_feed(g, temp);
}
void sensor_tcal_heated_accum_finish(void)
{
	assert(!output_duty && heat.state == HEAT_RUNNING);
	partial_finishes++;
	production_sensor_tcal_heated_accum_finish();
}

int sys_write(uint16_t id, void *ptr, const void *data, size_t size)
{
	(void)id;
	memcpy(ptr, data, size);
	return 0;
}
void retained_update(void) {}
void cal_event_end(uint16_t operation, uint8_t outcome, uint8_t phase, uint8_t reason)
{ (void)operation; (void)outcome; (void)phase; (void)reason; }
void cal_event_step(uint16_t operation, uint8_t phase, uint8_t detail)
{ (void)operation; (void)phase; (void)detail; }
void tracker_events_notify(void) {}

void sys_warm_transaction_begin(void)
{
	assert(!gate_depth && !model_depth && !storage_depth && !output_duty);
	if (on_storage_begin) {
		void (*hook)(void) = on_storage_begin;
		on_storage_begin = NULL;
		hook();
	}
	storage_depth++;
}
void sys_warm_transaction_mark(uint16_t id, const void *data, size_t size)
{
	(void)id;
	(void)data;
	(void)size;
	assert(storage_depth == 1 && !gate_depth && !model_depth && refreshes > 0);
	assert(expected_model_matches());
	dirty_marks++;
}
void sys_warm_transaction_end(bool schedule)
{
	assert(storage_depth == 1 && !gate_depth && !model_depth);
	storage_depth--;
	scheduled += schedule;
}

static float from_bits(uint32_t bits)
{
	float result;
	memcpy(&result, &bits, sizeof(result));
	return result;
}
static void near(float actual, float expected) { assert(fabsf(actual - expected) < 0.0001f); }
static void new_sample(int64_t dt, float raw, float filtered, bool rest)
{
	now_ms += dt;
	sample = (struct sensor_temperature_observation){raw, filtered, now_ms, sample.sequence + 1};
	sensor_tcal_heated_update(rest);
}
static void tick(bool rest) { new_sample(1000, sample.raw_c, sample.filtered_c, rest); }
static void unchanged(void)
{
	assert(!memcmp(retained->tempCalPoints, old_points, sizeof(old_points)));
	assert(refreshes == 0 && dirty_marks == 0 && scheduled == 0);
}
static void stopped(enum tcal_heated_stop_reason reason)
{
	assert(!sensor_tcal_heated_busy() && !imu_reserved && !output_duty);
	assert(heat.reason == reason && !heat.sampling && !heat.applied);
	unchanged();
}
static void start(void)
{
	assert(sensor_tcal_heated_start(44.0f) == 0);
	assert(sensor_tcal_heated_busy() && imu_reserved && !output_duty);
	sensor_tcal_heated_update(true);
	assert(heat.state == HEAT_RUNNING);
}
static void resting(void)
{
	start();
	for (int i = 0; i < CONFIG_SENSOR_TCAL_HEATED_RESUME_STABLE_MS / 1000; i++) tick(true);
	assert(heat.sampling);
}
static void feed_samples(unsigned count, const float gyro[3], float temp)
{
	for (unsigned i = 0; i < count; i++) {
		now_ms++;
		if (now_ms - heat.control_ms >= 1000)
			new_sample(0, sample.raw_c, sample.filtered_c, true);
		assert(sensor_tcal_heated_feed(gyro, temp));
	}
}
static void flush_owner(void)
{
	sensor_tcal_heated_lock();
	tcal_accum_flush(true);
	sensor_tcal_heated_unlock();
}
static void accept(float temp)
{
	sensor_tcal_heated_lock();
	sensor_tcal_heated_accept_point(TEMP_TO_IDX(temp), bias, temp);
	sensor_tcal_heated_unlock();
}
static void qualified_stage(void)
{
	resting();
	accept(25.0f);
	accept(31.0f);
	accept(37.0f);
	accept(44.0f);
	assert(heat.count == MLS_MIN_POINTS_FOR_FIT);
	unchanged();
}
static void expect_merge(void)
{
	memcpy(expected_points, old_points, sizeof(expected_points));
	for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
		if (stage[i].temp != 0.0f) expected_points[i] = stage[i];
	}
	expect_publication = true;
}
static void reach_target(void)
{
	/* A deterministic plant follows the real commanded ramp. Never forge
	 * controller state, observation history or dwell credit. */
	for (unsigned second = 0; second < CONFIG_SENSOR_TCAL_HEATED_TIMEOUT_MIN * 60 &&
	     (heat.setpoint < heat.target || sample.filtered_c < heat.target); second++) {
		float measured = heat.setpoint;
		new_sample(1000, measured, measured, true);
		assert(heat.state == HEAT_RUNNING);
	}
	assert(heat.setpoint == heat.target && sample.filtered_c == heat.target);
}
static void reach_finalizing(void)
{
	reach_target();
	for (unsigned second = 0; second <= CONFIG_SENSOR_TCAL_HEATED_STABLE_MS / 1000 &&
	     heat.state == HEAT_RUNNING; second++)
		tick(true);
	assert(heat.state == HEAT_FINALIZING && !output_duty && !heat.sampling);
	assert(sensor_tcal_heated_busy() && imu_reserved);
	unchanged();
	expect_merge();
}
static void published(void)
{
	assert(heat.applied && (heat.reason == TCAL_HEATED_STOP_COMPLETE ||
	                       heat.reason == TCAL_HEATED_STOP_USER));
	assert(!sensor_tcal_heated_busy() && !imu_reserved && !output_duty);
	assert(refreshes == 1 && dirty_marks == 4 && scheduled == 1);
	assert(model_generation == heat.base_generation + 1);
	assert(!memcmp(retained->tempCalPoints, expected_points, sizeof(expected_points)));
	unsigned count = 0;
	for (int i = 0; i < TCAL_BUFFER_SIZE; i++) count += expected_points[i].temp != 0.0f;
	assert(retained->tempCalState.count == count && retained->tempCalState.valid);
	assert(retained->tempCalState.degree == 0);
	for (unsigned i = 0; i < sizeof(retained->tempCalCoeffs) / sizeof(float); i++)
		near(((float *)retained->tempCalCoeffs)[i], 0);
	sensor_tcal_heated_finalize();
	assert(refreshes == 1 && scheduled == 1);
}
static void stop_hook(void) { assert(sensor_tcal_heated_stop() == 0); }
static void restart_hook(void)
{
	assert(sensor_tcal_heated_abort(TCAL_HEATED_STOP_SENSOR_STOP) == 0);
	sensor_tcal_heated_set_ready(true);
	sample.raw_c = sample.filtered_c = 25.0f;
	sample.sampled_at_ms = now_ms;
	sample.sequence++;
	assert(sensor_tcal_heated_start(44.0f) == 0);
}
static void reset_hook(void) { sensor_calibration_clear_begin(); sensor_calibration_clear_end(); }
static void power_loss_hook(void) { external_power = false; }
static void ota_hook(void) { ota_active = true; }
static void start_stop_hook(void)
{
	sample.sampled_at_ms = now_ms;
	sample.sequence++;
	assert(sensor_tcal_heated_start(44) == 0);
	stop_hook();
}
static void maintenance_hook(void) { maintenance_active = true; }

static void admission(void)
{
	const float bad[] = {from_bits(0x7fc00000), from_bits(0x7f800000), from_bits(0xff800000),
		0.0f, 9.9f, 45.0f, 49.0f};
	for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) assert(sensor_tcal_heated_start(bad[i]) == -EINVAL);
	sensor_tcal_heated_set_ready(false);
	assert(sensor_tcal_heated_start(44) == -EAGAIN);
	sensor_tcal_heated_set_ready(true);
	external_power = false;
	assert(sensor_tcal_heated_start(44) == -ENODEV);
	external_power = true;
	power_ready = false;
	assert(sensor_tcal_heated_start(44) == -ESHUTDOWN);
	power_ready = true;
	hw_available = false;
	assert(sensor_tcal_heated_start(44) == -ENODEV);
	hw_available = true;
	ota_active = true;
	assert(sensor_tcal_heated_start(44) == -EBUSY);
	ota_active = false;
	ota_suppressed = true;
	assert(sensor_tcal_heated_start(44) == -EBUSY);
	ota_suppressed = false;
	sample.sampled_at_ms = now_ms - HEAT_FRESH_MS - 1;
	assert(sensor_tcal_heated_start(44) == -ENODATA);
	sample.sampled_at_ms = now_ms + 1;
	assert(sensor_tcal_heated_start(44) == -ENODATA);
	sample.sampled_at_ms = now_ms;
	for (unsigned i = 0; i < 3; i++) {
		sample.raw_c = bad[i];
		assert(sensor_tcal_heated_start(44) == -ENODATA);
		sample.raw_c = 25;
		sample.filtered_c = bad[i];
		assert(sensor_tcal_heated_start(44) == -ENODATA);
		sample.filtered_c = 25;
	}
	accel_available = false;
	assert(sensor_tcal_heated_start(44) == -ENODATA);
	accel_available = true;
	accel[0] = from_bits(0x7fc00000);
	assert(sensor_tcal_heated_start(44) == -ENODATA);
	accel[0] = 0;
	accel_age = 2001;
	assert(sensor_tcal_heated_start(44) == -ENODATA);
	accel_age = 0;
	sample.raw_c = CONFIG_SENSOR_POLY_TEMP_MIN - 0.01f;
	assert(sensor_tcal_heated_start(44) == -ERANGE);
	sample.raw_c = 25;
	sample.filtered_c = CONFIG_SENSOR_POLY_TEMP_MIN - 0.01f;
	assert(sensor_tcal_heated_start(44) == -ERANGE);
	sample.filtered_c = 25;
	sample.raw_c = 42.51f;
	assert(sensor_tcal_heated_start(44) == -ERANGE);
	sample.raw_c = 25;
	sample.filtered_c = 42.51f;
	assert(sensor_tcal_heated_start(44) == -ERANGE);
	sample.filtered_c = 25;
	assert(arms == 0 && !sensor_tcal_heated_busy() && !imu_reserved);
	/* Exactly enough representable bins remains an admissible span. */
	sample.raw_c = sample.filtered_c = 42.5f;
	assert(sensor_tcal_heated_start(44) == 0);
}

/* The same input and analytic expected population exercise both publication
 * paths. No classifier is copied into the fixture. Rejected populations below
 * are deliberately planted, not inferred from production's acceptance mask. */
static void robust_case(const char *which, bool heated)
{
	static const char *const cases[] = {
		"clean", "linear", "first", "spikes", "paired", "noise", "noise_spikes",
		"odr104", "odr833", "odr1600", "bursty", "coverage", "majority",
		"tail1", "tail2", "tail3", "tail4", "tail3_bad", "tail4_bad",
		"partial_spike", "motion", "thermal", "reset", "accel", "motion_tail",
	};
	bool recognized = false;
	for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++)
		recognized |= !strcmp(which, cases[i]);
	assert(recognized);
	bool noisy = !strcmp(which, "noise") || !strcmp(which, "noise_spikes");
	bool linear = !strcmp(which, "linear") || !strcmp(which, "paired");
	bool coverage = !strcmp(which, "coverage");
	bool majority = !strcmp(which, "majority");
	bool motion = !strcmp(which, "motion");
	bool thermal = !strcmp(which, "thermal");
	bool reset = !strcmp(which, "reset");
	bool accel_motion = !strcmp(which, "accel");
	bool reject = coverage || majority || motion || thermal || reset || accel_motion;
	bool motion_tail = !strcmp(which, "motion_tail");
	bool bad_tail = !strcmp(which, "tail3_bad") || !strcmp(which, "tail4_bad");
	if (!strcmp(which, "odr104")) gyro_odr = 104;
	if (!strcmp(which, "odr833")) gyro_odr = 833;
	if (!strcmp(which, "odr1600")) gyro_odr = 1600;
	unsigned block_samples = (unsigned)(gyro_odr * 0.25f);
	unsigned blocks = 80;
	unsigned tail = 0;
	if (!strncmp(which, "tail", 4)) {
		tail = (unsigned)atoi(which + 4);
		assert(tail >= 1 && tail <= 4);
		blocks += tail;
	}
	if (motion_tail) blocks = 9;
	unsigned samples = blocks * block_samples;
	if (!strcmp(which, "partial_spike")) samples += block_samples - 1;
	if (heated) resting();
	else production_tcal_accum_apply_reset();
	int64_t started = now_ms;
	double expected_temp = 0, expected_gyro[3] = {0};
	unsigned expected_count = 0;
	uint32_t random = 0x6d2b79f5;
	for (unsigned i = 0; i < samples; i++) {
		unsigned block = i / block_samples;
		bool bad = (!strcmp(which, "first") && block == 0) ||
			((!strcmp(which, "spikes") || !strcmp(which, "paired") || !strcmp(which, "noise_spikes")) &&
			 (block == 0 || block == 22 || block == 24 || block == 56 ||
			  block == 72 || block == 74)) ||
			(coverage && block % 5 < 2) || (majority && block % 5 < 3) ||
			(thermal && block == 40) || (bad_tail && block == 80);
		bool partial = i >= blocks * block_samples;
		float temp = linear ? 26.05f + 0.2f * i / samples : 26.1f;
		if (tail && block >= 80) temp = 26.2f;
		if (bad && !coverage && !majority) temp = 26.4f;
		if (thermal) temp = bad ? 26.49f : 26.01f;
		float gyro[3];
		for (unsigned axis = 0; axis < 3; axis++) {
			gyro[axis] = noisy ? 30.0f * (axis + 1) : bias[axis];
			if (linear) gyro[axis] += (axis + 1) * (temp - 26.1f) * 0.5f;
			if (noisy) {
				/* Seeded bounded approximately Gaussian raw noise: unlike
				 * perfect +/- alternation it leaves noisy block means. */
				float noise = -6.0f;
				for (unsigned draw = 0; draw < 12; draw++) {
					random ^= random << 13;
					random ^= random >> 17;
					random ^= random << 5;
					noise += (random & 0xffffU) / 65536.0f;
				}
				gyro[axis] += 3.0f * noise;
			}
			if (tail && block >= 80) gyro[axis] += 0.01f;
		}
		if (bad) {
			/* Different axes and two same-side blocks in one group.
			 * Single raw impulses still contaminate a full block. */
			if (majority) gyro[block % 5] += 10;
			else if (i % block_samples == 0)
				gyro[block == 56 ? 0 : 2] += (block == 56 ? -1 : 1) *
					(noisy ? 20.0f : !strcmp(which, "first") ? 10.0f : 2.0f) * block_samples;
		}
		if (partial) gyro[2] -= 100;
		if (motion && block >= 75) gyro[0] += 6;
		bool retained_input = !bad && !partial &&
			!((tail < 3 || (bad_tail && tail == 3)) && tail && block >= 80);
		if (retained_input) {
			expected_count++;
			expected_temp += temp;
			for (unsigned axis = 0; axis < 3; axis++)
				expected_gyro[axis] += noisy ? 30.0f * (axis + 1) : gyro[axis];
		}
		/* Acquisition count, not feed-call spacing, defines blocks. FIFO
		 * bursts deliver 37 samples at one timestamp in the burst case. */
		unsigned delivered = !strcmp(which, "bursty") ? (i / 37) * 37 : i;
		now_ms = started + (int64_t)((delivered + 1) * 1000.0 / gyro_odr);
		if (thermal) now_ms = started + (i + 1) / 10;
		if (heated && now_ms - heat.control_ms >= 1000)
			new_sample(0, sample.raw_c, sample.filtered_c, true);
		if (accel_motion && block >= 78) accel[0] = 0.2f;
		if (heated) assert(sensor_tcal_heated_feed(gyro, temp));
		else tcal_accum_feed(gyro, temp, false);
	}
	if (reset) {
		sensor_tcal_heated_lock();
		tcal_accum_request_reset();
		sensor_tcal_heated_unlock();
		production_tcal_accum_apply_reset();
	}
	struct TempCalPoint expected = {.temp = (float)(expected_temp / expected_count)};
	for (unsigned axis = 0; axis < 3; axis++)
		expected.bias[axis] = (float)(expected_gyro[axis] / expected_count);
	memcpy(expected_points, old_points, sizeof(expected_points));
	expected_points[TEMP_TO_IDX(expected.temp)] = expected;
	publication_tolerance = noisy ? 0.06f : 0.0001f;
	expect_publication = !reject;
	if (heated) flush_owner();
	else if (motion_tail) sensor_tcal_continuous_motion_detected();
	else tcal_accum_flush(false);
	if (reject) {
		assert(heat.count == 0);
		unchanged();
	} else {
		const struct TempCalPoint *point = heated ?
			&stage[TEMP_TO_IDX(expected.temp)] : &retained->tempCalPoints[TEMP_TO_IDX(expected.temp)];
		near(point->temp, expected.temp);
		for (unsigned axis = 0; axis < 3; axis++)
			assert(fabsf(point->bias[axis] - expected.bias[axis]) < publication_tolerance);
		if (heated) {
			assert(heat.count == 1);
			unchanged();
		} else {
			assert(refreshes == 1 && scheduled == 1);
		}
	}
}

/* Dense gyro evidence is independent of the temperature polling cadence. */
static void rise_feed_ms(unsigned duration, float raw, float filtered)
{
	for (unsigned ms = 0; ms < duration; ms++) {
		now_ms++;
		if (now_ms % 20 == 0)
			new_sample(0, raw, filtered, true);
		assert(sensor_tcal_heated_feed(bias, filtered));
	}
}

static void rise_noise_case(void)
{
	/* User's held starting band: low admission anchor, one 20ms high raw
	 * endpoint at 1s, then bounded deterministic noise. No invented point. */
	sample.raw_c = 32.63f;
	sample.filtered_c = 32.78f;
	assert(sensor_tcal_heated_start(43) == 0);
	sensor_tcal_heated_update(true);
	uint32_t noise = 0x12345678;
	float raw = 32.78f;
	for (unsigned ms = 1; ms <= 31000; ms++) {
		now_ms++;
		if (ms % 20 == 0) {
			noise = noise * 1664525u + 1013904223u;
			raw = 32.78f + ((int)(noise >> 24) - 128) * (0.12f / 128);
			if (ms == 1000) raw = 32.93f;
			new_sample(0, raw, 32.78f, true);
		}
		assert(sensor_tcal_heated_feed(bias, 32.78f));
		assert(heat.state == HEAT_RUNNING && !heat.rise_limited);
		if (ms < 30000) assert(!heat.start_band_covered && heat.count == 0);
	}
	assert(heat.count == 1 && heat.start_band_covered && heat.setpoint > 32.78f);
	unchanged();
}

static void rise_cadence_case(unsigned cadence, bool worker_first, bool jitter)
{
	start();
	int64_t began = now_ms;
	unsigned index = 0;
	while (sensor_tcal_heated_busy()) {
		unsigned dt = jitter ? (index++ % 3 == 0 ? 337 : 113) : cadence;
		now_ms += dt;
		float raw = 25 + (now_ms - began) * 0.0003f;
		sample = (struct sensor_temperature_observation){raw, 25, now_ms, sample.sequence + 1};
		unsigned before = writes;
		int64_t deadline = lease_deadline;
		if (worker_first) {
			sensor_tcal_heated_finalize();
			sensor_tcal_heated_finalize();
			assert(writes == before && lease_deadline == deadline);
		}
		sensor_tcal_heated_update(true);
		before = writes;
		sensor_tcal_heated_update(true);
		assert(writes == before); /* no second write, no second slope */
		assert(!output_duty); /* external warming is hazardous even at zero */
		assert(now_ms - began < 9000);
	}
	stopped(TCAL_HEATED_STOP_RISE_FAST);
	assert(now_ms - began >= 3000);
	unsigned max_gap = jitter ? 337 : cadence;
	assert(now_ms - began < 3 * (1000 + max_gap));
	printf("rise cadence=%u jitter=%d worker=%d confirmation=%lldms excursion=%.3fC\n",
		cadence, jitter, worker_first, (long long)(now_ms - began), sample.raw_c - 25);
}

static void rise_recovery_case(void)
{
	resting();
	/* Start real unfinished evidence and build ordinary PI power. */
	rise_feed_ms(2000, 25, 24.5f);
	assert(tcal_accum.active && output_duty > 0 && heat.count == 0);
	float held = heat.setpoint;
	int64_t began = now_ms;
	while (now_ms - began < 2000) {
		float raw = 25 + (now_ms - began + 20) * 0.00030f;
		new_sample(20, raw, 24.5f, true);
	}
	assert(heat.rise_limited && !heat.sampling && !output_duty);
	assert(heat.rise_excess == 1); /* transient over-limit slope, not sustained */
	assert(heat.integral <= -150 && heat.setpoint == held && heat.stable_since == -1);
}

static void rise_plant_case(void)
{
	/* A duty-driven lumped body and lagged sensor, NOT reference-following.
	 * Units: C, seconds, full-scale duty fraction. A short coupling change
	 * stresses overspeed recovery; this is not hardware qualification. */
	sample.raw_c = sample.filtered_c = 32.78f;
	assert(sensor_tcal_heated_start(43) == 0);
	sensor_tcal_heated_update(true);
	float body = 32.78f, sensed = body, filtered = body;
	unsigned pulse = 0, limited_ms = 0;
	bool saw_cut = false, saw_recovery = false;
	/* The low-gain plant's slow PI pole is ~227s. At 180s it is still
	 * collecting the 33.0..33.5C slot; later slots only flush on real exit.
	 * Allow that physical exit rather than injecting/loosening coverage. */
	for (unsigned ms = 1; ms <= 360000; ms++) {
		float ambient = ms < 40000 || ms > 65000 ? 32.78f : 32.48f;
		if (!pulse && ms >= 40000 && output_duty >= 60)
			pulse = ms;
		float gain = pulse && ms - pulse < 1000 ? 65.0f : 3.0f;
		body += (gain * output_duty / 10000.0f - (body - ambient) / 4.0f) * 0.001f;
		sensed += (body - sensed) * (0.001f / 0.1f);
		filtered += (sensed - filtered) * (0.001f / 0.5f);
		now_ms++;
		if (ms % 20 == 0)
			new_sample(0, sensed, filtered, true);
		assert(sensor_tcal_heated_feed(bias, filtered));
		assert(heat.state == HEAT_RUNNING);
		assert(body > 32 && body < 36);
		if (heat.rise_limited) {
			assert(!output_duty && !heat.sampling);
			saw_cut = true;
			limited_ms++;
		} else if (saw_cut && heat.sampling) {
			saw_recovery = true;
		}
		if (ms == 31000) assert(heat.count == 1 && heat.start_band_covered);
	}
	assert(pulse && saw_cut && saw_recovery && limited_ms < 15000);
	assert(heat.count >= 2 && heat.setpoint > 33.5f);
	unchanged();
}

static void rise_zero_ki_case(void)
{
	assert(CONFIG_SENSOR_TCAL_HEATED_DEFAULT_KI == 0);
	resting();
	/* External cooling establishes positive proportional demand. Then a
	 * bounded external warming episode triggers limiting below the held
	 * starting reference, where artificial negative I would latch heat off. */
	for (unsigned i = 0; i < 3; i++) new_sample(1000, 24, 24, true);
	assert(output_duty > 0);
	for (unsigned i = 1; i <= 100; i++)
		new_sample(20, 24 + i * 0.006f, 24 + i * 0.006f, true);
	assert(heat.rise_limited && !output_duty && !heat.count);
	new_sample(1000, 24.6f, 24.6f, true);
	new_sample(1000, 24.6f, 24.6f, true);
	assert(!heat.rise_limited && output_duty > 0 && !heat.sampling);
	/* Evolve actual requested duty through a small thermal body, not the
	 * reference. Without released P drive it cannot warm enough in this
	 * horizon; rest and the complete starting accumulator remain required. */
	float body = 24.6f, filtered = body;
	for (unsigned ms = 1; ms <= 45000; ms++) {
		body += (5.0f * output_duty / 10000.0f - (body - 24.6f) / 20.0f) * 0.001f;
		filtered += (body - filtered) * (0.001f / 0.5f);
		now_ms++;
		if (ms % 20 == 0) new_sample(0, body, filtered, true);
		assert(sensor_tcal_heated_feed(bias, filtered));
		assert(heat.state == HEAT_RUNNING && !heat.rise_limited);
		if (ms < 29000) assert(!heat.start_band_covered);
	}
	assert(body > 24.8f && output_duty > 0);
	assert(heat.count == 1 && heat.start_band_covered && heat.setpoint > 25);
	unchanged();
}

int main(int argc, char **argv)
{
	assert(argc == 2);
	sensor_calibration_identity_accel(retained->accBAinv);
	sensor_calibration_imu_load();
	sensor_calibration_set_consumer_ready(true);
	sensor_tcal_heated_set_ready(true);
	/* Existing model and unrelated point must survive until complete publication. */
	retained->tempCalPoints[TEMP_TO_IDX(20.0f)] = (struct TempCalPoint){20.0f, {4, 5, 6}};
	retained->tempCalPoints[TEMP_TO_IDX(25.0f)] = (struct TempCalPoint){25.0f, {7, 8, 9}};
	retained->tempCalState.count = 2;
	retained->tempCalState.valid = true;
	retained->tempCalState.degree = 3;
	retained->tempCalCoeffs[0][0] = 7;
	memcpy(old_points, retained->tempCalPoints, sizeof(old_points));
	const char *which = argv[1];
	if (!strncmp(which, "robust_heated_", sizeof("robust_heated_") - 1)) {
		robust_case(which + sizeof("robust_heated_") - 1, true);
	} else if (!strncmp(which, "robust_ordinary_", sizeof("robust_ordinary_") - 1)) {
		robust_case(which + sizeof("robust_ordinary_") - 1, false);
	} else if (!strcmp(which, "admission")) {
		admission();
	} else if (!strcmp(which, "imu_exclusion")) {
		start();
		assert(sensor_calibration_commit_bias(zero, bias, true, 0, (struct led_token){0}, 0, 0) == -EBUSY);
		assert(sensor_calibration_reset_imu() == -EBUSY);
		assert(sensor_calibration_reset_accel() == -EBUSY);
		assert(sensor_calibration_commit_accel(retained->accBAinv, 0, (struct led_token){0}, false, 0) == -EBUSY);
		assert(sensor_tcal_heated_start(44) == -EBUSY);
		stop_hook();
		assert(sensor_calibration_commit_bias(zero, bias, true, 0, (struct led_token){0}, 0, 0) == 0);
	} else if (!strcmp(which, "pending_candidate")) {
		assert(sensor_calibration_commit_bias(zero, bias, true, 0, (struct led_token){0}, 0, 0) == 0);
		assert(sensor_tcal_heated_start(44) == -EBUSY);
		assert(!arms && !sensor_tcal_heated_busy());
		assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_BIAS_CHANGED);
		assert(sensor_tcal_heated_start(44) == -EBUSY);
		sensor_calibration_fusion_applied();
		sensor_calibration_persist_pending();
		assert(sensor_tcal_heated_start(44) == 0);
	} else if (!strcmp(which, "reset_barrier")) {
		start();
		sensor_calibration_clear_begin();
		stopped(TCAL_HEATED_STOP_SENSOR_STOP);
		assert(sensor_tcal_heated_start(44) == -EBUSY);
		assert(sensor_calibration_commit_bias(zero, bias, true, 0, (struct led_token){0}, 0, 0) == -EBUSY);
		sensor_tcal_heated_set_ready(true);
		assert(sensor_tcal_heated_start(44) == -EBUSY);
		sensor_calibration_clear_end();
		assert(sensor_tcal_heated_start(44) == -EAGAIN);
		sensor_tcal_heated_set_ready(true);
		assert(sensor_tcal_heated_start(44) == 0);
	} else if (!strcmp(which, "raw_overshoot") || !strcmp(which, "filtered_overshoot")) {
		start();
		/* Update below the 1s control interval must still apply the hard check. */
		bool raw = !strcmp(which, "raw_overshoot");
		new_sample(10, raw ? 46.0f : 25.0f, raw ? 25.0f : 46.0f, true);
		stopped(TCAL_HEATED_STOP_OVERTEMP);
		assert(writes == 0);
	} else if (!strcmp(which, "raw_rise")) {
		start();
		new_sample(1000, 25.21f, 25.0f, true);
		for (unsigned i = 0; i < 6; i++) tick(true);
		assert(heat.state == HEAT_RUNNING && !heat.rise_limited);
		unchanged();
	} else if (!strcmp(which, "rise_noise")) {
		rise_noise_case();
	} else if (!strcmp(which, "rise_plant")) {
		rise_plant_case();
	} else if (!strcmp(which, "rise_zero_ki")) {
		rise_zero_ki_case();
	} else if (!strcmp(which, "rise_recovery")) {
		rise_recovery_case();
		rise_feed_ms(2000, sample.raw_c, 25);
		assert(!heat.rise_limited && !heat.sampling && !tcal_accum.active);
		rise_feed_ms(5000, sample.raw_c, 25);
		assert(heat.sampling && !heat.count);
		rise_feed_ms(26000, sample.raw_c, 25);
		assert(heat.start_band_covered && heat.count == 1);
		unchanged();
	} else if (!strcmp(which, "rise_cadence20") || !strcmp(which, "rise_cadence113") ||
		   !strcmp(which, "rise_cadence1500") || !strcmp(which, "rise_jitter") ||
		   !strcmp(which, "rise_worker")) {
		unsigned cadence = !strcmp(which, "rise_cadence1500") ? 1500 :
			!strcmp(which, "rise_cadence113") ? 113 : 20;
		rise_cadence_case(cadence, !strcmp(which, "rise_worker"), !strcmp(which, "rise_jitter"));
	} else if (!strcmp(which, "rise_worker_cut")) {
		/* A fresh but 500ms-old admitted sample offsets acquisition windows
		 * from PI cadence. The worker must leave the early off write to owner. */
		sample.sampled_at_ms -= 500;
		start();
		int64_t began = now_ms;
		while (now_ms - began < 1500) {
			now_ms += 20;
			sample = (struct sensor_temperature_observation){
				25 + (now_ms - began + 500) * 0.00015f,
				24.5f, now_ms, sample.sequence + 1};
			unsigned before = writes;
			int64_t deadline = lease_deadline;
			sensor_tcal_heated_finalize();
			sensor_tcal_heated_finalize();
			assert(writes == before && lease_deadline == deadline);
			if (heat.rise_limited) {
				assert(output_duty > 0 && now_ms - heat.control_ms < 1000);
				int64_t controlled = heat.control_ms;
				sensor_tcal_heated_update(true);
				assert(!output_duty && writes == before + 1 && heat.control_ms == controlled);
				sensor_tcal_heated_update(true);
				assert(writes == before + 1);
			} else {
				sensor_tcal_heated_update(true);
			}
		}
		assert(heat.rise_limited && !output_duty && heat.integral <= -150);
		assert(lease_deadline == sample.sampled_at_ms + HEAT_FRESH_MS);
		unchanged();
	} else if (!strcmp(which, "rise_finalize")) {
		qualified_stage();
		new_sample(1000, 25.3f, 25, true);
		new_sample(1000, 25.6f, 25, true);
		assert(heat.rise_excess == 1);
		stop_hook();
		assert(heat.state == HEAT_FINALIZING && !output_duty);
		unsigned before = writes;
		/* Owner keeps the safety history current while the stage is frozen. */
		new_sample(1000, 25.9f, 25, true);
		assert(writes == before);
		sensor_tcal_heated_finalize();
		stopped(TCAL_HEATED_STOP_RISE_FAST);
	} else if (!strcmp(which, "rise_restart")) {
		rise_cadence_case(20, true, false);
		start();
		assert(!heat.rise_excess && !heat.rise_limited && !heat.rise_previous_duration);
		rise_feed_ms(31000, sample.raw_c, 25);
		assert(heat.state == HEAT_RUNNING && heat.count == 1);
		unchanged();
	} else if (!strncmp(which, "rise_invalid_", sizeof("rise_invalid_") - 1)) {
		start();
		new_sample(1000, 25.3f, 25, true);
		new_sample(1000, 25.6f, 25, true);
		assert(heat.rise_excess == 1);
		sample.sequence++;
		if (!strcmp(which, "rise_invalid_back")) sample.sampled_at_ms--;
		if (!strcmp(which, "rise_invalid_future")) sample.sampled_at_ms = now_ms + 1;
		if (!strcmp(which, "rise_invalid_gap")) {
			now_ms += HEAT_FRESH_MS + 1;
			sample.sampled_at_ms = now_ms;
		}
		if (!strcmp(which, "rise_invalid_stale")) now_ms += HEAT_FRESH_MS + 1;
		if (!strcmp(which, "rise_invalid_duplicate")) {
			sample.sequence--;
			sample.sampled_at_ms--;
		}
		if (!strcmp(which, "rise_invalid_nan")) sample.raw_c = from_bits(0x7fc00000);
		/* A new sequence without advancing acquisition time is not coverage. */
		sensor_tcal_heated_finalize();
		stopped(TCAL_HEATED_STOP_STALE_TEMP);
	} else if (!strcmp(which, "same_sequence")) {
		start();
		tick(true);
		unsigned before = writes;
		int64_t deadline = lease_deadline;
		now_ms += 1000;
		/* Getter and status must not invent a fresh physical observation. */
		sensor_tcal_heated_report();
		sensor_tcal_heated_update(true);
		assert(writes == before && lease_deadline == deadline);
		now_ms += 1000;
		sensor_tcal_heated_update(true);
		stopped(TCAL_HEATED_STOP_STALE_TEMP);
	} else if (!strcmp(which, "missed_deadline")) {
		start();
		tick(true);
		unsigned before = writes;
		/* Physical readings continued while the controller missed its deadline. */
		now_ms += 2000;
		sample.sampled_at_ms = now_ms;
		sample.sequence++;
		sensor_tcal_heated_update(true);
		stopped(TCAL_HEATED_STOP_STALE_TEMP);
		assert(writes == before);
	} else if (!strcmp(which, "power_loss")) {
		start();
		external_power = false;
		sensor_tcal_heated_finalize();
		stopped(TCAL_HEATED_STOP_POWER_DOWN);
	} else if (!strcmp(which, "ota_abort")) {
		start();
		ota_active = true;
		sensor_tcal_heated_finalize();
		stopped(TCAL_HEATED_STOP_SENSOR_STOP);
	} else if (!strcmp(which, "suppression_abort")) {
		start();
		ota_suppressed = true;
		sensor_tcal_heated_update(true);
		stopped(TCAL_HEATED_STOP_SENSOR_STOP);
	} else if (!strcmp(which, "hardware_fault")) {
		start();
		write_error = -EIO;
		tick(true);
		stopped(TCAL_HEATED_STOP_HEATER_ERROR);
	} else if (!strcmp(which, "motion_resume")) {
		resting();
		/* The start band must be collected before any ramp. */
		near(heat.setpoint, 25);
		accept(25);
		tick(true);
		assert(heat.setpoint > 25 && output_duty > 0);
		float held = heat.setpoint;
		unsigned before = feeds;
		tick(false);
		assert(!heat.sampling);
		assert(sensor_tcal_heated_feed(bias, 25));
		assert(feeds == before);
		near(heat.setpoint, held);
		for (int i = 0; i < 5; i++) {
			tick(true);
			assert(!heat.sampling);
			near(heat.setpoint, held);
		}
		tick(true);
		assert(heat.sampling && heat.setpoint > held);
		assert(sensor_tcal_heated_feed(bias, 25));
		assert(feeds == before + 1);
	} else if (!strcmp(which, "ramp_trajectory") || !strcmp(which, "ramp_jitter_pause")) {
		bool jitter = !strcmp(which, "ramp_jitter_pause");
		resting();
		accept(25);
		float resumed_rise = 0;
		for (unsigned interval = 1; interval <= 2; interval++) {
			unsigned intervals = jitter ? 30 : 45;
			for (unsigned step = 0; step < intervals; step++) {
				int64_t dt = jitter ? (step % 2 ? 1750 : 1250) : 1000;
				float measured = heat.setpoint;
				new_sample(dt, measured, measured, true);
				assert(heat.state == HEAT_RUNNING && heat.reason == TCAL_HEATED_STOP_NONE);
				assert(output_duty <= CONFIG_SYSTEM_IMU_HEATER_MAX_DUTY_PPTT);
			}
			/* Independent reference: exactly 0.5 C per 45 active seconds,
			 * not a rounded 667 mC/min approximation. */
			near(heat.setpoint, 25.0f + interval * 0.5f + resumed_rise);
			if (jitter && interval == 1) {
				float held = heat.setpoint;
				for (unsigned second = 0; second < 37; second++) {
					tick(false);
					near(heat.setpoint, held);
					assert(!heat.sampling);
				}
				for (unsigned second = 0; second < 5; second++) {
					tick(true);
					near(heat.setpoint, held);
					assert(!heat.sampling);
				}
				tick(true);
				assert(heat.sampling);
				resumed_rise = 0.5f / 45.0f;
				near(heat.setpoint, held + resumed_rise);
			}
		}
		unchanged();
	} else if (!strcmp(which, "ramp_target_clamp")) {
		/* This 42 -> 44 C trajectory fits the unchanged default deadline. */
		sample.raw_c = sample.filtered_c = 42;
		resting();
		accept(42); accept(42.5f); accept(43); accept(44);
		for (unsigned second = 0; second < 179; second++) {
			float measured = heat.setpoint;
			new_sample(1000, measured, measured, true);
			assert(heat.state == HEAT_RUNNING && heat.setpoint < 44);
		}
		near(heat.setpoint, 42.0f + 179.0f / 90.0f);
		new_sample(1000, heat.setpoint, heat.setpoint, true);
		near(heat.setpoint, 44);
		for (unsigned second = 0; second < 59; second++) {
			new_sample(1000, 44, 44, true);
			near(heat.setpoint, 44);
			assert(heat.state == HEAT_RUNNING && !heat.applied);
		}
		new_sample(1000, 44, 44, true);
		assert(heat.state == HEAT_FINALIZING && !output_duty);
		expect_merge();
		sensor_tcal_heated_finalize();
		published();
	} else if (!strcmp(which, "default_ramp_budget")) {
		/* Prequalified slots alone must not complete a thermally stalled
		 * session. Fresh 24 C observations keep all other safety checks
		 * healthy while the reference reaches 44 C and the budget expires. */
		sample.raw_c = sample.filtered_c = 24;
		resting();
		accept(24); accept(31); accept(37); accept(44);
		int64_t deadline = heat.started_ms + 90 * 60000;
		while (now_ms + 1000 < deadline) {
			new_sample(1000, 24, 24, true);
			assert(heat.state == HEAT_RUNNING && !heat.applied && !partial_finishes);
			assert(heat.reason == TCAL_HEATED_STOP_NONE);
		}
		new_sample(deadline - 1 - now_ms, 24, 24, true);
		assert(now_ms - heat.started_ms == 5399999);
		near(heat.setpoint, 44);
		assert(heat.state == HEAT_RUNNING && !heat.applied && !partial_finishes);
		assert(heat.reason == TCAL_HEATED_STOP_NONE && output_duty > 0);
		new_sample(1, 24, 24, true);
		assert(now_ms - heat.started_ms == 5400000);
		stopped(TCAL_HEATED_STOP_TIMEOUT);
		assert(!partial_finishes);
	} else if (!strcmp(which, "stop_commit")) {
		qualified_stage();
		tick(true);
		assert(output_duty > 0);
		unsigned before = resets;
		stop_hook();
		assert(!output_duty && heat.state == HEAT_FINALIZING && !heat.applied);
		assert(heat.reason == TCAL_HEATED_STOP_USER && imu_reserved);
		assert(reset_requested && resets == before && !partial_finishes);
		assert(sensor_tcal_heated_start(44) == -EBUSY);
		assert(sensor_calibration_commit_bias(zero, bias, true, 0, (struct led_token){0}, 0, 0) == -EBUSY);
		/* Owner may consume the reset without changing the frozen stage. */
		assert(sensor_tcal_heated_feed(bias, 44));
		unchanged();
		expect_merge();
		sensor_tcal_heated_finalize();
		published();
		assert(heat.reason == TCAL_HEATED_STOP_USER);
		assert(!sensor_tcal_heated_feed(bias, 44));
	} else if (!strcmp(which, "restart_epoch")) {
		qualified_stage();
		uint32_t previous = owner_epoch;
		assert(sensor_tcal_heated_abort(TCAL_HEATED_STOP_USER) == 0);
		assert(sensor_tcal_heated_start(44) == 0);
		assert(heat.epoch != previous && armed_epoch == heat.epoch);
		accept(44); /* stale owner has not synchronized the new session */
		assert(heat.count == 0);
		sensor_tcal_heated_finalize();
		unchanged();
		sensor_tcal_heated_update(true);
		assert(owner_epoch == heat.epoch && heat.count == 0);
		for (int i = 0; i < TCAL_BUFFER_SIZE; i++) assert(stage[i].temp == 0);
	} else if (!strcmp(which, "pi_feedback") || !strcmp(which, "pi_feedback_nonzero")) {
		bool nonzero = !strcmp(which, "pi_feedback_nonzero");
		qualified_stage();
		reach_target();
		uint16_t at_target = output_duty;
		assert(at_target > 0 && at_target < CONFIG_SYSTEM_IMU_HEATER_MAX_DUTY_PPTT);
		/* Legal sub-limit temperature rise, still within the target band.
		 * Negative feedback must reduce power without waiting for cutoff. */
		for (unsigned step = 1; step <= 5; step++) {
			float measured = heat.target + step * 0.05f;
			new_sample(1000, measured, measured, true);
			assert(heat.state == HEAT_RUNNING);
		}
		assert(output_duty < at_target);
		if (nonzero) {
			/* Motion pauses collection, not thermal control. Sustained small
			 * overshoot must let integral feedback cancel feedforward too. */
			for (unsigned second = 0; second < 120; second++)
				tick(false);
			float proportional_feedforward =
				CONFIG_SENSOR_TCAL_HEATED_DEFAULT_KFF * (heat.target - heat.start_temp) -
				CONFIG_SENSOR_TCAL_HEATED_DEFAULT_KP * 0.25f;
			assert(output_duty > 0 && output_duty < proportional_feedforward - 10.0f);
		}
		assert(sensor_tcal_heated_busy() && heat.reason == TCAL_HEATED_STOP_NONE);
		unchanged();
	} else if (!strcmp(which, "publish")) {
		resting();
		/* Deterministic plant follows the commanded ramp. Every stage point
		 * comes from actual 1kHz gyro accumulation; no accept-point injection. */
		for (unsigned second = 0; second < CONFIG_SENSOR_TCAL_HEATED_TIMEOUT_MIN * 60 &&
		     heat.state != HEAT_FINALIZING; second++) {
			float measured = heat.setpoint;
			uint16_t previous = output_duty;
			for (int ms = 0; ms < 1000; ms++) {
				now_ms++;
				assert(sensor_tcal_heated_feed(bias, measured));
			}
			new_sample(0, measured, measured, true);
			assert(output_duty <= previous + CONFIG_SENSOR_TCAL_HEATED_SLEW_PPTT_PER_S);
			unchanged();
		}
		assert(heat.state == HEAT_FINALIZING && partial_finishes == 1);
		/* Real initial collection and endpoint dwell cross the former
		 * 30-minute cutoff; the standard budget must allow publication. */
		assert(now_ms - heat.started_ms >= 30 * 60000);
		assert(now_ms - heat.started_ms < 90 * 60000);
		assert(heat.reason == TCAL_HEATED_STOP_COMPLETE);
		expect_merge();
		sensor_tcal_heated_finalize();
		published();
	} else if (!strcmp(which, "insufficient_bins")) {
		resting();
		accept(25); accept(31); accept(44);
		for (unsigned i = 0; i < 10; i++) accept(31.1f);
		assert(heat.count == 3);
		reach_finalizing();
		sensor_tcal_heated_finalize();
		stopped(TCAL_HEATED_STOP_INSUFFICIENT_COVERAGE);
	} else if (!strcmp(which, "missing_start")) {
		resting();
		accept(25.51f); accept(31); accept(37); accept(44);
		assert(heat.count == MLS_MIN_POINTS_FOR_FIT);
		for (unsigned second = 0; second < CONFIG_SENSOR_TCAL_HEATED_TIMEOUT_MIN * 60 &&
		     sensor_tcal_heated_busy(); second++) {
			tick(true);
			near(heat.setpoint, heat.start_temp);
		}
		stopped(TCAL_HEATED_STOP_TIMEOUT);
		assert(partial_finishes == 0);
	} else if (!strcmp(which, "missing_target")) {
		resting();
		accept(25); accept(31); accept(37); accept(43.49f);
		reach_finalizing();
		sensor_tcal_heated_finalize();
		stopped(TCAL_HEATED_STOP_INSUFFICIENT_COVERAGE);
	} else if (!strcmp(which, "invalid_stage")) {
		qualified_stage();
		unsigned count = heat.count;
		sensor_tcal_heated_lock();
		sensor_tcal_heated_accept_point(-1, bias, 25);
		sensor_tcal_heated_accept_point(TCAL_BUFFER_SIZE, bias, 25);
		sensor_tcal_heated_accept_point(TEMP_TO_IDX(26), bias, 25);
		float invalid[3] = {from_bits(0x7fc00000), 0, 0};
		sensor_tcal_heated_accept_point(TEMP_TO_IDX(26), invalid, 26);
		sensor_tcal_heated_unlock();
		assert(heat.count == count);
		/* Finalizer independently validates immutable contents, not only metadata. */
		stage[TEMP_TO_IDX(31)].temp = 32;
		reach_finalizing();
		sensor_tcal_heated_finalize();
		stopped(TCAL_HEATED_STOP_INSUFFICIENT_COVERAGE);
	} else if (!strcmp(which, "model_generation")) {
		qualified_stage();
		reach_finalizing();
		model_generation++;
		sensor_tcal_heated_finalize();
		stopped(TCAL_HEATED_STOP_INSUFFICIENT_COVERAGE);
	} else if (!strcmp(which, "stop_during_finalize") || !strcmp(which, "restart_during_finalize") ||
		   !strcmp(which, "reset_during_finalize")) {
		qualified_stage();
		reach_finalizing();
		bool restart = !strcmp(which, "restart_during_finalize");
		bool reset = !strcmp(which, "reset_during_finalize");
		on_storage_begin = restart ? restart_hook : (reset ? reset_hook : stop_hook);
		uint32_t old_epoch = heat.epoch;
		sensor_tcal_heated_finalize();
		if (!restart && !reset) {
			published();
			assert(heat.reason == TCAL_HEATED_STOP_USER);
		} else {
			unchanged();
			assert(heat.epoch != old_epoch);
			if (restart) {
				assert(heat.state == HEAT_RESERVED && sensor_tcal_heated_busy());
				sensor_tcal_heated_update(true);
				assert(heat.count == 0);
			} else stopped(TCAL_HEATED_STOP_SENSOR_STOP);
		}
	} else if (!strcmp(which, "timeout_discard")) {
		qualified_stage();
		now_ms = heat.started_ms + CONFIG_SENSOR_TCAL_HEATED_TIMEOUT_MIN * 60000;
		sample.sampled_at_ms = now_ms;
		sample.sequence++;
		sensor_tcal_heated_update(true);
		stopped(TCAL_HEATED_STOP_TIMEOUT);
		assert(partial_finishes == 0);
	} else if (!strcmp(which, "partial_finish")) {
		resting();
		accept(25); accept(31); accept(37);
		reach_target();
		while (now_ms - heat.stable_since < CONFIG_SENSOR_TCAL_HEATED_STABLE_MS - 3000)
			tick(true);
		feed_samples(2000, bias, sample.filtered_c);
		reach_finalizing();
		assert(partial_finishes == 1 && heat.count == 4);
		sensor_tcal_heated_finalize();
		published();
	} else if (!strcmp(which, "endpoint_bands")) {
		resting();
		accept(25.5f); accept(31); accept(37); accept(43.5f);
		reach_finalizing();
		sensor_tcal_heated_finalize();
		published();
	} else if (!strcmp(which, "off_failure")) {
		qualified_stage();
		tick(true);
		assert(output_duty > 0);
		off_error = -EIO;
		assert(sensor_tcal_heated_stop() == -EIO);
		assert(heat.state == HEAT_OFF_FAILED && !heat.sampling && !heat.applied);
		assert(sensor_tcal_heated_busy() && imu_reserved && output_duty > 0);
		assert(sensor_tcal_heated_start(44) == -EBUSY);
		assert(sensor_calibration_commit_bias(zero, bias, true, 0, (struct led_token){0}, 0, 0) == -EBUSY);
		sensor_tcal_heated_finalize();
		unchanged();
		off_error = 0;
		stop_hook();
		stopped(TCAL_HEATED_STOP_USER);
		assert(sensor_calibration_commit_bias(zero, bias, true, 0, (struct led_token){0}, 0, 0) == 0);
	} else if (!strcmp(which, "accum_minimum")) {
		resting();
		feed_samples(1999, bias, 25);
		flush_owner();
		assert(heat.count == 0);
		feed_samples(1, bias, 25);
		flush_owner();
		assert(heat.count == 1 && !tcal_accum.active);
		for (unsigned axis = 0; axis < 3; axis++)
			near(stage[TEMP_TO_IDX(25)].bias[axis], bias[axis]);
		unchanged();
	} else if (!strcmp(which, "accum_accel")) {
		resting();
		accel_available = false;
		feed_samples(2000, bias, 25);
		assert(!tcal_accum.active && heat.count == 0);
		accel_available = true;
		accel_age = 2001;
		feed_samples(2000, bias, 25);
		assert(!tcal_accum.active && heat.count == 0);
		accel_age = 0;
		accel[0] = from_bits(0x7fc00000);
		feed_samples(2000, bias, 25);
		assert(!tcal_accum.active && heat.count == 0);
		accel[0] = 0;
		feed_samples(2000, bias, 25);
		accel_available = false; /* freshness is rechecked at final acceptance */
		flush_owner();
		assert(!tcal_accum.active && heat.count == 0);
		unchanged();
	} else if (!strcmp(which, "accum_motion")) {
		resting();
		feed_samples(16, bias, 25);
		accel[0] = TCAL_ACCUM_ACCEL_MOTION_THRESHOLD * 2;
		feed_samples(16, bias, 25);
		assert(!tcal_accum.active && heat.count == 0);
		accel[0] = 0;
		feed_samples(1250, bias, 25);
		const float moving[3] = {10, 0, 0};
		feed_samples(1250, moving, 25);
		assert(!tcal_accum.active && heat.count == 0);
		unchanged();
	} else if (!strcmp(which, "accum_rate")) {
		resting();
		feed_samples(1000, bias, 25);
		feed_samples(1000, bias, 25.4f);
		flush_owner();
		assert(!tcal_accum.active && heat.count == 0);
		unchanged();
	} else if (!strcmp(which, "accum_abort")) {
		resting();
		feed_samples(2000, bias, 25);
		assert(tcal_accum.active && heat.count == 0);
		stop_hook();
		assert(reset_requested);
		sensor_tcal_heated_finalize();
		assert(!sensor_tcal_heated_feed(bias, 25));
		assert(!tcal_accum.active && heat.count == 0 && !partial_finishes);
		stopped(TCAL_HEATED_STOP_USER);
	} else if (!strcmp(which, "accum_nonfinite")) {
		resting();
		float invalid[3] = {from_bits(0x7fc00000), 0, 0};
		feed_samples(2000, invalid, 25);
		feed_samples(2000, bias, from_bits(0x7f800000));
		assert(!tcal_accum.active && heat.count == 0);
		feed_samples(2000, bias, 25);
		flush_owner();
		assert(heat.count == 1);
		unchanged();
	} else if (!strcmp(which, "power_during_finalize") || !strcmp(which, "ota_during_finalize")) {
		qualified_stage();
		reach_finalizing();
		bool power = !strcmp(which, "power_during_finalize");
		on_storage_begin = power ? power_loss_hook : ota_hook;
		sensor_tcal_heated_finalize();
		stopped(power ? TCAL_HEATED_STOP_POWER_DOWN : TCAL_HEATED_STOP_SENSOR_STOP);
	} else if (!strcmp(which, "quantized_rise")) {
		start();
		for (int i = 1; i <= 5000; i++) {
			float raw = 25.0f + floorf((i * 0.020f * 0.030f) / 0.03125f) * 0.03125f;
			new_sample(20, raw, raw, true);
			assert(sensor_tcal_heated_busy() && heat.reason == TCAL_HEATED_STOP_NONE);
		}
		unchanged();
	} else if (!strcmp(which, "sustained_rise")) {
		start();
		for (int i = 1; i <= 150 && sensor_tcal_heated_busy(); i++)
			new_sample(20, 25.0f + i * 0.020f * 0.30f, 25.0f, true);
		stopped(TCAL_HEATED_STOP_RISE_FAST);
	} else if (!strcmp(which, "ordinary_start_stop") || !strcmp(which, "ordinary_reset") ||
		   !strcmp(which, "ordinary_maintenance")) {
		/* An ordinary bucket starts outside a heated session, then its storage
		 * acquisition yields to a start/stop or reset before model publication. */
		production_tcal_accum_apply_reset();
		for (int i = 0; i < 2000; i++) {
			now_ms++;
			tcal_accum_feed(bias, 26, false);
		}
		on_storage_begin = !strcmp(which, "ordinary_start_stop") ? start_stop_hook :
			(!strcmp(which, "ordinary_reset") ? reset_hook : maintenance_hook);
		tcal_accum_flush(false);
		assert(on_storage_begin == NULL && !tcal_accum.active);
		assert(!sensor_tcal_heated_busy());
		unchanged();
	} else if (!strcmp(which, "integral_headroom")) {
		resting();
		accept(25);
		/* Keep a persistent load error after the real ramp reaches its target.
		 * Raw rise stays below the independent cutoff throughout the run. */
		for (unsigned second = 0; second < 2800; second++) {
			float measured = fminf(38.62f, heat.setpoint);
			uint16_t previous = output_duty;
			new_sample(1000, measured, measured, true);
			assert(heat.state == HEAT_RUNNING);
			assert(output_duty <= CONFIG_SYSTEM_IMU_HEATER_MAX_DUTY_PPTT);
			assert(output_duty <= previous + CONFIG_SENSOR_TCAL_HEATED_SLEW_PPTT_PER_S);
		}
		assert(heat.setpoint == 44 && output_duty > 6000);
		float saturated_integral = heat.integral;
		for (unsigned second = 0; second < 60; second++) tick(true);
		near(heat.integral, saturated_integral);
		/* Warm through target below the rise-governor entry rate; ordinary
		 * negative feedback must unwind without supervisory cancellation. */
		while (sample.filtered_c < 44.25f) {
			float measured = fminf(44.25f, sample.filtered_c + 0.08f);
			new_sample(1000, measured, measured, false);
		}
		float overshoot_integral = heat.integral;
		uint16_t before = output_duty;
		for (unsigned second = 0; second < 120; second++) tick(false);
		assert(output_duty < before && heat.integral < overshoot_integral);
		unchanged();
	} else if (!strcmp(which, "slot_boundary")) {
		resting();
		accept(25);
		feed_samples(3000, bias, 30.49f);
		const float other[3] = {1, 2, 3};
		feed_samples(251, other, 30.51f);
		assert(heat.count == 2);
		near(stage[TEMP_TO_IDX(30.49f)].temp, 30.49f);
		for (unsigned axis = 0; axis < 3; axis++)
			near(stage[TEMP_TO_IDX(30.49f)].bias[axis], bias[axis]);
		feed_samples(1999, other, 30.51f);
		feed_samples(251, bias, 31.01f);
		assert(heat.count == 3);
		near(stage[TEMP_TO_IDX(30.51f)].temp, 30.51f);
		for (unsigned axis = 0; axis < 3; axis++)
			near(stage[TEMP_TO_IDX(30.51f)].bias[axis], other[axis]);
		unchanged();
	} else if (!strcmp(which, "slot_chatter")) {
		resting();
		accept(25);
		feed_samples(2000, bias, 30.49f);
		const float border[3] = {2, 3, 4};
		for (unsigned visit = 0; visit < 20; visit++) {
			feed_samples(100, border, 30.51f);
			feed_samples(100, bias, 30.49f);
		}
		assert(heat.count == 1 && stage[TEMP_TO_IDX(30.49f)].temp == 0);
		feed_samples(251, border, 30.51f);
		assert(heat.count == 2);
		near(stage[TEMP_TO_IDX(30.49f)].temp, 30.49f);
		for (unsigned axis = 0; axis < 3; axis++)
			near(stage[TEMP_TO_IDX(30.49f)].bias[axis], bias[axis]);
		/* Accepted slot revisits cannot overwrite or increment coverage. */
		struct TempCalPoint saved = stage[TEMP_TO_IDX(30.49f)];
		feed_samples(251, border, 30.49f);
		feed_samples(30000, border, 30.49f);
		assert(heat.count == 2 && !memcmp(&saved, &stage[TEMP_TO_IDX(30.49f)], sizeof(saved)));
		unchanged();
	} else if (!strcmp(which, "slot_revisit")) {
		resting();
		accept(25);
		feed_samples(1999, bias, 30.1f);
		feed_samples(251, bias, 30.6f);
		assert(heat.count == 1 && stage[TEMP_TO_IDX(30.1f)].temp == 0);
		/* Return after a short rejected visit: no synthetic/skipped coverage. */
		feed_samples(2250, bias, 30.1f);
		feed_samples(251, bias, 32.1f);
		assert(heat.count == 2);
		near(stage[TEMP_TO_IDX(30.1f)].temp, 30.1f);
		assert(stage[TEMP_TO_IDX(30.6f)].temp == 0);
		assert(stage[TEMP_TO_IDX(31.1f)].temp == 0);
		unchanged();
	} else if (!strcmp(which, "slot_no_overwrite")) {
		resting();
		accept(25);
		struct TempCalPoint saved = stage[TEMP_TO_IDX(25)];
		const float changed[3] = {1, 2, 3};
		feed_samples(30000, changed, 25.2f);
		sensor_tcal_heated_lock();
		sensor_tcal_heated_accept_point(TEMP_TO_IDX(25), changed, 25.3f);
		sensor_tcal_heated_unlock();
		assert(heat.count == 1 && !tcal_accum.active);
		assert(!memcmp(&saved, &stage[TEMP_TO_IDX(25)], sizeof(saved)));
		unchanged();
	} else if (!strcmp(which, "start_dwell")) {
		resting();
		feed_samples(25000, bias, 25);
		assert(heat.count == 0 && heat.setpoint == 25);
		feed_samples(1, bias, 25);
		assert(heat.count == 1 && heat.start_band_covered);
		tick(true);
		assert(heat.setpoint > 25);
		feed_samples(30000, bias, 25);
		assert(heat.count == 1 && !tcal_accum.active);
		unchanged();
	} else if (!strcmp(which, "accum_epoch")) {
		resting();
		feed_samples(2000, bias, 25);
		sensor_tcal_heated_lock();
		tcal_accum_request_reset();
		sensor_tcal_heated_unlock();
		flush_owner();
		assert(heat.count == 0 && !tcal_accum.active);
		unchanged();
	} else if (!strcmp(which, "ordinary_interval_ema")) {
		/* Ordinary collection still spans adjacent bins and waits 25 seconds.
		 * It still EMA-blends an existing point rather than freezing it. */
		tcal_current_direction = TCAL_DIR_UNKNOWN;
		tcal_direction_ref_temp = 25.5f;
		memcpy(expected_points, old_points, sizeof(expected_points));
		/* The final unchecked one-sample tail is not calibration evidence. */
		expected_points[TEMP_TO_IDX(25)].temp = 25.49f;
		for (unsigned axis = 0; axis < 3; axis++)
			expected_points[TEMP_TO_IDX(25)].bias[axis] =
				0.5f * bias[axis] + 0.5f * old_points[TEMP_TO_IDX(25)].bias[axis];
		expect_publication = true;
		for (unsigned i = 0; i < 25000; i++) {
			now_ms++;
			tcal_accum_feed(bias, 25.49f, false);
		}
		unchanged();
		now_ms++;
		tcal_accum_feed(bias, 25.51f, false);
		assert(refreshes == 1 && scheduled == 1 && dirty_marks == 4);
		assert(retained->tempCalState.count == 2 && !tcal_accum.active);
		for (unsigned axis = 0; axis < 3; axis++)
			near(retained->tempCalPoints[TEMP_TO_IDX(25)].bias[axis],
			     expected_points[TEMP_TO_IDX(25)].bias[axis]);
	} else if (!strcmp(which, "stop_empty") || !strcmp(which, "stop_reserved")) {
		if (!strcmp(which, "stop_reserved")) {
			/* A preexisting ordinary window must never become a heated point. */
			for (unsigned i = 0; i < 2000; i++) {
				now_ms++;
				tcal_accum_feed(bias, 26, false);
			}
			sample.sampled_at_ms = now_ms;
			assert(sensor_tcal_heated_start(44) == 0);
		} else resting();
		stop_hook();
		sensor_tcal_heated_finalize();
		stopped(TCAL_HEATED_STOP_USER);
		assert(!partial_finishes);
	} else if (!strcmp(which, "stop_one") || !strcmp(which, "stop_two") ||
		   !strcmp(which, "stop_three") || !strcmp(which, "stop_merge_existing")) {
		bool existing = !strcmp(which, "stop_merge_existing");
		if (!existing) {
			memset(retained->tempCalPoints, 0, sizeof(retained->tempCalPoints));
			retained->tempCalState.count = 0;
			retained->tempCalState.valid = false;
			memcpy(old_points, retained->tempCalPoints, sizeof(old_points));
		}
		unsigned points = !strcmp(which, "stop_one") ? 1 : !strcmp(which, "stop_two") ? 2 : 3;
		resting();
		for (unsigned i = 0; i < points; i++) accept(26.0f + i);
		feed_samples(2000, bias, 30); /* unfinished bucket is deliberately discarded */
		stop_hook();
		unchanged();
		assert(!output_duty && heat.state == HEAT_FINALIZING && !heat.applied);
		expect_merge();
		sensor_tcal_heated_finalize();
		published();
		assert(heat.reason == TCAL_HEATED_STOP_USER && !partial_finishes);
		assert(retained->tempCalState.count == points + (existing ? 2 : 0));
		assert(retained->tempCalPoints[TEMP_TO_IDX(30)].temp == 0);
	} else if (!strcmp(which, "stop_fault") || !strcmp(which, "stop_rise_fault")) {
		qualified_stage();
		bool rise = !strcmp(which, "stop_rise_fault");
		if (rise) {
			for (unsigned i = 1; i <= 2; i++)
				new_sample(1000, 25 + i * 0.3f, 25, true);
			assert(heat.rise_excess == 1);
			now_ms += 1000;
			sample.raw_c += 0.3f;
			sample.sampled_at_ms = now_ms;
			sample.sequence++;
		} else hw_expiry = true;
		stop_hook();
		sensor_tcal_heated_finalize();
		stopped(rise ? TCAL_HEATED_STOP_RISE_FAST : TCAL_HEATED_STOP_STALE_TEMP);
	} else if (!strcmp(which, "warm_receipt")) {
		/* Completion belongs to the warm storage receipt, not to the RAM
		 * publication: only the armed identity may release the session, once. */
		qualified_stage();
		feed_samples(2000, bias, 30);
		stop_hook();
		unchanged();
		assert(!output_duty && heat.state == HEAT_FINALIZING && !heat.applied);
		expect_merge();
		sensor_tcal_heated_finalize();
		published();
		assert(heat.reason == TCAL_HEATED_STOP_USER && !partial_finishes);
		assert(heat.warm_pending && warm_arms == 1 && warm_armed == heat.feedback.session);
		led_results[LED_PARTIAL] = 0;
		led_results[LED_SUCCESS] = 0;
		/* A receipt for another transaction cannot complete this session. */
		sensor_tcal_feedback_persisted(warm_armed + 1, 0b1111, 0);
		assert(heat.warm_pending && led_results[LED_PARTIAL] == 0 && led_results[LED_SUCCESS] == 0);
		/* The three model writes cannot complete without gyroTemp. */
		sensor_tcal_feedback_persisted(warm_armed, 0b111, 0);
		assert(heat.warm_pending && led_results[LED_PARTIAL] == 0 && led_results[LED_SUCCESS] == 0);
		/* The fourth receipt completes exactly once as PARTIAL. */
		sensor_tcal_feedback_persisted(warm_armed, 0b1000, 0);
		assert(!heat.warm_pending && heat.feedback.session == 0);
		assert(led_results[LED_PARTIAL] == 1 && led_results[LED_SUCCESS] == 0);
		sensor_tcal_feedback_persisted(warm_armed, 0b1111, 0);
		assert(led_results[LED_PARTIAL] == 1);
		assert(led_states[LED_HEATED_ACTIVE] > 0 && led_operations >= 2);
		assert(led_faults == 1 && !led_fault_safety);
	} else if (!strcmp(which, "stop_pending_reset") || !strcmp(which, "stop_pending_abort") ||
		   !strcmp(which, "stop_pending_power") || !strcmp(which, "stop_pending_ota") ||
		   !strcmp(which, "stop_pending_generation") || !strcmp(which, "stop_pending_stale") ||
		   !strcmp(which, "stop_pending_overtemp") || !strcmp(which, "stop_pending_timeout")) {
		qualified_stage();
		stop_hook();
		assert(heat.state == HEAT_FINALIZING && !output_duty);
		if (!strcmp(which, "stop_pending_reset")) on_storage_begin = reset_hook;
		else if (!strcmp(which, "stop_pending_abort"))
			assert(sensor_tcal_heated_abort(TCAL_HEATED_STOP_TIMEOUT) == 0);
		else if (!strcmp(which, "stop_pending_power")) on_storage_begin = power_loss_hook;
		else if (!strcmp(which, "stop_pending_ota")) on_storage_begin = ota_hook;
		else if (!strcmp(which, "stop_pending_stale")) now_ms += HEAT_FRESH_MS + 1;
		else if (!strcmp(which, "stop_pending_overtemp")) sample.raw_c = 46;
		else if (!strcmp(which, "stop_pending_timeout")) {
			now_ms = heat.started_ms + CONFIG_SENSOR_TCAL_HEATED_TIMEOUT_MIN * 60000;
			sample.sampled_at_ms = now_ms;
			sample.sequence++;
		} else model_generation++;
		sensor_tcal_heated_finalize();
		assert(!heat.applied && !sensor_tcal_heated_busy() && !imu_reserved && !output_duty);
		unchanged();
	} else {
		assert(!"unknown scenario");
	}
	assert(!gate_depth && !model_depth && !storage_depth);
	printf("%s passed\n", which);
	return 0;
}
