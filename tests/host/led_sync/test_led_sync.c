#include <assert.h>
#include <errno.h>
#include <stdarg.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "../../../src/system/led.c"

struct sample { int64_t time; unsigned value; struct led_rgb rgb; int result; };
static struct sample samples[20000];
static unsigned sample_count;
static int64_t now_ticks, stop_ticks;
static jmp_buf stopped;
static jmp_buf shutdown_stopped;
static bool driving_shutdown;
static bool synced;
static uint32_t offset;
static unsigned clock_calls, waits, gate_off;
static void (*output_hook)(void);
struct event { int64_t at; enum sys_led_pattern pattern; int owner; int sync; uint32_t offset; };
static struct event events[32];
static unsigned event_count, next_event;
enum action_kind { GATE_ON, GATE_OFF, RESUME, SUSPEND, SLEEP, WRITE_RETURN };
struct action { enum action_kind kind; int64_t time; int value; };
static struct action actions[30000];
static unsigned action_count;
static int strip_failures;
static bool mutate_strip_buffer;
struct error_log { int64_t time; char text[128]; };
static struct error_log error_logs[64];
static unsigned error_log_count;

static void action(enum action_kind kind, int value)
{
	assert(action_count < sizeof(actions) / sizeof(actions[0]));
	actions[action_count++] = (struct action){kind, now_ticks, value};
}

void host_log_error(const char *format, ...)
{
	assert(error_log_count < sizeof(error_logs) / sizeof(error_logs[0]));
	struct error_log *entry = &error_logs[error_log_count++];
	entry->time = now_ticks;
	va_list args;
	va_start(args, format);
	vsnprintf(entry->text, sizeof(entry->text), format, args);
	va_end(args);
}

static int64_t ms(unsigned value) { return ((uint64_t)value * CONFIG_SYS_CLOCK_TICKS_PER_SEC + 999) / 1000; }
int64_t k_uptime_ticks(void) { return now_ticks; }
int32_t k_msleep(int32_t duration)
{
	action(SLEEP, duration);
	now_ticks += ms(duration);
	return 0;
}
bool esb_get_status_clock(uint32_t *local, uint32_t *network)
{
	clock_calls++;
	*local = led_local_ticks();
	*network = synced ? *local + offset : *local;
	return synced;
}
int k_sem_take(struct k_sem *sem, k_timeout_t timeout)
{
	if (driving_shutdown && led_quiesced.count) {
		longjmp(shutdown_stopped, 1);
	}
	if (sem == &led_quiesced && !sem->count) {
		driving_shutdown = true;
		if (!setjmp(shutdown_stopped)) { led_thread(); }
		driving_shutdown = false;
		assert((!HOST_GATE || gate_off > 0) && led_quiesced.count);
		sem->count = 0;
		return 0;
	}
	waits++;
	assert(waits < 100000);
	if (sem->count) { sem->count = 0; return 0; }
	int64_t wake = timeout == K_FOREVER ? INT64_MAX : now_ticks + timeout;
	if (next_event < event_count && events[next_event].at <= wake) {
		struct event e = events[next_event++];
		now_ticks = e.at;
		if (e.sync >= 0) { synced = e.sync; offset = e.offset; }
		if (e.owner >= 0) { set_led(e.pattern, e.owner); }
		if (e.owner == -2) { shutdown_pending = true; }
		return 0;
	}
	if (wake >= stop_ticks) { now_ticks = stop_ticks; longjmp(stopped, 1); }
	assert(wake > now_ticks);
	now_ticks = wake;
	return -1;
}
static void record(unsigned value)
{
	assert(sample_count < sizeof(samples) / sizeof(samples[0]));
	samples[sample_count++] = (struct sample){.time = now_ticks, .value = value};
	if (output_hook) { output_hook(); }
}
int gpio_pin_configure_dt(const struct gpio_dt_spec *spec, int flags) { (void)spec; (void)flags; return 0; }
int gpio_pin_set_dt(const struct gpio_dt_spec *spec, int value)
{
	if (spec->pin == 2) {
		action(value ? GATE_ON : GATE_OFF, value);
		if (!value) { gate_off++; }
	}
#if !HOST_PWM && !CONFIG_LED_STRIP
	if (spec->pin == 1) { record(value); }
#endif
	return 0;
}
int pwm_set_pulse_dt(const struct pwm_dt_spec *spec, uint32_t value) { (void)spec; record(value); return 0; }
int pm_device_action_run(const struct device *dev, int pm_action)
{
	(void)dev;
	action(pm_action == PM_DEVICE_ACTION_RESUME ? RESUME : SUSPEND, 0);
	return 0;
}
int led_strip_update_rgb(const struct device *dev, struct led_rgb *rgb, size_t count)
{
	(void)dev;
	assert(count == 1);
	record(rgb[0].r + rgb[0].g + rgb[0].b);
	samples[sample_count - 1].rgb = rgb[0];
	int result = strip_failures ? -EIO : 0;
	if (strip_failures > 0) { strip_failures--; }
	samples[sample_count - 1].result = result;
	/* The API permits a driver to use the RGB array as scratch space. */
	if (mutate_strip_buffer) { rgb[0] = (struct led_rgb){0}; }
	action(WRITE_RETURN, result);
	return result;
}
static void reset(void)
{
	for (int i = 0; i < SYS_LED_PATTERN_DEPTH; i++) { led_patterns[i] = SYS_LED_PATTERN_OFF; led_generations[i] = 0; }
	led_changed.count = 0;
	led_quiesced.count = 0; shutdown_pending = false;
	now_ticks = 0; sample_count = 0; event_count = 0; next_event = 0;
	clock_calls = waits = gate_off = 0; synced = false; offset = 0; output_hook = NULL;
	action_count = 0; strip_failures = 0; mutate_strip_buffer = false;
	error_log_count = 0;
#if CONFIG_LED_STRIP
	led_strip_fade_reset(&led_fade);
	strip_error_logged = false;
	strip_error_log_ticks = 0;
#endif
}
static void run(unsigned duration_ms)
{
	stop_ticks = now_ticks + ms(duration_ms);
	if (!setjmp(stopped)) { led_thread(); }
}
static void event(unsigned at_ms, enum sys_led_pattern pattern, int owner)
{
	events[event_count++] = (struct event){ms(at_ms), pattern, owner, -1, 0};
}
static unsigned on_count(void)
{
	unsigned count = 0;
	for (unsigned i = 0; i < sample_count; i++) { count += samples[i].value != 0; }
	return count;
}
static void test_active_off_first(void)
{
	reset();
	set_led(SYS_LED_PATTERN_ACTIVE_PERSIST, 4);
	event(2000, SYS_LED_PATTERN_ACTIVE_PERSIST, 4);
	event(9000, SYS_LED_PATTERN_ACTIVE_PERSIST, 4);
	run(20500);
	assert(on_count() == 2);
	int64_t first = -1, second = -1;
	for (unsigned i = 0; i < sample_count; i++) {
		if (samples[i].value) { if (first < 0) first = samples[i].time; else second = samples[i].time; }
	}
	assert(first >= ms(9700) && first <= ms(9701));
	assert(second - first >= ms(9999) && second - first <= ms(10001));
	assert(waits < 200); /* not a 5ms poller */
}
static void test_same_pattern_owner(void)
{
	reset();
	set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, 4);
	event(300, SYS_LED_PATTERN_ONESHOT_PROGRESS, 2);
	event(400, SYS_LED_PATTERN_OFF, 4);
	event(600, SYS_LED_PATTERN_ONESHOT_PROGRESS, 2);
	run(1200);
	assert(on_count() == 2);
	assert(led_patterns[2] == SYS_LED_PATTERN_OFF);
	assert(gate_off == HOST_GATE);
	reset();
	set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, 4);
	event(300, SYS_LED_PATTERN_ONESHOT_PROGRESS, 2);
	run(2000);
	assert(on_count() == 4);
	assert(led_patterns[2] == SYS_LED_PATTERN_OFF && led_patterns[4] == SYS_LED_PATTERN_OFF);
}
static void replace_during_final_frame(void)
{
	if (now_ticks >= ms(800)) {
		/* Simulate a request arriving while a synchronous driver owns the buffer. */
		set_led(SYS_LED_PATTERN_ON, 2);
		now_ticks += ms(2);
		output_hook = NULL;
	}
}
static void test_stale_completion(void)
{
	reset();
	set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, 2);
	output_hook = replace_during_final_frame;
	run(1200);
	assert(led_patterns[2] == SYS_LED_PATTERN_ON);
	assert(samples[sample_count - 1].value != 0);
}
static void test_priority_and_interruptible_wait(void)
{
	reset();
	set_led(SYS_LED_PATTERN_ACTIVE_PERSIST, 4);
	event(100, SYS_LED_PATTERN_ERROR_A, 3);
	event(200, SYS_LED_PATTERN_OFF_FORCE, 0);
	run(1000);
	assert(on_count() == 1);
	assert(gate_off == HOST_GATE);
	assert(samples[sample_count - 1].time == ms(200));
	assert(samples[sample_count - 1].value == 0);
}
static void test_solid_idle(void)
{
	reset(); set_led(SYS_LED_PATTERN_ON, 4);
	event(500, SYS_LED_PATTERN_ON, 4);
	run(10000);
	assert(on_count() == 1 && waits < 8);
	reset(); run(10000); assert(sample_count == 0 && waits == 1 && clock_calls == 0);
}
static void test_counts(void)
{
	const enum sys_led_pattern patterns[] = {SYS_LED_PATTERN_ONESHOT_POWERON, SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PATTERN_ONESHOT_PING};
	const unsigned counts[] = {3, 4, 10};
	for (unsigned i = 0; i < 3; i++) {
		reset(); set_led(patterns[i], 1); run(5000);
		assert(on_count() == counts[i]);
		assert(led_patterns[1] == SYS_LED_PATTERN_OFF);
	}
}
static void test_phase_math(void)
{
	assert(led_sync_phase(5U * LED_SYNC_HZ, 5U * LED_SYNC_HZ) == 0);
	assert(led_sync_pulse(0) == 0);
	assert(led_sync_pulse(5U * LED_SYNC_HZ / 2U) == 10000);
	assert(led_sync_until(UINT32_MAX, 100) == 1);
	assert(led_sync_until(UINT32_MAX - 20, 100) == 21);
	assert(led_sync_until(0, 100) == 100);
	struct led_sync_start start = {.entered = UINT32_MAX - 1000};
	uint32_t elapsed = LED_SYNC_ACTIVE_OFF - 1;
	assert(!led_sync_active(&start, start.entered + elapsed, LED_SYNC_ACTIVE_OFF - 1));
	assert(led_sync_active(&start, start.entered + elapsed + 1, LED_SYNC_ACTIVE_OFF));
}
#if CONFIG_LED_NETWORK_SYNC
static void test_late_join_and_loss(void)
{
	reset(); synced = true; offset = 2U * LED_SYNC_HZ;
	set_led(SYS_LED_PATTERN_ACTIVE_PERSIST, 4);
	events[event_count++] = (struct event){ms(18000), 0, -1, 0, 0};
	run(28500);
	assert(on_count() == 2);
	unsigned first = 0;
	while (!samples[first].value) first++;
	assert(samples[first].time >= ms(17700) && samples[first].time <= ms(17701));
	assert(clock_calls < 150);
}
static void test_clock_adoption(void)
{
	reset();
	set_led(SYS_LED_PATTERN_ACTIVE_PERSIST, 4);
	events[event_count++] = (struct event){ms(2000), 0, -1, 1, 2U * LED_SYNC_HZ};
	run(28100);
	assert(on_count() == 2);
	unsigned first = 0; while (!samples[first].value) first++;
	assert(samples[first].time >= ms(17700) && samples[first].time <= ms(17701));
}
static void test_raw_wrap(void)
{
	reset(); synced = true; offset = UINT32_MAX - LED_SYNC_HZ / 4U;
	set_led(SYS_LED_PATTERN_LONG_PERSIST, 4);
	run(2500);
	assert(on_count() == 3);
	/* Raw wrap is an on edge, followed by a normal 500ms half-cycle. */
	unsigned first = 0; while (!samples[first].value) first++;
	assert(samples[first].time >= ms(250) && samples[first].time <= ms(251));
	assert(samples[first + 1].value == 0);
	assert(samples[first + 1].time - samples[first].time >= ms(499));
}
#else
static void test_disabled_clock(void)
{
	reset(); synced = true; offset = 1234567;
	set_led(SYS_LED_PATTERN_LONG_PERSIST, 4); run(2200);
	assert(on_count() == 3 && clock_calls == 0);
}
#endif
static void shutdown_during_output(void)
{
	/* A concurrently published shutdown must win even if a normal highest
	 * priority request arrives before the worker next snapshots requests. */
	shutdown_pending = true;
	set_led(SYS_LED_PATTERN_ON, 0);
	output_hook = NULL;
}
static void test_shutdown_barrier(void)
{
	reset();
	set_led(SYS_LED_PATTERN_ON, 4);
	output_hook = shutdown_during_output;
	run(100);
	assert(gate_off == HOST_GATE && led_quiesced.count == 1);
	assert(samples[sample_count - 1].value == 0);
	reset();
	set_led(SYS_LED_PATTERN_ON, 0);
	led_shutdown();
	assert(gate_off == HOST_GATE && led_patterns[0] == SYS_LED_PATTERN_OFF_FORCE);
	/* The next caller must not consume the previous caller's acknowledgment. */
	led_shutdown();
	assert(gate_off == 2 * HOST_GATE);
}

static void slow_output(void)
{
	now_ticks += ms(2);
}
static void test_pulse_period_with_blocking_driver(void)
{
	reset();
	set_led(SYS_LED_PATTERN_PULSE_PERSIST, 4);
	output_hook = slow_output;
	run(10500);
	unsigned peak = 0, trough = UINT_MAX;
	int64_t first_peak = 0, second_peak = 0;
	for (unsigned i = 0; i < sample_count; i++) {
		if (samples[i].time < ms(5000) && samples[i].value > peak) {
			peak = samples[i].value; first_peak = samples[i].time;
		}
		if (samples[i].time >= ms(5000) && samples[i].time < ms(5010)) {
			if (samples[i].value < trough) trough = samples[i].value;
		}
	}
	for (unsigned i = 0; i < sample_count; i++) {
		if (samples[i].time >= ms(5000) && samples[i].value >= peak) {
			second_peak = samples[i].time; break;
		}
	}
	assert(peak > 0 && trough <= peak / 100 + 1);
	assert(second_peak - first_peak >= ms(4950));
	assert(second_peak - first_peak <= ms(5050));
}

static void test_power_cycle_margins(void)
{
	reset();
	set_led(SYS_LED_PATTERN_ON, 4);
	event(50, SYS_LED_PATTERN_ON_PERSIST, 4); /* Already powered: no settling. */
	event(100, SYS_LED_PATTERN_OFF, 4);
	event(200, SYS_LED_PATTERN_ON, 4);
	event(250, SYS_LED_PATTERN_ON, 4);
	event(300, SYS_LED_PATTERN_OFF, 4);
	run(500);
	unsigned starts = 0, stops = 0, sleeps = 0;
	for (unsigned i = 0; i < action_count; i++) {
		struct action a = actions[i];
		if (a.kind == GATE_ON) {
			assert(a.time == ms(starts ? 200 : 0));
			starts++;
#if CONFIG_LED_STRIP
			assert(actions[i + 1].kind == SLEEP && actions[i + 1].value == 2);
			assert(actions[i + 2].kind == RESUME);
			assert(actions[i + 2].time == a.time + ms(2));
#endif
		}
		if (a.kind == GATE_OFF) {
			int64_t margin = CONFIG_LED_STRIP ? ms(1) : 0;
			assert(a.time == ms(stops ? 300 : 100) + margin);
			stops++;
		}
		if (a.kind == SLEEP) {
			sleeps++;
			if (a.value == 1) {
				assert(i > 0 && actions[i - 1].kind == WRITE_RETURN);
				assert(actions[i - 1].time == a.time);
				assert(actions[i + 1].kind == SUSPEND);
				assert(actions[i + 1].time == a.time + ms(1));
			} else {
				assert(a.value == 2 && HOST_GATE);
			}
		}
	}
	assert(starts == 2 * HOST_GATE && stops == 2 * HOST_GATE);
	assert(sleeps == (CONFIG_LED_STRIP ? 2U + 2U * HOST_GATE : 0U));
#if CONFIG_LED_STRIP
	assert(samples[0].time == (HOST_GATE ? ms(2) : 0));
	assert(samples[1].time == ms(50));
#endif
}

#if CONFIG_LED_STRIP
static void assert_rgb(struct led_rgb actual, struct led_rgb expected)
{
	assert(actual.r == expected.r);
	assert(actual.g == expected.g);
	assert(actual.b == expected.b);
}

static void test_pulse_requests_preserve_frames(void)
{
	static struct sample baseline[2000];
	reset();
	set_led(SYS_LED_PATTERN_PULSE_PERSIST, 4);
	run(6000);
	unsigned count = sample_count;
	assert(count <= sizeof(baseline) / sizeof(baseline[0]));
	memcpy(baseline, samples, count * sizeof(samples[0]));
	for (unsigned transfer = 0; transfer < 2; transfer++) {
		reset();
		set_led(SYS_LED_PATTERN_PULSE_PERSIST, 4);
		int owner = transfer ? 2 : 4;
		event(25, SYS_LED_PATTERN_PULSE_PERSIST, owner);
		event(375, SYS_LED_PATTERN_PULSE_PERSIST, owner);
		event(900, SYS_LED_PATTERN_PULSE_PERSIST, owner);
		event(2500, SYS_LED_PATTERN_PULSE_PERSIST, owner);
		run(6000);
		assert(sample_count == count);
		for (unsigned i = 0; i < count; i++) {
			assert(samples[i].time == baseline[i].time);
			assert_rgb(samples[i].rgb, baseline[i].rgb);
		}
	}
}

static void test_new_pattern_renders_matching_first_frame(void)
{
	reset();
	set_led(SYS_LED_PATTERN_LONG, 4);
	event(100, SYS_LED_PATTERN_ON, 4);
	run(1000);
	/* Both patterns start at full default color; the steady frame still
	 * needs its own fresh rounding, rather than reusing the flashing frame. */
	assert(sample_count == 2);
	assert(samples[1].time == ms(100));
	assert_rgb(samples[1].rgb, samples[0].rgb);
	assert(gate_off == 0);
}

static void test_steady_retry_recovery(void)
{
	const enum sys_led_pattern patterns[] = {SYS_LED_PATTERN_ON, SYS_LED_PATTERN_ON_PERSIST};
	for (unsigned p = 0; p < 2; p++) {
		reset();
		strip_failures = 1;
		set_led(patterns[p], 4);
		event(25, patterns[p], 4);
		event(500, patterns[p], 4);
		run(1000);
		assert(sample_count == 2);
		assert(samples[0].result == -EIO && samples[1].result == 0);
		assert(samples[1].time - samples[0].time == ms(100));
		assert_rgb(samples[1].rgb, samples[0].rgb);
		assert(error_log_count == 1);
	}
}

static void test_steady_retry_exhaustion(void)
{
	reset();
	strip_failures = -1;
	set_led(SYS_LED_PATTERN_ON, 4);
	event(25, SYS_LED_PATTERN_ON, 4);
	event(150, SYS_LED_PATTERN_ON, 4);
	event(500, SYS_LED_PATTERN_ON, 4);
	event(750, SYS_LED_PATTERN_ON, 2); /* Inheritance is not a restart. */
	run(3000);
	assert(sample_count == 3 && waits < 20);
	for (unsigned i = 0; i < 3; i++) {
		assert(samples[i].result == -EIO);
		assert_rgb(samples[i].rgb, samples[0].rgb);
		if (i) { assert(samples[i].time - samples[i - 1].time == ms(100)); }
	}
	reset();
	strip_failures = -1;
	set_led(SYS_LED_PATTERN_ON, 4);
	event(500, SYS_LED_PATTERN_ON_PERSIST, 4);
	run(2000);
	assert(sample_count == 6);
	assert(samples[3].time == ms(500));
	assert(samples[4].time - samples[3].time == ms(100));
	assert(samples[5].time - samples[4].time == ms(100));
}

static void test_retry_preemption_and_failed_black(void)
{
	reset();
	strip_failures = 1;
	set_led(SYS_LED_PATTERN_ON, 4);
	event(50, SYS_LED_PATTERN_ON_PERSIST, 2);
	run(500);
	assert(sample_count == 2 && samples[1].time == ms(50));
	assert(samples[1].result == 0 && samples[1].rgb.r == 0 && samples[1].rgb.g > 0);
	for (unsigned shutdown = 0; shutdown < 2; shutdown++) {
		reset();
		strip_failures = -1;
		set_led(SYS_LED_PATTERN_ON, 4);
		event(50, SYS_LED_PATTERN_OFF_FORCE, shutdown ? -2 : 0);
		run(1000);
		assert(sample_count == 2);
		assert(samples[1].time == ms(50) && samples[1].value == 0);
		assert(samples[1].result == -EIO && gate_off == HOST_GATE);
		assert(led_quiesced.count == shutdown);
		bool suspended = false;
		for (unsigned i = 0; i < action_count; i++) {
			if (actions[i].kind == SUSPEND) {
				assert(actions[i].time == ms(50) + ms(1));
				suspended = true;
			}
			if (actions[i].kind == GATE_OFF) {
				assert(suspended && actions[i].time == ms(50) + ms(1));
			}
		}
		assert(suspended);
	}
}

static void test_failed_fade_preserves_carry(void)
{
	struct led_rgb baseline[100];
	reset();
	for (unsigned i = 0; i < 100; i++) {
		assert(led_pin_set(SYS_LED_COLOR_DEFAULT, 137 + 31 * i, 10000));
		baseline[i] = samples[i].rgb;
	}
	reset();
	for (unsigned i = 0; i < 100; i++) {
		strip_failures = 1;
		assert(!led_pin_set(SYS_LED_COLOR_DEFAULT, 137 + 31 * i, 10000));
		assert_rgb(samples[2 * i].rgb, baseline[i]);
		assert(led_pin_set(SYS_LED_COLOR_DEFAULT, 137 + 31 * i, 10000));
		assert_rgb(samples[2 * i + 1].rgb, baseline[i]);
	}
}

static void test_failed_fade_keeps_cadence(void)
{
	reset();
	strip_failures = -1;
	set_led(SYS_LED_PATTERN_PULSE_PERSIST, 4);
	run(1100);
	assert(sample_count >= 210 && sample_count <= 225);
	for (unsigned i = 1; i < sample_count; i++) {
		assert(samples[i].result == -EIO);
		assert(samples[i].time - samples[i - 1].time >= ms(4));
		assert(samples[i].time - samples[i - 1].time <= ms(6));
	}
	unsigned sleeps = 0;
	for (unsigned i = 0; i < action_count; i++) { sleeps += actions[i].kind == SLEEP; }
	assert(sleeps == HOST_GATE);
	assert(error_log_count == 2);
	assert(error_logs[1].time - error_logs[0].time >= ms(1000));
}

static void test_error_log_boundary(void)
{
	reset();
	strip_failures = -1;
	mutate_strip_buffer = true;
	const unsigned times[] = {0, 999, 1000, 1999, 2000};
	const unsigned counts[] = {1, 1, 2, 2, 3};
	for (unsigned i = 0; i < 5; i++) {
		now_ticks = ms(times[i]);
		assert(!led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, 10000));
		assert(error_log_count == counts[i]);
	}
	char expected[128];
	snprintf(expected, sizeof(expected), "strip RGB %u/%u/%u update failed: %d",
		samples[0].rgb.r, samples[0].rgb.g, samples[0].rgb.b, -EIO);
	for (unsigned i = 0; i < 3; i++) {
		assert(error_logs[i].time == ms(i * 1000));
		assert(strcmp(error_logs[i].text, expected) == 0);
	}
}
#endif

#if HOST_P10
static void test_pulse_to_steady_brightness(void)
{
	const unsigned transition_ms[] = {25, 375, 900, 925, 1000};
	const enum sys_led_pattern patterns[] = {
		SYS_LED_PATTERN_ON, SYS_LED_PATTERN_ON_PERSIST,
	};
	/* P10 default/success colors at 10% global brightness, with a fresh
	 * dither accumulator. These are fixed physical channel levels. */
	const struct led_rgb expected[] = {{26, 8, 13}, {0, 5, 0}};
	for (unsigned p = 0; p < sizeof(patterns) / sizeof(patterns[0]); p++) {
		for (unsigned t = 0; t < sizeof(transition_ms) / sizeof(transition_ms[0]); t++) {
			reset();
			set_led(SYS_LED_PATTERN_PULSE_PERSIST, 4);
			event(transition_ms[t], patterns[p], 4);
			run(transition_ms[t] + 1000);
			unsigned steady_frames = 0;
			for (unsigned i = 0; i < sample_count; i++) {
				if (samples[i].time >= ms(transition_ms[t])) {
					assert(samples[i].time == ms(transition_ms[t]));
					assert_rgb(samples[i].rgb, expected[p]);
					steady_frames++;
				}
			}
			assert(steady_frames == 1);
			assert(gate_off == 0);
		}
	}
}
#endif

int main(void)
{
	test_phase_math(); test_active_off_first(); test_same_pattern_owner();
	test_stale_completion(); test_priority_and_interruptible_wait();
	test_solid_idle(); test_counts();
	test_shutdown_barrier();
	test_pulse_period_with_blocking_driver();
	test_power_cycle_margins();
#if CONFIG_LED_STRIP
	test_pulse_requests_preserve_frames();
	test_new_pattern_renders_matching_first_frame();
	test_steady_retry_recovery();
	test_steady_retry_exhaustion();
	test_retry_preemption_and_failed_black();
	test_failed_fade_preserves_carry();
	test_failed_fade_keeps_cadence();
	test_error_log_boundary();
#endif
#if HOST_P10
	test_pulse_to_steady_brightness();
#endif
#if CONFIG_LED_NETWORK_SYNC
	test_late_join_and_loss(); test_clock_adoption(); test_raw_wrap();
#else
	test_disabled_clock();
#endif
	puts("production LED worker: phase, ownership, priority, off-first, wrap passed");
	return 0;
}
