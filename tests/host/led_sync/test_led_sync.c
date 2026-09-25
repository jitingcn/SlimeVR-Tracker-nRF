#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "../../../src/system/led.c"

struct sample { int64_t time; unsigned value; struct led_rgb rgb; };
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

static int64_t ms(unsigned value) { return ((uint64_t)value * CONFIG_SYS_CLOCK_TICKS_PER_SEC + 999) / 1000; }
int64_t k_uptime_ticks(void) { return now_ticks; }
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
		assert(gate_off > 0 && led_quiesced.count);
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
	if (spec->pin == 2 && !value) { gate_off++; }
#if !HOST_PWM && !CONFIG_LED_STRIP
	if (spec->pin == 1) { record(value); }
#endif
	return 0;
}
int pwm_set_pulse_dt(const struct pwm_dt_spec *spec, uint32_t value) { (void)spec; record(value); return 0; }
int pm_device_action_run(const struct device *dev, int action) { (void)dev; (void)action; return 0; }
int led_strip_update_rgb(const struct device *dev, struct led_rgb *rgb, size_t count)
{
	(void)dev;
	assert(count == 1);
	record(rgb[0].r + rgb[0].g + rgb[0].b);
	samples[sample_count - 1].rgb = rgb[0];
	return 0;
}
static void reset(void)
{
	for (int i = 0; i < SYS_LED_PATTERN_DEPTH; i++) { led_patterns[i] = SYS_LED_PATTERN_OFF; led_generations[i] = 0; }
	led_changed.count = 0;
	led_quiesced.count = 0; shutdown_pending = false;
	now_ticks = 0; sample_count = 0; event_count = 0; next_event = 0;
	clock_calls = waits = gate_off = 0; synced = false; offset = 0; output_hook = NULL;
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
	assert(gate_off == 1);
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
	assert(gate_off == 1);
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
	assert(gate_off == 1 && led_quiesced.count == 1);
	assert(samples[sample_count - 1].value == 0);
	reset();
	set_led(SYS_LED_PATTERN_ON, 0);
	led_shutdown();
	assert(gate_off == 1 && led_patterns[0] == SYS_LED_PATTERN_OFF_FORCE);
	/* The next caller must not consume the previous caller's acknowledgment. */
	led_shutdown();
	assert(gate_off == 2);
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
#if CONFIG_LED_STRIP
	test_pulse_requests_preserve_frames();
	test_new_pattern_renders_matching_first_frame();
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
