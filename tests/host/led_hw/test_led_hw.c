#include "globals.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

const struct device host_device = {.id = 1};
static bool device_ready = true;
static bool gate_on;
static int output_error;
static int resume_error;
static unsigned writes;
static unsigned gate_off_writes;
static unsigned suspends;
static unsigned slept_ms;
static uint32_t output[3];
static struct led_rgb attempted_pixel;
static uint64_t host_hw_ms;
static bool capture_active, smoke_output;
static unsigned driver_delay_ms;
static uint8_t held_trace[2001][3];
static uint32_t capture_origin_ms = 2000;
static void hw_advance(uint64_t to_ms)
{
	assert(to_ms >= host_hw_ms);
	if (capture_active) {
		/* Record each actually held byte, including producer lateness and
		 * driver time before the new frame becomes visible. Never average it. */
		for (uint64_t time = host_hw_ms; time < to_ms; time++) {
			if (time >= capture_origin_ms && time <= capture_origin_ms + 2000) {
				for (unsigned channel = 0; channel < 3; channel++) {
					held_trace[time - capture_origin_ms][channel] = output[channel];
				}
			}
		}
	}
	host_hw_ms = to_ms;
}
int64_t k_uptime_get(void) { return host_hw_ms; }

bool device_is_ready(const struct device *device) { (void)device; return device_ready; }
bool gpio_is_ready_dt(const struct gpio_dt_spec *gpio) { (void)gpio; return device_ready; }
bool pwm_is_ready_dt(const struct pwm_dt_spec *pwm) { (void)pwm; return device_ready; }
int gpio_pin_configure_dt(const struct gpio_dt_spec *gpio, int flags)
{
	(void)flags;
	if (gpio->pin == 8) { gate_on = false; }
	return device_ready ? 0 : -ENODEV;
}
int gpio_pin_set_dt(const struct gpio_dt_spec *gpio, int value)
{
	if (gpio->pin == 8) {
		gate_on = value;
		gate_off_writes += value == 0;
		return 0;
	}
	writes++;
	if (output_error) { return output_error; }
	assert(gpio->pin >= 10 && gpio->pin < 13);
	output[gpio->pin - 10] = value;
	return 0;
}
int pwm_set_pulse_dt(const struct pwm_dt_spec *pwm, uint32_t pulse)
{
	writes++;
	assert(pwm->flags == HOST_POLARITY);
	assert(pulse <= pwm->period);
	if (output_error) { return output_error; }
	output[pwm->channel] = pulse;
	return 0;
}
int pm_device_action_run(const struct device *device, enum pm_device_action action)
{
	(void)device;
	if (action == PM_DEVICE_ACTION_SUSPEND) { suspends++; }
	return action == PM_DEVICE_ACTION_RESUME ? resume_error : 0;
}
int led_strip_update_rgb(const struct device *device, struct led_rgb *rgb, size_t count)
{
	(void)device;
	assert(count == 1);
	writes++;
	attempted_pixel = *rgb;
	if (driver_delay_ms) { hw_advance(host_hw_ms + driver_delay_ms); }
	if (output_error) { return output_error; }
	output[0] = rgb->r; output[1] = rgb->g; output[2] = rgb->b;
	return 0;
}
int32_t k_msleep(int32_t duration_ms)
{
	assert(duration_ms == 2);
	slept_ms += duration_ms;
	hw_advance(host_hw_ms + duration_ms);
	return 0;
}

/* Production code, not a parallel renderer or copied mapping implementation. */
#ifndef LED_HW_SOURCE
#define LED_HW_SOURCE "../../../src/system/led_hw.c"
#endif
#include LED_HW_SOURCE
#include "../../../src/system/led_timeline.c"
#include "../../../src/system/led_behavior.c"

static unsigned lit_channels(void)
{
	return (output[0] != 0) + (output[1] != 0) + (output[2] != 0);
}

static void test_role_mapping_and_black(void)
{
	struct led_hardware_info info;
	led_hw_init();
	led_hw_info(&info);
	assert(info.capability == HOST_KIND);
	assert(info.gpio == !!HOST_GPIO_TRANSPORT);
	assert(info.visible_min_pptt == 0); /* Unmeasured remains explicit. */
	assert(led_hw_write(LED_ROLE_NEGATIVE, LED_LEVEL_NOTICE, 10000, -1, NULL));
#if HOST_KIND == 0 || CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT == 0
	assert(!lit_channels() && !gate_on && !slept_ms);
#if HOST_KIND == 0
	assert(writes == 0 && suspends == 0);
#endif
#else
	assert(lit_channels() == 1);
#if defined(HOST_UNKNOWN_COLORS)
	assert(output[HOST_PRIMARY] != 0);
#endif
#if HOST_GATE
	assert(gate_on);
#endif
	unsigned stable_writes = writes;
	assert(led_hw_write(LED_ROLE_NEGATIVE, LED_LEVEL_NOTICE, 10000, -1, NULL));
	assert(writes == stable_writes); /* No constant-output peripheral rewrite. */
	assert(led_hw_write(LED_ROLE_LINK, LED_LEVEL_TASK, 10000, -1, NULL));
	assert(lit_channels() == 1);
#if defined(HOST_UNKNOWN_COLORS)
	assert(output[HOST_PRIMARY] != 0);
#endif
#if HOST_KIND == 6
#ifdef HOST_RBG
	assert(output[1] && !output[0] && !output[2]);
#else
	assert(output[2] && !output[0] && !output[1]);
#endif
#elif HOST_KIND == 7
	assert(output[2] && !output[0] && !output[1]);
#endif
	assert(led_hw_write(LED_ROLE_POSITIVE, LED_LEVEL_NOTICE, 10000, -1, NULL));
#if HOST_KIND == 6
#ifdef HOST_RBG
	assert(output[2] && !output[0] && !output[1]);
#else
	assert(output[1] && !output[0] && !output[2]);
#endif
#elif HOST_KIND == 7
	assert(output[1] && !output[0] && !output[2]);
#endif
	assert(led_hw_write(LED_ROLE_NEUTRAL, LED_LEVEL_BACKGROUND, 10000, -1, NULL));
	assert(lit_channels() != 0); /* Default READY/HOLD never quantizes to black. */
#if HOST_KIND == 3 || HOST_KIND == 5 || HOST_KIND == 1 || HOST_KIND == 2
	assert(lit_channels() == 1); /* Discrete lamps never invent mixed colors. */
#endif
#endif
	led_hw_off();
	assert(!lit_channels() && !gate_on);
	led_hw_info(&info);
	assert(!info.powered);
}

static void test_raw_capability(void)
{
#if HOST_KIND == 0 || HOST_KIND == 1 || HOST_KIND == 2 || defined(HOST_UNKNOWN_COLORS)
	assert(!led_hw_color_supported(LED_PHYSICAL_BLUE));
#else
	assert(led_hw_color_supported(LED_PHYSICAL_RED));
	assert(led_hw_color_supported(LED_PHYSICAL_GREEN));
#endif
#if HOST_KIND == 4
	assert(led_hw_color_supported(LED_PHYSICAL_AMBER));
	assert(!led_hw_color_supported(LED_PHYSICAL_BLUE));
	assert(!led_hw_color_supported(LED_PHYSICAL_WHITE));
#endif
#if HOST_KIND == 3 || HOST_KIND == 5
	assert(!led_hw_color_supported(LED_PHYSICAL_AMBER));
	assert(!led_hw_color_supported(LED_PHYSICAL_WHITE));
#endif
	assert(!led_hw_color_supported((enum led_physical_color)99));
#if HOST_KIND != 0
	assert(led_hw_write(LED_ROLE_NEUTRAL, LED_LEVEL_TASK, 10000, -1, NULL));
	uint32_t old[3]; memcpy(old, output, sizeof(old));
	unsigned old_writes = writes;
	assert(!led_hw_write(LED_ROLE_NEUTRAL, LED_LEVEL_NOTICE, 10000, 99, NULL));
	assert(writes == old_writes && memcmp(old, output, sizeof(old)) == 0);
#endif
}

static void test_power_role_and_raw_amber(void)
{
#if HOST_KIND != 0 && CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT > 0
	assert(led_hw_write(LED_ROLE_POWER, LED_LEVEL_NOTICE, 10000, -1, NULL));
#if HOST_KIND == 4 || HOST_KIND == 6 || HOST_KIND == 7
	unsigned red = 0, green = 1;
#ifdef HOST_RBG
	green = 2;
#endif
	assert(output[red] && output[green] && lit_channels() == 2);
#if HOST_KIND == 7
	int64_t ratio_error = (int64_t)output[red] * 35 - (int64_t)output[green] * 100;
	assert(ratio_error >= -100 && ratio_error <= 100); /* One green-channel byte of quantization. */
#else
	/* Consumer-visible amber is red-dominant with approximately 35% green.
	 * Check that direction, not sub-PPTT rounding inside the current limiter. */
	assert((uint64_t)output[green] * 100 >= (uint64_t)output[red] * 34);
	assert((uint64_t)output[green] * 100 <= (uint64_t)output[red] * 36);
#endif
	uint32_t semantic[3]; memcpy(semantic, output, sizeof(semantic));
	assert(led_hw_write(LED_ROLE_NEUTRAL, LED_LEVEL_NOTICE, 10000, LED_PHYSICAL_AMBER, NULL));
	assert(memcmp(semantic, output, sizeof(semantic)) == 0);
#else
	assert(lit_channels() == 1); /* No alternating lamps or accidental mixing. */
#endif
#endif
}

static void test_current_caps(void)
{
#if HOST_PWM_TRANSPORT && CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT > 0
	assert(led_hw_write(LED_ROLE_NEUTRAL, LED_LEVEL_NOTICE, 10000, -1, NULL));
	uint64_t sum = (uint64_t)output[0] + output[1] + output[2];
	uint64_t ceiling = (uint64_t)1000000 * CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT *
		HOST_BOARD_LIMIT * HOST_TOTAL_LIMIT / 1000000000000ULL;
	assert(sum <= ceiling);
#endif
#if HOST_KIND == 7 && CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT > 0
	static const uint16_t levels[] = {10000, 6000, 2000, 1};
	static const enum led_role roles[] = {LED_ROLE_NEUTRAL, LED_ROLE_POWER, LED_ROLE_ACTIVITY, LED_ROLE_INTERACTION};
	for (unsigned role = 0; role < sizeof(roles) / sizeof(roles[0]); role++) {
		for (unsigned level = 0; level < sizeof(levels) / sizeof(levels[0]); level++) {
			assert(led_hw_write(roles[role], levels[level], 10000, -1, NULL));
			uint64_t sum = (uint64_t)output[0] + output[1] + output[2];
			uint64_t ceiling = (uint64_t)255 * CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT *
				HOST_BOARD_LIMIT * HOST_TOTAL_LIMIT / 1000000000000ULL;
			assert(sum <= ceiling);
		}
	}
#endif
#if HOST_PWM_TRANSPORT && CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT > 0 && defined(HOST_UNKNOWN_COLORS)
	/* An absent scalar uses full scale; an explicit limit must not silently
	 * take the fallback. The nonzero primary fixture checks physical output. */
	uint64_t duty = (uint64_t)1000000 * CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT *
		HOST_BOARD_LIMIT * HOST_TOTAL_LIMIT / 1000000000000ULL;
	assert(output[HOST_PRIMARY] <= duty && duty - output[HOST_PRIMARY] <= 1);
#endif
}

static void test_failure_and_terminal_cutoff(void)
{
#if HOST_KIND != 0 && CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT > 0
	assert(led_hw_write(LED_ROLE_NEGATIVE, LED_LEVEL_NOTICE, 10000, -1, NULL));
	/* A mono GPIO cannot distinguish NEGATIVE and POSITIVE or brightness
	 * tiers. Inject the error at a real black -> lit edge, not a semantic
	 * change that correctly suppresses an unchanged binary hardware frame. */
	assert(led_hw_write(LED_ROLE_NEGATIVE, LED_LEVEL_NOTICE, 0, -1, NULL));
	assert(!lit_channels());
	unsigned before = writes;
	output_error = -EIO;
	assert(!led_hw_write(LED_ROLE_POSITIVE, LED_LEVEL_TASK, 10000, -1, NULL));
	assert(writes > before);
	assert(!lit_channels()); /* Failed ON frame did not make the requested result visible. */
	before = writes;
	output_error = 0;
	assert(led_hw_write(LED_ROLE_POSITIVE, LED_LEVEL_TASK, 10000, -1, NULL));
	assert(writes > before); /* A failed frame never enters the stable-frame cache. */
	assert(lit_channels() != 0); /* Recovery must actually make the result visible. */
	output_error = -EIO;
	unsigned gates = gate_off_writes;
	unsigned previous_suspends = suspends;
	assert(!led_hw_write(LED_ROLE_POSITIVE, LED_LEVEL_TASK, 0, -1, NULL));
#if HOST_GATE
	assert(!gate_on && gate_off_writes > gates);
#else
	(void)gates;
#endif
#if HOST_PWM_TRANSPORT || HOST_KIND == 7
	assert(suspends > previous_suspends);
#else
	(void)previous_suspends;
#endif
	struct led_hardware_info info;
	led_hw_info(&info);
	assert(info.last_error == -EIO);
	output_error = 0;
	before = writes;
	assert(led_hw_write(LED_ROLE_POSITIVE, LED_LEVEL_TASK, 0, -1, NULL));
	assert(writes > before); /* Recovery retries black through the real frame API. */
	before = writes;
	assert(led_hw_write(LED_ROLE_POSITIVE, LED_LEVEL_TASK, 0, -1, NULL));
	assert(writes == before);
	assert(!lit_channels());
#if HOST_PWM_TRANSPORT || HOST_KIND == 7
	resume_error = -EIO;
	assert(!led_hw_write(LED_ROLE_NEGATIVE, LED_LEVEL_NOTICE, 10000, -1, NULL));
	assert(!gate_on);
	resume_error = 0;
#endif
#endif
}

static void test_failed_pixel_frame_retry(void)
{
#if HOST_KIND == 7 && CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT > 0
	led_hw_off();
	assert(led_hw_write(LED_ROLE_NEGATIVE, LED_LEVEL_NOTICE, 10000, -1, NULL));
	uint32_t previous[3]; memcpy(previous, output, sizeof(previous));
	output_error = -EIO;
	assert(!led_hw_write(LED_ROLE_NEGATIVE, LED_LEVEL_NOTICE, 1900, -1, NULL));
	struct led_rgb failed = attempted_pixel;
	assert(memcmp(previous, output, sizeof(previous)) == 0);
	unsigned before = writes;
	output_error = 0;
	assert(led_hw_write(LED_ROLE_NEGATIVE, LED_LEVEL_NOTICE, 1900, -1, NULL));
	assert(writes == before + 1); /* Failure never commits the candidate to cache. */
	assert(output[0] == failed.r && output[1] == failed.g && output[2] == failed.b);
	before = writes;
	assert(led_hw_write(LED_ROLE_NEGATIVE, LED_LEVEL_NOTICE, 1900, -1, NULL));
	assert(writes == before);
#endif
}

static void test_interaction_activity_fallback(void)
{
#if HOST_KIND != 0 && CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT > 0
	assert(led_hw_write(LED_ROLE_NEUTRAL, LED_LEVEL_TASK, 10000, -1, NULL));
	uint32_t neutral[3]; memcpy(neutral, output, sizeof(neutral));
	assert(led_hw_write(LED_ROLE_INTERACTION, LED_LEVEL_TASK, 10000, -1, NULL));
#if HOST_KIND == 6 || HOST_KIND == 7
	unsigned green = 1, blue = 2;
#ifdef HOST_RBG
	green = 2; blue = 1;
#endif
	assert(output[0] == 0 && output[green] && output[blue]);
	assert(output[green] == output[blue]);
	assert(led_hw_write(LED_ROLE_ACTIVITY, LED_LEVEL_TASK, 10000, -1, NULL));
	assert(output[green] == 0 && output[0] && output[blue]);
	assert(output[0] < output[blue]);
	(void)neutral;
#else
	assert(memcmp(neutral, output, sizeof(neutral)) == 0);
	assert(led_hw_write(LED_ROLE_ACTIVITY, LED_LEVEL_TASK, 10000, -1, NULL));
	assert(memcmp(neutral, output, sizeof(neutral)) == 0);
#endif
#else
	unsigned before = writes;
	/* Muting preserves black/rail cutoff even if the preceding unsupported raw
	 * request left a diagnostic error; it does not promise a success indication. */
	(void)led_hw_write(LED_ROLE_INTERACTION, LED_LEVEL_NOTICE, 10000, -1, NULL);
	(void)led_hw_write(LED_ROLE_ACTIVITY, LED_LEVEL_NOTICE, 10000, -1, NULL);
	assert(!lit_channels() && !gate_on && writes == before);
#endif
}

static void test_pixel_hue_quantization(void)
{
#if HOST_KIND == 7 && CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT > 0
	static const enum led_role roles[] = {LED_ROLE_POWER, LED_ROLE_ACTIVITY, LED_ROLE_INTERACTION, LED_ROLE_NEUTRAL,
		LED_ROLE_NEGATIVE, LED_ROLE_POSITIVE, LED_ROLE_LINK};
	/* Independent physical RGB directions, including an asymmetric color
	 * whose weakest component is not green. Descriptor order may be RBG. */
	static const uint32_t colors[][3] = {{10000, 3500, 0}, {5500, 0, 10000},
		{0, 10000, 10000}, {10000, 10000, 10000}, {10000, 0, 0}, {0, 10000, 0}, {0, 0, 10000}};
	static const uint16_t weights[3] = {HOST_RED_WEIGHT, HOST_GREEN_WEIGHT, HOST_BLUE_WEIGHT};
	uint64_t ceiling = (uint64_t)255 * CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT
		* HOST_BOARD_LIMIT * HOST_TOTAL_LIMIT / 1000000000000ULL;
	uint32_t channel_ceiling = (uint64_t)255 * CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT
		* HOST_BOARD_LIMIT / 100000000U;
	driver_delay_ms = 0;
	for (unsigned role = 0; role < sizeof(roles) / sizeof(roles[0]); role++) {
		uint32_t expected[3], sum = 0, strongest = 0;
		unsigned weakest = 0, lit_frames = 0, black_frames = 0;
		for (unsigned channel = 0; channel < 3; channel++) {
			expected[channel] = colors[role][channel] * weights[channel];
			sum += expected[channel];
			if (expected[channel] && (!expected[weakest] || expected[channel] < expected[weakest])) {
				weakest = channel;
			}
			if (expected[channel] > strongest) {
				strongest = expected[channel];
			}
		}
		long double normalization = sum > HOST_TOTAL_LIMIT * 10000U
			? (long double)HOST_TOTAL_LIMIT / 10000 / sum : 1.0L / 100000000;
		long double peak_weakest = 255.0L * CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT / 10000
			* HOST_BOARD_LIMIT / 10000 * LED_LEVEL_TASK / 10000 * expected[weakest] * normalization;
		uint32_t previous[3] = {UINT32_MAX, UINT32_MAX, UINT32_MAX};
		for (unsigned envelope = 10000; envelope > 0; envelope--) {
			assert(led_hw_write(roles[role], LED_LEVEL_TASK, envelope, -1, NULL));
			assert((uint64_t)output[0] + output[1] + output[2] <= ceiling);
			for (unsigned channel = 0; channel < 3; channel++) {
				assert(output[channel] <= channel_ceiling && output[channel] <= previous[channel]);
				previous[channel] = output[channel];
				if (!expected[channel]) {
					assert(output[channel] == 0);
				}
			}
			if (expected[weakest] == strongest && ceiling >= 3 && channel_ceiling >= 3) {
				long double difference = output[weakest] - peak_weakest * envelope / 10000;
				assert(difference > -0.504L && difference < 0.504L);
			}
			if (!lit_channels()) {
				struct led_hardware_info info;
				led_hw_info(&info);
				assert(info.powered); /* Quantized black does not cycle the rail. */
				black_frames++;
				continue;
			}
			lit_frames++;
			assert(output[weakest]);
			for (unsigned channel = 0; channel < 3; channel++) {
				if (!expected[channel]) {
					continue;
				}
				assert(output[channel]); /* No partial-color/red-only tail. */
				int64_t error = (int64_t)output[channel] * expected[weakest]
					- (int64_t)output[weakest] * expected[channel];
				assert(error >= -(int64_t)(expected[weakest] / 2U)
					&& error <= (int64_t)(expected[weakest] / 2U));
			}
			if (expected[weakest] != strongest) {
				assert(output[weakest] <= peak_weakest * envelope / 10000 + 0.00001L);
			}
		}
		if (peak_weakest >= 4 && ceiling >= 12 && channel_ceiling >= 4) {
			assert(lit_frames);
		}
		assert(black_frames); /* Sub-representable colors stay black, never boosted. */
		assert(led_hw_write(roles[role], LED_LEVEL_TASK, 0, -1, NULL));
		struct led_hardware_info info;
		led_hw_info(&info);
		assert(!lit_channels() && !gate_on && !info.powered);
		if (smoke_output) {
			printf("hue role=%d weights=%u,%u,%u lit=%u black=%u no_partial_color=1 "
				"shared_hue_error_half_byte=1 monotonic=1 cap=%llu\n",
				roles[role], weights[0], weights[1], weights[2], lit_frames, black_frames,
				(unsigned long long)ceiling);
		}
	}
#endif
}

#if HOST_KIND == 7 && CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT > 0
static long double fade_magnitude(long double value) { return value < 0 ? -value : value; }
static void fade_ideal(enum led_role role, uint16_t level, uint16_t value, long double ideal[3])
{
	static const uint16_t colors[LED_ROLE_COUNT][3] = {
		[LED_ROLE_LINK] = {0, 0, 10000}, [LED_ROLE_POSITIVE] = {0, 10000, 0},
		[LED_ROLE_NEGATIVE] = {10000, 0, 0}, [LED_ROLE_POWER] = {10000, 3500, 0},
		[LED_ROLE_NEUTRAL] = {10000, 10000, 10000}, [LED_ROLE_INTERACTION] = {0, 10000, 10000},
		[LED_ROLE_ACTIVITY] = {5500, 0, 10000}
	};
	static const uint16_t weights[] = {HOST_RED_WEIGHT, HOST_GREEN_WEIGHT, HOST_BLUE_WEIGHT};
	long double sum = 0, largest = 0;
	for (unsigned channel = 0; channel < 3; ++channel) {
		ideal[channel] = (long double)colors[role][channel] * weights[channel] / 100000000;
		sum += ideal[channel];
	}
	long double normalization = sum > HOST_TOTAL_LIMIT / 10000.0L
		? HOST_TOTAL_LIMIT / 10000.0L / sum : 1;
	long double total = 0;
	for (unsigned channel = 0; channel < 3; ++channel) {
		ideal[channel] *= normalization * 255 * CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT / 10000.0L
			* HOST_BOARD_LIMIT / 10000.0L * level / 10000.0L * value / 10000.0L;
		total += ideal[channel];
		if (ideal[channel] > largest) {
			largest = ideal[channel];
		}
	}
	uint32_t maximum = (uint64_t)255 * CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT * HOST_BOARD_LIMIT / 100000000U;
	uint32_t maximum_total = (uint64_t)255 * CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT
		* HOST_BOARD_LIMIT * HOST_TOTAL_LIMIT / 1000000000000ULL;
	long double scale = largest > maximum ? maximum / largest : 1;
	if (total * scale > maximum_total) {
		scale = maximum_total / total;
	}
	for (unsigned channel = 0; channel < 3; ++channel) {
		ideal[channel] *= scale;
	}
}
static void assert_fade_caps(void)
{
	uint32_t maximum = (uint64_t)255 * CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT * HOST_BOARD_LIMIT / 100000000U;
	uint32_t total = (uint64_t)255 * CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT
		* HOST_BOARD_LIMIT * HOST_TOTAL_LIMIT / 1000000000000ULL;
	assert(output[0] + output[1] + output[2] <= total);
	for (unsigned channel = 0; channel < 3; ++channel) {
		assert(output[channel] <= maximum);
	}
}
static void test_pixel_fractional_average(void)
{
	static const uint16_t levels[] = {LED_LEVEL_NOTICE, LED_LEVEL_TASK, LED_LEVEL_BACKGROUND};
	static const uint16_t values[] = {10000, 1900, 100};
	for (enum led_role role = 0; role < LED_ROLE_COUNT; ++role) {
		for (unsigned level = 0; level < sizeof(levels) / sizeof(levels[0]); ++level) {
			for (unsigned value = 0; value < sizeof(values) / sizeof(values[0]); ++value) {
				led_hw_off();
				struct led_fade_sample fade = {.source = role + 1, .origin_ms = 7};
				uint64_t held[3] = {0};
				long double ideal[3];
				fade_ideal(role, levels[level], values[value], ideal);
				for (unsigned sample = 0; sample < 1024; ++sample) {
					fade.slot = sample;
					assert(led_hw_write(role, levels[level], values[value], -1, &fade));
					assert_fade_caps();
					for (unsigned channel = 0; channel < 3; ++channel) {
						held[channel] += output[channel];
						if (!ideal[channel]) {
							assert(!output[channel]);
						}
					}
				}
				for (unsigned channel = 0; channel < 3; ++channel) {
					assert(fade_magnitude(held[channel] - ideal[channel] * 1024) < 2.2L);
				}
			}
		}
	}
}
static void test_pixel_fade_lifecycle(void)
{
	enum { SAMPLES = 128 };
	uint32_t reference[SAMPLES][3];
	struct led_fade_sample fade = {.source = 0x100000000ULL, .origin_ms = 123};
	led_hw_off();
	driver_delay_ms = 0;
	for (unsigned sample = 0; sample < SAMPLES; ++sample) {
		fade.slot = sample;
		assert(led_hw_write(LED_ROLE_POWER, LED_LEVEL_TASK, 331, -1, &fade));
		memcpy(reference[sample], output, sizeof(output));
	}
	for (unsigned mode = 0; mode < 3; ++mode) {
		led_hw_off();
		unsigned rejected = 0;
		for (unsigned sample = 0; sample < SAMPLES; ++sample) {
			/* Large gaps and uint32 slot rollover are equivalent to one
			 * actually rendered sample, never a catch-up burst. */
			fade.slot = mode == 1 ? sample * 37 : mode == 2 ? UINT32_MAX - 10 + sample : sample;
			if (mode == 0 && memcmp(output, reference[sample], sizeof(output)) != 0) {
				uint32_t held[3]; memcpy(held, output, sizeof(held));
				output_error = -EIO;
				assert(!led_hw_write(LED_ROLE_POWER, LED_LEVEL_TASK, 331, -1, &fade));
				assert(memcmp(output, held, sizeof(held)) == 0);
				assert(attempted_pixel.r == reference[sample][0]
					&& attempted_pixel.g == reference[sample][1] && attempted_pixel.b == reference[sample][2]);
				output_error = 0;
				++rejected;
			}
			assert(led_hw_write(LED_ROLE_POWER, LED_LEVEL_TASK, 331, -1, &fade));
			assert(memcmp(output, reference[sample], sizeof(output)) == 0);
			assert_fade_caps();
			unsigned before = writes;
			for (unsigned wake = 0; wake < 8; ++wake) {
				/* A changing same-slot envelope also cannot spend carry. */
				assert(led_hw_write(LED_ROLE_POWER, LED_LEVEL_TASK, 9000, -1, &fade));
				assert(memcmp(output, reference[sample], sizeof(output)) == 0);
			}
			assert(writes == before);
		}
		if (mode == 0 && (uint64_t)255 * CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT
			* HOST_BOARD_LIMIT * HOST_TOTAL_LIMIT / 1000000000000ULL) {
			assert(rejected);
		}
	}
	/* Every replacement begins from the same fresh fractional origin, even
	 * in the same slot. A stateless interruption and zero/off cannot lend debt. */
	for (unsigned reset = 0; reset < 7; ++reset) {
		led_hw_off();
		fade = (struct led_fade_sample){.source = 1, .origin_ms = 1};
		for (unsigned sample = 0; sample < 17; ++sample) {
			fade.slot = sample;
			assert(led_hw_write(LED_ROLE_POWER, LED_LEVEL_TASK, 331, -1, &fade));
		}
		enum led_role role = LED_ROLE_POWER;
		uint16_t level = LED_LEVEL_TASK;
		int color = -1;
		if (reset == 0) {
			fade.source += 1ULL << 48; /* distinct owner, same other identity */
		}
		if (reset == 1) {
			fade.origin_ms++;
		}
		if (reset == 2) {
			role = LED_ROLE_ACTIVITY;
		}
		if (reset == 3) {
			level = LED_LEVEL_NOTICE;
		}
		if (reset == 4) {
			color = LED_PHYSICAL_RED;
		}
		if (reset == 5) {
			assert(led_hw_write(role, level, 331, color, NULL));
		}
		if (reset == 6) {
			assert(led_hw_write(role, level, 0, color, &fade));
		}
		assert(led_hw_write(role, level, 331, color, &fade));
		uint32_t replaced[3]; memcpy(replaced, output, sizeof(replaced));
		led_hw_off();
		assert(led_hw_write(role, level, 331, color, &fade));
		assert(memcmp(replaced, output, sizeof(replaced)) == 0);
		for (unsigned sample = 0; sample < SAMPLES; ++sample) {
			fade.slot++;
			assert(led_hw_write(role, level, 331, color, &fade));
			assert_fade_caps();
		}
	}
}
#endif

static void test_pixel_breathe_carry(void)
{
#if HOST_KIND == 7 && CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT > 0
	test_pixel_fractional_average();
	test_pixel_fade_lifecycle();
#endif
}

#if HOST_KIND == 7 && (CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT == 1000 || CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT == 2000)
static long double absolute_error(long double value) { return value < 0 ? -value : value; }

/* Independent unrounded historical-shape ideal: interpolate the documented
 * normalized anchors rather than using production scaling/curve/rounding.
 * The P10 fixture uses equal weights, RGB ordering and one-channel total cap. */
static long double ideal_byte(enum led_semantic semantic, long double time_ms, unsigned channel)
{
	const struct led_behavior *behavior = led_behavior_get(semantic);
	uint32_t period = behavior->duration_ms;
	long double phase = time_ms - (uint32_t)(time_ms / period) * period;
	long double x = phase <= 2000 ? phase / 2000 : (4000 - phase) / 2000;
	if (x < 0) { x = 0; }
	static const long double positions[] = {0,0.4L,0.6L,0.8L,1};
	static const long double levels[] = {0,0.6L,0.8L,0.95L,1};
	unsigned segment = 0;
	while (segment < 3 && x > positions[segment + 1]) {
		++segment;
	}
	long double envelope = levels[segment] + (x - positions[segment])
		* (levels[segment + 1] - levels[segment]) / (positions[segment + 1] - positions[segment]);
	long double color[3] = {semantic == LED_CHARGING ? 1 : 0.55L,
		semantic == LED_CHARGING ? 0.35L : 0, semantic == LED_CHARGING ? 0 : 1};
	long double sum = color[0] + color[1] + color[2];
	return envelope * CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT / 10000.0L
		* HOST_BOARD_LIMIT / 10000.0L * behavior->level / 10000.0L
		* color[channel] / sum * 255;
}

static void run_descending_tail(enum led_semantic semantic, unsigned wake_mode)
{
	static const unsigned irregular[] = {1, 11, 2, 37, 5, 3, 17, 53, 7};
	const struct led_behavior *behavior = led_behavior_get(semantic);
	uint64_t ceiling = (uint64_t)255 * CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT
		* HOST_BOARD_LIMIT * HOST_TOTAL_LIMIT / 1000000000000ULL;
	led_hw_off();
	host_hw_ms = 0;
	driver_delay_ms = 0;
	struct led_fade_sample fade = {.source = semantic, .origin_ms = 0, .slot = 0};
	assert(led_hw_write(behavior->role, behavior->level, 10000, -1, &fade));
	memset(held_trace, 0, sizeof(held_trace));
	unsigned before = writes, gates_before = gate_off_writes, calls = 0, black_while_powered = 0;
	capture_active = true;
	uint32_t time = 2000;
	while (time < 4000) {
		hw_advance(time);
		struct led_envelope envelope = led_timeline(behavior->style, time, false, false);
		driver_delay_ms = wake_mode == 2 && calls % 7 == 3 && time < 3970 ? 13 : 0;
		fade.slot = time / LED_FRAME_MS;
		assert(led_hw_write(behavior->role, behavior->level, envelope.value_pptt, -1,
			envelope.fading ? &fade : NULL));
		assert(output[0] + output[1] + output[2] <= ceiling);
		if (envelope.value_pptt && !lit_channels()) {
			struct led_hardware_info info;
			led_hw_info(&info);
			assert(info.powered);
#if HOST_GATE
			assert(gate_on);
#endif
			black_while_powered++;
		}
		calls++;
		unsigned increment = wake_mode == 0 ? LED_FRAME_MS : wake_mode == 1 ? 1
			: irregular[(calls - 1) % (sizeof(irregular) / sizeof(irregular[0]))];
		time = host_hw_ms + increment;
	}
	hw_advance(4000);
	driver_delay_ms = 0;
	assert(led_hw_write(behavior->role, behavior->level, 0, -1, NULL));
	hw_advance(4001);
	capture_active = false;
	assert(!lit_channels() && !gate_on);
#if HOST_GATE
	assert(gate_off_writes == gates_before + 1); /* No sub-LSB rail cycling. */
#else
	(void)gates_before;
#endif
	unsigned max_rise = 0, rebrightenings = 0;
	for (unsigned ms = 1; ms <= 2000; ms++) {
		for (unsigned channel = 0; channel < 3; channel++) {
			unsigned previous = held_trace[ms - 1][channel], current = held_trace[ms][channel];
			if (current > previous) {
				rebrightenings++;
				if (current - previous > max_rise) { max_rise = current - previous; }
			}
			assert(current <= (uint64_t)255 * CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT * HOST_BOARD_LIMIT / 100000000U);
		}
		if (smoke_output && ms % 100 == 0) {
			printf("held semantic=%d wake_mode=%u ms=%u rgb=%u,%u,%u\n",
				semantic, wake_mode, 2000 + ms,
				held_trace[ms][0], held_trace[ms][1], held_trace[ms][2]);
		}
	}
	static const unsigned windows[] = {20, 50, 100, 200};
	for (unsigned window_index = 0; window_index < sizeof(windows) / sizeof(windows[0]); ++window_index) {
		unsigned window = windows[window_index];
		for (unsigned begin = 0; begin + window <= 2000; ++begin) {
			for (unsigned channel = 0; channel < 3; ++channel) {
				long double held = 0, ideal = 0;
				for (unsigned offset = 0; offset < window; ++offset) {
					held += held_trace[begin + offset][channel];
					ideal += ideal_byte(semantic, 2000.5L + begin + offset, channel);
				}
				/* Exact-grid/duplicate-wake traces retain sub-byte averaged
				 * color. Missed/13ms-delayed slots intentionally do not accrue
				 * compensation, so their held error includes the maximum lag. */
				long double bound = wake_mode < 2 ? 10.0L / window + 0.12L
					: 1.1L + ideal_byte(semantic, 2000, channel) * 0.00075L * 80;
				assert(absolute_error(held - ideal) / window < bound);
			}
		}
	}
	printf("breathing_held global=%d semantic=%d wake_mode=%u calls=%u writes=%u "
		"instant_rebrightenings=%u max_rise=%u rounded_black_rail_on=%u cap=%llu\n",
		CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT, semantic, wake_mode, calls, writes - before,
		rebrightenings, max_rise, black_while_powered, (unsigned long long)ceiling);
}

static void constant_sub_half_byte(void)
{
	static const unsigned cadence[] = {1, 5, 39, 2, 17, 251, 3};
	led_hw_off();
	host_hw_ms = 0;
	assert(led_hw_write(LED_ROLE_POWER, LED_LEVEL_TASK, 10000, -1, NULL));
	assert(lit_channels());
	/* Changed targets in the same 5ms slot must not be discarded. The ideal
	 * largest channel is 0.11334/0.22667 byte at 10%/20%, strictly below half. */
	assert(led_hw_write(LED_ROLE_POWER, LED_LEVEL_TASK, 100, -1, NULL));
	assert(!lit_channels());
	unsigned before = writes, gates_before = gate_off_writes;
	for (unsigned wake = 0; wake < 4000; wake++) {
		hw_advance(host_hw_ms + cadence[wake % (sizeof(cadence) / sizeof(cadence[0]))]);
		driver_delay_ms = wake % 11 == 4 ? 19 : 0;
		assert(led_hw_write(LED_ROLE_POWER, LED_LEVEL_TASK, 100, -1, NULL));
		assert(!lit_channels()); /* No sparse pulses at any observed hold. */
		struct led_hardware_info info;
		led_hw_info(&info);
		assert(info.powered);
#if HOST_GATE
		assert(gate_on);
#endif
	}
	driver_delay_ms = 0;
	assert(writes == before && gate_off_writes == gates_before);
	printf("constant_sub_half global=%d wakes=4000 pulses=0 writes=0 rail_held=1\n",
		CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT);
}
#endif

static void test_pixel_dark_tail(void)
{
#if HOST_KIND == 7 && (CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT == 1000 || CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT == 2000)
	assert(HOST_BOARD_LIMIT == 10000 && HOST_TOTAL_LIMIT == 10000);
	for (unsigned mode = 0; mode < 3; mode++) {
		run_descending_tail(LED_CHARGING, mode);
		run_descending_tail(LED_PROCESSING, mode);
	}
	constant_sub_half_byte();
#endif
}

static void test_pixel_press_exit(void)
{
#if HOST_KIND == 7 && (CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT == 1000 || CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT == 2000)
	const struct led_behavior *hold = led_behavior_get(LED_BUTTON_HOLD);
	const struct led_behavior *exit = led_behavior_get(LED_MANUAL_EXIT);
	led_hw_off();
	host_hw_ms = 0;
	driver_delay_ms = 0;
	uint32_t peak[3] = {0};
	bool previous_steady_on = false;
	memset(held_trace, 0, sizeof(held_trace));
	capture_origin_ms = 1250;
	capture_active = true;
	struct led_fade_sample fade = {.source = LED_BUTTON_HOLD, .origin_ms = 0};
	for (unsigned elapsed = 0; elapsed <= 3500; elapsed += LED_FRAME_MS) {
		hw_advance(elapsed);
		struct led_envelope envelope = led_timeline_behavior(hold, elapsed, false, false);
		fade.slot = elapsed / LED_FRAME_MS;
		unsigned before = writes;
		assert(led_hw_write(hold->role, hold->level, envelope.value_pptt, -1,
			envelope.fading ? &fade : NULL));
		assert_fade_caps();
		assert(!output[0]); /* Current INTERACTION color remains cyan. */
		if (!elapsed) {
			assert(lit_channels() == 2);
			memcpy(peak, output, sizeof(peak));
		}
		if (!envelope.value_pptt) {
			assert(!lit_channels() && !gate_on);
		}
		if (!envelope.fading && envelope.value_pptt) {
			assert(memcmp(peak, output, sizeof(peak)) == 0);
			if (previous_steady_on) {
				assert(writes == before); /* ON/held blink never dithers. */
			}
		}
		previous_steady_on = !envelope.fading && envelope.value_pptt;
		if (smoke_output && elapsed % 100 == 0) {
			printf("press global=%d ms=%u rgb=%u,%u,%u fading=%u\n",
				CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT, elapsed, output[0], output[1], output[2], envelope.fading);
		}
	}
	capture_active = false;
	static const unsigned windows[] = {20,50,100,200};
	long double ideal_peak = 255.0L * CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT / 10000
		* HOST_BOARD_LIMIT / 10000 * LED_LEVEL_NOTICE / 10000 * 0.5L;
	for (unsigned i = 0; i < sizeof(windows) / sizeof(windows[0]); ++i) {
		unsigned window = windows[i];
		for (unsigned begin = 0; begin + window <= 1000; ++begin) {
			for (unsigned channel = 1; channel < 3; ++channel) {
				long double held = 0, ideal = 0;
				for (unsigned offset = 0; offset < window; ++offset) {
					held += held_trace[begin + offset][channel];
					ideal += ideal_peak * (1 - (begin + offset + 0.5L) / 1000);
				}
				/* The real pixel rail needs its unchanged 2ms settling edge
				 * after the marker. Include that initial black hold explicitly. */
				long double bound = 10.0L / window + 0.12L
					+ (begin < 2 ? ideal_peak * (2 - begin) / window : 0);
				assert(absolute_error(held - ideal) / window < bound);
			}
		}
	}
	capture_origin_ms = 2000;
	/* Standalone manual EXIT retains its own marker and full linear fade;
	 * qualified release fixtures use the threshold origin, never replay it. */
	static const unsigned releases[] = {0, 1000, 1600, 2800};
	for (unsigned i = 0; i < sizeof(releases) / sizeof(releases[0]); ++i) {
		led_hw_off();
		host_hw_ms = 0;
		fade = (struct led_fade_sample){.source = LED_MANUAL_EXIT, .origin_ms = i ? 1000 : 0};
		for (unsigned elapsed = 0; elapsed <= LED_MANUAL_EXIT_MS; elapsed += LED_FRAME_MS) {
			hw_advance(elapsed);
			unsigned visual_age = i ? releases[i] + elapsed - 1000 : elapsed;
			struct led_envelope envelope = led_timeline_behavior(exit, visual_age, false, true);
			fade.slot = elapsed / LED_FRAME_MS;
			assert(led_hw_write(exit->role, exit->level, envelope.value_pptt, -1,
				envelope.fading ? &fade : NULL));
			assert_fade_caps();
			assert(!output[0]);
			if (visual_age < 250 || visual_age >= 1250) {
				assert(!lit_channels() && !gate_on);
			}
		}
		assert(!lit_channels() && !gate_on);
	}
	if (smoke_output) {
		printf("press_exit global=%d immediate_full=1 marker_black_ms=250 linear_down_ms=1000 held_blink_ms=500 inherited_phase=1 peak_rgb=%u,%u,%u\n",
			CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT, peak[0], peak[1], peak[2]);
	}
#endif
}

static void test_initialization_failure(void)
{
#if HOST_KIND != 0
	led_hw_off();
	device_ready = false;
	led_hw_init();
	unsigned previous_writes = writes;
	assert(!led_hw_write(LED_ROLE_NEGATIVE, LED_LEVEL_NOTICE, 10000, -1, NULL));
	assert(writes == previous_writes && !gate_on);
	struct led_hardware_info info;
	led_hw_info(&info);
	assert(info.last_error == -ENODEV);
	led_hw_off();
	assert(writes == previous_writes); /* No unready GPIO/PWM/strip driver entry. */
	led_hw_info(&info);
	assert(info.last_error == -ENODEV);
	device_ready = true;
#endif
}

#if HOST_MISSING_COLOR
static void test_missing_color_direction(void)
{
	static const enum led_role roles[] = {LED_ROLE_INTERACTION, LED_ROLE_ACTIVITY, LED_ROLE_POWER};
	for (size_t role = 0; role < sizeof(roles) / sizeof(roles[0]); role++) {
		struct {
			uint16_t before[3];
			uint16_t direction[3];
			uint16_t after[3];
		} guarded = {.before = {1234, 2345, 3456}, .after = {4567, 5678, 6789}};
		/* Exercise the renderer directly, independently of initialization's
		 * descriptor rejection. Missing colors must never index direction[-1]. */
		color_direction(roles[role], -1, guarded.direction);
		assert(guarded.before[0] == 1234 && guarded.before[1] == 2345 && guarded.before[2] == 3456);
		assert(guarded.after[0] == 4567 && guarded.after[1] == 5678 && guarded.after[2] == 6789);
		for (size_t channel = 0; channel < HOST_CHANNELS; channel++) {
			uint16_t expected = 0;
			unsigned color = channel_colors[channel];
			if (roles[role] == LED_ROLE_POWER) {
				expected = color == 1 ? 10000 : color == 2 ? 3500 : 0;
			} else if (HOST_KIND == 6) {
				expected = color == 3 ? 10000
					: roles[role] == LED_ROLE_INTERACTION && color == 2 ? 10000
					: roles[role] == LED_ROLE_ACTIVITY && color == 1 ? 5500 : 0;
			} else {
				expected = channel == HOST_PRIMARY ? 10000 : 0;
			}
			assert(guarded.direction[channel] == expected);
		}
	}
	led_hw_init();
	unsigned before = writes;
	assert(!led_hw_write(LED_ROLE_POWER, LED_LEVEL_NOTICE, 10000, -1, NULL));
	assert(writes == before && !gate_on);
	struct led_hardware_info info;
	led_hw_info(&info);
	assert(info.last_error == -EINVAL);
}
#endif

int main(int argc, char **argv)
{
#if HOST_MISSING_COLOR
	test_missing_color_direction();
	puts("LED missing-color guarded renderer and descriptor rejection passed");
	return 0;
#endif
	smoke_output = argc == 2 && strcmp(argv[1], "--smoke") == 0;
	if (argc == 2 && strcmp(argv[1], "--fade-regression") == 0) {
		led_hw_init();
		test_pixel_breathe_carry();
		puts("LED pixel fractional-average/caps/successful-slot/reset regressions passed");
		return 0;
	}
	if (argc == 2 && strcmp(argv[1], "--hue-regression") == 0) {
		smoke_output = true;
		led_hw_init();
		test_pixel_hue_quantization();
		puts("LED pixel hue regression passed");
		return 0;
	}
	test_role_mapping_and_black();
	test_raw_capability();
	test_current_caps();
	test_power_role_and_raw_amber();
	test_failure_and_terminal_cutoff();
	test_failed_pixel_frame_retry();
	test_interaction_activity_fallback();
	test_pixel_hue_quantization();
	test_pixel_breathe_carry();
	test_pixel_dark_tail();
	test_pixel_press_exit();
	led_hw_off();
	test_initialization_failure();
	printf("LED hardware kind=%d gpio=%d global=%d mapping/caps/cache/cutoff/dark-tail scenarios passed\n",
		HOST_KIND, !!HOST_GPIO_TRANSPORT, CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT);
	return 0;
}
