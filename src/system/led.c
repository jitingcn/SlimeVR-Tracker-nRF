#include "globals.h"

#include <math.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>

#include "led.h"
#include "led_strip_fade.h"
#include "led_sync.h"
#if CONFIG_LED_NETWORK_SYNC
#include "connection/esb.h"
#endif

LOG_MODULE_REGISTER(led, LOG_LEVEL_INF);

static void led_thread(void);
K_THREAD_DEFINE(led_thread_id, CONFIG_LED_THREAD_STACK_SIZE, led_thread, NULL, NULL, NULL, LED_THREAD_PRIORITY, 0, 0);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, led_en_gpios)
#define LED_EN_EXISTS true
static const struct gpio_dt_spec led_en = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, led_en_gpios);
#endif

#if CONFIG_LED_STRIP
#define LED_STRIP_EXISTS true
#include <zephyr/drivers/led_strip.h>
#define STRIP_NODE DT_ALIAS(led_strip)
static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);
static struct led_strip_fade led_fade;
#endif

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, led_gpios)
#define LED_EXISTS true
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, led_gpios);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led0))
#ifndef LED_EXISTS
#define LED_EXISTS true
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#else
#define LED0_EXISTS true
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#endif
#endif
#ifndef LED_EXISTS
#ifndef LED_STRIP_EXISTS
#warning "LED GPIO does not exist"
// static const struct gpio_dt_spec led = {0};
#endif
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led1))
#define LED1_EXISTS true
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led2))
#define LED2_EXISTS true
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led3))
#define LED3_EXISTS true
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);
#endif

#if DT_NODE_EXISTS(DT_ALIAS(pwm_led0))
#define PWM_LED_EXISTS true
static const struct pwm_dt_spec pwm_led = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));
#else
#ifndef LED_STRIP_EXISTS
#warning "PWM LED node does not exist"
#endif
#endif
#if DT_NODE_EXISTS(DT_ALIAS(pwm_led1))
#define PWM_LED1_EXISTS true
static const struct pwm_dt_spec pwm_led1 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led1));
#endif
#if DT_NODE_EXISTS(DT_ALIAS(pwm_led2))
#define PWM_LED2_EXISTS true
static const struct pwm_dt_spec pwm_led2 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led2));
#endif

#if LED_EXISTS || LED_STRIP_EXISTS || PWM_LED_EXISTS
static struct k_spinlock led_request_lock;
K_SEM_DEFINE(led_changed, 0, 1);
K_SEM_DEFINE(led_quiesced, 0, 1);
K_MUTEX_DEFINE(led_shutdown_lock);
static bool shutdown_pending;
static enum sys_led_pattern led_patterns[SYS_LED_PATTERN_DEPTH]
	= {[0 ...(SYS_LED_PATTERN_DEPTH - 1)] = SYS_LED_PATTERN_OFF};
static uint32_t led_generations[SYS_LED_PATTERN_DEPTH];

static int led_pin_init(void)
{
	LOG_DBG("led_pin_init");
#ifdef LED_STRIP_EXISTS
	led_strip_fade_reset(&led_fade);
#endif
#if LED_EXISTS
	gpio_pin_configure_dt(&led, GPIO_OUTPUT);
	gpio_pin_set_dt(&led, 0);
#endif
#if LED0_EXISTS
	gpio_pin_configure_dt(&led0, GPIO_OUTPUT);
	gpio_pin_set_dt(&led0, 0);
#endif
#if LED1_EXISTS
	gpio_pin_configure_dt(&led1, GPIO_OUTPUT);
	gpio_pin_set_dt(&led1, 0);
#endif
#if LED2_EXISTS
	gpio_pin_configure_dt(&led2, GPIO_OUTPUT);
	gpio_pin_set_dt(&led2, 0);
#endif
#if LED3_EXISTS
	gpio_pin_configure_dt(&led3, GPIO_OUTPUT);
	gpio_pin_set_dt(&led3, 0);
#endif
	return 0;
}

SYS_INIT(led_pin_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

static void led_pin_reset(void)
{
	LOG_DBG("led_pin_reset");
#if LED_EXISTS
	gpio_pin_configure_dt(&led, GPIO_DISCONNECTED);
#endif
#if LED0_EXISTS
	gpio_pin_configure_dt(&led0, GPIO_DISCONNECTED);
#endif
#if LED1_EXISTS
	gpio_pin_configure_dt(&led1, GPIO_DISCONNECTED);
#endif
#if LED2_EXISTS
	gpio_pin_configure_dt(&led2, GPIO_DISCONNECTED);
#endif
#if LED3_EXISTS
	gpio_pin_configure_dt(&led3, GPIO_DISCONNECTED);
#endif
}

static void led_suspend(void)
{
	LOG_DBG("led_suspend");
#ifdef LED_STRIP_EXISTS
	pm_device_action_run(strip, PM_DEVICE_ACTION_SUSPEND);
#endif
#ifdef PWM_LED_EXISTS
	pm_device_action_run(pwm_led.dev, PM_DEVICE_ACTION_SUSPEND);
#endif
#ifdef PWM_LED1_EXISTS
	pm_device_action_run(pwm_led1.dev, PM_DEVICE_ACTION_SUSPEND);
#endif
#ifdef PWM_LED2_EXISTS
	pm_device_action_run(pwm_led2.dev, PM_DEVICE_ACTION_SUSPEND);
#endif
	led_pin_reset();
	// disable power
#if LED_EN_EXISTS
	gpio_pin_configure_dt(&led_en, GPIO_OUTPUT);
	gpio_pin_set_dt(&led_en, 0);
#endif
}

static void led_resume(void)
{
	LOG_DBG("led_resume");
	// enable power
#if LED_EN_EXISTS
	gpio_pin_configure_dt(&led_en, GPIO_OUTPUT);
	gpio_pin_set_dt(&led_en, 1);
#endif
#ifdef LED_STRIP_EXISTS
	pm_device_action_run(strip, PM_DEVICE_ACTION_RESUME);
#endif
#ifdef PWM_LED_EXISTS
	pm_device_action_run(pwm_led.dev, PM_DEVICE_ACTION_RESUME);
#endif
#ifdef PWM_LED1_EXISTS
	pm_device_action_run(pwm_led1.dev, PM_DEVICE_ACTION_RESUME);
#endif
#ifdef PWM_LED2_EXISTS
	pm_device_action_run(pwm_led2.dev, PM_DEVICE_ACTION_RESUME);
#endif
	led_pin_init();
}

#ifdef LED_STRIP_EXISTS
#define LED_RGB_COLOR
#else
#ifdef CONFIG_LED_RGB_COLOR
#define LED_RGB_COLOR
#define LED_RG_COLOR
#endif

#if PWM_LED_EXISTS && PWM_LED1_EXISTS && PWM_LED2_EXISTS
#define LED_TRI_COLOR
#else
#undef LED_RGB_COLOR
#undef LED_TRI_COLOR
#if PWM_LED_EXISTS && PWM_LED1_EXISTS
#define LED_DUAL_COLOR
#else
#undef LED_RG_COLOR
#undef LED_DUAL_COLOR
#endif
#endif
#endif

#ifdef LED_RGB_COLOR
static int led_pwm_period[5][3] = {
	{CONFIG_LED_DEFAULT_COLOR_R, CONFIG_LED_DEFAULT_COLOR_G, CONFIG_LED_DEFAULT_COLOR_B}, // Default
	{0, 10000, 0},                                                                        // Success
	{10000, 0, 0},                                                                        // Error
	{8000, 2000, 0},                                                                      // Charging
	{0, 0, 10000},                                                                        // Pairing
};
#elif defined(LED_TRI_COLOR)
static int led_pwm_period[5][3] = {
	{0, 0, 10000},   // Default
	{0, 10000, 0},   // Success
	{10000, 0, 0},   // Error
	{6000, 4000, 0}, // Charging
	{0, 0, 10000},   // Pairing
};
#elif defined(LED_RG_COLOR)
static int led_pwm_period[5][2] = {
	{CONFIG_LED_DEFAULT_COLOR_R, CONFIG_LED_DEFAULT_COLOR_G}, // Default
	{0, 10000},                                               // Success
	{10000, 0},                                               // Error
	{8000, 2000},                                             // Charging
	{4000, 6000},                                             // Pairing
};
#elif defined(LED_DUAL_COLOR)
static int led_pwm_period[5][2] = {
	{0, 10000},   // Default
	{0, 10000},   // Success
	{10000, 0},   // Error
	{6000, 4000}, // Charging
	{0, 10000},   // Pairing
};
#else
static int led_pwm_period[5][1] = {
	{10000}, // Default
	{10000}, // Success
	{10000}, // Error
	{10000}, // Charging
	{10000}, // Pairing
};
#endif

// Using brightness and value if PWM is supported, otherwise value is coerced to on/off
// TODO: use computed constants for high/low brightness and color values
#if CONFIG_LED_STRIP_TIMING_LOG
/*
 * Diagnostic for the fade smoothness: the sub-level carry relies on the fade
 * patterns refreshing at their nominal rate, so report the actual frame period
 * (between two strip updates) and how long the update itself blocked, once per
 * 1000 frames - about every 5 s while a pattern is fading.
 */
static void led_strip_timing_note(uint32_t start_ticks)
{
	static uint32_t frames;
	static uint32_t previous_start;
	static uint32_t period_min_us = UINT32_MAX;
	static uint32_t period_max_us;
	static uint64_t period_sum_us;
	static uint32_t update_min_us = UINT32_MAX;
	static uint32_t update_max_us;
	static uint64_t update_sum_us;
	static uint32_t late_frames;

	uint32_t now = k_uptime_ticks();
	uint32_t update_us = k_ticks_to_us_floor32(now - start_ticks);

	if (frames > 0) {
		uint32_t period_us = k_ticks_to_us_floor32(start_ticks - previous_start);

		if (period_us < period_min_us) {
			period_min_us = period_us;
		}
		if (period_us > period_max_us) {
			period_max_us = period_us;
		}
		period_sum_us += period_us;
		if (period_us > 20000) {
			late_frames++;
		}
	}
	if (update_us < update_min_us) {
		update_min_us = update_us;
	}
	if (update_us > update_max_us) {
		update_max_us = update_us;
	}
	update_sum_us += update_us;
	previous_start = start_ticks;
	frames++;

	if (frames >= 1000) {
		LOG_INF("strip frames %u: period %u/%u/%u us (min/avg/max), late(>20ms) %u; "
			"update blocked %u/%u/%u us",
			frames, period_min_us, (uint32_t)(period_sum_us / (frames - 1)),
			period_max_us, late_frames, update_min_us,
			(uint32_t)(update_sum_us / frames), update_max_us);
		frames = 0;
		period_min_us = UINT32_MAX;
		period_max_us = 0;
		period_sum_us = 0;
		update_min_us = UINT32_MAX;
		update_max_us = 0;
		update_sum_us = 0;
		late_frames = 0;
	}
}
#endif


static void led_pin_set(enum sys_led_color color, int brightness_pptt, int value_pptt)
{
	LOG_DBG("led_pin_set: color %d, brightness %d, value %d", color, brightness_pptt, value_pptt);
	if (brightness_pptt < 0) {
		brightness_pptt = 0;
	} else if (brightness_pptt > 10000) {
		brightness_pptt = 10000;
	}
	if (value_pptt < 0) {
		value_pptt = 0;
	} else if (value_pptt > 10000) {
		value_pptt = 10000;
	}
#if LED_STRIP_EXISTS
	static struct led_rgb pixel[1];
	value_pptt = value_pptt * brightness_pptt / 10000;
	value_pptt = value_pptt * CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT / 10000;
	pixel[0].r = led_strip_fade_next(&led_fade, led_pwm_period[color][0], value_pptt, 0);
	pixel[0].g = led_strip_fade_next(&led_fade, led_pwm_period[color][1], value_pptt, 1);
	pixel[0].b = led_strip_fade_next(&led_fade, led_pwm_period[color][2], value_pptt, 2);
#if CONFIG_LED_STRIP_TIMING_LOG
	uint32_t led_frame_start = k_uptime_ticks();
#endif
	led_strip_update_rgb(strip, pixel, 1);
#if CONFIG_LED_STRIP_TIMING_LOG
	led_strip_timing_note(led_frame_start);
#endif
#elif PWM_LED_EXISTS
	value_pptt = value_pptt * brightness_pptt / 10000;
	value_pptt = value_pptt * CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT / 10000;
	// only supporting color if PWM is supported
	pwm_set_pulse_dt(&pwm_led, pwm_led.period / 10000 * (led_pwm_period[color][0] * value_pptt / 10000));
#if PWM_LED1_EXISTS
	pwm_set_pulse_dt(&pwm_led1, pwm_led1.period / 10000 * (led_pwm_period[color][1] * value_pptt / 10000));
#if PWM_LED2_EXISTS
	pwm_set_pulse_dt(&pwm_led2, pwm_led2.period / 10000 * (led_pwm_period[color][2] * value_pptt / 10000));
#endif
#endif
#else
	gpio_pin_set_dt(&led, value_pptt > 5000);
#endif
}
#endif

/* Only the worker owns driver calls, including power and strip transfers. */
void set_led(enum sys_led_pattern pattern, int priority)
{
#if LED_EXISTS || LED_STRIP_EXISTS || PWM_LED_EXISTS
	if (priority < 0 || priority >= SYS_LED_PATTERN_DEPTH) {
		return;
	}
	k_spinlock_key_t key = k_spin_lock(&led_request_lock);
	if (led_patterns[priority] != pattern) {
		led_patterns[priority] = pattern;
		led_generations[priority]++;
	}
	k_spin_unlock(&led_request_lock, key);
	k_sem_give(&led_changed);
#else
	(void)pattern;
	(void)priority;
#endif
}

void led_shutdown(void)
{
#if LED_EXISTS || LED_STRIP_EXISTS || PWM_LED_EXISTS
	/* Shutdown callers may overlap; this operation cannot be superseded by
	 * an ordinary request while the driver finishes its current transfer. */
	k_mutex_lock(&led_shutdown_lock, K_FOREVER);
	k_sem_reset(&led_quiesced);
	k_spinlock_key_t key = k_spin_lock(&led_request_lock);
	shutdown_pending = true;
	k_spin_unlock(&led_request_lock, key);
	k_sem_give(&led_changed);
	k_sem_take(&led_quiesced, K_FOREVER);
	k_mutex_unlock(&led_shutdown_lock);
#endif
}

#if LED_EXISTS || LED_STRIP_EXISTS || PWM_LED_EXISTS
struct led_request {
	enum sys_led_pattern pattern;
	int owner;
	uint32_t generation;
};

static struct led_request led_request_snapshot(void)
{
	struct led_request request = {.pattern = SYS_LED_PATTERN_OFF, .owner = -1};
	k_spinlock_key_t key = k_spin_lock(&led_request_lock);
	for (int i = 0; i < SYS_LED_PATTERN_DEPTH; i++) {
		if (led_patterns[i] != SYS_LED_PATTERN_OFF) {
			request.pattern = led_patterns[i];
			request.owner = i;
			request.generation = led_generations[i];
			break;
		}
	}
	k_spin_unlock(&led_request_lock, key);
	return request;
}

static bool led_complete(struct led_request request, enum sys_led_pattern result)
{
	k_spinlock_key_t key = k_spin_lock(&led_request_lock);
	int winner = -1;
	for (int i = 0; i < SYS_LED_PATTERN_DEPTH; i++) {
		if (led_patterns[i] != SYS_LED_PATTERN_OFF) {
			winner = i;
			break;
		}
	}
	bool matches = request.owner >= 0 && winner == request.owner
		&& led_patterns[request.owner] == request.pattern
		&& led_generations[request.owner] == request.generation;
	if (matches) {
		led_patterns[request.owner] = result;
		led_generations[request.owner]++;
	}
	k_spin_unlock(&led_request_lock, key);
	k_sem_give(&led_changed);
	return matches;
}

static uint32_t led_local_ticks(void)
{
	/* nRF54 kernel ticks are 31250 Hz; LED phase is always 32768 Hz. */
	uint64_t ticks = k_uptime_ticks();
	return (uint32_t)((ticks / CONFIG_SYS_CLOCK_TICKS_PER_SEC) * LED_SYNC_HZ
		+ (ticks % CONFIG_SYS_CLOCK_TICKS_PER_SEC) * LED_SYNC_HZ
			/ CONFIG_SYS_CLOCK_TICKS_PER_SEC);
}
#endif

static void led_thread(void)
{
#if !LED_EXISTS && !LED_STRIP_EXISTS && !PWM_LED_EXISTS
	LOG_WRN("LED GPIO does not exist");
	return;
#else
	enum sys_led_pattern current = SYS_LED_PATTERN_OFF;
	int owner = -1;
	uint32_t generation = 0;
	int state = 0;
	bool powered = false;
	int last_value = -1;
	int last_brightness = -1;
	enum sys_led_color last_color = SYS_LED_COLOR_DEFAULT;
	int64_t due = 0;
	uint32_t local_origin = 0;
	struct led_sync_start active = {0};
#if CONFIG_LED_NETWORK_SYNC
	uint32_t held_offset = 0;
	bool have_offset = false;
#endif
	for (;;) {
		k_spinlock_key_t key = k_spin_lock(&led_request_lock);
		bool shutting_down = shutdown_pending;
		k_spin_unlock(&led_request_lock, key);
		if (shutting_down) {
			if (powered) {
				led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, 0);
			}
			led_suspend();
			powered = false;
			last_value = -1;
			key = k_spin_lock(&led_request_lock);
			led_patterns[SYS_LED_PRIORITY_HIGHEST] = SYS_LED_PATTERN_OFF_FORCE;
			led_generations[SYS_LED_PRIORITY_HIGHEST]++;
			shutdown_pending = false;
			k_spin_unlock(&led_request_lock, key);
			k_sem_give(&led_quiesced);
		}
		struct led_request request = led_request_snapshot();
		int64_t now = k_uptime_ticks();
		if (request.pattern != current ||
		    (request.owner == owner && request.generation != generation)) {
			current = request.pattern;
			state = 0;
			due = now;
			local_origin = led_local_ticks();
			active = (struct led_sync_start){.entered = local_origin};
			/* A new effect starts with a fresh first frame, not the previous
			 * effect's fractional strip brightness. Keep hardware powered. */
#if LED_STRIP_EXISTS
			led_strip_fade_reset(&led_fade);
#endif
			last_value = -1;
		}
		owner = request.owner;
		generation = request.generation;
		if (current <= SYS_LED_PATTERN_OFF) {
			if (powered) {
				led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, 0);
				led_suspend();
				powered = false;
			}
			k_sem_take(&led_changed, K_FOREVER);
			continue;
		}
		if (!powered) {
			led_resume();
			powered = true;
			last_value = -1;
		}
		if (now < due) {
			k_sem_take(&led_changed, due == INT64_MAX ? K_FOREVER : K_TICKS(due - now));
			continue;
		}
		uint32_t wait_us = 0;
		bool forever = false;
		int brightness = 10000;
		int value = 0;
		enum sys_led_color color = SYS_LED_COLOR_DEFAULT;
		enum sys_led_pattern completed = SYS_LED_PATTERN_OFF;
		bool complete = false;
		switch (current) {
		case SYS_LED_PATTERN_ON:
		case SYS_LED_PATTERN_ON_PERSIST:
			color = current == SYS_LED_PATTERN_ON ? SYS_LED_COLOR_DEFAULT : SYS_LED_COLOR_SUCCESS;
			brightness = current == SYS_LED_PATTERN_ON ? 10000 : 2000;
			value = 10000;
			forever = true;
			break;
		case SYS_LED_PATTERN_SHORT:
		case SYS_LED_PATTERN_LONG:
		case SYS_LED_PATTERN_FLASH:
		case SYS_LED_PATTERN_DFU:
			state = (state + 1) % 2;
			value = state * 10000;
			color = current == SYS_LED_PATTERN_SHORT ? SYS_LED_COLOR_PAIRING :
				current == SYS_LED_PATTERN_DFU ? SYS_LED_COLOR_CHARGING : SYS_LED_COLOR_DEFAULT;
			wait_us = current == SYS_LED_PATTERN_SHORT ? (state ? 100000 : 900000) :
				current == SYS_LED_PATTERN_LONG ? 500000 :
				current == SYS_LED_PATTERN_FLASH ? 200000 : 100000;
			break;
		case SYS_LED_PATTERN_ONESHOT_POWERON:
		case SYS_LED_PATTERN_ONESHOT_PROGRESS:
		case SYS_LED_PATTERN_ONESHOT_COMPLETE:
		case SYS_LED_PATTERN_ONESHOT_PING:
			state++;
			color = current == SYS_LED_PATTERN_ONESHOT_PROGRESS ||
				current == SYS_LED_PATTERN_ONESHOT_COMPLETE ? SYS_LED_COLOR_SUCCESS : SYS_LED_COLOR_DEFAULT;
			value = (current == SYS_LED_PATTERN_ONESHOT_PING ? state % 2 : !(state % 2)) * 10000;
			complete = state == (current == SYS_LED_PATTERN_ONESHOT_POWERON ? 7 :
				current == SYS_LED_PATTERN_ONESHOT_PROGRESS ? 5 :
				current == SYS_LED_PATTERN_ONESHOT_COMPLETE ? 9 : 20);
			wait_us = 200000;
			break;
		case SYS_LED_PATTERN_ONESHOT_POWEROFF:
			state++;
			brightness = state == 1 ? 10000 : (202 - state) * 50;
			value = state == 1 || state == 202 ? 0 : 10000;
			wait_us = state == 1 ? 250000 : 5000;
			complete = state == 202;
			completed = SYS_LED_PATTERN_OFF_FORCE;
			break;
		case SYS_LED_PATTERN_LONG_PERSIST:
		case SYS_LED_PATTERN_PULSE_PERSIST:
		case SYS_LED_PATTERN_ACTIVE_PERSIST: {
			uint32_t local = led_local_ticks();
			uint32_t clock = local - local_origin;
#if CONFIG_LED_NETWORK_SYNC
			uint32_t network;
			if (esb_get_status_clock(&local, &network)) {
				if (!have_offset && !active.started) {
					active.joined = false;
				}
				held_offset = network - local;
				have_offset = true;
			}
			clock = have_offset ? local + held_offset : local - local_origin;
#endif
			uint32_t distance;
			if (current == SYS_LED_PATTERN_PULSE_PERSIST) {
				color = SYS_LED_COLOR_CHARGING;
				value = led_sync_pulse(led_sync_phase(clock, 5U * LED_SYNC_HZ));
				distance = 164U;
			} else if (current == SYS_LED_PATTERN_LONG_PERSIST) {
				color = SYS_LED_COLOR_CHARGING;
				brightness = 2000;
				uint32_t phase = led_sync_phase(clock, LED_SYNC_HZ);
				value = phase < LED_SYNC_HZ / 2U ? 10000 : 0;
				distance = LED_SYNC_HZ / 2U - phase % (LED_SYNC_HZ / 2U);
			} else {
				uint32_t phase = led_sync_phase(clock, LED_SYNC_ACTIVE_PERIOD);
				value = led_sync_active(&active, local, phase) ? 10000 : 0;
				distance = phase < LED_SYNC_ACTIVE_OFF ?
					LED_SYNC_ACTIVE_OFF - phase : LED_SYNC_ACTIVE_PERIOD - phase;
				if (!active.eligible) {
					uint32_t hold = LED_SYNC_ACTIVE_OFF - (local - active.entered);
					if (hold < distance) {
						distance = hold;
					}
				}
			}
			distance = led_sync_until(clock, distance);
#if CONFIG_LED_NETWORK_SYNC
			if (distance > LED_SYNC_HZ / 4U) {
				distance = LED_SYNC_HZ / 4U;
			}
#endif
			wait_us = (uint32_t)(((uint64_t)distance * 1000000U + LED_SYNC_HZ - 1U) / LED_SYNC_HZ);
			break;
		}
		case SYS_LED_PATTERN_ERROR_A:
		case SYS_LED_PATTERN_ERROR_B:
		case SYS_LED_PATTERN_ERROR_C:
		case SYS_LED_PATTERN_ERROR_D:
			color = SYS_LED_COLOR_ERROR;
			state = (state + 1) % (current == SYS_LED_PATTERN_ERROR_D ? 2 : 10);
			value = (state % 2 && (current == SYS_LED_PATTERN_ERROR_D ||
				state < 4 + 2 * ((int)current - SYS_LED_PATTERN_ERROR_A))) * 10000;
			wait_us = 500000;
			break;
		default:
			forever = true;
			break;
		}
		/* Fade frames retain sub-level dither; stable outputs need no
		 * transfer merely to check clock freshness. */
		if (value != last_value || brightness != last_brightness || color != last_color
		    || current == SYS_LED_PATTERN_PULSE_PERSIST
		    || current == SYS_LED_PATTERN_ONESHOT_POWEROFF) {
			led_pin_set(color, brightness, value);
			last_value = value;
			last_brightness = brightness;
			last_color = color;
		}
		if (complete) {
			if (!led_complete(request, completed)) {
				/* A new winning owner may inherit this final frame. A
				 * replacement in the same slot instead restarts above. */
				state--;
			} else {
				/* A lower-priority one-shot may now become visible. */
				state = 0;
			}
			due = k_uptime_ticks();
		} else if (forever) {
			due = INT64_MAX;
			k_sem_take(&led_changed, K_FOREVER);
		} else {
			due = now + k_us_to_ticks_ceil64(wait_us);
		}
	}
#endif
}
