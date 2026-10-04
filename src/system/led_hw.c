#include "globals.h"
#include "led_internal.h"
#include "led_strip_fade.h"

#include <errno.h>
#include <string.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>

/* Board descriptors deliberately do not inspect led0/1/2 aliases. Colors are
 * source/electrical declarations, not optical measurements: 0 unknown,
 * 1 red, 2 green, 3 blue, 4 amber, 5 white. A missing descriptor is NO_LED. */
#define USER_NODE DT_PATH(zephyr_user)
#define HW_KIND_no_led 0
#define HW_KIND_mono_gpio 1
#define HW_KIND_mono_pwm 2
#define HW_KIND_dual_discrete 3
#define HW_KIND_rg_mix 4
#define HW_KIND_tri_discrete 5
#define HW_KIND_rgb_pwm 6
#define HW_KIND_rgb_pixel 7
BUILD_ASSERT(
	HW_KIND_no_led == LED_CAP_NO_LED && HW_KIND_mono_gpio == LED_CAP_MONO_GPIO
		&& HW_KIND_mono_pwm == LED_CAP_MONO_PWM && HW_KIND_dual_discrete == LED_CAP_DUAL_DISCRETE
		&& HW_KIND_rg_mix == LED_CAP_RG_MIX && HW_KIND_tri_discrete == LED_CAP_TRI_DISCRETE
		&& HW_KIND_rgb_pwm == LED_CAP_RGB_PWM && HW_KIND_rgb_pixel == LED_CAP_RGB_PIXEL,
	"LED hardware kinds must match capability values"
);
#if DT_NODE_HAS_PROP(USER_NODE, led_capability)
#define HW_KIND UTIL_CAT(HW_KIND_, DT_STRING_TOKEN(USER_NODE, led_capability))
#else
#define HW_KIND HW_KIND_no_led
#endif
#define HW_PIXEL (HW_KIND == HW_KIND_rgb_pixel)
/* zephyr,user pwms may belong to the heater on a pixel board. Capability, not
 * property presence, decides whether those PWM channels belong to this owner. */
#define HW_PWM (!HW_PIXEL && HW_KIND != HW_KIND_no_led && DT_NODE_HAS_PROP(USER_NODE, pwms))
#define HW_GPIO (!HW_PIXEL && HW_KIND != HW_KIND_no_led && DT_NODE_HAS_PROP(USER_NODE, led_channel_gpios))
#define HW_GATE DT_NODE_HAS_PROP(USER_NODE, led_en_gpios)
#define HW_MIX (HW_KIND == HW_KIND_rg_mix || HW_KIND == HW_KIND_rgb_pwm || HW_PIXEL)
#define HW_RGB (HW_KIND == HW_KIND_rgb_pwm || HW_PIXEL)

#if HW_KIND != HW_KIND_no_led
#if HW_KIND == HW_KIND_mono_gpio || HW_KIND == HW_KIND_mono_pwm
/* Unbound zephyr,user infers a singleton <number> as a scalar, not an array.
 * The explicit MONO capability supplies its channel count; never inspect aliases. */
#define HW_CHANNELS 1
static const uint16_t channel_colors[] = {DT_PROP(USER_NODE, led_channel_colors)};
static const uint16_t channel_weights[] = {DT_PROP(USER_NODE, led_channel_weights)};
#else
#define HW_CHANNELS DT_PROP_LEN(USER_NODE, led_channel_colors)
static const uint16_t channel_colors[] = DT_PROP(USER_NODE, led_channel_colors);
static const uint16_t channel_weights[] = DT_PROP(USER_NODE, led_channel_weights);
#endif
#define HW_PRIMARY DT_PROP_OR(USER_NODE, led_primary_channel, 0)
#define HW_BOARD_LIMIT DT_PROP_OR(USER_NODE, led_brightness_limit_pptt, 10000)
#define HW_TOTAL_LIMIT DT_PROP_OR(USER_NODE, led_total_limit_pptt, 10000)
BUILD_ASSERT(HW_CHANNELS >= 1 && HW_CHANNELS <= 3, "LED channel count must be explicit and bounded");
BUILD_ASSERT(HW_PRIMARY < HW_CHANNELS, "LED primary channel is outside descriptor");
BUILD_ASSERT(HW_BOARD_LIMIT > 0 && HW_BOARD_LIMIT <= 10000, "Invalid LED board limit");
BUILD_ASSERT(HW_TOTAL_LIMIT > 0 && HW_TOTAL_LIMIT <= 10000, "Invalid LED aggregate duty limit");
BUILD_ASSERT(sizeof(channel_weights) / sizeof(channel_weights[0]) == HW_CHANNELS, "Missing channel weights");
BUILD_ASSERT(HW_PIXEL || (HW_PWM != HW_GPIO), "Declare one explicit LED output transport");
BUILD_ASSERT(
	(HW_KIND != HW_KIND_mono_gpio || (HW_GPIO && HW_CHANNELS == 1))
		&& (HW_KIND != HW_KIND_mono_pwm || (HW_PWM && HW_CHANNELS == 1)),
	"Mono capability transport mismatch"
);
BUILD_ASSERT(
	(HW_KIND != HW_KIND_dual_discrete && HW_KIND != HW_KIND_rg_mix) || HW_CHANNELS == 2,
	"Dual capability needs two channels"
);
BUILD_ASSERT(
	(HW_KIND != HW_KIND_tri_discrete && HW_KIND != HW_KIND_rgb_pwm && !HW_PIXEL) || HW_CHANNELS == 3,
	"Three-color capability needs three channels"
);
BUILD_ASSERT(!HW_MIX || !HW_GPIO, "Binary GPIO outputs cannot implement dimmed mixed colors");
#if HW_PWM
#define PWM_CHANNEL(node, prop, index) PWM_DT_SPEC_GET_BY_IDX(node, index),
static const struct pwm_dt_spec channels[] = {DT_FOREACH_PROP_ELEM(USER_NODE, pwms, PWM_CHANNEL)};
BUILD_ASSERT(DT_PROP_LEN(USER_NODE, pwms) == HW_CHANNELS, "Missing PWM channels");
#elif HW_GPIO
#define GPIO_CHANNEL(node, prop, index) GPIO_DT_SPEC_GET_BY_IDX(node, prop, index),
static const struct gpio_dt_spec channels[] = {DT_FOREACH_PROP_ELEM(USER_NODE, led_channel_gpios, GPIO_CHANNEL)};
BUILD_ASSERT(DT_PROP_LEN(USER_NODE, led_channel_gpios) == HW_CHANNELS, "Missing GPIO channels");
#elif HW_PIXEL
#include <zephyr/drivers/led_strip.h>
static const struct device *const pixel = DEVICE_DT_GET(DT_PHANDLE(USER_NODE, led_pixel));
BUILD_ASSERT(DT_PROP(DT_PHANDLE(USER_NODE, led_pixel), chain_length) == 1, "Only a single status pixel is supported");
#endif
#if HW_GATE
static const struct gpio_dt_spec rail = GPIO_DT_SPEC_GET(USER_NODE, led_en_gpios);
#endif
#else
#define HW_CHANNELS 0
#define HW_BOARD_LIMIT 0
#endif

static struct led_hardware_info hardware = {
	.capability = HW_KIND,
	.global_limit_pptt = CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT,
	.board_limit_pptt = HW_BOARD_LIMIT,
	/* 0 explicitly means unmeasured: no supported optical minimum is claimed. */
	.visible_min_pptt = DT_PROP_OR(USER_NODE, led_visible_min_pptt, 0),
	.gpio = HW_GPIO,
};
/* Worker state is never exposed concurrently; only these two mutable fields
 * are published under a short lock after a bounded hardware operation. */
static struct k_spinlock hardware_info_lock;
static bool reported_powered;
static int reported_error;

static void publish_hardware(void)
{
	k_spinlock_key_t key = k_spin_lock(&hardware_info_lock);
	reported_powered = hardware.powered;
	reported_error = hardware.last_error;
	k_spin_unlock(&hardware_info_lock, key);
}

#if HW_KIND != HW_KIND_no_led
static bool initialized;
static int initialization_error;
static bool frame_valid;
static uint32_t previous_frame[3];
#if HW_PIXEL
static struct {
	struct led_fade_sample sample;
	struct led_strip_fade carry;
	uint16_t level;
	uint8_t role;
	int8_t color;
	bool active;
	bool committed;
} pixel_fade;
#endif

static void remember_error(int error)
{
	if (error && !hardware.last_error) {
		hardware.last_error = error;
	}
}

#if HW_PWM || HW_PIXEL
static int device_power(const struct device *device, enum pm_device_action action)
{
	if (!device_is_ready(device)) {
		return -ENODEV;
	}
	int error = pm_device_action_run(device, action);
	/* Drivers without PM remain usable; their output is still explicitly black. */
	return error == -ENOSYS || error == -ENOTSUP || error == -EALREADY ? 0 : error;
}
#endif

static int output_frame(const uint32_t frame[3], bool force)
{
	if (!force && frame_valid && memcmp(frame, previous_frame, sizeof(previous_frame)) == 0) {
		return 0;
	}
	int first_error = 0;
#if HW_PIXEL
	struct led_rgb rgb = {0};
	for (size_t index = 0; index < HW_CHANNELS; index++) {
		switch (channel_colors[index]) {
		case 1:
			rgb.r = frame[index];
			break;
		case 2:
			rgb.g = frame[index];
			break;
		case 3:
			rgb.b = frame[index];
			break;
		default:
			return -EINVAL;
		}
	}
	/* The single owner cannot contend on ws2812-i2s's mutex. Its TX slab
	 * waits/retries are bounded by the one-pixel frame time in the pinned SDK.
	 * GRB/RGB serialization is the driver's explicit DTS color-mapping. */
	first_error = led_strip_update_rgb(pixel, &rgb, 1);
#else
	for (size_t index = 0; index < HW_CHANNELS; index++) {
#if HW_PWM
		int error = pwm_set_pulse_dt(&channels[index], frame[index]);
#else
		int error = gpio_pin_set_dt(&channels[index], frame[index]);
#endif
		if (error && !first_error) {
			first_error = error;
		}
	}
#endif
	if (!first_error) {
		memcpy(previous_frame, frame, sizeof(previous_frame));
		frame_valid = true;
	} else {
		/* A partly applied multi-channel frame must never be cache-hit success. */
		frame_valid = false;
	}
	return first_error;
}

static bool power_on(void)
{
	if (hardware.powered) {
		return true;
	}
#if HW_GATE
	int error = gpio_pin_set_dt(&rail, 1);
	remember_error(error);
	if (error) {
		return false;
	}
#if HW_PIXEL
	k_msleep(2); /* Only the actual rail off -> on edge; within admission bound. */
#endif
#endif
#if HW_PIXEL
	remember_error(device_power(pixel, PM_DEVICE_ACTION_RESUME));
#elif HW_PWM
	for (size_t index = 0; index < HW_CHANNELS; index++) {
		if (!index || channels[index].dev != channels[index - 1].dev) {
			remember_error(device_power(channels[index].dev, PM_DEVICE_ACTION_RESUME));
		}
	}
#endif
	hardware.powered = true;
	frame_valid = false;
	return hardware.last_error == 0;
}

static int color_channel(unsigned color)
{
	for (size_t index = 0; index < HW_CHANNELS; index++) {
		if (channel_colors[index] == color) {
			return index;
		}
	}
	return -1;
}

static void color_direction(enum led_role role, int override, uint16_t direction[3])
{
	unsigned color;
	if (override < 0 && (role == LED_ROLE_INTERACTION || role == LED_ROLE_ACTIVITY)) {
		if (HW_RGB) {
			int blue = color_channel(3);
			if (blue >= 0) {
				direction[blue] = 10000;
			}
			if (role == LED_ROLE_INTERACTION) {
				int green = color_channel(2);
				if (green >= 0) {
					direction[green] = 10000;
				}
			} else {
				int red = color_channel(1);
				if (red >= 0) {
					direction[red] = 5500;
				}
			}
			return;
		}
		/* Do not invent cyan/violet with alternating or RG discrete lamps. */
		role = LED_ROLE_NEUTRAL;
	}
	if (override >= 0) {
		color = (unsigned) override + 1;
	} else {
		switch (role) {
		case LED_ROLE_LINK:
			color = 3;
			break;
		case LED_ROLE_POSITIVE:
			color = 2;
			break;
		case LED_ROLE_NEGATIVE:
			color = 1;
			break;
		case LED_ROLE_POWER:
			color = 4;
			break;
		case LED_ROLE_NEUTRAL:
			color = 5;
			break;
		default:
			return;
		}
	}
	int channel = color_channel(color);
	if (channel >= 0) {
		direction[channel] = 10000;
		return;
	}
	if (HW_MIX && color == 4) {
		int red = color_channel(1);
		int green = color_channel(2);
		if (red >= 0) {
			direction[red] = 10000;
		}
		if (green >= 0) {
			direction[green] = 3500;
		}
		return;
	}
	if (HW_RGB && color == 5) {
		for (size_t index = 0; index < HW_CHANNELS; index++) {
			direction[index] = 10000;
		}
		return;
	}
	if (override < 0) {
		if (role == LED_ROLE_NEGATIVE) {
			channel = color_channel(4);
		} else if (role == LED_ROLE_POWER) {
			channel = color_channel(1);
		}
		direction[channel >= 0 ? (size_t)channel : HW_PRIMARY] = 10000;
	}
}

#if HW_PIXEL
static void pixel_frame(
	uint64_t brightness_q32, const uint32_t direction_q24[3], const uint32_t weighted[3], uint32_t frame[3]
)
{
	size_t weakest = 0;
	uint32_t strongest = 0;
	for (size_t index = 0; index < HW_CHANNELS; index++) {
		if (weighted[index] && (!weighted[weakest] || weighted[index] < weighted[weakest])) {
			weakest = index;
		}
		if (weighted[index] > strongest) {
			strongest = weighted[index];
		}
	}
	if (!strongest) {
		return;
	}
	/* Unequal mixed colors share one integer scale: independently rounding
	 * channels loses the weakest first (amber becomes red). Round the other
	 * components from the weakest whole byte, never lift a sub-byte color.
	 * Equal-component and single-channel colors retain nearest-byte rounding.
	 * This accepts coarser, darker fade steps instead of hue drift or dither. */
	uint32_t unit = weighted[weakest] == strongest
		? led_strip_byte(brightness_q32, direction_q24[weakest])
		: brightness_q32 * direction_q24[weakest] * 255U >> 56;
	uint32_t maximum = (uint64_t)CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT * HW_BOARD_LIMIT * 255U / 100000000U;
	uint32_t maximum_total = (uint64_t)CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT * HW_BOARD_LIMIT
		* HW_TOTAL_LIMIT * 255U / 1000000000000ULL;
	for (;;) {
		uint32_t total = 0;
		bool fits = true;
		for (size_t index = 0; index < HW_CHANNELS; index++) {
			frame[index] = weighted[index] == weighted[weakest] ? unit
				: weighted[index] && unit
					? ((uint64_t)unit * weighted[index] + weighted[weakest] / 2U) / weighted[weakest] : 0;
			total += frame[index];
			fits &= frame[index] <= maximum;
		}
		if (fits && total <= maximum_total) {
			return;
		}
		/* Rounding can cross a hard cap by a byte. Lower the shared scale,
		 * never trim one component and change its hue. Zero always fits. */
		unit--;
	}
}
#endif
#endif

bool led_hw_color_supported(enum led_physical_color color)
{
#if HW_KIND == HW_KIND_no_led
	(void)color;
	return false;
#else
	if ((unsigned)color > LED_PHYSICAL_WHITE) {
		return false;
	}
	unsigned physical = (unsigned)color + 1;
	return color_channel(physical) >= 0 || (HW_MIX && physical == 4) || (HW_RGB && physical == 5);
#endif
}

void led_hw_info(struct led_hardware_info *info)
{
	info->capability = hardware.capability;
	info->global_limit_pptt = hardware.global_limit_pptt;
	info->board_limit_pptt = hardware.board_limit_pptt;
	info->visible_min_pptt = hardware.visible_min_pptt;
	info->gpio = hardware.gpio;
	k_spinlock_key_t key = k_spin_lock(&hardware_info_lock);
	info->powered = reported_powered;
	info->last_error = reported_error;
	k_spin_unlock(&hardware_info_lock, key);
}

void led_hw_init(void)
{
#if HW_PIXEL
	pixel_fade.active = false;
#endif
#if HW_KIND != HW_KIND_no_led
	hardware.last_error = 0;
#if HW_GATE
	remember_error(gpio_is_ready_dt(&rail) ? gpio_pin_configure_dt(&rail, GPIO_OUTPUT_INACTIVE) : -ENODEV);
#endif
	for (size_t index = 0; index < HW_CHANNELS; index++) {
		if (channel_colors[index] > 5 || channel_weights[index] > 10000 || !channel_weights[index]) {
			remember_error(-EINVAL);
		}
#if HW_GPIO
		remember_error(
			gpio_is_ready_dt(&channels[index]) ? gpio_pin_configure_dt(&channels[index], GPIO_OUTPUT_INACTIVE) : -ENODEV
		);
#elif HW_PWM
		remember_error(pwm_is_ready_dt(&channels[index]) ? 0 : -ENODEV);
#endif
	}
#if HW_PIXEL
	remember_error(device_is_ready(pixel) ? 0 : -ENODEV);
#endif
	if (HW_MIX && (color_channel(1) < 0 || color_channel(2) < 0 || (HW_RGB && color_channel(3) < 0))) {
		remember_error(-EINVAL);
	}
	initialized = hardware.last_error == 0;
	initialization_error = hardware.last_error;
	/* Never call an uninitialized driver's API. Gated pixels already start
	 * unpowered; ungated pixels must explicitly clear any retained latch. */
#if !HW_PIXEL || !HW_GATE
	if (initialized) {
		uint32_t black[3] = {0};
		remember_error(output_frame(black, true));
	}
#endif
#endif /* Hardware capability present. */
	publish_hardware();
}

void led_hw_off(void)
{
#if HW_PIXEL
	pixel_fade.active = false;
#endif
#if HW_KIND != HW_KIND_no_led
	hardware.last_error = initialized ? 0 : initialization_error;
	uint32_t black[3] = {0};
	if (initialized && (hardware.powered || !frame_valid)) {
		remember_error(output_frame(black, false));
	}
	/* Black failure never bypasses the independent rail cutoff. No retry loop. */
#if HW_GATE
	int rail_error = gpio_is_ready_dt(&rail) ? gpio_pin_set_dt(&rail, 0) : -ENODEV;
	remember_error(rail_error);
#endif
#if HW_PIXEL
	remember_error(device_power(pixel, PM_DEVICE_ACTION_SUSPEND));
#elif HW_PWM
	for (size_t index = 0; index < HW_CHANNELS; index++) {
		if (!index || channels[index].dev != channels[index - 1].dev) {
			remember_error(device_power(channels[index].dev, PM_DEVICE_ACTION_SUSPEND));
		}
	}
#endif
#if HW_GATE
	/* On a failed gate write retain the last known rail state, not a fictitious
	 * successful cutoff. The terminal owner proceeds regardless of this error. */
	hardware.powered = rail_error && hardware.powered;
#else
	hardware.powered = false;
#endif
#endif
	publish_hardware();
}

#if HW_KIND == HW_KIND_no_led
bool led_hw_write(
	enum led_role role __attribute__((unused)), uint16_t level_pptt __attribute__((unused)),
	uint16_t value_pptt __attribute__((unused)), int color_override __attribute__((unused)),
	const struct led_fade_sample *fade __attribute__((unused))
)
{
	publish_hardware();
	return true; /* Local completion, not a visual-output claim. */
}
#else
bool led_hw_write(
	enum led_role role, uint16_t level_pptt, uint16_t value_pptt, int color_override,
	const struct led_fade_sample *fade
)
{
	if (!initialized) {
		publish_hardware();
		return false;
	}
	if ((unsigned)role >= LED_ROLE_COUNT || level_pptt > 10000 || value_pptt > 10000 || color_override < -1
		|| (color_override >= 0 && !led_hw_color_supported((enum led_physical_color)color_override))) {
		hardware.last_error = -EINVAL;
		publish_hardware();
		return false;
	}
#if HW_PIXEL
	if (!fade || !level_pptt || !value_pptt) {
		pixel_fade.active = false;
	} else if (!pixel_fade.active || pixel_fade.sample.source != fade->source
		|| pixel_fade.sample.origin_ms != fade->origin_ms || pixel_fade.role != role
		|| pixel_fade.level != level_pptt || pixel_fade.color != color_override) {
		pixel_fade.sample.source = fade->source;
		pixel_fade.sample.origin_ms = fade->origin_ms;
		pixel_fade.carry = (struct led_strip_fade){0};
		pixel_fade.role = role;
		pixel_fade.level = level_pptt;
		pixel_fade.color = color_override;
		pixel_fade.active = true;
		pixel_fade.committed = false;
	}
#else
	(void)fade;
#endif
	if (!CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT || !level_pptt || !value_pptt) {
		if (hardware.powered || !frame_valid) {
			led_hw_off();
		}
		publish_hardware();
		return hardware.last_error == 0;
	}
	hardware.last_error = 0;
#if HW_PIXEL
	if (pixel_fade.active && pixel_fade.committed && pixel_fade.sample.slot == fade->slot
		&& hardware.powered && frame_valid) {
		publish_hardware();
		return true;
	}
#endif
	if (!power_on()) {
		int cause = hardware.last_error;
		led_hw_off();
		hardware.last_error = cause;
		publish_hardware();
		return false;
	}
	uint16_t direction[3] = {0};
	color_direction(role, color_override, direction);
#if HW_PWM || HW_PIXEL
	uint32_t weighted[3] = {0};
	uint32_t direction_q24[3] = {0};
	uint32_t sum = 0;
	for (size_t index = 0; index < HW_CHANNELS; index++) {
		weighted[index] = (uint32_t)direction[index] * channel_weights[index];
		sum += weighted[index];
	}
	for (size_t index = 0; index < HW_CHANNELS; index++) {
		/* Preserve weighted/aggregate color fractions rather than rounding the
		 * normalized direction back to whole parts per ten-thousand. */
		direction_q24[index] = sum > HW_TOTAL_LIMIT * 10000U
			? (uint64_t)weighted[index] * HW_TOTAL_LIMIT * LED_DIRECTION_ONE / ((uint64_t)sum * 10000U)
			: (uint64_t)weighted[index] * LED_DIRECTION_ONE / 100000000U;
	}
	uint64_t brightness_q32 = led_brightness_q32(
		CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT, HW_BOARD_LIMIT, level_pptt, value_pptt
	);
#endif /* Binary GPIO needs neither duty scaling nor brightness computation. */
	uint32_t frame[3] = {0};
#if HW_PIXEL
	struct led_strip_fade next_fade;
	if (fade) {
		uint32_t maximum = (uint64_t)CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT * HW_BOARD_LIMIT * 255U / 100000000U;
		uint32_t maximum_total = (uint64_t)CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT * HW_BOARD_LIMIT
			* HW_TOTAL_LIMIT * 255U / 1000000000000ULL;
		uint32_t target[3];
		led_strip_fade_targets(brightness_q32, direction_q24, maximum, maximum_total, target);
		led_strip_fade_frame(&pixel_fade.carry, &next_fade, target, maximum, maximum_total, frame);
	} else {
		pixel_frame(brightness_q32, direction_q24, weighted, frame);
	}
#else
	for (size_t index = 0; index < HW_CHANNELS; index++) {
#if HW_PWM
		uint64_t duty_q32 = brightness_q32 * direction_q24[index] >> 24;
		frame[index] = (uint64_t)channels[index].period * duty_q32 >> 32;
#else
		/* Global zero was handled above. No ramp threshold and no fake dimming. */
		frame[index] = direction[index] != 0;
#endif
	}
#endif
	int error = output_frame(frame, false);
	hardware.last_error = error;
#if HW_PIXEL
	if (!error && fade) {
		pixel_fade.carry = next_fade;
		pixel_fade.sample.slot = fade->slot;
		pixel_fade.committed = true;
	}
#endif
	publish_hardware();
	return error == 0;
}
#endif
