#ifndef LED_HW_HOST_GLOBALS_H
#define LED_HW_HOST_GLOBALS_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <errno.h>

#ifndef HOST_KIND
#define HOST_KIND 6
#endif
#ifndef HOST_KIND_TOKEN
#define HOST_KIND_TOKEN rgb_pwm
#endif
#ifndef CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT
#define CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT 10000
#endif
#ifndef HOST_GATE
#define HOST_GATE 1
#endif
#ifdef HOST_BOARD_LIMIT
#define HAS_led_brightness_limit_pptt 1
#define OR_led_brightness_limit_pptt(fallback) HOST_BOARD_LIMIT
#define VALUE_led_brightness_limit_pptt HOST_BOARD_LIMIT
#else
#define HAS_led_brightness_limit_pptt 0
#define OR_led_brightness_limit_pptt(fallback) fallback
#define HOST_BOARD_LIMIT 10000
#endif
#ifdef HOST_TOTAL_LIMIT
#define HAS_led_total_limit_pptt 1
#define OR_led_total_limit_pptt(fallback) HOST_TOTAL_LIMIT
#define VALUE_led_total_limit_pptt HOST_TOTAL_LIMIT
#else
#define HAS_led_total_limit_pptt 0
#define OR_led_total_limit_pptt(fallback) fallback
#define HOST_TOTAL_LIMIT 10000
#endif
#ifdef HOST_PRIMARY
#define HAS_led_primary_channel 1
#define OR_led_primary_channel(fallback) HOST_PRIMARY
#define VALUE_led_primary_channel HOST_PRIMARY
#else
#define HAS_led_primary_channel 0
#define OR_led_primary_channel(fallback) fallback
#define HOST_PRIMARY 0
#endif
#define OR_led_visible_min_pptt(fallback) fallback
#ifndef HOST_MISSING_COLOR
#define HOST_MISSING_COLOR 0
#endif
#ifndef HOST_RED_WEIGHT
#define HOST_RED_WEIGHT 10000
#endif
#ifndef HOST_GREEN_WEIGHT
#define HOST_GREEN_WEIGHT 10000
#endif
#ifndef HOST_BLUE_WEIGHT
#define HOST_BLUE_WEIGHT 10000
#endif
#ifndef HOST_POLARITY
#define HOST_POLARITY 0
#endif
#ifndef HOST_GPIO_TRANSPORT
#define HOST_GPIO_TRANSPORT (HOST_KIND == 1)
#endif
#if HOST_KIND == 0
#define HOST_CHANNELS 0
#elif HOST_KIND == 1 || HOST_KIND == 2
#define HOST_CHANNELS 1
#elif HOST_KIND == 3 || HOST_KIND == 4
#define HOST_CHANNELS 2
#else
#define HOST_CHANNELS 3
#endif
#define HOST_PWM_TRANSPORT (HOST_KIND != 0 && HOST_KIND != 7 && !HOST_GPIO_TRANSPORT)
#define CAT_INNER(a, b) a##b
#define CAT(a, b) CAT_INNER(a, b)
#define UTIL_CAT(a, b) CAT(a, b)
#define BUILD_ASSERT(test, message) _Static_assert(test, message)
#define DT_PATH(name) name
#define DT_NODE_HAS_PROP(node, prop) CAT(HAS_, prop)
#define HAS_led_capability 1
#define HAS_pwms (HOST_PWM_TRANSPORT || HOST_KIND == 7) /* Pixel board's unrelated heater PWM. */
#define HAS_led_channel_gpios HOST_GPIO_TRANSPORT
#define HAS_led_en_gpios HOST_GATE
#define DT_STRING_TOKEN(node, prop) HOST_KIND_TOKEN
#define DT_PROP(node, prop) CAT(VALUE_, prop)
#define DT_PROP_OR(node, prop, fallback) CAT(OR_, prop)(fallback)
#define VALUE_chain_length 1
#if HOST_CHANNELS <= 1
#define VALUE_led_channel_colors 0
#define VALUE_led_channel_weights 10000
#elif HOST_CHANNELS == 2
#ifdef HOST_UNKNOWN_COLORS
#define VALUE_led_channel_colors {0, 0}
#else
#define VALUE_led_channel_colors {HOST_MISSING_COLOR == 1 ? 0 : 1, HOST_MISSING_COLOR == 2 ? 0 : 2}
#endif
#define VALUE_led_channel_weights {10000, 10000}
#else
#ifdef HOST_RBG
#define VALUE_led_channel_colors {1, 3, 2}
#define VALUE_led_channel_weights {HOST_RED_WEIGHT, HOST_BLUE_WEIGHT, HOST_GREEN_WEIGHT}
#elif defined(HOST_UNKNOWN_COLORS)
#define VALUE_led_channel_colors {0, 0, 0}
#define VALUE_led_channel_weights {HOST_RED_WEIGHT, HOST_GREEN_WEIGHT, HOST_BLUE_WEIGHT}
#else
#define VALUE_led_channel_colors {HOST_MISSING_COLOR == 1 ? 0 : 1, HOST_MISSING_COLOR == 2 ? 0 : 2, HOST_MISSING_COLOR == 3 ? 0 : 3}
#define VALUE_led_channel_weights {HOST_RED_WEIGHT, HOST_GREEN_WEIGHT, HOST_BLUE_WEIGHT}
#endif
#endif
#define DT_PROP_LEN(node, prop) CAT(LENGTH_, prop)
#if HOST_CHANNELS > 1
#define LENGTH_led_channel_colors HOST_CHANNELS
#define LENGTH_led_channel_weights HOST_CHANNELS
#endif /* Real unbound singleton numeric properties have no generated LEN. */
#define LENGTH_led_channel_gpios HOST_CHANNELS
#define LENGTH_pwms (HOST_KIND == 7 ? 1 : HOST_CHANNELS)
#define DT_PHANDLE(node, prop) pixel
#if HOST_CHANNELS <= 1
#define DT_FOREACH_PROP_ELEM(node, prop, function) function(node, prop, 0)
#elif HOST_CHANNELS == 2
#define DT_FOREACH_PROP_ELEM(node, prop, function) function(node, prop, 0) function(node, prop, 1)
#else
#define DT_FOREACH_PROP_ELEM(node, prop, function) function(node, prop, 0) function(node, prop, 1) function(node, prop, 2)
#endif
struct device { int id; };
extern const struct device host_device;
#define DEVICE_DT_GET(node) (&host_device)
struct pwm_dt_spec { const struct device *dev; uint32_t channel; uint32_t period; uint32_t flags; };
#define PWM_DT_SPEC_GET_BY_IDX(node, index) { .dev = &host_device, .channel = index, .period = 1000000, .flags = HOST_POLARITY }
struct gpio_dt_spec { const struct device *port; uint32_t pin; uint32_t dt_flags; };
#define GPIO_DT_SPEC_GET(node, prop) { .port = &host_device, .pin = 8, .dt_flags = HOST_POLARITY }
#define GPIO_DT_SPEC_GET_BY_IDX(node, prop, index) { .port = &host_device, .pin = 10 + index, .dt_flags = HOST_POLARITY }
#define GPIO_OUTPUT_INACTIVE 0
struct led_rgb { uint8_t r, g, b; };
enum pm_device_action { PM_DEVICE_ACTION_SUSPEND, PM_DEVICE_ACTION_RESUME };
struct k_spinlock { int unused; };
typedef int k_spinlock_key_t;
static inline k_spinlock_key_t k_spin_lock(struct k_spinlock *lock) { (void)lock; return 0; }
static inline void k_spin_unlock(struct k_spinlock *lock, k_spinlock_key_t key) { (void)lock; (void)key; }
bool device_is_ready(const struct device *device);
bool gpio_is_ready_dt(const struct gpio_dt_spec *gpio);
bool pwm_is_ready_dt(const struct pwm_dt_spec *pwm);
int gpio_pin_configure_dt(const struct gpio_dt_spec *gpio, int flags);
int gpio_pin_set_dt(const struct gpio_dt_spec *gpio, int value);
int pwm_set_pulse_dt(const struct pwm_dt_spec *pwm, uint32_t pulse);
int pm_device_action_run(const struct device *device, enum pm_device_action action);
int led_strip_update_rgb(const struct device *device, struct led_rgb *rgb, size_t count);
int32_t k_msleep(int32_t duration_ms);
int64_t k_uptime_get(void);
#endif
