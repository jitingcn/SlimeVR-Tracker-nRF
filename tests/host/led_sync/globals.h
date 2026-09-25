#ifndef LED_HOST_GLOBALS_H
#define LED_HOST_GLOBALS_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <limits.h>
#define CONFIG_LED_THREAD_STACK_SIZE 512
#ifndef CONFIG_SYS_CLOCK_TICKS_PER_SEC
#define CONFIG_SYS_CLOCK_TICKS_PER_SEC 32768
#endif
#ifndef CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT
#define CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT 10000
#endif
#ifndef CONFIG_LED_DEFAULT_COLOR_R
#define CONFIG_LED_DEFAULT_COLOR_R 4000
#endif
#ifndef CONFIG_LED_DEFAULT_COLOR_G
#define CONFIG_LED_DEFAULT_COLOR_G 6000
#endif
#ifndef CONFIG_LED_DEFAULT_COLOR_B
#define CONFIG_LED_DEFAULT_COLOR_B 0
#endif
#define CONFIG_APPLICATION_INIT_PRIORITY 0
#define CONFIG_LED_STRIP_TIMING_LOG 0
#define CONFIG_LED_RGB_COLOR 0
#define LED_THREAD_PRIORITY 0
#define LOG_MODULE_REGISTER(...)
#define LOG_DBG(...)
#define LOG_WRN(...)
#define LOG_INF(...)
#define LOG_LEVEL_INF 0
#define DT_PATH(x) x
#define DT_ALIAS(x) x
#define HOST_CAT_INNER(a,b) a##b
#define HOST_CAT(a,b) HOST_CAT_INNER(a,b)
#define DT_NODE_HAS_PROP(n,p) HOST_CAT(DT_PROP_, p)
#define DT_PROP_led_en_gpios 1
#ifndef HOST_LED
#define HOST_LED 1
#endif
#define DT_PROP_led_gpios HOST_LED
#ifndef HOST_PWM
#define HOST_PWM 1
#endif
#define DT_NODE_EXISTS(n) HOST_CAT(DT_EXISTS_, n)
#define DT_EXISTS_led0 0
#define DT_EXISTS_led1 0
#define DT_EXISTS_led2 0
#define DT_EXISTS_led3 0
#define DT_EXISTS_pwm_led1 0
#define DT_EXISTS_pwm_led0 HOST_PWM
#define DT_EXISTS_pwm_led2 0
#define GPIO_DT_SPEC_GET(n,p) {.pin = HOST_CAT(DT_PIN_, p)}
#define DT_PIN_led_gpios 1
#define DT_PIN_led_en_gpios 2
#define PWM_DT_SPEC_GET(n) {.period = 10000}
#define DEVICE_DT_GET(n) NULL
#define SYS_INIT(...)
#define APPLICATION 0
#define GPIO_OUTPUT 1
#define GPIO_DISCONNECTED 0
#define PM_DEVICE_ACTION_SUSPEND 0
#define PM_DEVICE_ACTION_RESUME 1
struct device { int unused; };
struct gpio_dt_spec { int pin; };
struct pwm_dt_spec { const struct device *dev; uint32_t period; };
struct led_rgb { uint8_t r, g, b; };
struct k_spinlock { int unused; };
typedef int k_spinlock_key_t;
typedef int64_t k_timeout_t;
struct k_sem { unsigned count; };
#define K_SEM_DEFINE(n,initial,max) struct k_sem n = {initial}
#define K_THREAD_DEFINE(...)
#define K_FOREVER INT64_MAX
#define K_TICKS(n) (n)
static inline k_spinlock_key_t k_spin_lock(struct k_spinlock *s) {(void)s; return 0;}
static inline void k_spin_unlock(struct k_spinlock *s, k_spinlock_key_t k) {(void)s; (void)k;}
static inline void k_sem_give(struct k_sem *s) {s->count = 1;}
static inline uint64_t k_us_to_ticks_ceil64(uint64_t us) {return (us * CONFIG_SYS_CLOCK_TICKS_PER_SEC + 999999) / 1000000;}
int64_t k_uptime_ticks(void);
struct k_mutex { int unused; };
#define K_MUTEX_DEFINE(n) struct k_mutex n
static inline void k_mutex_lock(struct k_mutex *m, k_timeout_t t) {(void)m; (void)t;}
static inline void k_mutex_unlock(struct k_mutex *m) {(void)m;}
static inline void k_sem_reset(struct k_sem *s) {s->count = 0;}
int k_sem_take(struct k_sem *, k_timeout_t);
int gpio_pin_configure_dt(const struct gpio_dt_spec *, int);
int gpio_pin_set_dt(const struct gpio_dt_spec *, int);
int pwm_set_pulse_dt(const struct pwm_dt_spec *, uint32_t);
int pm_device_action_run(const struct device *, int);
int led_strip_update_rgb(const struct device *, struct led_rgb *, size_t);
#endif
