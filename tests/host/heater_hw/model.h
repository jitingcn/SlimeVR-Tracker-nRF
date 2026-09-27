#ifndef HEATER_REGISTER_MODEL_H
#define HEATER_REGISTER_MODEL_H
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "heater.h"

#define CONFIG_SYSTEM_IMU_HEATER 1
#define CONFIG_ESB_SYS_TIMER_INSTANCE 2
#define CONFIG_SYSTEM_IMU_HEATER_MAX_DUTY_PPTT 7000
#define CONFIG_SENSOR_TCAL_HEATED_MAX_TEMP_C 50
#define CONFIG_PM 0
#define CONFIG_PM_DEVICE_RUNTIME 0
#define IS_ENABLED(x) (x)
#define DT_PATH(x) 0
#define DT_NODELABEL(x) 0
#define DT_NODE_HAS_PROP(n, p) 1
#define DT_PHANDLE(n, p) 0
#define DT_SAME_NODE(a, b) 1
#define DT_NODE_HAS_STATUS(n, s) 1
#define DT_PWMS_CTLR(n) 0
#define DT_PWMS_CHANNEL(n) 0
#define DT_PWMS_PERIOD(n) HEATER_HW_PERIOD_NS
#define DT_PWMS_FLAGS(n) 0
#define DT_GPIO_PIN(n, p) 31
#define DT_GPIO_CTLR(n, p) 0
#define DT_GPIO_FLAGS(n, p) 0
#define PWM_POLARITY_NORMAL 0
#define BUILD_ASSERT(condition, ...) _Static_assert(condition, #condition)
#define SYS_INIT(fn, level, priority)
#define K_MUTEX_DEFINE(name) struct k_mutex name
#define K_MSEC(ms) (ms)
#define NRF_PWM_CHANNEL_COUNT 4
#define NRF_PWM_PIN_NOT_CONNECTED UINT32_MAX
#define NRF_PWM_TASK_STOP 0
#define NRF_PWM_EVENT_STOPPED 0
#define NRF_TIMER_EVENT_COMPARE0 0
#define NRF_TIMER_EVENT_COMPARE1 1
#define NRF_TIMER_CC_CHANNEL0 0
#define NRF_TIMER_CC_CHANNEL1 1
#define NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK 1
#define NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK 2
#define NRF_TIMER_BIT_WIDTH_32 32
#define NRFX_TIMER_DEFAULT_CONFIG(hz) ((nrfx_timer_config_t){.frequency = (hz)})
#define NRFX_TIMER_INSTANCE(reg) {0}
#define NRF_TIMER4 (&timer_registers)
#define NRF_PWM1 (&pwm_registers)
#define PWM_DT_SPEC_GET(node) { .dev = &pwm_registers }

typedef int atomic_t;
static int atomic_get(atomic_t *v) { return *v; }
static bool atomic_cas(atomic_t *v, int old, int value)
{
	if (*v != old) { return false; }
	*v = value;
	return true;
}
struct k_mutex { bool held; };
static int k_mutex_lock(struct k_mutex *m, int timeout)
{
	(void)timeout;
	if (m->held) { return -EBUSY; }
	m->held = true;
	return 0;
}
static void k_mutex_unlock(struct k_mutex *m) { assert(m->held); m->held = false; }
struct pwm_dt_spec { void *dev; };
typedef struct { int unused; } nrfx_timer_t;
typedef struct { unsigned bit_width, frequency; } nrfx_timer_config_t;
typedef unsigned nrfx_gppi_handle_t;
enum pm_device_state { PM_DEVICE_STATE_ACTIVE, PM_DEVICE_STATE_SUSPENDED };
static struct {
	uint32_t counter, cc[2], shorts;
	bool event[2], running;
} timer_registers;
static struct {
	bool enabled, running, stopped_event, stop_requested, driver_running;
	int64_t stop_at_us;
	uint32_t pulse_ns;
} pwm_registers;
static struct { bool allocated, enabled; unsigned event; } ppi[2];
static int64_t clock_us;
static bool gpio_high, irq_masked, clock_failed, ready = true, pm_busy;
static int init_error, alloc_fail_at, allocations, set_error, pm_error;
static unsigned extra_pin = UINT32_MAX;
static enum pm_device_state power_state;
static void (*before_pwm_start)(void), (*after_pwm_start)(void);
static void (*before_cc_write)(void), (*after_timer_clear)(void), (*after_timer_enable)(void);

static bool pin_heating(void) { return pwm_registers.running || gpio_high; }
static void stop_task(void)
{
	if (clock_failed || !pwm_registers.running) { return; }
	if (pwm_registers.stop_at_us < 0) {
		pwm_registers.stop_at_us = (clock_us / 20000 + 1) * 20000;
	}
}
static void advance_us(int64_t delta)
{
	for (int64_t i = 0; i < delta; ++i) {
		++clock_us;
		if (timer_registers.running) {
			++timer_registers.counter;
			bool clear = false;
			for (unsigned event = 0; event < 2; ++event) {
				if (timer_registers.counter == timer_registers.cc[event]) {
					timer_registers.event[event] = true;
					for (unsigned channel = 0; channel < 2; ++channel) {
						if (ppi[channel].allocated && ppi[channel].enabled &&
						    ppi[channel].event == event) { stop_task(); }
					}
					clear |= (timer_registers.shorts & (1U << event)) != 0;
				}
			}
			if (clear) { timer_registers.counter = 0; }
		}
		if (pwm_registers.stop_at_us >= 0 && clock_us >= pwm_registers.stop_at_us) {
			pwm_registers.running = false;
			pwm_registers.stopped_event = true;
			pwm_registers.stop_at_us = -1;
		}
		if (pwm_registers.stopped_event && !irq_masked) {
			pwm_registers.enabled = false;
			pwm_registers.driver_running = false;
			pwm_registers.stopped_event = false;
		}
	}
}
static void advance_ms(int64_t delta) { advance_us(delta * 1000); }
static int64_t k_uptime_get(void) { return clock_us / 1000; }
static void k_busy_wait(unsigned us) { advance_us(us); }
static void nrf_gpio_pin_clear(unsigned pin) { assert(pin == 31); gpio_high = false; }
static void nrf_gpio_cfg_output(unsigned pin) { assert(pin == 31); }
static void nrf_pwm_task_trigger(void *reg, unsigned task)
{
	assert(reg == NRF_PWM1 && task == NRF_PWM_TASK_STOP);
	stop_task();
}
static void nrf_pwm_disable(void *reg)
{
	assert(reg == NRF_PWM1);
	pwm_registers.enabled = pwm_registers.running = false;
	pwm_registers.stop_at_us = -1;
}
static bool nrf_pwm_enable_check(void *reg) { assert(reg == NRF_PWM1); return pwm_registers.enabled; }
static unsigned nrf_pwm_pin_get(void *reg, unsigned channel)
{
	assert(reg == NRF_PWM1);
	return channel == 0 ? 31 : extra_pin;
}
static bool pwm_is_ready_dt(const struct pwm_dt_spec *spec) { (void)spec; return ready; }
static int pwm_set_dt(const struct pwm_dt_spec *spec, unsigned period, unsigned pulse)
{
	(void)spec;
	assert(period == HEATER_HW_PERIOD_NS);
	assert(pulse < period); /* Any forbidden constant-HIGH write fails the scenario. */
	if (set_error) { return set_error; }
	if (!pulse) {
		gpio_high = false;
		pwm_registers.stop_requested = true;
		stop_task();
		return 0;
	}
	/* Actual nrfx driver waits for its IRQ-owned state after any zero write.
	 * A wrapper that enters too early would hang in the real driver. */
	if (pwm_registers.stop_requested) { assert(!pwm_registers.driver_running); }
	pwm_registers.stop_requested = false;
	if (before_pwm_start) { void (*hook)(void) = before_pwm_start; before_pwm_start = NULL; hook(); }
	pwm_registers.enabled = pwm_registers.running = pwm_registers.driver_running = true;
	pwm_registers.pulse_ns = pulse;
	pwm_registers.stopped_event = false;
	if (after_pwm_start) { void (*hook)(void) = after_pwm_start; after_pwm_start = NULL; hook(); }
	return 0;
}
static int pm_device_state_get(void *dev, enum pm_device_state *out)
{
	(void)dev; *out = power_state; return pm_error;
}
static void pm_device_busy_set(void *dev) { (void)dev; pm_busy = true; }
static void pm_device_busy_clear(void *dev) { (void)dev; pm_busy = false; }
static int nrfx_timer_init(nrfx_timer_t *timer, const nrfx_timer_config_t *config, void *handler)
{
	(void)timer; assert(config->frequency == 1000000 && config->bit_width == 32 && !handler);
	return init_error;
}
static void nrfx_timer_uninit(nrfx_timer_t *timer) { (void)timer; timer_registers.running = false; }
static void nrfx_timer_disable(nrfx_timer_t *timer) { (void)timer; timer_registers.running = false; }
static void nrfx_timer_enable(nrfx_timer_t *timer)
{
	(void)timer; timer_registers.running = true;
	if (after_timer_enable) { void (*hook)(void) = after_timer_enable; after_timer_enable = NULL; hook(); }
}
static void nrfx_timer_clear(nrfx_timer_t *timer)
{
	(void)timer; timer_registers.counter = 0;
	if (after_timer_clear) { void (*hook)(void) = after_timer_clear; after_timer_clear = NULL; hook(); }
}
static bool nrf_timer_event_check(void *reg, unsigned event)
{
	assert(reg == NRF_TIMER4); return timer_registers.event[event];
}
static void nrf_timer_event_clear(void *reg, unsigned event)
{
	assert(reg == NRF_TIMER4); timer_registers.event[event] = false;
}
static void nrf_timer_cc_set(void *reg, unsigned channel, uint32_t value)
{
	assert(reg == NRF_TIMER4);
	if (before_cc_write) { void (*hook)(void) = before_cc_write; before_cc_write = NULL; hook(); }
	timer_registers.cc[channel] = value;
}
static void nrfx_timer_extended_compare(nrfx_timer_t *timer, unsigned channel,
				       uint32_t value, uint32_t shortcuts, bool interrupt)
{
	(void)timer; assert(!interrupt);
	timer_registers.cc[channel] = value;
	timer_registers.shorts |= shortcuts;
}
static uint32_t nrf_timer_event_address_get(void *reg, unsigned event)
{
	assert(reg == NRF_TIMER4); return 100 + event;
}
static uint32_t nrf_pwm_task_address_get(void *reg, unsigned task)
{
	assert(reg == NRF_PWM1 && task == 0); return 200;
}
static int nrfx_gppi_conn_alloc(uint32_t event, uint32_t task, nrfx_gppi_handle_t *handle)
{
	assert(task == 200 && event >= 100 && event < 102);
	if (++allocations == alloc_fail_at) { return -ENOMEM; }
	for (unsigned i = 0; i < 2; ++i) {
		if (!ppi[i].allocated) {
			ppi[i].allocated = true; ppi[i].event = event - 100; *handle = i; return 0;
		}
	}
	return -ENOMEM;
}
static void nrfx_gppi_conn_free(uint32_t event, uint32_t task, nrfx_gppi_handle_t handle)
{
	assert(ppi[handle].event == event - 100 && task == 200);
	ppi[handle].allocated = ppi[handle].enabled = false;
}
static void nrfx_gppi_conn_enable(nrfx_gppi_handle_t handle) { assert(ppi[handle].allocated); ppi[handle].enabled = true; }
static void nrfx_gppi_conn_disable(nrfx_gppi_handle_t handle) { assert(ppi[handle].allocated); ppi[handle].enabled = false; }
static bool v_finite(const float *values, size_t count)
{
	for (size_t i = 0; i < count; ++i) {
		uint32_t bits; memcpy(&bits, &values[i], sizeof(bits));
		if ((bits & 0x7f800000U) == 0x7f800000U) { return false; }
	}
	return true;
}
#endif
