/*
	Thin wrappers around nrfx/Zephyr Nordic pin helpers.
	Prefer these over hand-rolled bit masks on PSEL / absolute pins.

	nrf_psel_cfg_default / nrf_gpio_configure_dt_log call LOG_INF: include this
	header AFTER LOG_MODULE_REGISTER() in the .c file.
*/
#ifndef SLIMENRF_NRF_GPIO_UTIL_H
#define SLIMENRF_NRF_GPIO_UTIL_H

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <hal/nrf_gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Peripheral PSEL "not connected" encoding.
 * Same as NRF_SPIM_PIN_NOT_CONNECTED / zephyr pinctrl PSEL_DISCONNECTED.
 */
static inline bool nrf_psel_is_connected(uint32_t psel)
{
	return psel != UINT32_MAX;
}

/** Connected PSEL register value -> absolute pin for nrf_gpio_*. */
static inline uint32_t nrf_psel_to_abs_pin(uint32_t psel)
{
	return NRF_GPIO_PIN_MAP(NRF_PIN_NUMBER_TO_PORT(psel), NRF_PIN_NUMBER_TO_PIN(psel));
}

/** gpio_dt_spec -> absolute pin via DT controller `port` property. */
static inline uint32_t nrf_gpio_dt_to_abs_pin(const struct gpio_dt_spec *spec)
{
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(gpio0))
	if (spec->port == DEVICE_DT_GET(DT_NODELABEL(gpio0))) {
		return NRF_GPIO_PIN_MAP(DT_PROP(DT_NODELABEL(gpio0), port), spec->pin);
	}
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(gpio1))
	if (spec->port == DEVICE_DT_GET(DT_NODELABEL(gpio1))) {
		return NRF_GPIO_PIN_MAP(DT_PROP(DT_NODELABEL(gpio1), port), spec->pin);
	}
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(gpio2))
	if (spec->port == DEVICE_DT_GET(DT_NODELABEL(gpio2))) {
		return NRF_GPIO_PIN_MAP(DT_PROP(DT_NODELABEL(gpio2), port), spec->pin);
	}
#endif
	return NRF_GPIO_PIN_MAP(0, spec->pin);
}

/** LOG_* format helpers: LOG_INF("x " NRF_ABS_PIN_LOG_FMT, NRF_ABS_PIN_LOG_ARGS(abs)); */
#define NRF_ABS_PIN_LOG_FMT "P%u.%02u"
#define NRF_ABS_PIN_LOG_ARGS(abs_pin) NRF_PIN_NUMBER_TO_PORT(abs_pin), NRF_PIN_NUMBER_TO_PIN(abs_pin)

/** If PSEL connected: Hi-Z reset default (nrf_gpio_cfg_default) + LOG_INF. Uses caller LOG_MODULE. */
static inline void nrf_psel_cfg_default(const char *msg, uint32_t psel)
{
	uint32_t abs_pin;

	if (!nrf_psel_is_connected(psel)) {
		return;
	}
	abs_pin = nrf_psel_to_abs_pin(psel);
	nrf_gpio_cfg_default(abs_pin);
	LOG_INF("%s " NRF_ABS_PIN_LOG_FMT, msg, NRF_ABS_PIN_LOG_ARGS(abs_pin));
}

/** gpio_pin_configure_dt + LOG_INF with Pport.pin. Uses caller LOG_MODULE. */
static inline void nrf_gpio_configure_dt_log(const char *msg, const struct gpio_dt_spec *spec,
					     gpio_flags_t flags)
{
	uint32_t abs_pin;

	if (!device_is_ready(spec->port)) {
		return;
	}
	gpio_pin_configure_dt(spec, flags);
	abs_pin = nrf_gpio_dt_to_abs_pin(spec);
	LOG_INF("%s " NRF_ABS_PIN_LOG_FMT, msg, NRF_ABS_PIN_LOG_ARGS(abs_pin));
}

#ifdef __cplusplus
}
#endif

#endif /* SLIMENRF_NRF_GPIO_UTIL_H */
