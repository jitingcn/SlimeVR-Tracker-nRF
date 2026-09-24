/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/devicetree.h>
#include <zephyr/init.h>

#include <hal/nrf_gpio.h>

/*
 * Power-pin bring-up shared by the SK CheeseCake NRF P00 and P10 tracker
 * boards; both packages carry this file unchanged. The P10-only external clock
 * enable is handled when its devicetree property is present.
 */

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#define USER_GPIO_PIN(node_id, prop)                                                               \
	NRF_GPIO_PIN_MAP(DT_PROP(DT_GPIO_CTLR(node_id, prop), port), DT_GPIO_PIN(node_id, prop))

/* Drive the pin low and keep it driven, so a floating enable cannot turn the
 * heater, the LED supply or the external power cutoff on before the drivers
 * configure the pins themselves. */
static void board_output_off(uint32_t pin)
{
	nrf_gpio_pin_clear(pin);
	nrf_gpio_cfg(
		pin,
		NRF_GPIO_PIN_DIR_OUTPUT,
		NRF_GPIO_PIN_INPUT_DISCONNECT,
		NRF_GPIO_PIN_NOPULL,
		NRF_GPIO_PIN_S0S1,
		NRF_GPIO_PIN_NOSENSE
	);
}

static int board_sk_cheesecake_init(void)
{
	board_output_off(USER_GPIO_PIN(ZEPHYR_USER_NODE, heat_en_gpios));
	board_output_off(USER_GPIO_PIN(ZEPHYR_USER_NODE, led_en_gpios));

	/* Keep the active-high external power cutoff inactive during normal operation. */
	board_output_off(USER_GPIO_PIN(ZEPHYR_USER_NODE, sysoff_gpios));

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, eclk_en_gpios)
	/* Leave the external clock enable in its reset state: the IMU enables its
	 * own clock, and a warm reset must not keep driving the pin. */
	nrf_gpio_cfg_default(USER_GPIO_PIN(ZEPHYR_USER_NODE, eclk_en_gpios));
#endif

	return 0;
}

SYS_INIT(board_sk_cheesecake_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
