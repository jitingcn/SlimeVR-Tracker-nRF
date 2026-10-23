/*
 * Copyright (c) 2018 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <hal/nrf_gpio.h>
#include <zephyr/devicetree.h>

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
#define SENSOR_POWER_PIN \
	NRF_GPIO_PIN_MAP(DT_PROP(DT_GPIO_CTLR(ZEPHYR_USER_NODE, pwr_gpios), port), \
		DT_GPIO_PIN(ZEPHYR_USER_NODE, pwr_gpios))

static int board_foxsnacklitev3_init(void)
{
	/* P1.02 is an LDO enable, not the sensor supply itself. Preload high
	 * before enabling the output; System OFF later drives it low through
	 * the application's pwr-gpios lifecycle. WoM must retain sensor power.
	 */
	nrf_gpio_pin_set(SENSOR_POWER_PIN);
	nrf_gpio_cfg(SENSOR_POWER_PIN, NRF_GPIO_PIN_DIR_OUTPUT,
		NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,
		NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);
	return 0;
}

SYS_INIT(board_foxsnacklitev3_init, PRE_KERNEL_1,
	CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
