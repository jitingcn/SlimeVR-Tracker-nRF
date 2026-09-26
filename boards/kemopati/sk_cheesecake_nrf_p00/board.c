/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/devicetree.h>
#include <zephyr/init.h>

#include <hal/nrf_gpio.h>

#if CONFIG_CUSTOMER_INFO && CONFIG_CUSTOMER_INFO_SKT0
#include <hal/nrf_uicr.h>
#include <zephyr/sys/byteorder.h>
#include "../../../src/system/customer_info.h"
#include "../sk_common/customer_info_skt0.h"
#endif

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

#if CONFIG_CUSTOMER_INFO && CONFIG_CUSTOMER_INFO_SKT0
void customer_info_read_board_variant(struct customer_info_result *out)
{
	uint8_t record[CUSTOMER_INFO_RECORD_SIZE];
	const volatile uint32_t *customer = &NRF_UICR->CUSTOMER[0];

	for (size_t i = 0; i < CUSTOMER_INFO_RECORD_SIZE / sizeof(uint32_t); i++) {
		uint32_t word = customer[i];
		sys_put_le32(word, record + i * sizeof(uint32_t));
	}
	customer_info_skt0_decode(record, sizeof(record), out);
	out->source = out->status == CUSTOMER_INFO_ABSENT
		? CUSTOMER_INFO_SOURCE_NONE : CUSTOMER_INFO_SOURCE_SLOT_A;
#if CONFIG_CUSTOMER_INFO_SKT0_PRODUCT_ID >= 0 && CONFIG_CUSTOMER_INFO_SKT0_HARDWARE_REVISION >= 0
	if (out->status == CUSTOMER_INFO_VALID) {
		out->identity = out->info.product_id == CONFIG_CUSTOMER_INFO_SKT0_PRODUCT_ID &&
			out->info.hardware_revision == CONFIG_CUSTOMER_INFO_SKT0_HARDWARE_REVISION
			? CUSTOMER_INFO_IDENTITY_MATCH : CUSTOMER_INFO_IDENTITY_MISMATCH;
	}
#endif
}
#endif
