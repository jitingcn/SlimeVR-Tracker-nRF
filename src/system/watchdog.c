/*
 * Copyright (c) 2025 SlimeVR Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>

/* Only compile when Task WDT is enabled */
#if defined(CONFIG_TASK_WDT)

#include "watchdog.h"
#include "globals.h"
#include <zephyr/task_wdt/task_wdt.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>
#include <hal/nrf_power.h>

LOG_MODULE_REGISTER(watchdog, LOG_LEVEL_INF);

/* Adafruit bootloader DFU magic number (DFU_MAGIC_UF2_RESET) */
#define ADAFRUIT_DFU_MAGIC ADAFRUIT_DFU_MAGIC_UF2_RESET

/* Channel handles from task_wdt_add() */
static int channel_ids[WDT_CHANNEL_COUNT];

/* Store timeout values for pause/resume functionality */
static uint32_t channel_timeouts[WDT_CHANNEL_COUNT];

/* Watchdog state */
static bool watchdog_initialized = false;
static bool boot_success_marked = false;
static uint8_t saved_gpregret = 0;  /* Saved at PRE_KERNEL for OTA debug */

/* Channel names for logging */
static const char *channel_names[] = {
	"sensor",
	"esb",
	"connection",
	"power",
	"button",
	"led",
	"status",
	"calibration"
};

/* Store WDT reset status before RESETREAS is cleared by early_check */
static bool last_reset_was_wdt = false;

/* Default timeout values in milliseconds */
static const uint32_t default_timeouts[] = {
	10000,    // sensor - I2C/SPI may need retries
	10000,    // esb - wireless retransmits
	10000,    // connection - depends on esb
	10000,    // power - increased for boot delay
	10000,    // button - increased for boot delay
	30000,   // led - can be suspended
	15000,   // status - error display cycles
	60000    // calibration - long operations
};

/**
 * @brief Timeout callback - called before hardware WDT triggers reset
 */
static void watchdog_timeout_callback(int channel_id, void *user_data)
{
	wdt_channel_id_t channel = (wdt_channel_id_t)(intptr_t)user_data;

	/* Disable interrupts to prevent being interrupted */
	unsigned int key = irq_lock();

	/* Save fault information to retained memory (outside CRC, no update needed) */
	if (retained) {
		retained->watchdog_state.last_failed_channel = channel;
		retained->watchdog_state.last_reset_uptime = k_uptime_get_32();
		retained->watchdog_state.magic = WATCHDOG_STATE_MAGIC;
	}

	/* Log critical failure information */
	LOG_ERR("=== WATCHDOG TIMEOUT ===");
	LOG_ERR("Failed channel: %d (%s)", channel_id, channel_names[channel]);
	LOG_ERR("System uptime: %u ms", k_uptime_get_32());

	/* Important: When a callback is provided to task_wdt_add(), the task_wdt
	 * does NOT automatically reboot. The hardware WDT fallback will only
	 * trigger if we don't return from this callback. We spin here to let
	 * the hardware WDT timeout and reset the system.
	 */
	LOG_ERR("Waiting for hardware WDT to reset system...");

	/* Spin forever - hardware WDT will reset the system */
	uint32_t start_cycles = k_cycle_get_32();
	uint32_t last_print_ms = 0;

	while (1) {
		/* Calculate elapsed time and print status every second */
		uint32_t elapsed_ms = k_cyc_to_ms_floor32(k_cycle_get_32() - start_cycles);
		if (elapsed_ms >= last_print_ms + 1000) {
			printk("WDT: Still waiting... %u ms\n", elapsed_ms);
			last_print_ms = elapsed_ms;
		}

		/* Small delay to reduce CPU load */
		k_busy_wait(1000);
	}

	/* Restore interrupts (unreachable, but for completeness) */
	irq_unlock(key);
}

/**
 * @brief Check if we should enter DFU mode due to repeated WDT resets
 */
static bool should_enter_dfu(void)
{
	/* Use saved WDT reset status (RESETREAS was cleared in early_check) */
	if (!last_reset_was_wdt) {
		return false;
	}

	if (!retained) {
		return false;
	}

	uint8_t count = retained->watchdog_state.reset_count;
	return count >= WATCHDOG_RESET_THRESHOLD;
}

/**
 * @brief Enter DFU/Bootloader mode for firmware recovery
 *
 * When WDT reset count reaches the threshold, enter DFU mode for recovery.
 * The reset counter is cleared before entering DFU to prevent repeated
 * DFU entry loops when returning from bootloader.
 */
static void enter_dfu_mode(void)
{
	LOG_WRN("WDT reset count reached threshold (%d), entering DFU mode",
		WATCHDOG_RESET_THRESHOLD);

	/* Clear reset counter BEFORE entering DFU to prevent looping back into DFU
	 * when bootloader times out and returns to firmware.
	 */
	if (retained) {
		retained->watchdog_state.reset_count = 0;

		/* Update retained data to persist the cleared counter.
		 * Note: retained_update() updates CRC and uptime tracking.
		 * The watchdog_state is outside CRC but this ensures
		 * proper state management.
		 */
		retained_update();
	}

#if CONFIG_BUILD_OUTPUT_UF2
	/* Adafruit bootloader: Set GPREGRET to enter UF2 DFU mode */
	NRF_POWER->GPREGRET = ADAFRUIT_DFU_MAGIC;
	k_msleep(100);
	sys_reboot(SYS_REBOOT_COLD);
#elif CONFIG_BOARD_HAS_NRF5_BOOTLOADER
	/* nRF5 SDK bootloader - implementation depends on specific bootloader */
	sys_reboot(SYS_REBOOT_COLD);
#else
	/* No bootloader available, perform cold reboot */
	LOG_ERR("No bootloader available, performing cold reboot");
	sys_reboot(SYS_REBOOT_COLD);
#endif
}

/**
 * @brief Early check for WDT reset (must be called before RESETREAS is cleared)
 */
static int watchdog_early_check(void)
{
	/* Check if last reset was caused by watchdog - save for later use */
	last_reset_was_wdt = watchdog_caused_reset();

	/* Save GPREGRET for OTA RAM engine debug (survives system reset) */
	saved_gpregret = NRF_POWER->GPREGRET & 0xFF;
	if (saved_gpregret >= 0xD0 && saved_gpregret <= 0xDE) {
		/* Clear it so bootloader doesn't see it on next reset */
		NRF_POWER->GPREGRET = 0;
	}

	/* Clear reset reason flags early to prevent other code from seeing stale values */
#ifdef NRF_RESET
	NRF_RESET->RESETREAS = NRF_RESET->RESETREAS;
#else
	NRF_POWER->RESETREAS = NRF_POWER->RESETREAS;
#endif

	return 0;
}

/* Run early check at PRE_KERNEL level, before anything else */
SYS_INIT(watchdog_early_check, PRE_KERNEL_1, 0);

int watchdog_init(void)
{
	if (watchdog_initialized) {
		return 0;
	}

	/* Initialize channel ID array */
	for (int i = 0; i < WDT_CHANNEL_COUNT; i++) {
		channel_ids[i] = -1;
		channel_timeouts[i] = 0;  /* Will be set when registered */
	}

	/* Process WDT reset state */
	if (last_reset_was_wdt && retained) {
		LOG_WRN("System was reset by watchdog!");

		/* Check if watchdog_state is valid (magic number matches) */
		bool state_valid = (retained->watchdog_state.magic == WATCHDOG_STATE_MAGIC);

		if (state_valid) {
			/* Increment reset counter */
			retained->watchdog_state.reset_count++;
			retained->watchdog_state.total_wdt_resets++;
			LOG_WRN("WDT reset count: %d (total: %d)",
				retained->watchdog_state.reset_count,
				retained->watchdog_state.total_wdt_resets);

			/* Log last failed channel */
			if (retained->watchdog_state.last_failed_channel < WDT_CHANNEL_COUNT) {
				LOG_WRN("Last failed channel: %s",
					channel_names[retained->watchdog_state.last_failed_channel]);
			}

			/* Check if we should enter DFU due to repeated WDT resets */
			if (should_enter_dfu()) {
				enter_dfu_mode();
				return 1;  /* Won't reach here */
			}
		} else {
			/* First WDT reset or corrupted state - initialize */
			LOG_WRN("WDT state not valid, initializing");
			retained->watchdog_state.reset_count = 1;
			retained->watchdog_state.total_wdt_resets = 1;
			retained->watchdog_state.magic = WATCHDOG_STATE_MAGIC;
		}
	} else if (retained) {
		/* Non-WDT reset - clear counter but keep magic */
		retained->watchdog_state.reset_count = 0;
		retained->watchdog_state.magic = WATCHDOG_STATE_MAGIC;
	}

	/* Get the hardware WDT device */
	const struct device *wdt_dev = DEVICE_DT_GET(DT_NODELABEL(wdt));
	if (!device_is_ready(wdt_dev)) {
		LOG_WRN("WDT device not ready, watchdog disabled");
		/* Don't fail - allow system to boot without watchdog */
		watchdog_initialized = false;
		return 0;
	}

	/* Initialize Task WDT with the hardware WDT device */
	int err = task_wdt_init(wdt_dev);
	if (err < 0) {
		LOG_ERR("Failed to initialize task WDT: %d", err);
		/* Don't fail - allow system to boot without watchdog */
		watchdog_initialized = false;
		return 0;
	}

	watchdog_initialized = true;
	LOG_INF("Task watchdog initialized");
	return 0;
}

/**
 * @brief System init function to initialize watchdog
 *
 * This runs at APPLICATION level, AFTER retained_validate() has run.
 * This ensures retained memory is properly validated before we access it.
 * Priority 99 to run after retained_init (priority CONFIG_APPLICATION_INIT_PRIORITY).
 */
static int watchdog_sys_init(void)
{
	return watchdog_init();
}

uint8_t watchdog_get_ota_gpregret(void)
{
	return saved_gpregret;
}

/* Initialize watchdog at APPLICATION level, after retained memory is validated */
SYS_INIT(watchdog_sys_init, APPLICATION, 99);

int watchdog_register_thread(wdt_channel_id_t channel, uint32_t timeout_ms)
{
	if (!watchdog_initialized) {
		/* Silently skip if watchdog not initialized - may be called before init */
		LOG_WRN("%s: Watchdog not initialized, skipping", __func__);
		return 0;
	}

	if (channel >= WDT_CHANNEL_COUNT) {
		return -EINVAL;
	}

	/* Use default timeout if not specified */
	if (timeout_ms == 0) {
		timeout_ms = default_timeouts[channel];
	}

	/* Save timeout for pause/resume functionality */
	channel_timeouts[channel] = timeout_ms;

	int id = task_wdt_add(timeout_ms, watchdog_timeout_callback,
			      (void *)(intptr_t)channel);
	if (id < 0) {
		LOG_ERR("Failed to add watchdog channel %d: %d", channel, id);
		return id;
	}

	channel_ids[channel] = id;

	/* Feed immediately after registration to reset the timeout counter */
	task_wdt_feed(id);

	LOG_INF("Registered %s thread with %u ms timeout (channel %d)",
		channel_names[channel], timeout_ms, id);
	return id;
}

void watchdog_feed(wdt_channel_id_t channel)
{
	if (channel < WDT_CHANNEL_COUNT && channel_ids[channel] >= 0) {
		task_wdt_feed(channel_ids[channel]);
	}
}

void watchdog_pause(wdt_channel_id_t channel)
{
	if (channel < WDT_CHANNEL_COUNT && channel_ids[channel] >= 0) {
		/* Task WDT doesn't directly support pause, so we delete and re-add later */
		int err = task_wdt_delete(channel_ids[channel]);
		if (err == 0) {
			LOG_DBG("Watchdog channel %s paused", channel_names[channel]);
			channel_ids[channel] = -1;
			/* Note: channel_timeouts[channel] is preserved for resume */
		}
	}
}

void watchdog_resume(wdt_channel_id_t channel)
{
	if (channel < WDT_CHANNEL_COUNT && channel_ids[channel] < 0) {
		/* Re-register the channel with saved timeout (or default if not set) */
		uint32_t timeout = channel_timeouts[channel];
		if (timeout == 0) {
			timeout = default_timeouts[channel];
		}
		watchdog_register_thread(channel, timeout);
		LOG_DBG("Watchdog channel %s resumed with %u ms timeout",
			channel_names[channel], timeout);
	}
}

bool watchdog_caused_reset(void)
{
#ifdef NRF_RESET
	uint32_t reset_reason = NRF_RESET->RESETREAS;
	return (reset_reason & RESET_RESETREAS_DOG_Msk) != 0;
#else
	uint32_t reset_reason = NRF_POWER->RESETREAS;
	return (reset_reason & POWER_RESETREAS_DOG_Msk) != 0;
#endif
}

uint8_t watchdog_get_reset_count(void)
{
	if (retained) {
		return retained->watchdog_state.reset_count;
	}
	return 0;
}

void watchdog_clear_reset_count(void)
{
	if (retained) {
		retained->watchdog_state.reset_count = 0;
		/* Update retained data to persist the change */
		retained_update();
	}
	LOG_INF("WDT reset count cleared");
}

void watchdog_mark_boot_success(void)
{
	if (!boot_success_marked) {
		boot_success_marked = true;
		watchdog_clear_reset_count();
		LOG_INF("Boot success marked, WDT reset count cleared");
	}
}

void watchdog_suspend_all(void)
{
	if (watchdog_initialized) {
		task_wdt_suspend();
		LOG_DBG("All watchdog channels suspended");
	}
}

void watchdog_resume_all(void)
{
	if (watchdog_initialized) {
		task_wdt_resume();
		LOG_DBG("All watchdog channels resumed");
	}
}

const char *watchdog_get_channel_name(wdt_channel_id_t channel)
{
	if (channel < WDT_CHANNEL_COUNT) {
		return channel_names[channel];
	}
	return "unknown";
}

#endif /* CONFIG_TASK_WDT */
