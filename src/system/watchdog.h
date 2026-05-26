/*
 * Copyright (c) 2025 SlimeVR Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef _WATCHDOG_H_
#define _WATCHDOG_H_

#include <zephyr/kernel.h>
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Watchdog reset threshold - exceed this to enter DFU mode
 */
#define WATCHDOG_RESET_THRESHOLD 3

/**
 * @brief Watchdog channel IDs (ordered by priority)
 */
typedef enum {
	// Critical - must monitor
	WDT_CHANNEL_SENSOR = 0,     // Sensor acquisition
	WDT_CHANNEL_ESB,            // ESB wireless protocol

	// Recommended - important functions
	WDT_CHANNEL_CONNECTION,     // Connection management
	WDT_CHANNEL_POWER,          // Power management
	WDT_CHANNEL_BUTTON,         // Button handling

	// Optional - auxiliary functions
	WDT_CHANNEL_LED,            // LED control
	WDT_CHANNEL_STATUS,         // Status display
	WDT_CHANNEL_CALIBRATION,    // Calibration processing

	WDT_CHANNEL_COUNT
} wdt_channel_id_t;

#if defined(CONFIG_TASK_WDT)
/*
 * Full implementation when Task WDT is enabled
 */

/**
 * @brief Initialize the watchdog system
 *
 * Checks reset count and may enter DFU mode if threshold exceeded.
 *
 * @return 0 on normal startup, 1 if entering DFU mode, <0 on error
 */
int watchdog_init(void);

/**
 * @brief Register a thread to the watchdog
 *
 * @param channel Channel ID to register
 * @param timeout_ms Timeout in milliseconds (0 = use default)
 * @return Channel handle on success, <0 on error
 */
int watchdog_register_thread(wdt_channel_id_t channel, uint32_t timeout_ms);

/**
 * @brief Feed (kick) a watchdog channel
 *
 * Each thread must call this periodically to prevent timeout.
 *
 * @param channel Channel ID to feed
 */
void watchdog_feed(wdt_channel_id_t channel);

/**
 * @brief Pause a watchdog channel
 *
 * Use before long operations or low power modes.
 *
 * @param channel Channel ID to pause
 */
void watchdog_pause(wdt_channel_id_t channel);

/**
 * @brief Resume a paused watchdog channel
 *
 * @param channel Channel ID to resume
 */
void watchdog_resume(wdt_channel_id_t channel);

/**
 * @brief Check if last reset was caused by watchdog
 *
 * @return true if WDT caused the reset
 */
bool watchdog_caused_reset(void);

/**
 * @brief Get the current watchdog reset count
 *
 * @return Number of consecutive WDT resets
 */
uint8_t watchdog_get_reset_count(void);

/**
 * @brief Clear the watchdog reset count
 */
void watchdog_clear_reset_count(void);

/**
 * @brief Mark system boot as successful
 *
 * Call this after all critical systems have initialized successfully.
 * This clears the reset count to prevent entering DFU mode.
 */
void watchdog_mark_boot_success(void);

/**
 * @brief Get OTA RAM engine GPREGRET value saved at boot
 * @return GPREGRET value (0xD0-0xDE if from OTA engine, 0 otherwise)
 */
uint8_t watchdog_get_ota_gpregret(void);

/**
 * @brief Suspend all watchdog channels before entering low power mode
 */
void watchdog_suspend_all(void);

/**
 * @brief Resume all watchdog channels after exiting low power mode
 */
void watchdog_resume_all(void);

/**
 * @brief Get the name of a watchdog channel
 *
 * @param channel Channel ID
 * @return Channel name string
 */
const char *watchdog_get_channel_name(wdt_channel_id_t channel);

#else /* !CONFIG_TASK_WDT */
/*
 * Stub implementations when Task WDT is disabled.
 * This allows the code to compile and run without watchdog support.
 */
static inline int watchdog_init(void) { return 0; }
static inline int watchdog_register_thread(wdt_channel_id_t channel, uint32_t timeout_ms)
{
	(void)channel;
	(void)timeout_ms;
	return 0;
}
static inline void watchdog_feed(wdt_channel_id_t channel) { (void)channel; }
static inline void watchdog_pause(wdt_channel_id_t channel) { (void)channel; }
static inline void watchdog_resume(wdt_channel_id_t channel) { (void)channel; }
static inline bool watchdog_caused_reset(void) { return false; }
static inline uint8_t watchdog_get_reset_count(void) { return 0; }
static inline void watchdog_clear_reset_count(void) {}
static inline void watchdog_mark_boot_success(void) {}
static inline uint8_t watchdog_get_ota_gpregret(void) { return 0; }
static inline void watchdog_suspend_all(void) {}
static inline void watchdog_resume_all(void) {}
static inline const char *watchdog_get_channel_name(wdt_channel_id_t channel)
{
	(void)channel;
	return "disabled";
}
#endif /* CONFIG_TASK_WDT */

#endif /* _WATCHDOG_H_ */
