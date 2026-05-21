/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "retained.h"

#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/retained_mem.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>

#include "build_defines.h"

#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(retainedmemdevice))
#define MEMORY_REGION DT_PARENT(DT_ALIAS(retainedmemdevice))
#else
#error "retained_mem region not defined"
#endif

struct retained_data *retained = (struct retained_data *)DT_REG_ADDR(MEMORY_REGION);

#define RETAINED_CRC_OFFSET offsetof(struct retained_data, crc)
#define RETAINED_CHECKED_SIZE (RETAINED_CRC_OFFSET + sizeof(retained->crc))

static uint64_t init_time;

static int retained_init(void)
{
	init_time = k_uptime_ticks(); // Get current uptime in ticks as soon as possible
	return 0;
}

// TODO: priority?
SYS_INIT(retained_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

bool retained_validate(void)
{
	NRF_STATIC_ASSERT((RETAINED_CHECKED_SIZE <= 3072), "Retained data size exceeds 3 KB limit");

	uint64_t now = init_time;
//	uint64_t now = k_uptime_ticks(); // Get current uptime in ticks as soon as possible

	/* The residue of a CRC is what you get from the CRC over the
	 * message catenated with its CRC.  This is the post-final-xor
	 * residue for CRC-32 (CRC-32/ISO-HDLC) which Zephyr calls
	 * crc32_ieee.
	 */
	const uint32_t residue = 0x2144df1c;
	uint32_t crc = crc32_ieee((const uint8_t *)retained,
				  RETAINED_CHECKED_SIZE);
	bool valid = (crc == residue);

	/* Check the build timestamp of the firmware that last updated
	 * the retained data.
	 */
	valid &= (retained->build_timestamp == BUILD_TIMESTAMP);

	/* If the CRC isn't valid or the build timestamp is different
	 * from the current build timestamp, reset the retained data.
	 */
	if (!valid) {
		/* Clear entire retained data including watchdog_state when CRC/build_timestamp invalid
		 * (e.g., after firmware update). This ensures total_wdt_resets doesn't have garbage values.
		 */
		memset(retained, 0, sizeof(struct retained_data));
		retained->build_timestamp = BUILD_TIMESTAMP;
		retained->gyroSensScale[0] = 1.0f;
		retained->gyroSensScale[1] = 1.0f;
		retained->gyroSensScale[2] = 1.0f;
		// Initialize battery tracker to invalid state (-1)
		retained->max_battery_pptt = -1;
		retained->min_battery_pptt = -1;
		retained->battery_pptt_saved = -1;
#if CONFIG_SENSOR_USE_TCAL
		// Initialize boot calibration state
		#ifdef CONFIG_SENSOR_USE_BOOT_CALIBRATION
		retained->bootCalState.enabled = true;
		#else
		retained->bootCalState.enabled = false;
		#endif
		retained->bootCalState.completed = false;
		retained->bootCalState.attempt_count = 0;
		retained->bootCalState.doffset_valid = false;
		retained->bootCalState.doffset[0] = 0.0f;
		retained->bootCalState.doffset[1] = 0.0f;
		retained->bootCalState.doffset[2] = 0.0f;
		retained->tcal_enabled = true; // T-Cal compensation enabled by default
#endif
		/* Initialize watchdog_state with valid magic but zero counters */
		retained->watchdog_state.magic = WATCHDOG_STATE_MAGIC;
		retained->watchdog_state.reset_count = 0;
		retained->watchdog_state.total_wdt_resets = 0;
		retained->watchdog_state.last_failed_channel = 0;
		retained->watchdog_state.last_reset_uptime = 0;
	}

	/* Reset to accrue runtime from this session. */
	retained->uptime_latest = now;
	retained->battery_uptime_latest = now;

	return valid;
}

void retained_update(void)
{
	uint64_t now = k_uptime_ticks();

	retained->uptime_sum += (now - retained->uptime_latest);
	retained->uptime_latest = now;

	uint32_t crc = crc32_ieee((const uint8_t *)retained,
				  RETAINED_CRC_OFFSET);

	retained->crc = sys_cpu_to_le32(crc);
}
