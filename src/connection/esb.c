/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include "globals.h"
#include "sensor/calibration/calibration.h"
#include "sensor/calibration/tcal_heated.h"
#include "sensor/sensor.h"
#include "system/system.h"
#include "system/battery_tracker.h"
#include "system/test_mode.h"
#include "system/watchdog.h"
#include "system/wom_timeout.h"
#include "system/esb_ota.h"
#include "connection.h"
#include "tracker_events.h"
#include "zephyr/sys/byteorder.h"
#include "zephyr/sys/time_units.h"

#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <hal/nrf_clock.h>
#include <nrfx_power.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>

#include <stdlib.h>
#include "esb.h"
#include "tdma.h"
#if defined(CONFIG_TDMA_DIAGNOSTICS)
#include "radio_capture.h"
#endif
#include "channel_control.h"
#include "system/clock_control.h"

uint8_t last_reset = 0;
bool esb_state = false;
uint16_t led_clock = 0;
uint32_t led_clock_offset = 0;

int64_t connection_error_start_time = 0;
static bool shutdown_requested = false;
static bool pair_ack_pending = false; // True once step 1 is sent and we expect a receiver response

static struct esb_payload rx_payload;
static struct esb_payload tx_payload_pair = ESB_CREATE_PAYLOAD(0, 0, 0, 0, 0, 0, 0, 0, 0);

static uint8_t paired_addr[8] = {0};

/* ---------------------------------------------------------------------------
 * ESB connection state machine.
 *
 * Explicit state replaces the implicit esb_paired/ping_failures matrix so
 * pairing, connection, and error recovery transitions are visible in one
 * place. Radio/data readiness is unchanged: esb_ready() == PAIRED.
 *
 *   PAIRING    - pairing burst to discovery address (entered on boot if
 *                unpaired, or via esb_reset_pair/remote clear)
 *   PAIRED     - paired address active, normal operation
 *   RECOVERING - ping loss detected; ping counter keeps accumulating
 *                failures for the shutdown watchdog, while the flag
 *                gates repeated recovery bursts from esb_thread
 * ------------------------------------------------------------------------- */
typedef enum {
	ESB_ST_PAIRING,
	ESB_ST_PAIRED,
	ESB_ST_RECOVERING,
} esb_conn_state_t;

static esb_conn_state_t esb_conn_state = ESB_ST_PAIRING;

static bool esb_initialized = false;

/* Preferred ESB channels: even channels outside WiFi/BT-heavy spectrum
 * (upstream SlimeVR selection). */
static const uint8_t __maybe_unused ESB_ALLOWED_CHANNELS[] = {
	0, 2, 52, 72, 74, 76, 78, 82, 84, 86, 88, 50, 24, 48,
	70, 68, 46, 44, 20, 54, 56, 28, 30,
	6, 8, 10, 12, 14, 16, 18, 32, 34,
	36, 38, 40, 42, 58, 60, 62, 64, 66,
};
#define ESB_ALLOWED_CHANNELS_COUNT ARRAY_SIZE(ESB_ALLOWED_CHANNELS)

/* Shared producer/lifecycle admission; recursive for owner-driven probes. */
K_MUTEX_DEFINE(esb_radio_lock);
static uint8_t radio_channel;
static uint32_t own_pong_time;
static bool own_pong_seen;
static bool pairing_search_active;
static bool radio_user_disabled;
static uint32_t pairing_request;
static uint32_t radio_session_generation;
static uint8_t search_home;
static uint8_t search_index;
static bool channel_search;
static bool channel_wait_normal;
static volatile bool channel_found;
static volatile bool channel_heard;
static int64_t search_deadline;
static int64_t search_probe_at;
static volatile bool channel_redirect_pending;
static uint8_t channel_redirect;
static bool channel_confirm_capable;
static bool channel_legacy_peer;
static bool channel_confirmed;
static bool ping_channel_confirm_sent;
static bool pair_provisional;
static struct led_token pair_feedback;
static int64_t pair_confirm_deadline;

/* Blind search visits all 51 even channels; an explicit odd home stays first. */
static uint8_t channel_candidate_count(uint8_t home)
{
	return 51 + (home & 1);
}

static uint8_t channel_candidate(uint8_t home, unsigned index)
{
	if (index == 0) {
		return home;
	}
	for (unsigned i = 0; i < ESB_ALLOWED_CHANNELS_COUNT; ++i) {
		uint8_t ch = ESB_ALLOWED_CHANNELS[i];
		if (ch != home && --index == 0) {
			return ch;
		}
	}
	for (unsigned ch = 0; ch <= 100; ch += 2) {
		bool preferred = ch == home;
		for (unsigned i = 0; i < ESB_ALLOWED_CHANNELS_COUNT; ++i) {
			preferred |= ESB_ALLOWED_CHANNELS[i] == ch;
		}
		if (!preferred && --index == 0) {
			return ch;
		}
	}
	return home;
}
#define TX_ERROR_THRESHOLD 300
#define RADIO_RETRANSMIT_DELAY CONFIG_RADIO_RETRANSMIT_DELAY
#define RADIO_RF_CHANNEL CONFIG_RADIO_RF_CHANNEL

#if defined(CONFIG_CONNECTION_ENABLE_ACK)
#define CONNECTION_ENABLE_ACK true
#else
#define CONNECTION_ENABLE_ACK false
#endif

// Require N consecutive successful ACK probes before clearing connection error
#ifndef PING_RECOVERY_THRESHOLD
#define PING_RECOVERY_THRESHOLD 1
#endif

/* Keep recovery probes comfortably inside the receiver membership timeout.
 * Long exponential backoff created a feedback loop after receiver reboot:
 * ~10 s PING cadence versus a 5 s active timeout repeatedly removed slots. */
#define PING_BACKOFF_LVL1_THRESHOLD 2
#define PING_BACKOFF_LVL1_MS        500

LOG_MODULE_REGISTER(esb_event, LOG_LEVEL_INF);

static void esb_thread(void);
K_THREAD_DEFINE(esb_thread_id, 1024, esb_thread, NULL, NULL, NULL, ESB_THREAD_PRIORITY, 0, 0);
static int64_t last_tx_time = 0;

/*
 * nRF54L only: how long to keep HFCLK (and constant latency) up after the last
 * ESB TX before stopping it. Frequent HFXO/PLL restarts are unreliable on
 * nRF54L (anomaly 20/39), so clocks are stopped only after a longer idle
 * period instead of after every drained TX.
 */
#define ESB_CLOCK_IDLE_STOP_MS 3000

static uint32_t ping_success_streak = 0; // consecutive success counter
static bool ping_pending = false;
static bool ping_failed = false;

static uint32_t ping_failures = 0;
static uint32_t ping_ctr_sent = 0;
static uint8_t ping_counter = 0;
static int64_t ping_send_time = 0;


// Track send cycles for recent PINGs (circular buffer)
#define PING_HISTORY_SIZE 10
struct ping_history_entry {
	uint8_t counter;
	// ticks at ping send time, NETWORK tick domain (sync/offset math)
	uint32_t ping_ticks;
	// ticks at ping send time, KERNEL tick domain (RTT vs t4 stamp)
	uint32_t ping_ticks_kernel;
	bool valid;
};
static struct ping_history_entry ping_history[PING_HISTORY_SIZE] = {0};
static uint8_t ping_history_idx = 0;

static uint8_t received_remote_command = ESB_PONG_FLAG_NORMAL;
static uint8_t acked_remote_command = ESB_PONG_FLAG_NORMAL;
static bool remote_command_rejected;
static uint32_t remote_command_generation;
static uint32_t executing_shutdown_generation;
static uint32_t shutdown_feedback_generation, shutdown_feedback_request;
static uint32_t shutdown_accepted_event, shutdown_terminal_event;
static uint16_t acked_test_rate_tps = 0; // TEST_MODE_ON payload at ack time
static uint16_t executing_test_rate_tps = 0; // Snapshot for the in-flight TEST_MODE_ON execution
static int64_t remote_command_receive_time = 0;
static uint32_t received_channel_value = 0; // Store channel value from PONG data[8-11]
static uint32_t executing_channel_value;
static uint32_t received_test_rate_tps = 0; // Optional target TPS riding on TEST_MODE_ON (data[8-9])
#if CONFIG_CONNECTION_RAW_COLLECTION
static uint8_t received_batch_rate_hz;
static uint8_t acked_batch_rate_hz;
static uint8_t executing_batch_rate_hz;
static uint8_t received_metadata_mask;
static uint8_t received_metadata_chunk;
static uint16_t received_metadata_token;
static bool metadata_echo_pending;
#endif
static uint8_t received_sens_auto_axis;
static uint16_t received_sens_auto_revolutions;
static float received_sens_data[3] = {0};

/* Cache a completed transaction until its admitted response is consumed.
 * NORMAL and detected receiver restarts retire the receiver's token epoch. */
static struct {
	uint16_t minutes, token;
	uint16_t cached_minutes, cached_token, result;
	uint16_t sent_token;
	uint8_t result_flag, sent_flag;
	bool requested, cached, response_pending;
	uint32_t generation;
} wom_command;

static void esb_wom_reset(void)
{
	unsigned key = irq_lock();
	uint32_t generation = wom_command.generation + 1;
	memset(&wom_command, 0, sizeof(wom_command));
	wom_command.generation = generation;
	irq_unlock(key);
}

static void esb_wom_receive(uint16_t minutes, uint16_t token)
{
	/* Local changes invalidate successful readbacks, but CLEAR and ENTER
	 * acknowledge completed actions and must never rerun on a duplicate.
	 * The getter is atomic; persistence and power admission stay deferred. */
	if (wom_command.cached && minutes == wom_command.cached_minutes
	    && token == wom_command.cached_token
	    && (wom_command.result_flag == ESB_PING_FLAG_WOM_ERROR
	        || minutes == ESB_WOM_TIMEOUT_CLEAR || minutes == ESB_WOM_ENTER
	        || wom_timeout_get_minutes() == wom_command.result)) {
		wom_command.response_pending = true;
		return;
	}
	if (wom_command.requested && minutes == wom_command.minutes
	    && token == wom_command.token) {
		return;
	}
	wom_command.minutes = minutes;
	wom_command.token = token;
	wom_command.requested = true;
	wom_command.response_pending = false;
	wom_command.sent_token = 0;
	wom_command.sent_flag = ESB_PONG_FLAG_NORMAL;
	++wom_command.generation;
}

static void esb_wom_process(void)
{
	unsigned key = irq_lock();
	if (!wom_command.requested || (!channel_confirmed && !channel_legacy_peer)
	    || channel_redirect_pending) {
		irq_unlock(key);
		return;
	}
	uint16_t minutes = wom_command.minutes;
	uint16_t token = wom_command.token;
	uint32_t generation = wom_command.generation;
	irq_unlock(key);

	/* Persistence and power admission run only in the radio owner, IRQs on. */
	bool clear = minutes == ESB_WOM_TIMEOUT_CLEAR;
	bool enter = minutes == ESB_WOM_ENTER;
	int err = token == 0 || (!clear && !enter && minutes > 35791) ? -EINVAL
		: enter ? sys_request_WOM_now()
		: clear ? wom_timeout_set_minutes(0)
		: minutes == 0 ? 0 : wom_timeout_set_minutes(minutes);
	uint16_t result = err ? (uint16_t)-err
		: enter ? ESB_WOM_ENTER : clear ? 0 : minutes ? minutes : wom_timeout_get_minutes();
	if (!err && !clear && !enter && result == 0) {
		err = -ENOTSUP;
		result = ENOTSUP;
	}

	key = irq_lock();
	if (generation == wom_command.generation) {
		wom_command.requested = false;
		wom_command.cached = true;
		wom_command.cached_minutes = minutes;
		wom_command.cached_token = token;
		wom_command.result = result;
		wom_command.result_flag = err ? ESB_PING_FLAG_WOM_ERROR
			: ESB_PING_FLAG_WOM_RESULT;
		wom_command.response_pending = true;
	}
	irq_unlock(key);
}

static void esb_wom_prepare_ping(uint8_t *data)
{
	/* Flag/data getters run separately. Re-snapshot a WOM result as one unit
	 * immediately before CRC so an intervening RX cannot splice transactions. */
	unsigned key = irq_lock();
	if (data[7] == ESB_PING_FLAG_WOM_RESULT
	    || data[7] == ESB_PING_FLAG_WOM_ERROR) {
		data[7] = wom_command.response_pending ? wom_command.result_flag : ESB_PONG_FLAG_NORMAL;
		uint16_t result = wom_command.response_pending ? wom_command.result : 0;
		uint16_t token = wom_command.response_pending ? wom_command.cached_token : 0;
		data[8] = result >> 8;
		data[9] = result;
		data[10] = token >> 8;
		data[11] = token;
	}
	irq_unlock(key);
}


typedef void (*esb_remote_cmd_fn)(void);

struct esb_remote_cmd {
	uint8_t flag;
	const char *name;
	esb_remote_cmd_fn fn;
};

static void remote_print_meow(void);

static int esb_remote_cmd_shutdown(void)
{
	LOG_WRN("Executing remote command: SHUTDOWN");
	if (!shutdown_feedback_request || shutdown_feedback_generation != executing_shutdown_generation) {
		shutdown_feedback_generation = executing_shutdown_generation;
		shutdown_feedback_request = led_request_id();
		shutdown_accepted_event = led_event_id();
		shutdown_terminal_event = led_event_id();
	}
	return sys_command_shutdown_request(shutdown_feedback_request, shutdown_accepted_event, shutdown_terminal_event);
}

static void esb_remote_cmd_calibrate(void)
{
	LOG_INF("Executing remote command: CALIBRATE");
	sensor_request_calibration();
}

static void esb_remote_cmd_calibrate_acc(void)
{
#if CONFIG_SENSOR_USE_ACCEL_CALIBRATION
	LOG_INF("Executing remote command: accelerometer calibration (18 orientations)");
	sensor_request_calibration_accel();
#else
	LOG_WRN("Remote accelerometer calibration not supported (disabled in config)");
	cal_event_reject(CAL_KIND_ACCEL_POSES, CAL_REASON_UNSUPPORTED);
	tracker_events_notify();
	led_request_event(LED_OWNER_ACC, led_request_id(), led_event_id(), LED_REJECTED);
#endif
}

static void esb_remote_cmd_meow(void)
{
	LOG_INF("Executing remote command: MEOW");
	remote_print_meow();
}

static void esb_remote_cmd_scan(void)
{
	LOG_INF("Executing remote command: SCAN");
	int err = sensor_request_scan(true, true);
	if (err) {
		LOG_ERR("Remote sensor scan failed: %d", err);
	}
}

#if CONFIG_SENSOR_USE_MAGNETOMETER
static void esb_remote_cmd_mag_clear(void)
{
	LOG_INF("Executing remote command: MAG_CLEAR");
	sensor_calibration_clear_mag(NULL, true, true);
}

static void esb_remote_cmd_mag_cal(void)
{
	LOG_INF("Executing remote command: MAG_CAL");
	sensor_request_calibration_mag();
}

static void esb_remote_cmd_mag_on(void)
{
	LOG_INF("Executing remote command: MAG_ON");
	sensor_set_mag_enabled(true);
}

static void esb_remote_cmd_mag_off(void)
{
	LOG_INF("Executing remote command: MAG_OFF");
	sensor_set_mag_enabled(false);
}

static void esb_remote_cmd_mag_auto_on(void)
{
	LOG_INF("Executing remote command: MAG_AUTO_ON");
	sensor_calibration_set_online_mag_enabled(true);
}

static void esb_remote_cmd_mag_auto_off(void)
{
	LOG_INF("Executing remote command: MAG_AUTO_OFF");
	sensor_calibration_set_online_mag_enabled(false);
}
#endif

static void esb_remote_cmd_tcal_on(void)
{
#if CONFIG_SENSOR_USE_TCAL
	LOG_INF("Executing remote command: TCAL_ON");
	sensor_tcal_set_enabled(true);
#else
	led_request_event(LED_OWNER_TCAL, led_request_id(), led_event_id(), LED_REJECTED);
#endif
}

static void esb_remote_cmd_tcal_off(void)
{
#if CONFIG_SENSOR_USE_TCAL
	LOG_INF("Executing remote command: TCAL_OFF");
	sensor_tcal_set_enabled(false);
#else
	led_request_event(LED_OWNER_TCAL, led_request_id(), led_event_id(), LED_REJECTED);
#endif
}

static void esb_remote_cmd_tcal_heated_start(void)
{
#if CONFIG_SENSOR_TCAL_HEATED
	/* Consume this request even on refusal; never retry a rejected start later.
	 * The radio echo acknowledges delivery, not heater acceptance. */
	int err = sensor_tcal_heated_start(CONFIG_SENSOR_TCAL_HEATED_DEFAULT_TARGET_C);
	if (err) {
		LOG_WRN("Remote command: TCAL_HEATED_START rejected: %d", err);
	} else {
		LOG_INF("Executing remote command: TCAL_HEATED_START");
	}
#else
	LOG_WRN("Remote command: TCAL_HEATED_START unsupported (heated T-Cal disabled)");
	led_request_event(LED_OWNER_TCAL, led_request_id(), led_event_id(), LED_REJECTED);
#endif
}

static void esb_remote_cmd_tdma_on(void)
{
	LOG_INF("Executing remote command: TDMA_ON");
	tdma_user_set_enabled(true);
}

static void esb_remote_cmd_tdma_off(void)
{
	LOG_INF("Executing remote command: TDMA_OFF");
	tdma_user_set_enabled(false);
}

static void esb_remote_cmd_test_mode_on(void)
{
	/* Optional target TPS rides in PING data[8-9]; 0 = built-in default.
	 * Read the pre-execution snapshot (see esb_thread) so a PONG landing
	 * mid-execution cannot split apply/ack across different values. */
	uint16_t tps = executing_test_rate_tps;
	test_mode_set_target_tps(tps);
	test_mode_user_set(true);
	if (tps == 0) {
		LOG_INF("Executing remote command: TEST_MODE_ON (default rate)");
		return;
	}
	/* Field-diagnosis view: raw target vs capacity-clamped effective rate. */
	uint16_t effective_tps = test_mode_effective_tps();
	uint16_t frame_ticks = tdma_frame_ticks_get();
	LOG_INF(
		"Executing remote command: TEST_MODE_ON target=%u TPS effective=%u TPS (frame=%u ticks)",
		tps,
		effective_tps,
		frame_ticks
	);
}

static void esb_remote_cmd_test_mode_off(void)
{
	LOG_INF("Executing remote command: TEST_MODE_OFF");
	test_mode_user_set(false);
}

static void esb_remote_cmd_reboot(void)
{
	LOG_WRN("Executing remote command: REBOOT");
	int err = sys_user_reboot();
	if (err) {
		LOG_WRN("Reboot request rejected: %d", err);
	}
}

static void esb_remote_cmd_clear(void)
{
	LOG_WRN("Executing remote command: CLEAR (clear pairing)");
	esb_clear_pair();
}

static void esb_remote_cmd_dfu(void)
{
#if CONFIG_BUILD_OUTPUT_UF2 || CONFIG_BOARD_HAS_NRF5_BOOTLOADER || CONFIG_BOOTLOADER_MCUBOOT
	LOG_WRN("Executing remote command: DFU (enter bootloader)");
	sys_enter_dfu(false);
#else
	LOG_WRN("Remote command: DFU not supported (no bootloader)");
	led_request_event(LED_OWNER_SYSTEM, led_request_id(), led_event_id(), LED_REJECTED);
#endif
}

static void esb_remote_cmd_dfu_ota(void)
{
#if CONFIG_BUILD_OUTPUT_UF2 || CONFIG_BOARD_HAS_NRF5_BOOTLOADER || CONFIG_BOOTLOADER_MCUBOOT
	LOG_WRN("Executing remote command: DFU_OTA (enter OTA bootloader)");
	sys_enter_dfu(true);
#else
	LOG_WRN("Remote command: DFU_OTA not supported (no bootloader)");
	led_request_event(LED_OWNER_SYSTEM, led_request_id(), led_event_id(), LED_REJECTED);
#endif
}

static void esb_remote_cmd_set_channel(void)
{
	LOG_INF("Executing remote command: SET_CHANNEL to %u", executing_channel_value);
	int err = channel_control_set(executing_channel_value);
	if (err) {
		LOG_ERR("Channel update failed: %d (RAM/radio may already be updated)", err);
	} else {
		LOG_INF("RF channel saved and ESB reinitialized with channel %u", executing_channel_value);
	}
}

static void esb_remote_cmd_clear_channel(void)
{
	LOG_INF("Executing remote command: CLEAR_CHANNEL (restore default)");
	int err = channel_control_reset();
	if (err) {
		LOG_ERR("Channel reset failed: %d (RAM/radio may already be updated)", err);
	} else {
		LOG_INF("RF channel cleared and ESB reinitialized with default channel %u", RADIO_RF_CHANNEL);
	}
}

static void esb_remote_cmd_sens_set(void)
{
	LOG_INF("Executing remote command: SENS_SET");
	int err = sensor_calibration_set_sensitivity(received_sens_data);
	if (err) {
		LOG_ERR("Sensitivity update failed: %d", err);
	}
}

static void esb_remote_cmd_sens_reset(void)
{
	LOG_INF("Executing remote command: SENS_RESET");
	int err = sensor_calibration_reset_sensitivity();
	if (err) {
		LOG_ERR("Sensitivity reset failed: %d", err);
	}
}

static void esb_remote_cmd_sens_auto(void)
{
	LOG_INF(
		"Executing remote command: SENS_AUTO axis=%u revolutions=%u",
		received_sens_auto_axis,
		received_sens_auto_revolutions
	);
#if CONFIG_SENSOR_USE_SENS_CALIBRATION
	int err = sensor_request_calibration_sens(received_sens_auto_axis, received_sens_auto_revolutions);
	if (err) {
		LOG_ERR("Sensitivity calibration request rejected: %d", err);
	}
#else
	LOG_WRN("Sensitivity calibration not enabled");
	led_request_event(LED_OWNER_SENS, led_request_id(), led_event_id(), LED_REJECTED);
#endif
}

static void esb_remote_cmd_reset_zro(void)
{
	LOG_INF("Executing remote command: RESET_ZRO");
	int err = sensor_calibration_reset_imu();
	if (err) {
		LOG_WRN("IMU calibration reset rejected: %d", err);
	}
}

static void esb_remote_cmd_reset_acc(void)
{
	LOG_INF("Executing remote command: RESET_ACC");
	int err = sensor_calibration_reset_accel();
	if (err) {
		LOG_WRN("Accelerometer calibration reset rejected: %d", err);
	}
}

static void esb_remote_cmd_reset_bat(void)
{
	LOG_INF("Executing remote command: RESET_BAT");
	sys_reset_battery_tracker();
}

static void esb_remote_cmd_reset_tcal(void)
{
	LOG_INF("Executing remote command: RESET_TCAL");
#if CONFIG_SENSOR_USE_TCAL
	sensor_tcal_clear();
#else
	LOG_WRN("Temperature calibration not enabled");
	led_request_event(LED_OWNER_TCAL, led_request_id(), led_event_id(), LED_REJECTED);
#endif
}

static void esb_remote_cmd_tcal_auto_on(void)
{
#if CONFIG_SENSOR_USE_TCAL
	LOG_INF("Executing remote command: TCAL_AUTO_ON");
	sensor_tcal_set_auto_calibration(true);
#else
	LOG_WRN("Remote command: TCAL_AUTO_ON not supported (T-Cal disabled in config)");
	led_request_event(LED_OWNER_TCAL, led_request_id(), led_event_id(), LED_REJECTED);
#endif
}

static void esb_remote_cmd_tcal_auto_off(void)
{
#if CONFIG_SENSOR_USE_TCAL
	LOG_INF("Executing remote command: TCAL_AUTO_OFF");
	sensor_tcal_set_auto_calibration(false);
#else
	LOG_WRN("Remote command: TCAL_AUTO_OFF not supported (T-Cal disabled in config)");
	led_request_event(LED_OWNER_TCAL, led_request_id(), led_event_id(), LED_REJECTED);
#endif
}

static void esb_remote_cmd_ping(void)
{
	LOG_INF("Executing remote command: PING");
	led_identify();
}

static void esb_remote_cmd_fusion_reset(void)
{
	LOG_INF("Executing remote command: FUSION_RESET");
	sensor_request_fusion_reset(true);
}

static void esb_remote_cmd_tcal_boot_on(void)
{
#if CONFIG_SENSOR_USE_TCAL
	LOG_INF("Executing remote command: TCAL_BOOT_ON");
	sensor_boot_cal_set_enabled(true);
#else
	LOG_WRN("Remote command: TCAL_BOOT_ON not supported (T-Cal disabled in config)");
	led_request_event(LED_OWNER_TCAL, led_request_id(), led_event_id(), LED_REJECTED);
#endif
}

static void esb_remote_cmd_tcal_boot_off(void)
{
#if CONFIG_SENSOR_USE_TCAL
	LOG_INF("Executing remote command: TCAL_BOOT_OFF");
	sensor_boot_cal_set_enabled(false);
#else
	LOG_WRN("Remote command: TCAL_BOOT_OFF not supported (T-Cal disabled in config)");
	led_request_event(LED_OWNER_TCAL, led_request_id(), led_event_id(), LED_REJECTED);
#endif
}

#if CONFIG_CONNECTION_RAW_COLLECTION
static void esb_remote_cmd_data_collect_on(void)
{
	LOG_INF("Executing remote command: DATA_COLLECT_ON");
	connection_set_data_collection(true);
	test_mode_set(true);  // Prevent sleep during data collection
	led_request_event(LED_OWNER_SENSOR, led_request_id(), led_event_id(), LED_SUCCESS);
}

static void esb_remote_cmd_data_collect_off(void)
{
	LOG_INF("Executing remote command: DATA_COLLECT_OFF");
	connection_set_data_collection(false);
	test_mode_set(false);
	led_request_event(LED_OWNER_SENSOR, led_request_id(), led_event_id(), LED_SUCCESS);
}

static int esb_remote_cmd_data_collect_batch_on(void)
{
	LOG_INF("Executing remote command: DATA_COLLECT_BATCH_ON at %u Hz", executing_batch_rate_hz);
	int err = connection_set_data_collection_batch(true, executing_batch_rate_hz);
	if (err) {
		led_request_event(LED_OWNER_SENSOR, led_request_id(), led_event_id(), LED_REJECTED);
		return err;
	}
	test_mode_set(true);  // Prevent sleep during data collection
	led_request_event(LED_OWNER_SENSOR, led_request_id(), led_event_id(), LED_SUCCESS);
	return 0;
}

static void esb_remote_cmd_data_collect_batch_off(void)
{
	LOG_INF("Executing remote command: DATA_COLLECT_BATCH_OFF");
	connection_set_data_collection_batch(false, 0);
	test_mode_set(false);
	led_request_event(LED_OWNER_SENSOR, led_request_id(), led_event_id(), LED_SUCCESS);
}
#endif

static void esb_remote_cmd_ota_query_info(void)
{
	LOG_INF("Executing remote command: OTA_QUERY_INFO");
	esb_ota_handle_query_info();
}

static void esb_remote_cmd_ota_abort(void)
{
	LOG_WRN("Executing remote command: OTA_ABORT");
	esb_ota_request_abort();
}

static void esb_remote_cmd_ota_suppress(void)
{
	LOG_INF("Executing remote command: OTA_SUPPRESS (reducing poll rate)");
	connection_set_ota_suppressed(true);
	led_request_event(LED_OWNER_RADIO, led_request_id(), led_event_id(), LED_SUCCESS);
}

static void esb_remote_cmd_ota_unsuppress(void)
{
	LOG_INF("Executing remote command: OTA_UNSUPPRESS (resuming normal rate)");
	connection_set_ota_suppressed(false);
	led_request_event(LED_OWNER_RADIO, led_request_id(), led_event_id(), LED_SUCCESS);
}

static const struct esb_remote_cmd esb_remote_cmds[] = {
	{ESB_PONG_FLAG_SHUTDOWN, "SHUTDOWN", NULL},
	{ESB_PONG_FLAG_CALIBRATE, "CALIBRATE", esb_remote_cmd_calibrate},
	{ESB_PONG_FLAG_CALIBRATE_ACC, "CALIBRATE_ACC", esb_remote_cmd_calibrate_acc},
	{ESB_PONG_FLAG_MEOW, "MEOW", esb_remote_cmd_meow},
	{ESB_PONG_FLAG_SCAN, "SCAN", esb_remote_cmd_scan},
#if CONFIG_SENSOR_USE_MAGNETOMETER
	{ESB_PONG_FLAG_MAG_CLEAR, "MAG_CLEAR", esb_remote_cmd_mag_clear},
	{ESB_PONG_FLAG_MAG_CAL, "MAG_CAL", esb_remote_cmd_mag_cal},
	{ESB_PONG_FLAG_MAG_ON, "MAG_ON", esb_remote_cmd_mag_on},
	{ESB_PONG_FLAG_MAG_OFF, "MAG_OFF", esb_remote_cmd_mag_off},
	{ESB_PONG_FLAG_MAG_AUTO_ON, "MAG_AUTO_ON", esb_remote_cmd_mag_auto_on},
	{ESB_PONG_FLAG_MAG_AUTO_OFF, "MAG_AUTO_OFF", esb_remote_cmd_mag_auto_off},
#endif
	{ESB_PONG_FLAG_REBOOT, "REBOOT", esb_remote_cmd_reboot},
	{ESB_PONG_FLAG_CLEAR, "CLEAR", esb_remote_cmd_clear},
	{ESB_PONG_FLAG_DFU, "DFU", esb_remote_cmd_dfu},
	{ESB_PONG_FLAG_DFU_OTA, "DFU_OTA", esb_remote_cmd_dfu_ota},
	{ESB_PONG_FLAG_SET_CHANNEL, "SET_CHANNEL", esb_remote_cmd_set_channel},
	{ESB_PONG_FLAG_CLEAR_CHANNEL, "CLEAR_CHANNEL", esb_remote_cmd_clear_channel},
	{ESB_PONG_FLAG_SENS_SET, "SENS_SET", esb_remote_cmd_sens_set},
	{ESB_PONG_FLAG_SENS_RESET, "SENS_RESET", esb_remote_cmd_sens_reset},
	{ESB_PONG_FLAG_SENS_AUTO, "SENS_AUTO", esb_remote_cmd_sens_auto},
	{ESB_PONG_FLAG_RESET_ZRO, "RESET_ZRO", esb_remote_cmd_reset_zro},
	{ESB_PONG_FLAG_RESET_ACC, "RESET_ACC", esb_remote_cmd_reset_acc},
	{ESB_PONG_FLAG_RESET_BAT, "RESET_BAT", esb_remote_cmd_reset_bat},
	{ESB_PONG_FLAG_RESET_TCAL, "RESET_TCAL", esb_remote_cmd_reset_tcal},
	{ESB_PONG_FLAG_TCAL_AUTO_ON, "TCAL_AUTO_ON", esb_remote_cmd_tcal_auto_on},
	{ESB_PONG_FLAG_TCAL_AUTO_OFF, "TCAL_AUTO_OFF", esb_remote_cmd_tcal_auto_off},
	{ESB_PONG_FLAG_TCAL_HEATED_START, "TCAL_HEATED_START", esb_remote_cmd_tcal_heated_start},
	{ESB_PONG_FLAG_PING, "PING", esb_remote_cmd_ping},
	{ESB_PONG_FLAG_FUSION_RESET, "FUSION_RESET", esb_remote_cmd_fusion_reset},
	{ESB_PONG_FLAG_TCAL_BOOT_ON, "TCAL_BOOT_ON", esb_remote_cmd_tcal_boot_on},
	{ESB_PONG_FLAG_TCAL_BOOT_OFF, "TCAL_BOOT_OFF", esb_remote_cmd_tcal_boot_off},
	{ESB_PONG_FLAG_TCAL_ON, "TCAL_ON", esb_remote_cmd_tcal_on},
	{ESB_PONG_FLAG_TCAL_OFF, "TCAL_OFF", esb_remote_cmd_tcal_off},
	{ESB_PONG_FLAG_TDMA_ON, "TDMA_ON", esb_remote_cmd_tdma_on},
	{ESB_PONG_FLAG_TDMA_OFF, "TDMA_OFF", esb_remote_cmd_tdma_off},
	{ESB_PONG_FLAG_TEST_MODE_ON, "TEST_MODE_ON", esb_remote_cmd_test_mode_on},
	{ESB_PONG_FLAG_TEST_MODE_OFF, "TEST_MODE_OFF", esb_remote_cmd_test_mode_off},
#if CONFIG_CONNECTION_RAW_COLLECTION
	{ESB_PONG_FLAG_DATA_COLLECT_ON, "DATA_COLLECT_ON", esb_remote_cmd_data_collect_on},
	{ESB_PONG_FLAG_DATA_COLLECT_OFF, "DATA_COLLECT_OFF", esb_remote_cmd_data_collect_off},
	{ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON, "DATA_COLLECT_BATCH_ON", NULL},
	{ESB_PONG_FLAG_DATA_COLLECT_BATCH_OFF, "DATA_COLLECT_BATCH_OFF", esb_remote_cmd_data_collect_batch_off},
	{ESB_PONG_FLAG_DATA_COLLECT_METADATA, "DATA_COLLECT_METADATA", NULL},
#endif
	{ESB_PONG_FLAG_OTA_QUERY_INFO, "OTA_QUERY_INFO", esb_remote_cmd_ota_query_info},
	{ESB_PONG_FLAG_OTA_ABORT, "OTA_ABORT", esb_remote_cmd_ota_abort},
	{ESB_PONG_FLAG_OTA_SUPPRESS, "OTA_SUPPRESS", esb_remote_cmd_ota_suppress},
	{ESB_PONG_FLAG_OTA_UNSUPPRESS, "OTA_UNSUPPRESS", esb_remote_cmd_ota_unsuppress},
};

static bool esb_pong_is_legacy(uint8_t flag)
{
	/* The two allocated legacy command ranges exclude reserved flags. */
	return flag <= ESB_PONG_FLAG_WOM_CONTROL
		|| (flag >= ESB_PONG_FLAG_OTA_QUERY_INFO
		    && flag <= ESB_PONG_FLAG_DATA_COLLECT_METADATA);
}

static const char *esb_remote_cmd_name(uint8_t flag)
{
	for (size_t i = 0; i < ARRAY_SIZE(esb_remote_cmds); i++) {
		if (esb_remote_cmds[i].flag == flag) {
			return esb_remote_cmds[i].name;
		}
	}
	return "UNKNOWN";
}

/* ── OTA packet queue (ISR → thread) ─────────────────────────────
 * OTA packets received in ESB ISR are queued here and processed
 * in the connection thread where flash/logging is safe. */
#define OTA_RX_QUEUE_SIZE 16
static struct {
	uint8_t data[CONFIG_ESB_MAX_PAYLOAD_LENGTH];
	uint8_t length;
} ota_rx_queue[OTA_RX_QUEUE_SIZE];
static volatile uint8_t ota_rx_head;
static volatile uint8_t ota_rx_tail;

// Server time synchronization for TDMA scheduling (using ticks)
static bool server_time_synced = false;

// Server time synchronization
/*
 * Network (TDMA) tick domain conversion. The TDMA network runs at 32768 Hz
 * on all nodes. On nRF52 the kernel tick is also 32768 Hz, so kernel and
 * network ticks are the same domain. On nRF54L the kernel runs at 31250 Hz
 * (GRTC 1 MHz / 32): kernel ticks advance 4.88% slower than network ticks,
 * so any sync-path stamp kept in kernel ticks injects a 4.63% rate error
 * into the server-time extrapolation.
 *
 * Conversion takes the full 64-bit monotonic kernel time and yields 64-bit
 * network ticks. Truncating a 32-bit kernel stamp BEFORE converting makes
 * the network value jump at every kernel-tick wrap (the two 32-bit domains
 * wrap at different points, corrupting wrap-safe differences); converting
 * the 64-bit timeline first keeps truncated (wire) stamps and 32-bit
 * wrap-safe differences exact. RTT math (t1/t4 differences) stays in kernel
 * ticks where both stamps share the domain and k_ticks_to_us is
 * kernel-aware. Network ticks convert back to us/ms explicitly (32768 Hz),
 * never via kernel-tick macros.
 */
static inline uint64_t net_ticks_from_kernel64(uint64_t kernel_ticks)
{
#if defined(CONFIG_SOC_SERIES_NRF54L) || defined(CONFIG_SOC_COMPATIBLE_NRF54L)
	return (k_ticks_to_us_near64(kernel_ticks) * 32768ULL) / 1000000ULL;
#else
	return kernel_ticks;
#endif
}

static int64_t net_tick_difference(uint32_t a, uint32_t b)
{
	uint32_t difference = a - b;
	return difference < 0x80000000U ? (int64_t)difference
		: (int64_t)difference - 0x100000000LL;
}

/* Reassemble a wrapping 32-bit kernel-tick stamp onto the 64-bit monotonic
 * timeline: the nearest 64-bit value whose low 32 bits equal short_ticks.
 * Valid for stamps within +/-2^31 kernel ticks of the current time. */
static inline uint64_t kernel_ticks_extend32(uint32_t short_ticks)
{
	uint64_t now = k_uptime_ticks();
	uint64_t candidate = (now & ~(uint64_t)0xFFFFFFFFULL) | short_ticks;
	int64_t diff = (int64_t)(candidate - now);
	if (diff < -(int64_t)0x80000000LL) {
		candidate += 0x100000000ULL;
	} else if (diff > (int64_t)0x7FFFFFFFLL) {
		candidate -= 0x100000000ULL;
	}
	return candidate;
}


struct esb_sync_filter {
	uint32_t offset;
	uint32_t last_local_ticks;
	uint32_t reference_offset;
	uint32_t reference_local_ticks;
	int32_t skew_ppb;
	uint32_t update_count;
	bool initialized;
};
static struct esb_sync_filter sync_filter;

static uint32_t g_last_rx_raw_ticks = 0;
static int64_t g_last_sync_timestamp = 0;
#define TIME_SYNC_TIMEOUT_MS 15000

// Clock skew compensation (tracker vs receiver crystal frequency difference)
#define SKEW_REF_REFRESH_TICKS (60 * 32768)  // Refresh skew reference every ~60s

// Minimum RTT tracking for PONG acceptance threshold and diagnostics.
// Init to ~305µs (10 ticks): conservative estimate for clean 2Mbps ESB RTT.
static uint32_t g_min_rtt_ticks = 10;
static uint32_t g_min_rtt_age = 0; // PONGs since last min_rtt update
#define MIN_RTT_AGE_LIMIT 120      // Age out after ~120 PONGs (~2 min at 1/s)
#define MIN_RTT_CEILING   20       // Never age beyond this (conservative upper bound)

#define RECEIVER_RESTART_BACKWARD_TICKS (32768U * 2U)
static uint32_t receiver_restart_detections;
// Warm-up counter: first few PONGs use faster EMA for quick convergence
#define SYNC_WARM_UP_COUNT 5

// Track last sent packet for TX_FAILED diagnostics
struct last_tx_info {
	uint8_t type;        // First byte of payload (packet type)
	bool noack;          // noack flag
	uint8_t length;      // Packet length
	int64_t timestamp;   // When it was sent
};
static struct last_tx_info last_tx = {0};

// Meow arrays for remote meow command
static const char *meows[] = {
	"Mew", "Meww", "Meow", "Meow meow", "Mrrrp", "Mrrf", "Mreow", "Mrrrow", "Mrrr", "Purr",
	"mew", "meww", "meow", "meow meow", "mrrrp", "mrrf", "mreow", "mrrrow", "mrrr", "purr",
};
static const char *meow_punctuations[] = {".", "?", "!", "-", "~", ""};
static const char *meow_suffixes[]
	= {" :3", " :3c", " ;3", " ;3c", " x3", " x3c", " X3", " X3c", " >:3", " >:3c", " >;3", " >;3c", ""};

static void remote_print_meow(void)
{
	int64_t ticks = k_uptime_ticks();
	ticks %= ARRAY_SIZE(meows) * ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes);
	uint8_t meow = ticks / (ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes));
	ticks %= (ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes));
	uint8_t punctuation = ticks / ARRAY_SIZE(meow_suffixes);
	uint8_t suffix = ticks % ARRAY_SIZE(meow_suffixes);
	LOG_INF("%s%s%s", meows[meow], meow_punctuations[punctuation], meow_suffixes[suffix]);
}


static bool esb_remote_command_supported(uint8_t cmd)
{
	switch (cmd) {
#if !CONFIG_CONNECTION_RAW_COLLECTION
	case ESB_PONG_FLAG_DATA_COLLECT_ON:
	case ESB_PONG_FLAG_DATA_COLLECT_OFF:
	case ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON:
	case ESB_PONG_FLAG_DATA_COLLECT_BATCH_OFF:
	case ESB_PONG_FLAG_DATA_COLLECT_METADATA:
#endif
#if !CONFIG_SENSOR_USE_MAGNETOMETER
	case ESB_PONG_FLAG_MAG_ON:
	case ESB_PONG_FLAG_MAG_OFF:
	case ESB_PONG_FLAG_MAG_CLEAR:
	case ESB_PONG_FLAG_MAG_CAL:
	case ESB_PONG_FLAG_MAG_AUTO_ON:
	case ESB_PONG_FLAG_MAG_AUTO_OFF:
#endif
#if !CONFIG_CONNECTION_RAW_COLLECTION || !CONFIG_SENSOR_USE_MAGNETOMETER
		return false;
#endif
	default:
		return true;
	}
}

static int esb_remote_command_execute(uint8_t cmd)
{
	if (!esb_remote_command_supported(cmd)) {
		LOG_WRN("Remote command 0x%02X unavailable in this firmware", cmd);
		enum led_owner owner = LED_OWNER_SENSOR;
		if (cmd == ESB_PONG_FLAG_MAG_ON || cmd == ESB_PONG_FLAG_MAG_OFF
		    || cmd == ESB_PONG_FLAG_MAG_CLEAR || cmd == ESB_PONG_FLAG_MAG_CAL
		    || cmd == ESB_PONG_FLAG_MAG_AUTO_ON || cmd == ESB_PONG_FLAG_MAG_AUTO_OFF) {
			owner = LED_OWNER_MAG;
			if (cmd == ESB_PONG_FLAG_MAG_CAL) {
				cal_event_reject(CAL_KIND_MAG_MANUAL, CAL_REASON_UNSUPPORTED);
				tracker_events_notify();
			} else if (cmd == ESB_PONG_FLAG_MAG_AUTO_ON || cmd == ESB_PONG_FLAG_MAG_AUTO_OFF) {
				cal_event_reject(CAL_KIND_MAG_ONLINE, CAL_REASON_UNSUPPORTED);
				tracker_events_notify();
			}
		}
		led_request_event(owner, led_request_id(), led_event_id(), LED_REJECTED);
		return -ENOTSUP;
	}
	/* These requests may be refused. Do not acknowledge them until accepted. */
	if (cmd == ESB_PONG_FLAG_SHUTDOWN) {
		return esb_remote_cmd_shutdown();
	}
#if CONFIG_CONNECTION_RAW_COLLECTION
	if (cmd == ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON) {
		return esb_remote_cmd_data_collect_batch_on();
	}
#endif
	for (size_t i = 0; i < ARRAY_SIZE(esb_remote_cmds); i++) {
		if (esb_remote_cmds[i].flag == cmd) {
			if (esb_remote_cmds[i].fn) {
				esb_remote_cmds[i].fn();
			}
			return 0;
		}
	}
	LOG_WRN("Unknown remote command: 0x%02X", cmd);
	return -EINVAL;
}




static uint8_t tracker_id = 0;
static void set_tracker_id(uint8_t id)
{
	tracker_id = id;
}

// --- esb_write() rate logging ---
static uint32_t esb_write_calls = 0;
static uint32_t esb_write_queued = 0;
static uint32_t esb_write_dup_queued = 0;
static uint32_t esb_write_dropped = 0;
static int64_t esb_rate_last_ts = 0;

void esb_write_rate_tick(void)
{
	int64_t now = k_uptime_get();
	if (esb_rate_last_ts == 0) {
		esb_rate_last_ts = now;
	}
	esb_write_calls++;
	if (now - esb_rate_last_ts >= 5000) {
		LOG_INF(
			"esb_write rate: calls=%u/s queued=%u/s dup=%u/s drop=%u/s",
			esb_write_calls / 5,
			esb_write_queued / 5,
			esb_write_dup_queued / 5,
			esb_write_dropped / 5
		);
		esb_write_calls = 0;
		esb_write_queued = 0;
		esb_write_dup_queued = 0;
		esb_write_dropped = 0;
		esb_rate_last_ts = now;
	}
}

// ESB recovery mechanism for persistent ENOMEM errors
static uint32_t consecutive_enomem_errors = 0;
static int64_t last_enomem_time = 0;
static atomic_t tx_failed_pop_pending;
#define ENOMEM_ERROR_THRESHOLD 3    // Force recovery after N consecutive errors
#define ENOMEM_ERROR_WINDOW_MS 1000 // Reset counter if no error for this duration

static void drop_failed_tx_payload(void)
{
	int err = esb_pop_tx();

	if (err == 0) {
		LOG_DBG("Dropped failed TX payload from ESB FIFO");
	} else if (err == -EBUSY) {
		atomic_set(&tx_failed_pop_pending, 1);
		LOG_DBG("Deferring failed TX payload drop: ESB busy");
	} else if (err != -ENODATA) {
		LOG_WRN("Failed to drop failed TX payload: %d", err);
	}
}

static void drop_failed_tx_payload_if_pending(void)
{
	if (atomic_cas(&tx_failed_pop_pending, 1, 0)) {
		drop_failed_tx_payload();
	}
}

static void esb_start_queued_tx(void)
{
	int tx_ret = esb_start_tx();

	if (tx_ret != 0 && tx_ret != -EBUSY && tx_ret != -ENODATA) {
		LOG_WRN("esb_start_tx failed: %d", tx_ret);
	}
}

struct esb_sync_sample {
	uint32_t local_ticks;
	uint32_t offset;
};

static int64_t esb_sync_filter_update(
	struct esb_sync_filter *state, const struct esb_sync_sample *sample
)
{
	int64_t innovation = 0;
	uint32_t prev_sync_local_ticks = state->last_local_ticks;
	state->last_local_ticks = sample->local_ticks;

	if (!state->initialized) {
		state->offset = sample->offset;
		state->reference_offset = sample->offset;
		state->reference_local_ticks = sample->local_ticks;
		state->update_count = 0;
		state->initialized = true;
	} else {
		uint32_t delta_from_ref = sample->local_ticks - state->reference_local_ticks;
		int64_t predicted_drift = (int64_t)state->skew_ppb
			* delta_from_ref / 1000000000LL;
		uint32_t predicted_offset = state->reference_offset + (uint32_t)predicted_drift;
		innovation = net_tick_difference(sample->offset, predicted_offset);
		if (innovation > 32000 || innovation < -32000) {
			state->offset = sample->offset;
			state->reference_offset = sample->offset;
			state->reference_local_ticks = sample->local_ticks;
			state->skew_ppb = 0;
		} else {
			if (delta_from_ref >= 32768) {
				int64_t total_drift = net_tick_difference(sample->offset, state->reference_offset);
				int64_t raw_skew_ppb = total_drift * 1000000000LL / delta_from_ref;
				int64_t filtered_skew = (int64_t)state->skew_ppb
					+ (raw_skew_ppb - state->skew_ppb) / 4;
				state->skew_ppb = (int32_t)CLAMP(filtered_skew, INT32_MIN, INT32_MAX);
			}
			state->update_count++;
			uint32_t delta_since_sync = sample->local_ticks - prev_sync_local_ticks;
			uint32_t predicted_current = state->offset
				+ (uint32_t)((int64_t)state->skew_ppb * delta_since_sync / 1000000000LL);
			int64_t offset_innovation = net_tick_difference(sample->offset, predicted_current);
			if (state->update_count <= SYNC_WARM_UP_COUNT) {
				state->offset = predicted_current + (uint32_t)((offset_innovation * 3 + 2) / 4);
			} else {
				state->offset = predicted_current + (uint32_t)((offset_innovation + 2) / 4);
			}
			if (delta_from_ref > SKEW_REF_REFRESH_TICKS) {
				state->reference_offset = sample->offset;
				state->reference_local_ticks = sample->local_ticks;
			}
		}
	}
	return innovation;
}


static void esb_clear_time_sync_state(void)
{
	server_time_synced = false;
	sync_filter.initialized = false;
	g_last_sync_timestamp = 0;
	g_last_rx_raw_ticks = 0;
	sync_filter.last_local_ticks = 0;
	sync_filter.offset = 0;
	sync_filter.skew_ppb = 0;
	sync_filter.reference_offset = 0;
	sync_filter.reference_local_ticks = 0;
	g_min_rtt_ticks = 10;
	sync_filter.update_count = 0;
}



uint32_t esb_get_ping_backoff_ms(void)
{
	return ping_failures >= PING_BACKOFF_LVL1_THRESHOLD ? PING_BACKOFF_LVL1_MS : 0;
}

bool clock_status = false;
/* Each preparation owns a reference through admission and queueing, so an
 * earlier TX completion cannot stop the clock beneath a waiting successor. */
static atomic_t tx_clock_users;

static void esb_release_tx_clock(bool keep_clock_warm)
{
	atomic_dec(&tx_clock_users);
	if (keep_clock_warm) {
		return;
	}
#if !defined(CONFIG_SOC_SERIES_NRF54L)
	if (esb_is_idle() && esb_conn_state != ESB_ST_PAIRING && !connection_get_data_collection()) {
		clocks_stop();
	}
#endif
}

#if defined(CONFIG_CLOCK_CONTROL_NRF)
static struct onoff_manager *clk_mgr;

/* Starters may sleep, but stop is also called by the ESB interrupt. */
static K_MUTEX_DEFINE(clocks_start_lock);
static K_SEM_DEFINE(clocks_start_sem, 0, 1);
static struct onoff_client clocks_start_client;
static int clocks_start_result;
static uint32_t clocks_stop_generation;
/* A failed release must not be mistaken for a released reference. */
static bool clocks_hf_owned;

static void clocks_start_complete(struct onoff_manager *mgr, struct onoff_client *cli,
				  uint32_t state, int res)
{
	ARG_UNUSED(mgr);
	ARG_UNUSED(cli);
	ARG_UNUSED(state);
	clocks_start_result = res;
	k_sem_give(&clocks_start_sem);
}

static uint32_t clocks_capture_generation(void)
{
	unsigned key = irq_lock();
	uint32_t generation = clocks_stop_generation;
	irq_unlock(key);
	return generation;
}

static int clocks_init(void)
{
	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr) {
		LOG_ERR("Unable to get the Clock manager");
		return -ENOTSUP;
	}

	return 0;
}

SYS_INIT(clocks_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

static int clocks_start_generation(uint32_t generation)
{
	int err = 0;
	k_mutex_lock(&clocks_start_lock, K_FOREVER);

	unsigned key = irq_lock();
	if (generation != clocks_stop_generation) {
		irq_unlock(key);
		err = -ECANCELED;
		goto out;
	}
	if (clock_status) {
		irq_unlock(key);
		goto out;
	}
	if (clocks_hf_owned) {
		/* A previous release failed; do not acquire an unrepresented ref. */
		irq_unlock(key);
		err = -EIO;
		goto out;
	}
	irq_unlock(key);

	/* The callback may run before onoff_request returns. The client and
	 * semaphore persist until notification, including across ISR completion. */
	sys_notify_init_callback(&clocks_start_client.notify, clocks_start_complete);
	err = onoff_request(clk_mgr, &clocks_start_client);
	if (err < 0) {
		LOG_ERR("Clock request failed: %d", err);
		goto out;
	}
	k_sem_take(&clocks_start_sem, K_FOREVER);
	err = clocks_start_result;
	if (err < 0) {
		LOG_ERR("Clock could not be started: %d", err);
		goto out;
	}

	key = irq_lock();
	clocks_hf_owned = true;
	if (generation != clocks_stop_generation) {
		/* Stop won while this request was in flight. Release only the
		 * successfully acquired reference, without publishing radio clocks. */
		int release_err = onoff_release(clk_mgr);
		if (release_err >= 0) {
			clocks_hf_owned = false;
		}
		irq_unlock(key);
		if (release_err < 0) {
			LOG_ERR("Canceled clock release failed: %d", release_err);
		}
		err = -ECANCELED;
		goto out;
	}

#if NRF_CLOCK_HAS_PLL
	/* MLTPAN-20: CLOCK PLL must be running for radio (nRF54L and later). */
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_PLLSTART);
#endif

#if defined(CONFIG_SOC_SERIES_NRF54L)
	/* -EALREADY still acquires a constlat reference, so every published
	 * HFXO epoch owns one matching constlat request. */
	(void)nrfx_power_constlat_mode_request();
#endif
	clock_status = true;
	irq_unlock(key);
	err = 0;
out:
	k_mutex_unlock(&clocks_start_lock);
	return err;
}

int clocks_start(void)
{
	/* Capture before waiting for another starter; a later stop wins. */
	return clocks_start_generation(clocks_capture_generation());
}

void clocks_stop(void)
{
	unsigned key = irq_lock();
	if (atomic_get(&tx_clock_users) != 0) {
		irq_unlock(key);
		return;
	}

	/* LF synthesizer uses HFXO as its source, including during startup. */
	if (IS_ENABLED(CONFIG_CLOCK_USE_LF_SYNTH)) {
		irq_unlock(key);
		LOG_DBG("HF clock kept running for LF_SYNTH");
		return;
	}

	/* Invalidate queued and in-flight starts even before they are ready. */
	++clocks_stop_generation;
	if (!clocks_hf_owned) {
		irq_unlock(key);
		return;
	}

	int err = onoff_release(clk_mgr);
	if (err >= 0) {
		clocks_hf_owned = false;
#if defined(CONFIG_SOC_SERIES_NRF54L)
		if (clock_status) {
			(void)nrfx_power_constlat_mode_free();
		}
#endif
		clock_status = false;
	}
	irq_unlock(key);

	if (err < 0) {
		LOG_ERR("Clock release failed: %d", err);
	} else {
		LOG_DBG("HF clock stop request");
	}
}

#else
BUILD_ASSERT(false, "No Clock Control driver");
#endif

static struct k_thread clocks_thread_id;
static K_THREAD_STACK_DEFINE(clocks_thread_id_stack, 512);

// Wrapper function with correct signature for k_thread_entry_t
static void clocks_start_wrapper(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg3);
	int err = clocks_start_generation((uint32_t)(uintptr_t)arg1);
	if (err == 0 && (bool)(uintptr_t)arg2) {
		clock_init_external_async();
	}
}

static void clocks_request_start_internal(uint32_t delay_us, bool init_lf)
{
	/* Snapshot at submission, rather than when the delayed worker runs. */
	uint32_t generation = clocks_capture_generation();
	k_thread_create(
		&clocks_thread_id,
		clocks_thread_id_stack,
		K_THREAD_STACK_SIZEOF(clocks_thread_id_stack),
		clocks_start_wrapper,
		(void *)(uintptr_t)generation,
		(void *)(uintptr_t)init_lf,
		NULL,
		CLOCKS_START_THREAD_PRIORITY,
		0,
		K_USEC(delay_us)
	);
}

void clocks_request_start(uint32_t delay_us)
{
	clocks_request_start_internal(delay_us, false);
}

static void esb_ping_timeout(int64_t now)
{
	bool threshold_reached = false;
	unsigned key = irq_lock();
	if (ping_pending && now >= ping_send_time
	    && now - ping_send_time > (get_ping_interval_ms() - 100)) {
		ping_pending = false;
		ping_failed = true;
		ping_success_streak = 0;
		ping_failures++;
		if (ping_failures == TX_ERROR_THRESHOLD) {
			connection_error_start_time = now;
			esb_conn_state = ESB_ST_RECOVERING;
			threshold_reached = true;
		}
	}
	irq_unlock(key);
	if (threshold_reached) {
		LOG_WRN("Ping failure threshold reached (%d failures), starting timeout timer",
			TX_ERROR_THRESHOLD);
	}
}

static void esb_receive_pong(const struct esb_payload *payload)
{
	if (payload->data[0] == ESB_PONG_TYPE) {
		// check CRC first
		uint8_t crc_calc = crc8_ccitt(0x07, payload->data, ESB_PONG_LEN - 1);
		if (payload->data[ESB_PONG_LEN - 1] != crc_calc) {
			LOG_WRN("PONG CRC mismatch");
			return;
		}
		uint8_t rx_id = payload->data[1];
		if (rx_id != tracker_id) {
			return;
		}
		uint8_t pong_flags = payload->data[7];
		/* OTA status ACK cancellation is not a PING/time-sync reply. */
		if (pong_flags == ESB_PONG_FLAG_OTA_ABORT && esb_ota_is_active()) {
			esb_ota_request_abort();
			return;
		}
		if (!ping_pending || payload->data[2] != ping_ctr_sent) {
			return;
		}
		if (pong_flags == ESB_PONG_FLAG_CHANNEL_CONFIRM) {
			uint8_t advertised = payload->data[8];
			if (!ping_channel_confirm_sent || advertised > 100
			    || payload->data[9] != ESB_CHANNEL_CONFIRM_VERSION
			    || payload->data[10] != 0 || payload->data[11] != 0) {
				return;
			}
			channel_confirm_capable = true;
			channel_legacy_peer = false;
			if (advertised != radio_channel) {
				/* IRQ publishes intent; only the idle TX owner retunes. */
				channel_confirmed = false;
				channel_redirect = advertised;
				channel_redirect_pending = true;
				channel_wait_normal = true;
				channel_found = false;
				ping_pending = false;
				return;
			}
			channel_confirmed = true;
		} else {
			if (!esb_pong_is_legacy(pong_flags)) {
				return;
			}
			if (ping_channel_confirm_sent && !channel_confirm_capable) {
				channel_legacy_peer = true;
			}
			if (!channel_confirmed && !channel_legacy_peer) {
				return;
			}
		}
		if (channel_redirect_pending) {
			return;
		}
		own_pong_time = k_uptime_get();
		own_pong_seen = true;
		channel_heard = true;
		uint8_t rx_ctr = payload->data[2];
		// set ping valid first
		ping_pending = false;
		ping_failed = false;
		ping_failures = 0;
		esb_conn_state = ESB_ST_PAIRED;
		if (get_status(SYS_STATUS_CONNECTION_ERROR)) {
			ping_success_streak++;
			if (ping_success_streak >= PING_RECOVERY_THRESHOLD) {
				set_status(SYS_STATUS_CONNECTION_ERROR, false);
				connection_error_start_time = 0;
				shutdown_requested = false;
				ping_success_streak = 0;
			}
		} else {
			ping_success_streak = 0;
		}

		uint32_t ping_rx_ticks = sys_get_be32(&payload->data[3]);

		// Find send ticks for this PONG's counter in history
		uint32_t ping_ticks_for_this_ctr = 0;
		uint32_t ping_ticks_kernel_for_this_ctr = 0;
		bool ping_history_valid = false;
		for (int i = 0; i < PING_HISTORY_SIZE; i++) {
			if (ping_history[i].valid && ping_history[i].counter == rx_ctr) {
				ping_ticks_for_this_ctr = ping_history[i].ping_ticks;
				ping_ticks_kernel_for_this_ctr = ping_history[i].ping_ticks_kernel;
				ping_history_valid = true;
				break;
			}
		}

		uint32_t rtt_us = 0;
		float pong_sens_data[3] = {0.0f, 0.0f, 0.0f};
		uint8_t pong_sens_auto_axis = 0;
		uint16_t pong_sens_auto_revolutions = 0;
		bool receiver_clock_restarted = false;
		if (pong_flags == ESB_PONG_FLAG_SENS_SET) {
			// Special case: SENS_SET command repurposes time sync bytes for data
			// Skip time sync update
			int16_t x_int = (int16_t)((payload->data[3] << 8) | payload->data[4]);
			int16_t y_int = (int16_t)((payload->data[5] << 8) | payload->data[6]);
			int16_t z_int = (int16_t)((payload->data[8] << 8) | payload->data[9]);

			pong_sens_data[0] = (float)x_int / 100.0f;
			pong_sens_data[1] = (float)y_int / 100.0f;
			pong_sens_data[2] = (float)z_int / 100.0f;

			LOG_INF(
				"Received SENS_SET data: %.2f, %.2f, %.2f",
				(double)pong_sens_data[0],
				(double)pong_sens_data[1],
				(double)pong_sens_data[2]
			);
		} else if (pong_flags == ESB_PONG_FLAG_SENS_AUTO) {
			pong_sens_auto_axis = payload->data[3];
			pong_sens_auto_revolutions
				= ((uint16_t)payload->data[4] << 8) | (uint16_t)payload->data[5];
			if (pong_sens_auto_revolutions == 0) {
				LOG_INF("Received SENS_AUTO data: axis=%u, revolutions=default", pong_sens_auto_axis);
			} else {
				LOG_INF(
					"Received SENS_AUTO data: axis=%u, revolutions=%u",
					pong_sens_auto_axis,
					pong_sens_auto_revolutions
				);
			}
		} else if (ping_history_valid) {
			uint32_t rx_difference = ping_rx_ticks - g_last_rx_raw_ticks;
			receiver_clock_restarted = sync_filter.initialized
				&& rx_difference > 0x80000000U
				&& 0U - rx_difference > RECEIVER_RESTART_BACKWARD_TICKS;
			if (receiver_clock_restarted) {
				receiver_restart_detections++;
				LOG_WRN(
					"Receiver clock restart detected old=%u new=%u count=%u",
					g_last_rx_raw_ticks,
					ping_rx_ticks,
					receiver_restart_detections
				);
				/* A restarted receiver may reuse the previous 8-bit
				 * layout epoch. Stop ordinary data until fresh channel
				 * proof, time sync and a usable NORMAL re-enable TDMA. */
				channel_wait_normal = true;
				channel_found = false;
				channel_confirmed = false;
				channel_legacy_peer = false;
				esb_clear_time_sync_state();
				esb_wom_reset();
				tdma_set_enabled(false);
				ping_failures = 0;
				connection_request_ping_resync();
				/* Do not accept this pre-proof PONG as the new
				 * schedule or execute its control payload. */
				return;
			}
			// ====================================================================
			// RTT and Server Time Offset Calculation (Reference-Point Model)
			// ====================================================================
			// In ESB, the return path (ACK) has FIXED delay regardless of
			// retransmissions. All retransmission time is on the forward path.
			//
			// No retransmit:    T1 --[air]--> T2,  T4 <--[ACK]-- T3≈T2
			// With retransmit:  T1 --[fail]--[fail]--[air]--> T2
			//                   T4 <--[ACK]-- T3≈T2
			//
			// offset = T2 - T4 (constant one-way bias cancels for TDMA)
			// ====================================================================
			/* T4 = ACK RX stamp exported by the ESB lib (radio
			 * moment). Read the wrapping uint32 once; RTT keeps
			 * the kernel-domain pair, and only the network-domain
			 * view extends it to the nearest 64-bit epoch. */
			uint32_t t4_ticks = esb_last_ack_rx_ticks;
			uint32_t t4_net_ticks =
				(uint32_t)net_ticks_from_kernel64(kernel_ticks_extend32(t4_ticks));

			// Calculate full RTT: from PING send (T1) to PONG receive (T4)
			uint32_t rtt_ticks = t4_ticks - ping_ticks_kernel_for_this_ctr;
			rtt_us = k_ticks_to_us_floor32(rtt_ticks);

			// Track minimum RTT (no-retransmission baseline)
			// with aging: if min hasn't been refreshed in
			// MIN_RTT_AGE_LIMIT PONGs, nudge it upward by 1 tick
			// to recover from anomalously low measurements.
			if (rtt_ticks > 0 && rtt_ticks <= g_min_rtt_ticks) {
				g_min_rtt_ticks = rtt_ticks;
				g_min_rtt_age = 0;
			} else {
				g_min_rtt_age++;
				if (g_min_rtt_age >= MIN_RTT_AGE_LIMIT &&
				    g_min_rtt_ticks < MIN_RTT_CEILING) {
					g_min_rtt_ticks++;
					g_min_rtt_age = 0;
					LOG_DBG("min_rtt aged up to %u ticks", g_min_rtt_ticks);
				}
			}

			// log ping and rtt
			if (rtt_us > 1000) {
				LOG_DBG(
					"PONG ok, ack rtt=%u.%03u ms (ctr=%u)",
					(unsigned)(rtt_us / 1000),
					(unsigned)(rtt_us % 1000),
					rx_ctr
				);
			} else if (rtt_us < 1000) {
				LOG_DBG("PONG ok, ack rtt=%u us (ctr=%u)", (unsigned)rtt_us, rx_ctr);
			}

			/*
			 * Adaptive RTT acceptance threshold.
			 * Accept PONGs with RTT up to 4× min_rtt (handles minor
			 * retransmissions) or 1000µs absolute floor (during min_rtt
			 * warm-up when min is unreliable).
			 */
			uint32_t rtt_threshold_us = k_ticks_to_us_floor32(g_min_rtt_ticks * 4);
			if (rtt_threshold_us < 1000) {
				rtt_threshold_us = 1000;
			}
			if (rtt_us < rtt_threshold_us) {
				// Reference-point offset: T2 - T4
				// The constant one-way delay bias is the same for
				// all trackers and cancels out in TDMA slot alignment.
				// Decoupling from min_rtt avoids noise injection when
				// min_rtt ages/updates.
				uint32_t server_offset_ticks = ping_rx_ticks - t4_net_ticks;

				g_last_rx_raw_ticks = ping_rx_ticks;
				g_last_sync_timestamp = k_uptime_get();
				struct esb_sync_sample sample = {
					.local_ticks = ping_ticks_for_this_ctr,
					.offset = server_offset_ticks,
				};
				bool initialized = sync_filter.initialized;
				uint32_t old_offset = sync_filter.offset;
				int64_t innovation = esb_sync_filter_update(&sync_filter, &sample);
				if (!initialized) {
					LOG_DBG("Server offset initialized: %u ticks", sample.offset);
				} else if (innovation > 32000 || innovation < -32000) {
					LOG_WRN(
						"Large offset jump detected (%lld ticks), resetting (old=%u new=%u)",
						(long long)innovation, old_offset, sample.offset
					);
				} else {
					LOG_DBG(
						"Offset update: innovation=%lld offset=%u skew=%d ppb (rtt=%u min=%u)",
						(long long)innovation, sync_filter.offset, sync_filter.skew_ppb,
						rtt_ticks, g_min_rtt_ticks
					);
				}

				server_time_synced = true;

			}
		} else {
			// No history found - likely too old or buffer wrapped
		}

		/* Dedicated replies never touch TDMA or command/echo state. */
		if (pong_flags == ESB_PONG_FLAG_CHANNEL_CONFIRM) {
			return;
		}
		/* NORMAL retains the legacy slot/total layout. */
		if (pong_flags == ESB_PONG_FLAG_NORMAL) {
			uint8_t tdma_slot   = payload->data[8];
			uint8_t tdma_total  = payload->data[9];
			uint8_t tdma_sticks = payload->data[10];
			uint8_t tdma_epoch  = payload->data[11];

			if (channel_wait_normal || tdma_epoch != tdma_get_config_epoch()) {
				bool valid_schedule = tdma_update_config(
					tdma_slot, tdma_total, tdma_sticks, tdma_epoch
				);
				if (valid_schedule && server_time_synced && channel_wait_normal) {
					channel_found = true;
				}
			}
		}

		if (pong_flags == ESB_PONG_FLAG_WOM_CONTROL) {
#if CONFIG_CONNECTION_RAW_COLLECTION
			metadata_echo_pending = false;
#endif
			esb_wom_receive(sys_get_be16(&payload->data[8]),
				sys_get_be16(&payload->data[10]));
#if CONFIG_CONNECTION_RAW_COLLECTION
		} else if (pong_flags == ESB_PONG_FLAG_DATA_COLLECT_METADATA) {
			received_metadata_mask = payload->data[8];
			received_metadata_chunk = payload->data[9];
			received_metadata_token = sys_get_be16(&payload->data[10]);
			connection_request_raw_metadata(
				received_metadata_mask, received_metadata_chunk, received_metadata_token);
			metadata_echo_pending = true;
#endif
		} else if (pong_flags != ESB_PONG_FLAG_NORMAL) {
			/* Real controls may supersede a pending metadata echo. */
#if CONFIG_CONNECTION_RAW_COLLECTION
			metadata_echo_pending = false;
#endif
			esb_wom_reset();
			uint16_t pong_test_rate_tps = pong_flags == ESB_PONG_FLAG_TEST_MODE_ON
				&& payload->length >= 10 ? sys_get_be16(&payload->data[8]) : 0;
#if CONFIG_CONNECTION_RAW_COLLECTION
			uint8_t pong_batch_rate_hz = pong_flags == ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON
				&& payload->length >= 9 ? payload->data[8] : 0;
#endif
			bool test_rate_changed = pong_flags == ESB_PONG_FLAG_TEST_MODE_ON
				&& pong_test_rate_tps != received_test_rate_tps;
#if CONFIG_CONNECTION_RAW_COLLECTION
			bool batch_rate_changed = pong_flags == ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON
				&& pong_batch_rate_hz != received_batch_rate_hz;
#endif
			bool channel_changed = pong_flags == ESB_PONG_FLAG_SET_CHANNEL
				&& sys_get_be32(&payload->data[8]) != received_channel_value;
			if (received_remote_command == ESB_PONG_FLAG_NORMAL
			    || test_rate_changed || channel_changed
#if CONFIG_CONNECTION_RAW_COLLECTION
			    || batch_rate_changed
#endif
			    || ((received_remote_command == acked_remote_command || remote_command_rejected)
			        && pong_flags != received_remote_command)) {
				if (channel_changed && acked_remote_command == ESB_PONG_FLAG_SET_CHANNEL) {
					acked_remote_command = ESB_PONG_FLAG_NORMAL;
				}
				received_remote_command = pong_flags;
				remote_command_rejected = false;
				remote_command_generation++;
				remote_command_receive_time = k_uptime_get();
				if (pong_flags == ESB_PONG_FLAG_SET_CHANNEL) {
					received_channel_value = ((uint32_t)payload->data[8] << 24)
						| ((uint32_t)payload->data[9] << 16)
						| ((uint32_t)payload->data[10] << 8) | payload->data[11];
				} else if (pong_flags == ESB_PONG_FLAG_SENS_SET) {
					memcpy(received_sens_data, pong_sens_data, sizeof(received_sens_data));
				} else if (pong_flags == ESB_PONG_FLAG_SENS_AUTO) {
					received_sens_auto_axis = pong_sens_auto_axis;
					received_sens_auto_revolutions = pong_sens_auto_revolutions;
				} else if (pong_flags == ESB_PONG_FLAG_TEST_MODE_ON) {
					received_test_rate_tps = pong_test_rate_tps;
#if CONFIG_CONNECTION_RAW_COLLECTION
				} else if (pong_flags == ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON) {
					received_batch_rate_hz = pong_batch_rate_hz;
#endif
				}
				LOG_INF("Remote command %s (0x%02X) received",
					esb_remote_cmd_name(pong_flags), pong_flags);
			}
		} else if (!channel_wait_normal && !channel_search) {
			/* A requested proof NORMAL did not consume the low-bit
			 * command/metadata echo. Keep it for the post-proof PING. */
#if CONFIG_CONNECTION_RAW_COLLECTION
			metadata_echo_pending = false;
#endif
			if (wom_command.response_pending
			    && wom_command.sent_token == wom_command.cached_token
			    && wom_command.sent_flag == wom_command.result_flag) {
				wom_command.response_pending = false;
				wom_command.cached = false;
			}
			if (acked_remote_command != ESB_PONG_FLAG_NORMAL
			    || !esb_remote_command_supported(received_remote_command)) {
				received_remote_command = ESB_PONG_FLAG_NORMAL;
				acked_remote_command = ESB_PONG_FLAG_NORMAL;
				remote_command_receive_time = 0;
				remote_command_rejected = false;
				remote_command_generation++;
			}
		}
	}
}

void event_handler(struct esb_evt const *event)
{
	static uint32_t tx_success_count = 0;
	static uint32_t tx_failed_count = 0;
	static uint32_t last_log_time = 0;

	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		tx_success_count++;
		// Reset ENOMEM error counter on successful transmission
		consecutive_enomem_errors = 0;
		if (esb_conn_state != ESB_ST_PAIRING && !connection_get_data_collection() && esb_is_idle()) {
#if defined(CONFIG_SOC_SERIES_NRF54L)
			/* nRF54L: keep HFCLK up while ESB is active; idle stop is
			 * handled in esb_thread after ESB_CLOCK_IDLE_STOP_MS. */
#else
			clocks_stop();
#endif
		}
		break;
	case ESB_EVENT_TX_FAILED:
		drop_failed_tx_payload();
		esb_start_queued_tx();
		tx_failed_count++;

		// Detailed packet type diagnostics for TX_FAILED
		const char *pkt_desc = "UNKNOWN";
		if (last_tx.type == 0x00) {
			pkt_desc = "device info";
		} else if (last_tx.type == 0x01) {
			pkt_desc = "packet 1";
		} else if (last_tx.type == 0x02) {
			pkt_desc = "packet 2";
		} else if (last_tx.type == 0x03) {
			pkt_desc = "status";
		} else if (last_tx.type == 0x04) {
			pkt_desc = "packet 4";
		} else if (last_tx.type == ESB_PING_TYPE) {
			pkt_desc = "PING";
		} else {
			pkt_desc = "OTHER";
		}

		LOG_DBG(
			"TX FAILED: type=%s(0x%02X) len=%u noack=%d age=%lldms attempts=%u",
			pkt_desc,
			last_tx.type,
			last_tx.length,
			last_tx.noack,
			k_uptime_get() - last_tx.timestamp,
			event->tx_attempts
		);

		// Log TX statistics every 100 failures for debugging
		uint32_t now = k_uptime_get_32();
		if (tx_failed_count % 100 == 0 || (now - last_log_time > 5000)) {
			last_log_time = now;
			uint32_t total = tx_success_count + tx_failed_count;
			uint32_t fail_rate = total > 0 ? (tx_failed_count * 100 / total) : 0;
			if (esb_conn_state == ESB_ST_PAIRING) {
				LOG_INF("TX Stats (pairing: waiting for receiver, normal): success=%u failed=%u rate=%u%%",
					tx_success_count, tx_failed_count, fail_rate);
			} else {
				LOG_INF("TX Stats: success=%u failed=%u rate=%u%%", tx_success_count, tx_failed_count, fail_rate);
			}
		}

		esb_ping_timeout(k_uptime_get());

		if (esb_conn_state != ESB_ST_PAIRING && !connection_get_data_collection() && esb_is_idle()) {
#if defined(CONFIG_SOC_SERIES_NRF54L)
			/* nRF54L: keep HFCLK up while ESB is active; idle stop is
			 * handled in esb_thread after ESB_CLOCK_IDLE_STOP_MS. */
#else
			clocks_stop();
#endif
		}
		break;
	case ESB_EVENT_RX_RECEIVED: {
		int err = 0;
		err = esb_read_rx_payload(&rx_payload);
		if (err == -ENODATA) {
			return;
		} else if (err) {
			LOG_ERR("Error while reading rx packet: %d", err);
			return;
		}
		if (!paired_addr[0]) // zero, not paired
		{
			LOG_DBG("tx: %16llX rx: %16llX", *(uint64_t *)tx_payload_pair.data, *(uint64_t *)rx_payload.data);
			if (rx_payload.length == 8) {
				if (!pair_ack_pending) {
					LOG_DBG("Ignoring unsolicited pairing response");
					break;
				}
				if (rx_payload.data[0] != tx_payload_pair.data[0]) {
					LOG_DBG(
						"Ignoring pairing response with mismatched checksum "
						"%02X",
						rx_payload.data[0]
					);
					pair_ack_pending = false;
					break;
				}
				uint64_t responder_addr = 0;
				memcpy(&responder_addr, &rx_payload.data[2], 6);
				responder_addr &= 0xFFFFFFFFFFFFULL;
				uint64_t local_addr = (*(uint64_t *)NRF_FICR->DEVICEADDR) & 0xFFFFFFFFFFFFULL;
				if (responder_addr == local_addr) {
					LOG_WRN(
						"Ignoring pairing response sourced from local device "
						"address"
					);
					pair_ack_pending = false;
					break;
				}
				memcpy(paired_addr, rx_payload.data, sizeof(paired_addr));
				pair_ack_pending = false;
			}
		} else {
			switch (rx_payload.length) {
			case ESB_PONG_LEN: {
				esb_receive_pong(&rx_payload);
			} break;
			case ESB_SENSOR_DATA_LEN: {
				// received other tracker's sensor data, likely due to shared pipe, just ignore
			} break;
			default:
#if CONFIG_CONNECTION_RAW_COLLECTION
				/* ACK payload from receiver carrying ARQ retransmit requests */
				if (rx_payload.length >= 4 &&
				    rx_payload.data[0] == RAW_ARQ_MARKER) {
					uint8_t retx_n = rx_payload.data[1];
					uint8_t max_entries = (rx_payload.length - 2) / 2;
					if (retx_n > max_entries) {
						retx_n = max_entries;
					}
					for (uint8_t i = 0; i < retx_n; i++) {
						uint16_t seq = sys_get_be16(&rx_payload.data[2 + i * 2]);
						(void)connection_request_raw_retransmit(seq);
					}
				}
				/* OTA packets from receiver (in ACK payload) —
				 * queue for deferred processing in thread context
				 * (flash ops and logging not safe in ISR) */
				else
#endif
				if (rx_payload.length >= 2 &&
					 rx_payload.data[0] >= ESB_OTA_DATA_TYPE &&
					 rx_payload.data[0] <= ESB_OTA_ACTIVATE_TYPE) {
					uint8_t next = (ota_rx_head + 1) % OTA_RX_QUEUE_SIZE;
					if (next != ota_rx_tail) {
						memcpy(ota_rx_queue[ota_rx_head].data,
						       rx_payload.data, rx_payload.length);
						ota_rx_queue[ota_rx_head].length = rx_payload.length;
						__DMB();
						ota_rx_head = next;
					}
				}
				else {
					LOG_WRN("Ignoring invalid payload length %u", rx_payload.length);
				}
			} // end of rx_payload length switch
		}
		break;
	} // end of ESB_EVENT_RX_RECEIVED
	} // end of event switch
}

/* ESB pipe addresses (base addresses big-endian; prefixes for pipes 0-7). */
static const uint8_t discovery_base_addr_0[4] = {0x62, 0x39, 0x8A, 0xF2};
static const uint8_t discovery_base_addr_1[4] = {0x28, 0xFF, 0x50, 0xB8};
static const uint8_t discovery_addr_prefix[8] = {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6};

static uint8_t base_addr_0[4], base_addr_1[4], addr_prefix[8] = {0};

int esb_initialize(bool tx)
{
	k_mutex_lock(&esb_radio_lock, K_FOREVER);
	/* Retire an earlier instance before starting a new configuration. */
	if (esb_initialized) {
		esb_deinitialize();
	}
	int err;

	struct esb_config config = ESB_DEFAULT_CONFIG;

	if (tx) {
		config.protocol = ESB_PROTOCOL_ESB_DPL;
		config.event_handler = event_handler;
#if defined(CONFIG_TDMA_DIAGNOSTICS)
		config.tx_capture_handler = radio_capture_record;
#endif
		config.bitrate = ESB_BITRATE_2MBPS;
		config.tx_output_power = CONFIG_RADIO_TX_POWER;
		config.retransmit_delay = RADIO_RETRANSMIT_DELAY;
		config.retransmit_count = 1;
		config.tx_mode = ESB_TXMODE_MANUAL_START;
		config.selective_auto_ack = true;
		config.use_fast_ramp_up = true;
	} else {
		config.protocol = ESB_PROTOCOL_ESB_DPL;
		config.mode = ESB_MODE_PRX;
		config.event_handler = event_handler;
#if defined(CONFIG_TDMA_DIAGNOSTICS)
		config.tx_capture_handler = radio_capture_record;
#endif
		config.tx_output_power = CONFIG_RADIO_TX_POWER;
		config.retransmit_delay = RADIO_RETRANSMIT_DELAY;
		config.selective_auto_ack = true;
		config.use_fast_ramp_up = true;
	}
#if defined(CONFIG_TDMA_DIAGNOSTICS)
	radio_capture_deinit();
#endif

	err = esb_init(&config);
	bool driver_initialized = err == 0;
#if defined(CONFIG_TDMA_DIAGNOSTICS)
	if (!err) {
		int capture_err = radio_capture_init();
		if (capture_err) {
			LOG_WRN("RADIO capture unavailable: %d", capture_err);
		}
	}
#endif

	if (!err) {
		// Read and apply RF channel from retained/NVS (stored value is encoded).
		uint8_t ch = esb_rf_channel_decode(retained->rf_channel);
		uint8_t configured_channel = ch == ESB_RF_CHANNEL_DEFAULT ? RADIO_RF_CHANNEL : ch;
		if (ch != ESB_RF_CHANNEL_DEFAULT) {
			LOG_INF("Restoring RF channel from NVS: %u", ch);
			err = esb_set_rf_channel(ch);
		} else {
			LOG_INF("Using default RF channel: %u", RADIO_RF_CHANNEL);
			err = esb_set_rf_channel(RADIO_RF_CHANNEL);
			if (retained->rf_channel != ESB_RF_CHANNEL_DEFAULT) {
				retained->rf_channel = ESB_RF_CHANNEL_DEFAULT;
				retained_update();
			}
		}
		if (!err) {
			radio_channel = configured_channel;
		}
	}

	if (!err) {
		err = esb_set_base_address_0(base_addr_0);
	}

	if (!err) {
		err = esb_set_base_address_1(base_addr_1);
	}

	if (!err) {
		err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
	}

	if (err) {
		/* The driver is live after esb_init even if address setup failed. */
#if defined(CONFIG_TDMA_DIAGNOSTICS)
		radio_capture_deinit();
#endif
		if (driver_initialized) {
			esb_disable();
		}
		esb_deinitialize();
		LOG_ERR("ESB initialization failed: %d", err);
		set_status(SYS_STATUS_CONNECTION_ERROR, true);
		k_mutex_unlock(&esb_radio_lock);
		return err;
	}
	LOG_INF("ESB initialized, %sX mode", tx ? "T" : "R");
	esb_initialized = true;
	channel_wait_normal = true;
	channel_redirect_pending = false;
	channel_confirmed = false;
	channel_legacy_peer = false;
	ping_channel_confirm_sent = false;
	esb_wom_reset();
	channel_found = false;
	channel_heard = false;
	ping_pending = false;
	memset(ping_history, 0, sizeof(ping_history));
	esb_clear_time_sync_state();
	tdma_set_enabled(false);
	own_pong_seen = false;
	own_pong_time = k_uptime_get();
	++radio_session_generation;
	k_mutex_unlock(&esb_radio_lock);
	return 0;
}

void esb_deinitialize(void)
{
	k_mutex_lock(&esb_radio_lock, K_FOREVER);
	++radio_session_generation;
	pair_ack_pending = false;
	if (pairing_search_active && esb_conn_state == ESB_ST_PAIRING) {
		memset(paired_addr, 0, sizeof(paired_addr));
	}
	channel_search = false;
	channel_wait_normal = true;
	channel_found = false;
	channel_heard = false;
	channel_redirect_pending = false;
	channel_confirmed = false;
	channel_legacy_peer = false;
	ping_channel_confirm_sent = false;
	esb_wom_reset();
	if (pair_provisional) {
		pair_provisional = false;
		pairing_search_active = false;
		channel_confirm_capable = false;
		memset(paired_addr, 0, sizeof(paired_addr));
		esb_conn_state = ESB_ST_PAIRING;
		led_result(pair_feedback, led_event_id(), LED_FAILED);
	}
	ping_pending = false;
	memset(ping_history, 0, sizeof(ping_history));
	esb_clear_time_sync_state();
	tdma_set_enabled(false);
	if (esb_initialized) {
		esb_initialized = false;
#if defined(CONFIG_TDMA_DIAGNOSTICS)
		radio_capture_deinit();
#endif
		/* Writers cannot enqueue across this disable/init boundary. */
		esb_disable();
	}
	esb_initialized = false;
	k_mutex_unlock(&esb_radio_lock);
}

int esb_reinitialize(void)
{
	esb_deinitialize();
	k_msleep(10);
	return esb_initialize(true);
}

inline void esb_set_addr_discovery(void)
{
	memcpy(base_addr_0, discovery_base_addr_0, sizeof(base_addr_0));
	memcpy(base_addr_1, discovery_base_addr_1, sizeof(base_addr_1));
	memcpy(addr_prefix, discovery_addr_prefix, sizeof(addr_prefix));
}

inline void esb_set_addr_paired(void)
{
	// Recreate receiver address
	uint8_t addr_buffer[16] = {0};
	for (int i = 0; i < 4; i++) {
		addr_buffer[i] = paired_addr[i + 2];
		addr_buffer[i + 4] = paired_addr[i + 2] + paired_addr[6];
	}
	for (int i = 0; i < 8; i++) {
		addr_buffer[i + 8] = paired_addr[7] + i;
	}
	for (int i = 0; i < 16; i++) {
		if (addr_buffer[i] == 0x00 || addr_buffer[i] == 0x55
			|| addr_buffer[i] == 0xAA) { // Avoid invalid addresses (see nrf datasheet)
			addr_buffer[i] += 8;
		}
	}
	memcpy(base_addr_0, addr_buffer, sizeof(base_addr_0));
	memcpy(base_addr_1, addr_buffer + 4, sizeof(base_addr_1));
	memcpy(addr_prefix, addr_buffer + 8, sizeof(addr_prefix));
}

static int esb_send_pair_step(uint8_t step)
{
	tx_payload_pair.data[1] = step;
	int err = esb_write_payload(&tx_payload_pair);
	if (err == -ENOSPC) {
		esb_flush_tx();
		err = esb_write_payload(&tx_payload_pair);
	}
	if (err) {
		LOG_ERR("Failed to queue pairing burst step %u: %d", step, err);
		return err;
	}
	err = esb_start_tx();
	if (err == -EBUSY) {
		LOG_DBG("Pairing burst step %u already pending", step);
		err = 0;
	} else if (err) {
		LOG_ERR("Failed to start pairing burst step %u: %d", step, err);
	}
	return err;
}

int esb_set_pair(uint64_t addr)
{
	// Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
	uint64_t *device_addr = (uint64_t *)NRF_FICR->DEVICEADDR;
	uint8_t buf[6] = {0};
	memcpy(buf, device_addr, 6);
	uint8_t checksum = crc8_ccitt(0x07, buf, 6);
	if (checksum == 0) {
		checksum = 8;
	}
	if ((addr & 0xFF) != checksum) {
		LOG_INF("Incorrect checksum");
		led_request_event(LED_OWNER_RADIO, led_request_id(), led_event_id(), LED_REJECTED);
		return -EINVAL;
	}
	esb_reset_pair();
	memcpy(paired_addr, &addr, sizeof(paired_addr));
	tracker_events_session_changed();
	LOG_INF("Paired");
	int err = sys_write(PAIRED_ID, retained->paired_addr, paired_addr, sizeof(paired_addr));
	led_request_event(LED_OWNER_RADIO, led_request_id(), led_event_id(),
		err ? LED_PARTIAL : LED_SUCCESS);
	return err;
}

void esb_pair(void)
{
	// Reset ping state when starting pairing
	ping_failures = 0;
	set_status(SYS_STATUS_CONNECTION_ERROR, false);
	connection_error_start_time = 0;
	shutdown_requested = false;
	ping_failed = false;
	ping_pending = false;
	// Reset time sync state
	esb_clear_time_sync_state();
	if (!paired_addr[0]) // zero, no receiver paired
	{
		LOG_INF("Pairing");
		esb_conn_state = ESB_ST_PAIRING;
		esb_set_addr_discovery();
		esb_initialize(true);
		//		timer_init(); // TODO: shouldn't be here!!!
		tx_payload_pair.noack = false;
		// Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
		uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR;
		memcpy(&tx_payload_pair.data[2], addr, 6);
		LOG_INF("Device address: %012llX", *addr & 0xFFFFFFFFFFFF);
		uint8_t checksum = crc8_ccitt(0x07, &tx_payload_pair.data[2], 6);
		if (checksum == 0) {
			checksum = 8;
		}
		LOG_INF("Checksum: %02X", checksum);
		tx_payload_pair.data[0] = checksum; // Use checksum to make sure packet is for this device
		struct led_token pairing = led_begin(LED_OWNER_RADIO, pairing_request ? pairing_request : led_request_id());
		pairing_request = 0;
		pairing_search_active = true;
		bool timeout_reported = false;
		int64_t pair_start_time = k_uptime_get();
		uint8_t pair_home = radio_channel;
		unsigned pair_index = 0;
		unsigned pair_bursts = 0;
		uint32_t pair_generation = radio_session_generation;
		while (paired_addr[0] != checksum) {
			k_mutex_lock(&esb_radio_lock, K_FOREVER);
			if (!esb_initialized || radio_user_disabled
			    || pair_generation != radio_session_generation) {
				pairing_search_active = false;
				led_result(pairing, led_event_id(), LED_FAILED);
				k_mutex_unlock(&esb_radio_lock);
				return;
			}
			if (!esb_is_idle()) {
				k_mutex_unlock(&esb_radio_lock);
				k_msleep(5);
				continue;
			}
			if (pair_bursts >= 3) {
				pair_ack_pending = false;
				esb_flush_tx();
				esb_flush_rx();
				unsigned next = (pair_index + 1) % channel_candidate_count(pair_home);
				uint8_t ch = channel_candidate(pair_home, next);
				if (esb_set_rf_channel(ch) == 0) {
					radio_channel = ch;
					pair_index = next;
				}
				pair_bursts = 0;
			}
			++pair_bursts;

#if USER_SHUTDOWN_ENABLED
			// During pairing, only use connection timeout to decide shutdown
			if (!shutdown_requested
#if CONFIG_SENSOR_USE_TCAL
				&& !sensor_tcal_get_auto_calibration()
#endif
				&& (k_uptime_get() - pair_start_time) > CONFIG_CONNECTION_TIMEOUT_DELAY) {
				LOG_WRN("Pairing timeout after %dm", CONFIG_CONNECTION_TIMEOUT_DELAY / 60000);
				shutdown_requested = sys_request_system_off() == 0;
				if (shutdown_requested && !timeout_reported) {
					led_result(pairing, led_event_id(), LED_FAILED);
					timeout_reported = true;
				}
			}
#endif
			if (paired_addr[0]) {
				LOG_INF("Incorrect checksum: %02X", paired_addr[0]);
				paired_addr[0] = 0; // Packet not for this device
			}
			esb_flush_rx();
			esb_flush_tx();
			pair_ack_pending = false; // Reset before sending

			/* Feed ESB watchdog during pairing to prevent timeout */
			watchdog_feed(WDT_CHANNEL_ESB);

			/* The radio remains active between the three burst steps. */
			atomic_inc(&tx_clock_users);
			int clock_err = clocks_start();
			if (clock_err) {
				atomic_dec(&tx_clock_users);
				LOG_ERR("Pairing clock start failed: %d", clock_err);
				pairing_search_active = false;
				led_result(pairing, led_event_id(), LED_FAILED);
				k_mutex_unlock(&esb_radio_lock);
				return;
			}

			if (esb_send_pair_step(0)) {
				atomic_dec(&tx_clock_users);
				k_mutex_unlock(&esb_radio_lock);
				k_msleep(100);
				continue;
			}
			k_msleep(20); /* Deferred receiver registration may miss the first burst. */
			pair_ack_pending = true; // Set before step 1 which expects receiver response
			if (esb_send_pair_step(1)) {
				atomic_dec(&tx_clock_users);
				pair_ack_pending = false;
				k_mutex_unlock(&esb_radio_lock);
				k_msleep(100);
				continue;
			}
			k_msleep(2);
			esb_send_pair_step(2); // "acknowledge" pairing from receiver
			k_msleep(60 + (k_cycle_get_32() % 23));
			atomic_dec(&tx_clock_users);
			k_mutex_unlock(&esb_radio_lock);
		}
		/* Pair ACK is only an address hint. Confirm on the paired pipes before
		 * publishing success or writing either address or RF channel. */
		k_mutex_lock(&esb_radio_lock, K_FOREVER);
		if (radio_user_disabled || pair_generation != radio_session_generation
		    || paired_addr[0] != checksum) {
			pairing_search_active = false;
			led_result(pairing, led_event_id(), LED_FAILED);
			k_mutex_unlock(&esb_radio_lock);
			return;
		}
		pairing_search_active = false;
		tracker_events_session_changed();
		esb_deinitialize();
		pair_feedback = pairing;
		pair_provisional = true;
		pairing_search_active = true;
		pair_confirm_deadline = k_uptime_get() + CONFIG_CONNECTION_TIMEOUT_DELAY;
		k_mutex_unlock(&esb_radio_lock);
	}
	k_mutex_lock(&esb_radio_lock, K_FOREVER);
	if (!paired_addr[0] || radio_user_disabled) {
		k_mutex_unlock(&esb_radio_lock);
		return;
	}
	LOG_INF("Tracker ID: %u", paired_addr[1]);
	uint64_t receiver_address = 0;
	memcpy(&receiver_address, &paired_addr[2], 6);
	LOG_INF("Receiver address: %012llX", receiver_address);

	connection_set_id(paired_addr[1]);
	set_tracker_id(paired_addr[1]);

	esb_set_addr_paired();
	esb_conn_state = ESB_ST_PAIRED;
	clocks_stop();
	k_mutex_unlock(&esb_radio_lock);
}

void esb_reset_pair(void)
{
	if (paired_addr[0] || esb_conn_state != ESB_ST_PAIRING) {
		esb_deinitialize(); // make sure esb is off
		esb_conn_state = ESB_ST_PAIRING;
		channel_confirm_capable = false;
		memset(paired_addr, 0, sizeof(paired_addr));
		tracker_events_session_changed();
		LOG_INF("Pairing requested");
	}
}

int esb_clear_pair(void)
{
	esb_reset_pair();
	int err = sys_write(PAIRED_ID, &retained->paired_addr, paired_addr, sizeof(paired_addr));
	LOG_INF("Pairing data reset");
	led_request_event(LED_OWNER_RADIO, led_request_id(), led_event_id(),
		err ? LED_PARTIAL : LED_SUCCESS);
	return err;
}

int esb_user_pair(void)
{
	pairing_request = led_request_id();
	esb_reset_pair();
	led_request_event(LED_OWNER_RADIO, pairing_request, led_event_id(), LED_ACCEPTED);
	return 0;
}

int esb_user_set_enabled(bool enabled)
{
	k_mutex_lock(&esb_radio_lock, K_FOREVER);
	int err = 0;
	if (enabled) {
		err = esb_reinitialize();
	} else {
		esb_deinitialize();
		/* Terminal radio-off also fences a queued or in-flight start. */
		clocks_stop();
	}
	if (!err) radio_user_disabled = !enabled;
	k_mutex_unlock(&esb_radio_lock);
	led_maintenance_publish(LED_OWNER_RADIO, radio_user_disabled);
	led_request_event(LED_OWNER_RADIO, led_request_id(), led_event_id(),
		err ? LED_FAILED : LED_SUCCESS);
	return err;
}

void esb_process_ota_rx_queue(void)
{
	while (ota_rx_tail != ota_rx_head) {
		uint8_t idx = ota_rx_tail;
		if (ota_rx_queue[idx].data[0] != ESB_OTA_DATA_TYPE) {
			LOG_WRN("OTA queue: non-data type=0x%02X len=%u",
				ota_rx_queue[idx].data[0], ota_rx_queue[idx].length);
		}
		esb_ota_process_rx_packet(ota_rx_queue[idx].data,
					  ota_rx_queue[idx].length);
		__DMB();
		ota_rx_tail = (idx + 1) % OTA_RX_QUEUE_SIZE;
	}
}

/* MANUAL_START can drain an already-active FIFO. Keep RX masked until the
 * successful queue operation and its PING identity and timestamps are visible. */
static int esb_queue_ping(const struct esb_payload *payload)
{
	unsigned key = irq_lock();
	int status = esb_write_payload(payload);
	if (status == 0) {
		uint64_t kernel_ticks = k_uptime_ticks();
		struct ping_history_entry *entry = &ping_history[ping_history_idx];
		entry->counter = payload->data[2];
		entry->ping_ticks = (uint32_t)net_ticks_from_kernel64(kernel_ticks);
		entry->ping_ticks_kernel = (uint32_t)kernel_ticks;
		entry->valid = true;
		ping_history_idx = (ping_history_idx + 1) % PING_HISTORY_SIZE;
		ping_send_time = (int64_t)k_ticks_to_ms_floor64(kernel_ticks);
		ping_ctr_sent = payload->data[2];
		ping_channel_confirm_sent = (payload->data[7] & ESB_PING_FLAG_CHANNEL_CONFIRM) != 0;
		wom_command.sent_flag = payload->data[7];
		wom_command.sent_token =
			(payload->data[7] == ESB_PING_FLAG_WOM_RESULT
			 || payload->data[7] == ESB_PING_FLAG_WOM_ERROR)
			? ((uint16_t)payload->data[10] << 8) | payload->data[11] : 0;
		ping_counter++;
		ping_failed = false;
		ping_pending = true;
	}
	irq_unlock(key);
	return status;
}

static void esb_prepare_ping(uint8_t *data)
{
	data[7] &= ~ESB_PING_FLAG_CHANNEL_CONFIRM;
	if (!channel_confirmed && !channel_legacy_peer) {
		/* Never echo an unexecuted control while asking for channel proof. */
		data[7] = ESB_PING_FLAG_CHANNEL_CONFIRM;
	}
	// Set sequence number
	data[2] = ping_counter;
	struct esb_network_clock clock;
	if (esb_get_network_clock(&clock)) {
		uint32_t server_time_ticks = clock.receiver_ticks;
		data[3] = (server_time_ticks >> 24) & 0xFF;
		data[4] = (server_time_ticks >> 16) & 0xFF;
		data[5] = (server_time_ticks >> 8) & 0xFF;
		data[6] = server_time_ticks & 0xFF;
	} else {
		LOG_DBG("Sending PING while time not synced - attempting to re-sync");
	}
	esb_wom_prepare_ping(data);
	// Calculate crc8 checksum over first 12 bytes
	uint8_t crc_calc = crc8_ccitt(0x07, data, ESB_PING_LEN - 1);
	data[ESB_PING_LEN - 1] = crc_calc;
}

static int esb_write_clocked(uint8_t *data, bool no_ack, size_t data_length)
{
	if (!esb_initialized || esb_conn_state == ESB_ST_PAIRING) {
		return -EACCES;
	}
	if (channel_wait_normal && !esb_ota_is_active()
	    && data_length > 0 && data[0] != ESB_PING_TYPE
	    && data[0] != ESB_OTA_FW_INFO_TYPE) {
		return -EAGAIN;
	}
	drop_failed_tx_payload_if_pending();
	if (data_length < 1) {
		LOG_ERR("Invalid data length %u", data_length);
		return -EINVAL;
	}

	bool is_ping = data[0] == ESB_PING_TYPE;
	bool is_raw = (data[0] >= 0x10 && data[0] <= 0x14);
	bool is_batch_raw = is_raw && connection_get_data_collection_batch();
	/* Meta (0x12) and calibration (0x14) are the host's only calibration
	 * source; keep their reliable retry path even in lossy batch mode.
	 * TDMA admission below still schedules them like batch stream data. */
	bool is_batch_stream = is_batch_raw
		&& data[0] != ESB_RAW_META_TYPE && data[0] != ESB_RAW_CAL_TYPE;
	/* Batch stream loss is intentional: an ACK timeout plus hardware retry
	 * can outlive the slot reserved for one raw packet. Keep ACK/retries for
	 * metadata, calibration, PING, and reliable single-target collection. */
	no_ack = no_ack || is_batch_stream;
	bool drop_on_fifo_full = is_batch_stream || (no_ack && !is_raw);

	struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0);
	tx_payload.pipe = 1 + (tracker_id % 7);
	tx_payload.noack = no_ack;
	tx_payload.length = data_length;

	// int64_t now = k_uptime_get();
	// Tick rate counter
	esb_write_rate_tick();

	if (is_ping) {
		esb_prepare_ping(data);
	}
	memcpy(tx_payload.data, data, data_length);

	// Record this packet for TX_FAILED diagnostics
	last_tx.type = data[0];
	last_tx.noack = no_ack;
	last_tx.length = data_length;
	last_tx.timestamp = k_uptime_get();

	/*
	 * Synchronized PING already claimed the shared own-slot frame in the
	 * connection thread. Normal NoACK claims it here. Both must queue only while
	 * ESB is idle: MANUAL_START auto-drains queued payloads once a TX chain runs.
	 */
	int64_t sync_age = is_ping && tdma_is_enabled() ? esb_get_sync_age_ms() : -1;
	if (sync_age >= 0 && sync_age <= PING_INTERVAL_MS * 10) {
		if (!esb_is_idle()) {
			tdma_note_radio_busy();
			return -EBUSY;
		}
	} else if (is_batch_raw || (no_ack && !is_raw)) {
#if CONFIG_CONNECTION_TDMA
		if (tdma_is_enabled()) {
			do {
				if (!tdma_wait_for_slot((uint8_t)data_length)) {
					return -EAGAIN;
				}
				if (esb_is_idle()) {
					break;
				}
				tdma_note_radio_busy();
			} while (esb_ready());
			if (!esb_is_idle()) {
				return -EBUSY;
			}
		} else
#endif
		{
			uint32_t jitter_us = (k_cycle_get_32() & 0x3FF) % 1000;
			if (jitter_us > 100) {
				k_usleep(jitter_us);
			}
		}
	}

	// Try to queue the packet (now inside the TDMA slot window)
	int queue_status = is_ping && data_length == ESB_PING_LEN
		? esb_queue_ping(&tx_payload) : esb_write_payload(&tx_payload);

	if (queue_status == -ENOMEM && drop_on_fifo_full) {
		esb_write_dropped++;
		if (esb_is_idle()) {
			esb_start_queued_tx();
		}
		return queue_status;
	}

	if (queue_status == -ENOMEM) {
		if (esb_is_idle()) {
			(void)esb_flush_tx();
		} else {
			k_msleep(1);
			drop_failed_tx_payload_if_pending();
		}
		queue_status = is_ping && data_length == ESB_PING_LEN
			? esb_queue_ping(&tx_payload) : esb_write_payload(&tx_payload);
	}

	// manually repeat raw IMU/mag packets for better reliability
	// Skip duplication for raw meta and calibration
	// which are sent at controlled intervals with guaranteed delivery
	if (queue_status == 0 && is_raw && !is_batch_raw
	    && data[0] != ESB_RAW_META_TYPE && data[0] != ESB_RAW_CAL_TYPE) {
		tx_payload.noack = true;
		int dup_status = esb_write_payload(&tx_payload);
		if (dup_status == 0) {
			esb_write_dup_queued++;
		}
	}
# if 0
	if (no_ack && !is_raw) {
		// manually repeat packet for noack packets for better reliability
		int dup_ret = esb_write_payload(&tx_payload);
		if (dup_ret != 0) {
			LOG_WRN("Redundant copy queue failed: %d", dup_ret);
		} else {
			esb_write_dup_queued++;
		}
		queue_status = dup_ret;
	}
#endif
	if (is_ping && queue_status == 0 && data_length == ESB_PING_LEN) {
		LOG_DBG("PING queued (ctr=%u)", (unsigned)tx_payload.data[2]);
	} else if (is_ping && queue_status != 0) {
		// PING failed to queue - this is critical!
		const char *err_str = "unknown";
		if (queue_status == -ENOMEM) {
			err_str = "ENOMEM (FIFO full)";
		} else if (queue_status == -ENOSPC) {
			err_str = "ENOSPC (FIFO full)";
		} else if (queue_status == -EACCES) {
			err_str = "EACCES (access denied)";
		} else if (queue_status == -ENODATA) {
			err_str = "ENODATA (no data available)";
		}

		// Only log if this is the first failure or every 10th failure
		if (consecutive_enomem_errors == 1 || consecutive_enomem_errors % 10 == 0) {
			LOG_ERR(
				"esb_write: PING failed to queue (ctr=%u, err=%d %s, consecutive=%u)",
				tx_payload.data[2],
				queue_status,
				err_str,
				consecutive_enomem_errors
			);
		}
	}

	// Handle -ENOMEM error (ESB in bad state) with recovery mechanism
	if (queue_status == -ENOMEM || queue_status == -ENOSPC) {
		int64_t now = k_uptime_get();

		// Reset counter if this is the first error in a while
		if (now - last_enomem_time > ENOMEM_ERROR_WINDOW_MS) {
			consecutive_enomem_errors = 0;
		}

		consecutive_enomem_errors++;
		last_enomem_time = now;

		// Try simple flush first (only if ESB is idle)
		int flush_result = esb_flush_tx();

		if (flush_result == 0) {
			// Flush succeeded
			LOG_DBG("TX FIFO flushed successfully, err_count=%u", consecutive_enomem_errors);
			consecutive_enomem_errors = 0; // Reset after successful flush
		} else if (flush_result == -EBUSY) {
			// ESB is busy transmitting, this is normal - just wait
			if (consecutive_enomem_errors >= ENOMEM_ERROR_THRESHOLD) {
				// Only log warning if we've hit threshold
				LOG_WRN("ESB TX FIFO full for %u consecutive attempts (ESB busy)", consecutive_enomem_errors);

				// Only use suspend as last resort after many failures
				if (consecutive_enomem_errors >= ENOMEM_ERROR_THRESHOLD * 2) {
					LOG_ERR("Forcing recovery after %u errors", consecutive_enomem_errors);

					// Suspend and flush
					int suspend_result = esb_suspend();
					if (suspend_result == 0 || suspend_result == -EALREADY) {
						flush_result = esb_flush_tx();
						if (flush_result == 0) {
							LOG_INF("Recovered via suspend+flush");
							consecutive_enomem_errors = 0;
						} else {
							// Complete reinitialization
							LOG_ERR("Reinitializing ESB");
							esb_deinitialize();
							k_msleep(1);
							esb_initialize(true);
							consecutive_enomem_errors = 0;
						}
					}
				}
			}
			// Wait for hardware to finish current transmission
			k_msleep(1);
		}
	} else if (queue_status == 0) {
		// Success - reset error counter
		consecutive_enomem_errors = 0;
	}

	// Log error if queue failed
	if (queue_status != 0 && consecutive_enomem_errors % 10 == 1) {
		// Only log every 10th error to reduce noise
		LOG_ERR("esb_write: failed to queue packet, err=%d (logged every 10 errors)", queue_status);
	}

	// Record last TX time
	if (queue_status == 0) {
		last_tx_time = k_uptime_get();
		esb_write_queued++;
	}

	/*
	 * In MANUAL_START mode the radio auto-drains the FIFO once started.
	 * esb_start_tx() only needs to kick the first packet; if -EBUSY,
	 * the TX chain is already running and our queued packet will be
	 * sent automatically.  No retry or recovery needed.
	 */
	if (queue_status == 0) {
		esb_start_queued_tx();
	}
	return queue_status;
}

int esb_write(uint8_t *data, bool no_ack, size_t data_length)
{
	if (!esb_initialized || esb_conn_state == ESB_ST_PAIRING) {
		return -EACCES;
	}
	k_mutex_lock(&esb_radio_lock, K_FOREVER);
	if (!esb_initialized || esb_conn_state == ESB_ST_PAIRING || radio_user_disabled) {
		k_mutex_unlock(&esb_radio_lock);
		return -EACCES;
	}
	atomic_inc(&tx_clock_users);
	int err = clocks_start();
	if (err == 0) {
		err = esb_write_clocked(data, no_ack, data_length);
	}
	esb_release_tx_clock(false);
	k_mutex_unlock(&esb_radio_lock);
	return err;
}

int esb_write_ping(uint8_t *data, bool force_resync)
{
	if (!esb_initialized || esb_conn_state == ESB_ST_PAIRING) {
		return -EACCES;
	}
	k_mutex_lock(&esb_radio_lock, K_FOREVER);
	if (!esb_initialized || esb_conn_state == ESB_ST_PAIRING || radio_user_disabled) {
		k_mutex_unlock(&esb_radio_lock);
		return -EACCES;
	}
	atomic_inc(&tx_clock_users);
	int err = clocks_start();
	if (err == 0) {
		/* Clock startup must finish before guarded admission. */
		if (!force_resync && tdma_wait_for_ping_window() == TDMA_PING_DEFERRED) {
			err = -EAGAIN;
		} else {
			err = esb_write_clocked(data, false, ESB_PING_LEN);
		}
	}
	/* Deferred PINGs keep HFXO warm for the next window without retaining
	 * ownership; TX completion or an explicit lifecycle stop can release it. */
	bool keep_clock_warm = err == -EAGAIN;
	esb_release_tx_clock(keep_clock_warm);
	k_mutex_unlock(&esb_radio_lock);
	return err;
}

/* Called only by the connection TX owner, never by an ISR/second producer.
 * Returns true while ordinary producer work must yield to rendezvous. */
bool esb_channel_search_poll(bool blocked)
{
	static int64_t ping_warning_at;
	static uint32_t ping_warning_session;

	k_mutex_lock(&esb_radio_lock, K_FOREVER);
	if (pair_provisional && k_uptime_get() >= pair_confirm_deadline) {
		esb_deinitialize(); /* Roll back the RAM-only identity and feedback. */
		pairing_search_active = false;
		k_mutex_unlock(&esb_radio_lock);
		return true;
	}
	if (ping_warning_session != radio_session_generation) {
		ping_warning_session = radio_session_generation;
		ping_warning_at = 0;
	}
	if (!esb_initialized || esb_conn_state == ESB_ST_PAIRING) {
		ping_warning_at = 0;
		k_mutex_unlock(&esb_radio_lock);
		return false;
	}
	blocked |= esb_ota_is_active() || ota_rx_head != ota_rx_tail;
	if (blocked && !channel_found) {
		/* Holds freeze retunes, not same-channel proof. Keep an outstanding
		 * proof PING alive so a pending UNSUPPRESS/ABORT can follow it. */
		unsigned key = irq_lock();
		if (channel_redirect_pending) {
			/* An advertisement held across another channel owner's exchange
			 * must be re-probed, not replayed as a deferred retune. */
			channel_redirect_pending = false;
			ping_pending = false;
		}
		irq_unlock(key);
		int64_t now = k_uptime_get();
		search_deadline = now + 300;
		search_probe_at = now + 80;
		k_mutex_unlock(&esb_radio_lock);
		return false;
	}
	if (channel_found) {
		uint8_t stored = retained->rf_channel;
		uint8_t configured = esb_rf_channel_decode(stored);
		if (configured == ESB_RF_CHANNEL_DEFAULT) {
			configured = RADIO_RF_CHANNEL;
		}
		/* Do not reinterpret an explicit same-channel setting on startup.
		 * Only pairing or an automatic relocation chooses a new encoding. */
		if (channel_confirmed && (pair_provisional || configured != radio_channel)) {
			stored = radio_channel == RADIO_RF_CHANNEL
				? ESB_RF_CHANNEL_DEFAULT : esb_rf_channel_encode(radio_channel);
		}
		int address_error = 0;
		if (pair_provisional) {
			address_error = sys_write(PAIRED_ID, retained->paired_addr, paired_addr, sizeof(paired_addr));
		}
		int channel_error = 0;
		if (channel_confirmed && retained->rf_channel != stored) {
			channel_error = sys_write(RF_CHANNEL_ID, &retained->rf_channel, &stored, sizeof(stored));
			if (channel_error < 0) {
				LOG_WRN("Recovered channel persistence failed: %d", channel_error);
			}
		}
		if (pair_provisional) {
			pair_provisional = false;
			pairing_search_active = false;
			led_result(pair_feedback, led_event_id(),
				address_error || channel_error ? LED_PARTIAL : LED_SUCCESS);
			LOG_INF("Pairing confirmed on channel %u", radio_channel);
		}
		channel_found = false;
		channel_search = false;
		channel_wait_normal = false;
		channel_heard = false;
		ping_warning_at = 0;
		set_status(SYS_STATUS_CONNECTION_ERROR, false);
		connection_error_start_time = 0;
		shutdown_requested = false;
		ping_success_streak = 0;
		k_mutex_unlock(&esb_radio_lock);
		return false;
	}
	/* PONG RX updates own_pong_time in IRQ context. Sample it with the
	 * clock so a newer PONG cannot wrap the unsigned age into a false loss. */
	unsigned age_key = irq_lock();
	int64_t now = k_uptime_get();
	uint32_t lost_ms = (uint32_t)((uint32_t)now - own_pong_time);
	irq_unlock(age_key);
	if (!channel_search && !channel_wait_normal && !channel_redirect_pending
	    && ping_failures < 3 && lost_ms < 4500) {
		ping_warning_at = 0;
		k_mutex_unlock(&esb_radio_lock);
		return false;
	}
	/* Fast probes replace ping_send_time: retain legacy loss accounting
	 * independently of their cadence, including configured shutdown. */
	uint32_t missed = lost_ms / get_ping_interval_ms();
	if (missed > ping_failures) {
		ping_failures = missed;
	}
	/* Search probes refresh ping_send_time, so neither callback failures nor
	 * pending-PONG timeouts reliably report sustained loss. This owner reports
	 * progress by elapsed time, including counter jumps and a busy radio. */
	if (ping_failures >= 3 && now >= ping_warning_at) {
		LOG_WRN("Ping failed, total failures: %u", ping_failures);
		ping_warning_at = now + 10000;
	}
	if (ping_failures >= TX_ERROR_THRESHOLD && connection_error_start_time == 0) {
		connection_error_start_time = now;
		esb_conn_state = ESB_ST_RECOVERING;
		set_status(SYS_STATUS_CONNECTION_ERROR, true);
	}
	/* MANUAL_START drains the FIFO: only idle + locked admission permits
	 * changing physical channel. No queued pose or OTA frame crosses it. */
	if (!esb_is_idle()) {
		k_mutex_unlock(&esb_radio_lock);
		return channel_search || channel_wait_normal || channel_redirect_pending;
	}
	drop_failed_tx_payload_if_pending();
	if (!channel_search) {
		search_home = radio_channel;
		search_index = 0;
		unsigned key = irq_lock();
		if (!channel_wait_normal) {
			/* Re-probe legacy peers on recovery, but never downgrade a
			 * receiver that demonstrated dedicated support this session. */
			channel_legacy_peer = false;
			channel_confirmed = false;
		}
		channel_search = true;
		channel_wait_normal = true;
		channel_heard = false;
		channel_found = false;
		ping_pending = false;
		irq_unlock(key);
		search_deadline = now + 240 + (k_cycle_get_32() % 61);
		search_probe_at = now;
		esb_flush_tx();
		esb_flush_rx();
		esb_clear_time_sync_state();
		memset(ping_history, 0, sizeof(ping_history));
		tdma_set_enabled(false);
	}
	if (channel_redirect_pending) {
		unsigned key = irq_lock();
		uint8_t target = channel_redirect;
		ping_pending = false;
		channel_found = false;
		channel_heard = false;
		channel_confirmed = false;
		irq_unlock(key);
		esb_flush_tx();
		esb_flush_rx();
		esb_clear_time_sync_state();
		memset(ping_history, 0, sizeof(ping_history));
		tdma_set_enabled(false);
		if (esb_set_rf_channel(target) != 0) {
			/* Keep the redirect pending; never publish a failed tune. */
			k_mutex_unlock(&esb_radio_lock);
			return true;
		}
		key = irq_lock();
		radio_channel = target;
		channel_redirect_pending = false;
		irq_unlock(key);
		search_deadline = now + 300;
		search_probe_at = now;
	}
	if (now >= search_deadline) {
		/* A valid own NORMAL keeps this candidate while sync converges. */
		/* Invalidate the old outstanding probe atomically with the final
		 * RX result check; a late ISR must never confirm the next channel. */
		unsigned key = irq_lock();
		bool heard = channel_heard || channel_found;
		channel_heard = false;
		if (!heard) {
			ping_pending = false;
			channel_confirmed = false;
			channel_legacy_peer = false;
		}
		irq_unlock(key);
		if (heard) {
			search_deadline = now + 1000;
		} else {
			uint8_t next_index = (search_index + 1) % channel_candidate_count(search_home);
			uint8_t candidate = channel_candidate(search_home, next_index);
			esb_flush_tx();
			esb_flush_rx();
			if (esb_set_rf_channel(candidate) == 0) {
				radio_channel = candidate;
				search_index = next_index;
			}
			esb_clear_time_sync_state();
			memset(ping_history, 0, sizeof(ping_history));
			tdma_set_enabled(false);
			search_deadline = now + 240 + (k_cycle_get_32() % 61);
			search_probe_at = now;
		}
	}
	if (now >= search_probe_at) {
		uint8_t ping[ESB_PING_LEN] = {ESB_PING_TYPE};
		ping[1] = tracker_id;
		ping[7] = esb_get_ping_ack_flag();
		esb_get_ping_request_data(&ping[8]);
		(void)esb_write_ping(ping, true);
		search_probe_at = now + 80 + (k_cycle_get_32() % 23);
	}
	k_mutex_unlock(&esb_radio_lock);
	return true;
}

void esb_channel_control_begin(void)
{
	k_mutex_lock(&esb_radio_lock, K_FOREVER);
}

void esb_channel_control_end(void)
{
	connection_request_ping_resync();
	k_mutex_unlock(&esb_radio_lock);
}

bool esb_ready(void)
{
	return esb_initialized && esb_conn_state != ESB_ST_PAIRING;
}

void esb_led_connection_facts(struct led_connection_facts *facts)
{
	unsigned key = irq_lock();
	facts->healthy = esb_initialized && esb_conn_state == ESB_ST_PAIRED && own_pong_seen
		&& !esb_ota_is_active() && !channel_search && !channel_wait_normal && ping_failures < 3
		&& (uint32_t)(k_uptime_get_32() - own_pong_time) < 4500;
	facts->radio_required = !radio_user_disabled;
	facts->paired = paired_addr[0] != 0 && !pair_provisional;
	facts->pairing = pairing_search_active;
	irq_unlock(key);
}

uint8_t esb_get_ping_ack_flag(void)
{
	if (!channel_confirmed && !channel_legacy_peer) {
		return ESB_PING_FLAG_CHANNEL_CONFIRM;
	}
#if CONFIG_CONNECTION_RAW_COLLECTION
	/* A metadata PONG can replace the normal confirmation of a prior
	 * control command; acknowledge its token before that older flag. */
	if (metadata_echo_pending) {
		return ESB_PONG_FLAG_DATA_COLLECT_METADATA;
	}
#endif
	if (wom_command.response_pending) {
		return wom_command.result_flag;
	}
	if (!esb_remote_command_supported(received_remote_command)) {
		return ESB_PONG_FLAG_NORMAL;
	}
	if (acked_remote_command != ESB_PONG_FLAG_NORMAL) {
		return acked_remote_command;
	}
	if (received_remote_command != ESB_PONG_FLAG_NORMAL) {
		return received_remote_command;
	}
	return ESB_PONG_FLAG_NORMAL;
}
void esb_get_ping_request_data(uint8_t out[4])
{
	unsigned key = irq_lock();
	uint8_t flag = esb_get_ping_ack_flag();
	memset(out, 0, 4);
	if (flag == ESB_PONG_FLAG_TEST_MODE_ON) {
		uint16_t tps = test_mode_get_target_tps();
		out[0] = (uint8_t)(tps >> 8);
		out[1] = (uint8_t)tps;
#if CONFIG_CONNECTION_RAW_COLLECTION
	} else if (flag == ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON) {
		out[0] = (uint8_t)connection_get_data_collection_batch_rate();
	} else if (flag == ESB_PONG_FLAG_DATA_COLLECT_METADATA) {
		out[0] = received_metadata_mask;
		out[1] = received_metadata_chunk;
		out[2] = (uint8_t)(received_metadata_token >> 8);
		out[3] = (uint8_t)received_metadata_token;
#endif
	} else if (flag == ESB_PING_FLAG_WOM_RESULT
	           || flag == ESB_PING_FLAG_WOM_ERROR) {
		out[0] = wom_command.result >> 8;
		out[1] = wom_command.result;
		out[2] = wom_command.cached_token >> 8;
		out[3] = wom_command.cached_token;
	}
	irq_unlock(key);
}

static uint32_t esb_project_network_ticks(
	uint32_t local, uint32_t reference, uint32_t offset, int32_t skew_ppb
)
{
	uint32_t elapsed = local - reference;
	int64_t correction = (int64_t)skew_ppb * elapsed / 1000000000LL;
	/* Keep the receiver's low32 epoch; zero is a valid synchronized tick. */
	return local + offset + (uint32_t)correction;
}

bool esb_get_status_clock(uint32_t *local_ticks, uint32_t *network_ticks)
{
	/* ESB RX publishes these fields in interrupt context. Capture the kernel
	 * timestamp and all eligibility/estimator state together; do the 64-bit
	 * conversions and skew arithmetic after releasing the interrupt lock. */
	unsigned key = irq_lock();
	uint64_t kernel_ticks = k_uptime_ticks();
	bool synced = server_time_synced && esb_conn_state == ESB_ST_PAIRED
		&& !channel_wait_normal && !channel_redirect_pending
		&& get_status(SYS_STATUS_CONNECTION_ERROR) == 0;
	int64_t max_age_ms = tdma_status_clock_max_age_ms();
	int64_t last_sync_ms = g_last_sync_timestamp;
	uint32_t offset = sync_filter.offset;
	uint32_t reference_ticks = sync_filter.last_local_ticks;
	int32_t skew_ppb = sync_filter.skew_ppb;
	irq_unlock(key);

	uint32_t local = (uint32_t)net_ticks_from_kernel64(kernel_ticks);
	*local_ticks = local;
	*network_ticks = local;
	int64_t age_ms = (int64_t)k_ticks_to_ms_floor64(kernel_ticks) - last_sync_ms;
	if (!synced || max_age_ms < 0 || age_ms < 0 || age_ms > max_age_ms) {
		return false;
	}

	*network_ticks = esb_project_network_ticks(local, reference_ticks, offset, skew_ppb);
	return true;
}

bool esb_get_network_clock(struct esb_network_clock *clock)
{
	unsigned key = irq_lock();
	uint64_t kernel_ticks = k_uptime_ticks();
	bool synced = server_time_synced;
	int64_t last_sync_ms = g_last_sync_timestamp;
	uint32_t offset = sync_filter.offset;
	uint32_t reference_ticks = sync_filter.last_local_ticks;
	int32_t skew_ppb = sync_filter.skew_ppb;
	irq_unlock(key);

	int64_t age_ms = (int64_t)k_ticks_to_ms_floor64(kernel_ticks) - last_sync_ms;
	if (!synced || age_ms < 0) {
		return false;
	}
	if (age_ms > TIME_SYNC_TIMEOUT_MS) {
		bool expired = false;
		key = irq_lock();
		if (server_time_synced && g_last_sync_timestamp == last_sync_ms) {
			server_time_synced = false;
			expired = true;
		}
		irq_unlock(key);
		if (expired) {
			LOG_WRN("Time sync timeout: %lld ms since last sync, clearing sync state", (long long)age_ms);
		}
		return false;
	}
	uint32_t local = (uint32_t)net_ticks_from_kernel64(kernel_ticks);
	clock->receiver_ticks = esb_project_network_ticks(local, reference_ticks, offset, skew_ppb);
	clock->local_kernel_ticks = kernel_ticks;
	clock->sync_age_ms = age_ms;
	return true;
}

int64_t esb_get_sync_age_ms(void)
{
	struct esb_network_clock clock;
	return esb_get_network_clock(&clock) ? clock.sync_age_ms : -1;
}

static void esb_thread(void)
{
#if CONFIG_CONNECTION_OVER_HID
	int64_t start_time = k_uptime_get();
#endif

	/* Register ESB thread with watchdog */
	if (watchdog_register_thread(WDT_CHANNEL_ESB, 0) < 0) {
		LOG_ERR("ESB watchdog registration failed");
		sys_reboot(SYS_REBOOT_COLD);
		return;
	}

	// Read paired address from retained
	memcpy(paired_addr, retained->paired_addr, sizeof(paired_addr));

	if (IS_ENABLED(CONFIG_CLOCK_USE_LF_SYNTH) && !IS_ENABLED(CONFIG_CLOCK_USE_LFXO)) {
		/* LF_SYNTH must not be selected before the HFXO request succeeds. */
		clocks_request_start_internal(0, true);
	} else {
		/* Do not submit a late boot start after terminal radio-off. */
		k_mutex_lock(&esb_radio_lock, K_FOREVER);
		if (!radio_user_disabled) {
			clocks_request_start(0);
		}
		k_mutex_unlock(&esb_radio_lock);
		/* XTAL/other LF startup is independent of HFXO readiness. */
		clock_init_external_async();
	}

	while (1) {
#if CONFIG_CONNECTION_OVER_HID
		if (!radio_user_disabled && esb_conn_state == ESB_ST_PAIRING && get_status(SYS_STATUS_USB_CONNECTED) == false
			&& k_uptime_get() - 750 > start_time) // only automatically enter pairing while not
												  // potentially communicating by usb
#else
		if (!radio_user_disabled && esb_conn_state == ESB_ST_PAIRING)
#endif
		{
			esb_pair();
			k_mutex_lock(&esb_radio_lock, K_FOREVER);
			if (!radio_user_disabled && paired_addr[0] && esb_conn_state != ESB_ST_PAIRING) {
				esb_initialize(true);
			}
			k_mutex_unlock(&esb_radio_lock);
		}
#if defined(CONFIG_SOC_SERIES_NRF54L)
		/* Stop HFCLK only after a longer idle period without any ESB TX.
		 * Frequent XO/PLL restarts are unreliable on nRF54L (anomaly 20). */
		unsigned key = irq_lock();
		bool clock_ready = clock_status;
		irq_unlock(key);
		if (clock_ready && esb_conn_state != ESB_ST_PAIRING && !connection_get_data_collection() &&
		    esb_is_idle() && last_tx_time != 0 &&
		    (k_uptime_get() - last_tx_time) > ESB_CLOCK_IDLE_STOP_MS) {
			clocks_stop();
		}
#endif

		// Check for shutdown timeout if connection errors persist
		if (ping_failures >= TX_ERROR_THRESHOLD && !test_mode_get()) {
#if CONFIG_CONNECTION_OVER_HID
			// only raise error while not potentially communicating by usb
			if (get_status(SYS_STATUS_CONNECTION_ERROR) == false && get_status(SYS_STATUS_USB_CONNECTED) == false && get_status(SYS_STATUS_CALIBRATION_RUNNING) == false)
#else
			if (get_status(SYS_STATUS_CONNECTION_ERROR) == false && get_status(SYS_STATUS_CALIBRATION_RUNNING) == false)
#endif
				set_status(SYS_STATUS_CONNECTION_ERROR, true);
#if USER_SHUTDOWN_ENABLED
			if (!shutdown_requested && connection_error_start_time > 0
				&& !connection_get_ota_suppressed()
#if CONFIG_SENSOR_USE_TCAL
				&& !sensor_tcal_get_auto_calibration()
#endif
				&& k_uptime_get() - connection_error_start_time
					   > CONFIG_CONNECTION_TIMEOUT_DELAY && get_status(SYS_STATUS_CALIBRATION_RUNNING) == false) // shutdown if receiver is not detected and not in calibrating
			{
				LOG_WRN("No response from receiver in %dm", CONFIG_CONNECTION_TIMEOUT_DELAY / 60000);
				shutdown_requested = sys_request_system_off() == 0;
			}
#endif
		}
		int64_t now_idle = k_uptime_get();
		esb_wom_process();

		bool command_pending = (channel_confirmed || channel_legacy_peer) && !channel_redirect_pending
			&& received_remote_command != ESB_PONG_FLAG_NORMAL
#if CONFIG_CONNECTION_RAW_COLLECTION
			&& received_remote_command != ESB_PONG_FLAG_DATA_COLLECT_METADATA
#endif
			&& !(remote_command_rejected && !esb_remote_command_supported(received_remote_command))
			&& remote_command_receive_time > 0;
		if (command_pending) {
			if (received_remote_command == ESB_PONG_FLAG_TEST_MODE_ON) {
				command_pending = acked_remote_command != ESB_PONG_FLAG_TEST_MODE_ON
					|| received_test_rate_tps != acked_test_rate_tps;
#if CONFIG_CONNECTION_RAW_COLLECTION
			} else if (received_remote_command == ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON) {
				command_pending = acked_remote_command != ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON
					|| received_batch_rate_hz != acked_batch_rate_hz;
#endif
			} else {
				command_pending = received_remote_command != acked_remote_command;
			}
		}
		if (command_pending) {
			bool is_ota_cmd = received_remote_command >= ESB_PONG_FLAG_OTA_QUERY_INFO
				&& received_remote_command <= ESB_PONG_FLAG_OTA_UNSUPPRESS;
			if (is_ota_cmd || now_idle - remote_command_receive_time >= 100) {
				unsigned key = irq_lock();
				uint8_t executing_command = received_remote_command;
				uint32_t executing_generation = remote_command_generation;
				executing_shutdown_generation = executing_generation;
				if (executing_command == ESB_PONG_FLAG_TEST_MODE_ON) executing_test_rate_tps = received_test_rate_tps;
#if CONFIG_CONNECTION_RAW_COLLECTION
				else if (executing_command == ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON) executing_batch_rate_hz = received_batch_rate_hz;
#endif
				else if (executing_command == ESB_PONG_FLAG_SET_CHANNEL) executing_channel_value = received_channel_value;
				irq_unlock(key);
				int err = esb_remote_command_execute(executing_command);
				key = irq_lock();
				if (executing_generation == remote_command_generation) {
					/* A batch rate refusal is final for this request. The PING
					 * still echoes the actual rate, never the rejected rate. */
					bool consumed = err == 0 || (executing_command == ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON && err == -EBUSY);
					remote_command_rejected = !consumed;
					if (consumed) {
						acked_remote_command = executing_command;
						if (executing_command == ESB_PONG_FLAG_TEST_MODE_ON) acked_test_rate_tps = executing_test_rate_tps;
#if CONFIG_CONNECTION_RAW_COLLECTION
						else if (executing_command == ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON) acked_batch_rate_hz = executing_batch_rate_hz;
#endif
					}
				}
				irq_unlock(key);
			}
		}

		esb_ping_timeout(now_idle);

		/* Feed watchdog at end of each loop iteration */
		watchdog_feed(WDT_CHANNEL_ESB);

		k_msleep(100);
	}
}
