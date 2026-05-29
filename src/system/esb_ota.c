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
/*
 * ESB OTA Firmware Update – Tracker Side
 *
 * Receives firmware data over ESB, writes to internal flash, validates,
 * and activates via bootloader settings page update.
 *
 * Flash layout (nRF52840 with Adafruit bootloader):
 *   0x00000 - 0x00FFF  MBR (4 KB)
 *   0x01000 - 0xE9FFF  Application (CONFIG_FLASH_LOAD_OFFSET = 0x1000)
 *   0xEA000 - 0xF3FFF  App Data / NVS
 *   0xF4000 - 0xFFFFF  Bootloader + Settings
 *
 * The new firmware overwrites the application region starting at
 * FLASH_LOAD_OFFSET. This is a single-bank in-place update – power loss
 * during flash write will brick the device (recoverable via UF2 bootloader).
 *
 * Transport architecture:
 *   The tracker is ESB PTX (transmitter), the receiver is PRX (receiver).
 *   The receiver can only send data to the tracker via ACK payloads.
 *   During OTA mode, the tracker sends frequent OTA_STATUS poll packets
 *   and the receiver responds with OTA_DATA in the ACK payload (up to 48 bytes).
 *   This is the same mechanism used for PING/PONG, just at higher frequency.
 */

#include "esb_ota.h"
#include "globals.h"
#include "build_defines.h"
#include "connection/esb.h"
#include "connection/connection.h"
#include "system/power.h"
#include "system/watchdog.h"
#include "sensor/sensor.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_nvmc.h>
#include <hal/nrf_radio.h>
#include <string.h>

LOG_MODULE_REGISTER(esb_ota, LOG_LEVEL_INF);

/* ── Flash configuration ─────────────────────────────────────────── */

#ifndef CONFIG_FLASH_LOAD_OFFSET
#error "CONFIG_FLASH_LOAD_OFFSET must be defined by board defconfig"
#endif

/* MBR occupies 0x0-0x1000 and must not be overwritten via OTA */
#define OTA_FLASH_BASE      MAX(CONFIG_FLASH_LOAD_OFFSET, 0x1000)
#define OTA_FLASH_PAGE_SIZE  4096

/*
 * Flash partition layout and OTA engine selection.
 * nRF52840: uses staging area (upper flash), needs half of app region free
 * nRF52833: uses RAM engine for in-place writes (no staging needed)
 * Other SoCs: OTA disabled at runtime
 */
#if CONFIG_SOC_NRF52840
#define OTA_FLASH_END        0xEE000 /* End of app partition (before NVS) */
#define OTA_USE_RAM_ENGINE   0
#define BOOTLOADER_SETTINGS_ADDR BOOTLOADER_SETTINGS_ADDR_52840
#define OTA_SUPPORTED        1
#elif CONFIG_SOC_NRF52833
#define OTA_FLASH_END        0x74000
#define OTA_USE_RAM_ENGINE   1  /* Use RAM engine for in-place writes */
#define BOOTLOADER_SETTINGS_ADDR BOOTLOADER_SETTINGS_ADDR_52833
#define OTA_SUPPORTED        1
#else
#define OTA_FLASH_END        (OTA_FLASH_BASE + 256 * 1024)
#define OTA_USE_RAM_ENGINE   0
#define BOOTLOADER_SETTINGS_ADDR 0
#define OTA_SUPPORTED        0
#endif

/* ── Board target string (set at compile time) ───────────────────── */

#ifndef CONFIG_BOARD_TARGET
/* Zephyr sets CONFIG_BOARD_TARGET as "board/soc/variant" */
#define BOARD_TARGET_STRING CONFIG_BOARD
#else
#define BOARD_TARGET_STRING CONFIG_BOARD_TARGET
#endif

/* ── OTA State ───────────────────────────────────────────────────── */

enum ota_state {
	OTA_STATE_IDLE,
	OTA_STATE_ERASING,
	OTA_STATE_READY,
	OTA_STATE_RECEIVING,
	OTA_STATE_VERIFYING,
	OTA_STATE_ACTIVATING,
	OTA_STATE_COMPLETE,
	OTA_STATE_ERROR,
};

static struct {
	enum ota_state state;
	uint32_t image_size;
	uint32_t image_crc32;
	uint16_t total_packets;
	uint16_t next_expected_seq;
	uint32_t bytes_written;
	uint32_t last_page_erased;      /* Last flash page address that was erased */
	int64_t  last_data_time;        /* Timestamp of last received data */
	int64_t  last_status_time;      /* Timestamp of last status report */
	uint8_t  error_code;
	char     expected_board[OTA_BOARD_TARGET_MAX];

	/* Page write buffer: accumulate data until a full page (4 KB) is ready */
	uint8_t  page_buf[OTA_FLASH_PAGE_SIZE];
	uint16_t page_buf_offset;       /* Current position in page_buf */
	uint32_t page_buf_flash_addr;   /* Flash address this buffer maps to (staging area) */

	/* Staging area: OTA data is first written to a staging area in upper flash,
	 * then copied to the final location with interrupts disabled + reset. */
	uint32_t staging_base;          /* Start of staging area in flash */
	uint32_t target_flash_base;     /* Destination base address for the new firmware */
} ota;

static const struct device *flash_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));

/* Prepared bootloader settings (computed in thread context, written by RAM copier) */
static struct bootloader_settings prepared_bl_settings;
static bool bl_settings_prepared;

/* ── Forward declarations ────────────────────────────────────────── */

static void ota_send_status(void);
static void ota_send_fw_info(void);
static void ota_erase_page(uint32_t addr);
static void ota_write_page(uint32_t addr, const uint8_t *data, size_t len);
static int  ota_flush_page_buf(void);
static void ota_write_first_page_and_reset(void);
static int  ota_update_bootloader_settings(void);
static int  ota_prepare_bootloader_settings(void);
static uint32_t ota_compute_crc32(uint32_t addr, uint32_t size);
static uint16_t ota_compute_crc16_nordic(uint32_t addr, uint32_t size);

#if OTA_USE_RAM_ENGINE
/* External: bare-metal RAM OTA engine (defined in ota_ram_engine.c) */
struct ota_ram_engine_params;
extern void ota_ram_engine(const struct ota_ram_engine_params *p);
static void ota_launch_ram_engine(void);
#endif

/* ── Public API ──────────────────────────────────────────────────── */

bool esb_ota_is_active(void)
{
	return ota.state != OTA_STATE_IDLE;
}

uint8_t esb_ota_get_status(void)
{
	switch (ota.state) {
	case OTA_STATE_IDLE:      return OTA_STATUS_IDLE;
	case OTA_STATE_ERASING:   return OTA_STATUS_READY; /* Still show ready during erase */
	case OTA_STATE_READY:     return OTA_STATUS_READY;
	case OTA_STATE_RECEIVING: return OTA_STATUS_RECEIVING;
	case OTA_STATE_VERIFYING: return ota.error_code ? ota.error_code : OTA_STATUS_RECEIVING;
	case OTA_STATE_ACTIVATING: return OTA_STATUS_ACTIVATING;
	case OTA_STATE_COMPLETE:  return OTA_STATUS_COMPLETE;
	case OTA_STATE_ERROR:     return ota.error_code;
	default:                  return OTA_STATUS_ERROR;
	}
}

void esb_ota_handle_query_info(void)
{
	LOG_INF("OTA: Firmware info requested");
	ota_send_fw_info();
}

int esb_ota_handle_begin(const uint8_t *data, size_t len)
{
	if (len < OTA_BEGIN_PACKET_SIZE) {
		LOG_ERR("OTA BEGIN: packet too short (%zu)", len);
		return -EINVAL;
	}

	/* nRF5 OpenDFU bootloader sets ACL write-protection on the app region,
	 * preventing in-place flash copy.  Reject OTA early. */
#if CONFIG_BOARD_HAS_NRF5_BOOTLOADER && !CONFIG_BUILD_OUTPUT_UF2
	LOG_ERR("OTA: blocked — nRF5 OpenDFU bootloader ACL write-protects "
		"app region. Use DFU/SWD to update this device.");
	ota.state = OTA_STATE_ERROR;
	ota.error_code = OTA_STATUS_ERROR;
	ota_send_status();
	return -ENOTSUP;
#endif

#if !OTA_SUPPORTED
	LOG_ERR("OTA: not supported on this SoC");
	ota.state = OTA_STATE_ERROR;
	ota.error_code = OTA_STATUS_ERROR;
	ota_send_status();
	return -ENOTSUP;
#endif

	/* Reject duplicate BEGIN if already in progress */
	if (ota.state != OTA_STATE_IDLE && ota.state != OTA_STATE_ERROR &&
	    ota.state != OTA_STATE_COMPLETE) {
		LOG_WRN("OTA BEGIN: session already active (state=%d), ignoring", ota.state);
		ota_send_status();
		return -EALREADY;
	}

	/* Validate CRC-8 */
	uint8_t pkt_crc = data[63];
	uint8_t calc_crc = esb_ota_crc8(data, 63);
	if (pkt_crc != calc_crc) {
		LOG_ERR("OTA BEGIN: CRC mismatch (got 0x%02X, expected 0x%02X)", pkt_crc, calc_crc);
		return -EINVAL;
	}

	/* Parse parameters */
	uint32_t image_size = sys_get_le32(&data[2]);
	uint32_t image_crc32 = sys_get_le32(&data[6]);
	uint16_t total_packets = sys_get_be16(&data[10]);
	uint8_t protocol_ver = data[12];
	const char *board_target = (const char *)&data[13];
	uint32_t flash_base = (uint32_t)sys_get_be16(&data[61]) << 12; /* Page-aligned */

	LOG_INF("OTA BEGIN: size=%u, crc32=0x%08X, packets=%u, proto=%u, board=%s, base=0x%X",
		image_size, image_crc32, total_packets, protocol_ver, board_target, flash_base);

	/* Validate protocol version */
	if (protocol_ver != OTA_PROTOCOL_VERSION) {
		LOG_ERR("OTA BEGIN: unsupported protocol version %u", protocol_ver);
		ota.state = OTA_STATE_ERROR;
		ota.error_code = OTA_STATUS_ERROR;
		ota_send_status();
		return -ENOTSUP;
	}

	/* Validate image size.
	 * Use theoretical max (flash end - MBR) as the early check.
	 * The precise bounds check (flash_base + image_size > OTA_FLASH_END) and
	 * the staging overlap check (nRF52840) below catch the real limits. */
	if (image_size == 0 || image_size > (OTA_FLASH_END - 0x1000)) {
		LOG_ERR("OTA BEGIN: invalid image size %u (max %u)", image_size, OTA_FLASH_END - 0x1000);
		ota.state = OTA_STATE_ERROR;
		ota.error_code = OTA_STATUS_SIZE_ERROR;
		ota_send_status();
		return -EINVAL;
	}

	/* Validate board target */
	const char *my_board = BOARD_TARGET_STRING;
	if (strncmp(board_target, my_board, OTA_BOARD_TARGET_MAX) != 0) {
		LOG_ERR("OTA BEGIN: board mismatch (got '%s', expected '%s')", board_target, my_board);
		ota.state = OTA_STATE_ERROR;
		ota.error_code = OTA_STATUS_BOARD_MISMATCH;
		ota_send_status();
		return -EINVAL;
	}

	/* Validate flash base address (if provided, 0 = don't check).
	 * Allow lower or equal base (e.g., SoftDevice → no-SoftDevice transition).
	 * Block higher base: target firmware expects SoftDevice that isn't present. */
	if (flash_base != 0 && flash_base < 0x1000) {
		LOG_ERR("OTA BEGIN: flash base 0x%X below MBR (minimum 0x1000)", flash_base);
		ota.state = OTA_STATE_ERROR;
		ota.error_code = OTA_STATUS_SIZE_ERROR;
		ota_send_status();
		return -EINVAL;
	}
	if (flash_base != 0 && flash_base > OTA_FLASH_BASE) {
		LOG_ERR("OTA BEGIN: flash base 0x%X > running base 0x%X — "
			"target firmware requires SoftDevice not present",
			flash_base, OTA_FLASH_BASE);
		ota.state = OTA_STATE_ERROR;
		ota.error_code = OTA_STATUS_SIZE_ERROR;
		ota_send_status();
		return -EINVAL;
	}
	if (flash_base != 0 && (flash_base + image_size) > OTA_FLASH_END) {
		LOG_ERR("OTA BEGIN: image at 0x%X + %u exceeds flash end 0x%X",
			flash_base, image_size, OTA_FLASH_END);
		ota.state = OTA_STATE_ERROR;
		ota.error_code = OTA_STATUS_SIZE_ERROR;
		ota_send_status();
		return -EINVAL;
	}

	/* Validate flash device */
	if (!device_is_ready(flash_dev)) {
		LOG_ERR("OTA BEGIN: flash device not ready");
		ota.state = OTA_STATE_ERROR;
		ota.error_code = OTA_STATUS_FLASH_ERROR;
		ota_send_status();
		return -EIO;
	}

	/* Initialize OTA state */
	memset(&ota, 0, sizeof(ota));
	ota.image_size = image_size;
	ota.image_crc32 = image_crc32;
	ota.total_packets = total_packets;
	ota.next_expected_seq = 0;
	ota.bytes_written = 0;
	ota.last_page_erased = 0;
	ota.page_buf_offset = 0;
	ota.target_flash_base = (flash_base != 0) ? flash_base : OTA_FLASH_BASE;
	strncpy(ota.expected_board, board_target, OTA_BOARD_TARGET_MAX - 1);

	if (ota.target_flash_base != OTA_FLASH_BASE) {
		LOG_WRN("OTA: Cross-base update: running at 0x%X, target at 0x%X",
			OTA_FLASH_BASE, ota.target_flash_base);
	}

#if OTA_USE_RAM_ENGINE
	/*
	 * RAM engine mode: no staging area needed.
	 * The RAM engine will receive data directly via bare-metal ESB
	 * and write to the target flash address in-place.
	 */
	ota.staging_base = 0; /* Not used in RAM engine mode */
	ota.state = OTA_STATE_RECEIVING;
	ota.last_data_time = k_uptime_get();

	LOG_WRN("OTA: Using RAM engine for in-place update (%u bytes)", image_size);
	ota_send_status();

	/* Brief delay for status to be transmitted */
	k_msleep(100);

	/* Prepare bootloader settings before entering RAM engine */
	/* Note: CRC16 will be computed by the RAM engine after writing all data,
	 * so we can't prepare settings here. The RAM engine handles it. */

	/* Launch RAM engine (never returns) */
	ota_launch_ram_engine();

	/* Should not reach here */
	return 0;

#else /* !OTA_USE_RAM_ENGINE */
	/*
	 * Staging area: write OTA data to upper flash first, then copy to final
	 * location with IRQs disabled. Page-align the staging base.
	 */
	uint32_t image_pages = (image_size + OTA_FLASH_PAGE_SIZE - 1) / OTA_FLASH_PAGE_SIZE;
	ota.staging_base = OTA_FLASH_END - (image_pages * OTA_FLASH_PAGE_SIZE);
	ota.staging_base &= ~(OTA_FLASH_PAGE_SIZE - 1); /* Page-align */
	ota.page_buf_flash_addr = ota.staging_base;

	/* Verify staging area doesn't overlap with running firmware */
	if (ota.staging_base < OTA_FLASH_BASE + image_size) {
		LOG_ERR("OTA BEGIN: image too large for staging (%u bytes, staging at 0x%05X)",
			image_size, ota.staging_base);
		ota.state = OTA_STATE_ERROR;
		ota.error_code = OTA_STATUS_SIZE_ERROR;
		ota_send_status();
		return -ENOMEM;
	}

	LOG_INF("OTA: Staging area at 0x%05X (image %u bytes, %u pages)",
		ota.staging_base, image_size, image_pages);

	/* Suspend sensor thread and hardware to free CPU, SPI/I2C bus,
	 * and GPIO interrupts during OTA. OTA always ends with reboot. */
	LOG_WRN("OTA: Suspending sensor subsystem for OTA update");
	watchdog_pause(WDT_CHANNEL_SENSOR);
	main_imu_suspend();
	sensor_shutdown();

	/* VTOR relocation not needed: data goes to staging area, not running firmware */

	/* Erase first page — deferred to first DATA packet to keep BEGIN fast.
	 * The erase-ahead logic in handle_data already handles page erasing. */
	ota.state = OTA_STATE_READY;
	ota.last_page_erased = 0; /* Nothing erased yet */

	ota.state = OTA_STATE_READY;
	ota.last_data_time = k_uptime_get();

	LOG_INF("OTA: Ready to receive %u bytes (%u packets)", image_size, total_packets);
	ota_send_status();
	return 0;
#endif /* OTA_USE_RAM_ENGINE */
}

int esb_ota_handle_data(const uint8_t *data, size_t len)
{
	if (ota.state != OTA_STATE_READY && ota.state != OTA_STATE_RECEIVING) {
		LOG_WRN("OTA DATA: not in receiving state (state=%d)", ota.state);
		return -EINVAL;
	}

	if (len < OTA_DATA_HEADER_SIZE + 1) {
		LOG_ERR("OTA DATA: packet too short (%zu)", len);
		return -EINVAL;
	}

	uint16_t seq = sys_get_be16(&data[2]);
	size_t payload_len = len - OTA_DATA_HEADER_SIZE;
	const uint8_t *payload = &data[OTA_DATA_HEADER_SIZE];

	/* Check sequence number (wraparound-safe using signed diff) */
	if (seq != ota.next_expected_seq) {
		int16_t diff = (int16_t)(seq - ota.next_expected_seq);
		if (diff < 0) {
			/* Duplicate packet, ignore */
			LOG_DBG("OTA DATA: duplicate seq %u (expected %u)", seq, ota.next_expected_seq);
			return 0;
		}
		/* Gap detected */
		LOG_DBG("OTA DATA: seq gap (got %u, expected %u)", seq, ota.next_expected_seq);
		ota_send_status(); /* Tell receiver what we need */
		return -EAGAIN;
	}

	/* Ensure we don't write beyond image size */
	uint32_t remaining = ota.image_size - ota.bytes_written;
	if (payload_len > remaining) {
		payload_len = remaining;
	}

	ota.state = OTA_STATE_RECEIVING;
	ota.last_data_time = k_uptime_get();

	/* Append data to page buffer */
	size_t copied = 0;
	while (copied < payload_len) {
		size_t space = OTA_FLASH_PAGE_SIZE - ota.page_buf_offset;
		size_t chunk = payload_len - copied;
		if (chunk > space) {
			chunk = space;
		}

		memcpy(&ota.page_buf[ota.page_buf_offset], &payload[copied], chunk);
		ota.page_buf_offset += chunk;
		copied += chunk;

		/* Page buffer full → write to flash */
		if (ota.page_buf_offset >= OTA_FLASH_PAGE_SIZE) {
			LOG_DBG("OTA: flushing to 0x%05X (seq=%u)", ota.page_buf_flash_addr, seq);
			int err = ota_flush_page_buf();
			if (err) {
				ota.state = OTA_STATE_ERROR;
				ota.error_code = OTA_STATUS_FLASH_ERROR;
				ota_send_status();
				return err;
			}
		}
	}

	ota.bytes_written += payload_len;
	ota.next_expected_seq = seq + 1;

	/* Check if all data received */
	if (ota.bytes_written >= ota.image_size) {
		/* Flush any remaining data in page buffer */
		if (ota.page_buf_offset > 0) {
			int err = ota_flush_page_buf();
			if (err) {
				ota.state = OTA_STATE_ERROR;
				ota.error_code = OTA_STATUS_FLASH_ERROR;
				ota_send_status();
				return err;
			}
		}
		LOG_INF("OTA: All data received (%u bytes, %u packets)",
			ota.bytes_written, ota.next_expected_seq);
	}

	/* Periodic status report */
	if (k_uptime_get() - ota.last_status_time >= OTA_STATUS_INTERVAL_MS) {
		ota_send_status();
	}

	return 0;
}

int esb_ota_handle_verify(void)
{
	if (ota.bytes_written < ota.image_size) {
		LOG_ERR("OTA VERIFY: not all data received (%u/%u bytes)",
			ota.bytes_written, ota.image_size);
		ota_send_status();
		return -EINVAL;
	}

	LOG_INF("OTA: Verifying CRC32...");
	ota.state = OTA_STATE_VERIFYING;
	ota_send_status();

	uint32_t calc_crc = ota_compute_crc32(ota.staging_base, ota.image_size);
	if (calc_crc != ota.image_crc32) {
		LOG_ERR("OTA VERIFY: CRC32 mismatch (calculated 0x%08X, expected 0x%08X)",
			calc_crc, ota.image_crc32);
		ota.state = OTA_STATE_ERROR;
		ota.error_code = OTA_STATUS_VERIFY_FAIL;
		ota_send_status();
		return -EINVAL;
	}

	LOG_INF("OTA: CRC32 verified OK (0x%08X)", calc_crc);
	ota.state = OTA_STATE_VERIFYING; /* Stay in VERIFYING — get_status checks error_code */
	ota.error_code = OTA_STATUS_VERIFY_OK;
	ota.last_data_time = k_uptime_get(); /* Reset timeout — waiting for ACTIVATE */
	LOG_INF(">>> verify: about to send status");
	k_msleep(100);
	ota_send_status();
	LOG_INF(">>> verify: status sent, returning");
	k_msleep(100);
	return 0;
}

int esb_ota_handle_activate(void)
{
	LOG_INF(">>> OTA ACTIVATE called (state=%d error_code=0x%02X)", ota.state, ota.error_code);
	k_msleep(100);
	if (ota.error_code != OTA_STATUS_VERIFY_OK) {
		LOG_ERR("OTA ACTIVATE: firmware not verified");
		return -EINVAL;
	}

	LOG_WRN("OTA: Activating new firmware...");
	ota.state = OTA_STATE_ACTIVATING;
	ota_send_status();

	LOG_INF(">>> about to update bootloader settings");
	k_msleep(50);

	/* Compute bootloader settings (will be written by RAM copier in protected context) */
	int err = ota_prepare_bootloader_settings();

	LOG_INF(">>> bootloader settings prepared: %d", err);
	k_msleep(50);

	if (err) {
		LOG_ERR("OTA ACTIVATE: failed to update bootloader settings (err %d)", err);
		ota.state = OTA_STATE_ERROR;
		ota.error_code = OTA_STATUS_FLASH_ERROR;
		ota_send_status();
		return err;
	}

	LOG_WRN("OTA: Activation complete, rebooting in 500ms...");
	ota.state = OTA_STATE_COMPLETE;
	ota_send_status();

	/* Brief delay to allow final status to be transmitted */
	k_msleep(500);

	LOG_INF("OTA: About to call ota_write_first_page_and_reset");
	LOG_INF("OTA: staging=0x%05X final=0x%05X size=%u",
		ota.staging_base, ota.target_flash_base, ota.image_size);
	k_msleep(200);

	/* Copy from staging to final location (with IRQs disabled) and reset */
	ota_write_first_page_and_reset();

	/* If first page wasn't deferred, just reboot */
	sys_request_system_reboot(false);

	/* Should not reach here */
	return 0;
}

void esb_ota_handle_abort(void)
{
	if (ota.state == OTA_STATE_IDLE) {
		return;
	}

	LOG_WRN("OTA: Aborted (was in state %d, %u/%u bytes written)",
		ota.state, ota.bytes_written, ota.image_size);

	/*
	 * After abort, the flash may contain partial firmware data.
	 * The bootloader settings page has NOT been updated, so the bootloader
	 * will still try to run the old firmware. However, since we've been
	 * overwriting flash in-place, the old firmware is corrupted.
	 *
	 * Best action: reboot into UF2 bootloader for recovery.
	 */
	ota.state = OTA_STATE_IDLE;
	memset(&ota, 0, sizeof(ota));
	ota_send_status();

	LOG_WRN("OTA: Rebooting to UF2 bootloader for recovery...");
	k_msleep(200);

#if CONFIG_BUILD_OUTPUT_UF2
	NRF_POWER->GPREGRET = ADAFRUIT_DFU_MAGIC_UF2_RESET;
#endif
	sys_request_system_reboot(false);
}

void esb_ota_check_timeout(void)
{
	if (ota.state == OTA_STATE_IDLE || ota.state == OTA_STATE_COMPLETE) {
		return;
	}

	if ((k_uptime_get() - ota.last_data_time) > OTA_TIMEOUT_MS) {
		LOG_ERR("OTA: Timed out after %d ms with no data", OTA_TIMEOUT_MS);
		ota.state = OTA_STATE_ERROR;
		ota.error_code = OTA_STATUS_TIMEOUT;
		ota_send_status();

		/* Reboot to UF2 bootloader */
		k_msleep(200);
#if CONFIG_BUILD_OUTPUT_UF2
		NRF_POWER->GPREGRET = ADAFRUIT_DFU_MAGIC_UF2_RESET;
#endif
		sys_request_system_reboot(false);
	}
}

void esb_ota_periodic_status(void)
{
	if (ota.state == OTA_STATE_IDLE) {
		return;
	}

	/*
	 * Send an OTA_STATUS packet every call (~120 Hz from connection thread).
	 * Each TX triggers an ACK from the receiver which carries OTA data.
	 * This is the primary mechanism for pulling firmware data.
	 *
	 * The status packet always contains the current state and
	 * next_expected_seq so the receiver knows what to send next.
	 * The PC-side sees forwarded status at whatever rate the receiver relays.
	 */
	ota_send_status();
}

/* ── ESB Packet Handlers (called from esb.c event_handler) ───────── */

void esb_ota_process_rx_packet(const uint8_t *data, size_t len)
{
	if (len < 2) {
		return;
	}

	uint8_t type = data[0];
	switch (type) {
	case ESB_OTA_BEGIN_TYPE:
		esb_ota_handle_begin(data, len);
		break;
	case ESB_OTA_DATA_TYPE:
		esb_ota_handle_data(data, len);
		break;
	case ESB_OTA_VERIFY_TYPE:
		esb_ota_handle_verify();
		break;
	case ESB_OTA_ACTIVATE_TYPE:
		esb_ota_handle_activate();
		break;
	default:
		LOG_WRN("OTA: Unknown packet type 0x%02X", type);
		break;
	}
}

/* ── Internal Helpers ────────────────────────────────────────────── */

static void ota_send_status(void)
{
	uint8_t pkt[OTA_STATUS_PACKET_SIZE];

	pkt[0] = ESB_OTA_STATUS_TYPE;
	pkt[1] = connection_get_id();
	pkt[2] = esb_ota_get_status();
	sys_put_be16(ota.next_expected_seq, &pkt[3]);
	sys_put_le32(ota.bytes_written, &pkt[5]);
	pkt[9] = 0;
	pkt[10] = 0;
	pkt[11] = 0;
	pkt[12] = esb_ota_crc8(pkt, 12);

	esb_write(pkt, false, OTA_STATUS_PACKET_SIZE);
	ota.last_status_time = k_uptime_get();
}

static void ota_send_fw_info(void)
{
	uint8_t pkt[OTA_FW_INFO_PACKET_SIZE];
	memset(pkt, 0, sizeof(pkt));

	pkt[0] = ESB_OTA_FW_INFO_TYPE;
	pkt[1] = connection_get_id();
	pkt[2] = FW_VERSION_MAJOR;
	pkt[3] = FW_VERSION_MINOR;
	pkt[4] = FW_VERSION_PATCH;

	/* Build datetime packed (32-bit, BE):
	 *   bits 31-25: year-2020, bits 24-21: month, bits 20-16: day,
	 *   bits 15-11: hour, bits 10-5: minute, bits 4-0: second/2 */
	uint32_t build_dt = ((uint32_t)(BUILD_YEAR - 2020) & 0x7F) << 25 |
			    ((uint32_t)BUILD_MONTH & 0x0F) << 21 |
			    ((uint32_t)BUILD_DAY & 0x1F) << 16 |
			    ((uint32_t)BUILD_HOUR & 0x1F) << 11 |
			    ((uint32_t)BUILD_MIN & 0x3F) << 5 |
			    ((uint32_t)(BUILD_SEC / 2) & 0x1F);
	sys_put_be32(build_dt, &pkt[5]);

	/* Firmware size from linker symbol */
	extern char _flash_used[];
	sys_put_le32((uint32_t)_flash_used, &pkt[9]);

	/* Bootloader type */
#if CONFIG_BUILD_OUTPUT_UF2
	pkt[13] = 1; /* Adafruit UF2 */
#elif CONFIG_BOARD_HAS_NRF5_BOOTLOADER
	pkt[13] = 2; /* nRF5 bootloader */
#else
	pkt[13] = 0; /* None */
#endif

	pkt[14] = OTA_PROTOCOL_VERSION;

	/* Board target string */
	const char *board = BOARD_TARGET_STRING;
	strncpy((char *)&pkt[15], board, OTA_BOARD_TARGET_MAX - 1);

	/* Flash base address (page-aligned, >> 12) */
	sys_put_be16((uint16_t)(OTA_FLASH_BASE >> 12), &pkt[63]);

	pkt[65] = esb_ota_crc8(pkt, 65);

	esb_write(pkt, false, OTA_FW_INFO_PACKET_SIZE);
}

static void ota_erase_page(uint32_t addr)
{
	LOG_DBG("OTA: Erasing flash page at 0x%05X", addr);
	int err = flash_erase(flash_dev, addr, OTA_FLASH_PAGE_SIZE);
	if (err) {
		LOG_ERR("OTA: Flash erase failed at 0x%05X (err %d)", addr, err);
	}
}

static void ota_write_page(uint32_t addr, const uint8_t *data, size_t len)
{
	LOG_DBG("OTA: Writing %zu bytes to flash at 0x%05X", len, addr);
	int err = flash_write(flash_dev, addr, data, len);
	if (err) {
		LOG_ERR("OTA: Flash write failed at 0x%05X (err %d)", addr, err);
	}
}

static int ota_flush_page_buf(void)
{
	if (ota.page_buf_offset == 0) {
		return 0;
	}

	uint32_t flash_addr = ota.page_buf_flash_addr;

	/* Pad to 4-byte alignment */
	size_t write_len = ota.page_buf_offset;
	if (write_len & 3) {
		while (write_len & 3) {
			ota.page_buf[write_len++] = 0xFF;
		}
	}

	LOG_DBG("OTA: Flush page 0x%05X (%zu bytes)", flash_addr, write_len);

	/* Erase and write to staging area */
	if (flash_addr > ota.last_page_erased) {
		ota_erase_page(flash_addr);
		ota.last_page_erased = flash_addr;
	}
	ota_write_page(flash_addr, ota.page_buf, write_len);

	/* Advance to next page */
	ota.page_buf_flash_addr += OTA_FLASH_PAGE_SIZE;
	ota.page_buf_offset = 0;
	memset(ota.page_buf, 0xFF, OTA_FLASH_PAGE_SIZE);

	/* Pre-erase next page in staging area */
	uint32_t next_page = ota.page_buf_flash_addr;
	if (next_page < ota.staging_base + ota.image_size) {
		ota_erase_page(next_page);
		ota.last_page_erased = next_page;
	}

	return 0;
}

/*
 * RAM-resident flash copier: copies image from staging area to final location.
 * This function is copied to RAM before execution because it erases the flash
 * pages containing the running firmware (including itself).
 *
 * Must be self-contained — no calls to external functions.
 * Parameters passed via a struct to keep the interface simple.
 */
struct flash_copy_params {
	uint32_t src_addr;      /* Staging area start (flash offset) */
	uint32_t dst_addr;      /* Final location start (flash offset) */
	uint32_t size;          /* Image size in bytes */
	uint32_t page_size;     /* Flash page size (4096) */
	uint32_t settings_addr; /* Bootloader settings page address (0 to skip) */
	uint32_t settings_data[8]; /* Bootloader settings (32 bytes max) */
	uint32_t settings_words;   /* Number of 32-bit words to write */
};

__attribute__((noinline))
static void ota_flash_copy_from_ram(const struct flash_copy_params *p)
{
	uint32_t pages = (p->size + p->page_size - 1) / p->page_size;

	for (uint32_t i = 0; i < pages; i++) {
		uint32_t dst_page = p->dst_addr + i * p->page_size;

		/* Feed hardware WDT to prevent reset during long copy.
		 * NRF_WDT->RR[0] = 0x6E524635 (reload register) */
		((volatile uint32_t *)0x40010600)[0] = 0x6E524635;

		uint32_t src_page = p->src_addr + i * p->page_size;

		/* Determine words to compare/write for this page */
		uint32_t remaining = p->size - i * p->page_size;
		uint32_t words = p->page_size / 4;
		if (remaining < p->page_size) {
			words = (remaining + 3) / 4;
		}

		/* Compare destination with source — skip erase+write if identical */
		const volatile uint32_t *cmp_dst = (const volatile uint32_t *)dst_page;
		const volatile uint32_t *cmp_src = (const volatile uint32_t *)src_page;
		bool page_match = true;
		for (uint32_t w = 0; w < words; w++) {
			if (cmp_dst[w] != cmp_src[w]) {
				page_match = false;
				break;
			}
		}
		if (page_match) {
			continue;
		}

		/* Erase destination page via NVMC */
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
		__DSB();
		NRF_NVMC->ERASEPAGE = dst_page;
		while (!NRF_NVMC->READY) {}

		/* Write from staging source */
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
		__DSB();

		volatile uint32_t *dst = (volatile uint32_t *)dst_page;
		const volatile uint32_t *src = (const volatile uint32_t *)src_page;

		for (uint32_t w = 0; w < words; w++) {
			dst[w] = src[w];
			while (!NRF_NVMC->READY) {}
		}
	}

	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
	__DSB();

	/* Write bootloader settings page if requested */
	if (p->settings_addr != 0) {
		/* Feed WDT */
		((volatile uint32_t *)0x40010600)[0] = 0x6E524635;

		/* Erase settings page */
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
		__DSB();
		NRF_NVMC->ERASEPAGE = p->settings_addr;
		while (!NRF_NVMC->READY) {}

		/* Write settings */
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
		__DSB();
		volatile uint32_t *dst = (volatile uint32_t *)p->settings_addr;
		for (uint32_t w = 0; w < p->settings_words; w++) {
			dst[w] = p->settings_data[w];
			while (!NRF_NVMC->READY) {}
		}

		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
		__DSB();
	}

	/* Reset */
	SCB->AIRCR = (0x5FA << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk;
	__DSB();
	for (;;) {} /* Wait for reset */
}

static void ota_write_first_page_and_reset(void)
{
	LOG_WRN("OTA: Copying %u bytes from staging 0x%05X to final 0x%05X — IRQs off, then reset",
		ota.image_size, ota.staging_base, ota.target_flash_base);
	k_msleep(500); /* Flush logs */

	/* Copy the flash copier function to RAM */
	static uint8_t __aligned(4) ram_func_buf[768]; /* Generous size for the copier + settings write */
	uintptr_t func_addr = (uintptr_t)ota_flash_copy_from_ram;
	/* Thumb functions have bit 0 set; clear it for copy, set it for call */
	uintptr_t func_start = func_addr & ~1U;
	memcpy(ram_func_buf, (void *)func_start, sizeof(ram_func_buf));

	/* Prepare params */
	static struct flash_copy_params params;
	params.src_addr = ota.staging_base;
	params.dst_addr = ota.target_flash_base;
	params.size = ota.image_size;
	params.page_size = OTA_FLASH_PAGE_SIZE;

	/* Include bootloader settings if prepared */
	if (bl_settings_prepared) {
		params.settings_addr = BOOTLOADER_SETTINGS_ADDR;
		BUILD_ASSERT(sizeof(prepared_bl_settings) <= sizeof(params.settings_data),
			"bootloader_settings too large for flash_copy_params");
		memcpy(params.settings_data, &prepared_bl_settings,
		       sizeof(prepared_bl_settings));
		params.settings_words = sizeof(prepared_bl_settings) / 4;
	} else {
		params.settings_addr = 0; /* Skip */
		params.settings_words = 0;
	}

	/* Call the RAM copy with IRQs disabled */
	typedef void (*flash_copy_fn)(const struct flash_copy_params *);
	flash_copy_fn ram_copy = (flash_copy_fn)((uintptr_t)ram_func_buf | 1U); /* Thumb bit */

	__disable_irq();

	/* Disable MPU so we can execute code from RAM (SRAM is XN by default with Zephyr's MPU config) */
	MPU->CTRL = 0;
	__DSB();
	__ISB();

	ram_copy(&params);
	/* Never reached */
}

#if OTA_USE_RAM_ENGINE
/*
 * Launch the bare-metal RAM OTA engine.
 * Captures current RADIO configuration, stops the SDK ESB driver,
 * copies the engine function to RAM, and jumps to it with IRQs disabled.
 * This function never returns.
 */
#include "ota_ram_engine.inc"  /* Include for struct definition + function body */

static void ota_launch_ram_engine(void)
{
	LOG_WRN("OTA: Launching RAM engine for in-place update");
	LOG_WRN("OTA: target=0x%05X size=%u crc32=0x%08X",
		ota.target_flash_base, ota.image_size, ota.image_crc32);
	k_msleep(200); /* Flush logs */

	/* Capture RADIO configuration before stopping ESB */
	static struct ota_ram_engine_params params;
	params.radio_frequency   = NRF_RADIO->FREQUENCY;
	params.radio_mode        = NRF_RADIO->MODE;
	params.radio_pcnf0       = NRF_RADIO->PCNF0;
	params.radio_pcnf1       = NRF_RADIO->PCNF1;
	params.radio_crccnf      = NRF_RADIO->CRCCNF;
	params.radio_crcpoly     = NRF_RADIO->CRCPOLY;
	params.radio_crcinit     = NRF_RADIO->CRCINIT;

	params.radio_base0       = NRF_RADIO->BASE0;
	params.radio_base1       = NRF_RADIO->BASE1;
	params.radio_prefix0     = NRF_RADIO->PREFIX0;
	params.radio_prefix1     = NRF_RADIO->PREFIX1;
	params.radio_txaddress   = NRF_RADIO->TXADDRESS;
	params.radio_rxaddresses = NRF_RADIO->RXADDRESSES;
	params.radio_txpower     = NRF_RADIO->TXPOWER;

	LOG_DBG("OTA: RADIO capture: FREQ=%u MODE=%u PCNF0=0x%08X PCNF1=0x%08X",
		params.radio_frequency, params.radio_mode, params.radio_pcnf0, params.radio_pcnf1);
	LOG_DBG("OTA: RADIO capture: CRC_CNF=%u BASE0=0x%08X PREFIX0=0x%08X TXADDR=%u RXADDR=0x%02X",
		params.radio_crccnf, params.radio_base0, params.radio_prefix0,
		params.radio_txaddress, params.radio_rxaddresses);

	/* OTA state */
	params.image_size        = ota.image_size;
	params.image_crc32       = ota.image_crc32;
	params.flash_target      = ota.target_flash_base;
	params.page_size         = OTA_FLASH_PAGE_SIZE;
	params.next_expected_seq = 0;
	params.bytes_received    = 0;
	params.tracker_id        = connection_get_id();

	/* Bootloader settings: pre-compute CRC16 is not possible since
	 * data hasn't been written yet. The RAM engine will need to compute
	 * CRC16 after writing. For now, prepare the settings template and
	 * the RAM engine will fill in the CRC16 field. */
	params.settings_addr  = BOOTLOADER_SETTINGS_ADDR;

	/* Provide page buffer (static, avoids stack overflow) */
	static uint8_t __aligned(4) ota_page_buf[OTA_FLASH_PAGE_SIZE];
	params.page_buf = ota_page_buf;

	/* Stop ESB driver */
	LOG_WRN("OTA: Stopping ESB driver");
	esb_disable();
	k_msleep(50);

	/* Copy RAM engine to RAM buffer */
	/* The engine function is large (~3-4KB), allocate generous buffer.
	 * Using static to avoid stack overflow. */
	static uint8_t __aligned(4) ram_engine_buf[8192];
	uintptr_t func_addr = (uintptr_t)ota_ram_engine;
	uintptr_t func_start = func_addr & ~1U;
	memcpy(ram_engine_buf, (void *)func_start, sizeof(ram_engine_buf));

	LOG_WRN("OTA: RAM engine at %p, size up to %zu bytes", (void *)func_start, sizeof(ram_engine_buf));
	k_msleep(200);

	/* Jump to RAM with IRQs disabled */
	typedef void (*ram_engine_fn)(const struct ota_ram_engine_params *);
	ram_engine_fn engine = (ram_engine_fn)((uintptr_t)ram_engine_buf | 1U);

	__disable_irq();

	/* Disable MPU (SRAM is XN by default) */
	MPU->CTRL = 0;
	__DSB();
	__ISB();

	engine(&params);
	/* Never reached */
}
#endif /* OTA_USE_RAM_ENGINE */

static uint32_t ota_compute_crc32(uint32_t addr, uint32_t size)
{
	uint32_t crc = 0;
	uint32_t remaining = size;
	uint32_t offset = addr;

	while (remaining > 0) {
		size_t chunk = MIN(remaining, OTA_FLASH_PAGE_SIZE);
		int err = flash_read(flash_dev, offset, ota.page_buf, chunk);
		if (err) {
			LOG_ERR("OTA: Flash read failed at 0x%05X (err %d)", offset, err);
			return 0;
		}
		crc = crc32_ieee_update(crc, ota.page_buf, chunk);
		offset += chunk;
		remaining -= chunk;
	}

	return crc;
}

/**
 * Compute the Nordic SDK CRC-16 used by the Adafruit bootloader
 * for application validation (bank_0_crc field).
 */
static uint16_t ota_compute_crc16_nordic(uint32_t addr, uint32_t size)
{
	uint16_t crc = 0xFFFF;
	uint32_t remaining = size;
	uint32_t offset = addr;

	while (remaining > 0) {
		size_t chunk = MIN(remaining, OTA_FLASH_PAGE_SIZE);
		int err = flash_read(flash_dev, offset, ota.page_buf, chunk);
		if (err) {
			LOG_ERR("OTA: Flash read failed at 0x%05X (err %d)", offset, err);
			return 0;
		}

		for (size_t i = 0; i < chunk; i++) {
			crc = (uint8_t)(crc >> 8) | (crc << 8);
			crc ^= ota.page_buf[i];
			crc ^= (uint8_t)(crc & 0xFF) >> 4;
			crc ^= (crc << 8) << 4;
			crc ^= ((crc & 0xFF) << 4) << 1;
		}

		offset += chunk;
		remaining -= chunk;
	}

	return crc;
}

static int __attribute__((unused)) ota_update_bootloader_settings(void)
{
	/* Kept for reference — no longer called directly */
	return -ENOTSUP;
}

static int ota_prepare_bootloader_settings(void)
{
#if BOOTLOADER_SETTINGS_ADDR == 0
	LOG_ERR("OTA: Bootloader settings address not defined for this SoC");
	return -ENOTSUP;
#else
	struct bootloader_settings settings = {
		.bank_0 = BANK_VALID_APP,
		.bank_0_crc = 0,
		.bank_1 = BANK_INVALID_APP,
		.padding = 0,
		.bank_0_size = ota.image_size,
		.sd_image_size = 0,
		.bl_image_size = 0,
		.app_image_size = 0,
		.sd_image_start = 0,
	};

	settings.bank_0_crc = ota_compute_crc16_nordic(ota.staging_base, ota.image_size);
	LOG_INF(">>> CRC16 computed: 0x%04X", settings.bank_0_crc);
	if (settings.bank_0_crc == 0) {
		LOG_WRN("OTA: CRC-16 is 0, bootloader will skip CRC check");
	} else {
		LOG_INF("OTA: Bootloader CRC-16 = 0x%04X", settings.bank_0_crc);
	}

	prepared_bl_settings = settings;
	bl_settings_prepared = true;

	LOG_INF("OTA: Bootloader settings prepared (will be written by RAM copier)");
	return 0;
#endif
}
