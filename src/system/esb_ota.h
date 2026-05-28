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
 * ESB OTA Firmware Update Protocol
 *
 * Enables over-the-air firmware updates from PC → Receiver → Tracker
 * using the existing ESB (Enhanced ShockBurst) wireless protocol.
 *
 * Data flow:
 *   PC (USB HID) → Receiver (ESB TX) → Tracker (Flash Write)
 *
 * The tracker receives firmware data in 44-byte chunks via ESB,
 * writes them to internal flash, validates CRC32, and updates
 * the Adafruit bootloader settings page to activate the new firmware.
 */
#ifndef SLIMENRF_ESB_OTA_H
#define SLIMENRF_ESB_OTA_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*
 * ── ESB OTA Packet Types ────────────────────────────────────────────
 * These are used in byte[0] of ESB packets during OTA mode.
 */
#define ESB_OTA_DATA_TYPE       0x20  /* Receiver → Tracker: firmware data chunk */
#define ESB_OTA_STATUS_TYPE     0x21  /* Tracker → Receiver: OTA status report */
#define ESB_OTA_FW_INFO_TYPE    0x22  /* Tracker → Receiver: firmware info report */
#define ESB_OTA_BEGIN_TYPE      0x23  /* Receiver → Tracker: begin OTA session */
#define ESB_OTA_VERIFY_TYPE     0x24  /* Receiver → Tracker: request CRC verify */
#define ESB_OTA_ACTIVATE_TYPE   0x25  /* Receiver → Tracker: activate new firmware */

/*
 * ── OTA Status Codes ────────────────────────────────────────────────
 * Returned by tracker in ESB_OTA_STATUS_TYPE packets.
 */
#define OTA_STATUS_IDLE             0x00  /* Not in OTA mode */
#define OTA_STATUS_READY            0x01  /* Erased & ready for data */
#define OTA_STATUS_RECEIVING        0x02  /* Actively receiving data */
#define OTA_STATUS_VERIFY_OK        0x03  /* CRC32 verification passed */
#define OTA_STATUS_VERIFY_FAIL      0x04  /* CRC32 verification failed */
#define OTA_STATUS_ACTIVATING       0x05  /* Writing bootloader settings */
#define OTA_STATUS_COMPLETE         0x06  /* Activation complete, rebooting */
#define OTA_STATUS_ERROR            0x10  /* Generic error */
#define OTA_STATUS_BOARD_MISMATCH   0x11  /* Board target string mismatch */
#define OTA_STATUS_FLASH_ERROR      0x12  /* Flash erase/write error */
#define OTA_STATUS_SIZE_ERROR       0x13  /* Firmware too large for flash */
#define OTA_STATUS_SEQ_ERROR        0x14  /* Sequence number error */
#define OTA_STATUS_TIMEOUT          0x15  /* OTA timed out */

/*
 * ── OTA Data Packet Format (Receiver → Tracker, up to 48 bytes) ─────
 *
 * Byte  0:     ESB_OTA_DATA_TYPE (0x20)
 * Byte  1:     tracker_id
 * Byte  2-3:   sequence number (big-endian uint16, 0-based)
 * Byte  4-63:  firmware data (up to 60 bytes)
 */
#define OTA_DATA_HEADER_SIZE    4
#define OTA_DATA_MAX_PAYLOAD    60  /* 64 - 4 header bytes */

/*
 * ── OTA Begin Packet Format (Receiver → Tracker, 64 bytes) ─────────
 *
 * Byte  0:     ESB_OTA_BEGIN_TYPE (0x23)
 * Byte  1:     tracker_id
 * Byte  2-5:   image_size (uint32 LE)
 * Byte  6-9:   image_crc32 (uint32 LE, CRC32 of firmware binary)
 * Byte 10-11:  total_packets (uint16 BE)
 * Byte 12:     ota_protocol_version (1)
 * Byte 13-60:  board_target (null-terminated string, max 48 bytes)
 * Byte 61-62:  flash_base_address (uint16 BE, page-aligned: actual_addr >> 12)
 * Byte 63:     CRC-8 CCITT
 */
#define OTA_BEGIN_PACKET_SIZE   64
#define OTA_BOARD_TARGET_MAX    48
#define OTA_PROTOCOL_VERSION    1

/*
 * ── OTA Status Packet Format (Tracker → Receiver, 13 bytes) ────────
 *
 * Byte  0:     ESB_OTA_STATUS_TYPE (0x21)
 * Byte  1:     tracker_id
 * Byte  2:     status_code (OTA_STATUS_*)
 * Byte  3-4:   next_expected_seq (big-endian uint16)
 * Byte  5-8:   bytes_written (uint32 LE)
 * Byte  9-11:  reserved
 * Byte 12:     CRC-8 CCITT
 */
#define OTA_STATUS_PACKET_SIZE  13

/*
 * ── OTA Firmware Info Packet (Tracker → Receiver, 48 bytes) ────────
 *
 * Byte  0:     ESB_OTA_FW_INFO_TYPE (0x22)
 * Byte  1:     tracker_id
 * Byte  2:     version_major
 * Byte  3:     version_minor
 * Byte  4:     version_patch
 * Byte  5-8:   build_datetime packed (uint32 BE):
 *                bits 31-25: year-2020, 24-21: month, 20-16: day,
 *                15-11: hour, 10-5: minute, 4-0: second/2
 * Byte  9-12:  firmware_size (uint32 LE, current running firmware)
 * Byte 13:     bootloader_type (0=none, 1=adafruit_uf2, 2=nrf5_opendfu)
 * Byte 14:     ota_protocol_version
 * Byte 15-62:  board_target (null-terminated string, max 48 bytes)
 * Byte 63-64:  flash_base_address (uint16 BE, page-aligned: actual_addr >> 12)
 * Byte 65:     CRC-8 CCITT
 */
#define OTA_FW_INFO_PACKET_SIZE 66

/*
 * ── OTA Timing Constants ────────────────────────────────────────────
 */
#define OTA_TIMEOUT_MS          30000   /* Abort if no data for 30 seconds */
#define OTA_STATUS_INTERVAL_MS  200     /* Send status every 200ms */
#define OTA_WINDOW_SIZE         32      /* Sliding window: 32 packets */

/*
 * ── HID Report OTA Types (PC ↔ Receiver, in 64-byte HID reports) ──
 *
 * HID reports use byte[0] as report type. The following types are
 * reserved for OTA operations on the HID interface.
 */
#define HID_OTA_QUERY_INFO      0xF0  /* PC → Receiver: query tracker info */
#define HID_OTA_FW_INFO         0xF1  /* Receiver → PC: tracker firmware info */
#define HID_OTA_BEGIN           0xF2  /* PC → Receiver: start OTA session */
#define HID_OTA_DATA            0xF3  /* PC → Receiver: firmware data chunk */
#define HID_OTA_STATUS          0xF4  /* Receiver → PC: OTA status report */
#define HID_OTA_VERIFY          0xF5  /* PC → Receiver: request CRC verify */
#define HID_OTA_ACTIVATE        0xF6  /* PC → Receiver: activate new firmware */
#define HID_OTA_ABORT           0xF7  /* PC → Receiver: abort OTA */

/*
 * ── HID OTA Data Packet (PC → Receiver, 64 bytes) ──────────────────
 *
 * Byte  0:     HID_OTA_DATA (0xF3)
 * Byte  1:     target_tracker_id
 * Byte  2-3:   sequence number (big-endian uint16)
 * Byte  4-47:  firmware data (44 bytes, matches ESB payload)
 * Byte 48-63:  padding (0x00)
 */

/*
 * ── HID OTA Begin Packet (PC → Receiver, 64 bytes) ─────────────────
 *
 * Byte  0:     HID_OTA_BEGIN (0xF2)
 * Byte  1:     target_tracker_id
 * Byte  2-5:   image_size (uint32 LE)
 * Byte  6-9:   image_crc32 (uint32 LE)
 * Byte 10-11:  total_packets (uint16 BE)
 * Byte 12:     ota_protocol_version
 * Byte 13-44:  board_target (null-terminated string)
 * Byte 45-63:  padding
 */

/*
 * ── Bootloader Settings (Adafruit nRF52 Bootloader) ────────────────
 * Written to BOOTLOADER_SETTINGS_ADDRESS (0xFF000 on nRF52840) after
 * successful firmware update to mark the new app as valid.
 */
#define BOOTLOADER_SETTINGS_ADDR_52840  0xFF000
#define BOOTLOADER_SETTINGS_ADDR_52833  0x7F000

/* Bootloader bank status codes */
#define BANK_VALID_APP      0x01
#define BANK_VALID_SD       0xA5
#define BANK_VALID_BOOT     0xAA
#define BANK_ERASED         0xFE
#define BANK_INVALID_APP    0xFF

/*
 * Bootloader settings page layout (28 bytes):
 *   uint16_t bank_0;          // BANK_VALID_APP after successful update
 *   uint16_t bank_0_crc;      // CRC16 of application (0 = skip check)
 *   uint16_t bank_1;          // BANK_INVALID_APP
 *   uint16_t padding;
 *   uint32_t bank_0_size;     // Application size in bytes
 *   uint32_t sd_image_size;   // 0 for app-only update
 *   uint32_t bl_image_size;   // 0 for app-only update
 *   uint32_t app_image_size;  // 0 for app-only update
 *   uint32_t sd_image_start;  // 0 for app-only update
 */
struct bootloader_settings {
	uint16_t bank_0;
	uint16_t bank_0_crc;
	uint16_t bank_1;
	uint16_t padding;
	uint32_t bank_0_size;
	uint32_t sd_image_size;
	uint32_t bl_image_size;
	uint32_t app_image_size;
	uint32_t sd_image_start;
} __attribute__((packed));

/*
 * ── CRC Helpers ─────────────────────────────────────────────────────
 */

/* CRC-8 CCITT (polynomial 0x07), used for ESB packet validation */
static inline uint8_t esb_ota_crc8(const uint8_t *data, size_t len)
{
	uint8_t crc = 0;
	for (size_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (int j = 0; j < 8; j++) {
			crc = (crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1);
		}
	}
	return crc;
}

/*
 * ── API Functions ───────────────────────────────────────────────────
 */

/** Check if an OTA session is active */
bool esb_ota_is_active(void);

/** Handle firmware info query (sends FW_INFO packet) */
void esb_ota_handle_query_info(void);

/** Handle OTA begin packet; returns 0 on success */
int esb_ota_handle_begin(const uint8_t *data, size_t len);

/** Handle OTA data packet; returns 0 on success */
int esb_ota_handle_data(const uint8_t *data, size_t len);

/** Handle verify request; returns 0 on success */
int esb_ota_handle_verify(void);

/** Handle activate request; returns 0 on success */
int esb_ota_handle_activate(void);

/** Handle abort request (reboots to UF2 bootloader) */
void esb_ota_handle_abort(void);

/** Check for OTA timeout; call periodically from connection thread */
void esb_ota_check_timeout(void);

/** Send periodic OTA status packets; call from connection thread */
void esb_ota_periodic_status(void);

/** Dispatch an incoming OTA RX packet by type */
void esb_ota_process_rx_packet(const uint8_t *data, size_t len);

#endif /* SLIMENRF_ESB_OTA_H */
