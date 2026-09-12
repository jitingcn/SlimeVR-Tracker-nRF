/* Production flash CRC32, VERIFY, status and ACTIVATE bodies are extracted.
 * Only flash I/O, clock, transport and activation hardware leaves are modeled.
 * This covers staging OTA (MCUboot dual-slot and legacy nRF52840), not the
 * separate RAM engine, MCUboot image validation, or legacy CRC16 policy. */
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "system/esb_ota_flash.h"
#include "vectors.h"

#define LOG_INF(...) ((void)0)
#define LOG_ERR(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define __aligned(n) __attribute__((aligned(n)))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
typedef int atomic_t;
#define atomic_get(value) (*(value))
#define atomic_set(value, next) (*(value) = (next))

static uint8_t flash_image[OTA_FLASH_PAGE_SIZE + 4];
static const uint32_t flash_base = 0x10000;
static const void *flash_dev = flash_image;
static unsigned reads, fail_read, reserve_calls, preparations, copies, reboots;
static int read_error;
static int64_t now_ms;
static uint8_t reported_status;
uint8_t esb_ota_get_status(void);
static int64_t k_uptime_get(void) { return now_ms; }
static void k_msleep(int ms) { now_ms += ms; }
static void ota_send_status(void) { reported_status = esb_ota_get_status(); }

static int flash_read(const void *dev, uint32_t address, void *buffer, size_t size)
{
	assert(dev == flash_dev);
	assert(address >= flash_base);
	assert(address - flash_base <= sizeof(flash_image));
	assert(size <= sizeof(flash_image) - (address - flash_base));
	reads++;
	if (reads == fail_read) {
		return read_error;
	}
	memcpy(buffer, flash_image + address - flash_base, size);
	return 0;
}

/* IEEE CRC32 reference primitive, not a scripted return value. zlib-generated
 * long-vector and fixed standard vectors independently check this leaf. */
static uint32_t crc32_ieee_update(uint32_t crc, const uint8_t *data, size_t size)
{
	crc = ~crc;
	for (size_t i = 0; i < size; i++) {
		crc ^= data[i];
		for (int bit = 0; bit < 8; bit++) {
			crc = (crc >> 1) ^ ((crc & 1) ? UINT32_C(0xEDB88320) : 0);
		}
	}
	return ~crc;
}

static int sys_ota_reboot_reserve(void) { reserve_calls++; return 0; }
static void sys_ota_reboot_resolve(bool commit) { if (commit) { reboots++; } }
#if OTA_USE_MCUBOOT
int esb_ota_flash_request_mcuboot_upgrade(void) { preparations++; return 0; }
#else
int esb_ota_flash_prepare_bootloader_settings(uint32_t base, uint32_t size, uint8_t *scratch)
{
	(void)base; (void)size; (void)scratch;
	preparations++;
	return 0;
}
void esb_ota_flash_copy_and_reset(uint32_t base, uint32_t target, uint32_t size)
{
	(void)base; (void)target; (void)size;
	copies++;
}
#endif

#include "production.inc"

static void fixture(uint32_t size, uint32_t expected)
{
	memset(&ota, 0, sizeof(ota));
	memset(flash_image, 0xFF, sizeof(flash_image));
	ota_reboot_pending = 0;
	ota.state = OTA_STATE_RECEIVING;
	ota.image_size = ota.bytes_written = size;
	ota.image_crc32 = expected;
	ota.staging_base = flash_base;
	reads = fail_read = reserve_calls = preparations = copies = reboots = 0;
	read_error = -EIO;
	now_ms = 0;
	reported_status = OTA_STATUS_IDLE;
}

static void valid_full_range_crc(void)
{
	fixture(4, UINT32_MAX); /* CRC32 of FF FF FF FF is legitimately FFFFFFFF. */
	assert(esb_ota_handle_verify() == 0);
	assert(reported_status == OTA_STATUS_VERIFY_OK);
	assert(esb_ota_handle_activate() == 0);
	assert(reported_status == OTA_STATUS_COMPLETE);
	assert(reboots == 1);
	assert(copies == !OTA_USE_MCUBOOT);
}

static void flash_error_blocks_activation(bool after_success, unsigned failing_chunk, int error)
{
	fixture(after_success ? 4 : sizeof(flash_image), UINT32_MAX);
	if (after_success) {
		assert(esb_ota_handle_verify() == 0);
		assert(reported_status == OTA_STATUS_VERIFY_OK);
	}
	reads = 0;
	fail_read = failing_chunk;
	read_error = error;
	assert(esb_ota_handle_verify() == error);
	assert(reported_status == OTA_STATUS_FLASH_ERROR);
	assert(reads == failing_chunk);
	/* Even a transient read error cannot authorize ACTIVATE after I/O recovers. */
	fail_read = 0;
	assert(esb_ota_handle_activate() == -EINVAL);
	assert(reserve_calls == 0 && preparations == 0 && copies == 0 && reboots == 0);
}

static void mismatch_blocks_activation(void)
{
	fixture(4, 0);
	assert(esb_ota_handle_verify() == -EINVAL);
	assert(reported_status == OTA_STATUS_VERIFY_FAIL);
	assert(esb_ota_handle_activate() == -EINVAL);
	assert(reserve_calls == 0 && preparations == 0 && reboots == 0);
}

static void flash_helper_contract(void)
{
	fixture(sizeof(flash_image), LONG_IMAGE_CRC);
	uint32_t crc = 0;
#if OTA_LEGACY_CRC32
	crc = esb_ota_flash_compute_crc32(flash_base, sizeof(flash_image), ota.page_buf);
#else
	assert(esb_ota_flash_compute_crc32(flash_base, sizeof(flash_image), ota.page_buf, &crc) == 0);
#endif
	assert(crc == LONG_IMAGE_CRC);
#if !OTA_LEGACY_CRC32
	reads = 0;
	fail_read = 2;
	read_error = -ETIMEDOUT;
	crc = UINT32_C(0x12345678);
	assert(esb_ota_flash_compute_crc32(flash_base, sizeof(flash_image), ota.page_buf, &crc) == -ETIMEDOUT);
	assert(crc == UINT32_C(0x12345678)); /* No partial CRC is published. */
#endif
}

int main(void)
{
	assert(crc32_ieee_update(0, (const uint8_t *)"123456789", 9) == UINT32_C(0xCBF43926));
	valid_full_range_crc();
	flash_helper_contract();
	flash_error_blocks_activation(false, 1, -EIO);
	flash_error_blocks_activation(false, 2, -ETIMEDOUT);
	flash_error_blocks_activation(true, 1, -EIO);
	mismatch_blocks_activation();
	printf("PASS OTA CRC32 full range, read failures and activation gate (MCUboot=%d)\n", OTA_USE_MCUBOOT);
	return 0;
}
