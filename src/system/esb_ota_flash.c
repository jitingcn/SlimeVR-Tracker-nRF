/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors
*/
/*
 * ESB OTA flash storage helpers: page erase/write, staging flush, CRC over
 * flash, Adafruit bootloader settings prep, and RAM-resident staging→app copy.
 */

#include "esb_ota_flash.h"
#include "esb_ota.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/sys/crc.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_nvmc.h>
#include <string.h>
#include <errno.h>

LOG_MODULE_REGISTER(esb_ota_flash, LOG_LEVEL_INF);

#if CONFIG_SOC_NRF52840
#define BOOTLOADER_SETTINGS_ADDR BOOTLOADER_SETTINGS_ADDR_52840
#elif CONFIG_SOC_NRF52833
#define BOOTLOADER_SETTINGS_ADDR BOOTLOADER_SETTINGS_ADDR_52833
#else
#define BOOTLOADER_SETTINGS_ADDR 0
#endif

static const struct device *flash_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));

/* Prepared bootloader settings (computed in thread context, written by RAM copier) */
static struct bootloader_settings prepared_bl_settings;
static bool bl_settings_prepared;

bool esb_ota_flash_ready(void)
{
	return device_is_ready(flash_dev);
}

void esb_ota_flash_erase_page(uint32_t addr)
{
	LOG_DBG("OTA: Erasing flash page at 0x%05X", addr);
	int err = flash_erase(flash_dev, addr, OTA_FLASH_PAGE_SIZE);
	if (err) {
		LOG_ERR("OTA: Flash erase failed at 0x%05X (err %d)", addr, err);
	}
}

void esb_ota_flash_write_page(uint32_t addr, const uint8_t *data, size_t len)
{
	LOG_DBG("OTA: Writing %zu bytes to flash at 0x%05X", len, addr);
	int err = flash_write(flash_dev, addr, data, len);
	if (err) {
		LOG_ERR("OTA: Flash write failed at 0x%05X (err %d)", addr, err);
	}
}

int esb_ota_flash_flush_page_buf(struct esb_ota_page_buf *pb)
{
	if (*pb->offset == 0) {
		return 0;
	}

	uint32_t flash_addr = *pb->flash_addr;

	/* Pad to 4-byte alignment */
	size_t write_len = *pb->offset;
	if (write_len & 3) {
		while (write_len & 3) {
			pb->buf[write_len++] = 0xFF;
		}
	}

	LOG_DBG("OTA: Flush page 0x%05X (%zu bytes)", flash_addr, write_len);

	/* Erase and write to staging area */
	if (flash_addr > *pb->last_erased) {
		esb_ota_flash_erase_page(flash_addr);
		*pb->last_erased = flash_addr;
	}
	esb_ota_flash_write_page(flash_addr, pb->buf, write_len);

	/* Advance to next page */
	*pb->flash_addr += OTA_FLASH_PAGE_SIZE;
	*pb->offset = 0;
	memset(pb->buf, 0xFF, OTA_FLASH_PAGE_SIZE);

	/* Pre-erase next page in staging area */
	uint32_t next_page = *pb->flash_addr;
	if (next_page < pb->staging_base + pb->image_size) {
		esb_ota_flash_erase_page(next_page);
		*pb->last_erased = next_page;
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

void esb_ota_flash_copy_and_reset(uint32_t staging_base, uint32_t target_base,
				  uint32_t image_size)
{
	LOG_WRN("OTA: Copying %u bytes from staging 0x%05X to final 0x%05X — IRQs off, then reset",
		image_size, staging_base, target_base);
	k_msleep(500); /* Flush logs */

	/* Copy the flash copier function to RAM */
	static uint8_t __aligned(4) ram_func_buf[768]; /* Generous size for the copier + settings write */
	uintptr_t func_addr = (uintptr_t)ota_flash_copy_from_ram;
	/* Thumb functions have bit 0 set; clear it for copy, set it for call */
	uintptr_t func_start = func_addr & ~1U;
	memcpy(ram_func_buf, (void *)func_start, sizeof(ram_func_buf));

	/* Prepare params */
	static struct flash_copy_params params;
	params.src_addr = staging_base;
	params.dst_addr = target_base;
	params.size = image_size;
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

uint32_t esb_ota_flash_compute_crc32(uint32_t addr, uint32_t size, uint8_t *scratch)
{
	uint32_t crc = 0;
	uint32_t remaining = size;
	uint32_t offset = addr;

	while (remaining > 0) {
		size_t chunk = MIN(remaining, OTA_FLASH_PAGE_SIZE);
		int err = flash_read(flash_dev, offset, scratch, chunk);
		if (err) {
			LOG_ERR("OTA: Flash read failed at 0x%05X (err %d)", offset, err);
			return 0;
		}
		crc = crc32_ieee_update(crc, scratch, chunk);
		offset += chunk;
		remaining -= chunk;
	}

	return crc;
}

/**
 * Compute the Nordic SDK CRC-16 used by the Adafruit bootloader
 * for application validation (bank_0_crc field).
 */
uint16_t esb_ota_flash_compute_crc16_nordic(uint32_t addr, uint32_t size, uint8_t *scratch)
{
	uint16_t crc = 0xFFFF;
	uint32_t remaining = size;
	uint32_t offset = addr;

	while (remaining > 0) {
		size_t chunk = MIN(remaining, OTA_FLASH_PAGE_SIZE);
		int err = flash_read(flash_dev, offset, scratch, chunk);
		if (err) {
			LOG_ERR("OTA: Flash read failed at 0x%05X (err %d)", offset, err);
			return 0;
		}

		for (size_t i = 0; i < chunk; i++) {
			crc = (uint8_t)(crc >> 8) | (crc << 8);
			crc ^= scratch[i];
			crc ^= (uint8_t)(crc & 0xFF) >> 4;
			crc ^= (crc << 8) << 4;
			crc ^= ((crc & 0xFF) << 4) << 1;
		}

		offset += chunk;
		remaining -= chunk;
	}

	return crc;
}

int esb_ota_flash_prepare_bootloader_settings(uint32_t staging_base, uint32_t image_size,
					      uint8_t *scratch)
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
		.bank_0_size = image_size,
		.sd_image_size = 0,
		.bl_image_size = 0,
		.app_image_size = 0,
		.sd_image_start = 0,
	};

	settings.bank_0_crc = esb_ota_flash_compute_crc16_nordic(staging_base, image_size,
								scratch);
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
