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
#if defined(CONFIG_BOOTLOADER_MCUBOOT) && DT_NODE_EXISTS(DT_NODELABEL(slot1_partition))
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/storage/flash_map.h>
#define OTA_MCUBOOT_HAS_SECONDARY 1
#else
#define OTA_MCUBOOT_HAS_SECONDARY 0
#endif
#if defined(CONFIG_SOC_NRF52840)
#include <hal/nrf_nvmc.h>
#endif
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

#if BOOTLOADER_SETTINGS_ADDR != 0
/* Prepared bootloader settings (computed in thread context, written by RAM copier) */
static struct bootloader_settings prepared_bl_settings;
static bool bl_settings_prepared;
#endif

bool esb_ota_flash_ready(void)
{
	return device_is_ready(flash_dev);
}

int esb_ota_flash_erase_page(uint32_t addr)
{
	LOG_DBG("OTA: Erasing flash page at 0x%05X", addr);
	int err = flash_flatten(flash_dev, addr, OTA_FLASH_PAGE_SIZE);
	if (err) {
		LOG_ERR("OTA: Flash erase failed at 0x%05X (err %d)", addr, err);
	}
	return err;
}

int esb_ota_flash_write_page(uint32_t addr, const uint8_t *data, size_t len)
{
	LOG_DBG("OTA: Writing %zu bytes to flash at 0x%05X", len, addr);
	int err = flash_write(flash_dev, addr, data, len);
	if (err) {
		LOG_ERR("OTA: Flash write failed at 0x%05X (err %d)", addr, err);
	}
	return err;
}

int esb_ota_flash_flush_page_buf(struct esb_ota_page_buf *pb)
{
	if (*pb->offset == 0) {
		return 0;
	}

	uint32_t flash_addr = *pb->flash_addr;

	size_t write_len = *pb->offset;
	size_t write_align = flash_get_write_block_size(flash_dev);
	if (write_align == 0 || write_align > OTA_FLASH_PAGE_SIZE) {
		return -EINVAL;
	}
	if (write_len % write_align) {
		while (write_len % write_align) {
			pb->buf[write_len++] = 0xFF;
		}
	}

	LOG_DBG("OTA: Flush page 0x%05X (%zu bytes)", flash_addr, write_len);

	/* Erase and write to staging area */
	if (!pb->pre_erased && flash_addr > *pb->last_erased) {
		int err = esb_ota_flash_erase_page(flash_addr);
		if (err) {
			return err;
		}
		*pb->last_erased = flash_addr;
	}
	int err = esb_ota_flash_write_page(flash_addr, pb->buf, write_len);
	if (err) {
		return err;
	}

	/* Advance to next page — current page write already committed. */
	*pb->flash_addr += OTA_FLASH_PAGE_SIZE;
	*pb->offset = 0;
	memset(pb->buf, 0xFF, OTA_FLASH_PAGE_SIZE);

	/* Pre-erase next page (best-effort). Next flush retries via last_erased. */
	uint32_t next_page = *pb->flash_addr;
	if (!pb->pre_erased && next_page < pb->staging_base + pb->image_size) {
		err = esb_ota_flash_erase_page(next_page);
		if (err) {
			LOG_WRN("OTA: next-page pre-erase failed at 0x%05X (err %d)", next_page, err);
		} else {
			*pb->last_erased = next_page;
		}
	}

	return 0;
}

int esb_ota_flash_mcuboot_region(uint32_t *addr, uint32_t *capacity)
{
#if OTA_MCUBOOT_HAS_SECONDARY
	if (!boot_is_img_confirmed()) {
		return -EBUSY;
	}

	const uint8_t area_id = FIXED_PARTITION_ID(slot1_partition);
	const struct flash_area *area;
	int err = flash_area_open(area_id, &area);
	if (err) {
		return err;
	}

	size_t image_offset = boot_get_image_start_offset(area_id);
	ssize_t trailer_offset = boot_get_area_trailer_status_offset(area_id);
	if (trailer_offset < 0 || area->fa_size <= image_offset ||
	    (size_t)trailer_offset <= image_offset) {
		flash_area_close(area);
		return -ENOMEM;
	}

	*addr = area->fa_off + image_offset;
	*capacity = (uint32_t)trailer_offset - image_offset;
	flash_area_close(area);
	return 0;
#else
	ARG_UNUSED(addr);
	ARG_UNUSED(capacity);
	return -ENOTSUP;
#endif
}

int esb_ota_flash_prepare_mcuboot_slot(void)
{
#if OTA_MCUBOOT_HAS_SECONDARY
	const struct flash_area *area;
	int err = flash_area_open(FIXED_PARTITION_ID(slot1_partition), &area);
	if (err) {
		return err;
	}

	err = flash_area_flatten(area, 0, area->fa_size);
	flash_area_close(area);
	return err;
#else
	return -ENOTSUP;
#endif
}

int esb_ota_flash_request_mcuboot_upgrade(void)
{
#if OTA_MCUBOOT_HAS_SECONDARY
	return boot_request_upgrade(BOOT_UPGRADE_TEST);
#else
	return -ENOTSUP;
#endif
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

#if defined(CONFIG_SOC_NRF52840)
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
#endif

void esb_ota_flash_copy_and_reset(uint32_t staging_base, uint32_t target_base,
				  uint32_t image_size)
{
#if defined(CONFIG_SOC_NRF52840)
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
#else
	ARG_UNUSED(staging_base);
	ARG_UNUSED(target_base);
	ARG_UNUSED(image_size);
	LOG_ERR("OTA: in-place flash copy is not supported on this SoC");
#endif
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
			/* Nonzero poison so callers cannot treat failure as a valid CRC. */
			return 0xFFFFFFFFu;
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
			/* 0 means "skip CRC" in Adafruit BL — never return it on I/O fail. */
			return 0xFFFF;
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
	if (settings.bank_0_crc == 0xFFFF) {
		LOG_ERR("OTA: CRC-16 read failed, refusing activate");
		bl_settings_prepared = false;
		return -EIO;
	}
	if (settings.bank_0_crc == 0) {
		/* Legitimate zero is rare; still refuse skip-check activate. */
		LOG_ERR("OTA: CRC-16 is 0; refusing activate (bootloader would skip check)");
		bl_settings_prepared = false;
		return -EINVAL;
	}
	LOG_INF("OTA: Bootloader CRC-16 = 0x%04X", settings.bank_0_crc);

	prepared_bl_settings = settings;
	bl_settings_prepared = true;

	LOG_INF("OTA: Bootloader settings prepared (will be written by RAM copier)");
	return 0;
#endif
}
