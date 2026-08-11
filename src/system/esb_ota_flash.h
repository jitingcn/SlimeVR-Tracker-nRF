/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors
*/
#ifndef SLIMENRF_ESB_OTA_FLASH_H
#define SLIMENRF_ESB_OTA_FLASH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define OTA_FLASH_PAGE_SIZE 4096

struct esb_ota_page_buf {
	uint8_t *buf;
	uint16_t *offset;
	uint32_t *flash_addr;
	uint32_t *last_erased;
	uint32_t staging_base;
	uint32_t image_size;
	bool pre_erased;
};

bool esb_ota_flash_ready(void);

int esb_ota_flash_erase_page(uint32_t addr);
int esb_ota_flash_write_page(uint32_t addr, const uint8_t *data, size_t len);
int esb_ota_flash_flush_page_buf(struct esb_ota_page_buf *pb);

uint32_t esb_ota_flash_compute_crc32(uint32_t addr, uint32_t size, uint8_t *scratch);
uint16_t esb_ota_flash_compute_crc16_nordic(uint32_t addr, uint32_t size, uint8_t *scratch);

int esb_ota_flash_prepare_bootloader_settings(uint32_t staging_base, uint32_t image_size,
					      uint8_t *scratch);
int esb_ota_flash_mcuboot_region(uint32_t *addr, uint32_t *capacity);
int esb_ota_flash_prepare_mcuboot_slot(void);
int esb_ota_flash_request_mcuboot_upgrade(void);
void esb_ota_flash_copy_and_reset(uint32_t staging_base, uint32_t target_base,
				  uint32_t image_size);

#endif /* SLIMENRF_ESB_OTA_FLASH_H */
