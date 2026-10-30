/*
 * OTA RAM Engine — Bare-Metal ESB PTX + Flash Writer
 *
 * A RAM-resident engine performing:
 *   1. ESB PTX polling (send STATUS, receive DATA via ACK payload)
 *   2. Direct NVMC flash erase/write to target address
 *   3. CRC32 verification of written data
 *   4. System reset
 *
 * This enables in-place OTA updates on devices with limited flash
 * (e.g. nRF52833 with 512KB) where dual-bank staging is not possible.
 *
 * All reachable code and literal pools must remain in RAM. Register access
 * is done directly through hardware register pointers.
 *
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ── Parameters passed to the RAM engine ─────────────────────────── */

struct ota_ram_engine_params {
	/* ESB radio configuration (captured from RADIO registers before entry) */
	uint32_t radio_frequency;    /* RADIO->FREQUENCY value */
	uint32_t radio_mode;         /* RADIO->MODE value */
	uint32_t radio_pcnf0;       /* RADIO->PCNF0 value */
	uint32_t radio_pcnf1;       /* RADIO->PCNF1 value */
	uint32_t radio_crccnf;      /* RADIO->CRCCNF value */
	uint32_t radio_crcpoly;     /* RADIO->CRCPOLY value */
	uint32_t radio_crcinit;     /* RADIO->CRCINIT value */
	uint32_t radio_base0;       /* RADIO->BASE0 value */
	uint32_t radio_base1;       /* RADIO->BASE1 value */
	uint32_t radio_prefix0;     /* RADIO->PREFIX0 value */
	uint32_t radio_prefix1;     /* RADIO->PREFIX1 value */
	uint32_t radio_txaddress;   /* RADIO->TXADDRESS value */
	uint32_t radio_rxaddresses; /* RADIO->RXADDRESSES value */
	uint32_t radio_txpower;     /* RADIO->TXPOWER value */

	/* OTA transfer state */
	uint32_t image_size;         /* Total firmware size in bytes */
	uint32_t image_crc32;        /* Expected CRC32 of firmware */
	uint32_t required_image_magic;
	uint32_t flash_target;       /* Target flash address (e.g. 0x27000 or 0x1000) */
	uint32_t flash_limit;        /* Exclusive app/NVS boundary; 0 disables check */
	uint32_t page_size;          /* Flash page size (4096) */
	uint16_t next_expected_seq;  /* Next sequence number to expect */
	uint32_t bytes_received;     /* Bytes received so far */
	uint8_t  tracker_id;         /* Tracker ID for status packets */

	/* Bootloader settings */
	uint32_t settings_addr;      /* Bootloader settings page address (0 = skip) */

	/* External page buffer (must be 4KB, 4-byte aligned, in RAM) */
	uint8_t *page_buf;
};

static __ramfunc __attribute__((noinline))
bool ota_ram_wait(volatile uint32_t *event, uint32_t timeout_ms)
{
	volatile uint32_t *cycles = (volatile uint32_t *)0xE0001004;
	volatile uint32_t *wdt_rr0 = (volatile uint32_t *)0x40010600;
	uint32_t wait_start = *cycles;

	while (!*event) {
		*wdt_rr0 = 0x6E524635u;
		if ((uint32_t)(*cycles - wait_start) >= timeout_ms * 64000u) {
			return false;
		}
	}
	return true;
}

static __ramfunc __attribute__((noinline))
uint8_t ota_ram_crc8(const uint8_t *bytes, uint32_t len, uint8_t initial)
{
	uint8_t crc = initial;

	for (uint32_t i = 0; i < len; i++) {
		crc ^= bytes[i];
		for (uint32_t b = 0; b < 8; b++) {
			crc = crc & 0x80 ? (crc << 1) ^ 0x07 : crc << 1;
		}
	}
	return crc;
}

static __ramfunc __attribute__((noinline))
bool ota_ram_radio_disable(volatile uint32_t *radio)
{
	if (radio[0x550 / 4] != 0) {
		radio[0x200 / 4] = 0;
		radio[0x110 / 4] = 0;
		radio[0x010 / 4] = 1;
		return ota_ram_wait(&radio[0x110 / 4], 10);
	}
	return true;
}

/*
 * ota_ram_engine() — The main RAM-resident OTA engine
 *
 * MUST be linked into .ramfunc and initialized in RAM by startup.
 * All reachable code and literal pools MUST reside in RAM.
 *
 * Packet formats:
 *   TX (STATUS): [0x21][id][status][seq_hi][seq_lo][bytes_LE32][0][0][0][crc8]  (13 bytes)
 *   RX (DATA):   [0x20][id][seq_hi][seq_lo][up to 60 bytes payload]              (up to 64 bytes)
 *
 * ESB DPL packet buffer layout (for RADIO DMA):
 *   Byte 0: [length:6/8][rfu]  — LENGTH field (6 bits if max<=32, else 8 bits)
 *   Byte 1: [noack:1][pid:2][rfu:5]  — S1 field
 *   Byte 2..N: payload data
 */
/* Keep reachable instructions and literal pools in RAM. Interrupts
 * are disabled on entry; nRF52833/840 run this Cortex-M4 at a fixed 64 MHz.
 * The wire format remains protocol v1, including legacy 44-byte DATA payloads. */
__ramfunc __attribute__((used))
void ota_ram_engine(const struct ota_ram_engine_params *p)
{
	volatile uint32_t *radio = (volatile uint32_t *)0x40001000;
	volatile uint32_t *nvmc = (volatile uint32_t *)0x4001E000;
	volatile uint32_t *clock = (volatile uint32_t *)0x40000000;
	volatile uint32_t *wdt_rr0 = (volatile uint32_t *)0x40010600;
	volatile uint32_t *gpregret = (volatile uint32_t *)0x4000051C;
	volatile uint32_t *aircr = (volatile uint32_t *)0xE000ED0C;
	volatile uint32_t *demcr = (volatile uint32_t *)0xE000EDFC;
	volatile uint32_t *dwt_ctrl = (volatile uint32_t *)0xE0001000;
	volatile uint32_t *cycles = (volatile uint32_t *)0xE0001004;

#define RAM_REG(base, offset) ((base)[(offset) / 4])
#define RAM_FEED() (*wdt_rr0 = 0x6E524635u)
#define RAM_DSB() __asm volatile("dsb" ::: "memory")
#define RAM_WAIT(event, milliseconds) do { \
	if (!ota_ram_wait(&(event), (milliseconds))) \
		goto recover; \
} while (0)

	/* CYCCNT needs no TIMER/PPI resource. Unsigned elapsed comparisons cover
	 * wraparound; the longest deadline, 30 s, is below its 67 s wrap period. */
	*demcr |= 1u << 24;
	*dwt_ctrl |= 1u;
	*cycles = 0;
	RAM_DSB();

	if (p == NULL || p->image_size == 0 || p->page_size != 4096 ||
	    p->page_buf == NULL || ((uintptr_t)p->page_buf & 3u) != 0 ||
	    p->flash_target < 0x1000 || (p->flash_target & 4095u) != 0 ||
	    (p->flash_limit != 0 &&
	     ((p->flash_limit & 4095u) != 0 || p->flash_target >= p->flash_limit ||
	      p->image_size > p->flash_limit - p->flash_target))) {
		goto recover;
	}

	uint8_t *page_buf = p->page_buf;
	uint32_t bytes_received = p->bytes_received;
	uint16_t next_seq = p->next_expected_seq;
	uint32_t page_offset = bytes_received & 4095u;
	uint32_t page_addr = (p->flash_target + bytes_received) & ~4095u;
	uint8_t tx_buf[64] __attribute__((aligned(4)));
	uint8_t rx_buf[68] __attribute__((aligned(4)));
	uint8_t pid = 0;
	uint8_t status = OTA_STATUS_RECEIVING;
	uint8_t terminal_rounds = 0;
	uint32_t image_crc = 0;

	for (uint32_t i = 0; i < 4096; i++) {
		((volatile uint8_t *)page_buf)[i] = 0xFF;
	}

	RAM_REG(clock, 0x100) = 0;
	RAM_REG(clock, 0x000) = 1;
	RAM_WAIT(RAM_REG(clock, 0x100), 1000);
	if (!ota_ram_radio_disable(radio)) {
		goto recover;
	}
	RAM_REG(radio, 0x110) = 0;

	RAM_REG(radio, 0x510) = p->radio_mode;
	RAM_REG(radio, 0x514) = p->radio_pcnf0;
	RAM_REG(radio, 0x518) = p->radio_pcnf1;
	RAM_REG(radio, 0x534) = p->radio_crccnf;
	RAM_REG(radio, 0x538) = p->radio_crcpoly;
	RAM_REG(radio, 0x53C) = p->radio_crcinit;
	RAM_REG(radio, 0x51C) = p->radio_base0;
	RAM_REG(radio, 0x520) = p->radio_base1;
	RAM_REG(radio, 0x524) = p->radio_prefix0;
	RAM_REG(radio, 0x528) = p->radio_prefix1;
	RAM_REG(radio, 0x52C) = p->radio_txaddress;
	RAM_REG(radio, 0x530) = p->radio_rxaddresses;
	RAM_REG(radio, 0x50C) = p->radio_txpower;
	RAM_REG(radio, 0x508) = p->radio_frequency;
	RAM_REG(radio, 0x650) = 1; /* Fast ramp-up, matching the existing engine. */

	uint32_t last_progress = *cycles;
	uint32_t activate_start = 0;
	*gpregret = 0xD0;

	for (;;) {
		RAM_FEED();
		if ((status == OTA_STATUS_RECEIVING &&
		     (uint32_t)(*cycles - last_progress) >= 10u * 64000000u) ||
		    (status == OTA_STATUS_VERIFY_OK &&
		     (uint32_t)(*cycles - activate_start) >= 30u * 64000000u)) {
			goto recover;
		}

		/* One STATUS encoder and radio transaction serve every phase. */
		tx_buf[0] = OTA_STATUS_PACKET_SIZE;
		tx_buf[1] = (pid << 1) | 1u; /* S1 bit 0 requests an ACK. */
		uint8_t *out = &tx_buf[2];
		out[0] = ESB_OTA_STATUS_TYPE;
		out[1] = p->tracker_id;
		out[2] = status;
		out[3] = next_seq >> 8;
		out[4] = next_seq;
		uint32_t reported = status == OTA_STATUS_VERIFY_FAIL ? image_crc : bytes_received;
		out[5] = reported;
		out[6] = reported >> 8;
		out[7] = reported >> 16;
		out[8] = reported >> 24;
		out[9] = status == OTA_STATUS_VERIFY_FAIL ? p->image_crc32 : 0;
		out[10] = status == OTA_STATUS_VERIFY_FAIL ? p->image_crc32 >> 8 : 0;
		out[11] = status == OTA_STATUS_VERIFY_FAIL ? p->image_crc32 >> 16 : 0;
		out[12] = ota_ram_crc8(out, 12, 0);

		RAM_REG(radio, 0x100) = 0;
		RAM_REG(radio, 0x10C) = 0;
		RAM_REG(radio, 0x110) = 0;
		RAM_REG(radio, 0x130) = 0;
		RAM_REG(radio, 0x134) = 0;
		RAM_REG(radio, 0x504) = (uint32_t)tx_buf;
		/* READY_START | END_DISABLE | DISABLED_RXEN. */
		RAM_REG(radio, 0x200) = (1u << 0) | (1u << 1) | (1u << 3);
		RAM_REG(radio, 0x000) = 1;
		RAM_WAIT(RAM_REG(radio, 0x110), 10);
		RAM_REG(radio, 0x110) = 0;
		RAM_REG(radio, 0x504) = (uint32_t)rx_buf;
		RAM_REG(radio, 0x200) = (1u << 0) | (1u << 1);
		RAM_REG(radio, 0x130) = 0;
		RAM_REG(radio, 0x134) = 0;

		uint32_t rx_start = *cycles;
		while (!RAM_REG(radio, 0x110) &&
		       (uint32_t)(*cycles - rx_start) < 500u * 64u) {
			RAM_FEED();
		}
		bool got_packet = RAM_REG(radio, 0x110) != 0;
		if (!got_packet) {
			RAM_REG(radio, 0x010) = 1;
			RAM_WAIT(RAM_REG(radio, 0x110), 10);
		}
		RAM_REG(radio, 0x110) = 0;
		__asm volatile("" ::: "memory"); /* Reload EasyDMA writes. */
		uint8_t rx_len = rx_buf[0];
		bool valid_ack = got_packet && RAM_REG(radio, 0x130) && rx_len <= 64 &&
			((rx_buf[1] >> 1) & 3u) == pid;
		pid = (pid + 1u) & 3u;

		/* Existing PONG cancellation works without a new packet type/version.
		 * Once ACTIVATE is accepted, commit is no longer cancellable. */
		if (valid_ack && (status == OTA_STATUS_RECEIVING || status == OTA_STATUS_VERIFY_OK) &&
		    rx_len == ESB_PONG_LEN && rx_buf[2] == ESB_PONG_TYPE &&
		    rx_buf[3] == p->tracker_id && rx_buf[9] == ESB_PONG_FLAG_OTA_ABORT) {
			if (ota_ram_crc8(&rx_buf[2], 12, 0x07) == rx_buf[14]) {
				goto recover;
			}
		}

		if (status == OTA_STATUS_RECEIVING) {
			if (!valid_ack || rx_len < 5 || rx_buf[2] != ESB_OTA_DATA_TYPE ||
			    rx_buf[3] != p->tracker_id ||
			    (((uint16_t)rx_buf[4] << 8) | rx_buf[5]) != next_seq) {
				continue;
			}
			uint32_t data_len = rx_len - 4;
			if (bytes_received == 0) {
				if (data_len < 4) {
					goto recover;
				}
				uint32_t magic = (uint32_t)rx_buf[6] | (uint32_t)rx_buf[7] << 8 |
					(uint32_t)rx_buf[8] << 16 | (uint32_t)rx_buf[9] << 24;
				if ((p->required_image_magic != 0 && magic != p->required_image_magic) ||
				    (p->required_image_magic == 0 && magic == OTA_MCUBOOT_IMAGE_MAGIC)) {
					goto recover;
				}
			}
			if (data_len > p->image_size - bytes_received) {
				data_len = p->image_size - bytes_received;
			}
			for (uint32_t i = 0; i < data_len; i++) {
				page_buf[page_offset++] = rx_buf[6 + i];
				if (page_offset != 4096 && bytes_received + i + 1 != p->image_size) {
					continue;
				}
				volatile uint32_t *dst = (volatile uint32_t *)page_addr;
				const uint32_t *src = (const uint32_t *)page_buf;
				bool same = true;
				for (uint32_t w = 0; w < 1024; w++) {
					if (dst[w] != src[w]) {
						same = false;
						break;
					}
				}
				if (!same) {
					RAM_FEED();
					RAM_REG(nvmc, 0x504) = 2;
					RAM_DSB();
					RAM_REG(nvmc, 0x508) = page_addr;
					RAM_WAIT(RAM_REG(nvmc, 0x400), 500);
					RAM_REG(nvmc, 0x504) = 1;
					RAM_DSB();
					for (uint32_t w = 0; w < (page_offset + 3) / 4; w++) {
						dst[w] = src[w];
						RAM_WAIT(RAM_REG(nvmc, 0x400), 1);
					}
					RAM_REG(nvmc, 0x504) = 0;
					RAM_DSB();
				}
				page_addr += 4096;
				page_offset = 0;
				for (uint32_t b = 0; b < 4096; b++) {
					((volatile uint8_t *)page_buf)[b] = 0xFF;
				}
			}
			bytes_received += data_len;
			next_seq++;
			last_progress = *cycles;
			if (bytes_received != p->image_size) {
				continue;
			}

			*gpregret = 0xD1;
			image_crc = 0xFFFFFFFF;
			const volatile uint8_t *flash = (const volatile uint8_t *)p->flash_target;
			for (uint32_t i = 0; i < p->image_size; i++) {
				if ((i & 4095u) == 0) {
					RAM_FEED();
				}
				image_crc ^= flash[i];
				for (uint32_t b = 0; b < 8; b++) {
					image_crc = image_crc & 1 ? (image_crc >> 1) ^ 0xEDB88320u : image_crc >> 1;
				}
			}
			image_crc ^= 0xFFFFFFFF;
			terminal_rounds = 0;
			if (image_crc != p->image_crc32) {
				status = OTA_STATUS_VERIFY_FAIL;
			} else {
				status = OTA_STATUS_VERIFY_OK;
				activate_start = *cycles;
				*gpregret = 0xD2;
			}
			continue;
		}

		if (status == OTA_STATUS_VERIFY_OK) {
			if (valid_ack && rx_len >= 2 && rx_buf[2] == ESB_OTA_ACTIVATE_TYPE &&
			    rx_buf[3] == p->tracker_id) {
				status = OTA_STATUS_ACTIVATING;
				terminal_rounds = 0;
				*gpregret = 0xD3;
			}
			continue;
		}

		terminal_rounds++;
		if (status == OTA_STATUS_VERIFY_FAIL && terminal_rounds >= 50) {
			goto recover;
		}
		if (status == OTA_STATUS_ACTIVATING && terminal_rounds >= 3) {
			*gpregret = 0xD4;
			if (p->settings_addr != 0) {
				uint16_t crc = 0xFFFF;
				const volatile uint8_t *flash = (const volatile uint8_t *)p->flash_target;
				for (uint32_t i = 0; i < p->image_size; i++) {
					if ((i & 4095u) == 0) {
						RAM_FEED();
					}
					crc = (uint8_t)(crc >> 8) | (crc << 8);
					crc ^= flash[i];
					crc ^= (uint8_t)(crc & 0xFF) >> 4;
					crc ^= (crc << 8) << 4;
					crc ^= ((crc & 0xFF) << 4) << 1;
				}
				*gpregret = 0xD5;
				RAM_REG(nvmc, 0x504) = 2;
				RAM_DSB();
				RAM_REG(nvmc, 0x508) = p->settings_addr;
				RAM_WAIT(RAM_REG(nvmc, 0x400), 500);
				*gpregret = 0xD6;
				RAM_REG(nvmc, 0x504) = 1;
				RAM_DSB();
				volatile uint32_t *settings = (volatile uint32_t *)p->settings_addr;
				for (uint32_t w = 1; w < 7; w++) {
					settings[w] = w == 1 ? BANK_INVALID_APP : w == 2 ? p->image_size : 0;
					RAM_WAIT(RAM_REG(nvmc, 0x400), 1);
				}
				/* Final commit marker, after every metadata word. */
				uint32_t first_word = (uint32_t)crc << 16 | BANK_VALID_APP;
				settings[0] = first_word;
				RAM_WAIT(RAM_REG(nvmc, 0x400), 1);
				RAM_REG(nvmc, 0x504) = 0;
				RAM_DSB();
				if (settings[0] != first_word || settings[1] != BANK_INVALID_APP ||
				    settings[2] != p->image_size) {
					goto recover;
				}
				for (uint32_t w = 3; w < 7; w++) {
					if (settings[w] != 0) {
						goto recover;
					}
				}
				*gpregret = 0xD7;
			}
			status = OTA_STATUS_COMPLETE;
			terminal_rounds = 0;
			*gpregret = 0xD8;
			continue;
		}
		if (status == OTA_STATUS_COMPLETE && terminal_rounds >= 5) {
			*gpregret = 0xDE;
			if (!ota_ram_radio_disable(radio)) {
				goto recover;
			}
			goto reset;
		}
		/* Space terminal repeats without compiler/ACK-length-dependent delays. */
		uint32_t pause_start = *cycles;
		while ((uint32_t)(*cycles - pause_start) < 5000u * 64u) {
			RAM_FEED();
		}
	}

recover:
	*gpregret = 0x57; /* Existing Adafruit UF2 recovery request. */
reset:
	*aircr = (0x5FAu << 16) | (1u << 2);
	RAM_DSB();
	for (;;) {
	}

#undef RAM_WAIT
#undef RAM_DSB
#undef RAM_FEED
#undef RAM_REG
}
