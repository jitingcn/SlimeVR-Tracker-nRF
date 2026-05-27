/*
 * OTA RAM Engine — Bare-Metal ESB PTX + Flash Writer
 *
 * A self-contained function that runs entirely from RAM, performing:
 *   1. ESB PTX polling (send STATUS, receive DATA via ACK payload)
 *   2. Direct NVMC flash erase/write to target address
 *   3. CRC32 verification of written data
 *   4. System reset
 *
 * This enables in-place OTA updates on devices with limited flash
 * (e.g. nRF52833 with 512KB) where dual-bank staging is not possible.
 *
 * The function uses NO external calls — all register access is done
 * directly through hardware register pointers.
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
	uint32_t flash_target;       /* Target flash address (e.g. 0x27000 or 0x1000) */
	uint32_t page_size;          /* Flash page size (4096) */
	uint16_t next_expected_seq;  /* Next sequence number to expect */
	uint32_t bytes_received;     /* Bytes received so far */
	uint8_t  tracker_id;         /* Tracker ID for status packets */

	/* Bootloader settings */
	uint32_t settings_addr;      /* Bootloader settings page address (0 = skip) */

	/* External page buffer (must be 4KB, 4-byte aligned, in RAM) */
	uint8_t *page_buf;
};

/*
 * ota_ram_engine() — The main RAM-resident OTA engine
 *
 * MUST be compiled with __attribute__((noinline)) and copied to RAM.
 * MUST NOT call any external functions.
 *
 * Packet formats:
 *   TX (STATUS): [0x21][id][status][seq_hi][seq_lo][bytes_LE32][0][0][0][crc8]  (13 bytes)
 *   RX (DATA):   [0x20][id][seq_hi][seq_lo][44 bytes payload]                   (48 bytes)
 *
 * ESB DPL packet buffer layout (for RADIO DMA):
 *   Byte 0: [length:6/8][rfu]  — LENGTH field (6 bits if max<=32, else 8 bits)
 *   Byte 1: [noack:1][pid:2][rfu:5]  — S1 field
 *   Byte 2..N: payload data
 */
__attribute__((noinline, used))
void ota_ram_engine(const struct ota_ram_engine_params *p)
{
	/* ── Hardware register base addresses ────────────────────────── */
	volatile uint32_t *RADIO   = (volatile uint32_t *)0x40001000;
	volatile uint32_t *NVMC    = (volatile uint32_t *)0x4001E000;
	volatile uint32_t *WDT_RR0 = (volatile uint32_t *)0x40010600;
	volatile uint32_t *CLOCK   = (volatile uint32_t *)0x40000000;

	/* GPREGRET: survives system reset, used for progress tracking */
	volatile uint32_t *GPREGRET = (volatile uint32_t *)0x4000051C;

	/* CLOCK register offsets */
	#define C_HFCLKSTART    (0x000/4)  /* TASKS_HFCLKSTART */
	#define C_HFCLKSTARTED  (0x100/4)  /* EVENTS_HFCLKSTARTED */

	/* RADIO register offsets (word index = byte_offset / 4) */
	/* Tasks */
	#define R_TXEN        (0x000/4)  /* TASKS_TXEN */
	#define R_RXEN        (0x004/4)  /* TASKS_RXEN */
	#define R_START       (0x008/4)  /* TASKS_START */
	#define R_STOP        (0x00C/4)  /* TASKS_STOP */
	#define R_DISABLE     (0x010/4)  /* TASKS_DISABLE */
	/* Events */
	#define R_READY       (0x100/4)  /* EVENTS_READY */
	#define R_ADDRESS     (0x104/4)  /* EVENTS_ADDRESS */
	#define R_PAYLOAD     (0x108/4)  /* EVENTS_PAYLOAD */
	#define R_END         (0x10C/4)  /* EVENTS_END */
	#define R_DISABLED    (0x110/4)  /* EVENTS_DISABLED */
	#define R_CRCOK       (0x130/4)  /* EVENTS_CRCOK */
	#define R_CRCERROR    (0x134/4)  /* EVENTS_CRCERROR */
	/* Registers */
	#define R_SHORTS      (0x200/4)
	/* SHORTS bit positions (nRF52840) */
	#define SHORT_READY_START    (1 << 0)
	#define SHORT_END_DISABLE    (1 << 1)
	#define SHORT_DISABLED_TXEN  (1 << 2)
	#define SHORT_DISABLED_RXEN  (1 << 3)
	#define R_PACKETPTR   (0x504/4)
	#define R_FREQUENCY   (0x508/4)
	#define R_TXPOWER     (0x50C/4)
	#define R_MODE        (0x510/4)
	#define R_PCNF0       (0x514/4)
	#define R_PCNF1       (0x518/4)
	#define R_BASE0       (0x51C/4)
	#define R_BASE1       (0x520/4)
	#define R_PREFIX0     (0x524/4)
	#define R_PREFIX1     (0x528/4)
	#define R_TXADDRESS   (0x52C/4)
	#define R_RXADDRESSES (0x530/4)
	#define R_CRCCNF      (0x534/4)
	#define R_CRCPOLY     (0x538/4)
	#define R_CRCINIT     (0x53C/4)
	#define R_STATE       (0x550/4)

	/* NVMC register offsets */
	#define N_READY       (0x400/4)
	#define N_CONFIG      (0x504/4)
	#define N_ERASEPAGE   (0x508/4)

	/* ── Local state (all in stack / registers) ──────────────────── */
	uint16_t next_seq = p->next_expected_seq;
	uint32_t bytes_received = p->bytes_received;
	uint32_t target_addr = p->flash_target + bytes_received;
	uint8_t  pid = 0;  /* ESB packet ID (2-bit, cycles 0-3) */

	/* Page buffer from caller (4KB, avoids stack overflow) */
	uint8_t *page_buf = p->page_buf;
	uint32_t page_buf_offset = bytes_received & (p->page_size - 1);
	uint32_t page_addr = target_addr & ~(p->page_size - 1);

	/* Fill page_buf with 0xFF (erased state)
	 * Use volatile cast to prevent compiler from optimizing to memset() */
	{
		volatile uint8_t *pb = page_buf;
		for (uint32_t i = 0; i < p->page_size; i++) {
			pb[i] = 0xFF;
		}
	}

	/* ESB radio packet buffers (DPL format) */
	/* TX: LENGTH(1) + S1(1) + payload(13) = 15 bytes */
	uint8_t __attribute__((aligned(4))) tx_buf[64];
	/* RX: LENGTH(1) + S1(1) + payload(64) = 66 bytes */
	uint8_t __attribute__((aligned(4))) rx_buf[68];

	/* ── Ensure HFXO (32 MHz crystal) is running ────────────────── */
	/* RADIO requires HFXO for proper frequency accuracy.
	 * Without USB, Zephyr's clock management may have stopped HFXO
	 * after the last ESB transmission (power saving). */
	CLOCK[C_HFCLKSTARTED] = 0;
	CLOCK[C_HFCLKSTART] = 1;
	while (!CLOCK[C_HFCLKSTARTED]) {
		*WDT_RR0 = 0x6E524635;
	}

	/* ── Configure RADIO ─────────────────────────────────────────── */

	/* First disable radio cleanly (if not already disabled) */
	/* STATE: 0=Disabled, 1=RxRU, 2=RxIdle, 3=Rx, ... 9=TxRU, 10=TxIdle, 11=Tx ... */
	if (RADIO[R_STATE] != 0) {
		RADIO[R_DISABLED] = 0;
		RADIO[R_DISABLE] = 1;
		while (!RADIO[R_DISABLED]) {}
	}
	RADIO[R_DISABLED] = 0;

	/* Restore configuration from params */
	RADIO[R_MODE]        = p->radio_mode;
	RADIO[R_PCNF0]       = p->radio_pcnf0;
	RADIO[R_PCNF1]       = p->radio_pcnf1;
	RADIO[R_CRCCNF]      = p->radio_crccnf;
	RADIO[R_CRCPOLY]     = p->radio_crcpoly;
	RADIO[R_CRCINIT]     = p->radio_crcinit;
	RADIO[R_BASE0]       = p->radio_base0;
	RADIO[R_BASE1]       = p->radio_base1;
	RADIO[R_PREFIX0]     = p->radio_prefix0;
	RADIO[R_PREFIX1]     = p->radio_prefix1;
	RADIO[R_TXADDRESS]   = p->radio_txaddress;
	RADIO[R_RXADDRESSES] = p->radio_rxaddresses;
	RADIO[R_TXPOWER]     = p->radio_txpower;
	RADIO[R_FREQUENCY]   = p->radio_frequency;

	/* Enable fast ramp-up (40µs instead of 130µs) — must match PRX config */
	#define R_MODECNF0    (0x650/4)
	RADIO[R_MODECNF0] = 1;  /* RU=Fast */

	/* ── Inline CRC-8 CCITT helper (bit-by-bit) ─────────────────── */
	/* Used for STATUS packet CRC — cannot call external crc8() */
	#define CRC8_INIT 0xFF
	#define CRC8_POLY 0x8D  /* x^8+x^7+x^3+x^2+1 reversed: CCITT-like */

	/* ── Main OTA receive loop ───────────────────────────────────── */
	*GPREGRET = 0xD0;  /* Entering main receive loop */

	uint32_t idle_count = 0;
	uint32_t max_retries = 0;  /* Track consecutive no-data rounds */
	/* Idle timeout: ~10 seconds at ~500 Hz poll rate (2ms/cycle) */
	#define IDLE_TIMEOUT_CYCLES 5000

	while (bytes_received < p->image_size) {
		/* Feed WDT */
		*WDT_RR0 = 0x6E524635;

		/* Check idle timeout — abort if no data for ~10 seconds */
		if (idle_count > IDLE_TIMEOUT_CYCLES) {
			/* Reset to UF2 bootloader */
			((volatile uint32_t *)0x40000000)[0x51C/4] = 0x57;
			volatile uint32_t *SCB_AIRCR = (volatile uint32_t *)0xE000ED0C;
			*SCB_AIRCR = (0x5FA << 16) | (1 << 2);
			__asm volatile("dsb");
			for (;;) {}
		}

		/* ── Build STATUS packet payload ────────────────────────── */
		uint8_t status_payload[13];
		status_payload[0] = 0x21;  /* ESB_OTA_STATUS_TYPE */
		status_payload[1] = p->tracker_id;
		status_payload[2] = 0x02;  /* OTA_STATUS_RECEIVING */
		status_payload[3] = (uint8_t)(next_seq >> 8);
		status_payload[4] = (uint8_t)(next_seq & 0xFF);
		/* bytes_received as LE32 */
		status_payload[5] = (uint8_t)(bytes_received);
		status_payload[6] = (uint8_t)(bytes_received >> 8);
		status_payload[7] = (uint8_t)(bytes_received >> 16);
		status_payload[8] = (uint8_t)(bytes_received >> 24);
		status_payload[9] = 0;
		status_payload[10] = 0;
		status_payload[11] = 0;
		/* CRC-8 over bytes 0..11 */
		{
			uint8_t crc = 0x00;
			for (uint32_t i = 0; i < 12; i++) {
				crc ^= status_payload[i];
				for (uint32_t b = 0; b < 8; b++) {
					if (crc & 0x80)
						crc = (crc << 1) ^ 0x07;
					else
						crc = crc << 1;
				}
			}
			status_payload[12] = crc;
		}

		/* ── Build TX radio buffer (DPL format) ─────────────────── */
		/* Byte 0: length field (6 or 8 bits, value = payload length) */
		tx_buf[0] = 13;  /* STATUS packet is 13 bytes */
		/* Byte 1: S1 field — [noack:1][pid:2]
		 * In ESB-DPL, no_ack=1 means "request ACK" (inverted naming convention).
		 * See esb.c line 1272: pdu.no_ack = payload->noack ? 0x00 : 0x01 */
		tx_buf[1] = ((pid & 0x03) << 1) | 0x01;  /* PID in bits [2:1], noack=1 (request ACK) */
		/* Byte 2+: payload */
		for (uint32_t i = 0; i < 13; i++) {
			tx_buf[2 + i] = status_payload[i];
		}

		/* ── TX phase: send STATUS, auto-switch to RX for ACK ───── */
		/* Clear events */
		RADIO[R_READY] = 0;
		RADIO[R_END] = 0;
		RADIO[R_DISABLED] = 0;
		RADIO[R_CRCOK] = 0;
		RADIO[R_CRCERROR] = 0;

		/* Set packet pointer for TX */
		RADIO[R_PACKETPTR] = (uint32_t)tx_buf;
		/* SHORTS: READY_START | END_DISABLE | DISABLED_RXEN
		 * TX done → DISABLE → RXEN auto (ramp-up gives time to switch PACKETPTR) */
		RADIO[R_SHORTS] = SHORT_READY_START | SHORT_END_DISABLE | SHORT_DISABLED_RXEN;

		/* Start TX */
		RADIO[R_TXEN] = 1;

		/* Wait for TX DISABLED (TX done, RXEN auto-triggered by short) */
		while (!RADIO[R_DISABLED]) {
			*WDT_RR0 = 0x6E524635; /* Feed WDT while waiting */
		}
		RADIO[R_DISABLED] = 0;

		/* ── RX phase: receive ACK ──────────────────────────────── */
		/* RXEN already triggered by DISABLED_RXEN short. Switch PACKETPTR and SHORTS. */
		RADIO[R_PACKETPTR] = (uint32_t)rx_buf;
		RADIO[R_SHORTS] = SHORT_READY_START | SHORT_END_DISABLE;
		RADIO[R_CRCOK] = 0;
		RADIO[R_CRCERROR] = 0;

		/* Wait for ACK with timeout. */
		uint32_t timeout = 5000;  /* ~500us timeout — enough for ESB ACK turnaround */
		bool got_packet = false;
		while (timeout--) {
			if (RADIO[R_DISABLED]) {
				/* Packet received (or timeout via SHORT) */
				got_packet = true;
				break;
			}
		}

		if (!got_packet) {
			/* Timeout — disable radio manually */
			RADIO[R_DISABLE] = 1;
			while (!RADIO[R_DISABLED]) {}
		}
		RADIO[R_DISABLED] = 0;

		/* ── Check if we got a valid ACK with data ──────────────── */
		/* Memory barrier: rx_buf is written by radio DMA */
		__asm volatile("" ::: "memory");
		if (got_packet && RADIO[R_CRCOK]) {
			/* Parse DPL: rx_buf[0] = length, rx_buf[1] = S1, rx_buf[2+] = payload */
			uint8_t rx_len = rx_buf[0];

			/* Check if this is a DATA packet (type 0x20, length >= 5) */
			if (rx_len >= 5 && rx_buf[2] == 0x20) {
				uint16_t seq = ((uint16_t)rx_buf[4] << 8) | rx_buf[5];

				if (seq == next_seq) {
					/* Data payload starts at rx_buf[6], length = rx_len - 4 */
					uint32_t data_len = rx_len - 4;
					if (data_len > 60) data_len = 60;

					/* Copy to page buffer, handling page boundary splits */
					uint32_t data_off = 0;
					while (data_off < data_len) {
						if (page_buf_offset < p->page_size) {
							page_buf[page_buf_offset++] = rx_buf[6 + data_off];
							data_off++;
						}

						/* If page buffer is full or all data received, write to flash */
						if (page_buf_offset >= p->page_size ||
						    (bytes_received + data_off) >= p->image_size) {
							/* Feed WDT before flash ops */
							*WDT_RR0 = 0x6E524635;

							/* Compare page buffer with existing flash content.
							 * Skip erase+write if identical — saves flash wear
							 * and speeds up re-flashing same/similar firmware. */
							const volatile uint32_t *flash_cmp = (const volatile uint32_t *)page_addr;
							uint32_t *buf_cmp = (uint32_t *)page_buf;
							uint32_t cmp_words = p->page_size / 4;
							bool page_match = true;
							for (uint32_t w = 0; w < cmp_words; w++) {
								if (flash_cmp[w] != buf_cmp[w]) {
									page_match = false;
									break;
								}
							}

							if (!page_match) {
							/* Erase page */
							NVMC[N_CONFIG] = 2;  /* Een */
							__asm volatile("dsb");
							NVMC[N_ERASEPAGE] = page_addr;
							while (!NVMC[N_READY]) {}

							/* Write page */
							NVMC[N_CONFIG] = 1;  /* Wen */
							__asm volatile("dsb");

							uint32_t write_words = (page_buf_offset + 3) / 4;
							volatile uint32_t *dst = (volatile uint32_t *)page_addr;
							uint32_t *src = (uint32_t *)page_buf;
							for (uint32_t w = 0; w < write_words; w++) {
								dst[w] = src[w];
								while (!NVMC[N_READY]) {}
							}

							NVMC[N_CONFIG] = 0;  /* Ren */
							__asm volatile("dsb");
							} /* end if (!page_match) */

							/* Advance to next page */
							page_addr += p->page_size;
							page_buf_offset = 0;
							/* Clear page buffer */
							{
								volatile uint8_t *pb = page_buf;
								for (uint32_t i = 0; i < p->page_size; i++) {
									pb[i] = 0xFF;
								}
							}
						}
					}
					bytes_received += data_len;
					next_seq++;

					idle_count = 0;
					max_retries = 0;
				}
				/* else: duplicate or out-of-order, just ignore */
			}
			/* else: empty ACK or non-DATA packet, continue polling */
		}

		/* Increment PID for next transmission */
		pid = (pid + 1) & 0x03;

		idle_count++;
	}

	/* ── All data received — verify CRC32 ────────────────────────── */
	*GPREGRET = 0xD1;  /* All data received, starting CRC32 */

	/* CRC32-IEEE (same as zlib/crc32_ieee) */
	/* Polynomial: 0xEDB88320 (bit-reversed 0x04C11DB7) */
	uint32_t crc = 0xFFFFFFFF;
	uint32_t remaining = p->image_size;
	uint32_t addr = p->flash_target;

	while (remaining > 0) {
		/* Feed WDT */
		*WDT_RR0 = 0x6E524635;

		uint32_t chunk = remaining;
		if (chunk > p->page_size) chunk = p->page_size;

		/* Read flash into page_buf */
		const volatile uint8_t *flash = (const volatile uint8_t *)addr;
		for (uint32_t i = 0; i < chunk; i++) {
			page_buf[i] = flash[i];
		}

		/* Update CRC32 */
		for (uint32_t i = 0; i < chunk; i++) {
			crc ^= page_buf[i];
			for (uint32_t b = 0; b < 8; b++) {
				if (crc & 1)
					crc = (crc >> 1) ^ 0xEDB88320;
				else
					crc = crc >> 1;
			}
		}

		addr += chunk;
		remaining -= chunk;
	}
	crc ^= 0xFFFFFFFF;

	/* If CRC doesn't match, we're in trouble — can't really recover.
	 * Try sending error status for a while, then reset to bootloader. */
	if (crc != p->image_crc32) {
		/* Send error status with TX+RX for reliability */
		for (uint32_t retry = 0; retry < 50; retry++) {
			*WDT_RR0 = 0x6E524635;

			tx_buf[0] = 13;
			tx_buf[1] = ((pid & 0x03) << 1) | 0x01; /* noack=1 = request ACK */
			uint8_t *sp = &tx_buf[2];
			sp[0] = 0x21;
			sp[1] = p->tracker_id;
			sp[2] = 0x04;  /* OTA_STATUS_VERIFY_FAIL */
			sp[3] = (uint8_t)(next_seq >> 8);
			sp[4] = (uint8_t)(next_seq);
			/* Pack computed CRC for debug */
			sp[5] = (uint8_t)(crc);
			sp[6] = (uint8_t)(crc >> 8);
			sp[7] = (uint8_t)(crc >> 16);
			sp[8] = (uint8_t)(crc >> 24);
			/* Pack expected CRC */
			sp[9] = (uint8_t)(p->image_crc32);
			sp[10] = (uint8_t)(p->image_crc32 >> 8);
			sp[11] = (uint8_t)(p->image_crc32 >> 16);
			{
				uint8_t c = 0;
				for (uint32_t i = 0; i < 12; i++) {
					c ^= sp[i];
					for (uint32_t b = 0; b < 8; b++) {
						if (c & 0x80) c = (c << 1) ^ 0x07;
						else c = c << 1;
					}
				}
				sp[12] = c;
			}

			/* TX+RX cycle */
			RADIO[R_READY] = 0;
			RADIO[R_END] = 0;
			RADIO[R_DISABLED] = 0;
			RADIO[R_CRCOK] = 0;
			RADIO[R_CRCERROR] = 0;

			RADIO[R_PACKETPTR] = (uint32_t)tx_buf;
			RADIO[R_SHORTS] = SHORT_READY_START | SHORT_END_DISABLE | SHORT_DISABLED_RXEN;
			RADIO[R_TXEN] = 1;

			while (!RADIO[R_DISABLED]) {
				*WDT_RR0 = 0x6E524635;
			}
			RADIO[R_DISABLED] = 0;

			/* RX ACK (brief wait) */
			RADIO[R_PACKETPTR] = (uint32_t)rx_buf;
			RADIO[R_SHORTS] = SHORT_READY_START | SHORT_END_DISABLE;

			uint32_t rx_timeout = 5000;
			while (rx_timeout--) {
				if (RADIO[R_DISABLED]) break;
			}
			if (!RADIO[R_DISABLED]) {
				RADIO[R_DISABLE] = 1;
				while (!RADIO[R_DISABLED]) {}
			}
			RADIO[R_DISABLED] = 0;
			pid = (pid + 1) & 0x03;

			for (volatile uint32_t d = 0; d < 50000; d++) {}
		}

		/* Reset to UF2 bootloader */
		/* GPREGRET = 0x57 (ADAFRUIT_DFU_MAGIC_UF2_RESET) */
		((volatile uint32_t *)0x40000000)[0x51C/4] = 0x57;

		/* System reset */
		volatile uint32_t *SCB_AIRCR = (volatile uint32_t *)0xE000ED0C;
		*SCB_AIRCR = (0x5FA << 16) | (1 << 2);
		__asm volatile("dsb");
		for (;;) {}
	}

	/* ── CRC OK — Poll with VERIFY_OK status, wait for ACTIVATE ── */
	*GPREGRET = 0xD2;  /* CRC OK, waiting for ACTIVATE */
	/* The PC sends ACTIVATE after seeing VERIFY_OK.
	 * We poll with TX+RX (like the main receive loop) to get the
	 * ACTIVATE command via ACK payload. Timeout after ~30 seconds. */
	#define ACTIVATE_TIMEOUT_CYCLES 15000  /* ~30s at 2ms/cycle */
	uint32_t activate_wait = 0;
	bool activated = false;

	while (!activated && activate_wait < ACTIVATE_TIMEOUT_CYCLES) {
		*WDT_RR0 = 0x6E524635;

		/* Build VERIFY_OK status packet */
		tx_buf[0] = 13;
		tx_buf[1] = ((pid & 0x03) << 1) | 0x01;
		uint8_t *sp = &tx_buf[2];
		sp[0] = 0x21;  /* ESB_OTA_STATUS_TYPE */
		sp[1] = p->tracker_id;
		sp[2] = 0x03;  /* OTA_STATUS_VERIFY_OK */
		sp[3] = (uint8_t)(next_seq >> 8);
		sp[4] = (uint8_t)(next_seq);
		sp[5] = (uint8_t)(bytes_received);
		sp[6] = (uint8_t)(bytes_received >> 8);
		sp[7] = (uint8_t)(bytes_received >> 16);
		sp[8] = (uint8_t)(bytes_received >> 24);
		sp[9] = 0; sp[10] = 0; sp[11] = 0;
		{
			uint8_t c = 0;
			for (uint32_t i = 0; i < 12; i++) {
				c ^= sp[i];
				for (uint32_t b = 0; b < 8; b++) {
					if (c & 0x80) c = (c << 1) ^ 0x07;
					else c = c << 1;
				}
			}
			sp[12] = c;
		}

		/* TX+RX cycle: send VERIFY_OK, receive ACK with command */
		RADIO[R_READY] = 0;
		RADIO[R_END] = 0;
		RADIO[R_DISABLED] = 0;
		RADIO[R_CRCOK] = 0;
		RADIO[R_CRCERROR] = 0;

		RADIO[R_PACKETPTR] = (uint32_t)tx_buf;
		RADIO[R_SHORTS] = SHORT_READY_START | SHORT_END_DISABLE | SHORT_DISABLED_RXEN;
		RADIO[R_TXEN] = 1;

		/* Wait for TX done (auto-switches to RX) */
		while (!RADIO[R_DISABLED]) {
			*WDT_RR0 = 0x6E524635;
		}
		RADIO[R_DISABLED] = 0;

		/* RX ACK */
		RADIO[R_PACKETPTR] = (uint32_t)rx_buf;
		RADIO[R_SHORTS] = SHORT_READY_START | SHORT_END_DISABLE;
		RADIO[R_CRCOK] = 0;
		RADIO[R_CRCERROR] = 0;

		uint32_t rx_timeout = 5000;
		bool got_ack = false;
		while (rx_timeout--) {
			if (RADIO[R_DISABLED]) {
				got_ack = true;
				break;
			}
		}

		if (!got_ack) {
			RADIO[R_DISABLE] = 1;
			while (!RADIO[R_DISABLED]) {}
		}
		RADIO[R_DISABLED] = 0;

		/* Check ACK payload for ACTIVATE command (type 0x25) */
		/* Memory barrier: rx_buf is written by radio DMA, compiler
		 * must reload from memory, not use cached register values. */
		__asm volatile("" ::: "memory");
		if (got_ack && RADIO[R_CRCOK]) {
			uint8_t rx_len = rx_buf[0];
			if (rx_len >= 2 && rx_buf[2] == 0x25) {
				activated = true;
			}
			/* VERIFY command (0x24): keep polling (resend VERIFY_OK) */
			/* DATA or other: ignore, keep polling */
		}

		pid = (pid + 1) & 0x03;
		activate_wait++;
	}

	if (!activated) {
		/* Timeout — reset to UF2 bootloader */
		((volatile uint32_t *)0x40000000)[0x51C/4] = 0x57;
		volatile uint32_t *SCB_AIRCR = (volatile uint32_t *)0xE000ED0C;
		*SCB_AIRCR = (0x5FA << 16) | (1 << 2);
		__asm volatile("dsb");
		for (;;) {}
	}

	/* ── Send ACTIVATING status to confirm ACTIVATE receipt ──────── */
	*GPREGRET = 0xD3;  /* Sending ACTIVATING */
	/* Just 3 sends with spacing — enough for receiver to confirm.
	 * Don't flood: receiver's ESB FIFO is limited (~8 entries). */
	for (uint32_t ack_retry = 0; ack_retry < 3; ack_retry++) {
		*WDT_RR0 = 0x6E524635;

		tx_buf[0] = 13;
		tx_buf[1] = ((pid & 0x03) << 1) | 0x01;
		uint8_t *sp = &tx_buf[2];
		sp[0] = 0x21;
		sp[1] = p->tracker_id;
		sp[2] = 0x05;  /* OTA_STATUS_ACTIVATING */
		sp[3] = (uint8_t)(next_seq >> 8);
		sp[4] = (uint8_t)(next_seq);
		sp[5] = (uint8_t)(bytes_received);
		sp[6] = (uint8_t)(bytes_received >> 8);
		sp[7] = (uint8_t)(bytes_received >> 16);
		sp[8] = (uint8_t)(bytes_received >> 24);
		sp[9] = 0; sp[10] = 0; sp[11] = 0;
		{
			uint8_t c = 0;
			for (uint32_t i = 0; i < 12; i++) {
				c ^= sp[i];
				for (uint32_t b = 0; b < 8; b++) {
					if (c & 0x80) c = (c << 1) ^ 0x07;
					else c = c << 1;
				}
			}
			sp[12] = c;
		}

		RADIO[R_READY] = 0;
		RADIO[R_END] = 0;
		RADIO[R_DISABLED] = 0;
		RADIO[R_CRCOK] = 0;
		RADIO[R_CRCERROR] = 0;

		RADIO[R_PACKETPTR] = (uint32_t)tx_buf;
		RADIO[R_SHORTS] = SHORT_READY_START | SHORT_END_DISABLE | SHORT_DISABLED_RXEN;
		RADIO[R_TXEN] = 1;

		while (!RADIO[R_DISABLED]) {
			*WDT_RR0 = 0x6E524635;
		}
		RADIO[R_DISABLED] = 0;

		RADIO[R_PACKETPTR] = (uint32_t)rx_buf;
		RADIO[R_SHORTS] = SHORT_READY_START | SHORT_END_DISABLE;

		uint32_t rx_timeout = 5000;
		while (rx_timeout--) {
			if (RADIO[R_DISABLED]) break;
		}
		if (!RADIO[R_DISABLED]) {
			RADIO[R_DISABLE] = 1;
			while (!RADIO[R_DISABLED]) {}
		}
		RADIO[R_DISABLED] = 0;

		pid = (pid + 1) & 0x03;
		/* Longer delay to let receiver process */
		for (volatile uint32_t d = 0; d < 200000; d++) {}
	}

	/* ── Compute CRC16 and write bootloader settings ────────────── */
	/* DISABLED: settings write crashes at 0xFF000 for unknown reason.
	 * Bootloader boots the app without valid settings anyway.
	 * Kept as reference for future investigation. */
#if 0
	*GPREGRET = 0xD4;  /* Starting CRC16 + settings write */
	if (p->settings_addr != 0) {
		*WDT_RR0 = 0x6E524635;

		/* Compute Nordic SDK CRC-16 over written firmware */
		uint16_t crc16 = 0xFFFF;
		uint32_t rem16 = p->image_size;
		uint32_t addr16 = p->flash_target;
		while (rem16 > 0) {
			*WDT_RR0 = 0x6E524635;
			uint32_t chunk16 = rem16;
			if (chunk16 > p->page_size) chunk16 = p->page_size;
			const volatile uint8_t *flash16 = (const volatile uint8_t *)addr16;
			for (uint32_t i = 0; i < chunk16; i++) {
				crc16 = (uint8_t)(crc16 >> 8) | (crc16 << 8);
				crc16 ^= flash16[i];
				crc16 ^= (uint8_t)(crc16 & 0xFF) >> 4;
				crc16 ^= crc16 << 12;
				crc16 ^= (crc16 & 0xFF) << 5;
			}
			addr16 += chunk16;
			rem16 -= chunk16;
		}

		/* Build bootloader settings struct (28 bytes = 7 words):
		 *   u16 bank_0 = 0x01 (BANK_VALID_APP)
		 *   u16 bank_0_crc = crc16
		 *   u16 bank_1 = 0x00 (BANK_INVALID_APP)
		 *   u16 padding = 0
		 *   u32 bank_0_size = image_size
		 *   u32 sd_image_size = 0
		 *   u32 bl_image_size = 0
		 *   u32 app_image_size = 0
		 *   u32 sd_image_start = 0
		 */
		uint32_t settings[7];
		settings[0] = (uint32_t)crc16 << 16 | 0x0001;  /* bank_0_crc:bank_0 */
		settings[1] = 0x00000000;  /* padding:bank_1 */
		settings[2] = p->image_size;  /* bank_0_size */
		settings[3] = 0;  /* sd_image_size */
		settings[4] = 0;  /* bl_image_size */
		settings[5] = 0;  /* app_image_size */
		settings[6] = 0;  /* sd_image_start */

		/* Erase settings page */
		*GPREGRET = 0xD5;  /* Erasing settings page */
		NVMC[N_CONFIG] = 2;  /* Een */
		__asm volatile("dsb");
		NVMC[N_ERASEPAGE] = p->settings_addr;
		while (!NVMC[N_READY]) {}

		/* Write settings */
		*GPREGRET = 0xD6;  /* Writing settings words */
		NVMC[N_CONFIG] = 1;  /* Wen */
		__asm volatile("dsb");
		volatile uint32_t *dst = (volatile uint32_t *)p->settings_addr;
		for (uint32_t w = 0; w < 7; w++) {
			dst[w] = settings[w];
			while (!NVMC[N_READY]) {}
		}

		NVMC[N_CONFIG] = 0;  /* Ren */
		__asm volatile("dsb");
		*GPREGRET = 0xD7;  /* Settings write complete */
	}
#endif

	/* ── Send COMPLETE status with TX+RX, then reset ────────────── */
	*GPREGRET = 0xD8;  /* Sending COMPLETE */
	/* 5 sends with generous spacing — receiver must forward to PC */
	for (uint32_t retry = 0; retry < 5; retry++) {
		*WDT_RR0 = 0x6E524635;

		tx_buf[0] = 13;
		tx_buf[1] = ((pid & 0x03) << 1) | 0x01;
		uint8_t *sp = &tx_buf[2];
		sp[0] = 0x21;
		sp[1] = p->tracker_id;
		sp[2] = 0x06;  /* OTA_STATUS_COMPLETE */
		sp[3] = (uint8_t)(next_seq >> 8);
		sp[4] = (uint8_t)(next_seq);
		sp[5] = (uint8_t)(bytes_received);
		sp[6] = (uint8_t)(bytes_received >> 8);
		sp[7] = (uint8_t)(bytes_received >> 16);
		sp[8] = (uint8_t)(bytes_received >> 24);
		sp[9] = 0; sp[10] = 0; sp[11] = 0;
		{
			uint8_t c = 0;
			for (uint32_t i = 0; i < 12; i++) {
				c ^= sp[i];
				for (uint32_t b = 0; b < 8; b++) {
					if (c & 0x80) c = (c << 1) ^ 0x07;
					else c = c << 1;
				}
			}
			sp[12] = c;
		}

		/* TX+RX cycle for reliability */
		RADIO[R_READY] = 0;
		RADIO[R_END] = 0;
		RADIO[R_DISABLED] = 0;
		RADIO[R_CRCOK] = 0;
		RADIO[R_CRCERROR] = 0;

		RADIO[R_PACKETPTR] = (uint32_t)tx_buf;
		RADIO[R_SHORTS] = SHORT_READY_START | SHORT_END_DISABLE | SHORT_DISABLED_RXEN;
		RADIO[R_TXEN] = 1;

		while (!RADIO[R_DISABLED]) {
			*WDT_RR0 = 0x6E524635;
		}
		RADIO[R_DISABLED] = 0;

		/* RX ACK (brief wait, don't need the data) */
		RADIO[R_PACKETPTR] = (uint32_t)rx_buf;
		RADIO[R_SHORTS] = SHORT_READY_START | SHORT_END_DISABLE;
		RADIO[R_CRCOK] = 0;

		uint32_t rx_timeout = 5000;
		while (rx_timeout--) {
			if (RADIO[R_DISABLED]) break;
		}
		if (!RADIO[R_DISABLED]) {
			RADIO[R_DISABLE] = 1;
			while (!RADIO[R_DISABLED]) {}
		}
		RADIO[R_DISABLED] = 0;

		pid = (pid + 1) & 0x03;

		/* 5ms spacing to let receiver event handler process + forward */
		for (volatile uint32_t d = 0; d < 300000; d++) {}
	}

	/* Disable radio */
	*GPREGRET = 0xDE;  /* COMPLETE sends done, resetting */
	RADIO[R_DISABLE] = 1;
	while (!RADIO[R_DISABLED]) {}

	/* System reset */
	volatile uint32_t *SCB_AIRCR = (volatile uint32_t *)0xE000ED0C;
	*SCB_AIRCR = (0x5FA << 16) | (1 << 2);
	__asm volatile("dsb");
	for (;;) {}
}
