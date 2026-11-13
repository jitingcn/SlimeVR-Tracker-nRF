/* Raw RADIO receive windows borrowed from an idle ESB PTX instance.
 *
 * ESB's PTX only receives ACK payloads, and switching ESB into PRX/monitor
 * mode re-initialises it (flushing FIFOs and PIDs), far too heavy for a window
 * every frame. Instead, while ESB is idle and the caller holds the esb.c raw
 * RX arbitration (esb_raw_rx_begin/end), this module reuses ESB's packet
 * configuration (addresses, DPL format, CRC, bitrate) and drives RX directly
 * with its interrupt sources masked, restoring everything it touches.
 */
#ifndef SLIMENRF_RELAY_RADIO_H
#define SLIMENRF_RELAY_RADIO_H

#include <stdbool.h>
#include <stdint.h>

#include "relay_protocol.h"

struct relay_rx_frame {
	uint8_t data[RELAY_FRAME_MAX];
	uint8_t length;
	uint8_t pipe;
	uint8_t rssi;            /* -dBm magnitude, as ESB reports it */
	uint32_t end_kernel_ticks; /* sys_clock_tick_get_32() at END detection */
};

/* Begin receiving on channel for the given logical pipes (bit n = pipe n).
 * single: the radio disables itself after the first packet (END->DISABLE),
 * so the caller may sleep until the window ends. Otherwise it restarts after
 * after the caller copies each packet in relay_radio_poll(). */
bool relay_radio_start(uint8_t channel, uint8_t pipe_mask, bool single);

/* Split startup for a scheduled edge. The caller retains raw RX arbitration
 * until stop, including any wait between these calls. Prepare leaves RADIO
 * disabled; receive only triggers RXEN. stop also cancels an unstarted prepare. */
bool relay_radio_prepare(uint8_t channel, uint8_t pipe_mask, bool single);
bool relay_radio_receive(void);

/* Continuous mode: returns true and fills frame for each CRC-valid packet. */
bool relay_radio_poll(struct relay_rx_frame *frame);

/* Single mode: true if a packet arrived or is arriving (address matched). */
bool relay_radio_busy(void);

/* Diagnostics, valid until relay_radio_stop(): an address matched / a packet
 * ended (CRC not checked) during this window. */
bool relay_radio_addressed(void);
bool relay_radio_ended(void);
/* Diagnostics after END: raw length byte, CRC status and RSSI (-dBm). */
void relay_radio_peek(uint8_t *length, bool *crc_ok, uint8_t *rssi);

/* Single mode: copy the captured packet if CRC-valid. */
bool relay_radio_take(struct relay_rx_frame *frame);

/* Disable the radio and restore every register this module changed. */
/* Failure retains the exclusive radio fence until reboot. */
bool relay_radio_stop(void);
bool relay_radio_faulted(void);

#endif

#ifdef CONFIG_RELAY_LEDGER_TIMING
/* Low bits: READY, ADDRESS, PAYLOAD, END, DISABLED, CRCOK, CRCERROR; state in bits 8+. */
uint16_t relay_radio_observation(void);
#endif
