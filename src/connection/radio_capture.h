/*
 * Nonintrusive RADIO event timestamp capture for TDMA diagnostics.
 */
#ifndef SLIMENRF_RADIO_CAPTURE_H
#define SLIMENRF_RADIO_CAPTURE_H

#include <stdbool.h>
#include <stdint.h>

int radio_capture_init(void);
void radio_capture_deinit(void);
void radio_capture_set_enabled(bool enabled);
bool radio_capture_is_enabled(void);
void radio_capture_record(
	uint8_t packet_type,
	uint8_t payload_length,
	bool no_ack,
	uint8_t attempts,
	bool success,
	bool transaction_start
);
void radio_capture_process(void);
void radio_capture_print_stats(void);

#endif
