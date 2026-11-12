/* Local diagnostic ABI, never sent in relay protocol packets. */
#pragma once
#include <stdint.h>
enum ledger_event {
	LEDGER_SOURCE = 1,
	LEDGER_CAPTURE = 2,
	LEDGER_MISS = 3,
	LEDGER_SELECTED = 4,
	LEDGER_REPAIR = 5,
	LEDGER_B = 6,
	LEDGER_ACCEPT = 7,
	LEDGER_DUPLICATE = 8,
	LEDGER_OBSOLETE = 9,
	LEDGER_EXPIRE = 10,
	LEDGER_TARGET = 11,
	LEDGER_SOURCE_REFUSAL = 12,
	LEDGER_HELPER_B = 13,
	LEDGER_REPAIR_REFUSAL = 14,
	LEDGER_SOURCE_WINDOW = 15,
	LEDGER_REPAIR_WINDOW = 16,
	LEDGER_SOURCE_BUILD_CYCLES = 17,
	LEDGER_SOURCE_ENCODE_CYCLES = 18,
	LEDGER_SOURCE_ADMIT_CYCLES = 19,
	LEDGER_REPAIR_BUILD_CYCLES = 20,
	LEDGER_REPAIR_WAIT_LATE = 21,
	LEDGER_REPAIR_FINAL_CYCLES = 22,
	LEDGER_SOURCE_WAIT_LATE = 23,
	LEDGER_CAPTURE_START = 24,
	LEDGER_SOURCE_SETUP_CYCLES = 25,
	LEDGER_SOURCE_WAIT_CYCLES = 26,
	LEDGER_SOURCE_FINAL_CYCLES = 27,
	LEDGER_SOURCE_START_CYCLES = 28,
	LEDGER_SOURCE_TX_START = 29,
	LEDGER_SOURCE_TX_COMPLETE = 30,
	LEDGER_SOURCE_TX_END_TICK = 31,
	LEDGER_CAPTURE_RADIO_STATE = 32,
};
enum ledger_refusal {
	LEDGER_REFUSAL_NONE = 0,
	LEDGER_REFUSAL_CHANNEL = 1,
	LEDGER_REFUSAL_LAYOUT = 2,
	LEDGER_REFUSAL_ADMISSION = 3,
	LEDGER_REFUSAL_FINAL = 4,
	LEDGER_REFUSAL_NO_TARGET = 5,
	LEDGER_REFUSAL_EMPTY = 6,
	LEDGER_REFUSAL_WAIT = 7,
	LEDGER_REFUSAL_BUSY = 8,
	LEDGER_REFUSAL_GATE = 9,
	LEDGER_REFUSAL_QUEUE = 10,
};
#ifdef CONFIG_RELAY_FRAME_LEDGER
extern uint8_t ledger_source_refusal, ledger_repair_refusal;
extern int16_t ledger_window_detail, ledger_wait_lateness, ledger_repair_wait_lateness, ledger_source_wait_lateness;
#ifdef CONFIG_RELAY_LEDGER_TIMING
extern uint32_t ledger_repair_build_cycles, ledger_repair_final_cycles;
extern uint32_t ledger_source_target_tick;
extern int16_t ledger_source_tx_start;
extern uint32_t ledger_source_setup_cycles, ledger_source_wait_cycles, ledger_source_final_cycles, ledger_source_start_cycles;
extern uint32_t ledger_source_wake_cycles, ledger_source_encode_cycles, ledger_source_write_cycles, ledger_source_admit_cycles;
#endif
#define LEDGER_WINDOW_RESET() (ledger_window_detail = -32768)
#define LEDGER_REPAIR_REASON(reason) (ledger_repair_refusal = (reason))
#define LEDGER_SOURCE_REASON(reason) (ledger_source_refusal = (reason))
void ledger_note(uint32_t session, uint32_t frame, uint8_t child,
                 uint8_t event, int16_t detail, uint8_t peer, uint8_t age);
void ledger_command(int child);
void ledger_command_mask(uint16_t mask);
int ledger_child_enabled(unsigned child);
#else
#define LEDGER_WINDOW_RESET() ((void)0)
#define LEDGER_REPAIR_REASON(reason) ((void)0)
#define LEDGER_SOURCE_REASON(reason) ((void)0)
#define ledger_note(...) ((void)0)
static inline void ledger_command(int child) { (void)child; }
static inline void ledger_command_mask(uint16_t mask) { (void)mask; }
#endif
