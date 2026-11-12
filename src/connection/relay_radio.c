/*
 * Raw RADIO receive windows for peer relay and passive PONG presence.
 * See relay_radio.h. Callers hold esb.c raw RX arbitration: ESB is idle, no
 * writer can queue, and no lifecycle path can retune or reinitialise ESB.
 */
#include "relay_radio.h"
#include "system/power.h"

#include <hal/nrf_radio.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <string.h>

#if defined(CONFIG_SOC_SERIES_NRF54L)
#define RR_EVENT_END NRF_RADIO_EVENT_PHYEND
#define RR_SHORT_END_DISABLE NRF_RADIO_SHORT_PHYEND_DISABLE_MASK
#define RR_SHORT_END_START NRF_RADIO_SHORT_PHYEND_START_MASK
#define RR_IRQ RADIO_0_IRQn
#else
#define RR_EVENT_END NRF_RADIO_EVENT_END
#define RR_SHORT_END_DISABLE NRF_RADIO_SHORT_END_DISABLE_MASK
#define RR_SHORT_END_START NRF_RADIO_SHORT_END_START_MASK
#define RR_IRQ RADIO_IRQn
#endif

#if defined(RADIO_SHORTS_DISABLED_RSSISTOP_Msk) || defined(RADIO_SHORTS0_DISABLED_RSSISTOP_Msk)
#define RR_RSSI_SHORTS (NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK | NRF_RADIO_SHORT_DISABLED_RSSISTOP_MASK)
#else
#define RR_RSSI_SHORTS NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK
#endif

#define RR_BASE_FREQUENCY_MHZ 2400U
/* ESB DPL in RAM: [length][S1: ack/pid] then payload. */
#define RR_DPL_HEADER 2U
/* Disabling from RX takes a few microseconds; bound the wait generously. */
#define RR_DISABLE_SPINS 200

static uint8_t rx_buffer[RR_DPL_HEADER + CONFIG_ESB_MAX_PAYLOAD_LENGTH] __aligned(4);
static uint32_t last_poll_ticks;
/* A disable timeout fences both owners and reboots; never recycle DMA that
 * RADIO may still own, and never leave a battery tracker silently offline. */
static bool radio_fault;
bool relay_radio_faulted(void) { return radio_fault; }

static bool rr_fault(void)
{
	radio_fault = true;
	(void)sys_request_system_reboot();
	return false;
}
/* The passive clock gate admits at most four network ticks of innovation.
 * Discard a delayed observation rather than assigning it a misleading END
 * timestamp. One 32 kHz kernel tick rounds down to 30 us. */
#define RR_MAX_POLL_GAP_US 30U

static struct {
	uint32_t shorts;
	uint32_t inten;
	void *packetptr;
	uint8_t rxaddresses;
	bool active;
} saved;

static const nrf_radio_event_t cleared_events[] = {
	NRF_RADIO_EVENT_READY, NRF_RADIO_EVENT_ADDRESS, NRF_RADIO_EVENT_PAYLOAD,
	NRF_RADIO_EVENT_END, NRF_RADIO_EVENT_DISABLED, NRF_RADIO_EVENT_CRCOK,
	NRF_RADIO_EVENT_CRCERROR,
#if defined(CONFIG_SOC_SERIES_NRF54L)
	NRF_RADIO_EVENT_PHYEND,
#endif
};

static void rr_clear_events(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(cleared_events); i++) {
		nrf_radio_event_clear(NRF_RADIO, cleared_events[i]);
	}
}

static bool rr_disable(void)
{
	if (nrf_radio_state_get(NRF_RADIO) == NRF_RADIO_STATE_DISABLED) {
		return true;
	}
	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);
	nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_DISABLE);
	for (int i = 0; i < RR_DISABLE_SPINS; i++) {
		if (nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_DISABLED)
		    || nrf_radio_state_get(NRF_RADIO) == NRF_RADIO_STATE_DISABLED) {
			return true;
		}
		k_busy_wait(1);
	}
	return false;
}

#if defined(__GNUC__) && !defined(__clang__)
/* Attribute provenance: Startup cohort 0035f0ce retained O3; no standalone attribute measurement. Keep the accepted startup group pending A/B. */
__attribute__((optimize("O3")))
#endif
bool relay_radio_prepare(uint8_t channel, uint8_t pipe_mask, bool single)
{
	unsigned key = irq_lock();
	if (radio_fault) { irq_unlock(key); return false; }
	saved.shorts = nrf_radio_shorts_get(NRF_RADIO);
	saved.inten = nrf_radio_int_enable_check(NRF_RADIO, UINT32_MAX);
	saved.packetptr = nrf_radio_packetptr_get(NRF_RADIO);
	saved.rxaddresses = nrf_radio_rxaddresses_get(NRF_RADIO);
	saved.active = true;
	/* ESB's handlers key off on_radio_disabled; none may run for our events. */
	nrf_radio_int_disable(NRF_RADIO, UINT32_MAX);
	NVIC_ClearPendingIRQ(RR_IRQ);
	/* The caller owns an idle radio. Prepare may precede the RX edge,
	 * with interrupts restored while it waits; RADIO remains disabled. */
	nrf_radio_shorts_set(NRF_RADIO, 0);
	if (!rr_disable()) {
		irq_unlock(key);
		return rr_fault();
	}
	rr_clear_events();
	nrf_radio_frequency_set(NRF_RADIO, RR_BASE_FREQUENCY_MHZ + channel);
	nrf_radio_rxaddresses_set(NRF_RADIO, pipe_mask);
	nrf_radio_packetptr_set(NRF_RADIO, rx_buffer);
	/* Continuous receive stops in RXIDLE at END. Copy the completed DMA
	 * buffer before issuing START in poll(); IRQ masking cannot stop DMA.
	 * RXIDLE -> START has no additional 40 us radio ramp-up. */
	nrf_radio_shorts_set(NRF_RADIO, RR_RSSI_SHORTS | NRF_RADIO_SHORT_READY_START_MASK
		| (single ? RR_SHORT_END_DISABLE : 0U));

	irq_unlock(key);
	return true;
}

/* No configuration work belongs between the caller's RX edge and RXEN. */
bool relay_radio_receive(void)
{
	unsigned key = irq_lock();
	if (radio_fault || !saved.active) {
		irq_unlock(key);
		return false;
	}
	last_poll_ticks = sys_clock_tick_get_32();
	__DMB();
	nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_RXEN);
	irq_unlock(key);
	return true;
}

#if defined(__GNUC__) && !defined(__clang__)
__attribute__((optimize("O3")))
#endif
bool relay_radio_start(uint8_t channel, uint8_t pipe_mask, bool single)
{
	unsigned key = irq_lock();
	bool ok = relay_radio_prepare(channel, pipe_mask, single) && relay_radio_receive();
	irq_unlock(key);
	return ok;
}

static bool rr_copy(struct relay_rx_frame *frame)
{
	if (!nrf_radio_crc_status_check(NRF_RADIO)) {
		return false;
	}
	uint8_t length = rx_buffer[0];
	if (length == 0 || length > RELAY_FRAME_MAX || length > CONFIG_ESB_MAX_PAYLOAD_LENGTH) {
		return false;
	}
	frame->length = length;
	frame->pipe = nrf_radio_rxmatch_get(NRF_RADIO);
	frame->rssi = nrf_radio_rssi_sample_get(NRF_RADIO);
	memcpy(frame->data, &rx_buffer[RR_DPL_HEADER], length);
	return true;
}

bool relay_radio_poll(struct relay_rx_frame *frame)
{
	unsigned key = irq_lock();
	uint32_t ticks = sys_clock_tick_get_32();
	bool timely = k_ticks_to_us_floor32(ticks - last_poll_ticks) <= RR_MAX_POLL_GAP_US;
	last_poll_ticks = ticks;
	if (!nrf_radio_event_check(NRF_RADIO, RR_EVENT_END)) {
		irq_unlock(key);
		return false;
	}
	nrf_radio_event_clear(NRF_RADIO, RR_EVENT_END);
	bool ok = timely && rr_copy(frame);
	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_ADDRESS);
	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_PAYLOAD);
	__DMB();
	nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_START);
	irq_unlock(key);
	frame->end_kernel_ticks = ticks;
	return ok;
}

bool relay_radio_busy(void)
{
	return nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_ADDRESS)
		&& !nrf_radio_event_check(NRF_RADIO, RR_EVENT_END);
}

bool relay_radio_addressed(void)
{
	return nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_ADDRESS);
}

bool relay_radio_ended(void)
{
	return nrf_radio_event_check(NRF_RADIO, RR_EVENT_END);
}

void relay_radio_peek(uint8_t *length, bool *crc_ok, uint8_t *rssi)
{
	*length = rx_buffer[0];
	*crc_ok = nrf_radio_crc_status_check(NRF_RADIO);
	*rssi = nrf_radio_rssi_sample_get(NRF_RADIO);
}

bool relay_radio_take(struct relay_rx_frame *frame)
{
	if (!nrf_radio_event_check(NRF_RADIO, RR_EVENT_END)) {
		return false;
	}
	frame->end_kernel_ticks = sys_clock_tick_get_32();
	return rr_copy(frame);
}

bool relay_radio_stop(void)
{
	if (!saved.active) {
		return true;
	}
	nrf_radio_shorts_set(NRF_RADIO, 0);
	if (!rr_disable()) {
		return rr_fault();
	}
	unsigned key = irq_lock();
	rr_clear_events();
	nrf_radio_packetptr_set(NRF_RADIO, saved.packetptr);
	nrf_radio_rxaddresses_set(NRF_RADIO, saved.rxaddresses);
	nrf_radio_shorts_set(NRF_RADIO, saved.shorts);
	NVIC_ClearPendingIRQ(RR_IRQ);
	nrf_radio_int_enable(NRF_RADIO, saved.inten);
	saved.active = false;
	irq_unlock(key);
	return true;
}

#ifdef CONFIG_RELAY_LEDGER_TIMING
uint16_t relay_radio_observation(void)
{
	return (nrf_radio_state_get(NRF_RADIO) << 8)
		| nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_READY)
		| (nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_ADDRESS) << 1)
		| (nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_PAYLOAD) << 2)
		| (nrf_radio_event_check(NRF_RADIO, RR_EVENT_END) << 3)
		| (nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_DISABLED) << 4)
		| (nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_CRCOK) << 5)
		| (nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_CRCERROR) << 6);
}
#endif
