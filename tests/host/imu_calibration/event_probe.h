#ifndef HOST_IMU_EVENT_PROBE_H
#define HOST_IMU_EVENT_PROBE_H

#include "connection/tracker_events.h"
#include <assert.h>

/* Observe calls from the actual coefficient owner; event-core behavior is
 * covered separately by the production tracker_events host fixture. */
static struct tracker_event observed_events[16];
static unsigned event_count;
static unsigned event_notifications;

void cal_event_end(uint16_t operation_id, uint8_t outcome, uint8_t phase, uint8_t reason)
{
	if (!operation_id) {
		return;
	}
	assert(event_count < sizeof(observed_events) / sizeof(observed_events[0]));
	observed_events[event_count++] = (struct tracker_event){
		.operation_id = operation_id, .event = CAL_EVENT_END,
		.outcome = outcome, .phase = phase, .detail = reason,
	};
}

void cal_event_step(uint16_t operation_id, uint8_t phase, uint8_t detail)
{
	if (!operation_id) {
		return;
	}
	assert(event_count < sizeof(observed_events) / sizeof(observed_events[0]));
	observed_events[event_count++] = (struct tracker_event){
		.operation_id = operation_id, .event = CAL_EVENT_STEP,
		.outcome = CAL_OUTCOME_NONE, .phase = phase, .detail = detail,
	};
}

void tracker_events_notify(void)
{
	event_notifications++;
}

static void assert_event(unsigned index, uint16_t operation_id, uint8_t event,
			 uint8_t outcome, uint8_t phase, uint8_t detail)
{
	assert(index < event_count);
	const struct tracker_event *observed = &observed_events[index];
	assert(observed->operation_id == operation_id);
	assert(observed->event == event);
	assert(observed->outcome == outcome);
	assert(observed->phase == phase);
	assert(observed->detail == detail);
}

#endif
