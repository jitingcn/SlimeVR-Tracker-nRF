#ifndef TRACKER_HOST_LED_FEEDBACK_STUB_H
#define TRACKER_HOST_LED_FEEDBACK_STUB_H

#include "../../src/system/led.h"

/* Hardware-free observations for actual business-owner extraction harnesses.
 * Renderer/policy tests link production LED code instead of this fixture. */
static uint32_t led_test_identity;
static unsigned led_test_events[LED_SEMANTIC_COUNT];
static struct led_connection_facts led_test_connection;
static enum led_power_state led_test_power;
static bool led_test_low, led_test_quiesced;

uint32_t led_request_id(void) { return ++led_test_identity; }
uint32_t led_event_id(void) { return ++led_test_identity; }
struct led_token led_begin(enum led_owner owner, uint32_t request)
{
	return (struct led_token){ .owner = owner, .session = ++led_test_identity, .request_id = request };
}
enum led_admission led_state(struct led_token token, uint32_t revision, enum led_semantic semantic)
{
	(void)token; (void)revision; (void)semantic;
	return LED_ADMITTED;
}
enum led_admission led_result(struct led_token token, uint32_t event, enum led_semantic semantic)
{
	(void)token; (void)event;
	++led_test_events[semantic];
	return LED_ADMITTED;
}
enum led_admission led_request_event(enum led_owner owner, uint32_t request, uint32_t event,
	enum led_semantic semantic)
{
	(void)owner; (void)request; (void)event;
	++led_test_events[semantic];
	return LED_ADMITTED;
}
uint32_t led_button_input(void) { return ++led_test_identity; }
enum led_admission led_button_group(uint32_t generation, uint32_t count)
{
	(void)generation; (void)count;
	return led_test_quiesced ? LED_SHUTDOWN : LED_ADMITTED;
}
enum led_admission led_button_hold(uint32_t generation, bool active, uint32_t started_ms)
{
	(void)generation; (void)active; (void)started_ms;
	return led_test_quiesced ? LED_SHUTDOWN : LED_ADMITTED;
}
enum led_admission led_button_exit(struct led_token token, uint32_t revision, uint32_t generation)
{
	(void)generation;
	return led_test_quiesced ? LED_SHUTDOWN : led_state(token, revision, LED_MANUAL_EXIT);
}
bool led_output_enabled(void) { return true; }
void led_identify(void) { ++led_test_events[LED_IDENTIFY]; }
void led_connection_publish(const struct led_connection_facts *facts) { led_test_connection = *facts; }
void led_power_publish(enum led_power_state state, bool low) { led_test_power = state; led_test_low = low; }
void led_fault_publish(enum led_owner owner, enum led_fault_kind fault, uint32_t protection)
{
	(void)owner; (void)fault; (void)protection;
}
void led_maintenance_publish(enum led_owner owner, bool active) { (void)owner; (void)active; }
void led_operation_publish(enum led_owner owner, bool ota, bool heated) { (void)owner; (void)ota; (void)heated; }
void led_quiesce(void) { led_test_quiesced = true; }
void led_shutdown(void) { led_quiesce(); }

#endif
