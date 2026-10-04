#include "channel_control.h"
#include "esb.h"
#include "retained.h"
#include "system/system.h"
#include "system/led.h"

#include <errno.h>
#include <zephyr/kernel.h>

static K_MUTEX_DEFINE(channel_control_lock);

static int channel_control_apply(uint8_t stored)
{
	k_mutex_lock(&channel_control_lock, K_FOREVER);
	if (!retained) {
		k_mutex_unlock(&channel_control_lock);
		led_request_event(LED_OWNER_RADIO, led_request_id(), led_event_id(), LED_REJECTED);
		return -ENODEV;
	}

	esb_channel_control_begin();
	int storage_error = sys_write(RF_CHANNEL_ID, &retained->rf_channel, &stored, sizeof(stored));
	int radio_error = esb_reinitialize();
	esb_channel_control_end();
	k_mutex_unlock(&channel_control_lock);
	/* Storage and reinitialization are distinct postconditions; retained/radio
	 * may already be changed after either error, never imply link recovery. */
	int result = storage_error < 0 ? storage_error : radio_error;
	led_request_event(LED_OWNER_RADIO, led_request_id(), led_event_id(), result ? LED_PARTIAL : LED_SUCCESS);
	return storage_error < 0 ? storage_error : radio_error;
}

int channel_control_set(int channel)
{
	if (channel < 0 || channel > 100) {
		led_request_event(LED_OWNER_RADIO, led_request_id(), led_event_id(), LED_REJECTED);
		return -EINVAL;
	}
	return channel_control_apply(esb_rf_channel_encode((uint8_t)channel));
}

int channel_control_reset(void)
{
	return channel_control_apply(ESB_RF_CHANNEL_DEFAULT);
}
