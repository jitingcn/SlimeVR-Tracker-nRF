#include "channel_control.h"
#include "esb.h"
#include "retained.h"
#include "system/system.h"

#include <errno.h>
#include <zephyr/kernel.h>

static K_MUTEX_DEFINE(channel_control_lock);

static int channel_control_apply(uint8_t stored)
{
	k_mutex_lock(&channel_control_lock, K_FOREVER);
	if (!retained) {
		k_mutex_unlock(&channel_control_lock);
		return -ENODEV;
	}

	int storage_error = sys_write(RF_CHANNEL_ID, &retained->rf_channel, &stored, sizeof(stored));
	int radio_error = esb_reinitialize();
	k_mutex_unlock(&channel_control_lock);
	return storage_error < 0 ? storage_error : radio_error;
}

int channel_control_set(int channel)
{
	if (channel < 0 || channel > 100) {
		return -EINVAL;
	}
	return channel_control_apply(esb_rf_channel_encode((uint8_t)channel));
}

int channel_control_reset(void)
{
	return channel_control_apply(ESB_RF_CHANNEL_DEFAULT);
}
