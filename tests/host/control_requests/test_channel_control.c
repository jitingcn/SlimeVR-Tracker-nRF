#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include "../../../src/retained.h"

/* Replace only the storage backend; compile the actual channel business code
 * and actual RF encoding. Model sys_write's documented eager-RAM contract. */
#define SLIMENRF_SYSTEM
#define RF_CHANNEL_ID 31
int sys_write(uint16_t id, void *ptr, const void *data, size_t len);
#include "../../../src/connection/channel_control.c"

static struct retained_data state;
struct retained_data *retained = &state;
static int storage_error;
static int radio_error;
static uint8_t persisted_channel;
static uint8_t active_channel;

int sys_write(uint16_t id, void *ptr, const void *data, size_t len)
{
	(void)id;
	memcpy(ptr, data, len);
	if (storage_error) {
		return storage_error;
	}
	persisted_channel = *(const uint8_t *)data;
	return 0;
}

int esb_reinitialize(void)
{
	if (radio_error) {
		return radio_error;
	}
	active_channel = esb_rf_channel_decode(retained->rf_channel);
	return 0;
}

int main(void)
{
	assert(channel_control_set(0) == 0);
	assert(active_channel == 0);
	assert(esb_rf_channel_decode(persisted_channel) == 0);
	assert(channel_control_set(100) == 0);
	assert(active_channel == 100);
	assert(esb_rf_channel_decode(persisted_channel) == 100);

	assert(channel_control_set(-1) == -EINVAL);
	assert(channel_control_set(101) == -EINVAL);
	assert(channel_control_set(256) == -EINVAL);
	assert(state.rf_channel == 100 && persisted_channel == 100 && active_channel == 100);

	/* Persistence failure must be visible, yet keep the original eager apply
	 * semantics: the radio uses the new RAM channel, not stale flash. */
	storage_error = -EIO;
	assert(channel_control_set(25) == -EIO);
	assert(active_channel == 25 && state.rf_channel == 25 && persisted_channel == 100);

	/* A radio failure does not undo a successful persistent update. */
	storage_error = 0;
	radio_error = -EBUSY;
	assert(channel_control_set(50) == -EBUSY);
	assert(active_channel == 25 && state.rf_channel == 50 && persisted_channel == 50);
	storage_error = -ENOSPC;
	assert(channel_control_reset() == -ENOSPC);
	assert(state.rf_channel == ESB_RF_CHANNEL_DEFAULT && persisted_channel == 50);

	storage_error = 0;
	radio_error = 0;
	assert(channel_control_reset() == 0);
	assert(active_channel == ESB_RF_CHANNEL_DEFAULT);
	assert(esb_rf_channel_decode(persisted_channel) == ESB_RF_CHANNEL_DEFAULT);
	retained = NULL;
	assert(channel_control_set(25) == -ENODEV);
	assert(channel_control_reset() == -ENODEV);
	puts("channel control: PASS");
	return 0;
}
