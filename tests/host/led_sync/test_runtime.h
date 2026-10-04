#ifndef LED_TEST_RUNTIME_H
#define LED_TEST_RUNTIME_H
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "globals.h"
#include "../../../src/system/led.c"
static uint32_t host_now_ms;
static struct led_hardware_info host_hardware;
static bool host_write_failure, host_network_available;
static uint32_t host_network_raw, host_writes, host_offs;
static enum led_role host_role;
static uint16_t host_level, host_value;
static int host_color;
static void (*host_info_hook)(void);
int64_t k_uptime_get(void) { return host_now_ms; }
int64_t k_uptime_ticks(void) { return (uint64_t)host_now_ms*CONFIG_SYS_CLOCK_TICKS_PER_SEC/1000; }
int k_sem_take(struct k_sem *sem,k_timeout_t timeout)
{
	if (sem->count) { sem->count=0; return 0; }
	if (timeout!=K_FOREVER) { host_now_ms+=(uint32_t)timeout; }
	return -1;
}
#if CONFIG_LED_NETWORK_SYNC
bool esb_get_status_clock(uint32_t *local,uint32_t *network)
{
	*local=led_sync_kernel_ticks(k_uptime_ticks(),CONFIG_SYS_CLOCK_TICKS_PER_SEC);
	*network=host_network_raw;
	return host_network_available;
}
#endif
void led_hw_init(void) { host_hardware.powered=false; }
void led_hw_off(void) { ++host_offs; host_hardware.powered=false; host_value=0; }
bool led_hw_write(
	enum led_role role,uint16_t level,uint16_t value,int color,const struct led_fade_sample *fade
)
{
	++host_writes;
	(void)fade;
	host_role=role; host_level=level; host_color=color;
	if (host_write_failure) { host_hardware.last_error=-5; return false; }
	host_value=value;
	host_hardware.powered=value!=0;
	host_hardware.last_error=0;
	return true;
}
void led_hw_info(struct led_hardware_info *out)
{
	*out=host_hardware;
	if (host_info_hook) {
		void (*hook)(void)=host_info_hook;
		host_info_hook=NULL;
		hook();
	}
}
bool led_hw_color_supported(enum led_physical_color color)
{
	return color>=LED_PHYSICAL_RED && color<=LED_PHYSICAL_WHITE && host_hardware.capability==LED_CAP_RGB_PWM;
}
static inline void host_reset(enum led_capability capability)
{
	memset(&engine,0,sizeof(engine)); initialized=false; hardware_quiesced=false;
	request_counter=event_counter=0; host_now_ms=0; host_writes=host_offs=0;
	led_changed.count=led_quiesced.count=0; host_write_failure=false; host_network_available=false;
	host_network_raw=0;
	host_info_hook=NULL;
	host_hardware=(struct led_hardware_info){.capability=capability,.global_limit_pptt=10000,
		.board_limit_pptt=10000,.visible_min_pptt=1000,.gpio=capability==LED_CAP_MONO_GPIO};
	host_role=LED_ROLE_NEUTRAL; host_value=host_level=0; host_color=-1;
}
static inline uint32_t host_step(uint32_t now_ms) { host_now_ms=now_ms; return led_worker_step(); }
static inline struct led_token host_begin(enum led_owner owner,enum led_semantic state)
{
	struct led_token token=led_begin(owner,led_request_id());
	assert(token.session!=0);
	assert(led_state(token,1,state)==LED_ADMITTED);
	return token;
}
#endif
