#include "uptime.h"

#include <zephyr/init.h>
#include <zephyr/kernel.h>

/* k_uptime_ticks() at the earliest application init point. */
static uint64_t boot_uptime_ticks;

static int system_uptime_init(void)
{
	boot_uptime_ticks = (uint64_t)k_uptime_ticks();
	return 0;
}

SYS_INIT(system_uptime_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

uint64_t system_uptime_since_boot_ticks(void)
{
	return (uint64_t)k_uptime_ticks() - boot_uptime_ticks;
}

int64_t system_uptime_since_boot_ms(void)
{
	return (int64_t)k_ticks_to_ms_near64(system_uptime_since_boot_ticks());
}
