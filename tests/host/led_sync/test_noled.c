#include "../../../src/system/led.c"

/* Intentionally no scheduler, clock or driver definitions: a noLED worker
 * must return and both public operations must complete without any of them. */
int main(void)
{
	set_led(SYS_LED_PATTERN_ACTIVE_PERSIST, SYS_LED_PRIORITY_SYSTEM);
	led_thread();
	led_shutdown();
	return 0;
}
