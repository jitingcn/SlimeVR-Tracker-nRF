#ifndef SLIMENRF_TEST_MODE
#define SLIMENRF_TEST_MODE

#include <stdbool.h>

/* Battery drain / endurance test mode.
 *
 * When active the tracker behaves as if it is in continuous use:
 *   - Single-press button reboot is blocked (long-press shutdown still works)
 *   - Connection-error timeout shutdown is suppressed
 *   - WOM (wake-on-motion) and activity-timeout sleep are suppressed
 *   - Sensor low-power modes are limited (min ~80 TPS instead of dropping to 10)
 *
 * The mode can be toggled via receiver remote command or local trigger.
 * Powering off or rebooting clears the flag (it is not retained).
 */

/* Minimum forced-send interval in test mode (ms).
 * Normal mode uses 1000 ms; test mode sends at least every 12 ms (~80 TPS). */
#define TEST_MODE_MIN_SEND_INTERVAL_MS 12

bool test_mode_get(void);
void test_mode_set(bool enable);

#endif
