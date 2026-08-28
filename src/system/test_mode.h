#ifndef SLIMENRF_TEST_MODE
#define SLIMENRF_TEST_MODE

#include <stdbool.h>
#include <stdint.h>

/* Battery drain / endurance test mode.
 *
 * When active the tracker behaves as if it is in continuous use:
 *   - Single-press button reboot is blocked (long-press shutdown still works)
 *   - Connection-error timeout shutdown is suppressed
 *   - WOM (wake-on-motion) and activity-timeout sleep are suppressed
 *
 * The mode can be toggled via receiver remote command or local trigger.
 * Powering off or rebooting clears the flag (it is not retained).
 */

/* Built-in default test rate interval: 10 ms = 100 TPS. */
#define TEST_MODE_MIN_SEND_INTERVAL_MS 10

bool test_mode_get(void);
void test_mode_set(bool enable);

/* Target test packet rate in packets per second. Test mode treats this as
 * both floor and ceiling; target is clamped to TDMA frame capacity. tps == 0
 * restores the built-in 100 TPS default. */
void test_mode_set_target_tps(uint16_t tps);
uint16_t test_mode_get_target_tps(void);
/* Capacity-clamped target TPS; 0 outside test mode. */
uint16_t test_mode_effective_tps(void);
/* Rounded sensor-side heartbeat interval; radio scheduling uses exact TPS. */
int test_mode_min_send_interval_ms(void);

#endif
