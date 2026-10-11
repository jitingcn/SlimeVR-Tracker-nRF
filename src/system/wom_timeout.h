#ifndef SLIMENRF_WOM_TIMEOUT_H
#define SLIMENRF_WOM_TIMEOUT_H

#include <stdint.h>

#define WOM_TIMEOUT_MIN_MINUTES 1
#define WOM_TIMEOUT_MAX_MINUTES 35791

/* Normal/first WOM only; post-WOM low-activity policy is independent.
 * Thread context setter: persist before publishing, negative errno on failure.
 * Zero deletes the saved override and restores the exact compiled default;
 * positive values save whole minutes. Clearing does not disable WOM.
 * Getters read an atomic runtime snapshot, never mutable retained state. */
int wom_timeout_set_minutes(uint16_t minutes);
/* Zero means unsupported mode or legacy default not exactly representable as
 * 1..35791 whole minutes. A valid set can replace a nonrepresentable default. */
uint16_t wom_timeout_get_minutes(void);
int64_t wom_timeout_get_delay_ms(void);

#endif
