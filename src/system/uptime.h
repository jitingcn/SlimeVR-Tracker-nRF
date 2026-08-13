#ifndef SLIMENRF_UPTIME_H
#define SLIMENRF_UPTIME_H

#include <stdint.h>

/*
 * Boot-relative uptime helpers.
 *
 * On nRF54 the GRTC SYSCOUNTER keeps running across resets and is not
 * guaranteed to read zero when the kernel starts, so raw k_uptime_get()
 * values must never be compared against absolute "since boot" thresholds.
 * These helpers measure elapsed time against a reference captured once at
 * application init and are origin-independent.
 */
uint64_t system_uptime_since_boot_ticks(void);
int64_t system_uptime_since_boot_ms(void);

#endif /* SLIMENRF_UPTIME_H */
