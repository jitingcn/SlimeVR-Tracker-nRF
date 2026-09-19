#ifndef SLIMENRF_SYSTEM_POWER
#define SLIMENRF_SYSTEM_POWER

#include <stdbool.h>
#include <stdint.h>

void sys_interface_suspend(void);
void sys_interface_resume(void);

/* Sensor-owned reversible WOM plan. Refresh while continuously eligible and
 * cancel on interruption. Deadline is the original absolute idle deadline;
 * physical sleep never precedes an announced plan by less than five seconds.
 * Ordinary OFF/REBOOT and OTA supersede reversible WOM. Returns 0 when accepted
 * or idempotent before the physical gate, -EBUSY for another owner or an
 * irreversible transition, and -ENOTSUP without an IMU wake interrupt. */
int sys_plan_WOM(bool force, int64_t deadline);
void sys_cancel_WOM(void);

/* Queue an ordinary transition; never shuts hardware down inline. Returns 0
 * for acceptance/idempotence before the physical gate, or -EBUSY for another
 * owner or an in-flight irreversible physical transition. */
int sys_request_system_off(void);
int sys_request_system_reboot(void);

/* OTA-only two-phase reboot handoff. Reserve BEFORE preparing bootloader state;
 * -EBUSY means another reservation or irreversible physical shutdown has begun.
 * A successful reservation blocks physical transitions without blocking the
 * caller. The caller whose reservation succeeds must resolve it: true commits
 * the reboot, false cancels and preserves ordinary OFF/REBOOT requests.
 *
 * Narrow first-winner exception: committed OTA activation/recovery supersedes
 * deferred OFF/WOM work, including a claimed request still in reversible checks.
 * It never preempts physical shutdown. A reservation cancels reversible WOM.
 */
int sys_ota_reboot_reserve(void);
void sys_ota_reboot_resolve(bool prepared);

bool vin_read(void);
bool vbus_read(void);

#endif
