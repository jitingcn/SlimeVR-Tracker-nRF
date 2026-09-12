#ifndef SLIMENRF_SYSTEM_POWER
#define SLIMENRF_SYSTEM_POWER

#include <stdbool.h>

void sys_interface_suspend(void);
void sys_interface_resume(void);

/* Queue a transition for the power thread; never shuts hardware down inline.
 * The first accepted request retains ownership while executing or awaiting
 * retry. Returns 0 if accepted (including an identical outstanding request),
 * or -EBUSY if a different transition already owns the slot.
 */
int sys_request_WOM(bool force);
int sys_request_system_off(void);
int sys_request_system_reboot(void);

/* OTA-only two-phase reboot handoff. Reserve BEFORE preparing bootloader state;
 * -EBUSY means another reservation or irreversible physical shutdown has begun.
 * A successful reservation blocks physical transitions without blocking the
 * caller. The caller whose reservation succeeds must resolve it: true commits
 * the reboot, false cancels and preserves the ordinary pending request.
 *
 * Narrow first-winner exception: committed OTA activation/recovery supersedes
 * deferred OFF/WOM work, including a claimed request still in reversible checks.
 * It never preempts physical shutdown. Ordinary producers remain first-winner.
 */
int sys_ota_reboot_reserve(void);
void sys_ota_reboot_resolve(bool prepared);

bool vin_read(void);
bool vbus_read(void);

#endif
