# SlimeNRF tracker firmware

Zephyr/NCS firmware for SlimeNRF motion trackers on Nordic nRF52 and nRF54L SoCs.

## Project history

This firmware is originally based on [SlimeVR/SlimeVR-Tracker-nRF](https://github.com/SlimeVR/SlimeVR-Tracker-nRF). Thanks for their work and for maintaining the project.

This repo was forked from [LyallUlric/Stacked-SmolSlime](https://github.com/LyallUlric/Stacked-SmolSlime) (stacked Promicro builds on top of the official tracker firmware), not from upstream `main`. When Stacked-SmolSlime stopped being maintained, We continued development here.

A hard fork followed because upstream `main` history was hard-reset, development direction was unclear and slowly, and upstream discouraged AI agent coding. This fork keeps history from before that reset and continues development. It started from my personal gaming needs. It now also covers many requests from the community and from Chinese DIY SlimeVR-compatible tracker builders.

Changes are driven by real playtesting, user needs, and normal software engineering practice. Untested patches are not accepted on that basis alone.

## Major differences from upstream

- Remote command from receiver side
- Tweaked VQF fusion parameters and backend
- Add mag support properly for ICM45686/LSM6DSV and QMC6309 (other mags are not well tested)
- More sensor calibration options and features
- Custom sdk-nrf fork with more features and fixes
- Different TDMA radio scheduling, clock sync methods
- OTA support via BLE/ESB
- Configurable IMU/mag sensor driver whitelist
- Resting state policy refinements, Different WoM policy
- Raw sensor data collection and analysis
- Best-effort calibration, rest, button-group, and power-intent events reports
- Experimental EqF fusion backend
- More community board vendors and Promicro variants

## SDK and build environment

`west.yml` selects [jitingcn/sdk-nrf](https://github.com/jitingcn/sdk-nrf)
`v3.4-branch`, based on the official NCS `v3.4.0` release.

Builds require Zephyr SDK **1.0.1 GNU** (`zephyr/gnu`, GCC 14.3.0) and
**Python 3.12**. The firmware uses Picolibc; CI runs on Ubuntu 24.04.

The firmware depends on the SDK's ESB extensions and USB fixes; stock NCS
is not a drop-in replacement. UF2 generation uses `CONFIG_BUILD_OUTPUT_HEX=y`
to read image addresses from HEX output. The SDK includes the
[upstream HEX-first UF2 fix](https://github.com/zephyrproject-rtos/zephyr/pull/107944)
required for mapped partitions.

## Synchronized status LEDs

`CONFIG_LED_NETWORK_SYNC` is enabled by default with TDMA support. Trackers
connected to the same receiver with a valid, fresh TDMA clock share the phase
of ordinary status indications, on GPIO LEDs, PWM LEDs and LED strips:

- Normal operation: 300 ms on every 10 seconds.
- Charging: the existing 5-second breathing waveform.
- Low battery: 500 ms on, 500 ms off.

Error, pairing, calibration, DFU and finite event indications keep local timing,
nominal durations, flash counts and priority. Their deadlines include output
time rather than adding driver latency to every step. Steady lights have no
phase to synchronize. Brightness settings and LED-strip low-brightness dithering
remain in effect. Changing the visible pattern resets the strip's quantization
residual without cycling device power, so the previous animation cannot change
the next steady color. Repeating the same request does not reset dithering.

No receiver update or protocol change is required. The existing 32-bit network
clock wraps every 36 hours, 24 minutes and 32 seconds. A 5/10-second indication
can have one altered interval at that boundary (the normal 10-second flash can
have a 12-second gap); trackers use the same wrapped phase and resume their
normal periods. The firmware does not independently extend the clock epoch on
each tracker, so late joiners can use the same phase.

Before initial synchronization, indications run locally. On loss of valid
synchronization they continue locally using the last offset; this is not a
guarantee of continued inter-device alignment. Fresh synchronization realigns
them. Ordinary non-fading indications check clock changes at most 250 ms apart,
without high-frequency LED writes; breathing retains its 5 ms rendering cadence.

Entering the normal-operation indication retains at least 9.7 seconds of
initial darkness, then joins a complete shared flash window. This can delay the
first flash to just under 19.7 seconds. Repeated requests for the same state do
not restart that waiting period. Set `CONFIG_LED_NETWORK_SYNC=n` for local-only
timing.

The LED worker owns rendering and device power transitions. Shutdown paths wait
for its black/off operation acknowledgment instead of suspending a thread during
a driver transfer. Gated LED strips wait 2 ms after power enable before device
resume; strip shutdown waits 1 ms after the black-frame call returns before
suspend/power removal. These are conservative timing margins, not confirmation
of I2S hardware completion or successful black-frame delivery. A failed black
write does not prevent power removal or block shutdown indefinitely.

## Tracker events

Matching receiver firmware can expose calibration lifecycle, tracker rest,
fusion rest, completed button groups, and power lifecycle notifications
through its `scripts/hid_cmd.py` client. See the
[receiver event guide](https://github.com/jitingcn/SlimeVR-Tracker-nRF-Receiver#tracker-events)
for subscriptions and calibration watches.

The `Tracker event telemetry` Kconfig menu provides independent build-time
switches, all enabled by default:

| Option | Reports |
| --- | --- |
| `CONFIG_TRACKER_EVENT_CALIBRATION` | Calibration lifecycle, progress, and results |
| `CONFIG_TRACKER_EVENT_TRACKER_REST` | Tracker motion/rest state |
| `CONFIG_TRACKER_EVENT_FUSION_REST` | VQF/EqF rest-detection state |
| `CONFIG_TRACKER_EVENT_POWER` | Power intentions, boot/wake, and watchdog reset |

For example, set `CONFIG_TRACKER_EVENT_FUSION_REST=n` in your build configuration
to disable only fusion-state reports, including their repeats and heartbeats.
These switches affect telemetry only: fusion, rest detection, calibration,
sensor lifecycle, and power-management timing remain unchanged. Button-group
events remain enabled independently of these four switches.

This is a bounded, best-effort channel, not a reliable action log. Rest reports
are current observations; button groups are distinct actions. `BOOT` or `WAKE`
is deferred for three seconds after early boot classification; `WAKE` means a
hardware SYSTEMOFF wake, not identification of a particular wake GPIO.
A detected hardware watchdog reset adds `WATCHDOG_RESET`; retained historical
channel diagnostics are not treated as proof of the current failure.

`WILL_WOM` is queued at least five seconds before physical sleep. The original idle
deadline is retained unless the full warning interval requires a later sleep.
Motion or another eligibility interruption withdraws the reversible request and
emits `WOM_CANCELLED`; readiness waits do not publish a premature intention.
The 30-second ESB/status fallback budget starts once per boot, at the first
not-ready normal sleep attempt that reaches its original idle deadline;
cancelling or replanning does not renew it. An early warning interrupted before
that deadline does not change the next idle-timeout ramp anchor. The IMU ramp
and activity-timeout delay settings have a 5000 ms Kconfig minimum to match the
warning floor. Announcement/cancellation logs include forced mode, deadline and
remaining lead for diagnosis.
`WILL_SHUTDOWN` and `WILL_REBOOT` get a bounded 500 ms transmission opportunity
before normal subsystem teardown. Power intentions are not confirmation that
the transition completed, and radio admission and finite repetitions do not
guarantee delivery. Emergency watchdog resets and abrupt power loss cannot wait
for notification delivery.

## License

Unless otherwise specified, all code in this repository is dual-licensed under either:

- MIT License ([LICENSE-MIT](LICENSE-MIT) or https://opensource.org/license/mit/)
- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or https://opensource.org/license/apache-2-0/)

at your option. This means you can select the license you prefer!

Unless you explicitly state otherwise, any contribution intentionally submitted for
inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual
licensed as above, without any additional terms or conditions.
