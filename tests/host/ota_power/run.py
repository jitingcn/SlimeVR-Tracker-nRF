#!/usr/bin/env python3
"""Compile current OTA/power bodies with hardware leaves stubbed, not copied logic."""

import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
SRC = HERE.parents[2] / "src"


def block(source, pattern, semicolon=False):
    """Extract a named C construct; ignore braces inside comments and literals."""
    match = re.search(pattern, source, re.MULTILINE)
    if match is None:
        raise ValueError(f"Production construct not found: {pattern}")
    start = source.index("{", match.start())
    tokens = re.compile(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]', re.DOTALL)
    depth = 0
    for token in tokens.finditer(source, start):
        if token.group() == "{":
            depth += 1
        elif token.group() == "}":
            depth -= 1
            if depth == 0:
                return source[match.start():token.end() + int(semicolon)]
    raise ValueError(f"Unclosed production construct: {pattern}")


def function(source, name):
    return block(source, rf"^(?:static )?(?:bool|int|int64_t|void|uint8_t) {name}\([^;\n]*\)[^\n]*\n\{{")


power = (SRC / "system/power.c").read_text()
ota = (SRC / "system/esb_ota.c").read_text()
sensor = (SRC / "sensor/sensor.c").read_text()
ota_header = (SRC / "system/esb_ota.h").read_text()
constants = "\n".join(re.findall(r"^#define OTA_.*$", ota_header, re.MULTILINE))
parts = [constants, block(ota, r"^enum ota_state \{", True),
         block(ota, r"^struct ota_context \{", True), "static struct ota_context ota;"]
parts.append(re.search(r"^static atomic_t ota_reboot_pending;", ota, re.MULTILINE).group())
for pattern in (r"^static struct power_request_mailbox power_requests;",
                r"^static K_SEM_DEFINE\(power_wake_sem,.*?;",
                r"^static K_MUTEX_DEFINE\(power_plan_lock\);",
                r"^static (?:bool|int64_t) wom_[^;]+;",
                r"^#define WOM_ELIGIBILITY_LEASE_MS .*$"):
    parts.extend(re.findall(pattern, power, re.MULTILINE))
parts.append(function(sensor, "main_imu_is_suspended"))
for name in ("sys_cancel_WOM_locked", "sys_cancel_WOM", "sys_wom_ready", "sys_plan_WOM",
             "sys_power_state_request", "sys_request_system_off", "sys_request_system_reboot",
             "sys_ota_reboot_reserve", "sys_ota_reboot_resolve", "sys_power_notice",
             "sys_WOM", "sys_system_off", "sys_system_reboot"):
    parts.append(function(power, name))
parts += [block(sensor, r"^enum sensor_sensor_mode \{", True),
          block(sensor, r"^enum sensor_sensor_timeout \{", True)]
for name in ("sensor_mode", "sensor_timeout", "was_ota_suppressed"):
    parts.append(re.search(rf"^static [^\n]* {name}[^;]*;", sensor, re.MULTILINE).group())
for name in ("sensor_get_active_timeout_delay", "sensor_update_session_motion",
             "sensor_update_sensor_state"):
    parts.append(function(sensor, name))
for name in ("esb_ota_is_active", "esb_ota_get_status", "esb_ota_handle_verify",
             "esb_ota_handle_activate", "esb_ota_handle_abort", "esb_ota_check_timeout"):
    parts.append(function(ota, name))
# BEGIN's admission prefix is sufficient here: the abort-gap call must reject
# before reaching packet validation or flash work. A return of 0 below exposes
# accidental admission without introducing mock copies of the admission rules.
begin = function(ota, "esb_ota_handle_begin")
parts.append(begin[:begin.index("\t/* Validate CRC-8 */")] + "\treturn 0;\n}")
# Exercise the actual power-loop dispatch, including its completion semantics.
start = power.index("\t\tuint32_t generation = 0;")
end = power.index("power_request_finish(&power_requests, requested, generation, consumed);", start)
end += len("power_request_finish(&power_requests, requested, generation, consumed);")
parts.append("static void power_iteration(void) {\n" + power[start:end] + "\n}")

with tempfile.TemporaryDirectory(prefix="tracker-ota-power-") as directory:
    temporary = Path(directory)
    (temporary / "zephyr").mkdir()
    (temporary / "zephyr/sys").mkdir()
    (temporary / "zephyr/sys/atomic.h").write_text("""
#pragma once
#include <stdatomic.h>
typedef atomic_int atomic_t;
#define atomic_get(value) atomic_load(value)
#define atomic_set(value, new_value) atomic_exchange(value, new_value)
""")
    (temporary / "production.inc").write_text("\n\n".join(parts))
    (temporary / "zephyr/kernel.h").write_text("""
#pragma once
#include <stdint.h>
#include <assert.h>
#include <stddef.h>
struct k_mutex { bool locked; };
#define K_MUTEX_DEFINE(name) struct k_mutex name
#define K_FOREVER (-1)
static void (*before_mutex_lock)(void);
static inline void k_mutex_lock(struct k_mutex *mutex, int timeout) {
    (void)timeout;
    if (before_mutex_lock) {
        void (*callback)(void) = before_mutex_lock;
        before_mutex_lock = NULL;
        callback();
    }
    assert(!mutex->locked); mutex->locked = true;
}
static inline void k_mutex_unlock(struct k_mutex *mutex) {
    assert(mutex->locked); mutex->locked = false;
}
struct k_sem { unsigned count; };
#define K_SEM_DEFINE(name, initial, maximum) struct k_sem name = { initial }
static inline void k_sem_give(struct k_sem *sem) { sem->count = 1; }
static int64_t now_ms;
static inline int64_t k_uptime_get(void) { return now_ms; }
static void (*sleep_observer)(int);
static inline void k_msleep(int ms) {
    if (sleep_observer) { sleep_observer(ms); }
    now_ms += ms;
}
""")
    (temporary / "zephyr/spinlock.h").write_text("""
#pragma once
#include <assert.h>
struct k_spinlock { bool locked; };
typedef int k_spinlock_key_t;
static inline int k_spin_lock(struct k_spinlock *lock) {
    assert(!lock->locked); lock->locked = true; return 0;
}
static inline void k_spin_unlock(struct k_spinlock *lock, int key) {
    (void)key; assert(lock->locked); lock->locked = false;
}
""")
    for variant in range(16):
        mcuboot, imu_int = (variant // 2) % 2, variant % 2
        low_power_2, active_delay = (variant // 4) % 2, 5000 if variant // 8 else 90000
        binary = temporary / f"ota-power-{mcuboot}-{imu_int}-{low_power_2}-{active_delay}"
        command = shlex.split(os.environ.get("CC", "cc")) + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter", "-g", "-O1",
            "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie",
            f"-DOTA_USE_MCUBOOT={mcuboot}", f"-DCONFIG_BOOTLOADER_MCUBOOT={mcuboot}",
            f"-DIMU_INT_EXISTS={imu_int}",
            f"-DCONFIG_SENSOR_USE_LOW_POWER_2={low_power_2}",
            f"-DCONFIG_ACTIVE_TIMEOUT_DELAY={active_delay}",
            "-I", str(temporary), "-I", str(SRC), str(HERE / "test_ota_power.c"),
            "-o", str(binary),
        ]
        subprocess.run(command, check=True)
        subprocess.run([str(binary)], check=True)
