#!/usr/bin/env python3
"""Compile current OTA/power bodies with hardware leaves stubbed, not copied logic."""

import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'harness/python'))
from c_extract import extract_block as block

HERE = Path(__file__).resolve().parent
SRC = Path(os.environ.get("TRACKER_SOURCE_ROOT", HERE.parents[2] / "src"))


def function(source, name):
    return block(source, rf"^(?:static )?(?:bool|int|int64_t|void|uint8_t) {re.escape(name)}\([^;\n]*\)[^\n]*\n\{{")


power = (SRC / "system/power.c").read_text()
ota = (SRC / "system/esb_ota.c").read_text()
sensor = (SRC / "sensor/sensor.c").read_text()
ota_header = (SRC / "system/esb_ota.h").read_text()
tcal = (SRC / "sensor/calibration/tcal_runtime.c").read_text()
radio = (SRC / "connection/esb.c").read_text()
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
parts.append("#if CONFIG_SENSOR_TCAL_HEATED\n" +
             re.search(r"^static atomic_t heater_power_terminal;", power, re.MULTILINE).group() +
             "\n" + function(power, "heater_power_ready") + "\n#endif")
parts.append(function(sensor, "main_imu_is_suspended"))
parts.append("#if CONFIG_SENSOR_USE_TCAL\n" +
             re.search(r"^static (?:bool|atomic_t) tcal_auto_calibration_enabled[^;]*;", tcal, re.MULTILINE).group() +
             "\n" + re.search(r"^static bool tcal_compensation_enabled[^;]*;", tcal, re.MULTILINE).group() +
             "\n" + function(tcal, "sensor_tcal_get_enabled") +
             "\n" + function(tcal, "sensor_tcal_get_auto_calibration") +
             "\n" + function(tcal, "sensor_tcal_set_auto_calibration") + "\n#endif")
for name in ("sys_cancel_WOM_locked", "sys_cancel_WOM", "sys_wom_ready", "sys_plan_WOM",
             "sys_power_state_request", "sys_request_system_off", "sys_request_system_reboot",
             "sys_ota_reboot_reserve", "sys_ota_reboot_resolve", "sys_power_notice",
             "sys_WOM", "sys_system_off", "sys_system_reboot"):
    parts.append(function(power, name))
parts += [block(sensor, r"^enum sensor_sensor_mode \{", True),
          block(sensor, r"^enum sensor_sensor_timeout \{", True)]
for name in ("sensor_mode", "sensor_timeout", "was_ota_suppressed"):
    parts.append(re.search(rf"^static [^\n]* {name}[^;]*;", sensor, re.MULTILINE).group())
for name in ("sensor_get_active_timeout_delay", "sensor_update_sensor_state"):
    parts.append(function(sensor, name))
for name in ("esb_ota_is_active", "esb_ota_get_status", "esb_ota_handle_verify",
             "esb_ota_handle_activate", "esb_ota_handle_abort", "esb_ota_check_timeout"):
    parts.append(function(ota, name))
# BEGIN's admission prefix is sufficient here: the abort-gap call must reject
# before reaching packet validation or flash work. A return of 0 below exposes
# accidental admission without introducing mock copies of the admission rules.
begin = function(ota, "esb_ota_handle_begin")
parts.append(begin[:begin.index("\t/* Validate CRC-8 */")] + "\treturn 0;\n}")
# Exercise the common lifecycle gate and both real handoff tails. Staging
# address/flash preparation is outside this lifecycle contract.
start = begin.index("\t/* Suspend sensor thread and hardware")
branch = begin.index("#if OTA_USE_RAM_ENGINE", start)
ram_end = begin.index("#else /* !OTA_USE_RAM_ENGINE */", branch)
ready = begin.index("\t/* VTOR relocation not needed", ram_end)
end = begin.index("#endif /* OTA_USE_RAM_ENGINE */", ready)
parts.append("static int ota_suspend_and_ready(uint32_t image_size, uint16_t total_packets) {\n" +
             begin[start:ram_end] + "\n#else\n" + begin[ready:end] + "\n#endif\n}")
# Exercise the actual power-loop dispatch, including its completion semantics.
start = power.index("\t\tuint32_t generation = 0;")
end = power.index("power_request_finish(&power_requests, requested, generation, consumed);", start)
end += len("power_request_finish(&power_requests, requested, generation, consumed);")
parts.append("static void power_iteration(void) {\n" + power[start:end] + "\n}")
for name in ("esb_remote_cmd_tcal_auto_on", "esb_remote_cmd_tcal_auto_off"):
    parts.append(function(radio, name))
start = radio.index("\t\t// Check for shutdown timeout if connection errors persist")
end = radio.index("\t\tint64_t now_idle", start)
parts.append("static void lost_link_timeout(void) {\n" + radio[start:end] + "\n}")
start = radio.index("\t\t\t// During pairing, only use connection timeout")
end = radio.index("#endif", start)
# The outer USER_SHUTDOWN_ENABLED block encloses the calibration feature guard.
end = radio.index("\n\t\t\tif (paired_addr[0])", end)
parts.append("static void pairing_timeout(void) {\n#if USER_SHUTDOWN_ENABLED\n" +
             radio[start:end] + "\n}")

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
    for variant in range(128):
        mcuboot, imu_int = (variant // 2) % 2, variant % 2
        low_power_2 = (variant // 4) % 2
        active_delay = 5000 if (variant // 8) % 2 else 90000
        heated = (variant // 16) % 2
        forced_ram = (variant // 32) % 2
        tcal_enabled = variant // 64
        if forced_ram and mcuboot:
            continue
        binary = temporary / f"ota-power-{mcuboot}-{imu_int}-{low_power_2}-{active_delay}-{heated}-{forced_ram}-{tcal_enabled}"
        command = shlex.split(os.environ.get("CC", "cc")) + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter", "-g", "-O1",
            "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie",
            f"-DOTA_USE_MCUBOOT={mcuboot}", f"-DCONFIG_BOOTLOADER_MCUBOOT={mcuboot}",
            f"-DIMU_INT_EXISTS={imu_int}",
            f"-DCONFIG_SENSOR_USE_LOW_POWER_2={low_power_2}",
            f"-DCONFIG_ACTIVE_TIMEOUT_DELAY={active_delay}",
            f"-DCONFIG_SENSOR_TCAL_HEATED={heated}",
            f"-DCONFIG_SENSOR_USE_TCAL={tcal_enabled}",
            "-DCONFIG_SOC_NRF52840=1",
            f"-DCONFIG_ESB_OTA_FORCE_RAM_ENGINE={forced_ram}",
            f"-DOTA_USE_RAM_ENGINE={forced_ram}",
            "-I", str(temporary), "-I", str(SRC), str(HERE / "test_ota_power.c"),
            "-o", str(binary),
        ]
        subprocess.run(command, check=True)
        subprocess.run([str(binary)], check=True)
