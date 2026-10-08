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
radio_header = (SRC / "connection/esb.h").read_text()
constants += "\n" + "\n".join(re.findall(r"^#define ESB_OTA_.*$", radio_header, re.MULTILINE))
parts = [constants, block(ota, r"^enum ota_state \{", True),
         block(ota, r"^struct ota_context \{", True), "static struct ota_context ota;"]
parts.extend(re.findall(r"^static atomic_t ota_(?:reboot_pending|abort_requested);", ota, re.MULTILINE))
parts.extend(re.findall(r"^static (?:struct led_token ota_feedback|uint32_t ota_feedback_revision|bool ota_feedback_terminal|enum led_semantic ota_feedback_state);", ota, re.MULTILINE))
calibration_header = (SRC / "sensor/calibration/calibration.h").read_text()
parts.append(block(calibration_header, r"^static inline int sensor_operation_result\([^;{]*\)\s*\{", False))
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
parts.append(block(ota_header, r"^static inline uint8_t esb_ota_crc8\([^;{]*\)\s*\{"))
flash_header = (SRC / "system/esb_ota_flash.h").read_text()
parts.append(block(flash_header, r"^struct esb_ota_page_buf \{", True))
parts.append("static int esb_ota_flash_flush_page_buf(struct esb_ota_page_buf *pb);")
parts.append(block(ota, r"^static struct esb_ota_page_buf ota_page_buf_view\(void\)\n\{"))
for name in ("esb_ota_is_active", "esb_ota_get_status", "ota_update_led", "esb_ota_handle_data",
             "esb_ota_handle_verify", "ota_activate_impl", "esb_ota_handle_activate",
             "esb_ota_request_abort", "ota_abort", "esb_ota_service",
             "ota_begin_impl", "esb_ota_handle_begin", "esb_ota_process_rx_packet"):
    parts.append(function(ota, name))
ram_engine = (SRC / "system/ota_ram_engine.inc").read_text()
parts.append("#if OTA_USE_RAM_ENGINE\n" +
             block(ram_engine, r"^struct ota_ram_engine_params \{", True) +
             "\nstatic void ota_ram_engine(const struct ota_ram_engine_params *params);\n" +
             function(ota, "ota_launch_ram_engine") + "\n#endif")
# One bounded iteration through the actual no-transport early-continue path.
# Transport-ready paths are exercised by radio_sessions' full connection loop.
connection = (SRC / "connection/connection.c").read_text()
thread = function(connection, "connection_thread")
loop = thread[thread.index("\twhile (1) {"):thread.index("\t\tesb_process_ota_rx_queue();")]
parts.append("static void connection_no_transport_iteration(void) {\n" +
             loop.replace("while (1)", "for (int step = 0; step < 1; step++)", 1) +
             "\nassert(!\"no-transport fixture unexpectedly admitted transport\");\n}\n}")
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
static inline bool atomic_cas(atomic_t *value, int expected, int desired) {
    return atomic_compare_exchange_strong(value, &expected, desired);
}
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
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
            "-Wno-pointer-to-int-cast", "-g", "-O1",
            *(["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
              if os.environ.get("HOST_SANITIZERS", "1") != "0" else []),
            "-fno-pie", "-no-pie",
            "-Wl,--defsym,_flash_used=0x40000",
            f"-DOTA_USE_MCUBOOT={mcuboot}",
            *([ "-DCONFIG_BOOTLOADER_MCUBOOT=1" ] if mcuboot else []),
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
