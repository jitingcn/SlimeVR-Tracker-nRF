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
    return block(source, rf"^(?:static )?(?:bool|int|void|uint8_t) {name}\([^;\n]*\)[^\n]*\n\{{")


power = (SRC / "system/power.c").read_text()
ota = (SRC / "system/esb_ota.c").read_text()
ota_header = (SRC / "system/esb_ota.h").read_text()
constants = "\n".join(re.findall(r"^#define OTA_.*$", ota_header, re.MULTILINE))
parts = [constants, block(ota, r"^enum ota_state \{", True),
         block(ota, r"^struct ota_context \{", True), "static struct ota_context ota;"]
parts.append(re.search(r"^static atomic_t ota_reboot_pending;", ota, re.MULTILINE).group())
for pattern in (r"^static struct power_request_mailbox power_requests;",
                r"^static K_SEM_DEFINE\(power_wake_sem,.*?;"):
    parts.append(re.search(pattern, power, re.MULTILINE).group())
for name in ("sys_power_state_request", "sys_request_WOM", "sys_request_system_off",
             "sys_request_system_reboot", "sys_ota_reboot_reserve", "sys_ota_reboot_resolve",
             "sys_WOM", "sys_system_off", "sys_system_reboot"):
    parts.append(function(power, name))
for name in ("esb_ota_is_active", "esb_ota_get_status", "esb_ota_handle_verify",
             "esb_ota_handle_activate", "esb_ota_handle_abort", "esb_ota_check_timeout"):
    parts.append(function(ota, name))
# BEGIN's admission prefix is sufficient here: the abort-gap call must reject
# before reaching packet validation or flash work. A return of 0 below exposes
# accidental admission without introducing mock copies of the admission rules.
begin = function(ota, "esb_ota_handle_begin")
parts.append(begin[:begin.index("\t/* Validate CRC-8 */")] + "\treturn 0;\n}")
# Exercise the actual power-loop dispatch, including its completion semantics.
start = power.index("\t\tenum sys_power_request requested = power_request_begin(")
end = power.index("power_request_finish(&power_requests, requested, consumed);", start)
end += len("power_request_finish(&power_requests, requested, consumed);")
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
struct k_sem { unsigned count; };
#define K_SEM_DEFINE(name, initial, maximum) struct k_sem name = { initial }
static inline void k_sem_give(struct k_sem *sem) { sem->count = 1; }
static int64_t now_ms;
static inline int64_t k_uptime_get(void) { return now_ms; }
static inline void k_msleep(int ms) { now_ms += ms; }
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
    for mcuboot in (1, 0):
        binary = temporary / f"ota-power-{mcuboot}"
        command = shlex.split(os.environ.get("CC", "cc")) + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter", "-g", "-O1",
            "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie",
            f"-DOTA_USE_MCUBOOT={mcuboot}", f"-DCONFIG_BOOTLOADER_MCUBOOT={mcuboot}",
            "-I", str(temporary), "-I", str(SRC), str(HERE / "test_ota_power.c"),
            "-o", str(binary),
        ]
        subprocess.run(command, check=True)
        subprocess.run([str(binary)], check=True)
