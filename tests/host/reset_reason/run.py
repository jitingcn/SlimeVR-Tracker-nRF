#!/usr/bin/env python3
"""Exercise production boot bodies against a write-one-to-clear register model.

C++ is used only to model MMIO assignment/read semantics without rewriting the C
bodies. SOURCE_ROOT may point to an archived pre-fix tree for the same repro.
"""

import argparse
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'harness/python'))
from c_extract import extract_block

HERE = Path(__file__).resolve().parent
SRC = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2])) / "src"
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--autoconf", type=Path,
                    help="Compile with a real generated Zephyr autoconf.h (no recovery/WDT overrides)")
args = parser.parse_args()
socs = (52, 54)
wdts = (1, 0)
if args.autoconf:
    args.autoconf = args.autoconf.resolve()
    config = args.autoconf.read_text()
    # Select only the modeled register layout; production guards still consume
    # the complete real header through the compiler's -include.
    series = re.search(r'^#define CONFIG_SOC_SERIES "([^"]+)"', config, re.MULTILINE)
    if not series or series.group(1) not in ("nrf52", "nrf54l"):
        parser.error("--autoconf requires an nRF52 or nRF54L application header")
    socs = (52 if series.group(1) == "nrf52" else 54,)
    wdts = (None,)


def function(source, name):
    return extract_block(source, rf"^(?:static )?(?:inline )?(?:bool|int|void|uint8_t|uint32_t) {re.escape(name)}\([^;\n]*\)[^\n]*\n\{{")


def registration(source, name):
    return re.search(rf"^SYS_INIT\({name},[^;]+;", source, re.MULTILINE).group()


system = (SRC / "system/system.c").read_text()
watchdog = (SRC / "system/watchdog.c").read_text()
main = (SRC / "main.c").read_text()
globals_source = (SRC / "globals.h").read_text()
parts = [re.search(r"^#define ADAFRUIT_FAULT_RECOVERY \\\n[^\n]*",
                   globals_source, re.MULTILINE).group()]
parts += re.findall(r"^#define ADAFRUIT_(?:WDT_RETRY_[12]|FATAL_RECOVERY|RECOVERY_CAPABILITY|DFU_MAGIC_UF2_RESET) .*$",
                    globals_source, re.MULTILINE)
snapshot = re.search(r"^static uint32_t boot_reset_reason;", system, re.MULTILINE)
if snapshot:
    parts += ["#define HAS_RESET_SNAPSHOT 1", snapshot.group(),
              "#if ADAFRUIT_FAULT_RECOVERY",
              re.search(r"^static bool bootloader_supports_recovery;", system, re.MULTILINE).group(),
              "#endif", function(system, "sys_bootloader_supports_recovery"),
              function(system, "sys_reset_reason_init"),
              function(system, "sys_get_reset_reason"),
              registration(system, "sys_reset_reason_init")]
# The complete live public header also compiles its CONFIG_TASK_WDT=n branch.
parts.append('#include "system/watchdog.h"')
reset_body = function(watchdog, "watchdog_caused_reset")
# Older trees put this query inside the TASK_WDT guard; use their actual stub
# when disabled rather than inventing a compatibility implementation.
if watchdog.index(reset_body) < watchdog.rindex("#endif /* CONFIG_TASK_WDT */"):
    parts += ["#if CONFIG_TASK_WDT", reset_body, "#endif"]
else:
    parts.append(reset_body)
parts.append("#if CONFIG_TASK_WDT")
for name in ("saved_gpregret", "last_reset_was_wdt"):
    declaration = re.search(rf"^static (?:bool|uint8_t) {name}[^;]*;", watchdog, re.MULTILINE)
    if declaration:
        parts.append(declaration.group())
parts += [function(watchdog, "watchdog_early_check"),
          registration(watchdog, "watchdog_early_check"), "#endif"]
parts.append(function(system, "k_sys_fatal_error_handler"))
parts.append("#if ADAFRUIT_FAULT_RECOVERY")
parts += ["#if CONFIG_TASK_WDT", "static bool boot_success_marked;", "#endif"]
for name in ("watchdog_get_reset_count", "watchdog_clear_reset_count", "watchdog_mark_boot_success"):
    parts.append(function(watchdog, name))
parts.append("#endif")
parts += [function(system, "sys_boot_woke_from_off"),
          function(system, "sys_boot_event_init"),
          registration(system, "sys_boot_event_init")]
parts.append(re.search(r"^static bool ram_retention_valid[^;]*;", system, re.MULTILINE).group())
# Retained validation/NVS restoration follows this policy decision. Stop at that
# boundary to inspect whether reset-pin correctly withholds automatic trust;
# no persistence/CRC logic is copied into this harness.
retained = function(system, "sys_retained_init")
parts += [retained[:retained.index("\t// All contents of NVS")] + "\treturn 0;\n}",
          registration(system, "sys_retained_init"),
          function(system, "sys_button_init"),
          registration(system, "sys_button_init"),
          function(system, "button_read_filtered"),
          function(main, "main").replace("int main(void)", "int tracker_main(void)", 1)]

with tempfile.TemporaryDirectory(prefix="tracker-reset-reason-") as directory:
    temporary = Path(directory)
    (temporary / "zephyr").mkdir()
    (temporary / "zephyr/kernel.h").write_text("#pragma once\n#include <stdint.h>\n")
    (temporary / "production.inc").write_text("\n\n".join(parts))
    for soc in socs:
        variants = [(wdt, mcuboot) for wdt in wdts
                    for mcuboot in ((None,) if args.autoconf else (0, 1))]
        for wdt, mcuboot in variants:
            for ignore_reset in (0, 1):
                binary = temporary / f"reset-{soc}-wdt{wdt}-mcuboot{mcuboot}-ignore{ignore_reset}"
                command = shlex.split(os.environ.get("CXX", "c++")) + [
                    "-std=c++17", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function",
                    "-Wno-unused-variable", "-Wno-unused-parameter", "-g", "-O1",
                    "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie",
                    f"-DMODEL_SOC={soc}", f"-DIGNORE_RESET={ignore_reset}",
                    "-I", str(temporary), "-I", str(SRC), str(HERE / "test_reset_reason.cpp"),
                    "-o", str(binary),
                ]
                if args.autoconf:
                    command += ["-include", str(args.autoconf)]
                else:
                    command += [f"-DCONFIG_SOC_SERIES_NRF52={int(soc == 52)}",
                                f"-DCONFIG_BUILD_OUTPUT_UF2={int(soc == 52)}",
                                f"-DCONFIG_BOOTLOADER_MCUBOOT={mcuboot}"]
                    if wdt:
                        command.append("-DCONFIG_TASK_WDT=1")
                subprocess.run(command, check=True)
                # Each invocation gets a genuine fresh boot (zero-initialized
                # cached state), not a test-only reset of production globals.
                off = 1 << (8 if soc == 54 else 16)
                reasons = [1, 1 << 20, 2, 0, 1 | 2 | (1 << 20) | (1 << 16),
                           off, off | 1 | 2 | (1 << 20), 1 | 2 | (1 << 20)]
                if soc == 54:
                    reasons += [4, off | 4 | 1 | (1 << 20), off | 2 | 4 | (1 << 20)]
                for reason in reasons:
                    markers = range(256) if reason == 0 else (0, 0xD3, 0xE1, 0xE2, 0xE3, 0x57, 0xA8, 0x4E, 0x6D, 0x9B)
                    for marker in markers:
                        for token in (0, 0x52435631, 0x52435630):
                            subprocess.run([str(binary), str(reason), str(marker), str(token)], check=True)
