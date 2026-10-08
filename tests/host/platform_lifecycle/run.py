#!/usr/bin/env python3
"""Run actual platform lifecycle functions against deterministic hardware failures."""
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
parser.add_argument("--clock-source", type=Path,
                    help="Extract clock functions from this file instead of SOURCE_ROOT")
args = parser.parse_args()


def function(text, name):
    return extract_block(text, rf"^(?:static )?(?:inline )?(?:void|bool|int|uint8_t|nrf_clock_lfclk_t) {re.escape(name)}\([^;{{]*\)\s*\{{")


globals_source = (SRC / "globals.h").read_text()
parts = re.findall(r"^#define ADAFRUIT_(?:WDT_RETRY_[12]|FATAL_RECOVERY|DFU_MAGIC_UF2_RESET) .*$",
                   globals_source, re.MULTILINE)
watchdog = (SRC / "system/watchdog.c").read_text()
parts += ["static bool boot_success_marked;"]
for name in ("should_enter_dfu", "enter_dfu_mode"):
    parts.append(function(watchdog, name))
for name in ("watchdog_get_reset_count", "watchdog_clear_reset_count", "watchdog_mark_boot_success"):
    parts.append(function(watchdog, name))
for path, names in (
    ("system/clock_control.c", ("normalize_source", "lfclk_running_source_get", "lfclk_start_source", "clock_switch")),
    ("system/watchdog.c", ("watchdog_init", "watchdog_register_thread", "watchdog_feed", "watchdog_pause", "watchdog_resume")),
    ("system/power.c", ("sys_interface_action", "sys_interface_suspend", "sys_interface_resume")),
    ("sensor/sensor.c", ("sensor_shutdown", "sensor_setup_WOM")),
):
    source_path = args.clock_source if path == "system/clock_control.c" and args.clock_source else SRC / path
    source = source_path.read_text()
    for name in names:
        # Helpers introduced by the fix are absent in baseline source.
        if name in ("lfclk_start_source", "sys_interface_action") and not re.search(rf"\b{name}\(", source):
            continue
        parts.append(function(source, name))
with tempfile.TemporaryDirectory(prefix="tracker-platform-") as directory:
    temp = Path(directory)
    (temp / "production.inc").write_text("\n\n".join(parts))
    for adafruit in (0, 1):
        binary = temp / f"test-{adafruit}"
        subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
            "-Wno-unused-variable", "-g", "-O1", "-fsanitize=address,undefined",
            "-fno-omit-frame-pointer", "-fno-pie", "-no-pie", "-I", str(temp),
            f"-DADAFRUIT_FAULT_RECOVERY={adafruit}",
            str(HERE / "test.c"), "-o", str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True, timeout=10)
