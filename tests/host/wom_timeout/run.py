#!/usr/bin/env python3
"""Actual timeout storage, command and policy bodies; flash/kernel leaves modeled."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get("TRACKER_SOURCE_ROOT", HERE.parents[2]))
sys.path.insert(0, str(HERE.parent / "harness/python"))
from c_extract import extract_block


def function(text, name):
    return extract_block(text, rf"^(?:static )?(?:void|bool|int|int64_t|uint16_t) {name}\([^;{{]*\)\s*\{{")


system = (ROOT / "src/system/system.c").read_text()
power = (ROOT / "src/system/power.c").read_text()
sensor = (ROOT / "src/sensor/sensor.c").read_text()
console = (ROOT / "src/console.c").read_text()
parts = [system[system.index("#if CONFIG_USE_ACTIVE_TIMEOUT\n#define WOM_TIMEOUT_DEFAULT_MS"):
                system.index("static bool ram_retention_valid")]]
# Power lock/change functions precede the setting setter's use via declarations.
parts += [function(power, name) for name in
          ("sys_cancel_WOM_locked", "sys_power_policy_lock", "sys_power_policy_unlock",
           "sys_wom_timeout_change_begin", "sys_wom_timeout_change_end")]
parts += [function(system, "sys_clear")]
parts.append("#if CONFIG_USE_ACTIVE_TIMEOUT && CONFIG_SLEEP_ON_ACTIVE_TIMEOUT && CONFIG_USE_IMU_WAKE_UP\n" +
             function(sensor, "sensor_uses_normal_timeout") + "\n#endif")
parts += [function(sensor, name) for name in
          ("sensor_get_active_timeout_delay", "sensor_update_session_motion")]
parts += [function((ROOT / "src/parse_args.c").read_text(), "parse_long_bounded"),
          function(console, "console_cmd_wom")]
with tempfile.TemporaryDirectory(prefix="tracker-wom-timeout-") as directory:
    temporary = Path(directory)
    (temporary / "production.inc").write_text("\n\n".join(parts))
    for supported, dynamic, default_ms in ((1, 1, 600000), (1, 0, 600000), (0, 0, 600000),
                                           (1, 1, 5000), (1, 1, 90001), (1, 1, 2147483647)):
        binary = temporary / f"timeout-{supported}-{dynamic}-{default_ms}"
        subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
            "-Wno-unused-function", "-g", "-O1", "-fsanitize=address,undefined",
            "-fno-omit-frame-pointer", "-fno-pie", "-no-pie",
            f"-DCONFIG_SLEEP_ON_ACTIVE_TIMEOUT={supported}",
            f"-DCONFIG_DYNAMIC_ACTIVE_TIMEOUT={dynamic}",
            f"-DCONFIG_ACTIVE_TIMEOUT_DELAY={default_ms}",
            "-DCONFIG_CONNECTION_RAW_COLLECTION=1",
            "-I", str(temporary), "-I", str(ROOT / "src"),
            str(HERE / "test.c"), "-o", str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True)
