#!/usr/bin/env python3
"""Host tests for shared-bus suspend gating and link-gated MCUboot trial confirmation."""

import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "harness/python"))
from c_extract import extract_block as block

HERE = Path(__file__).resolve().parent
SRC = Path(os.environ.get("TRACKER_SOURCE_ROOT", HERE.parents[2] / "src"))


def function(source, name):
    return block(source, rf"^(?:static )?(?:bool|int|void) {re.escape(name)}\([^;\n]*\)[^\n]*\n\{{")


power = (SRC / "system/power.c").read_text()

macro_start = power.index("#define IS_TRACKER_SENSOR_NODE(node)")
macro_end = power.index("static int sys_interface_action(", macro_start)
parts = [
    power[macro_start:macro_end],
    function(power, "sys_interface_action"),
    function(power, "sys_interface_suspend"),
    function(power, "sys_interface_resume"),
    "#if defined(CONFIG_BOOTLOADER_MCUBOOT) && IS_ENABLED(CONFIG_MCUBOOT_CONFIRM_REQUIRE_LINK)\n"
    + function(power, "power_mcuboot_trial_confirm_ready")
    + "\n#endif",
]

with tempfile.TemporaryDirectory(prefix="tracker-mcuboot-ota-") as directory:
    temp = Path(directory)
    (temp / "production.inc").write_text("\n\n".join(parts))

    cc = shlex.split(os.environ.get("CC", "cc"))
    sanitizers = (
        ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
        if os.environ.get("HOST_SANITIZERS", "1") != "0"
        else []
    )

    for shared_mag_bus in (0, 1):
        binary = temp / f"test_mcuboot_ota_{shared_mag_bus}"
        cmd = cc + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-g", "-O1",
            *sanitizers,
            "-DCONFIG_BOOTLOADER_MCUBOOT=1",
            "-DCONFIG_MCUBOOT_CONFIRM_REQUIRE_LINK=1",
            "-DCONFIG_MCUBOOT_CONFIRM_LINK_DEADLINE_MS=180000",
            f"-DHOST_SHARED_MAG_BUS={shared_mag_bus}",
            "-I", str(temp),
            str(HERE / "test_mcuboot_ota.c"),
            "-o", str(binary),
        ]
        subprocess.run(cmd, check=True)
        subprocess.run([str(binary)], check=True)
