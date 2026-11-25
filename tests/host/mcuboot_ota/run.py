#!/usr/bin/env python3
"""Host tests for shared-bus suspend and MCUboot boot-success control flow."""

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

# The extracted check owns eligibility, confirmation, watchdog marking and
# its completion latch; ensure the real power loop still invokes that check.
if not re.search(
    r"\bpower_check_boot_success\s*\(\s*system_uptime_since_boot_ms\s*\(\s*\)\s*\)\s*;",
    function(power, "power_thread"),
):
    raise ValueError("power_thread no longer invokes the boot-success check")

macro_start = power.index("#define IS_TRACKER_SENSOR_NODE(node)")
macro_end = power.index("static int sys_interface_action(", macro_start)
parts = [
    power[macro_start:macro_end],
    function(power, "sys_interface_action"),
    function(power, "sys_interface_suspend"),
    function(power, "sys_interface_resume"),
    function(power, "power_check_boot_success"),
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

    common = ["shared_bus", "boot_success"]
    nonrevertible = ["none", "test", "perm", "confirmed_late"]
    trials = [
        "trial_retry", "unknown_revert", "unknown_none",
        "unknown_healthy_panic", "unknown_healthy_eio",
        "deadline_trial_no_link", "deadline_trial_link",
        "deadline_panic_no_link", "deadline_panic_link",
        "deadline_eio_no_link", "deadline_eio_link", "confirmed_after_failure",
    ]
    variants = (
        ("no_mcuboot", False, False, 0, common),
        ("link_off", True, False, 0, common + nonrevertible),
        ("link_on", True, True, 180000, common + nonrevertible + trials),
        ("minimum_deadline", True, True, 61000, ["minimum_deadline"]),
    )

    for shared_mag_bus in (0, 1):
        for name, mcuboot, require_link, deadline, cases in variants:
            binary = temp / f"test_mcuboot_ota_{shared_mag_bus}_{name}"
            cmd = cc + [
                "-std=c11", "-Wall", "-Wextra", "-Werror", "-g", "-O1",
                *sanitizers,
                f"-DCONFIG_MCUBOOT_CONFIRM_REQUIRE_LINK={int(require_link)}",
                f"-DHOST_SHARED_MAG_BUS={shared_mag_bus}",
                "-I", str(temp),
                str(HERE / "test_mcuboot_ota.c"),
                "-o", str(binary),
            ]
            if mcuboot:
                cmd.append("-DCONFIG_BOOTLOADER_MCUBOOT=1")
            if require_link:
                cmd.append(f"-DCONFIG_MCUBOOT_CONFIRM_LINK_DEADLINE_MS={deadline}")
            subprocess.run(cmd, check=True)
            for case in cases:
                subprocess.run([str(binary), case], check=True)
