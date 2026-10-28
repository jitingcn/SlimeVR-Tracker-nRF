#!/usr/bin/env python3
"""Exercise production retained validation and reset with modeled kernel/NVS leaves."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "harness/python"))
from c_extract import extract_block

HERE = Path(__file__).resolve().parent
SRC = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2])) / "src"
baseline = os.environ.get("BASELINE_REV")


def source(path):
    if baseline:
        return subprocess.check_output(["rtk", "proxy", "git", "show", f"{baseline}:src/{path}"],
                                       cwd=HERE.parents[2], text=True)
    return (SRC / path).read_text()


retained = source("retained.c")
system = source("system/system.c")
build = source("build_defines.h")

def function(text, name):
    return extract_block(text, rf"^(?:static )?(?:int|void|bool) {re.escape(name)}\([^;{{]*\)\s*\{{")


names = ["retained_defaults", "retained_reset"] if "void retained_reset(" in retained else []
names += ["retained_init", "retained_validate", "retained_update"]
timestamp = build[build.index("#define COMPUTE_BUILD_YEAR"):build.index("\n#define TOSTRING")]

with tempfile.TemporaryDirectory(prefix="tracker-retained-") as directory:
    temp = Path(directory)
    (temp / "production_retained.inc").write_text("\n\n".join(function(retained, name) for name in names))
    (temp / "production_system_clear.inc").write_text(function(system, "sys_clear"))
    (temp / "test_timestamp.inc").write_text(timestamp)
    cc = shlex.split(os.environ.get("CC", "cc"))
    common = ["-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function",
              "-Wno-sign-compare", "-Wno-builtin-macro-redefined", "-g", "-O1", "-fsanitize=address,undefined",
              "-fno-omit-frame-pointer", "-fno-pie", "-no-pie",
              '-D__DATE__="Oct 29 2026"', "-DCONFIG_SENSOR_VQF_OUTPUT_OBSERVER=0",
              "-DCONFIG_SENSOR_USE_MAGNETOMETER=0", "-DCONFIG_SENSOR_USE_SENS_CALIBRATION=0",
              "-DCONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION=0",
              "-DCONFIG_SENSOR_POLY_TEMP_MIN=0", "-DCONFIG_SENSOR_POLY_TEMP_MAX=2",
              "-DCONFIG_SENSOR_POLY_STEPS_PER_DEGREE=1", "-DCONFIG_SENSOR_POLY_DEGREE=2",
              "-I", str(temp), "-I", str(SRC)]
    for tcal, bootcal in [(0, False), (1, False), (1, True)]:
        config = [f"-DCONFIG_SENSOR_USE_TCAL={tcal}"]
        if bootcal:
            config.append("-DCONFIG_SENSOR_USE_BOOT_CALIBRATION=1")
        objects = []
        for name, clock, implementation in [("retained", "12:34:56", True), ("system", "12:34:57", False)]:
            obj = temp / f"{name}.o"
            flags = [f'-D__TIME__="{clock}"']
            if implementation:
                flags.append("-DRETAINED_TEST_IMPLEMENTATION=1")
            subprocess.run(cc + common + config + flags + ["-c", str(HERE / "test_retained.c"), "-o", str(obj)], check=True)
            objects.append(str(obj))
        binary = temp / "test_retained"
        subprocess.run(cc + common + objects + ["-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
