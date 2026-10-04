#!/usr/bin/env python3
"""Run the complete production tracker with deterministic storage/clock leaves."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2]))
source = (ROOT / "src/system/battery_tracker.c").read_text()
source = re.sub(r'^#include[^\n]*\n', '', source, flags=re.MULTILINE)
header = (ROOT / "src/system/battery_tracker.h").read_text()
with tempfile.TemporaryDirectory(prefix="battery-tracker-") as directory:
    tmp = Path(directory)
    (tmp / "production.inc").write_text(header + "\n" + source)
    binary = tmp / "test"
    subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
        "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-Wno-type-limits",
        "-g", "-O1", "-fsanitize=address,undefined", "-fno-sanitize-recover=all",
        "-fno-omit-frame-pointer", "-fno-pie", "-no-pie", f"-I{tmp}",
        str(HERE / "test_tracker.c"), "-o", str(binary)], check=True)
    for scenario in ("retry", "short", "partial", "curve", "curve_failure", "reset", "reset_failure", "migration", "insufficient", "gaps"):
        subprocess.run([str(binary), scenario], check=True)
