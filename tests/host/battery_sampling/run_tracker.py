#!/usr/bin/env python3
"""Run the complete production tracker with deterministic storage/clock leaves."""
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
ROOT = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2]))
source = (ROOT / "src/system/battery_tracker.c").read_text()
source = re.sub(r'^#include[^\n]*\n', '', source, flags=re.MULTILINE)
header = (ROOT / "src/system/battery_tracker.h").read_text()
console = (ROOT / "src/console.c").read_text()
console_functions = "\n".join(
    extract_block(console, rf"^static void {name}\([^;\n]*\)\s*\{{")
    for name in ("print_battery", "print_uptime", "print_battery_tracker")
)
with tempfile.TemporaryDirectory(prefix="battery-tracker-") as directory:
    tmp = Path(directory)
    (tmp / "production.inc").write_text(header + "\n" + source)
    (tmp / "console.inc").write_text(console_functions)
    binary = tmp / "test"
    subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
        "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-Wno-type-limits",
        "-g", "-O1", "-fsanitize=address,undefined", "-fno-sanitize-recover=all",
        "-fno-omit-frame-pointer", "-fno-pie", "-no-pie", f"-I{tmp}",
        str(HERE / "test_tracker.c"), "-o", str(binary)], check=True)
    for scenario in ("retry", "short", "partial", "curve", "curve_failure", "reset", "reset_failure", "migration", "insufficient", "gaps", "console"):
        subprocess.run([str(binary), scenario], check=True)
