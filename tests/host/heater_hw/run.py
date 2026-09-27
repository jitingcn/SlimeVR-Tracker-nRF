#!/usr/bin/env python3
"""Exercise the complete production wrapper against a deterministic register model."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
source = (ROOT / "src/system/heater.c").read_text()
# SDK declarations are supplied by the register model, not replacement logic.
source = re.sub(r'^#include[^\n]*\n', '', source, flags=re.M)
with tempfile.TemporaryDirectory(prefix="heater-hw-") as directory:
    temporary = Path(directory)
    (temporary / "production.inc").write_text(source)
    binary = temporary / "heater-hw"
    subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
        "-std=c11", "-O2", "-ffast-math", "-Wall", "-Wextra", "-Werror",
        "-fsanitize=undefined", f"-I{temporary}", f"-I{ROOT / 'src/system'}",
        str(HERE / "test.c"), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
