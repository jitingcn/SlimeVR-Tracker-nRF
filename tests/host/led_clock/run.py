#!/usr/bin/env python3
"""Compile the production read-only clock snapshot against deterministic clocks."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
SRC = HERE.parents[2] / "src/connection"
esb = (SRC / "esb.c").read_text()
tdma = (SRC / "tdma.c").read_text()


def function(source, name):
    match = re.search(r"^[\w *]+\b" + name + r"\([^;]*?\n\{", source, re.M)
    if match is None:
        raise ValueError(name)
    end = source.index("\n}", match.end()) + 2
    return source[match.start():end]


macros = []
for source, names in ((tdma, ("TDMA_UNPACK_SLOT", "TDMA_UNPACK_STICKS", "TDMA_UNPACK_FRAME", "TDMA_MIN_SAFE_SLOT_TICKS", "TDMA_SYNC_STALE_MS")),
                      ((SRC / "esb.h").read_text(), ("PING_INTERVAL_MS",))):
    for name in names:
        macros.append(re.search(r"^#define " + name + r"\b[^\n]*", source, re.M).group())
production = "\n".join(macros + [function(tdma, "tdma_config_snapshot"),
    function(tdma, "tdma_status_clock_max_age_ms"),
    function(esb, "net_ticks_from_kernel64"), function(esb, "esb_get_status_clock")])
with tempfile.TemporaryDirectory(prefix="tracker-led-clock-") as directory:
    temporary = Path(directory)
    (temporary / "production.inc").write_text(production)
    for kernel_hz in (32768, 31250):
        for enabled in (0, 1):
            binary = temporary / f"clock-{kernel_hz}-{enabled}"
            subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
                "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function",
                "-g", "-O1", "-fsanitize=undefined", f"-DKERNEL_HZ={kernel_hz}",
                f"-DCONFIG_CONNECTION_TDMA={enabled}", f"-I{temporary}",
                str(HERE / "test.c"), "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)
