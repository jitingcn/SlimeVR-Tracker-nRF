#!/usr/bin/env python3
"""Exercise production boot reconstruction and the coherent T-Cal snapshot."""
import argparse
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

from run import function

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2])).resolve()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--boot-only", action="store_true")
    args = parser.parse_args()
    source = (ROOT / "src/sensor/calibration/tcal_runtime.c").read_text()
    names = ("sensor_tcal_refresh_apply_cache", "sensor_tcal_runtime_init_from_retained")
    if not args.boot_only:
        names += ("sensor_tcal_snapshot",)
    with tempfile.TemporaryDirectory(prefix="tcal-recovery-") as directory:
        work = Path(directory)
        (work / "recovery.inc").write_text("\n\n".join(function(name, source) for name in names))
        executable = work / "recovery"
        subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
            "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
            "-fsanitize=undefined", "-fno-sanitize-recover=undefined",
            *(["-DRECOVERY_BOOT_ONLY"] if args.boot_only else []),
            "-I", str(work), str(HERE / "test_recovery.c"), "-lm", "-o", str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
