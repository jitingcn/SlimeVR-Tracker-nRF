#!/usr/bin/env python3
"""Real T-Cal backup CLI, parser, model refresh and MLS/LUT; modeled NVS/HW.

Default executes sanitizer and fast-math variants. --export FILE emits a full
model payload file; --smoke FILE erases the modeled RAM/NVS and feeds the whole
line through the production UART IRQ and worker, then cold-restores the records.
"""
import argparse
import base64
import os
from pathlib import Path
import re
import shlex
import subprocess
import struct
import sys
import tempfile
import zlib

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2])).resolve()
sys.path.insert(0, str(HERE.parent / "harness/python"))
from c_extract import extract_block


def function(source, name):
    return extract_block(source, rf"^(?:static )?(?:(?:void|bool|int|size_t|uint32_t|sensor_tcal_apply_mode_t) |const char \*){re.escape(name)}\([^;{{]*\)\s*\{{")


def check_crc(path):
    lines = path.read_text().splitlines()
    assert len(lines) == 1 and len(lines[0]) == 1528
    packet = base64.b64decode(lines[0], validate=True)
    assert base64.b64encode(packet).decode() == lines[0]
    assert struct.unpack_from("<4I", packet) == (0x324C4354, 10, 45, 2)
    assert len(packet) == 24 + 70 * 16
    words = struct.unpack("<286I", packet)
    assert struct.pack("<286I", *words) == packet
    expected = zlib.crc32(packet[:-4])
    assert words[-1] == expected
    print(f"independent Python canonical LE bytes/one-line CRC-32 OK: {expected:08x}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--export", type=Path)
    mode.add_argument("--smoke", type=Path)
    mode.add_argument("--rtt", action="store_true", help="run only the real RTT line-reader regression")
    args = parser.parse_args()
    runtime = (ROOT / "src/sensor/calibration/tcal_runtime.c").read_text()
    start = runtime.index("/* V2: canonical little-endian words,")
    command = function(runtime, "sensor_tcal_backup_command")
    end = runtime.index(command, start) + len(command)
    production = "\n\n".join(function(runtime, name) for name in (
        "sensor_tcal_refresh_apply_cache", "sensor_tcal_clear_doffset",
        "sensor_tcal_refresh_model", "sensor_tcal_get_apply_mode",
        "sensor_tcal_get_apply_mode_name", "sensor_tcal_runtime_init_from_retained"))
    backup = runtime[start:end] + "\n" + function((ROOT / "src/parse_args.c").read_text(), "parse_args")
    console = (ROOT / "src/console.c").read_text()
    console_parts = [console[console.index("static const struct device *const console_uart_dev"):
                             console.index("\n#endif\n\n#if !USB_EXISTS")]]
    console_parts += [function(console, name) for name in (
        "console_serial_start", "console_serial_end", "console_serial_close",
        "console_serial_stop", "console_thread")]
    console_production = "\n\n".join(console_parts)
    rtt_worker = function(console, "console_thread").replace("console_thread(void)", "console_rtt_thread(void)", 1)
    zephyr = Path(os.environ.get("ZEPHYR_BASE", (ROOT.parent / "sdk-nrf").resolve().parent / "zephyr"))
    crc_source = (zephyr / "subsys/crc/crc32_sw.c").read_text()
    calibration = (ROOT / "src/sensor/calibration/calibration.c").read_text()
    header = (ROOT / "src/sensor/calibration/calibration.h").read_text()
    ownership = '#include "ownership.h"\n'
    for name in ("sensor_calibration_request_id", "cal_request_origin"):
        ownership += extract_block(header, rf"^enum {name}\s*\{{", semicolon=True) + "\n"
    for name in ("sensor_calibration_maintenance_begin", "sensor_calibration_maintenance_end", "sensor_calibration_request"):
        ownership += function(calibration, name) + "\n"
    accum_end = runtime.index("} tcal_accum;") + len("} tcal_accum;")
    accum_start = runtime.rindex("static struct {", 0, accum_end)
    ownership += runtime[accum_start:accum_end] + "\n"
    ownership += "static atomic_t tcal_accum_reset_generation;\nstatic uint32_t tcal_accum_seen_reset_generation;\n"
    for name in ("tcal_accum_reset", "tcal_accum_request_reset", "tcal_accum_apply_reset", "update_tcal_state", "tcal_save_point"):
        ownership += function(runtime, name) + "\n"
    finite = function((ROOT / "src/util.c").read_text(), "v_finite")
    with tempfile.TemporaryDirectory(prefix="tcal-backup-") as directory:
        work = Path(directory)
        for name, content in {
            "globals.h": '#include "adapter.h"\n',
            "zephyr/kernel.h": (HERE / "kernel.h").read_text(),
            "sensor/sensor.h": "",
            "util.h": "#pragma once\n" + finite,
            "production.inc": production,
            "backup.inc": backup,
            "console_production.inc": console_production,
            "rtt_worker.inc": rtt_worker,
            "crc_production.inc": crc_source,
            "zephyr/sys/crc.h": "#include <stdint.h>\n#include <stddef.h>\n#define __weak\nuint32_t crc32_ieee_update(uint32_t, const uint8_t *, size_t);\n",
            "ownership.inc": ownership,
            "SEGGER_RTT.h": "",
            "rtt_production.h": f'#include "{ROOT}/src/system/rtt_console.h"\n',
        }.items():
            destination = work / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_text(content)
        variants = ("sanitized",) if args.export or args.smoke else ("sanitized", "fast-math")
        for variant in variants:
            modes = ("backup",) if args.export or args.smoke else (("rtt",) if args.rtt else ("backup", "ownership", "rtt"))
            for test_mode in modes:
                actual_ownership = test_mode == "ownership"
                source = work / "main.c"
                source.write_text('#include <zephyr/kernel.h>\n#include "adapter.h"\n'
                                  f'#include "{ROOT}/src/sensor/calibration/tcal_mls_lut.c"\n'
                                  '#include "production.inc"\n'
                                  '#include "crc_production.inc"\n'
                                  + ('#include "ownership.inc"\n' if actual_ownership else "")
                                  + '#include "backup.inc"\n'
                                  + f'#include "test_{test_mode}.c"\n')
                binary = work / f"{variant}-{test_mode}"
                flags = (["-O1", "-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
                          "-fno-pie", "-no-pie"] if variant == "sanitized" else ["-O2", "-ffast-math"])
                subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
                    "-D_GNU_SOURCE", "-std=gnu11", "-Wall", "-Wextra", "-Werror",
                    "-Wno-unused-function", "-pthread", *flags,
                    f"-DACTUAL_OWNERSHIP={int(actual_ownership)}",
                    f"-DCONFIG_SENSOR_TCAL_HEATED={int(not actual_ownership)}",
                    "-I", str(work), "-I", str(HERE), str(source), "-lm", "-o", str(binary),
                ], check=True)
                if args.export:
                    subprocess.run([str(binary), "--export", str(args.export.resolve())], check=True)
                    check_crc(args.export)
                elif args.smoke:
                    check_crc(args.smoke)
                    subprocess.run([str(binary), "--smoke", str(args.smoke.resolve())], check=True)
                else:
                    subprocess.run([str(binary)], check=True)
                    if test_mode == "backup":
                        exported = work / "exported.txt"
                        subprocess.run([str(binary), "--export", str(exported)], check=True)
                        check_crc(exported)
                        subprocess.run([str(binary), "--smoke", str(exported)], check=True)


if __name__ == "__main__":
    main()
