#!/usr/bin/env python3
"""Exercise production editor, lifecycle and worker with host UART/kernel leaves."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'harness/python'))
from c_extract import extract_block
from console_extract import console_table

HERE = Path(__file__).resolve().parent
SRC = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2])) / "src"
baseline = os.environ.get("BASELINE_REV")
source = (subprocess.check_output(["rtk", "proxy", "git", "show", f"{baseline}:src/console.c"],
                                  cwd=HERE.parents[2], text=True) if baseline else
          (SRC / "console.c").read_text())


def function(name, text=source):
    return extract_block(text, rf"^(?:static )?(?:(?:void|bool|int|size_t|uint64_t) |const struct console_cmd \*){re.escape(name)}\([^;{{]*\)\s*\{{")


parts = ["static void console_thread(void);", "static bool console_feedback_enabled;",
         source[source.index("#define CONSOLE_RESET_CONFIRM_MS"):source.index("\n#if USB_EXISTS || UART_CONSOLE_EXISTS", source.index("#define CONSOLE_RESET_CONFIRM_MS"))],
         source[source.index("static const struct device *const console_uart_dev"):source.index("\n#endif\n\n#if !USB_EXISTS")]]
parts += [function(name, (SRC / "parse_args.c").read_text()) for name in
          ("parse_args", "parse_u64", "parse_long_bounded", "parse_float_triplet")]
parts += [function(name) for name in ("console_reject", "console_require_args",
          "console_reset_cancel", "console_reset_confirm", "cmd_sens_set", "cmd_shutdown",
          "console_cmd_sens", "console_cmd_reset", "console_cmd_channel",
          "console_cmd_radio", "console_cmd_set", "console_cmd_shutdown", "console_cmd_reboot",
          "console_cmd_scan", "console_cmd_pair", "console_cmd_clear", "console_cmd_clearchannel",
          "console_cmd_debug", "print_connection")]
parts.append(function("console_cmd_unavailable"))
parts.append("#if CONFIG_SENSOR_USE_MAGNETOMETER\n" +
             function("print_mag_calibration_status") + "\n" +
             function("console_cmd_mag") + "\n#endif")
declaration_start = source.index("typedef void (*console_cmd_fn)")
parts.append(source[declaration_start:source.index("\n};", declaration_start) + 3])
parts += [function(name) for name in ("console_calibrate_acc", "console_cmd_calibrate", "console_cmd_calibrate_acc_alias")]
table_start = source.index("static const struct console_cmd console_cmds[]")
table = source[table_start:source.index("\n};", table_start) + 3]
# Keep the actual registration and dispatch; unrelated command leaves are inert.
for handler in sorted(set(re.findall(r'\{"[^"]+",\s*(console_cmd_\w+)', table))):
    if handler not in ("console_cmd_calibrate", "console_cmd_calibrate_acc_alias",
                       "console_cmd_sens", "console_cmd_reset", "console_cmd_channel",
                       "console_cmd_radio", "console_cmd_set", "console_cmd_mag", "console_cmd_unavailable",
                       "console_cmd_shutdown", "console_cmd_reboot", "console_cmd_scan", "console_cmd_pair",
                       "console_cmd_clear", "console_cmd_clearchannel", "console_cmd_debug"):
        parts.append(f"#define {handler} handle_command")
parts.append(table)
parts += [function(name) for name in ("console_find_command", "console_command_mutates")]
parts += [function(name) for name in ("console_serial_start", "console_serial_end", "console_serial_close", "console_serial_stop", "console_thread")]
with tempfile.TemporaryDirectory(prefix="tracker-console-lifecycle-") as directory:
    temporary = Path(directory)
    (temporary / "production.inc").write_text("\n\n".join(parts))
    for accel_enabled in (0, 1):
        for raw_enabled in (0, 1):
            for mag_enabled in (0, 1):
                binary = temporary / f"console-lifecycle-{accel_enabled}-{raw_enabled}-{mag_enabled}"
                subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
                    "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
                    "-g", "-O1", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
                    "-fno-pie", "-no-pie", "-I", str(temporary), "-I", str(SRC),
                    f"-DCONFIG_SENSOR_USE_ACCEL_CALIBRATION={accel_enabled}",
                    f"-DCONFIG_CONNECTION_RAW_COLLECTION={raw_enabled}",
                    f"-DCONFIG_SENSOR_USE_MAGNETOMETER={mag_enabled}",
                    str(HERE / "test_console.c"), "-o", str(binary),
                ], check=True)
                subprocess.run([str(binary)], check=True)
    binary = temporary / "uart-reset"
    subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
        "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
        "-Wno-unused-function", "-Wno-unused-variable",
        "-g", "-O1", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
        "-fno-pie", "-no-pie", "-I", str(temporary), "-I", str(SRC),
        "-DCONSOLE_TEST_UART=1", "-DCONFIG_SENSOR_USE_ACCEL_CALIBRATION=0",
        "-DCONFIG_CONNECTION_RAW_COLLECTION=0", "-DCONFIG_SENSOR_USE_MAGNETOMETER=0",
        str(HERE / "test_console.c"), "-o", str(binary),
    ], check=True)
    subprocess.run([str(binary)], check=True)
    rtt_parts = [
        source[source.index("#define CONSOLE_RESET_CONFIRM_MS"):
               source.index("\n#if USB_EXISTS || UART_CONSOLE_EXISTS", source.index("#define CONSOLE_RESET_CONFIRM_MS"))],
        function("parse_args", (SRC / "parse_args.c").read_text()),
        *[function(name) for name in ("console_reject", "console_require_args",
                                      "console_reset_cancel", "console_reset_confirm",
                                      "console_cmd_reset")],
        console_table(source, {"console_cmd_reset"}),
        function("console_thread"),
    ]
    (temporary / "production_rtt.inc").write_text("\n\n".join(rtt_parts))
    (temporary / "SEGGER_RTT.h").write_text("")
    binary = temporary / "rtt-reset"
    subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
        "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
        "-g", "-O1", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
        "-fno-pie", "-no-pie", "-I", str(temporary), "-I", str(SRC),
        str(HERE / "test_rtt_reset.c"), "-o", str(binary),
    ], check=True)
    subprocess.run([str(binary)], check=True)
    tcal_parts = [
        function(name, (SRC / "parse_args.c").read_text())
        for name in ("parse_args", "parse_long_bounded", "parse_float_triplet")
    ]
    tcal_parts += [function("console_reject"), function("console_require_args")]
    tcal_parts += ["#if CONFIG_SENSOR_USE_SENS_CALIBRATION\n" + function("cmd_sens_auto") + "\n" +
                   function("console_cmd_sens") + "\n#endif"]
    tcal_parts += ["#if CONFIG_SENSOR_USE_TCAL\n#if CONFIG_SENSOR_TCAL_HEATED\n" +
                   function("console_cmd_tcal_heat") + "\n#endif\n" +
                   function("console_cmd_tcal") + "\n#endif"]
    tcal_parts += [function(name) for name in ("console_cmd_unavailable", "console_cmd_ping", "console_cmd_tdma")]
    tcal_parts += ["#if DFU_EXISTS\n" + function("console_cmd_dfu") + "\n#endif",
                   "#if CONFIG_VQF_BENCH\n" + function("console_cmd_vqfbench") + "\n#endif"]
    tcal_parts += [console_table(source, {"console_cmd_sens", "console_cmd_tcal", "console_cmd_unavailable",
                                         "console_cmd_ping", "console_cmd_tdma", "console_cmd_dfu",
                                         "console_cmd_vqfbench"}),
                   function("console_thread")]
    (temporary / "production_tcal.inc").write_text("\n\n".join(tcal_parts))
    for tcal_enabled, sens_enabled, heated, diagnostics, dfu, adafruit, bench in (
            (1, 1, 0, 0, 0, 0, 0), (1, 1, 1, 1, 0, 0, 0), (0, 0, 0, 0, 0, 0, 0),
            (0, 0, 0, 1, 0, 0, 0), (1, 0, 0, 0, 0, 0, 0), (0, 1, 0, 1, 0, 0, 0),
            (1, 1, 0, 0, 1, 1, 1), (1, 1, 0, 0, 1, 0, 1)):
        binary = temporary / f"console-tcal-{tcal_enabled}-{sens_enabled}-{heated}-{diagnostics}-{dfu}-{adafruit}-{bench}"
        flags = [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
            "-Wno-unused-function", "-g", "-O1", "-fsanitize=address,undefined",
            "-fno-omit-frame-pointer", "-fno-pie", "-no-pie",
            f"-DCONFIG_SENSOR_USE_TCAL={tcal_enabled}",
            f"-DCONFIG_SENSOR_USE_SENS_CALIBRATION={sens_enabled}",
            f"-DCONFIG_SENSOR_TCAL_HEATED={heated}",
            f"-DDFU_EXISTS={dfu}", f"-DADAFRUIT_BOOTLOADER={adafruit}", f"-DCONFIG_VQF_BENCH={bench}",
        ]
        if diagnostics:
            flags.append("-DCONFIG_TDMA_DIAGNOSTICS=1")
        subprocess.run(shlex.split(os.environ.get("CC", "cc")) + flags + [
            "-I", str(temporary), "-I", str(SRC), str(HERE / "test_tcal.c"), "-lm", "-o", str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True)
