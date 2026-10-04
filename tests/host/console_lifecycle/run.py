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

HERE = Path(__file__).resolve().parent
SRC = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2])) / "src"
source = (SRC / "console.c").read_text()


def function(name, text=source):
    return extract_block(text, rf"^(?:static )?(?:(?:void|bool|int|size_t) |const struct console_cmd \*){re.escape(name)}\([^;{{]*\)\s*\{{")


parts = ["static void console_thread(void);", "static bool console_feedback_enabled;",
         source[source.index("#define CONSOLE_RESET_CONFIRM_MS"):source.index("\n#if USB_EXISTS || UART_CONSOLE_EXISTS", source.index("#define CONSOLE_RESET_CONFIRM_MS"))],
         source[source.index("static const struct device *const console_uart_dev"):source.index("\n#endif\n\n#if !USB_EXISTS")]]
parts += [function("parse_args", (SRC / "parse_args.c").read_text())]
parts += [function(name) for name in ("console_reject", "console_require_args",
          "console_reset_cancel", "console_reset_confirm", "cmd_sens_set",
          "console_cmd_sens", "console_cmd_reset", "console_cmd_channel", "print_connection")]
declaration_start = source.index("typedef void (*console_cmd_fn)")
parts.append(source[declaration_start:source.index("\n};", declaration_start) + 3])
parts += [function(name) for name in ("console_calibrate_acc", "console_cmd_calibrate", "console_cmd_calibrate_acc_alias")]
table_start = source.index("static const struct console_cmd console_cmds[]")
table = source[table_start:source.index("\n};", table_start) + 3]
# Keep the actual registration and dispatch; unrelated command leaves are inert.
for handler in sorted(set(re.findall(r'\{"[^"]+", (console_cmd_\w+)(?:, \w+)?\}', table))):
    if handler not in ("console_cmd_calibrate", "console_cmd_calibrate_acc_alias",
                       "console_cmd_sens", "console_cmd_reset", "console_cmd_channel"):
        parts.append(f"#define {handler} handle_command")
parts.append(table)
parts += [function(name) for name in ("console_find_command", "console_command_mutates")]
parts += [function(name) for name in ("console_serial_start", "console_serial_end", "console_serial_close", "console_serial_stop", "console_thread")]
with tempfile.TemporaryDirectory(prefix="tracker-console-lifecycle-") as directory:
    temporary = Path(directory)
    (temporary / "production.inc").write_text("\n\n".join(parts))
    for accel_enabled in (0, 1):
        binary = temporary / f"console-lifecycle-accel-{accel_enabled}"
        subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
            "-g", "-O1", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
            "-fno-pie", "-no-pie", "-I", str(temporary), "-I", str(SRC),
            f"-DCONFIG_SENSOR_USE_ACCEL_CALIBRATION={accel_enabled}",
            str(HERE / "test_console.c"), "-o", str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True)
