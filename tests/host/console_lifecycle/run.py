#!/usr/bin/env python3
"""Exercise production editor, lifecycle and worker with host UART/kernel leaves."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
SRC = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2])) / "src"
source = (SRC / "console.c").read_text()


def function(name, text=source):
    match = re.search(rf"^(?:static )?(?:void|int|size_t) {name}\([^;{{]*\)\s*\{{", text, re.MULTILINE)
    start = text.index("{", match.start())
    tokens = re.compile(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]', re.DOTALL)
    depth = 0
    for token in tokens.finditer(text, start):
        if token.group() == "{":
            depth += 1
        elif token.group() == "}":
            depth -= 1
            if depth == 0:
                return text[match.start():token.end()]
    raise ValueError(f"Unclosed function: {name}")


parts = ["static void console_thread(void);",
         source[source.index("static const struct device *const console_uart_dev"):source.index("\n#endif\n\n#if !USB_EXISTS")]]
parts += [function("parse_args", (SRC / "parse_args.c").read_text())]
parts += [function(name) for name in ("console_calibrate_acc", "console_cmd_calibrate", "console_cmd_calibrate_acc_alias")]
table_start = source.index("static const struct console_cmd console_cmds[]")
table = source[table_start:source.index("\n};", table_start) + 3]
# Keep the actual registration and dispatch; unrelated command leaves are inert.
for handler in sorted(set(re.findall(r'\{"[^"]+", (console_cmd_\w+)\}', table))):
    if handler not in ("console_cmd_calibrate", "console_cmd_calibrate_acc_alias"):
        parts.append(f"#define {handler} handle_command")
parts.append(table)
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
