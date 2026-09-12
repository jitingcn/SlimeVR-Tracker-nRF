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


def function(name):
    match = re.search(rf"^(?:static )?(?:void|int) {name}\([^;{{]*\)\s*\{{", source, re.MULTILINE)
    start = source.index("{", match.start())
    tokens = re.compile(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]', re.DOTALL)
    depth = 0
    for token in tokens.finditer(source, start):
        if token.group() == "{":
            depth += 1
        elif token.group() == "}":
            depth -= 1
            if depth == 0:
                return source[match.start():token.end()]
    raise ValueError(f"Unclosed function: {name}")


parts = ["static void console_thread(void);",
         source[source.index("static const struct device *const console_uart_dev"):source.index("\n#endif\n\n#if !USB_EXISTS")]]
parts += [function(name) for name in ("console_serial_start", "console_serial_end", "console_serial_close", "console_serial_stop", "console_thread")]
with tempfile.TemporaryDirectory(prefix="tracker-console-lifecycle-") as directory:
    temporary = Path(directory)
    (temporary / "production.inc").write_text("\n\n".join(parts))
    binary = temporary / "console-lifecycle"
    subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
        "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
        "-g", "-O1", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
        "-fno-pie", "-no-pie", "-I", str(temporary), "-I", str(SRC),
        str(HERE / "test_console.c"), "-o", str(binary),
    ], check=True)
    subprocess.run([str(binary)], check=True)
