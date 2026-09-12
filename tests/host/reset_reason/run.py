#!/usr/bin/env python3
"""Exercise production boot bodies against a write-one-to-clear register model.

C++ is used only to model MMIO assignment/read semantics without rewriting the C
bodies. SOURCE_ROOT may point to an archived pre-fix tree for the same repro.
"""

import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
SRC = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2])) / "src"


def function(source, name):
    match = re.search(rf"^(?:static )?(?:inline )?(?:bool|int|void|uint8_t|uint32_t) {name}\([^;\n]*\)[^\n]*\n\{{", source, re.MULTILINE)
    if match is None:
        raise ValueError(f"Missing production function: {name}")
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
    raise ValueError(f"Unclosed production function: {name}")


def registration(source, name):
    return re.search(rf"^SYS_INIT\({name},[^;]+;", source, re.MULTILINE).group()


system = (SRC / "system/system.c").read_text()
watchdog = (SRC / "system/watchdog.c").read_text()
main = (SRC / "main.c").read_text()
parts = []
snapshot = re.search(r"^static uint32_t boot_reset_reason;", system, re.MULTILINE)
if snapshot:
    parts += ["#define HAS_RESET_SNAPSHOT 1", snapshot.group(),
              function(system, "sys_reset_reason_init"),
              function(system, "sys_get_reset_reason"),
              registration(system, "sys_reset_reason_init")]
# The complete live public header also compiles its CONFIG_TASK_WDT=n branch.
parts.append('#include "system/watchdog.h"')
reset_body = function(watchdog, "watchdog_caused_reset")
# Older trees put this query inside the TASK_WDT guard; use their actual stub
# when disabled rather than inventing a compatibility implementation.
if watchdog.index(reset_body) < watchdog.rindex("#endif /* CONFIG_TASK_WDT */"):
    parts += ["#if CONFIG_TASK_WDT", reset_body, "#endif"]
else:
    parts.append(reset_body)
parts.append("#if CONFIG_TASK_WDT")
for name in ("saved_gpregret", "last_reset_was_wdt"):
    declaration = re.search(rf"^static (?:bool|uint8_t) {name}[^;]*;", watchdog, re.MULTILINE)
    if declaration:
        parts.append(declaration.group())
parts += [function(watchdog, "watchdog_early_check"),
          registration(watchdog, "watchdog_early_check"), "#endif"]
parts.append(re.search(r"^static bool ram_retention_valid[^;]*;", system, re.MULTILINE).group())
# Retained validation/NVS restoration follows this policy decision. Stop at that
# boundary to inspect whether reset-pin correctly withholds automatic trust;
# no persistence/CRC logic is copied into this harness.
retained = function(system, "sys_retained_init")
parts += [retained[:retained.index("\t// All contents of NVS")] + "\treturn 0;\n}",
          registration(system, "sys_retained_init"),
          function(system, "sys_button_init"),
          registration(system, "sys_button_init"),
          function(system, "button_read_filtered"),
          function(main, "main").replace("int main(void)", "int tracker_main(void)", 1)]

with tempfile.TemporaryDirectory(prefix="tracker-reset-reason-") as directory:
    temporary = Path(directory)
    (temporary / "zephyr").mkdir()
    (temporary / "zephyr/kernel.h").write_text("#pragma once\n#include <stdint.h>\n")
    (temporary / "production.inc").write_text("\n\n".join(parts))
    for soc in (52, 54):
        for wdt in (1, 0):
            for ignore_reset in (0, 1):
                binary = temporary / f"reset-{soc}-wdt{wdt}-ignore{ignore_reset}"
                command = shlex.split(os.environ.get("CXX", "c++")) + [
                    "-std=c++17", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function",
                    "-Wno-unused-variable", "-Wno-unused-parameter", "-g", "-O1",
                    "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie",
                    f"-DMODEL_SOC={soc}", f"-DIGNORE_RESET={ignore_reset}",
                    "-I", str(temporary), "-I", str(SRC), str(HERE / "test_reset_reason.cpp"),
                    "-o", str(binary),
                ]
                if wdt:
                    command.append("-DCONFIG_TASK_WDT=1")
                subprocess.run(command, check=True)
                # Each invocation gets a genuine fresh boot (zero-initialized
                # cached state), not a test-only reset of production globals.
                reasons = [1, 1 << 20, 2, 0, 1 | 2 | (1 << 20) | (1 << 16)]
                if soc == 54:
                    reasons.append(4)
                for reason in reasons:
                    subprocess.run([str(binary), str(reason)], check=True)
