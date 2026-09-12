#!/usr/bin/env python3
"""Exercise current radio/session functions with scheduler and hardware leaves stubbed."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
SRC = HERE.parents[2] / "src"
SOURCE = Path(os.environ.get("RADIO_SOURCE_ROOT", SRC))


def block(source, pattern, semicolon=False):
    match = re.search(pattern, source, re.MULTILINE)
    if match is None:
        raise ValueError(f"Production construct not found: {pattern}")
    start = source.index("{", match.start())
    tokens = re.compile(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]', re.DOTALL)
    depth = 0
    for token in tokens.finditer(source, start):
        if token.group() == "{":
            depth += 1
        elif token.group() == "}":
            depth -= 1
            if depth == 0:
                return source[match.start():token.end() + int(semicolon)]
    raise ValueError(f"Unclosed production construct: {pattern}")


def function(source, name):
    return block(source, rf"^(?:static )?(?:bool|int|void|uint8_t|uint16_t) {name}\([^;]*?\)\n\{{")

esb = (SOURCE / "connection/esb.c").read_text()
connection = (SOURCE / "connection/connection.c").read_text()
header = (SRC / "connection/esb.h").read_text()
constants = "\n".join(re.findall(r"^#define (?:ESB_|PING_INTERVAL_MS)[^\n]*", header, re.MULTILINE))
payload = constants + "\n" + "\n".join(re.findall(r"^static struct esb_payload tx_payload = .*;", esb, re.MULTILINE))
payload += "\n" + function(esb, "esb_write")
commands = [constants, block(esb, r"^struct esb_remote_cmd \{", True)]
commands.extend(re.findall(r"^static (?:bool remote_command_rejected|uint32_t remote_command_generation);", esb, re.MULTILINE))
registry = block(esb, r"^static const struct esb_remote_cmd esb_remote_cmds\[\] = \{", True)
actual_handlers = ("esb_remote_cmd_shutdown", "esb_remote_cmd_data_collect_batch_on", "esb_remote_cmd_data_collect_batch_off")
for name in actual_handlers:
    commands.append(function(esb, name))
for name in sorted(set(re.findall(r", (esb_remote_cmd_\w+)\}", registry)) - set(actual_handlers)):
    commands.append(f"static void {name}(void) {{ }}")
commands.append(function((SRC / "system/system.c").read_text(), "sys_command_shutdown"))
commands += [registry, function(esb, "esb_remote_command_execute"), function(esb, "esb_thread")]
# Exercise the production control-admission tail after PONG validation.
start = esb.index("\t\t\t\t\tif (pong_flags == ESB_PONG_FLAG_DATA_COLLECT_METADATA)")
end = esb.index("\n\t\t\t\t}\n\t\t\t} break;", start)
commands.append("static void receive_control(uint8_t pong_flags) {\n"
                + "float pong_sens_data[3] = {0}; uint8_t pong_sens_auto_axis = 0; uint16_t pong_sens_auto_revolutions = 0;\n"
                + esb[start:end] + "\n}")
commands.append("static void receive_stop(void) { receive_control(ESB_PONG_FLAG_DATA_COLLECT_BATCH_OFF); }")
collection = []
for name in ("connection_raw_collection_active", "connection_reset_raw_collection", "connection_set_data_collection", "connection_get_data_collection", "connection_set_data_collection_batch", "connection_get_data_collection_batch", "connection_get_data_collection_batch_rate", "connection_send_raw_metadata"):
    collection.append(function(connection, name))
# Old API snapshots return void. Adapt only the test observation, not production
# control flow, so pre-fix comparison fails on behavior rather than compilation.
returns_status = re.search(r"^int connection_set_data_collection_batch\(", connection, re.MULTILINE)
collection.append("static int batch_request(bool enable, uint16_t rate) { " + ("return connection_set_data_collection_batch(enable, rate);" if returns_status else "connection_set_data_collection_batch(enable, rate); return 0;") + " }")

with tempfile.TemporaryDirectory(prefix="tracker-radio-sessions-") as directory:
    temporary = Path(directory)
    (temporary / "connection").mkdir()
    (temporary / "connection/connection.h").write_text((SOURCE / "connection/connection.h").read_text())
    for name, parts in (("payload", payload), ("commands", "\n\n".join(commands)), ("collection", "\n\n".join(collection))):
        (temporary / f"{name}.inc").write_text(parts)
    for name in ("payload", "commands", "collection"):
        if os.environ.get("RADIO_CASE") not in (None, name):
            continue
        binary = temporary / name
        command = shlex.split(os.environ.get("CC", "cc")) + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter", "-g", "-O1",
            "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie",
            "-I", str(temporary), "-I", str(SRC), "-I", str(HERE / "../raw_collection"),
            str(HERE / f"test_{name}.c"), "-lm", "-o", str(binary),
        ]
        subprocess.run(command, check=True)
        subprocess.run([str(binary)], check=True)
