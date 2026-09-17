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
clocked_writer = re.search(r"^static int esb_write_clocked\(", esb, re.MULTILINE)
if clocked_writer:
    payload += "\nstatic atomic_t tx_clock_users;\n"
payload += "\n" + function(esb, "clocks_stop")
if clocked_writer:
    payload += "\n" + function(esb, "esb_release_tx_clock")
    payload += "\n" + function(esb, "esb_write_clocked")
payload += "\n" + function(esb, "esb_write")
if re.search(r"^int esb_write_ping\(", esb, re.MULTILINE):
    payload += "\n" + function(esb, "esb_write_ping")
    payload += """
static int host_send_ping(uint8_t *data, bool force)
{
    return esb_write_ping(data, force);
}
"""
else:
    payload += """
static int host_send_ping(uint8_t *data, bool force)
{
    if (!force && tdma_wait_for_ping_window() == TDMA_PING_DEFERRED) {
        return -EAGAIN;
    }
    return esb_write(data, false, ESB_PING_LEN);
}
"""
# Run the actual TX_SUCCESS case, not a stub for its clock release decision.
start = esb.index("\tcase ESB_EVENT_TX_SUCCESS:") + len("\tcase ESB_EVENT_TX_SUCCESS:")
end = esb.index("\n\t\tbreak;", start)
payload += """
static void host_tx_success(void)
{
""" + esb[start:end] + """
}
"""
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
    # Keep hardware leaves in the established fixture; exercise the new private
    # packet through the same extracted production ESB dispatch.
    event_payload = temporary / "test_event_payload.c"
    event_payload.write_text(
        '#define main radio_existing_main\n'
        + f'#include "{HERE / "test_payload.c"}"\n'
        + '#undef main\n#include "connection/tracker_event_protocol.h"\n'
        + r'''
int main(void) {
    int result = radio_existing_main();
    if (result || getenv("RADIO_SCENARIO")) return result;
    uint8_t packet[TRACKER_EVENT_ESB_LEN];
    struct tracker_event event = {
        .nonce = 1, .event_seq = 9, .tracker_id = 2,
        .kind = TRACKER_EVENT_KIND_TRACKER_REST,
        .event = CAL_EVENT_STATE, .phase = TRACKER_REST_REST,
    };
    assert(tracker_event_encode(packet, &event));
    reset();
    tdma_enabled = true;
    unsigned ping_index = ping_history_idx;
    assert(esb_write(packet, true, sizeof(packet)) == 0);
    assert(queued_count == 1 && queue_calls == 1);
    assert_original(0, packet, sizeof(packet), true);
    assert(ping_history_idx == ping_index);
    reset();
    tdma_enabled = true;
    deny_admission = true;
    assert(esb_write(packet, true, sizeof(packet)) == -EAGAIN);
    assert(queued_count == 0);
    deny_admission = false;
    reset();
    queue_failures = 1;
    assert(esb_write(packet, true, sizeof(packet)) != 0);
    assert(queue_calls == 1 && queued_count == 0);
    puts("payload: private event uses TDMA/NoACK, no PING accounting or FIFO retry");
    return 0;
}
''')
    for name in ("payload", "commands", "collection"):
        if os.environ.get("RADIO_CASE") not in (None, name):
            continue
        binary = temporary / name
        command = shlex.split(os.environ.get("CC", "cc")) + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter", "-g", "-O1",
            "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie",
            "-I", str(temporary), "-I", str(SRC), "-I", str(HERE / "../raw_collection"),
            str(event_payload if name == "payload" else HERE / f"test_{name}.c"), "-lm", "-o", str(binary),
        ]
        subprocess.run(command, check=True)
        subprocess.run([str(binary)], check=True)
