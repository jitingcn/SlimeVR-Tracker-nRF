#!/usr/bin/env python3
"""Exercise current radio/session functions with scheduler and hardware leaves stubbed."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'harness/python'))
from c_extract import extract_block as block

HERE = Path(__file__).resolve().parent
SRC = HERE.parents[2] / "src"
SOURCE = Path(os.environ.get("RADIO_SOURCE_ROOT", SRC))


def function(source, name):
    return block(source, rf"^(?:static )?(?:bool|int|void|uint8_t|uint16_t) {re.escape(name)}\([^;]*?\)\n\{{")

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
commands.extend(re.findall(r"^static uint32_t (?:executing_shutdown_generation|shutdown_feedback_generation, shutdown_feedback_request|shutdown_accepted_event, shutdown_terminal_event);", esb, re.MULTILINE))
registry = block(esb, r"^static const struct esb_remote_cmd esb_remote_cmds\[\] = \{", True)
actual_handlers = ("esb_remote_cmd_shutdown", "esb_remote_cmd_data_collect_batch_on", "esb_remote_cmd_data_collect_batch_off", "esb_remote_cmd_tcal_heated_start", "esb_remote_cmd_set_channel")
for name in actual_handlers:
    commands.append(function(esb, name))
for name in sorted(set(re.findall(r", (esb_remote_cmd_\w+)\}", registry)) - set(actual_handlers)):
    commands.append(f"static void {name}(void) {{ }}")
commands.append(function((SRC / "system/system.c").read_text(), "sys_command_shutdown"))
commands.insert(0, function((SRC / "system/system.c").read_text(), "sys_command_shutdown_request"))
commands += [registry, function(esb, "esb_remote_command_execute"), function(esb, "esb_thread")]
# Exercise the production control-admission tail after PONG validation.
start = esb.index("\t\t\t\t\tif (pong_flags == ESB_PONG_FLAG_DATA_COLLECT_METADATA)")
end = esb.index("\n\t\t\t\t}\n\t\t\t} break;", start)
commands.append("static void receive_control(uint8_t pong_flags) {\n"
                + "float pong_sens_data[3] = {0}; uint8_t pong_sens_auto_axis = 0; uint16_t pong_sens_auto_revolutions = 0;\n"
                + esb[start:end] + "\n}")
commands.append("static void receive_stop(void) { receive_control(ESB_PONG_FLAG_DATA_COLLECT_BATCH_OFF); }")
collection = []
for name in ("connection_raw_collection_active", "connection_feedback_maintenance_update", "connection_reset_raw_collection", "connection_set_data_collection", "connection_get_data_collection", "connection_set_data_collection_batch", "connection_get_data_collection_batch", "connection_get_data_collection_batch_rate", "connection_send_raw_metadata"):
    collection.append(function(connection, name))
# Old API snapshots return void. Adapt only the test observation, not production
# control flow, so pre-fix comparison fails on behavior rather than compilation.
returns_status = re.search(r"^int connection_set_data_collection_batch\(", connection, re.MULTILINE)
collection.append("static int batch_request(bool enable, uint16_t rate) { " + ("return connection_set_data_collection_batch(enable, rate);" if returns_status else "connection_set_data_collection_batch(enable, rate); return 0;") + " }")

# Keep the real status bit values and getter: treating the connection-error
# mask as a bool in a fixture would hide a permanently blocked producer.
status = (SOURCE / "system/status.c").read_text()
start = esb.index("\t\t\t\t\tuint8_t rx_id = rx_payload.data[1];")
end = esb.index("\n\t\t\t\t\tbool match_ctr", start)
recovery = constants + "\n" + function(status, "get_status")
recovery += "\n" + "\n".join(re.findall(r"^#define PING_RECOVERY_THRESHOLD[^\n]*", esb, re.MULTILINE))
recovery += "\nstatic void receive_valid_pong(void) { do {\n" + esb[start:end] + "\n(void)counter_diff;\n} while (0); }\n"

channels = constants + "\n" + block(esb, r"^static const uint8_t __maybe_unused ESB_ALLOWED_CHANNELS\[\] = \{", True)
channels += "\n#define ESB_ALLOWED_CHANNELS_COUNT (sizeof(ESB_ALLOWED_CHANNELS))\n"
channels += esb[esb.index("K_MUTEX_DEFINE(esb_radio_lock);"):esb.index("#define TX_ERROR_THRESHOLD")]
channels += "\n" + block(header, r"^static inline uint8_t esb_rf_channel_encode\([^;{]*\)\s*\{", False)
channels += "\n" + function(esb, "esb_channel_search_poll")
channels += "\n" + function(esb, "esb_deinitialize")
start = esb.index("\t\t\t\tif (rx_payload.data[0] == ESB_PONG_TYPE)")
end = esb.index("\n\t\t\t\t\tif (rx_id != tracker_id)", start)
channels += "\nstatic bool accept_pong(void) { if (rx_payload.length != ESB_PONG_LEN) return false; do {\n" + esb[start:end] + "\nreturn true;\n}\n} while (0); return false; }\n"
start = esb.index("\t\t\t\t\tif (pong_flags == ESB_PONG_FLAG_NORMAL)")
end = esb.index("\n\n\t\t\t\t\tif (pong_flags == ESB_PONG_FLAG_DATA_COLLECT_METADATA)", start)
channels += "\nstatic void receive_schedule(uint8_t pong_flags) {\n" + esb[start:end] + "\n}\n"
channels += "\n" + function(esb, "esb_send_pair_step")
channels += "\n" + function(esb, "esb_pair")
channels += "\n" + function(esb, "esb_led_connection_facts")
start = esb.index("\t\tif (!paired_addr[0]) // zero, not paired")
end = esb.index("\n\t\t} else {\n\t\t\tswitch (rx_payload.length)", start)
channels += "\nstatic void receive_pair(void) { do {\n" + esb[start:end] + "\n}\n} while (0); }\n"

with tempfile.TemporaryDirectory(prefix="tracker-radio-sessions-") as directory:
    temporary = Path(directory)
    (temporary / "connection").mkdir()
    (temporary / "connection/connection.h").write_text((SOURCE / "connection/connection.h").read_text())
    for name, parts in (("payload", payload), ("commands", "\n\n".join(commands)), ("collection", "\n\n".join(collection)), ("recovery", recovery), ("channels", channels), ("lifecycle", function(connection, "connection_thread"))):
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
    for name in ("payload", "commands", "collection", "recovery", "channels", "lifecycle"):
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
        if name == "commands":
            subprocess.run(command + ["-DCONFIG_SENSOR_TCAL_HEATED=1"], check=True)
            subprocess.run([str(binary)], check=True)
