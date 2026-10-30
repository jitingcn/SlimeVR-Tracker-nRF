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
    return block(source, rf"^(?:static )?(?:bool|int|void|uint8_t|uint16_t|uint32_t) {re.escape(name)}\([^;]*?\)\n\{{")

esb = (SOURCE / "connection/esb.c").read_text()
connection = (SOURCE / "connection/connection.c").read_text()
pong = function(esb, "esb_receive_pong")
restart = block(pong, r"^\t+if \(receiver_clock_restarted\) \{")
snapshot = (SOURCE / "connection/sensor_data_snapshot.h").read_text()
snapshot_types = (
    block(snapshot, r"^enum sensor_stream \{", True) + "\n"
    + block(snapshot, r"^typedef struct \{(?=\n\tfloat q\[4\];)") + " sensor_data_capture_t;\n"
    + block(connection, r"^enum sub_packet_type \{", True)
)
header = (SRC / "connection/esb.h").read_text()
constants = "\n".join(re.findall(r"^#define (?:ESB_|PING_INTERVAL_MS)[^\n]*", header, re.MULTILINE))
wom_state = block(esb, r"^static struct \{(?=\n\tuint16_t minutes, token;)") + " wom_command;"
payload = constants + "\n" + "\n".join(re.findall(r"^static struct esb_payload tx_payload = .*;", esb, re.MULTILINE))
payload += "\n" + wom_state + "\n" + function(esb, "esb_wom_reset")
payload += "\n" + function(esb, "esb_wom_prepare_ping")
# Keep the whole production clock section so stop/start share its actual state
# and helper declarations; the fixture stubs only the kernel and onoff leaves.
clock_start = esb.index("bool clock_status = false;")
clock_control = re.search(r"^#if defined\(CONFIG_CLOCK_CONTROL_NRF\)$", esb[clock_start:], re.MULTILINE)
if not clock_control:
    raise ValueError("Clock control section not found")
control_start = clock_start + clock_control.start()
depth = 0
for directive in re.finditer(r"^[ \t]*#\s*(if|ifdef|ifndef|endif)\b[^\n]*", esb[control_start:], re.MULTILINE):
    depth += -1 if directive.group(1) == "endif" else 1
    if depth == 0:
        clock_end = control_start + directive.end()
        break
else:
    raise ValueError("Unclosed clock control section")
payload += "\n" + esb[clock_start:clock_end]
payload += "\n" + function(esb, "esb_queue_ping")
payload += "\n" + function(esb, "esb_prepare_ping")
clocked_writer = re.search(r"^static int esb_write_clocked\(", esb, re.MULTILINE)
if clocked_writer:
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
payload += "\nstatic void host_receiver_restart_body(unsigned *detections, unsigned *continued) {\n"
payload += "bool receiver_clock_restarted = true;\n#define receiver_restart_detections (*detections)\n"
payload += "uint32_t g_last_rx_raw_ticks = 123456, ping_rx_ticks = 1234;\n"
payload += restart + "\n*continued = 1;\n#undef receiver_restart_detections\n}\n"
payload += "static unsigned host_receiver_restart(void) {\n"
payload += "unsigned detections = 0, continued = 0;\nhost_receiver_restart_body(&detections, &continued);\n"
payload += "return detections + 256 * continued;\n}\n"

# Keep the actual TX cases and lifetime counters observable in the focused
# failure fixture. Only hardware leaves and the enclosing RX switch are omitted.
handler = function(esb, "event_handler")
start = handler.index("\n", handler.index("{")) + 1
switch = handler.index("\tswitch (event->evt_id)")
end = handler.index("\tcase ESB_EVENT_RX_RECEIVED:")
tx_failures = constants + "\n"
tx_failures += "\n".join(re.findall(r"^#define TX_ERROR_THRESHOLD[^\n]*", esb, re.MULTILINE))
tx_failures += "\n#ifndef HOST_PING_TIMEOUT_DEFINED\n" + function(esb, "esb_ping_timeout") + "\n#endif\n"
tx_failures += "\n" + handler[start:switch]
tx_failures += "\nstatic void host_tx_event(struct esb_evt const *event) {\n"
tx_failures += handler[switch:end] + "\n}\n}\n"
commands = [constants, block(esb, r"^struct esb_remote_cmd \{", True)]
commands.extend(re.findall(r"^static (?:bool remote_command_rejected|uint32_t remote_command_generation);", esb, re.MULTILINE))
commands.extend(re.findall(r"^static uint32_t (?:executing_shutdown_generation|shutdown_feedback_generation, shutdown_feedback_request|shutdown_accepted_event, shutdown_terminal_event);", esb, re.MULTILINE))
commands.append(wom_state)
commands.append(function(esb, "esb_remote_command_supported"))
for name in ("esb_wom_reset", "esb_wom_receive", "esb_wom_process",
             "esb_wom_prepare_ping", "esb_get_ping_ack_flag", "esb_get_ping_request_data"):
    commands.append(function(esb, name))
registry = block(esb, r"^static const struct esb_remote_cmd esb_remote_cmds\[\] = \{", True)
actual_handlers = ("esb_remote_cmd_shutdown", "esb_remote_cmd_data_collect_batch_on", "esb_remote_cmd_data_collect_batch_off", "esb_remote_cmd_tcal_heated_start", "esb_remote_cmd_set_channel", "esb_remote_cmd_ota_query_info", "esb_remote_cmd_ota_abort")
for name in actual_handlers:
    commands.append(function(esb, name))
for name in sorted(set(re.findall(r", (esb_remote_cmd_\w+)\}", registry)) - set(actual_handlers)):
    commands.append(f"static void {name}(void) {{ }}")
commands.append(function((SRC / "system/system.c").read_text(), "sys_command_shutdown"))
commands.insert(0, function((SRC / "system/system.c").read_text(), "sys_command_shutdown_request"))
commands += [registry, function(esb, "esb_remote_command_execute"),
             function(esb, "esb_ping_timeout"), function(esb, "esb_thread")]
# Exercise the production control-admission tail after PONG validation.
start = pong.index("\t\tif (pong_flags == ESB_PONG_FLAG_WOM_CONTROL)")
end = pong.rindex("\n\t}")
commands.append("static void receive_control(uint8_t pong_flags) {\n"
                + "const struct esb_payload *payload = &rx_payload;\n"
                + "float pong_sens_data[3] = {0}; uint8_t pong_sens_auto_axis = 0; uint16_t pong_sens_auto_revolutions = 0;\n"
                + pong[start:end] + "\n}")
commands.append("static void receive_stop(void) { receive_control(ESB_PONG_FLAG_DATA_COLLECT_BATCH_OFF); }")
commands.append(re.search(r"^static uint32_t g_last_rx_raw_ticks = [^;]+;", esb, re.MULTILINE).group(0))
restart_tick_start = pong.index("\t\tuint32_t ping_rx_ticks =")
restart_tick_end = pong.index(";\n", restart_tick_start) + 1
commands.append("static void host_receiver_restart_body(unsigned *detections, unsigned *continued) {\n"
                + "bool receiver_clock_restarted = true;\n#define receiver_restart_detections (*detections)\n"
                + "const struct esb_payload *payload = &rx_payload;\n"
                + pong[restart_tick_start:restart_tick_end] + "\n"
                + restart + "\n*continued = 1;\n#undef receiver_restart_detections\n}\n"
                + "static unsigned host_receiver_restart(void) {\n"
                + "unsigned detections = 0, continued = 0;\nhost_receiver_restart_body(&detections, &continued);\n"
                + "return detections + 256 * continued;\n}")
collection = []
for name in ("connection_raw_collection_active", "connection_feedback_maintenance_update", "connection_reset_raw_collection", "connection_set_data_collection", "connection_get_data_collection", "connection_set_data_collection_batch", "connection_get_data_collection_batch", "connection_get_data_collection_batch_rate", "connection_send_raw_metadata"):
    collection.append(function(connection, name))
collection.insert(3, function(connection, "connection_get_raw_collection_session"))
# Old API snapshots return void. Adapt only the test observation, not production
# control flow, so pre-fix comparison fails on behavior rather than compilation.
returns_status = re.search(r"^int connection_set_data_collection_batch\(", connection, re.MULTILINE)
collection.append("static int batch_request(bool enable, uint16_t rate) { " + ("return connection_set_data_collection_batch(enable, rate);" if returns_status else "connection_set_data_collection_batch(enable, rate); return 0;") + " }")

# Keep the real status bit values and getter: treating the connection-error
# mask as a bool in a fixture would hide a permanently blocked producer.
status = (SOURCE / "system/status.c").read_text()
start = pong.index("\t\tuint8_t rx_id = payload->data[1];")
end = pong.index("\n\t\tuint32_t ping_rx_ticks", start)
recovery = constants + "\n" + function(status, "get_status")
recovery += "\n" + function(esb, "esb_pong_is_legacy")
recovery += "\n" + "\n".join(re.findall(r"^#define PING_RECOVERY_THRESHOLD[^\n]*", esb, re.MULTILINE))
recovery += "\nstatic void receive_valid_pong(void) {\nconst struct esb_payload *payload = &rx_payload;\n" + pong[start:end] + "\n(void)rx_ctr;\n}\n"

channels = constants + "\n" + block(esb, r"^static const uint8_t __maybe_unused ESB_ALLOWED_CHANNELS\[\] = \{", True)
channels += "\n#define ESB_ALLOWED_CHANNELS_COUNT (sizeof(ESB_ALLOWED_CHANNELS))\n"
clock_users = re.search(r"^static atomic_t tx_clock_users;", esb, re.MULTILINE)
channels += "\n" + (clock_users.group() if clock_users else "static atomic_t tx_clock_users;") + "\n"
channels += esb[esb.index("static K_MUTEX_DEFINE(esb_radio_lock);"):esb.index("#define TX_ERROR_THRESHOLD")]
channels += "\n" + block(header, r"^static inline uint8_t esb_rf_channel_encode\([^;{]*\)\s*\{", False)
channels += "\n" + block(header, r"^static inline uint8_t esb_rf_channel_decode\([^;{]*\)\s*\{", False)
channels += "\n" + function(esb, "esb_pong_is_legacy")
channels += "\n" + wom_state + "\n" + function(esb, "esb_wom_reset")
channels += "\n" + function(esb, "esb_remote_command_supported")
channels += "\n" + function(esb, "esb_get_ping_ack_flag")
channels += "\n" + function(esb, "esb_get_ping_request_data")
channels += "\n#define HOST_PING_TIMEOUT_DEFINED 1\n" + function(esb, "esb_ping_timeout")
channels += "\nstatic int record_ping_payload(const struct esb_payload *p) { (void)p; assert(irq_depth); return 0; }\n"
channels += "#define esb_write_payload record_ping_payload\n" + function(esb, "esb_queue_ping")
channels += "\n#undef esb_write_payload\n"
channels += "\nstatic void record_ping_admission(uint8_t counter) {\n"
channels += "struct esb_payload tx_payload = { .length = ESB_PING_LEN, .data = {ESB_PING_TYPE, 0, counter} };\n"
channels += "tx_payload.data[7] = esb_get_ping_ack_flag();\n"
channels += "esb_get_ping_request_data(&tx_payload.data[8]);\n"
channels += "assert(esb_queue_ping(&tx_payload) == 0);\n}\n"
start = pong.index("\t\tuint32_t ping_ticks_for_this_ctr = 0;")
end = pong.index("uint32_t rtt_us = 0;", start)
channels += ("\nstatic bool host_ping_history(uint8_t rx_ctr) {\n" + pong[start:end]
             + "\n(void)ping_ticks_for_this_ctr; (void)ping_ticks_kernel_for_this_ctr;"
             + "\nreturn ping_history_valid;\n}\n")
channels += "\nstatic void maintenance_timeout(void) { int64_t now_idle = k_uptime_get();\n"
channels += "\tesb_ping_timeout(now_idle);\n}\n"
channels += "\n" + function(esb, "esb_deinitialize")
channels += "\n" + function(esb, "esb_channel_search_poll")
start = pong.index("\tif (payload->data[0] == ESB_PONG_TYPE)")
end = pong.index("\n\t\tuint32_t ping_rx_ticks", start)
channels += "\n#define PING_RECOVERY_THRESHOLD 1\nstatic bool pong_admitted;\nstatic void receive_pong_prefix(void) {\nconst struct esb_payload *payload = &rx_payload;\n" + pong[start:end] + "\n(void)rx_ctr;\npong_admitted = true;\n}\n}\n"
channels += "\nstatic bool accept_pong(void) { pong_admitted = false; if (rx_payload.length == ESB_PONG_LEN) receive_pong_prefix(); return pong_admitted; }\n"
start = pong.index("\t\t/* NORMAL retains the legacy slot/total layout.")
end = pong.index("\n\n\t\tif (pong_flags == ESB_PONG_FLAG_WOM_CONTROL)", start)
channels += "\nstatic void receive_schedule(uint8_t pong_flags) {\nconst struct esb_payload *payload = &rx_payload;\n" + pong[start:end] + "\n}\n"
channels += "\n" + function(esb, "esb_send_pair_step")
channels += "\n" + function(esb, "esb_pair")
channels += "\n" + function(esb, "esb_led_connection_facts")
channels += "\nstatic void host_receiver_restart_body(unsigned *detections, unsigned *continued) {\n"
channels += "bool receiver_clock_restarted = true;\n#define receiver_restart_detections (*detections)\n"
channels += "uint32_t g_last_rx_raw_ticks = 123456, ping_rx_ticks = 1234;\n"
channels += restart + "\n*continued = 1;\n#undef receiver_restart_detections\n}\n"
channels += "static unsigned host_receiver_restart(void) {\n"
channels += "unsigned detections = 0, continued = 0;\nhost_receiver_restart_body(&detections, &continued);\n"
channels += "return detections + 256 * continued;\n}\n"
start = esb.index("\t\tif (!paired_addr[0]) // zero, not paired")
end = esb.index("\n\t\t} else {\n\t\t\tswitch (rx_payload.length)", start)
channels += "\nstatic void receive_pair(void) { do {\n" + esb[start:end] + "\n}\n} while (0); }\n"

with tempfile.TemporaryDirectory(prefix="tracker-radio-sessions-") as directory:
    temporary = Path(directory)
    (temporary / "connection").mkdir()
    (temporary / "connection/connection.h").write_text((SOURCE / "connection/connection.h").read_text())
    (temporary / "snapshot_types.inc").write_text(snapshot_types)
    suppression_start = connection.index("static struct k_spinlock ota_suppress_lock;")
    suppression_end = connection.index("\n#if CONFIG_CONNECTION_RAW_COLLECTION", suppression_start)
    lifecycle = connection[suppression_start:suppression_end]
    lifecycle += "\n" + "\n".join(function(connection, name) for name in (
        "connection_set_ota_suppressed", "connection_get_ota_suppressed",
        "connection_ota_suppression_active", "connection_service_ping", "connection_thread"))
    suppression = connection[suppression_start:suppression_end]
    suppression += "\n" + "\n".join(function(connection, name) for name in (
        "connection_set_ota_suppressed", "connection_get_ota_suppressed",
        "connection_ota_suppression_active"))
    (temporary / "suppression.inc").write_text(suppression)
    queue_start = esb.index("#define OTA_RX_QUEUE_SIZE")
    queue_end = esb.index("\n// Server time synchronization", queue_start)
    ingress_start = esb.index("\n\t\t\tdefault:\n") + len("\n\t\t\tdefault:\n")
    ingress_end = esb.index("\n\t\t\t} // end of rx_payload length switch", ingress_start)
    ota_queue = constants + "\n" + esb[queue_start:queue_end]
    ota_queue += "\nstatic void receive_auxiliary(void) {\n" + esb[ingress_start:ingress_end] + "\n}\n"
    ota_queue += "\n" + function(esb, "esb_process_ota_rx_queue")
    (temporary / "ota_queue.inc").write_text(ota_queue)
    for name, parts in (("payload", payload), ("tx_failures", tx_failures), ("commands", "\n\n".join(commands)), ("collection", "\n\n".join(collection)), ("recovery", recovery), ("channels", channels), ("lifecycle", lifecycle)):
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
    for name in ("payload", "tx_failures", "commands", "collection", "recovery", "channels", "lifecycle"):
        if os.environ.get("RADIO_CASE") not in (None, name):
            continue
        binary = temporary / name
        command = shlex.split(os.environ.get("CC", "cc")) + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter", "-g", "-O1",
            "-DCONFIG_CONNECTION_RAW_COLLECTION=1", "-DCONFIG_SENSOR_USE_MAGNETOMETER=1",
            "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie",
            "-I", str(temporary), "-I", str(SRC), "-I", str(HERE / "../raw_collection"),
            str(event_payload if name == "payload" else HERE / f"test_{name}.c"), "-lm", "-o", str(binary),
        ]
        subprocess.run(command, check=True)
        subprocess.run([str(binary)], check=True)
        if name == "commands":
            subprocess.run(command + ["-DCONFIG_SENSOR_TCAL_HEATED=1"], check=True)
            subprocess.run([str(binary)], check=True)
    if os.environ.get("RADIO_CASE") in (None, "ota_sync"):
        for name in ("ota_suppression", "ota_queue"):
            binary = temporary / name
            command = shlex.split(os.environ.get("CC", "cc")) + [
                "-std=c11", "-Wall", "-Wextra", "-Werror", "-g", "-O1", "-pthread",
                "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie",
                "-I", str(temporary), "-I", str(HERE / "../concurrency"),
                str(HERE / f"test_{name}.c"), "-o", str(binary),
            ]
            subprocess.run(command, check=True)
            subprocess.run([str(binary)], check=True)
    if os.environ.get("RADIO_CASE") in (None, "capabilities"):
        actual = set(actual_handlers) | {
            "esb_remote_cmd_data_collect_on", "esb_remote_cmd_data_collect_off",
            "esb_remote_cmd_mag_clear", "esb_remote_cmd_mag_cal", "esb_remote_cmd_mag_on",
            "esb_remote_cmd_mag_off", "esb_remote_cmd_mag_auto_on", "esb_remote_cmd_mag_auto_off",
            "esb_remote_cmd_ping",
        }
        matrix_parts = list(commands)
        for name in sorted(actual - set(actual_handlers)):
            stub = f"static void {name}(void) {{ }}"
            guard = "CONFIG_SENSOR_USE_MAGNETOMETER" if "_mag_" in name else "CONFIG_CONNECTION_RAW_COLLECTION"
            body = function(esb, name)
            if name != "esb_remote_cmd_ping":
                body = f"#if {guard}\n{body}\n#endif"
            matrix_parts[matrix_parts.index(stub)] = body
        # The raw handlers already present in commands need the same outer
        # compilation boundary as their source translation unit.
        for name in ("esb_remote_cmd_data_collect_batch_on", "esb_remote_cmd_data_collect_batch_off"):
            body = function(esb, name)
            matrix_parts[matrix_parts.index(body)] = f"#if CONFIG_CONNECTION_RAW_COLLECTION\n{body}\n#endif"
        ingress_start = esb.index("\n\t\t\tdefault:\n") + len("\n\t\t\tdefault:\n")
        ota_end = esb.index("\n\t\t\t} // end of rx_payload length switch", ingress_start)
        matrix_parts.append("static void receive_auxiliary(void) {\n" + esb[ingress_start:ota_end] + "\n}")
        (temporary / "commands.inc").write_text("\n\n".join(matrix_parts))
        for raw_enabled in (0, 1):
            for mag_enabled in (0, 1):
                binary = temporary / f"capabilities-{raw_enabled}-{mag_enabled}"
                command = shlex.split(os.environ.get("CC", "cc")) + [
                    "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
                    "-Wno-unused-function", "-Wno-unused-variable", "-g", "-O1",
                    "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie",
                    f"-DCONFIG_CONNECTION_RAW_COLLECTION={raw_enabled}",
                    f"-DCONFIG_SENSOR_USE_MAGNETOMETER={mag_enabled}", "-DHOST_CAPABILITY_MATRIX=1",
                    "-I", str(temporary), "-I", str(SRC), "-I", str(HERE / "../raw_collection"),
                    str(HERE / "test_capabilities.c"), "-lm", "-o", str(binary),
                ]
                subprocess.run(command, check=True)
                subprocess.run([str(binary)], check=True)
