#!/usr/bin/env python3
"""Exercise actual button feedback, OTA ownership and heated durability boundaries."""
import argparse
from pathlib import Path
import os
import re
import shlex
import subprocess
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'harness/python'))
from c_extract import extract_block

HERE = Path(__file__).resolve().parent
SRC = HERE.parents[2] / "src"
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--fixture", help="Run only the named C fixture (all its production variants)")
args = parser.parse_args()
system = (SRC / "system/system.c").read_text()
status = (SRC / "system/status.c").read_text()
sensor = (SRC / "sensor/sensor.c").read_text()
power = (SRC / "system/power.c").read_text()
main = (SRC / "main.c").read_text()
heated = (SRC / "sensor/calibration/tcal_heated.c").read_text()
calibration = (SRC / "sensor/calibration/calibration.h").read_text()
ota = (SRC / "system/esb_ota.c").read_text()
ota_header = (SRC / "system/esb_ota.h").read_text()
ota_flash_header = (SRC / "system/esb_ota_flash.h").read_text()


def function(source, name):
    return extract_block(source, rf"^[a-zA-Z_][^;\n]*\b{re.escape(name)}\([^;\n]*\)[^\n]*\n\{{")


def declaration(source, name):
    return extract_block(source, rf"^(?:enum|struct) {re.escape(name)} \{{", semicolon=True)


start = system.index("/* Deferred NVS slots for warm-durable IDs")
end = system.index("\nvoid sys_read(", start)
production = system[start:end]
production += "\n" + function(calibration, "sensor_calibration_result")
production += "\n" + function(heated, "heat_feedback_finish_locked")
production += "\n" + function(heated, "sensor_tcal_feedback_persisted")
ota_production = "\n".join(re.findall(r"^#define OTA_(?:STATUS_[A-Z_]+|BEGIN_PACKET_SIZE|BOARD_TARGET_MAX) +.*$", ota_header, re.MULTILINE))
ota_production += "\n" + re.search(r"^#define OTA_FLASH_PAGE_SIZE +.*$", ota_flash_header, re.MULTILINE).group()
ota_production += "\n" + declaration(ota, "ota_state") + "\n" + declaration(ota, "ota_context")
ota_production += "\nstatic struct ota_context ota;\nstatic atomic_t ota_reboot_pending;\n"
ota_production += "\n".join(re.findall(r"^static (?:struct led_token ota_feedback|uint32_t ota_feedback_revision|bool ota_feedback_terminal|enum led_semantic ota_feedback_state);", ota, re.MULTILINE))
for name in ("esb_ota_is_active", "ota_update_led", "ota_activate_impl", "esb_ota_handle_activate", "esb_ota_handle_abort"):
    ota_production += "\n" + function(ota, name)
begin = function(ota, "ota_begin_impl")
ota_production += "\n" + begin[:begin.index("\t/* Validate CRC-8 */")] + "\treturn 0;\n}"
ota_production += "\n" + function(ota, "esb_ota_handle_begin")
button_production = "\n".join(function(system, name) for name in (
    "button_interrupt_handler", "button_press_cancel", "button_status_clear_if_idle", "button_thread",
))
button_production = function(status, "status_ready") + "\n" + button_production
manual_production = "\n".join(function(power, name) for name in (
    "sys_power_state_request", "sys_request_system_off", "sys_request_system_reboot", "sys_user_reboot",
    "sys_exit_feedback_allowed", "sys_power_notice", "sys_system_off", "sys_system_reboot",
))
manual_production += "\n" + "\n".join(function(system, name) for name in (
    "button_release_consume", "button_press_cancel", "button_status_clear_if_idle", "sys_manual_exit_request", "sys_button_reboot",
    "sys_button_shutdown", "sys_user_shutdown", "sys_command_shutdown_request", "button_interrupt_handler", "button_thread",
))
status_fault_production = status[
    status.index("static atomic_t status_state;"):status.index("\nLOG_MODULE_REGISTER")
]
status_fault_production += "\n" + "\n".join(function(status, name) for name in (
    "status_update", "set_status", "set_sensor_fault", "set_sensor_fault_if_unset",
    "get_status", "status_publish_faults", "status_ready",
))
scan = function(sensor, "sensor_scan")
decision_end = scan.index("\n\tint mag_id = -1;")
decision_start = scan.rindex("\tif (imu_id >= 0) {", 0, decision_end)
sensor_fault_production = "static int host_sensor_scan_result(int imu_id) {\n"
sensor_fault_production += scan[decision_start:decision_end] + "\n\treturn 0;\n}\n"
sensor_fault_production += "static int host_sensor_scan_success(void) {\n"
sensor_fault_production += scan[scan.index("\tsensor_sensor_init = true;"):] + "\n"
loop = function(sensor, "sensor_loop")
error_start = loop.index("\tif (err) {", loop.index("\tint err = sensor_init();"))
error_end = loop.index("\t} else {", error_start)
sensor_fault_production += "static void host_sensor_outer_init_failure(int err) {\n"
sensor_fault_production += loop[error_start:error_end] + "\t}\n}\n"
sensor_fault_production += re.search(r"^#define NO_PACKETS_TIMEOUT_MS .*?$", sensor, re.MULTILINE).group() + "\n"
packet_start = sensor.index("\t// Check packet processing")
packet_end = sensor.index("\tsensor_int_during_loop = false;", packet_start) + len("\tsensor_int_during_loop = false;")
sensor_fault_production += "static void host_sensor_packet_check(const struct host_sensor_frame *frame) {\n"
sensor_fault_production += sensor[packet_start:packet_end] + "\n}\n"
with tempfile.TemporaryDirectory(prefix="tracker-led-business-") as directory:
    temporary = Path(directory)
    (temporary / "production.inc").write_text(production)
    (temporary / "ota_production.inc").write_text(ota_production)
    (temporary / "button_production.inc").write_text(button_production)
    (temporary / "manual_production.inc").write_text(manual_production)
    (temporary / "status_fault_production.inc").write_text(status_fault_production)
    (temporary / "sensor_fault_production.inc").write_text(sensor_fault_production)
    (temporary / "startup_production.inc").write_text(function(main, "main"))
    (temporary / "zephyr").mkdir()
    for header in ("kernel.h", "spinlock.h"):
        (temporary / "zephyr" / header).write_text("#pragma once\n")
    fixtures = (
        ("test_heated_persistence.c", ()),
        ("test_ota_identity.c", ()),
        ("test_sensor_fault.c", ()),
        ("test_button_ack.c", ("-DCONFIG_USER_EXTRA_ACTIONS=0",)),
        ("test_button_ack.c", ("-DCONFIG_USER_EXTRA_ACTIONS=1",)),
        ("test_manual_exit.c", ("-DCONFIG_USER_EXTRA_ACTIONS=0", "-DUSER_SHUTDOWN_ENABLED=1")),
        ("test_manual_exit.c", ("-DCONFIG_USER_EXTRA_ACTIONS=1", "-DUSER_SHUTDOWN_ENABLED=1")),
        ("test_manual_exit.c", ("-DCONFIG_USER_EXTRA_ACTIONS=1", "-DUSER_SHUTDOWN_ENABLED=0")),
    )
    if args.fixture and args.fixture not in {fixture for fixture, _ in fixtures}:
        parser.error(f"unknown fixture: {args.fixture}")
    for index, (fixture, defines) in enumerate(fixtures):
        if args.fixture and fixture != args.fixture:
            continue
        binary = temporary / f"{Path(fixture).stem}_{index}"
        subprocess.run(shlex.split(os.environ.get("CC", "cc")) + list(defines) + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function", "-Wno-unused-parameter",
            "-g", "-fsanitize=address,undefined", "-fno-pie", "-no-pie", f"-I{temporary}", f"-I{HERE.parent / 'led_sync'}",
            str(HERE / fixture), *(str(SRC / "system" / name) for name in ("led_policy.c", "led_behavior.c", "led_timeline.c")),
            "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
