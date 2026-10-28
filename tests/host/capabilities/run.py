#!/usr/bin/env python3
"""Exercise raw wire bytes, retained restore and the shared accelerometer solver."""
import importlib.util
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2]))
SRC = ROOT / "src"
sys.path.insert(0, str(HERE.parent / "harness/python"))
from c_extract import extract_block


def function(source, name):
    return extract_block(source, rf"^(?:static )?(?:inline )?(?:bool|int|void|float|uint8_t|uint16_t) {re.escape(name)}\([^;{{]*\)\s*(?://[^\n]*)?\s*\{{")


def load_pose_fixture():
    spec = importlib.util.spec_from_file_location("mag_events", HERE.parent / "tracker_events/test_mag.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def raw_source():
    connection = (SRC / "connection/connection.c").read_text()
    sensor = (SRC / "sensor/sensor.c").read_text()
    producer = (SRC / "sensor/raw_collection.c").read_text()
    start = connection.index("#define RAW_IMU_QUEUE_SIZE ")
    end = connection.index("\n#endif\n\nvoid connection_feedback_maintenance_update", start)
    header = (SRC / "connection/esb.h").read_text()
    constants = "\n".join(re.findall(r"^#define (?:ESB_|RAW_CAL_)[^\n]+", header, re.MULTILINE))
    parts = [constants, connection[start:end]]
    names = (
        "connection_feedback_maintenance_update", "connection_reset_raw_collection",
        "connection_set_data_collection", "connection_get_data_collection",
        "connection_set_data_collection_batch", "connection_get_data_collection_batch",
        "connection_get_data_collection_batch_rate", "connection_raw_collection_startup_done",
        "connection_queue_raw_sample",
    )
    parts.extend(function(connection, name) for name in names)
    parts.append("#if CONFIG_SENSOR_USE_MAGNETOMETER\n" + function(connection, "connection_queue_raw_mag") + "\n#endif")
    parts.extend(function(connection, name) for name in (
        "connection_send_raw_metadata", "connection_send_calibration_mask",
        "connection_process_calibration_requests", "connection_process_raw_data"))
    parts.append(extract_block(producer, r"^struct raw_rate_plan \{", semicolon=True))
    parts.append("static struct raw_rate_plan raw_rate_plan;\nstatic atomic_t batch_collect_active, batch_collect_rate_hz;\nstatic float gyro_actual_time;")
    parts.extend(function(producer, name) for name in ("raw_rate_plan_refresh", "sensor_raw_collection_send_metadata"))
    parts.append(function(sensor, "sensor_send_raw_metadata"))
    return "\n\n".join(parts)


def restore_source():
    system = (SRC / "system/system.c").read_text()
    sensor = (SRC / "sensor/sensor.c").read_text()
    calibration = (SRC / "sensor/calibration/calibration.c").read_text()
    header = (SRC / "system/system.h").read_text()
    constants = "\n".join(re.findall(r"^#define (?:[A-Z0-9_]+_ID|MAG_ONLINE_CALIBRATION_[A-Z]+)\s+[^\n]+", header, re.MULTILINE))
    parts = [constants, function(system, "sys_retained_init"),
             function(calibration, "sensor_calibration_sensitivity_valid"),
             function(calibration, "sensor_calibration_read")]
    parts.extend(function(sensor, name) for name in (
        "sensor_scan_read", "sensor_scan_write", "sensor_scan_clear",
        "sensor_get_mag_enabled", "sensor_get_mag_available", "sensor_get_mag_calibrated",
        "sensor_refresh_sensor_ids", "sensor_get_mag_odr"))
    connection = (SRC / "connection/connection.c").read_text()
    parts.extend(function(connection, name) for name in (
        "connection_get_data_collection", "connection_get_data_collection_batch",
        "connection_get_data_collection_batch_rate"))
    util = (SRC / "util.c").read_text()
    magneto = (SRC / "sensor/calibration/magneto.c").read_text()
    parts[1:1] = [function(util, "v_finite"), function(util, "v_avg"), function(util, "v_epsilon"),
                  function(magneto, "mag_bainv_structurally_ok")]
    build = (SRC / "build_defines.h").read_text()
    parts.append("\n".join(re.findall(r"^#define SVR_MAG_STATUS_[^\n]+", build, re.MULTILINE)))
    parts.append(function(build, "get_server_constant_mag_id"))
    return "\n\n".join(parts)


def pose_source():
    fixture = load_pose_fixture()
    mag = (SRC / "sensor/calibration/cal_mag.c").read_text()
    bias = (SRC / "sensor/calibration/bias_collect.c").read_text()
    util = (SRC / "util.c").read_text()
    prelude = fixture.PRELUDE
    for name in ("magneto_sample", "magneto_current_calibration"):
        prelude = prelude.replace(function(prelude, name), "")
    sampler = r'''
static int sensor_wait_accel(float out[3], int timeout) {
    (void)timeout;
    float z = 1.0f - 2.0f * ((float)pose_index + 0.5f) / 18.0f;
    float radius = sqrtf(1.0f - z*z);
    float angle = 2.3999632297f * (float)pose_index;
    out[0] = radius*cosf(angle); out[1] = radius*sinf(angle); out[2] = z;
    now += 2; reads++;
    return 0;
}
'''
    prelude = prelude.replace(function(prelude, "sensor_wait_accel"), sampler)
    original_step = function(prelude, "cal_event_step")
    prelude = prelude.replace(original_step, original_step.replace(
        "pending_phase = p;", "pending_phase = p; if (p == CAL_PHASE_POSE_DONE) pose_index = detail;"))
    prelude = "static unsigned pose_index;\n" + prelude
    state = mag[mag.index("uint8_t magneto_progress;"):mag.index("static void magneto_update_dir_range")]
    constants = "\n".join(line for line in mag.splitlines() if line.startswith((
        "#define CALIB_", "#define MIN_ORIENTATION_", "#define THRESHOLD_ACC", "#define SAMPLES_PER_")))
    return "\n\n".join((
        prelude, '#include "sensor/magneto/magneto1_4.h"',
        "#if CONFIG_SENSOR_USE_MAGNETOMETER\n" + state + "\n#endif",
        constants, "typedef struct { float x,y,z; } Vector3;",
        function(util, "v_finite"), function(util, "v_diff_mag"), function(util, "v_epsilon"),
        function(mag, "magneto_reset"), function(bias, "isAccRest"),
        function(mag, "sensor_calibration_collect_accel_poses"),
    ))


def main():
    with tempfile.TemporaryDirectory(prefix="tracker-capabilities-") as directory:
        temporary = Path(directory)
        (temporary / "raw.inc").write_text(raw_source())
        (temporary / "restore.inc").write_text(restore_source())
        (temporary / "poses.inc").write_text(pose_source())
        cc = shlex.split(os.environ.get("CC", "cc"))
        common = ["-std=gnu11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function",
                  "-Wno-unused-variable", "-Wno-unused-parameter", "-g", "-O1",
                  "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie",
                  "-I", str(temporary), "-I", str(SRC), "-I", str(HERE.parent / "raw_collection")]
        for raw in (0, 1):
            for mag in (0, 1):
                flags = [f"-DCONFIG_CONNECTION_RAW_COLLECTION={raw}", f"-DCONFIG_SENSOR_USE_MAGNETOMETER={mag}",
                         "-DCONFIG_SENSOR_USE_ACCEL_CALIBRATION=1"]
                for name in ("restore", "poses") + (("raw",) if raw else ()):
                    binary = temporary / f"{name}-{raw}-{mag}"
                    sources = [str(HERE / f"test_{name}.c")]
                    if name == "poses":
                        sources = ["-I", str(HERE.parent / "magneto")] + sources
                        sources += [str(SRC / "sensor/magneto/magneto1_4.c"), str(SRC / "sensor/magneto/mymathlib_matrix.c")]
                    subprocess.run(cc + common + flags + sources + ["-lm", "-o", str(binary)], check=True)
                    subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
