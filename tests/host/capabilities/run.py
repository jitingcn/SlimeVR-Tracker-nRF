#!/usr/bin/env python3
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


def main():
    with tempfile.TemporaryDirectory(prefix="tracker-capabilities-") as directory:
        temporary = Path(directory)
        (temporary / "raw.inc").write_text(raw_source())
        binary = temporary / "raw"
        command = shlex.split(os.environ.get("CC", "cc")) + [
            "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function",
            "-Wno-unused-variable", "-Wno-unused-parameter", "-g", "-O1",
            "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie",
            "-DCONFIG_CONNECTION_RAW_COLLECTION=1", "-DCONFIG_SENSOR_USE_MAGNETOMETER=1",
            "-I", str(temporary), "-I", str(SRC), "-I", str(HERE.parent / "raw_collection"),
            str(HERE / "test_raw.c"), "-lm", "-o", str(binary),
        ]
        subprocess.run(command, check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
