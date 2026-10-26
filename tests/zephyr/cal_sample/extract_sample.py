#!/usr/bin/env python3
"""Extract all production sample state/functions; kernel calls remain real."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "host/harness/python"))
from c_extract import extract_block

source_path = Path(sys.argv[1]).resolve()
root = source_path.parents[3]
source = source_path.read_text()
body = source[source.index("#define CAL_SAMPLE_QUEUE_CAPACITY"):]
# Permit running the same behavioral assertions against a saved pre-fix source.
# This adapter only names its existing reset operation; it adds no admission.
if "void sensor_calibration_samples_reset(void)" in body:
    body += """
void sensor_calibration_samples_begin(uint8_t channels)
{
    ARG_UNUSED(channels);
    sensor_calibration_samples_reset();
}
void sensor_calibration_samples_end(void)
{
    sensor_calibration_samples_reset();
}
"""
# Exercise the real destructive acquisition path through the real queue and
# the AutoGSens admission guard. Only the SSI bus is injected by the test.
def function(text, name):
    return extract_block(text, rf"^(?:static )?(?:void|int|bool|uint16_t) {name}\([^;]*?\)\s*\{{")


sensor = (root / "src/sensor/sensor.c").read_text()
sens = (root / "src/sensor/calibration/cal_sens.c").read_text()
util = (root / "src/util.c").read_text()
dsv = (root / "src/sensor/imu/LSM6DSV.c").read_text()
bias = (root / "src/sensor/calibration/bias_collect.c").read_text()
dso = (root / "src/sensor/imu/LSM6DSO.c").read_text()
body += "\n" + function(sensor, "sensor_gyro_report_discontinuity")
body += "\n" + function(util, "v_finite")
body += "\n" + extract_block(sens, r"^struct sensitivity_stream\s*\{", semicolon=True)
body += "\n" + function(sens, "sensitivity_wait_sample")
body += "\n" + function(bias, "sensor_offsetBias_internal")
body += "\n" + "\n".join(line for line in dsv.splitlines() if line.startswith((
    "#define PACKET_SIZE ", "#define LSM6DSV_FIFO_MODE_", "#define LSM6DSV_UNKNOWN_TAG_",
)))
body += "\n" + "\n".join(line for line in dso.splitlines() if line.startswith((
    "#define LSM6DSO_FIFO_",
)))
for text, names in (
    (dsv, ("lsm_fifo_resync", "lsm_fifo_read", "lsm_decode_accel", "lsm_decode_gyro", "lsm_fifo_process")),
    (dso, ("lsm6dso_fifo_read",)),
):
    for name in names:
        body += "\n" + function(text, name)
body += "\n"
Path(sys.argv[2]).write_text(body)
