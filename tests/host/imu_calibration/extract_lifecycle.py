"""Compile actual caller bodies with hardware leaves stubbed by the host tests."""
from pathlib import Path
import re
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'harness/python'))
from c_extract import extract_block

root = Path(__file__).resolve().parents[3]


def function(path, name):
    source = (root / path).read_text()
    return extract_block(source, rf"^(?:static )?(?:void|int) {re.escape(name)}\([^;]*?\)\s*\{{") + "\n"


out = Path(sys.argv[1])
out.mkdir(parents=True, exist_ok=True)
(out / "clear_resume.inc").write_text(
    function("src/system/system.c", "sys_clear")
    + function("src/sensor/sensor.c", "main_imu_resume")
)
(out / "button.inc").write_text(
    function("src/system/system.c", "sys_user_shutdown")
    + function("src/system/system.c", "button_thread")
)
(out / "imu_worker.inc").write_text(
    function("src/sensor/calibration/cal_imu.c", "imu_step")
    + function("src/sensor/calibration/cal_imu.c", "imu_failed")
    + function("src/sensor/calibration/cal_imu.c", "sensor_calibrate_imu")
)
