"""Exercise production matrix validation and mag-clear, not stricter test doubles."""
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'harness/python'))
from c_extract import extract_block


def function(source, name):
    return extract_block(source, r"^(?:static\s+)?(?:bool|float|void|int)\s+" + re.escape(name) + r"\s*\([^;{}]*\)\s*\{")


def extract(root):
    util = (root / "src/util.c").read_text()
    magneto = (root / "src/sensor/calibration/magneto.c").read_text()
    calibration = (root / "src/sensor/calibration/calibration.c").read_text()
    return "\n\n".join([
        "#include <float.h>\n#ifndef MAX\n#define MAX(a, b) ((a) > (b) ? (a) : (b))\n#endif",
        *(function(util, name) for name in ("v_finite", "v_epsilon", "v_avg")),
        function(magneto, "mag_bainv_structurally_ok"),
        function(calibration, "calibration_clear_mag_owned"),
        function(calibration, "sensor_calibration_clear_mag"),
    ]) + "\n"


if __name__ == "__main__":
    output = Path(sys.argv[1])
    root = Path(sys.argv[2]) if len(sys.argv) > 2 else Path(__file__).resolve().parents[3]
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(extract(root))
