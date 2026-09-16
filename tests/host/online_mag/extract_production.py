"""Exercise production matrix validation and mag-clear, not stricter test doubles."""
import re
import sys
from pathlib import Path


def function(source, name):
    match = re.search(r"^(?:bool|float|void)\s+" + re.escape(name) + r"\s*\(", source, re.M)
    if match is None:
        raise ValueError(f"missing production function {name}")
    opening = source.index("{", match.start())
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


def extract(root):
    util = (root / "src/util.c").read_text()
    magneto = (root / "src/sensor/calibration/magneto.c").read_text()
    calibration = (root / "src/sensor/calibration/calibration.c").read_text()
    return "\n\n".join([
        "#include <float.h>\n#ifndef MAX\n#define MAX(a, b) ((a) > (b) ? (a) : (b))\n#endif",
        *(function(util, name) for name in ("v_finite", "v_epsilon", "v_avg")),
        function(magneto, "mag_bainv_structurally_ok"),
        function(calibration, "sensor_calibration_clear_mag"),
    ]) + "\n"


if __name__ == "__main__":
    output = Path(sys.argv[1])
    root = Path(sys.argv[2]) if len(sys.argv) > 2 else Path(__file__).resolve().parents[3]
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(extract(root))
