#!/usr/bin/env python3
"""Exercise the real explicit heater-button owner and canonical admission."""
from pathlib import Path
import subprocess
import tempfile

from run import ROOT, HERE, build_harness


def main():
    source = (ROOT / "src/system/system.c").read_text()
    begin = source.index("#if CONFIG_SENSOR_TCAL_HEATED && DT_NODE_HAS_PROP(DT_ALIAS(heater_button), gpios)")
    end = source.index("#if BUTTON_EXISTS", begin)
    fixture = (HERE / "test_heated_button.c").read_text()
    with tempfile.TemporaryDirectory(prefix="heated-button-") as directory:
        for mode in ("p00", "p10", "sw1-only", "heater-only", "disabled"):
            main_source = fixture.replace("/* PRODUCTION_BUTTON */", source[begin:end])
            main_source = (
                f"#define TEST_SW1 {int(mode in ('p00', 'sw1-only', 'disabled'))}\n"
                f"#define TEST_HEATER_ALIAS {int(mode in ('p00', 'heater-only', 'disabled'))}\n"
                f"#define TEST_HEAT {int(mode != 'disabled')}\n"
            ) + main_source
            executable = build_harness(Path(directory) / mode, main_source=main_source)
            subprocess.run([str(executable)], check=True)
            print(f"{mode}: actual button/admission checks passed", flush=True)


if __name__ == "__main__":
    main()
