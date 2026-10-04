#!/usr/bin/env python3
"""Exercise the live power-loop battery fragment and filter, with hardware leaves stubbed."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'harness/python'))
from c_extract import extract_block

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2]))
SRC = ROOT / "src/system"
power = (SRC / "power.c").read_text()
filter_source = (SRC / "power_battery.c").read_text()
battery = (SRC / "battery.c").read_text()
system = (SRC / "system.c").read_text()

def function(source, name):
    return extract_block(source, rf"^(?:static )?(?:int|bool) {re.escape(name)}\([^;\n]*\)\s*\{{")

feedback = "\n".join([
    function(battery, "battery_charger_snapshot"),
    function(system, "chg_read"),
    function(system, "stby_read"),
    function(system, "sys_charger_snapshot"),
    function(system, "sys_gpio_init"),
])
feedback += "\n#if CONFIG_SENSOR_TCAL_HEATED\n" + function(power, "heater_external_power_present") + "\n#endif"
# Live P00/P10 declarations supply capability, presence, polarity and pins.
# Regressions drive physical levels through the real GPIO snapshot/owner code.
def board_gpio(name):
    dts = (ROOT / f"boards/kemopati/sk_cheesecake_nrf_{name}/sk_cheesecake_nrf_{name}_nrf52840_uf2.dts").read_text()
    user = re.search(r"zephyr,user\s*\{(.*?)\n\s*\};", dts, re.DOTALL).group(1)
    macros = []
    for prop in ("chg", "stby", "plug"):
        gpio = re.search(rf"{prop}-gpios\s*=\s*<&gpio(\d+)\s+(\d+)\s+([^>]+)>;", user)
        macros.append(f"#define {prop.upper()}_EXISTS {int(gpio is not None)}")
        if gpio:
            macros += [
                f"#define TEST_{prop.upper()}_PORT {gpio.group(1)}",
                f"#define TEST_{prop.upper()}_PIN {gpio.group(2)}",
                f"#define TEST_{prop.upper()}_FLAGS {gpio.group(3)}",
            ]
    full_on_plug = bool(re.search(r"\bcharger-full-on-plug\s*;", user))
    macros.append(f"#define TEST_DT_HAS_charger_full_on_plug ({int(full_on_plug)} && !TEST_CHG_ONLY)")
    return "\n".join(macros) + "\n"
pmic_state_start = battery.index("static bool battery_ok;")
pmic_state_end = battery.index("static int battery_setup()", pmic_state_start)
pmic_source = battery[pmic_state_start:pmic_state_end] + "\n" + "\n".join(
    function(battery, name) for name in ("battery_sample", "battery_charger_state", "battery_charger_snapshot")
)
# Keep the filter's real state, functions, and configuration branches.
filter_source = re.sub(r'^#include[^\n]*\n', '', filter_source, flags=re.MULTILINE)
# Thread-lifetime locals live at translation-unit scope in this single-thread harness.
start = power.index('\tint battery_mV =')
end = power.index('\n#if CONFIG_SENSOR_TCAL_HEATED', start)
locals_source = power[start:end]
start = power.index('\t\tbool docked = dock_read();')
end = power.index('\n\t}', power.index('(void)k_sem_take(&power_wake_sem, K_MSEC(100));', start))
iteration = 'static void power_iteration(void) {\n' + power[start:end] + '\n}'

with tempfile.TemporaryDirectory(prefix='tracker-battery-sampling-') as directory:
    temporary = Path(directory)
    (temporary / 'production.inc').write_text(filter_source + '\n' + feedback + '\n' + locals_source + '\n' + iteration)
    for name, pmic, board, heater, chg_only in (
        ("gpio", 0, None, 0, 0),
        ("pmic", 1, None, 0, 0),
        ("p00", 0, "p00", 1, 0),
        ("p00-noheater", 0, "p00", 0, 0),
        ("p10", 0, "p10", 1, 0),
        ("p10-noheater", 0, "p10", 0, 0),
        ("chg-only", 0, "p10", 1, 1),
    ):
        if board:
            (temporary / 'board_gpio.inc').write_text(board_gpio(board))
        binary = temporary / f'battery-sampling-{name}'
        subprocess.run(shlex.split(os.environ.get('CC', 'cc')) + [
            '-std=gnu11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter',
            '-g', '-O1', '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
            '-fno-pie', '-no-pie', f'-DTEST_PMIC={pmic}', f'-DTEST_BOARD_GPIO={int(board is not None)}',
            f'-DTEST_HEATER={heater}', f'-DTEST_CHG_ONLY={chg_only}', f'-I{temporary}',
            str(HERE / 'test.c'), '-o', str(binary)], check=True)
        scenarios = ['cadence', 'filter', 'warmup', 'outlier', 'failure', 'dock', 'low', 'debounce',
                     'feedback_unknown', 'feedback_charge', 'feedback_low']
        if not pmic:
            scenarios += ['edges', 'settle', 'feedback_partial']
        if board:
            scenarios = ['feedback_chg_only'] if chg_only else [
                'feedback_board', 'feedback_board_errors', 'feedback_board_initfail',
            ]
        for scenario in scenarios:
            subprocess.run([str(binary), scenario], check=True)
    (temporary / 'pmic-production.inc').write_text(pmic_source)
    binary = temporary / 'pmic-snapshot'
    subprocess.run(shlex.split(os.environ.get('CC', 'cc')) + [
        '-std=gnu11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter',
        '-g', '-O1', '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
        '-fno-pie', '-no-pie', f'-I{temporary}',
        str(HERE / 'test_pmic_snapshot.c'), '-o', str(binary)], check=True)
    for scenario in ('charging', 'full', 'unknown', 'readfail', 'freshness'):
        subprocess.run([str(binary), scenario], check=True)
