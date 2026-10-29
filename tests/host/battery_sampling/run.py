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
power_request = (SRC / "power_request.h").read_text()
connection = (ROOT / "src/connection/connection.c").read_text()
connection_source = extract_block(connection, r"^void connection_update_battery\([^;{]*\)\s*\{")

def function(source, name):
    return extract_block(source, rf"^(?:static )?(?:int|bool|void) {re.escape(name)}\([^;\n]*\)\s*\{{")

feedback = "\n".join([
    function(power, "vbus_read"),
    function(power, "vin_read"),
    "#if USE_PMIC_CHARGER\n" + function(battery, "battery_sample") + "\n"
    + function(battery, "battery_charger_state") + "\n#endif",
    function(battery, "battery_charger_snapshot"),
    function(system, "chg_read"),
    function(system, "stby_read"),
    "#if CHARGER_STAT_EXISTS\n" + "\n".join(function(system, name) for name in (
        "charger_stat_verify_input", "charger_stat_configure_input", "charger_stat_read",
        "charger_stat_disable_sense", "charger_stat_snapshot_locked",
    )) + "\n#endif",
    function(system, "sys_charger_snapshot"),
    function(system, "sys_gpio_init"),
    function(system, "configure_sense_pins"),
    function(system, "sys_enter_poweroff"),
])
feedback += "\n#if CONFIG_SENSOR_TCAL_HEATED\n" + function(power, "heater_external_power_present") + "\n#endif"
# Live board declarations supply capability, presence, polarity and pins.
# Regressions drive physical levels through the real GPIO snapshot/owner code.
def board_gpio(name):
    if name == "generic-stat":
        # Independent non-USB STAT fixture: no board-specific source policy.
        user = "stat-gpios = <&gpio0 3 GPIO_ACTIVE_HIGH>;"
    else:
        path = "boards/nekumori/kawazu/kawazu_uf2.dts" if name == "kawazu" else (
            f"boards/kemopati/sk_cheesecake_nrf_{name}/sk_cheesecake_nrf_{name}_nrf52840_uf2.dts")
        dts = (ROOT / path).read_text()
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
    stat = re.search(r"stat-gpios\s*=\s*<&gpio(\d+)\s+(\d+)\s+([^>]+)>;", user)
    macros.append(f"#define CHARGER_STAT_EXISTS {int(stat is not None)}")
    if stat:
        macros += [
            f"#define TEST_STAT_PORT {stat.group(1)}",
            f"#define TEST_STAT_PIN {stat.group(2)}",
            f"#define TEST_STAT_FLAGS {stat.group(3)}",
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
start = power.index('\t\tpower_service_requests();')
end = power.index('\n\t}', power.index('(void)k_sem_take(&power_wake_sem, K_MSEC(100));', start))
private_off = "\n".join(
    extract_block(power_request, rf"^(?:enum|struct) {name} \{{", semicolon=True)
    for name in ("sys_power_request", "power_request_state", "power_ota_reboot_state", "power_request_mailbox")
)
private_off += "\nstatic struct power_request_mailbox power_requests;\n"
for name in ("power_request_begin", "power_request_finish", "power_request_finish_private"):
    private_off += "\n" + extract_block(
        power_request, rf"^static inline (?:enum sys_power_request|void) {name}\([^;{{]*\)\s*\{{")
private_off += "\n" + function(power, "sys_battery_dock_system_off")
private_off += "\n" + function(power, "power_service_requests")
iteration = 'static void power_iteration(void) {\n' + power[start:end] + '\n}'
main_source = (ROOT / "src/main.c").read_text()
boot_start = main_source.index("\n#if USER_SHUTDOWN_ENABLED\n")
boot_end = main_source.index("\n\tif (!booting_from_shutdown)", boot_start)
boot_policy = "static void boot_shutdown_check(void) {\n" + main_source[boot_start:boot_end] + "\n}"

with tempfile.TemporaryDirectory(prefix='tracker-battery-sampling-') as directory:
    temporary = Path(directory)
    (temporary / 'production.inc').write_text(
        filter_source + '\n' + feedback + '\n' + connection_source + '\n'
        + locals_source + '\n' + private_off + '\n' + iteration + '\n' + boot_policy)
    for name, pmic, board, heater, chg_only, no_gpio in (
        ("gpio", 0, None, 0, 0, 0),
        ("pmic", 1, None, 0, 0, 0),
        ("no-gpio", 0, None, 0, 0, 1),
        ("kawazu", 0, "kawazu", 0, 0, 0),
        ("generic-stat", 0, "generic-stat", 0, 0, 0),
        ("p00", 0, "p00", 1, 0, 0),
        ("p00-noheater", 0, "p00", 0, 0, 0),
        ("p10", 0, "p10", 1, 0, 0),
        ("p10-noheater", 0, "p10", 0, 0, 0),
        ("chg-only", 0, "p10", 1, 1, 0),
    ):
        if board:
            (temporary / 'board_gpio.inc').write_text(board_gpio(board))
        binary = temporary / f'battery-sampling-{name}'
        compile_command = shlex.split(os.environ.get('CC', 'cc')) + [
            '-std=gnu11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter',
            '-Wno-unused-function',  # Capability variants omit some hardware leaves.
            '-g', '-O1', '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
            '-fno-pie', '-no-pie',
            '-DCONFIG_CONNECTION_RAW_COLLECTION=1', '-DCONFIG_SENSOR_USE_MAGNETOMETER=1',
            f'-DTEST_PMIC={pmic}', f'-DTEST_BOARD_GPIO={int(board is not None)}',
            f'-DTEST_HEATER={heater}', f'-DTEST_CHG_ONLY={chg_only}',
            f'-DTEST_NO_GPIO={no_gpio}', f'-I{temporary}',
            str(HERE / 'test.c'), '-o', str(binary)]
        subprocess.run(compile_command, check=True)
        scenarios = ['cadence', 'filter', 'warmup', 'outlier', 'failure', 'dock', 'low', 'debounce',
                     'feedback_unknown', 'feedback_charge', 'feedback_low']
        scenarios += ['feedback_pulses', 'feedback_presence', 'poweroff']
        if pmic:
            scenarios += ['feedback_pmic_freshness']
        if not pmic:
            scenarios += ['edges', 'settle', 'feedback_partial', 'feedback_partial_charge',
                          'feedback_partial_boot_chg', 'feedback_partial_boot_stby',
                          'feedback_partial_edges']
        if board:
            scenarios = ['feedback_chg_only'] if chg_only else [
                'feedback_board', 'feedback_board_errors', 'feedback_board_initfail',
            ]
        if no_gpio:
            scenarios = ['feedback_no_gpio', 'poweroff']
        if board in ("kawazu", "generic-stat"):
            scenarios = [
                'stat_raw_levels', 'stat_transitions',
                'stat_config_readback', 'stat_latch', 'stat_errors', 'stat_owner',
                'stat_wake_low', 'stat_wake_high', 'stat_wake_hiz', 'stat_wake_dock',
                'stat_wake_ignore', 'stat_wake_error', 'stat_wake_bias_error', 'stat_boot',
            ]
            scenarios += ['stat_pulses', 'stat_presence']
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
