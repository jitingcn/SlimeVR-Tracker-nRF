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
    (temporary / 'production.inc').write_text(filter_source + '\n' + locals_source + '\n' + iteration)
    for pmic in (0, 1):
        binary = temporary / f'battery-sampling-{pmic}'
        subprocess.run(shlex.split(os.environ.get('CC', 'cc')) + [
            '-std=gnu11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter',
            '-g', '-O1', '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
            '-fno-pie', '-no-pie', f'-DTEST_PMIC={pmic}', f'-I{temporary}',
            str(HERE / 'test.c'), '-o', str(binary)], check=True)
        scenarios = ['cadence', 'filter', 'warmup', 'outlier', 'failure', 'dock', 'low', 'debounce']
        if not pmic:
            scenarios += ['edges', 'settle']
        for scenario in scenarios:
            subprocess.run([str(binary), scenario], check=True)
