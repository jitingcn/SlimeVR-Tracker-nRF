"""Extract complete on-disk production bodies into a native simulator TU."""
from pathlib import Path
import hashlib
import re
import sys

tracker, twim_path, dest = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
sensor = (tracker / 'src/sensor/sensor.c').read_text()
scan = (tracker / 'src/sensor/scan.c').read_text()
twim = twim_path.read_text()

def function(source, name):
    pattern = rf'^(?:static\s+)?(?:int|void|bool)\s+{re.escape(name)}\s*\([^;]*?\)\s*(?://[^\n]*)?\s*\{{'
    match = re.search(pattern, source, re.MULTILINE)
    if not match:
        raise RuntimeError(f'complete function not found: {name}')
    depth = 0
    tokens = re.compile(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]', re.DOTALL)
    for token in tokens.finditer(source, source.index('{', match.start())):
        if token.group() == '{':
            depth += 1
        elif token.group() == '}':
            depth -= 1
            if not depth:
                return source[match.start():token.end()] + '\n\n'
    raise RuntimeError(f'unclosed function: {name}')

def pieces(source, names):
    return ''.join(function(source, n) for n in names)

match = re.search(r'^struct i2c_nrfx_twim_data\s*\{.*?\};', twim, re.MULTILINE | re.DOTALL)
if not match:
    raise RuntimeError('TWIM data structure not found')
(dest / 'twim_data.inc').write_text(match.group(0) + '\n')
(dest / 'twim_funcs.inc').write_text(pieces(twim, [
    'i2c_nrfx_twim_exclusive_access_acquire', 'i2c_nrfx_twim_exclusive_access_release',
    'i2c_nrfx_twim_transfer', 'event_handler']))
(dest / 'scan_funcs.inc').write_text(function(scan, 'sensor_scan_i2c'))
(dest / 'lifecycle_funcs.inc').write_text(pieces(sensor, [
    'sensor_life_mark_idle', 'sensor_life_mark_busy', 'sensor_lifecycle_lock_take', 'sensor_activity_quiesce',
    'sensor_output_ready',
    'sensor_loop_enter', 'sensor_loop_leave', 'sensor_scan_imu_once', 'sensor_scan',
    'sensor_scan_thread', 'main_imu_suspend', 'main_imu_resume',
    'sensor_request_scan_locked', 'sensor_request_scan', 'sensor_transfer_suspension_to_power',
    'sensor_release_failed_restart',
    'sensor_shutdown', 'sensor_setup_WOM']))
(dest / 'sensor_loop.inc').write_text(function(sensor, 'sensor_loop'))
(dest / 'sensor_wait.inc').write_text(function(sensor, 'sensor_loop_wait'))
heated_suspend = function(sensor, 'main_imu_suspend').replace(
    'int main_imu_suspend(void)', 'static int sensor_suspend_heated(void)', 1)
(dest / 'sensor_suspend_heated.inc').write_text(heated_suspend)
for path, text in [(tracker / 'src/sensor/sensor.c', sensor),
                   (tracker / 'src/sensor/scan.c', scan), (twim_path, twim)]:
    print(f'SOURCE sha256={hashlib.sha256(text.encode()).hexdigest()} {path}')
