"""Extract the production clock section and SDK power functions for native_sim."""
from pathlib import Path
import hashlib
import re
import sys

esb_path, power_path, dest = map(Path, sys.argv[1:])
esb = esb_path.read_text()
power = power_path.read_text()


def function(source, name):
    pattern = rf'^(?:static\s+)?(?:int|void)\s+{re.escape(name)}\s*\([^;]*?\)\s*\{{'
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


start = re.search(r'^bool\s+clock_status\s*=\s*false\s*;', esb, re.MULTILINE)
clock_start = function(esb, 'clocks_request_start').rstrip('\n')
end = esb.find(clock_start, start.end()) if start else -1
if not start or end < 0:
    raise RuntimeError('tracker clock section boundaries not found')
clock = esb[start.start():end] + clock_start + '\n\n'
if not all(name in clock for name in ('clocks_start(', 'clocks_stop(', 'esb_release_tx_clock(')):
    raise RuntimeError('tracker clock section is incomplete')
pair_body = function(esb, 'esb_pair')
pair_start = re.search(r'(?m)^[ \t]*atomic_inc\(&tx_clock_users\)\s*;', pair_body)
response_wait = r'k_msleep\s*\(\s*60\s*\+\s*\(\s*k_cycle_get_32\(\)\s*%\s*23\s*\)\s*\)\s*;'
if pair_start:
    # The current code holds its production lease for the complete burst.
    gate = ''
    pair_end_pattern = response_wait + r'\s*atomic_dec\(&tx_clock_users\)\s*;'
else:
    # The pre-fix snapshot starts the clock earlier and had no pair lease.
    gate_match = re.search(
        r'(?m)^[ \t]*if\s*\(\s*!clock_status\s*\)\s*\{\s*clocks_start\(\)\s*;\s*\}',
        pair_body,
    )
    pair_start = re.search(r'(?m)^[ \t]*if\s*\(\s*esb_send_pair_step\s*\(\s*0\s*\)\s*\)', pair_body)
    if not gate_match:
        raise RuntimeError('legacy pairing clock admission not found')
    gate = gate_match.group()
    pair_end_pattern = response_wait
if not pair_start:
    raise RuntimeError('pairing start or first send not found')
pair_end = re.search(pair_end_pattern, pair_body[pair_start.start():])
if not pair_end:
    raise RuntimeError('pairing response wait not found')
pair_steps = pair_body[pair_start.start():pair_start.start() + pair_end.end()]
boot_body = function(esb, 'esb_thread')
boot = re.search(
    r'memcpy\s*\(\s*paired_addr\s*,\s*retained->paired_addr\s*,\s*sizeof\s*\(\s*paired_addr\s*\)\s*\)\s*;'
    r'(.*?)\bwhile\s*\(\s*1\s*\)\s*\{',
    boot_body, re.DOTALL,
)
if not boot or 'clocks_request_start' not in boot.group(1) or 'clock_init_external_async' not in boot.group(1):
    raise RuntimeError('boot clock startup block not found')
radio = ''.join(function(esb, name) for name in (
    'esb_deinitialize', 'esb_user_set_enabled', 'esb_write', 'esb_write_ping'))


counter = re.search(r'^static\s+uint8_t\s+m_power_mode_refs\s*=\s*0\s*;', power, re.MULTILINE)
if not counter:
    raise RuntimeError('nrfx constant-latency counter not found')
power_funcs = counter.group() + '\n\n' + ''.join(
    function(power, name) for name in ('nrfx_power_constlat_mode_request', 'nrfx_power_constlat_mode_free'))
has_owned_flag = bool(re.search(r'^static\s+bool\s+clocks_hf_owned\s*;', clock, re.MULTILINE))
(dest / 'tracker_clock.inc').write_text(
    f'#define FIXTURE_HAS_HF_OWNED {int(has_owned_flag)}\n' + clock)
(dest / 'tracker_pair_gate.inc').write_text(
    'static void pairing_clock_gate(void)\n{\n'
    '    k_mutex_lock(&esb_radio_lock, K_FOREVER);\n'
    '    do {\n' + gate + '\n' + pair_steps + '\n    } while (0);\n'
    '    k_mutex_unlock(&esb_radio_lock);\n}\n')
(dest / 'tracker_boot.inc').write_text('static void boot_clock_request(void)\n{\n' + boot.group(1) + '\n}\n')
(dest / 'tracker_radio.inc').write_text(radio)
(dest / 'nrfx_power.inc').write_text(power_funcs)
for path, text in ((esb_path, esb), (power_path, power)):
    print(f'SOURCE sha256={hashlib.sha256(text.encode()).hexdigest()} {path}', flush=True)
