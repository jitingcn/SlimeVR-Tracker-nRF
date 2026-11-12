#!/usr/bin/env python3
"""An interrupt between TIMER and kernel reads must not move hardware END."""
from pathlib import Path
import os, shlex, subprocess, tempfile
root = Path(__file__).resolve().parents[3]
src = (root / 'src/connection/radio_capture.c').read_text()
a = src.index('bool radio_capture_rx_end(')
b = src.index('\nvoid radio_capture_deinit', a)
code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#define CONFIG_SOC_SERIES_NRF52 1
struct capture_record { uint64_t end_us; };
static bool capture_initialized=true, valid=true, locked;
static uint64_t end_us=900, now_us=1000;
static unsigned irq_lock(void) { unsigned old=locked; locked=true; return old; }
static void irq_unlock(unsigned old) { locked=old; }
static bool capture_platform_read(struct capture_record *r) { r->end_us=end_us; return valid; }
static uint64_t capture_platform_now(void) { return now_us; }
/* Model a seven-tick interrupt at exactly the cross-clock sampling boundary. */
static uint64_t k_uptime_ticks(void) { return 32+(locked ? 0 : 7); }
static uint32_t k_us_to_ticks_floor32(uint32_t us) { return (uint64_t)us*32768/1000000; }
''' + src[a:b] + r'''
int main(void) {
 uint32_t ticks=0;
 assert(radio_capture_rx_end(&ticks) && ticks==29 && !locked);
 valid=false; assert(!radio_capture_rx_end(&ticks) && !locked);
 valid=true; now_us=2001; assert(!radio_capture_rx_end(&ticks) && !locked);
 capture_initialized=false; assert(!radio_capture_rx_end(&ticks) && !locked);
 return 0;
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d); (p/'test.c').write_text(code)
 subprocess.run(shlex.split(os.environ.get('CC','cc'))+['-std=c11','-Wall','-Wextra','-Werror',str(p/'test.c'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
print('Hardware END conversion survives interrupt injection; stale captures still rejected')
