#!/usr/bin/env python3
"""Replay PONG and replacement IRQs against the actual timeout operation."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'harness/python'))
from c_extract import extract_block

if len(sys.argv) > 2 or (len(sys.argv) == 2 and sys.argv[1] not in ('pong', 'replacement', 'timeout')):
    raise SystemExit('usage: timeout_race.py [pong|replacement|timeout]')

source = Path(os.environ.get('RADIO_ESB_SOURCE', HERE.parents[2] / 'src/connection/esb.c')).read_text()
if re.search(r'^static void esb_ping_timeout\(', source, re.M):
    timeout = extract_block(source, r'^static void esb_ping_timeout\([^;{]*\)\s*\{')
else:
    old = extract_block(source, r'^\s*if \(ping_pending && \(now_idle - ping_send_time\)[^\{]*\{')
    timeout = 'static void esb_ping_timeout(int64_t now_idle) {\n' + old + '\n}'
match = re.search(r'if \(ping_pending[^{}]*\{', timeout, re.S)
if not match:
    raise ValueError('PING timeout condition not found')
timeout = timeout[:match.end()] + '\n        probe_interleaving();\n' + timeout[match.end():]

fixture = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define TX_ERROR_THRESHOLD 3
#define ESB_ST_RECOVERING 2
static bool ping_pending, ping_failed, accepted;
static uint32_t ping_failures, ping_success_streak, ping_counter;
static int64_t ping_send_time, connection_error_start_time;
static int esb_conn_state;
static unsigned locked, deferred, action, logs;
static void deliver(unsigned which) {
    if (which == 1 && ping_pending) {
        ping_pending = false; ping_failed = false; ping_failures = 0; accepted = true;
    } else if (which == 2) {
        ping_counter++; ping_send_time = 1000; ping_pending = true; ping_failed = false;
    }
}
static void probe_interleaving(void) {
    unsigned which = action; action = 0;
    if (locked) deferred = which; else deliver(which);
}
static unsigned irq_lock(void) { return locked++; }
static void irq_unlock(unsigned key) {
    assert(locked == key + 1); locked = key;
    if (!locked && deferred) { unsigned which = deferred; deferred = 0; deliver(which); }
}
static unsigned get_ping_interval_ms(void) { return 1000; }
static void warning(const char *fmt, ...) { (void)fmt; assert(!locked); ++logs; }
#define LOG_WRN(...) warning(__VA_ARGS__)
'''
fixture += '\n' + timeout + r'''
static void reset(unsigned which) {
    ping_pending = true; ping_failed = accepted = false;
    ping_failures = ping_success_streak = ping_counter = 0;
    ping_send_time = connection_error_start_time = 0;
    esb_conn_state = 1; action = which; deferred = locked = logs = 0;
}
int main(int argc, char **argv) {
    bool all = argc == 1;
    if (all || strcmp(argv[1], "pong") == 0) {
        reset(1); esb_ping_timeout(1000);
        assert((accepted && !ping_failed && ping_failures == 0)
            || (!accepted && ping_failed && ping_failures == 1));
    }
    if (all || strcmp(argv[1], "replacement") == 0) {
        reset(2); esb_ping_timeout(1000);
        assert(ping_counter == 1 && ping_send_time == 1000 && ping_pending && !ping_failed);
    }
    if (all || strcmp(argv[1], "timeout") == 0) {
        reset(0); ping_failures = 2; esb_ping_timeout(1000);
        assert(!ping_pending && ping_failed && ping_failures == 3);
        assert(connection_error_start_time == 1000 && esb_conn_state == ESB_ST_RECOVERING && logs == 1);
        esb_ping_timeout(1000); assert(ping_failures == 3 && logs == 1);
    }
    puts("PASS actual timeout protects PONG, replacement and single threshold claim");
}
'''
with tempfile.TemporaryDirectory(prefix='tracker-ping-race-') as directory:
    directory = Path(directory)
    test = directory / 'timeout.c'
    binary = directory / 'timeout'
    test.write_text(fixture)
    subprocess.run(shlex.split(os.environ.get('CC', 'cc')) + [
        '-std=c11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function', '-O1', '-g',
        '-fsanitize=undefined', str(test), '-o', str(binary)], check=True)
    subprocess.run([str(binary), *sys.argv[1:]], check=True)
