#!/usr/bin/env python3
"""Exercise the production voltage maps and read path with modeled ADC leaves."""
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "harness/python"))
from c_extract import extract_block

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2]))
battery = (ROOT / "src/system/battery.c").read_text()
production = "\n".join((
    extract_block(battery, r"^unsigned int\s+battery_level_pptt\([^;]*\)\s*\{"),
    extract_block(battery, r"^static const struct battery_level_point levels\[\]\s*=\s*\{", semicolon=True),
    extract_block(battery, r"^int read_batt_mV\([^;]*\)\s*\{"),
))
prefix = r'''
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "system/battery.h"
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define DT_PROP_OR(node, prop, default_value) (default_value)
#define LOG_ERR(...) ((void)0)
#define LOG_DBG(...) ((void)0)
static int measurement, enable_error;
static unsigned enables, disables, samples, waits;
int battery_measure_enable(bool enabled) {
    if (enabled) { enables++; return enable_error; }
    disables++;
    return 0;
}
int battery_sample(void) { samples++; return measurement; }
static void k_usleep(int usec) { assert(usec == 200); waits++; }
'''
suffix = r'''
int main(void) {
    int mv = -1;
#if CONFIG_BATTERY_NO_MAPPING || TEST_FALLBACK
    const int voltages[] = {0, 3200, 3800, 5000};
    for (unsigned i = 0; i < sizeof(voltages) / sizeof(voltages[0]); i++) {
        measurement = voltages[i];
        assert(read_batt_mV(&mv) == 10000 && mv == measurement);
    }
#elif CONFIG_BATTERY_USE_REG_BUCK_MAPPING
    measurement = 3200; assert(read_batt_mV(&mv) == 0 && mv == 3200);
    measurement = 3775; assert(read_batt_mV(&mv) == 3000 && mv == 3775);
    measurement = 4150; assert(read_batt_mV(&mv) == 10000 && mv == 4150);
#else
    measurement = 3200; assert(read_batt_mV(&mv) == 0 && mv == 3200);
    measurement = 3650; assert(read_batt_mV(&mv) == 3000 && mv == 3650);
    measurement = 4150; assert(read_batt_mV(&mv) == 10000 && mv == 4150);
#endif
    assert(enables == samples && samples == disables && samples == waits);
    unsigned previous_samples = samples, previous_disables = disables;
    enable_error = -EBUSY;
    mv = 42;
    assert(read_batt_mV(&mv) == -EBUSY && mv == -1);
    assert(samples == previous_samples && disables == previous_disables);
    enable_error = 0;
    measurement = -EIO;
    assert(read_batt_mV(&mv) == -EIO && mv == -EIO);
    assert(samples == previous_samples + 1 && disables == previous_disables + 1);
    assert(read_batt_mV(NULL) == -EIO);
    puts("battery mapping and measurement errors passed");
    return 0;
}
'''

with tempfile.TemporaryDirectory(prefix="battery-mapping-") as directory:
    temp = Path(directory)
    source = temp / "test.c"
    source.write_text(prefix + production + suffix)
    variants = (
        ("no-mapping", "CONFIG_BATTERY_NO_MAPPING", False),
        ("ldo", "CONFIG_BATTERY_USE_REG_LDO_MAPPING", False),
        ("buck", "CONFIG_BATTERY_USE_REG_BUCK_MAPPING", False),
        ("missing-choice", "TEST_FALLBACK", True),
    )
    for name, definition, allow_warning in variants:
        binary = temp / name
        warning_flags = ["-Wno-error=cpp"] if allow_warning else []
        result = subprocess.run(shlex.split(os.environ.get("CC", "cc")) + warning_flags + [
            "-std=c11", "-Wall", "-Wextra", "-Werror",
            "-fsanitize=undefined", "-fno-sanitize-recover=all", f"-D{definition}=1",
            "-I", str(ROOT / "src"), str(source), "-o", str(binary),
        ], text=True, capture_output=True)
        assert result.returncode == 0, result.stderr
        subprocess.run([str(binary)], check=True)
