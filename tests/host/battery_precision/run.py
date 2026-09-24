#!/usr/bin/env python3
"""Run production battery sampling with the active Zephyr ADC converters."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2]))
SDK = Path(os.environ.get("NCS_ROOT", "/home/jiting/ncs/v3.4-branch"))
battery = (ROOT / "src/system/battery.c").read_text()
adc = (SDK / "zephyr/include/zephyr/drivers/adc.h").read_text()
common = (SDK / "zephyr/drivers/adc/adc_common.c").read_text()


def function(source, name):
    match = re.search(rf"^(?:static inline )?int {name}\([^;{{]*\)\s*\{{", source, re.MULTILINE)
    if match is None:
        raise ValueError(name)
    start = source.index("{", match.start())
    tokens = re.compile(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]', re.DOTALL)
    depth = 0
    for token in tokens.finditer(source, start):
        if token.group() == "{":
            depth += 1
        elif token.group() == "}":
            depth -= 1
            if depth == 0:
                return source[match.start():token.end()]
    raise ValueError(f"Unclosed function: {name}")


prefix = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <errno.h>
#include <limits.h>
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define __ASSERT_NO_MSG(x) assert(x)
#define __ASSERT_MSG_INFO(...) assert(0)
#define LOG_INF(...) ((void)0)
#define USE_PMIC_CHARGER 0
'''
enum_start = adc.index("enum adc_gain {")
enum_end = adc.index("};", enum_start) + 2
parts = [prefix, adc[enum_start:enum_end],
         common[common.index("struct gain_desc {"):common.index("\nint adc_gain_invert(")],
         function(common, "adc_gain_invert_64"),
         function(adc, "adc_raw_to_microvolts"),
         function(adc, "adc_raw_to_millivolts")]
parts += [r'''
struct device { int unused; };
struct adc_sequence { uint8_t resolution; bool calibrate; };
struct divider_data {
    const struct device *adc;
    struct { enum adc_gain gain; } adc_cfg;
    struct adc_sequence adc_seq;
    int16_t raw;
};
struct divider_config { uint32_t output_ohm, full_ohm; };
static struct divider_data divider_data;
static struct divider_config divider_config;
static bool battery_ok;
static int read_result;
static int adc_read(const struct device *dev, struct adc_sequence *seq) {
    (void)dev; (void)seq;
    return read_result;
}
static uint16_t adc_ref_internal(const struct device *dev) {
    (void)dev;
    return 600;
}
''', function(battery, "battery_sample"), r'''
static int sample(int16_t raw, enum adc_gain gain, uint32_t output, uint32_t full) {
    divider_data.raw = raw;
    divider_data.adc_cfg.gain = gain;
    divider_data.adc_seq.resolution = 14;
    divider_config.output_ohm = output;
    divider_config.full_ohm = full;
    return battery_sample();
}
int main(void) {
    battery_ok = true;
    /* P10: VDDHDIV5, 600 mV reference, gain 1/2, 14-bit ADC.
     * 10004 codes represent 3.66357421875 V before integer quantization. */
    assert(sample(10000, ADC_GAIN_1_2, 1, 5) == 3662);
    assert(sample(10004, ADC_GAIN_1_2, 1, 5) == 3663);
    assert(sample(11469, ADC_GAIN_1_2, 1, 5) == 4200);
    assert(sample(0, ADC_GAIN_1_2, 1, 5) == 0);
    assert(sample(-10000, ADC_GAIN_1_2, 1, 5) == -3662);
    /* Direct VDD: the public result remains integer millivolts. */
    assert(sample(15023, ADC_GAIN_1_6, 0, 0) == 3300);
    /* External divider, including a product exceeding signed 32-bit range. */
    assert(sample(5000, ADC_GAIN_1_6, 100000, 330000) == 3625);
    assert(sample(5000, ADC_GAIN_1_6, 1000000000, 3300000000U) == 3625);
    /* Zephyr rejects invalid gain; it must never become a voltage reading. */
    assert(sample(10004, (enum adc_gain)255, 1, 5) == -EINVAL);
    read_result = -EIO;
    assert(sample(10004, ADC_GAIN_1_2, 1, 5) == -EIO);
    read_result = 0;
    battery_ok = false;
    assert(sample(10004, ADC_GAIN_1_2, 1, 5) == -ENOENT);
    puts("PASS production battery precision: P10, direct VDD, external divider, signed scaling and errors");
    return 0;
}
''']
with tempfile.TemporaryDirectory(prefix="battery-precision-") as tmp:
    source = Path(tmp) / "test.c"
    executable = Path(tmp) / "test"
    source.write_text("\n".join(parts))
    subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
        "-std=c11", "-Wall", "-Wextra", "-Werror", "-fsanitize=undefined",
        "-fno-sanitize-recover=all", str(source), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
