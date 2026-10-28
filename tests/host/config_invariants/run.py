#!/usr/bin/env python3
"""Exercise the application Kconfig and retained-memory build constraints."""
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2]))
ZEPHYR = Path(os.environ.get("ZEPHYR_BASE", "/home/jiting/ncs/v3.4-branch/zephyr"))
CC = shlex.split(os.environ.get("CC", "cc"))

# The application sources its board/Zephyr definitions. Keep those unrelated
# definitions minimal while parsing the real application Kconfig and fragments.
ZEPHYR_STUB = """
config CRC
    bool
config PWM
    bool
config NRFX_TIMER
    bool
config NRFX_GPPI
    bool
config UDC_BUF_POOL_SIZE
    int
choice USBD_LOG_LEVEL_CHOICE
    bool "USB log"
config USBD_LOG_LEVEL_ERR
    bool "error"
endchoice
choice UDC_DRIVER_LOG_LEVEL_CHOICE
    bool "UDC log"
config UDC_DRIVER_LOG_LEVEL_ERR
    bool "error"
endchoice
choice USBD_CDC_ACM_LOG_LEVEL_CHOICE
    bool "CDC log"
config USBD_CDC_ACM_LOG_LEVEL_OFF
    bool "off"
endchoice
choice NRF_GRTC_TIMER_SOURCE
    bool "GRTC source"
config NRF_GRTC_TIMER_SOURCE_LFXO
    bool "lfxo"
config NRF_GRTC_TIMER_SOURCE_SYSTEM_LFCLK
    bool "system"
config NRF_GRTC_TIMER_SOURCE_LFLPRC
    bool "lprc"
endchoice
"""

STUBS = {
    "app_version.h": "",
    "zephyr/kernel.h": """
#include <stdint.h>
#define NRF_STATIC_ASSERT(expr, msg) _Static_assert(expr, msg)
#define K_MUTEX_DEFINE(name) int name
#define K_FOREVER 0
#define SYS_INIT(fn, level, priority)
static inline uint64_t k_uptime_ticks(void) { return 0; }
static inline int k_mutex_lock(int *m, int t) { (void)m; (void)t; return 0; }
static inline int k_mutex_unlock(int *m) { (void)m; return 0; }
""",
    "zephyr/devicetree.h": """
#define DT_NODE_HAS_STATUS_OKAY(node) 1
#define DT_ALIAS(node) 0
#define DT_PARENT(node) 0
#define DT_REG_ADDR(node) 0
""",
    "zephyr/drivers/retained_mem.h": "",
    "zephyr/sys/byteorder.h": """
static inline uint32_t sys_cpu_to_le32(uint32_t v) { return v; }
""",
    "zephyr/sys/crc.h": """
static inline uint32_t crc32_ieee(const uint8_t *p, size_t n) { (void)p; (void)n; return 0; }
""",
}


def expect(result, success, message):
    if (result.returncode == 0) != success:
        raise AssertionError(f"{message}: exit {result.returncode}\n{result.stdout}\n{result.stderr}")


def main():
    count = 0
    with tempfile.TemporaryDirectory(prefix="tracker-config-") as directory:
        temp = Path(directory)
        (temp / "Kconfig.zephyr").write_text(ZEPHYR_STUB)
        for path, contents in STUBS.items():
            target = temp / path
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(contents)
        env = os.environ.copy()
        env["srctree"] = str(temp)
        env["ZEPHYR_BASE"] = str(ZEPHYR)

        def config(name, assignments, success=True):
            nonlocal count
            fragment = temp / f"{name}.conf"
            output = temp / f"{name}.config"
            header = temp / f"{name}.h"
            listing = temp / f"{name}.list"
            fragment.write_text("".join(f"CONFIG_{key}={value}\n" for key, value in assignments.items()))
            result = subprocess.run([
                sys.executable, str(ZEPHYR / "scripts/kconfig/kconfig.py"),
                "--handwritten-input-configs", str(ROOT / "Kconfig"),
                str(output), str(header), str(listing), str(fragment),
            ], cwd=ROOT, env=env, text=True, capture_output=True)
            expect(result, success, name)
            if success:
                actual = output.read_text().splitlines()
                for key, value in assignments.items():
                    expected = f"CONFIG_{key}={value}" if value != "n" else f"# CONFIG_{key} is not set"
                    assert expected in actual, f"{name}: missing {expected}"
            count += 1
            return header

        def compile_retained(name, header, success):
            nonlocal count
            result = subprocess.run(CC + [
                "-std=c11", "-fsyntax-only", "-include", str(header),
                "-I", str(temp), "-I", str(ROOT / "src"), str(ROOT / "src/retained.c"),
            ], text=True, capture_output=True)
            expect(result, success, name)
            count += 1

        header = config("defaults", {})
        compile_retained("default retained data", header, True)
        for name in ("BATTERY_NO_MAPPING", "BATTERY_USE_REG_LDO_MAPPING", "BATTERY_USE_REG_BUCK_MAPPING"):
            config(name, {name: "y"})
        for value in (0, 100):
            config(f"rf-{value}", {"RADIO_RF_CHANNEL": value})
        for value in (-1, 101):
            config(f"rf-{value}", {"RADIO_RF_CHANNEL": value}, False)
        for key in ("SENSOR_ACCEL_ODR", "SENSOR_GYRO_ODR", "SENSOR_MAG_ODR", "SENSOR_POLY_STEPS_PER_DEGREE"):
            for value in (1, 2147483647):
                config(f"{key}-{value}", {key: value})
            for value in (0, -1):
                config(f"{key}-{value}", {key: value}, False)
        for name, minimum, maximum, valid in (
            ("adjacent", 0, 1, True),
            ("negative-adjacent", -10, -9, True),
            ("equal", 10, 10, False),
            ("reversed", 11, 10, False),
        ):
            header = config(name, {"SENSOR_POLY_TEMP_MIN": minimum, "SENSOR_POLY_TEMP_MAX": maximum})
            compile_retained(name, header, valid)
        header = config("tcal-disabled", {"SENSOR_USE_TCAL": "n"})
        compile_retained("tcal disabled", header, True)
        header = config("oversized-retained", {"SENSOR_POLY_TEMP_MIN": 10, "SENSOR_POLY_TEMP_MAX": 150})
        compile_retained("oversized retained data", header, False)
    print(f"Configuration invariant checks passed: {count}")


if __name__ == "__main__":
    main()
