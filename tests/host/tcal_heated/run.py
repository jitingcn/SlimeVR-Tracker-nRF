#!/usr/bin/env python3
"""Compile the complete production controller and IMU owner with boundary adapters."""
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
ROOT = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2])).resolve()

GLOBALS = r'''
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#define CONFIG_SENSOR_TCAL_HEATED 1
#define CONFIG_SENSOR_USE_TCAL 1
#define CONFIG_SENSOR_USE_ACCEL_CALIBRATION 1
#define CONFIG_SENSOR_USE_SENS_CALIBRATION 0
#define CONFIG_CMSIS_DSP 0
#define CONFIG_SENSOR_POLY_TEMP_MIN 10
#define CONFIG_SENSOR_POLY_TEMP_MAX 45
#define CONFIG_SENSOR_POLY_STEPS_PER_DEGREE 2
#define CONFIG_SENSOR_POLY_DEGREE 3
#define CONFIG_SENSOR_TCAL_HEATED_MAX_TEMP_C 50
#ifndef CONFIG_SENSOR_TCAL_HEATED_TIMEOUT_MIN
#define CONFIG_SENSOR_TCAL_HEATED_TIMEOUT_MIN 90
#endif
#define CONFIG_SENSOR_TCAL_HEATED_RESUME_STABLE_MS 5000
#define CONFIG_SENSOR_TCAL_HEATED_RAMP_HALF_DEGREE_MS 45000
#define CONFIG_SENSOR_TCAL_HEATED_MAX_RISE_MCPS 200
#define CONFIG_SENSOR_TCAL_HEATED_DEFAULT_KP 300
#ifndef CONFIG_SENSOR_TCAL_HEATED_DEFAULT_KI
#define CONFIG_SENSOR_TCAL_HEATED_DEFAULT_KI 5
#endif
#ifndef CONFIG_SENSOR_TCAL_HEATED_DEFAULT_KFF
#define CONFIG_SENSOR_TCAL_HEATED_DEFAULT_KFF 0
#endif
#define CONFIG_SYSTEM_IMU_HEATER_MAX_DUTY_PPTT 7000
#define CONFIG_SENSOR_TCAL_HEATED_SLEW_PPTT_PER_S 500
#define CONFIG_SENSOR_TCAL_HEATED_STABLE_BAND_MC 500
#define CONFIG_SENSOR_TCAL_HEATED_STABLE_MS 60000
#define LOG_LEVEL_INF 3
#define LOG_MODULE_REGISTER(...)
#define LOG_INF(...) ((void)0)
#define LOG_ERR(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define LOG_DBG(...) ((void)0)
#define CLAMP(x, low, high) ((x) < (low) ? (low) : ((x) > (high) ? (high) : (x)))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define printk(...) ((void)printf(__VA_ARGS__))
#include "retained.h"
int64_t k_uptime_get(void);
'''

SENSOR = r'''
#pragma once
#include <stdbool.h>
#include <stdint.h>
struct sensor_temperature_observation {
    float raw_c;
    float filtered_c;
    int64_t sampled_at_ms;
    uint32_t sequence;
};
int sensor_get_imu_temperature_observation(struct sensor_temperature_observation *, int64_t);
bool sensor_peek_accel(float out[3]);
bool sensor_peek_accel_fresh(float out[3], int64_t max_age);
float sensor_get_gyro_odr(void);
'''

SYSTEM = r'''
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define MAIN_ACCEL_BIAS_ID 3
#define MAIN_GYRO_BIAS_ID 4
#define MAIN_ACC_6_BIAS_ID 7
#define MAIN_GYRO_TEMP_ID 8
#define MAIN_GYRO_TCAL_STATE_ID 9
#define MAIN_GYRO_TCAL_POINTS_ID 10
#define MAIN_GYRO_TCAL_COEFFS_ID 11
int sys_write(uint16_t id, void *ptr, const void *data, size_t size);
void sys_warm_transaction_begin(void);
void sys_warm_transaction_mark(uint16_t id, const void *data, size_t size);
void sys_warm_transaction_end(bool schedule);
void sys_warm_feedback_arm(uint32_t identity);
'''

UTIL = r'''
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
/* Reuse the existing vector adapter, but keep the firmware's bit-based finite
 * boundary: ordinary isfinite is optimized away in the fast-math variant. */
#define v_finite host_unused_isfinite
#include "IMU_UTIL_PATH"
#undef v_finite
static inline bool v_finite(const float *v, size_t n) {
    for (size_t i = 0; i < n; i++) {
        uint32_t bits;
        memcpy(&bits, &v[i], sizeof(bits));
        if ((bits & 0x7f800000u) == 0x7f800000u) return false;
    }
    return true;
}
'''

SCENARIOS = (
    "admission", "imu_exclusion", "pending_candidate", "reset_barrier",
    "raw_overshoot", "filtered_overshoot", "raw_rise", "same_sequence",
    "rise_noise", "rise_recovery", "rise_plant", "rise_cadence20", "rise_cadence113",
    "rise_cadence1500", "rise_jitter", "rise_worker", "rise_worker_cut",
    "rise_finalize", "rise_restart", "rise_invalid_time",
    "rise_invalid_back", "rise_invalid_future", "rise_invalid_gap",
    "rise_invalid_stale", "rise_invalid_duplicate", "rise_invalid_nan",
    "missed_deadline", "power_loss", "hardware_fault", "motion_resume",
    "stop_commit", "restart_epoch", "publish", "insufficient_bins",
    "missing_start", "missing_target", "invalid_stage", "model_generation",
    "stop_during_finalize", "restart_during_finalize", "reset_during_finalize",
    "timeout_discard", "partial_finish", "endpoint_bands", "ota_abort",
    "suppression_abort", "off_failure", "accum_minimum", "accum_accel",
    "accum_motion", "accum_rate", "accum_abort", "accum_nonfinite",
    "power_during_finalize", "ota_during_finalize", "quantized_rise", "sustained_rise",
    "ordinary_start_stop", "ordinary_reset", "ordinary_maintenance", "pi_feedback",
    "integral_headroom", "slot_boundary", "slot_chatter", "slot_revisit",
    "slot_no_overwrite", "start_dwell", "ordinary_interval_ema", "accum_epoch",
    "stop_empty", "stop_reserved", "stop_one", "stop_two", "stop_three",
    "stop_merge_existing", "stop_fault", "stop_rise_fault", "stop_pending_reset", "stop_pending_abort",
    "stop_pending_power", "stop_pending_ota", "stop_pending_generation",
    "stop_pending_stale", "stop_pending_overtemp", "stop_pending_timeout",
    "warm_receipt",
    "ramp_trajectory", "ramp_jitter_pause", "ramp_target_clamp", "default_ramp_budget",
)

ROBUST_SCENARIOS = (
    "clean", "linear", "first", "spikes", "paired", "noise", "noise_spikes",
    "odr104", "odr833", "odr1600", "bursty", "coverage", "majority",
    "tail1", "tail2", "tail3", "tail4", "tail3_bad", "tail4_bad",
    "partial_spike", "motion", "thermal", "reset", "accel",
)
SCENARIOS += tuple(f"robust_{mode}_{case}" for mode in ("ordinary", "heated")
                   for case in ROBUST_SCENARIOS) + ("robust_ordinary_motion_tail",)



def function(name, source):
    """Extract the exact production definition."""
    return extract_block(source, rf"^(?:static )?(?:void|int|bool|float|uint32_t|sensor_tcal_apply_mode_t) {re.escape(name)}\([^;{{]*\)\s*\{{")


def accumulator_source():
    source = (ROOT / "src/sensor/calibration/tcal_runtime.c").read_text()
    state = re.search(r"static struct \{(?:(?!static struct).)*?\} tcal_accum;", source, re.S)
    if state is None:
        raise ValueError("production accumulator state")
    constants = "\n".join(line for line in source.splitlines()
                          if line.startswith(("#define TCAL_ACCUM_", "#define TCAL_SAVE_")))
    reset_state = source[state.end():source.index("static void tcal_accum_flush", state.end())]
    direction_state = source[source.index("tcal_temp_direction_t tcal_current_direction"):state.start()]
    names = ("tcal_accum_reset", "tcal_accum_request_reset", "tcal_accum_apply_reset",
             "sensor_tcal_heated_accum_feed", "sensor_tcal_heated_accum_finish")
    aliases = "\n".join(f"#define {name} production_{name}" for name in names)
    unaliases = "\n".join(f"#undef {name}" for name in names)
    functions = "\n\n".join(function(name, source) for name in (
        "tcal_accum_reset", "tcal_accum_request_reset", "tcal_accum_apply_reset",
        "tcal_save_point", "tcal_accum_check_accel", "tcal_accum_median",
        "tcal_accum_commit_blocks", "tcal_accum_flush", "tcal_accum_feed",
        "sensor_tcal_heated_accum_feed", "sensor_tcal_heated_accum_finish",
        "sensor_tcal_continuous_motion_detected"))
    return "\n".join((
        '#include <assert.h>\n#include <stdatomic.h>',
        "typedef atomic_int atomic_t;\ntypedef int atomic_val_t;",
        "static inline int atomic_set(atomic_t *p, int v) { return atomic_exchange(p, v); }",
        "static inline int atomic_clear(atomic_t *p) { return atomic_exchange(p, 0); }",
        "static inline int atomic_get(const atomic_t *p) { return atomic_load(p); }",
        "static inline int atomic_inc(atomic_t *p) { return atomic_fetch_add(p, 1); }",
        constants, direction_state, state.group(), reset_state,
        aliases, functions, unaliases,
    ))


def build_harness(work, main_source=None, fast_math=False, feedforward=0, timeout_min=90, integral_gain=5):
    """Build real controller+accumulator+IMU fixture; caller runs the executable.

    main_source replaces main only; adapter state/helpers remain accessible.
    Caller owns the temporary directory and executable lifetime.
    """
    work = Path(work)
    work.mkdir(parents=True, exist_ok=True)
    adapters = {
        "globals.h": GLOBALS,
        "sensor/sensor.h": SENSOR,
        "system/system.h": SYSTEM,
        "system/power.h": "#pragma once\n#include <stdbool.h>\nbool heater_external_power_present(void);\nbool heater_power_ready(void);\n",
        "system/esb_ota.h": "#pragma once\n#include <stdbool.h>\nbool esb_ota_is_active(void);\n",
        "connection/connection.h": "#pragma once\n#include <stdbool.h>\nbool connection_get_ota_suppressed(void);\n",
        "util.h": UTIL.replace("IMU_UTIL_PATH", (ROOT / "tests/host/imu_calibration/util.h").as_posix()),
    }
    for name, content in adapters.items():
        destination = work / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(content)
    harness = work / "main.c"
    controller = ROOT / "src/sensor/calibration/tcal_heated.c"
    fixture = (HERE / "test_tcal_heated.c").read_text()
    if main_source is not None:
        fixture = "#define main regression_main\n" + fixture + "\n#undef main\n" + main_source
    harness.write_text(f'#include "{controller.as_posix()}"\n' + accumulator_source() + "\n" + fixture)
    compiler = shlex.split(os.environ.get("CC", "cc"))
    executable = work / "tcal-heated"
    subprocess.run(compiler + [
        "-std=c11", "-O2", "-g", "-Wall", "-Wextra", "-Werror",
        "-Wno-format", "-pthread", "-fsanitize=undefined", "-fno-sanitize-recover=undefined",
        f"-DCONFIG_SENSOR_TCAL_HEATED_DEFAULT_KFF={int(feedforward)}",
        f"-DCONFIG_SENSOR_TCAL_HEATED_DEFAULT_KI={int(integral_gain)}",
        f"-DCONFIG_SENSOR_TCAL_HEATED_TIMEOUT_MIN={int(timeout_min)}",
        "-include", str(work / "globals.h"), "-I", str(work),
        "-I", str(ROOT / "tests/host/imu_calibration"), "-I", str(ROOT / "src"),
        *(["-ffast-math"] if fast_math else []),
        str(harness), str(ROOT / "src/sensor/calibration/imu_calibration.c"),
        "-lm", "-o", str(executable),
    ], check=True)
    return executable


def main():
    with tempfile.TemporaryDirectory(prefix="tcal-heated-") as directory:
        for label, fast_math in (("normal", False), ("fast-math", True)):
            executable = build_harness(Path(directory) / label, fast_math=fast_math)
            for scenario in SCENARIOS:
                subprocess.run([str(executable), scenario], check=True)
            print(f"{label}: {len(SCENARIOS)} controller/IMU scenarios passed", flush=True)
            feedforward_executable = build_harness(
                Path(directory) / f"{label}-feedforward", fast_math=fast_math,
                feedforward=100)
            subprocess.run([str(feedforward_executable), "pi_feedback_nonzero"], check=True)
            proportional_executable = build_harness(
                Path(directory) / f"{label}-proportional", fast_math=fast_math,
                integral_gain=0)
            subprocess.run([str(proportional_executable), "rise_zero_ki"], check=True)


if __name__ == "__main__":
    main()
