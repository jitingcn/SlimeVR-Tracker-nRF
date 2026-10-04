#!/usr/bin/env python3
"""Exercise production T-Cal compensation and D_offset lifecycles with fake hardware."""
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
source = (ROOT / "src/sensor/calibration/tcal_runtime.c").read_text()


def function(name, text=None):
    if text is None:
        text = runtime_functions
    return extract_block(text, rf"^(?:static )?(?:void|int|bool|uint32_t) {re.escape(name)}\([^;{{]*\)\s*\{{")


runtime_header = (ROOT / "src/sensor/calibration/tcal_runtime.h").read_text()
constants = "\n".join(line for line in runtime_header.splitlines()
                      if line.startswith(("#define BOOT_CAL_", "#define RUNTIME_CAL_")))
state = source[source.index("static bool runtime_cal_enabled"):source.index("static void tcal_feedback_finish_locked")]
state = "\n".join(line for line in state.splitlines()
                  if not line.startswith("static int tcal_compensation_storage_error"))
calibration = (ROOT / "src/sensor/calibration/calibration.c").read_text()
reference_state = calibration[calibration.index("static float last_gyro_tcal_offset"):calibration.index("#endif", calibration.index("static float last_gyro_tcal_offset"))]


def active_functions(text):
    """Resolve real C conditionals before counting function braces.

    This lifecycle fixture exercises ordinary T-Cal. Heated admission and
    accumulation have their own production-code harness. Dependency headers
    are supplied by this fixture, so strip includes only for preprocessing.
    """
    without_includes = re.sub(r"^\s*#\s*include[^\n]*$", "", text, flags=re.M)
    command = shlex.split(os.environ.get("CC", "cc")) + [
        "-E", "-P", "-x", "c", "-DCONFIG_SENSOR_USE_TCAL=1",
        "-DCONFIG_SENSOR_TCAL_HEATED=0", "-DCONFIG_CMSIS_DSP=0",
        "-DIS_ENABLED(x)=0", "-",
    ]
    return subprocess.run(command, input=without_includes, text=True,
                          capture_output=True, check=True).stdout


runtime_functions = active_functions(source)
calibration_functions = active_functions(calibration)
preamble = r'''
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>
typedef atomic_long atomic_t;
#define atomic_get(value) atomic_load(value)
#define CONFIG_SENSOR_USE_TCAL 1
#define CONFIG_SENSOR_TCAL_HEATED 0
#include "sensor/calibration/calibration.h"
#include "sensor/calibration/bias_collect.h"
#include "sensor/calibration/tcal_mls_lut.h"
#include "connection/tracker_event_protocol.h"
#define LOG_INF(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define LOG_ERR(...) ((void)0)
#define LOG_DBG(...) ((void)0)
#define TCAL_BUFFER_SIZE 16
static struct {
    struct { bool enabled, completed, doffset_valid; unsigned attempt_count; float doffset[3]; } bootCalState;
    struct { bool valid; unsigned count; } tempCalState;
    struct { float temp; } tempCalPoints[TCAL_BUFFER_SIZE];
    bool tcal_enabled;
    uint8_t fusion_id;
} retained_data;
static typeof(retained_data) *retained = &retained_data;
static int64_t clock_ms = 10000;
static float temperature = 25.0f;
static bool stationary = true;
static int collect_error, lookup_error, request_slot;
static bool curve_sloped;
static float model_shift;
static unsigned lock_depth;
void sensor_tcal_lock(void) { lock_depth++; }
void sensor_tcal_unlock(void) { assert(lock_depth); lock_depth--; }
static bool v_finite(const float *v, unsigned n) {
    for (unsigned i = 0; i < n; ++i) if (!isfinite(v[i])) return false;
    return true;
}
static void sensor_sample_gyro(float *g) { (void)g; }
static void sensor_calibration_gyro_bias(float *g) {
    g[0] = 1.0f; g[1] = -2.0f; g[2] = 0.5f;
}
void sensor_tcal_feed_continuous_sample(const float g[3], float temp) { (void)g; (void)temp; }
void sensor_tcal_model_changed(void) {}
void sensor_tcal_build_lut_priority(float temp) { (void)temp; }
#define TCAL_ENABLED_ID 1
/* Persistent compensation write failure cache (file-scope in tcal_runtime.c,
 * so the extraction needs it declared here). */
static int tcal_compensation_storage_error;
static int sys_write_error;
static unsigned sys_write_count;
static bool durable_tcal_enabled = true;
int sys_write(uint16_t id, void *dest, const void *value, size_t size) {
    assert(id == TCAL_ENABLED_ID && size == sizeof(durable_tcal_enabled));
    memmove(dest, value, size);
    sys_write_count++;
    if (!sys_write_error) memcpy(&durable_tcal_enabled, value, size);
    return sys_write_error;
}
/* Request-generation guard: the automatic boot/runtime path captures the
 * generation at admission and the owner validates it before applying. */
static uint32_t current_generation = 1;
static bool generation_valid_result = true;
void sensor_tcal_feedback_cancel(void);
void sensor_tcal_feedback_applied(uint32_t generation, bool offset_applied);
uint32_t sensor_calibration_current_generation(void) { return current_generation; }
bool sensor_calibration_generation_valid(uint32_t generation) {
    return generation_valid_result && generation == current_generation;
}
void sensor_calibration_invalidate_requests(void) {
    current_generation++;
    sensor_tcal_feedback_cancel();
}
void sensor_calibration_invalidate_kind(int kind) { (void)kind; }
/* Warm transaction leaf: the guarded write must leave the counter balanced. */
static int warm_tx;
void sys_warm_transaction_begin(void) { assert(warm_tx >= 0); warm_tx++; }
void sys_warm_transaction_mark(uint16_t id, const void *data, size_t size) {
    (void)id; (void)data; (void)size; assert(warm_tx > 0);
}
void sys_warm_transaction_end(bool schedule) { (void)schedule; assert(warm_tx > 0); warm_tx--; }
int sys_flush_warm(void) { return 0; }
/* Semantic LED leaves; the automatic owner submits facts, never rendering. */
static unsigned led_results[LED_SEMANTIC_COUNT];
static unsigned led_requests[LED_SEMANTIC_COUNT];
static unsigned led_states[LED_SEMANTIC_COUNT];
static struct led_token last_led_token;
static uint32_t led_identity;
uint32_t led_request_id(void) { return ++led_identity; }
uint32_t led_event_id(void) { return ++led_identity; }
struct led_token led_begin(enum led_owner owner, uint32_t request_id) {
    return (struct led_token){owner, ++led_identity, request_id};
}
enum led_admission led_state(struct led_token token, uint32_t revision, enum led_semantic semantic) {
    (void)revision;
    if (token.session) led_states[semantic]++;
    return LED_ADMITTED;
}
enum led_admission led_result(struct led_token token, uint32_t event_id,
                              enum led_semantic semantic) {
    (void)event_id;
    if (token.session) {
        led_results[semantic]++;
        last_led_token = token;
    }
    return LED_ADMITTED;
}
enum led_admission led_request_event(enum led_owner owner, uint32_t request_id, uint32_t event_id,
                                     enum led_semantic semantic) {
    (void)owner; (void)request_id; (void)event_id;
    led_requests[semantic]++;
    return LED_ADMITTED;
}
void led_identify(void) {}
void led_connection_publish(const struct led_connection_facts *facts) { (void)facts; }
void led_power_publish(enum led_power_state state, bool low) { (void)state; (void)low; }
void led_fault_publish(enum led_owner owner, enum led_fault_kind fault, uint32_t protection) {
    (void)owner; (void)fault; (void)protection;
}
void led_maintenance_publish(enum led_owner owner, bool active) { (void)owner; (void)active; }
void led_operation_publish(enum led_owner owner, bool ota_active, bool heated_active) {
    (void)owner; (void)ota_active; (void)heated_active;
}
void led_quiesce(void) {}
void led_shutdown(void) {}
static uint16_t current_operation = 41;
static struct led_token user_feedback;
struct led_token sensor_calibration_current_feedback(void) {
    return user_feedback;
}
int sensor_calibration_current_storage_error(void) { return 0; }
static unsigned event_count, terminal_count, begin_count, request_count;
static uint8_t last_outcome, last_phase, last_reason, last_kind;
static int last_request;
static enum cal_request_origin last_origin;
static int64_t k_uptime_get(void) { return clock_ms; }
static int64_t system_uptime_since_boot_ms(void) { return clock_ms; }
static void k_msleep(int ms) { clock_ms += ms; }
static float sensor_get_current_imu_temperature(void) { return temperature; }
bool wait_for_motion(bool motion, int count) { (void)motion; (void)count; return stationary; }
void sensor_request_fusion_bias_reset(void) {}
uint16_t sensor_calibration_current_operation(void) { return current_operation; }
int sensor_calibration_request(int id, enum cal_request_origin origin) {
    if (id == CAL_REQUEST_QUERY) return request_slot;
    request_count++;
    last_request = id;
    last_origin = origin;
    if (request_slot) return -1;
    request_slot = id;
    return 0;
}
int sensor_tcal_mls_lookup(float temp, float bias[3]) {
    (void)temp;
    bias[0] = 0.1f; bias[1] = 0.2f; bias[2] = 0.3f;
    if (curve_sloped) for (int i = 0; i < 3; ++i) bias[i] += (temp - 25.0f) * (i + 1) + model_shift;
    return lookup_error;
}
int sensor_tcal_lut_lookup(float temp, float bias[3]) { (void)temp; (void)bias; return -1; }
static bool disable_during_collect;
static bool invalidate_during_collect;
static int sensor_boot_bias_collect(float *bias, float *temp) {
    bias[0] = 0.2f; bias[1] = 0.1f; bias[2] = 0.5f;
    *temp = temperature;
    if (disable_during_collect) sensor_tcal_set_enabled(false);
    /* Stale-epoch scenario: reset-all invalidation lands inside the collect leaf,
     * before the production guard reads the captured generation. */
    if (invalidate_during_collect) sensor_calibration_invalidate_requests();
    return collect_error;
}
static int sensor_runtime_bias_collect(float *bias, float *temp) {
    return sensor_boot_bias_collect(bias, temp);
}
uint16_t cal_event_begin(uint8_t kind, uint8_t phase, uint8_t detail) {
    assert(kind == (CAL_KIND_TCAL_BOOT | CAL_EVENT_ORIGIN_AUTO));
    assert(phase == CAL_PHASE_WAIT_STILL && detail == 0);
    begin_count++; event_count++; last_kind = kind;
    return current_operation;
}
void cal_event_step(uint16_t op, uint8_t phase, uint8_t detail) {
    (void)phase; (void)detail;
    assert(op == current_operation);
    if (op) event_count++;
}
void cal_event_end(uint16_t op, uint8_t outcome, uint8_t phase, uint8_t reason) {
    assert(op == current_operation);
    if (!op) return;
    /* Observe live validity at the event boundary, not just after the call. */
    if (outcome == CAL_OUTCOME_SUCCESS) assert(retained->bootCalState.doffset_valid);
    event_count++; terminal_count++;
    last_outcome = outcome; last_phase = phase; last_reason = reason;
}
void tracker_events_notify(void) {}
'''
main = r'''
static void quality_points(void) {
    retained->tempCalState.count = 5;
    retained->tempCalState.valid = true;
    for (unsigned i = 0; i < 5; ++i) retained->tempCalPoints[i].temp = 23.0f + i;
}
static void expect_terminal(uint8_t outcome, uint8_t phase, uint8_t reason) {
    assert(terminal_count == 1);
    assert(last_outcome == outcome && last_phase == phase && last_reason == reason);
}
static void expect_vector(const float actual[3], const float expected[3]) {
    for (int i = 0; i < 3; ++i) assert(fabsf(actual[i] - expected[i]) < 0.00001f);
}
static bool sample(float g[3], float delta[3]) {
    g[0] = 10; g[1] = 20; g[2] = 30;
    delta[0] = delta[1] = delta[2] = 999;
    bool reset = sensor_calibration_process_gyro(g, delta);
    assert(!sensor_calibration_gyro_reference_pending());
    assert(lock_depth == 0);
    return reset;
}
static void compensation(const char *scenario) {
    const float zero[3] = {0};
    float before[3], after[3], delta[3], expected[3];
    sensor_tcal_refresh_apply_cache();
    assert(sensor_calibration_gyro_reference_pending());
    assert(!sample(before, delta));
    expect_vector(delta, zero);
    if (!strcmp(scenario, "toggle_continuity")) {
        float residual[3] = {0.7f, -0.2f, 0.4f};
        float original[3]; memcpy(original, residual, sizeof(original));
        for (int enabled = 0; enabled < 2; ++enabled) {
            sensor_tcal_set_enabled(enabled);
            assert(sensor_calibration_gyro_reference_pending());
            assert(!sample(after, delta));
            for (int i = 0; i < 3; ++i) {
                assert(fabsf((after[i] - residual[i] - delta[i]) - (before[i] - residual[i])) < 0.00001f);
                residual[i] += delta[i];
            }
            memcpy(before, after, sizeof(before));
            sensor_tcal_set_enabled(enabled);
            assert(!sample(after, delta));
            expect_vector(delta, zero);
        }
        expect_vector(residual, original);
    } else if (!strcmp(scenario, "natural_temperature")) {
        curve_sloped = true;
        temperature += 1;
        assert(!sensor_calibration_gyro_reference_pending());
        assert(!sample(after, delta));
        expect_vector(delta, zero);
        for (int i = 0; i < 3; ++i) expected[i] = before[i] - (i + 1);
        expect_vector(after, expected);
    } else if (!strcmp(scenario, "disabled_model")) {
        sensor_tcal_set_enabled(false);
        assert(!sample(before, delta));
        curve_sloped = true; model_shift = 5;
        retained->fusion_id = 1;
        sensor_tcal_refresh_model();
        assert(!sensor_calibration_gyro_reference_pending());
        assert(retained->fusion_id == 1);
        assert(!sample(after, delta));
        expect_vector(after, before);
        expect_vector(delta, zero);
    } else if (!strcmp(scenario, "write_failure_repeat")) {
        /* Applying RAM does not make a failed flash write durable. Even after
         * storage recovers, identical confirmation must report the old failure,
         * not silently retry. A successful opposite change resets that receipt. */
        assert(sensor_tcal_get_enabled() && retained->tcal_enabled && durable_tcal_enabled);
        sys_write_error = -EIO;
        unsigned before_first = sys_write_count;
        assert(sensor_tcal_set_enabled(false) == -EIO);
        assert(sys_write_count == before_first + 1);
        assert(!sensor_tcal_get_enabled() && !retained->tcal_enabled && durable_tcal_enabled);
        assert(warm_tx == 0 && lock_depth == 0);
        assert(!sample(after, delta));
        const float fallback[3] = {9, 22, 29.5f};
        expect_vector(after, fallback);
        assert(led_requests[LED_APPLIED_NOT_SAVED] == 1 && led_requests[LED_SUCCESS] == 0);
        sys_write_error = 0;
        assert(sensor_tcal_set_enabled(false) == -EIO);
        assert(sys_write_count == before_first + 1 && durable_tcal_enabled);
        assert(!sensor_tcal_get_enabled() && !retained->tcal_enabled);
        assert(led_requests[LED_APPLIED_NOT_SAVED] == 2 && led_requests[LED_SUCCESS] == 0);
        assert(sensor_tcal_set_enabled(true) == 0);
        assert(sys_write_count == before_first + 2);
        assert(sensor_tcal_get_enabled() && retained->tcal_enabled && durable_tcal_enabled);
        assert(led_requests[LED_APPLIED_NOT_SAVED] == 2 && led_requests[LED_SUCCESS] == 1);
        /* Cached durable success remains true without another storage attempt. */
        sys_write_error = -ENOSPC;
        assert(sensor_tcal_set_enabled(true) == 0);
        assert(sys_write_count == before_first + 2 && durable_tcal_enabled);
        assert(led_requests[LED_APPLIED_NOT_SAVED] == 2 && led_requests[LED_SUCCESS] == 2);
        assert(led_requests[LED_PARTIAL] == 0 && led_results[LED_SUCCESS] == 0);
        assert(warm_tx == 0 && lock_depth == 0);
    } else if (!strcmp(scenario, "initial_reset")) {
        sensor_calibration_reset_gyro_reference();
        assert(sensor_calibration_gyro_reference_pending());
        sensor_tcal_set_enabled(false);
        assert(!sample(after, delta));
        expect_vector(delta, zero);
    } else {
        const float measured[3] = {0.6f, -0.4f, 0.8f};
        retained->fusion_id = 1;
        assert(sensor_tcal_calculate_doffset(measured, temperature, current_operation,
            (struct led_token){0}, current_generation) == 0);
        assert(retained->fusion_id == 0);
        assert(sensor_calibration_gyro_reference_pending());
        if (!strcmp(scenario, "physical_once") || !strcmp(scenario, "fallback_no_d")) {
            assert(sample(after, delta));
            for (int i = 0; i < 3; ++i) expected[i] = after[i] - before[i];
            expect_vector(delta, expected);
            for (int i = 0; i < 3; ++i) expected[i] = (i + 1) * 10 - measured[i];
            expect_vector(after, expected);
            assert(!sample(after, delta));
            expect_vector(delta, zero);
            if (!strcmp(scenario, "physical_once")) {
                memcpy(before, after, sizeof(before));
                sensor_tcal_set_enabled(false);
                assert(!sample(after, delta));
                sensor_tcal_set_enabled(true);
                assert(!sample(after, delta));
                expect_vector(after, before);
                retained->fusion_id = 1;
                sensor_tcal_clear_doffset();
                assert(retained->fusion_id == 0);
                assert(sensor_calibration_gyro_reference_pending());
                assert(!sample(after, delta));
                for (int i = 0; i < 3; ++i) expected[i] = after[i] - before[i];
                expect_vector(delta, expected);
            }
            if (!strcmp(scenario, "fallback_no_d")) {
                lookup_error = -1;
                assert(!sample(after, delta));
                const float fallback[3] = {9, 22, 29.5f};
                expect_vector(after, fallback);
                expect_vector(delta, zero);
            }
        } else if (!strcmp(scenario, "model_invalidates_d") || !strcmp(scenario, "model_replaces_applied_d")) {
            if (!strcmp(scenario, "model_replaces_applied_d")) assert(sample(before, delta));
            curve_sloped = true; model_shift = 2;
            sensor_tcal_refresh_model();
            assert(!retained->bootCalState.doffset_valid);
            assert(!sample(after, delta));
            for (int i = 0; i < 3; ++i) expected[i] = (i + 1) * 10 - (0.1f * (i + 1) + 2) - before[i];
            expect_vector(delta, expected);
            for (int i = 0; i < 3; ++i) expected[i] = (i + 1) * 10 - (0.1f * (i + 1) + 2);
            expect_vector(after, expected);
        } else if (!strcmp(scenario, "toggle_cancels_physical")) {
            sensor_tcal_set_enabled(false);
            assert(!sample(after, delta));
            sensor_tcal_set_enabled(true);
            assert(!sample(after, delta));
            expect_vector(after, before);
        } else {
            assert(!"unknown compensation scenario");
        }
    }
}
int main(int argc, char **argv) {
    assert(argc == 2);
    const char *scenario = argv[1];
    bool user = !strncmp(scenario, "user_", 5);
    if (user) {
        scenario += 5;
        user_feedback = led_begin(LED_OWNER_TCAL, led_request_id());
    }
    retained->bootCalState.enabled = true;
    retained->tcal_enabled = true;
    quality_points();
    if (!strncmp(scenario, "comp_", 5)) {
        compensation(scenario + 5);
        printf("tcal compensation: %s passed\n", scenario);
        return 0;
    }
    if (!strcmp(scenario, "boot_disabled")) {
        sensor_tcal_set_enabled(false);
        sensor_tcal_boot_calibration_check();
        assert(request_count == 0 && event_count == 0);
        assert(sensor_perform_boot_calibration() == 0);
        assert(!retained->bootCalState.doffset_valid);
        expect_terminal(CAL_OUTCOME_SKIPPED, CAL_PHASE_WAIT_STILL, CAL_REASON_DISABLED);
    } else if (!strcmp(scenario, "boot_disabled_during_collect")) {
        disable_during_collect = true;
        assert(sensor_perform_boot_calibration() == 0);
        assert(!sensor_tcal_get_enabled() && !retained->bootCalState.doffset_valid);
        expect_terminal(CAL_OUTCOME_SKIPPED, CAL_PHASE_VALIDATE, CAL_REASON_DISABLED);
    } else if (!strcmp(scenario, "boot_skip") || !strcmp(scenario, "runtime_skip")) {
        retained->tempCalState.count = 4;
        retained->tempCalPoints[4].temp = 0;
        int result = !strcmp(scenario, "boot_skip") ? sensor_perform_boot_calibration() : sensor_perform_runtime_calibration();
        assert(result == 0 && !retained->bootCalState.doffset_valid);
        if (user) assert(led_results[LED_PARTIAL] == 1);
        expect_terminal(CAL_OUTCOME_SKIPPED, CAL_PHASE_VALIDATE, CAL_REASON_NO_TCAL_COVERAGE);
    } else if (!strcmp(scenario, "boot_applied") || !strcmp(scenario, "runtime_applied")) {
        int result = !strcmp(scenario, "boot_applied") ? sensor_perform_boot_calibration() : sensor_perform_runtime_calibration();
        assert(result == 0);
        assert(fabsf(retained->bootCalState.doffset[0] - 0.1f) < 0.00001f);
        assert(fabsf(retained->bootCalState.doffset[1] + 0.1f) < 0.00001f);
        assert(fabsf(retained->bootCalState.doffset[2] - 0.2f) < 0.00001f);
        expect_terminal(CAL_OUTCOME_SUCCESS, CAL_PHASE_APPLIED, CAL_REASON_NONE);
        /* A request CLEAR must not steal the immutable receipt; computation
         * alone is not local success until a gyro actually uses the offset. */
        struct led_token accepted = user_feedback;
        user_feedback = (struct led_token){0};
        current_operation = 0;
        assert(led_results[LED_SUCCESS] == 0);
        if (user) {
            assert(led_states[LED_COLLECT_STILL] == 1 && led_states[LED_PROCESSING] == 1);
            assert(led_results[LED_STAGE_ACK] == 1);
        }
        sensor_tcal_refresh_apply_cache();
        float g[3], delta[3];
        assert(sample(g, delta));
        expect_vector(g, (float[]){9.8f, 19.9f, 29.5f});
        if (user) {
            assert(led_results[LED_SUCCESS] == 1);
            assert(last_led_token.session == accepted.session);
            assert(!sample(g, delta) && led_results[LED_SUCCESS] == 1);
        }
    } else if (!strcmp(scenario, "boot_stale_generation") || !strcmp(scenario, "runtime_stale_generation")) {
        retained->bootCalState.doffset_valid = true;
        retained->bootCalState.doffset[0] = 4;
        retained->bootCalState.doffset[1] = -2;
        retained->bootCalState.doffset[2] = 1;
        float before[3];
        memcpy(before, retained->bootCalState.doffset, sizeof(before));
        invalidate_during_collect = true;
        int result = !strcmp(scenario, "boot_stale_generation")
            ? sensor_perform_boot_calibration() : sensor_perform_runtime_calibration();
        assert(result == -ECANCELED && warm_tx == 0);
        assert(retained->bootCalState.doffset_valid);
        assert(!memcmp(before, retained->bootCalState.doffset, sizeof(before)));
        expect_terminal(CAL_OUTCOME_CANCELLED, CAL_PHASE_APPLY_PENDING, CAL_REASON_RESET);
        assert(led_results[LED_SUCCESS] == 0 && led_requests[LED_SUCCESS] == 0);
    } else if (!strcmp(scenario, "fit_error")) {
        lookup_error = -1;
        retained->bootCalState.doffset_valid = true;
        assert(sensor_perform_boot_calibration() != 0);
        assert(retained->bootCalState.doffset_valid);
        expect_terminal(CAL_OUTCOME_FAILED, CAL_PHASE_VALIDATE, CAL_REASON_FIT_ERROR);
    } else if (!strcmp(scenario, "motion")) {
        stationary = false;
        assert(sensor_perform_boot_calibration() != 0);
        expect_terminal(CAL_OUTCOME_FAILED, CAL_PHASE_WAIT_STILL, CAL_REASON_MOTION);
    } else if (!strcmp(scenario, "temperature")) {
        temperature = NAN;
        assert(sensor_perform_runtime_calibration() != 0);
        expect_terminal(CAL_OUTCOME_FAILED, CAL_PHASE_WAIT_STILL, CAL_REASON_TEMPERATURE);
    } else if (!strcmp(scenario, "collection_reasons")) {
        const int errors[] = {-2, BIAS_COLLECT_INSUFFICIENT_SAMPLES};
        const uint8_t reasons[] = {CAL_REASON_SAMPLE_TIMEOUT, CAL_REASON_INSUFFICIENT_SAMPLES};
        for (unsigned i = 0; i < 2; ++i) {
            collect_error = errors[i];
            terminal_count = 0;
            unsigned failures = led_results[LED_FAILED];
            assert(sensor_perform_boot_calibration() == errors[i]);
            expect_terminal(CAL_OUTCOME_FAILED, CAL_PHASE_COLLECT, reasons[i]);
            if (user) assert(led_results[LED_FAILED] == failures + 1);
            terminal_count = 0;
            assert(sensor_perform_runtime_calibration() == errors[i]);
            expect_terminal(CAL_OUTCOME_FAILED, CAL_PHASE_COLLECT, reasons[i]);
            if (user) assert(led_results[LED_FAILED] == failures + 2);
        }
    } else if (!strcmp(scenario, "fallback_busy")) {
        retained->tempCalState.count = 0;
        retained->bootCalState.attempt_count = BOOT_CAL_MAX_ATTEMPTS - 1;
        request_slot = CAL_REQUEST_TCAL_BOOT;
        collect_error = -1;
        assert(sensor_perform_boot_calibration() == collect_error);
        assert(last_request == CAL_REQUEST_IMU && last_origin == CAL_REQUEST_AUTO_SILENT);
        assert(request_slot == CAL_REQUEST_TCAL_BOOT && retained->bootCalState.completed);
        expect_terminal(CAL_OUTCOME_FAILED, CAL_PHASE_COLLECT, CAL_REASON_MOTION);
    } else if (!strcmp(scenario, "gate_coverage") || !strcmp(scenario, "gate_expired")) {
        bool expired = !strcmp(scenario, "gate_expired");
        if (expired) clock_ms = BOOT_CAL_TIME_WINDOW_END_MS;
        else { retained->tempCalState.count = 0; memset(retained->tempCalPoints, 0, sizeof(retained->tempCalPoints)); }
        for (unsigned i = 0; i < 1000; ++i) sensor_tcal_boot_calibration_check();
        assert(retained->bootCalState.completed && begin_count == 1);
        assert(last_kind == (CAL_KIND_TCAL_BOOT | CAL_EVENT_ORIGIN_AUTO));
        expect_terminal(CAL_OUTCOME_SKIPPED, CAL_PHASE_WAIT_STILL, expired ? CAL_REASON_EXPIRED : CAL_REASON_NO_TCAL_COVERAGE);
    } else if (!strcmp(scenario, "gate_active")) {
        request_slot = CAL_REQUEST_TCAL_BOOT;
        clock_ms = BOOT_CAL_TIME_WINDOW_END_MS;
        sensor_tcal_boot_calibration_check();
        assert(event_count == 0);
        assert(sensor_perform_boot_calibration() == 0);
        expect_terminal(CAL_OUTCOME_SUCCESS, CAL_PHASE_APPLIED, CAL_REASON_NONE);
    } else if (!strcmp(scenario, "auto_busy")) {
        request_slot = CAL_REQUEST_IMU;
        for (unsigned i = 0; i < 1000; ++i) sensor_tcal_boot_calibration_check();
        assert(!retained->bootCalState.completed && event_count == 0);
        request_slot = 0;
        sensor_tcal_boot_calibration_check();
        assert(last_request == CAL_REQUEST_TCAL_BOOT && last_origin == CAL_REQUEST_AUTO);
    } else if (!strcmp(scenario, "supplement_busy")) {
        retained->tempCalState.count = 0;
        tcal_auto_calibration_enabled = true;
        clock_ms = 100000;
        request_slot = CAL_REQUEST_IMU;
        for (unsigned i = 0; i < 1000; ++i) sensor_tcal_check_auto_calibration(temperature);
        assert(event_count == 0 && request_count == 1000);
        request_slot = 0;
        sensor_tcal_check_auto_calibration(temperature);
        assert(request_slot == CAL_REQUEST_IMU && last_origin == CAL_REQUEST_AUTO_SILENT);
    } else if (!strcmp(scenario, "runtime_disabled")) {
        retained->bootCalState.completed = true;
        for (unsigned i = 0; i < 1000; ++i) { clock_ms += 1000; sensor_runtime_calibration_check(true); }
        assert(request_count == 0 && event_count == 0);
    } else if (!strcmp(scenario, "apply_fallback")) {
        assert(user && sensor_perform_runtime_calibration() == 0);
        assert(led_results[LED_SUCCESS] == 0);
        sensor_tcal_refresh_apply_cache();
        lookup_error = -1;
        float g[3], delta[3];
        assert(sample(g, delta));
        expect_vector(g, (float[]){9.0f, 22.0f, 29.5f});
        assert(led_results[LED_PARTIAL] == 1);
        assert(led_results[LED_SUCCESS] == 0);
        lookup_error = 0;
        assert(!sample(g, delta));
        assert(led_results[LED_SUCCESS] == 0 && led_results[LED_PARTIAL] == 1);
    } else if (!strcmp(scenario, "pending_reset") || !strcmp(scenario, "pending_model") ||
               !strcmp(scenario, "pending_replaced") || !strcmp(scenario, "pending_auto")) {
        assert(user && sensor_perform_runtime_calibration() == 0);
        struct led_token first = user_feedback;
        assert(led_results[LED_SUCCESS] == 0);
        user_feedback = (struct led_token){0};
        if (!strcmp(scenario, "pending_model")) {
            sensor_tcal_refresh_model();
        } else if (!strcmp(scenario, "pending_reset")) {
            sensor_calibration_invalidate_requests();
        } else {
            /* A newly admitted request replaces the collector generation,
             * but its LED token must never be completed by the old receipt. */
            current_generation++;
        }
        if (!strcmp(scenario, "pending_replaced")) {
            user_feedback = led_begin(LED_OWNER_TCAL, led_request_id());
            terminal_count = 0;
            assert(sensor_perform_boot_calibration() == 0);
        } else if (!strcmp(scenario, "pending_auto")) {
            current_operation = 0; /* AUTO_SILENT has neither telemetry nor LED token. */
            assert(sensor_perform_runtime_calibration() == 0);
        }
        assert(led_results[LED_CANCELLED] == 1);
        assert(last_led_token.session == first.session);
        sensor_tcal_refresh_apply_cache();
        float g[3], delta[3];
        sample(g, delta);
        if (!strcmp(scenario, "pending_replaced")) {
            assert(led_results[LED_SUCCESS] == 1);
            assert(last_led_token.session == user_feedback.session &&
                   last_led_token.session != first.session);
        } else {
            assert(led_results[LED_SUCCESS] == 0);
        }
        assert(led_results[LED_CANCELLED] == 1);
    } else if (!strcmp(scenario, "silent_applied")) {
        assert(!user);
        current_operation = 0;
        assert(sensor_perform_runtime_calibration() == 0);
        sensor_tcal_refresh_apply_cache();
        float g[3], delta[3];
        assert(sample(g, delta));
        expect_vector(g, (float[]){9.8f, 19.9f, 29.5f});
        assert(event_count == 0);
    } else {
        assert(!"unknown scenario");
    }
    if (user && last_outcome == CAL_OUTCOME_SKIPPED) {
        assert(led_results[LED_PARTIAL] == 1 && led_results[LED_SUCCESS] == 0);
        unsigned partials = led_results[LED_PARTIAL];
        sensor_tcal_refresh_apply_cache();
        float g[3], delta[3];
        sample(g, delta);
        assert(led_results[LED_SUCCESS] == 0 && led_results[LED_PARTIAL] == partials);
    } else if (user && last_outcome == CAL_OUTCOME_FAILED) {
        unsigned failures = !strcmp(scenario, "collection_reasons") ? 4 : 1;
        assert(led_results[LED_FAILED] == failures && led_results[LED_SUCCESS] == 0);
    } else if (user && last_outcome == CAL_OUTCOME_CANCELLED) {
        assert(led_results[LED_CANCELLED] == 1 && led_results[LED_SUCCESS] == 0);
    }
    if (!user) {
        for (unsigned i = 0; i < LED_SEMANTIC_COUNT; ++i) {
            assert(led_results[i] == 0 && led_states[i] == 0);
        }
    }
    printf("tcal lifecycle: %s passed\n", scenario);
    return 0;
}
'''
parts = [preamble, constants, state, reference_state]
parts.extend(function(name) for name in (
    "tcal_feedback_finish_locked", "sensor_tcal_feedback_cancel",
    "sensor_tcal_feedback_applied", "tcal_result",
    "sensor_tcal_mark_measured_bias",
    "sensor_tcal_reference_generation", "sensor_tcal_take_bias_reset",
    "sensor_tcal_clear_doffset", "sensor_tcal_refresh_apply_cache",
    "sensor_tcal_refresh_model", "sensor_tcal_curve_apply_ready",
    "sensor_tcal_get_auto_calibration", "sensor_tcal_get_enabled", "sensor_tcal_set_enabled",
))
parts.extend(function(name, calibration_functions) for name in (
    "sensor_calibration_process_gyro", "sensor_calibration_reset_gyro_reference",
    "sensor_calibration_gyro_reference_pending",
))
parts.extend(function(name) for name in (
    "sensor_tcal_assess_quality", "sensor_tcal_calculate_doffset_locked",
    "sensor_tcal_calculate_doffset", "sensor_boot_cal_abandon",
    "sensor_tcal_boot_calibration_check", "sensor_perform_boot_calibration",
    "sensor_perform_runtime_calibration", "sensor_runtime_calibration_check",
    "sensor_tcal_check_auto_calibration",
))
scenarios = (
    "boot_skip", "runtime_skip", "boot_applied", "runtime_applied", "fit_error", "motion",
    "temperature", "fallback_busy", "gate_coverage", "gate_expired", "gate_active",
    "auto_busy", "supplement_busy", "runtime_disabled", "collection_reasons",
    "boot_disabled", "boot_disabled_during_collect",
    "boot_stale_generation", "runtime_stale_generation",
    "comp_toggle_continuity", "comp_natural_temperature", "comp_disabled_model",
    "comp_initial_reset", "comp_physical_once", "comp_fallback_no_d",
    "comp_model_invalidates_d", "comp_toggle_cancels_physical",
    "comp_model_replaces_applied_d",
    "comp_write_failure_repeat",
    "user_boot_applied", "user_runtime_applied", "user_boot_skip", "user_runtime_skip",
    "user_boot_disabled", "user_boot_disabled_during_collect", "user_fit_error",
    "user_motion", "user_temperature", "user_collection_reasons",
    "user_boot_stale_generation", "user_runtime_stale_generation",
    "user_apply_fallback", "user_pending_reset", "user_pending_model",
    "user_pending_replaced", "user_pending_auto", "silent_applied",
)
with tempfile.TemporaryDirectory(prefix="tcal-events-") as directory:
    tmp = Path(directory)
    unit = tmp / "tcal.c"
    unit.write_text("\n\n".join(parts) + main)
    binary = tmp / "tcal"
    subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
        "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-variable",
        "-g", "-O1", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
        "-fno-pie", "-no-pie", "-I", str(ROOT / "src"), str(unit), "-lm", "-o", str(binary),
    ], check=True)
    for scenario in scenarios:
        subprocess.run([str(binary), scenario], check=True)
