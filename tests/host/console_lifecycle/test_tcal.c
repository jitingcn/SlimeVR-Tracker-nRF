#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define USB_EXISTS 0
#define UART_CONSOLE_EXISTS 0
#define CONFIG_LED_DEBUG 0
#define CONFIG_SENSOR_USE_MAGNETOMETER 0
#define CONFIG_SENSOR_POLY_STEPS_PER_DEGREE 2
#define CONFIG_SENSOR_TCAL_HEATED_DEFAULT_TARGET_C 42.0f
#define SENS_CAL_DEFAULT_REVOLUTIONS 5
#define SENS_CAL_MAX_REVOLUTIONS 100
#define TCAL_BUFFER_SIZE 2
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define ARG_UNUSED(x) (void)(x)
#define LED_OWNER_SYSTEM 0
#define LED_REJECTED 1
#define LED_OWNER_SENSOR 1
#define LED_ACCEPTED 2
#define LED_SUCCESS 3
#define FW_STRING "test"
#define FW_GIT_REPO_URL "test"
#define FW_GIT_BRANCH "test"

static struct {
    struct { int count; } tempCalState;
    struct { float temp, bias[3]; } tempCalPoints[TCAL_BUFFER_SIZE];
    struct { bool enabled, completed, doffset_valid; unsigned attempt_count; float doffset[3]; } bootCalState;
} retained_storage, *retained = &retained_storage;
static unsigned rejects, auto_writes, remove_calls, test_calls, status_calls;
static unsigned sens_writes, sens_resets, sens_auto_calls, backup_calls, input_losses;
static uint8_t sens_axis;
static uint16_t sens_revolutions;
static unsigned ping_calls, ping_stats, tdma_writes, tdma_stats, heat_reports;
static unsigned dfu_calls, bench_calls, bench_switches, bench_accepted, bench_success;
static uint32_t bench_iterations;
static bool dfu_ota, bench_active;
static int removed_index;
static bool tcal_enabled, auto_enabled, tdma_enabled;
#if CONFIG_SENSOR_USE_TCAL
static uint32_t console_command_input_generation;
#endif
static bool console_feedback_enabled;
static const char *input = "";
static jmp_buf worker_idle;
#define printk printf
static unsigned led_request_id(void) { return 1; }
static unsigned led_event_id(void) { return 1; }
static void led_request_event(int owner, unsigned request, unsigned event, int semantic)
{
    (void)request; (void)event;
    if (semantic == LED_REJECTED) rejects++;
    if (owner == LED_OWNER_SENSOR && semantic == LED_ACCEPTED) bench_accepted++;
    if (owner == LED_OWNER_SENSOR && semantic == LED_SUCCESS) bench_success++;
}
static void console_unrelated_command(size_t argc, char **argv) { (void)argc; (void)argv; }
static void console_reset_cancel(void) {}
static void strtolower(char *text)
{
    for (; *text; text++) *text = (char)tolower((unsigned char)*text);
}
static int SEGGER_RTT_HasKey(void) { return *input != '\0'; }
static int SEGGER_RTT_GetKey(void) { return *input ? (unsigned char)*input++ : -1; }
static void k_usleep(int delay) { (void)delay; longjmp(worker_idle, 1); }
static void k_msleep(int delay) { (void)delay; }
static bool sensor_tcal_backup_active(void) { return false; }
static void sensor_tcal_backup_process(void) {}
static int sensor_tcal_backup_input_byte(uint8_t byte) { (void)byte; return 0; }
static void sensor_tcal_backup_input_lost(void) { input_losses++; }
static uint32_t sensor_tcal_backup_input_generation(void) { return 7; }
static void sensor_tcal_backup_command(size_t argc, char **argv, uint32_t generation)
{
    (void)argv;
    assert(argc >= 2 && generation == 7);
    backup_calls++;
}
static void sensor_tcal_set_enabled(bool value) { tcal_enabled = value; }
static bool sensor_tcal_get_enabled(void) { return tcal_enabled; }
static void sensor_tcal_status(void) { status_calls++; }
static const char *sensor_tcal_get_apply_mode_name(void) { return "test"; }
static bool sensor_tcal_get_auto_calibration(void) { return auto_enabled; }
static void sensor_tcal_set_auto_calibration(bool value) { auto_enabled = value; auto_writes++; }
static void cmd_reset_tcal(void) {}
static void sensor_tcal_remove_point(int index) { removed_index = index; remove_calls++; }
static float sensor_get_current_imu_temperature(void) { return 25; }
static void sensor_tcal_test_methods(float temperature) { (void)temperature; test_calls++; }
static bool sensor_tcal_needs_nearby_point(float temperature, float *closest, float *distance)
{
    (void)temperature; *closest = 25; *distance = 0; return false;
}
static void sensor_boot_cal_set_enabled(bool value) { retained->bootCalState.enabled = value; }
static bool v_finite(const float *value, size_t count)
{
    for (size_t i = 0; i < count; i++) if (!isfinite(value[i])) return false;
    return true;
}
static int sensor_tcal_heated_start(float target) { (void)target; return 0; }
static int sensor_tcal_heated_stop(void) { return 0; }
static void sensor_tcal_heated_report(void) { heat_reports++; }
static void cmd_sens_set(float x, float y, float z) { (void)x; (void)y; (void)z; sens_writes++; }
static void cmd_sens_reset(void) { sens_resets++; }
static int sensor_request_calibration_sens(uint8_t axis, uint16_t revolutions)
{
    sens_axis = axis;
    sens_revolutions = revolutions;
    sens_auto_calls++;
    return 0;
}
static void connection_print_ping_stats(void) { ping_stats++; }
static void cmd_ping_start(void) { ping_calls++; }
static int tdma_user_set_enabled(bool enabled) { tdma_enabled = enabled; tdma_writes++; return 0; }
static bool tdma_is_enabled(void) { return tdma_enabled; }
static int radio_capture_user_set_enabled(bool enabled) { (void)enabled; return 0; }
static bool radio_capture_is_enabled(void) { return false; }
static void radio_capture_print_stats(void) {}
static void tdma_print_stats(void) { tdma_stats++; }
static void sys_enter_dfu(bool ota) { dfu_calls++; dfu_ota = ota; }
static void sensor_benchmark_active(bool enabled) { bench_active = enabled; bench_switches++; }
static void vqf_run_benchmark(uint32_t iterations) { bench_iterations = iterations; bench_calls++; }

#include "system/rtt_console.h"
#include "production_tcal.inc"

static void command(const char *text)
{
    char line[128];
    assert(snprintf(line, sizeof(line), "%s\n", text) > 0);
    input = line;
    if (setjmp(worker_idle) == 0) console_thread();
    assert(*input == '\0');
}

int main(void)
{
#if CONFIG_SENSOR_USE_TCAL
    command("tcal");
    assert(rejects == 1 && auto_writes == 0);
    command("tcal auto");
    assert(rejects == 2 && auto_writes == 0);
    command("tcal auto on");
    assert(auto_enabled && auto_writes == 1);
    command("tcal auto off");
    assert(!auto_enabled && auto_writes == 2);
    command("tcal auto maybe");
    command("tcal auto on extra");
    assert(rejects == 4 && auto_writes == 2);
    command("tcal remove");
    command("tcal remove 2junk");
    assert(rejects == 6 && remove_calls == 0);
    command("tcal remove 2");
    assert(remove_calls == 1 && removed_index == 2);
    command("tcal remove 1 extra");
    assert(rejects == 7 && remove_calls == 1);
    command("tcal status");
    assert(status_calls == 1);
    command("tcal status extra");
    command("tcal dump extra");
    command("tcal check extra");
    command("tcal test 25 extra");
    assert(rejects == 7 && status_calls == 1 && test_calls == 0);
    command("tcal test");
    assert(test_calls == 1);
    command("tcal boot");
    command("tcal boot on");
    assert(retained->bootCalState.enabled && rejects == 7);
    command("tcal boot off extra");
    command("tcal unknown");
    assert(retained->bootCalState.enabled && rejects == 9);
    command("tcal export extra");
    command("tcal import extra");
    assert(backup_calls == 2 && rejects == 9);
    command("tcal import a b c d e f");
    assert(input_losses == 1 && backup_calls == 2 && rejects == 10);
#if CONFIG_SENSOR_TCAL_HEATED
    command("tcal heat status extra");
    assert(rejects == 10 && heat_reports == 0);
    command("tcal heat status");
    assert(heat_reports == 1);
#endif
#else
    command("tcal");
    assert(rejects == 1 && backup_calls == 0);
    command("tcal status a b c d e");
    assert(rejects == 1 && status_calls == 0 && backup_calls == 0);
#endif
    unsigned sens_rejects = rejects;
#if CONFIG_SENSOR_USE_SENS_CALIBRATION
    command("sens");
    assert(rejects == sens_rejects + 1 && sens_writes == 0);
    command("sens 1,2,3");
    assert(sens_writes == 1);
    command("sens reset extra");
    command("sens auto x y z");
    assert(rejects == sens_rejects + 3 && sens_resets == 0 && sens_auto_calls == 0);
    command("sens auto");
    command("sens auto q");
    command("sens auto x 0");
    assert(rejects == sens_rejects + 6 && sens_auto_calls == 0);
    command("sens reset");
    command("sens auto x 3");
    assert(sens_resets == 1 && sens_auto_calls == 1 && sens_axis == 0 && sens_revolutions == 3);
#else
    command("sens");
    assert(rejects == sens_rejects + 1 && sens_writes == 0);
    command("sens a b c d e f");
    assert(rejects == sens_rejects + 2 && sens_writes == 0 && sens_auto_calls == 0);
#endif
    unsigned previous = rejects;
    command("ping stats");
    assert(ping_stats == 1 && ping_calls == 0);
    command("ping stats extra");
    assert(rejects == previous && ping_stats == 1 && ping_calls == 0);
    command("ping typo extra");
    assert(rejects == previous + 1 && ping_stats == 1 && ping_calls == 0);
    command("ping");
    assert(ping_calls == 1);
    command("tdma stats a b c d e");
    assert(rejects == previous + 1);
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    assert(tdma_stats == 1);
#endif
    command("tdma on extra");
    assert(rejects == previous + 2 && tdma_writes == 0);
    command("tdma on");
    assert(tdma_enabled && tdma_writes == 1);
    command("unknown a b");
    assert(rejects == previous + 2 && tdma_writes == 1);
    unsigned feature_rejects = rejects;
#if DFU_EXISTS
    command("dfu bad");
    command("dfu ota extra");
    assert(rejects == feature_rejects + 2 && dfu_calls == 0);
    command("dfu");
    assert(dfu_calls == 1 && !dfu_ota);
#if ADAFRUIT_BOOTLOADER
    command("dfu ota");
    assert(dfu_calls == 2 && dfu_ota && rejects == feature_rejects + 2);
#else
    command("dfu ota");
    assert(dfu_calls == 1 && rejects == feature_rejects + 3);
#endif
#else
    command("dfu a b c d e f");
    assert(dfu_calls == 0 && rejects == feature_rejects + 1);
#endif
    feature_rejects = rejects;
#if CONFIG_VQF_BENCH
    command("vqfbench 0");
    command("vqfbench 10 extra");
    assert(rejects == feature_rejects + 2 && bench_calls == 0);
    command("vqfbench");
    assert(bench_calls == 1 && bench_iterations == 1000 && !bench_active);
    command("vqfbench 20000");
    assert(bench_calls == 2 && bench_iterations == 20000 && bench_switches == 4 && !bench_active);
    assert(bench_accepted == 2 && bench_success == 2 && rejects == feature_rejects + 2);
#else
    command("vqfbench a b c d e f");
    assert(bench_calls == 0 && rejects == feature_rejects + 1);
#endif
    puts("production console dispatcher boundaries passed");
    return 0;
}
