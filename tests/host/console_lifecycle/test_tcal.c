#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CONFIG_SENSOR_TCAL_HEATED 0
#define CONFIG_SENSOR_POLY_STEPS_PER_DEGREE 2
#define TCAL_BUFFER_SIZE 2
static struct {
    struct { int count; } tempCalState;
    struct { float temp, bias[3]; } tempCalPoints[TCAL_BUFFER_SIZE];
    struct { bool enabled, completed, doffset_valid; unsigned attempt_count; float doffset[3]; } bootCalState;
} retained_storage, *retained = &retained_storage;
static unsigned rejects, auto_writes, remove_calls, test_calls;
static int removed_index;
static bool tcal_enabled, auto_enabled;
static uint32_t console_command_input_generation;

static void test_printk(const char *format, ...) { (void)format; }
#define printk test_printk
static void console_reject(void) { rejects++; }
static void sensor_tcal_backup_command(size_t argc, char **argv, uint32_t generation)
{
    (void)argc; (void)argv; (void)generation;
}
static void sensor_tcal_set_enabled(bool value) { tcal_enabled = value; }
static bool sensor_tcal_get_enabled(void) { return tcal_enabled; }
static void sensor_tcal_status(void) {}
static const char *sensor_tcal_get_apply_mode_name(void) { return "test"; }
static bool sensor_tcal_get_auto_calibration(void) { return auto_enabled; }
static void sensor_tcal_set_auto_calibration(bool value) { auto_enabled = value; auto_writes++; }
static void cmd_reset_tcal(void) {}
static void k_msleep(int delay) { (void)delay; }
static void sensor_tcal_remove_point(int index) { removed_index = index; remove_calls++; }
static float sensor_get_current_imu_temperature(void) { return 25; }
static void sensor_tcal_test_methods(float temperature) { (void)temperature; test_calls++; }
static bool sensor_tcal_needs_nearby_point(float temperature, float *closest, float *distance)
{
    (void)temperature; *closest = 25; *distance = 0; return false;
}
static void sensor_boot_cal_set_enabled(bool value) { retained->bootCalState.enabled = value; }

#include "production_tcal.inc"

static void command(const char *text)
{
    char line[128], *argv[8] = {0};
    snprintf(line, sizeof(line), "%s", text);
    size_t argc = parse_args(line, argv, 8);
    assert(argc > 0);
    console_cmd_tcal(argc, argv);
}

int main(void)
{
    command("tcal");
    assert(rejects == 1);
    command("tcal auto");
    assert(rejects == 2 && auto_writes == 0);
    command("tcal auto on");
    assert(auto_enabled && auto_writes == 1);
    command("tcal auto off");
    assert(!auto_enabled && auto_writes == 2);
    command("tcal auto maybe");
    assert(rejects == 3 && auto_writes == 2);
    command("tcal auto on extra");
    assert(rejects == 4 && auto_writes == 2);
    command("tcal remove");
    assert(rejects == 5 && remove_calls == 0);
    command("tcal remove 2junk");
    assert(rejects == 6 && remove_calls == 0);
    command("tcal remove 2");
    assert(remove_calls == 1 && removed_index == 2);
    command("tcal remove 1 extra");
    assert(rejects == 7 && remove_calls == 1);
    command("tcal test");
    assert(test_calls == 1);
    puts("production T-Cal console argument handling: PASS");
    return 0;
}
