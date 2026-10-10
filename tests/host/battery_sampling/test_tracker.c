#include <assert.h>
#include <errno.h>
#include <ctype.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define CONFIG_SYS_CLOCK_TICKS_PER_SEC 1000
#define BATT_STATS_INTERVAL_0 0
#define BATT_STATS_LAST_RUN_ID 19
#define BATT_STATS_CURVE_ID 20
#define LOG_MODULE_REGISTER(...)
#define LOG_LEVEL_INF 0
/* Evaluate arguments, without imposing host printf ABI on Zephyr formats. */
static void log_message(const char *format, ...) { (void)format; }
#define LOG_INF(...) log_message(__VA_ARGS__)
#define LOG_ERR(...) log_message(__VA_ARGS__)
#define LOG_WRN(...) log_message(__VA_ARGS__)
#define LOG_DBG(...) log_message(__VA_ARGS__)
#define printk(...) log_message(__VA_ARGS__)
#define LED_OWNER_POWER 0
#define LED_INPUT_ACK 1
#define LED_PARTIAL 2
#define LED_SUCCESS 3
static int led_request_id(void) { return 0; }
static int led_event_id(void) { return 0; }
static void led_request_event(int owner, int request, int event, int input)
{ (void)owner; (void)request; (void)event; (void)input; }

static struct {
    int16_t battery_pptt_curve[18];
    int16_t max_battery_pptt, min_battery_pptt, battery_pptt_saved;
    uint64_t battery_runtime_sum, battery_runtime_saved, battery_uptime_latest;
} retained_data, *retained = &retained_data;
static uint64_t ticks;
static uint64_t k_uptime_ticks(void) { return ticks; }
static uint64_t k_ticks_to_us_floor64(uint64_t value) { return value * 1000; }
static void retained_update(void) {}
static unsigned char storage[21][64];
static int write_error, write_calls;
static void sys_read(uint16_t id, void *data, size_t len)
{
    assert(id < ARRAY_SIZE(storage) && len <= sizeof(storage[0]));
    memcpy(data, storage[id], len);
}
static int sys_write(uint16_t id, void *retained_ptr, const void *data, size_t len)
{
    assert(id < ARRAY_SIZE(storage) && len <= sizeof(storage[0]));
    write_calls++;
    /* Match the eager storage contract: retained RAM changes even on failure. */
    if (retained_ptr) memcpy(retained_ptr, data, len);
    if (write_error) return write_error;
    memcpy(storage[id], data, len);
    return 0;
}
/* A baseline heap request fails the behavior test, rather than being silently mocked. */
static void *forbidden_alloc(size_t size) __attribute__((unused));
static void *forbidden_alloc(size_t size) { (void)size; assert(!"curve allocated heap"); return NULL; }
#define k_malloc forbidden_alloc
#define k_free free
#include "production.inc"

/* Capture only console rendering; tracker debug keeps the existing printk leaf. */
static char console_output[4096];
static size_t console_length;
static void console_printk(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int written = vsnprintf(console_output + console_length,
                            sizeof(console_output) - console_length, format, args);
    va_end(args);
    assert(written >= 0 && (size_t)written < sizeof(console_output) - console_length);
    console_length += (size_t)written;
}
#undef printk
#define printk(...) console_printk(__VA_ARGS__)
#include "console.inc"
#undef printk

static void assert_battery_rendering(int expected_mV, int16_t expected_pptt)
{
    assert(sys_get_valid_battery_mV() == expected_mV);
    assert(sys_get_valid_battery_pptt() == expected_pptt);
    void (*const renderers[])(void) = {print_battery, print_battery_tracker};
    for (size_t renderer = 0; renderer < ARRAY_SIZE(renderers); renderer++) {
        console_length = 0;
        console_output[0] = '\0';
        renderers[renderer]();
        char *line = strstr(console_output, "Battery:");
        assert(line && (line == console_output || line[-1] == '\n'));
        char *end = strchr(line, '\n');
        assert(end);
        *end = '\0';
        /* Calibration and cycle percentages elsewhere are not battery SOC. */
        unsigned percentages = 0;
        for (char *cursor = line; *cursor;) {
            if (isdigit((unsigned char)*cursor) || *cursor == '-' || *cursor == '+') {
                char *number_end;
                double value = strtod(cursor, &number_end);
                if (number_end != cursor) {
                    if (*number_end == '%') {
                        assert(expected_pptt >= 0);
                        assert(value >= 0.0 && value <= 100.0);
                        if (expected_pptt == 0)
                            assert(value == 0.0);
                        else
                            assert(value > 0.0);
                        if (percentages > 0)
                            assert(value == (double)expected_pptt / 100.0);
                        percentages++;
                    }
                    cursor = number_end;
                    continue;
                }
            }
            cursor++;
        }
        if (expected_pptt < 0) {
            assert(!strchr(line, '%'));
        } else {
            assert(percentages == (renderer == 0 ? 1U : 2U));
        }
        if (expected_mV > 0 && (expected_pptt < 0 || renderer == 1)) {
            char voltage[32];
            snprintf(voltage, sizeof(voltage), "%d mV", expected_mV);
            assert(strstr(line, voltage));
        } else if (expected_mV < 0) {
            assert(!strstr(line, " mV"));
        }
    }
}

static void console_scenario(void)
{
    /* Cold USB boot must not turn charged USB-side measurements into battery SOC. */
    ticks = 1000;
    sys_update_battery_tracker_voltage(4300, true);
    sys_update_battery_tracker(10000, true);
    assert(sys_get_battery_mV() == 4300);
    assert_battery_rendering(-1, -1);

    /* Voltage can settle before the SOC filter produces its first valid sample. */
    ticks += 1000;
    sys_update_battery_tracker_voltage(4211, false);
    sys_update_battery_tracker(-1, false);
    assert(sys_get_battery_mV() == 4211);
    assert_battery_rendering(4211, -1);

    /* Replug holds the last unplugged voltage, without inventing its missing SOC. */
    ticks += 1000;
    sys_update_battery_tracker_voltage(4350, true);
    sys_update_battery_tracker(10000, true);
    assert(sys_get_battery_mV() == 4350);
    assert_battery_rendering(4211, -1);

    ticks += 1000;
    sys_update_battery_tracker_voltage(3900, false);
    sys_update_battery_tracker(7500, false);
    assert_battery_rendering(3900, 7500);

    /* Real empty-battery SOC is valid and must not be hidden with the sentinel. */
    ticks += 1000;
    sys_update_battery_tracker_voltage(3000, false);
    sys_update_battery_tracker(0, false);
    assert_battery_rendering(3000, 0);
}

static struct battery_tracker_interval interval(unsigned id)
{
    struct battery_tracker_interval result;
    sys_read(id, &result, sizeof(result));
    return result;
}
static void seed(unsigned id, uint64_t runtime)
{
    struct battery_tracker_interval value = {
        .cycles = 1, .runtime = runtime, .runtime_min = runtime, .runtime_max = runtime,
    };
    memcpy(storage[id], &value, sizeof(value));
}
static void setup_discharge(void)
{
    retained->max_battery_pptt = 10000;
    retained->min_battery_pptt = 9000;
    retained->battery_pptt_saved = 9500;
    retained->battery_runtime_saved = 10000;
    retained->battery_runtime_sum = 10000;
    ticks = 300000;
}
int main(int argc, char **argv)
{
    assert(argc == 2);
    if (!strcmp(argv[1], "console")) {
        console_scenario();
    } else if (!strcmp(argv[1], "retry")) {
        setup_discharge();
        seed(18, 400000);
        for (int failure = 0; failure < 2; failure++) {
            write_error = failure ? -ENOSPC : -EIO;
            sys_update_battery_tracker(9000, false);
            assert(write_calls == failure + 1);
            assert(retained->battery_pptt_saved == 9500);
            assert(retained->battery_runtime_saved == 10000);
            assert(interval(18).cycles == 1 && interval(18).runtime == 400000);
            ticks += 1000;
        }
        write_error = 0;
        sys_update_battery_tracker(9000, false);
        assert(write_calls == 3);
        assert(retained->battery_pptt_saved == 9000);
        assert(retained->battery_runtime_saved == 312000);
        assert(interval(18).cycles == 2 && interval(18).runtime == 702000);
        sys_update_battery_tracker(9000, false);
        assert(write_calls == 3 && interval(18).cycles == 2);
    } else if (!strcmp(argv[1], "short")) {
        setup_discharge();
        ticks = 299999;
        sys_update_battery_tracker(9000, false);
        assert(write_calls == 0 && retained->battery_pptt_saved == 9500);
        ticks++;
        sys_update_battery_tracker(9000, false);
        assert(write_calls == 1 && retained->battery_pptt_saved == 9000);
    } else if (!strcmp(argv[1], "partial")) {
        setup_discharge();
        retained->min_battery_pptt = 9500;
        retained->battery_pptt_saved = 10000;
        sys_update_battery_tracker(9500, false);
        assert(write_calls == 0 && retained->battery_pptt_saved == 9500);
    } else if (!strcmp(argv[1], "curve") || !strcmp(argv[1], "curve_failure")) {
        seed(0, 300000);
        seed(1, 900000);
        assert(sys_get_calibrated_battery_pptt(500) == 500);
        write_error = !strcmp(argv[1], "curve_failure") ? -EIO : 0;
        update_curve();
        assert(write_calls == 1 && retained->battery_pptt_curve[0] == 250);
        assert(sys_get_calibrated_battery_pptt(500) == 250);
        assert(sys_get_calibrated_battery_pptt(500) == 250);
    } else if (!strcmp(argv[1], "reset") || !strcmp(argv[1], "reset_failure")) {
        retained->battery_pptt_curve[0] = 250;
        assert(sys_get_calibrated_battery_pptt(500) == 250);
        write_error = !strcmp(argv[1], "reset_failure") ? -EIO : 0;
        assert(sys_reset_battery_tracker() == 1);
        assert(sys_reset_battery_tracker() == write_error);
        assert(sys_get_calibrated_battery_pptt(500) == 500);
    } else if (!strcmp(argv[1], "migration")) {
        retained->battery_pptt_curve[5] = 3000;
        assert(sys_get_calibrated_battery_pptt(3000) == 3000);
        assert(sys_migrate_battery_curve());
        assert(sys_get_calibrated_battery_pptt(3000) == 4900);
    } else if (!strcmp(argv[1], "insufficient")) {
        retained->battery_pptt_curve[0] = 250;
        seed(0, 300000);
        update_curve();
        assert(write_calls == 0 && retained->battery_pptt_curve[0] == 250);
    } else if (!strcmp(argv[1], "gaps")) {
        seed(0, 300000);
        seed(2, 900000);
        update_curve();
        assert(retained->battery_pptt_curve[0] == 283);
        assert(retained->battery_pptt_curve[1] == 850);
        assert(write_calls == 1);
        /* Reuse the private workspace with a different learned region. */
        memset(storage, 0, sizeof(storage));
        seed(2, 300000);
        seed(3, 300000);
        update_curve();
        assert(retained->battery_pptt_curve[0] == 0);
        assert(retained->battery_pptt_curve[1] == 0);
        assert(retained->battery_pptt_curve[2] == 1750);
        assert(write_calls == 2);
    } else {
        assert(!"unknown scenario");
    }
    printf("PASS battery tracker %s\n", argv[1]);
    return 0;
}
