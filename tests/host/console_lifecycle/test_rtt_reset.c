#include <assert.h>
#include <ctype.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define USB_EXISTS 0
#define UART_CONSOLE_EXISTS 0
#define CONFIG_SENSOR_USE_TCAL 0
#define CONFIG_SENSOR_USE_ACCEL_CALIBRATION 0
#define CONFIG_SENSOR_USE_MAGNETOMETER 0
#define CONFIG_SENSOR_USE_SENS_CALIBRATION 0
#define CONFIG_LED_DEBUG 0
#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
#define ARG_UNUSED(x) (void)(x)
#define LED_OWNER_SYSTEM 0
#define LED_REJECTED 1
#define FW_STRING "test"
#define FW_GIT_REPO_URL "test"
#define FW_GIT_BRANCH "test"

/* SEGGER, clock and reset storage are modeled; the reader and worker are production. */
static const char *input;
static jmp_buf worker_idle;
static int64_t now_ms;
static unsigned clears;
static char output[4096];
static int64_t k_uptime_get(void) { return now_ms; }
static int SEGGER_RTT_HasKey(void) { return *input != '\0'; }
static int SEGGER_RTT_GetKey(void) { return *input ? (unsigned char)*input++ : -1; }
static void k_usleep(int usec) { (void)usec; longjmp(worker_idle, 1); }
static void test_printk(const char *format, ...)
{
    size_t used = strlen(output);
    va_list args;
    va_start(args, format);
    vsnprintf(output + used, sizeof(output) - used, format, args);
    va_end(args);
}
#define printk test_printk
#include "system/rtt_console.h"

static void strtolower(char *text)
{
    for (; *text; text++) *text = (char)tolower((unsigned char)*text);
}
static bool console_feedback_enabled;
static unsigned rejects;
static unsigned led_request_id(void) { return 1; }
static unsigned led_event_id(void) { return 1; }
static void led_request_event(int owner, unsigned request, unsigned event, int semantic)
{
    (void)owner; (void)request; (void)event;
    if (semantic == LED_REJECTED) rejects++;
}
static void console_unrelated_command(size_t argc, char **argv)
{ (void)argc; (void)argv; }
static void cmd_reset_zro(void) {}
static void cmd_reset_bat(void) {}
static void cmd_fusion_reset(void) {}
static void sys_clear(void) { clears++; }
static void console_cmd_reset(size_t argc, char **argv);
#include "production_rtt.inc"

static void receive(const char *text)
{
    input = text;
    if (setjmp(worker_idle) == 0) console_thread();
    assert(*input == '\0');
}

int main(void)
{
    receive("reset all\n");
    assert(clears == 0);
    assert(strstr(output, "any other submitted line cancels") != NULL);
    char overlong[80];
    memset(overlong, 'x', sizeof(overlong));
    overlong[sizeof(overlong) - 2] = '\n';
    overlong[sizeof(overlong) - 1] = '\0';
    receive(overlong);
    assert(strstr(output, "Input line too long") != NULL);
    receive("reset all\n");
    assert(clears == 0);
    receive("reset all\n");
    assert(clears == 1);

    receive("reset all\n");
    now_ms = 10000;
    receive("reset all\n");
    assert(clears == 1);
    now_ms++;
    receive("reset all\n");
    assert(clears == 2);
    receive("reset all\nhelp\nreset all\n");
    assert(clears == 2);
    receive("reset all\n");
    assert(clears == 3);
    unsigned previous = rejects;
    receive("ping stats extra\n");
    assert(rejects == previous);
    receive("ping typo extra\n");
    assert(rejects == previous + 1);
    receive("reset all\n");
    receive("reset all extra\n");
    assert(rejects == previous + 2 && clears == 3);
    receive("reset all\n");
    assert(clears == 3);
    receive("reset all\n");
    assert(clears == 4);
    receive("reset all\nunknown\nreset all\n");
    assert(clears == 4);
    receive("reset all\n");
    assert(clears == 5);
    puts("RTT reset confirmation and overflow passed");
    return 0;
}
