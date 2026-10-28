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
static void console_reject(void) {}
static bool console_require_args(size_t argc, size_t min, size_t max)
{ return argc >= min && argc <= max; }
static void cmd_reset_zro(void) {}
static void cmd_reset_bat(void) {}
static void cmd_fusion_reset(void) {}
static void sys_clear(void) { clears++; }
static void console_cmd_reset(size_t argc, char **argv);
struct console_cmd { const char *name; void (*fn)(size_t, char **); };
static const struct console_cmd reset_command = {"reset", console_cmd_reset};
static const struct console_cmd *console_find_command(const char *name)
{ return strcmp(name, "reset") == 0 ? &reset_command : NULL; }
static bool console_feedback_enabled;
static bool console_command_mutates(const struct console_cmd *cmd, size_t argc, char **argv)
{ (void)cmd; (void)argc; (void)argv; return true; }
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
    puts("RTT reset confirmation and overflow passed");
    return 0;
}
