#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#define DT_NODE_HAS_STATUS(node, status) TEST_PMIC
#define DT_NODELABEL(node) node
#define LOG_MODULE_REGISTER(...)
#define LOG_INF(...) do {} while (0)
#define LOG_ERR(...) do {} while (0)
#define LOG_WRN(...) do {} while (0)
#define NRFX_ABS(value) abs(value)
#define POWER_USBREGSTATUS_VBUSDETECT_Msk 1
#define K_MSEC(value) (value)
enum { SYS_STATUS_PLUGGED, SYS_STATUS_SYSTEM_ERROR, SYS_REGULATOR_DCDC,
       SYS_LED_PATTERN_PULSE_PERSIST, SYS_LED_PATTERN_ON_PERSIST,
       SYS_LED_PATTERN_LONG_PERSIST, SYS_LED_PATTERN_ACTIVE_PERSIST,
       SYS_LED_PRIORITY_SYSTEM, WDT_CHANNEL_POWER };
static struct { unsigned USBREGSTATUS; } power_registers;
#define NRF_POWER (&power_registers)
static int power_wake_sem;
static bool plugged, power_init;
static int64_t clock_ms, attempts_at[128];
static unsigned attempts, voltage_updates, soc_updates, shutdowns, watchdogs, waits;
static bool input_charging, input_charged, input_pmic, input_dock, status_plugged;
static int input_mv = 3800;
static int16_t input_soc = 5000;
static int last_voltage;
static bool dock_read(void) { return input_dock; }
static bool chg_read(void) { return input_charging; }
static bool stby_read(void) { return input_charged; }
static int battery_charger_state(bool *pmic, bool *charging, bool *charged)
{
    *pmic = input_pmic;
    return 0;
}
static int16_t read_batt_mV(int *mv)
{
    assert(attempts < 128);
    attempts_at[attempts++] = clock_ms;
    *mv = input_mv;
    return input_soc;
}
static int64_t k_uptime_get(void) { return clock_ms; }
static void set_status(int status, bool enabled)
{
    if (status == SYS_STATUS_PLUGGED) status_plugged = enabled;
}
static void set_regulator(int regulator) {}
static void set_led(int pattern, int priority) {}
static void sys_update_battery_tracker_voltage(int mv, bool charging)
{
    ++voltage_updates;
    last_voltage = mv;
}
static void sys_update_battery_tracker(int16_t soc, bool charging) { ++soc_updates; }
static int16_t sys_get_calibrated_battery_pptt(int16_t soc) { return soc; }
static bool sys_system_off(void) { ++shutdowns; return true; }
static void connection_update_battery(bool available, bool charging, bool charged,
                                      uint32_t soc, int mv) {}
static void watchdog_feed(int channel) { ++watchdogs; }
static int k_sem_take(int *sem, int timeout)
{
    assert(timeout == 100);
    ++waits;
    return 0;
}
#include "production.inc"

static void tick(int64_t time)
{
    clock_ms = time;
    power_iteration();
    assert(watchdogs == waits);
}
static void feed(int16_t soc)
{
    power_battery_feed_and_track(true, false, soc, true, 3800);
}
int main(int argc, char **argv)
{
    assert(argc == 2);
    const char *scenario = argv[1];
    if (!strcmp(scenario, "cadence")) {
        for (int t = 0; t <= 1000; t += 100) tick(t);
        assert(attempts == (TEST_PMIC ? 11 : 3));
        for (unsigned i = 0; i < attempts; ++i)
            assert(attempts_at[i] == i * (TEST_PMIC ? 100 : 500));
        assert(voltage_updates == attempts);
        assert(samples == (TEST_PMIC ? 11 : 3));
        unsigned before = attempts;
        tick(1001); tick(1010); tick(1099);
        assert(attempts == before + (TEST_PMIC ? 3 : 0));
        assert(voltage_updates == attempts);
        assert(watchdogs == 14);
    } else if (!strcmp(scenario, "edges")) {
        tick(0);
        input_charging = true; tick(200);
        input_charging = false; input_charged = true; tick(300);
        NRF_POWER->USBREGSTATUS = 1; tick(350);
        input_pmic = true; tick(400);
        tick(899);
        assert(attempts == 5);
        tick(900);
        assert(attempts == 6);
        const int expected[] = {0, 200, 300, 350, 400, 900};
        for (unsigned i = 0; i < attempts; ++i) assert(attempts_at[i] == expected[i]);
    } else if (!strcmp(scenario, "failure")) {
        tick(0);
        input_soc = -EIO; input_mv = 0;
        tick(500);
        assert(power_battery_average_pptt() == 5000);
        unsigned before = attempts;
        unsigned feeds = voltage_updates;
        tick(501); tick(600); tick(999);
        assert(attempts == before + (TEST_PMIC ? 3 : 0));
        assert(voltage_updates == feeds + (TEST_PMIC ? 3 : 0));
        assert(samples == 1);
        input_soc = 5200; input_mv = 3850; tick(1000);
        assert(samples == 2);
        assert(power_battery_average_pptt() == 5100);
        assert(last_voltage == 3850);
    } else if (!strcmp(scenario, "warmup")) {
        feed(5000); assert(power_battery_average_pptt() == 5000);
        feed(5200); assert(power_battery_average_pptt() == 5100);
        feed(5400); assert(power_battery_average_pptt() == 5200);
        feed(5600); assert(power_battery_average_pptt() == 5300);
        assert(soc_updates == 0);
    } else if (!strcmp(scenario, "filter")) {
        int fill = TEST_PMIC ? 24 : 5;
        int step = TEST_PMIC ? 100 : 500;
        for (int i = 0; i < fill - 1; ++i) tick(i * step);
        assert(soc_updates == 0);
        tick((fill - 1) * step);
        assert(soc_updates == 1);
        assert(power_battery_average_pptt() == 5000);
        if (TEST_PMIC) {
            /* Three extremes on either side must still be removed. */
            for (int i = 0; i < 3; ++i) feed(4100);
            for (int i = 0; i < 3; ++i) feed(5900);
            assert(power_battery_average_pptt() == 5000);
        }
    } else if (!strcmp(scenario, "outlier")) {
        for (int i = 0; i < (TEST_PMIC ? 24 : 5); ++i) feed(5000);
        feed(5900);
        assert(power_battery_average_pptt() == 5000);
        for (int i = 0; i < (TEST_PMIC ? 24 : 5); ++i) feed(5000);
        feed(4100);
        assert(power_battery_average_pptt() == 5000);
    } else if (!strcmp(scenario, "dock")) {
        tick(0);
        input_dock = true; tick(100);
        assert(shutdowns == 1);
        assert(attempts == (TEST_PMIC ? 2 : 1));
        assert(watchdogs == 2);
    } else if (!strcmp(scenario, "low")) {
        tick(0);
        input_soc = 0; input_mv = 3000; tick(500);
        assert(power_battery_average_pptt() == 0);
        assert(shutdowns == 0);
        tick(600);
        assert(shutdowns == 1);
        assert(attempts == (TEST_PMIC ? 3 : 2));
    } else if (!strcmp(scenario, "debounce")) {
        tick(0);
        input_charging = true; tick(100);
        input_charging = false; tick(200);
        tick(700);
        assert(!power_battery_device_plugged());
        assert(!status_plugged);
        input_charging = true; tick(800);
        tick(1299); assert(!power_battery_device_plugged());
        tick(1300); assert(power_battery_device_plugged());
        assert(status_plugged);
        tick(3800); assert(samples == 0);
        tick(4299); assert(samples == 0);
        tick(4300); assert(samples == 1);
    } else if (!strcmp(scenario, "settle")) {
        tick(0);
        input_charging = true; tick(100);
        /* A charged edge advances ADC deadline without changing raw plugged. */
        input_charged = true; tick(550);
        tick(600);
        assert(power_battery_device_plugged());
        assert(attempts == 3); /* debounce progressed without an ADC read */
        tick(3550); assert(samples == 0);
        tick(3600); assert(samples == 0); /* settled, but no new sample */
        tick(4050); assert(samples == 1);
    } else {
        assert(!"unknown scenario");
    }
    printf("battery sampling pmic=%d %s passed\n", TEST_PMIC, scenario);
    return 0;
}
