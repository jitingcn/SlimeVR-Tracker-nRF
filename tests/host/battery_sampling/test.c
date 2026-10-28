#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "../led_feedback_stub.h"
#define ARG_UNUSED(value) ((void)(value))
#define USE_PMIC_CHARGER TEST_PMIC
#define GPIO_ACTIVE_LOW 1
#define GPIO_ACTIVE_HIGH 0
#define GPIO_PULL_DOWN 16
#define GPIO_OUTPUT 8
typedef int gpio_flags_t;
#define GPIO_PULL_UP 2
#if TEST_BOARD_GPIO
#include "board_gpio.inc"
#else
#define CHG_EXISTS (!TEST_NO_GPIO)
#define STBY_EXISTS (!TEST_NO_GPIO)
#define CHARGER_STAT_EXISTS 0
#define PLUG_EXISTS 0
#define TEST_DT_HAS_charger_full_on_plug 0
#define TEST_CHG_PORT 0
#define TEST_CHG_PIN 2
#define TEST_CHG_FLAGS (GPIO_ACTIVE_LOW | GPIO_PULL_UP)
#endif
#ifndef TEST_CHG_PORT
#define TEST_CHG_PORT 0
#define TEST_CHG_PIN 2
#define TEST_CHG_FLAGS (GPIO_ACTIVE_LOW | GPIO_PULL_UP)
#endif
#define DOCK_EXISTS 0
#define TEST_DT_HAS_stat_gpios CHARGER_STAT_EXISTS
#define BUTTON_EXISTS 0
#define USER_SHUTDOWN_ENABLED 1
#define K_FOREVER 0
#define NRF_GPIO_PIN_NOSENSE 0
#define NRF_GPIO_PIN_SENSE_LOW 1
#define NRF_GPIO_PIN_SENSE_HIGH 2
#define NRF_GPIO_PIN_PULLUP GPIO_PULL_UP
#define NRF_DT_GPIOS_TO_PSEL(node, prop) TEST_PIN_##prop
#define TEST_PIN_chg_gpios TEST_CHG_PIN
#define TEST_PIN_stby_gpios 3
#if CHARGER_STAT_EXISTS
#define TEST_PIN_stat_gpios (TEST_STAT_PORT * 32 + TEST_STAT_PIN)
#endif
typedef int nrf_gpio_pin_sense_t;
typedef int nrf_gpio_pin_pull_t;
#define NRF_GPIO_PIN_PULLDOWN GPIO_PULL_DOWN
#define NRF_GPIO_PIN_DIR_INPUT 0
#define NRF_GPIO_PIN_DIR_OUTPUT 1
#define NRF_GPIO_PIN_INPUT_CONNECT 0
#define NRF_GPIO_PIN_INPUT_DISCONNECT 1
#define NRF_GPIO_LATCH_PRESENT 1
static unsigned irq_depth, physical_offs;
static unsigned irq_lock(void) { return irq_depth++; }
static void irq_unlock(unsigned key) { assert(irq_depth == key + 1); irq_depth = key; }
static bool host_ignore_charge_wake;
#define IGNORE_CHARGE_WAKE_ON_VBUS host_ignore_charge_wake

#define CONFIG_SENSOR_TCAL_HEATED TEST_HEATER
#define TEST_DT_HAS_plug_gpios PLUG_EXISTS
#define DT_NODE_HAS_PROP(node, prop) TEST_DT_HAS_##prop
#define CHARGER_FULL_ON_PLUG (TEST_DT_HAS_charger_full_on_plug && PLUG_EXISTS)
#define GPIO_INPUT 4
#define atomic_get(value) (*(value))
#define DT_NODE_HAS_STATUS(node, status) TEST_PMIC
#define DT_NODELABEL(node) node
#define LOG_MODULE_REGISTER(...)
#define LOG_INF(...) do {} while (0)
#define LOG_ERR(...) do {} while (0)
#define LOG_WRN(...) do {} while (0)
#define NRFX_ABS(value) abs(value)
#define POWER_USBREGSTATUS_VBUSDETECT_Msk 1
#define K_MSEC(value) (value)
enum { SYS_STATUS_PLUGGED, SYS_STATUS_SYSTEM_ERROR, SYS_REGULATOR_DCDC, WDT_CHANNEL_POWER };
static struct { unsigned USBREGSTATUS; } power_registers;
#define NRF_POWER (&power_registers)
static int power_wake_sem;
static bool plugged, power_init;
static int64_t clock_ms, attempts_at[128];
static unsigned attempts, voltage_updates, soc_updates, shutdowns, watchdogs, waits;
static bool input_charging, input_charged, input_pmic, input_dock, status_plugged;
static bool reported_charged, reported_plugged;
static unsigned boot_shutdowns;
static bool booting_from_shutdown;
static int reset_mode;
static int input_mv = 3800;
static int16_t input_soc = 5000;
static bool gpio_available = true;
static int charger_error, gpio_error, charge_read_error, standby_error, plug_error, plug_config_error;
static unsigned charge_reads, standby_reads;
static bool plug_available = true;
static bool gpio_use_raw;
static int raw_charge_level = 1, raw_standby_level = 1;
static int raw_plug_level = 1;
#if CONFIG_SENSOR_TCAL_HEATED
static bool heater_plug_ready = true;
#endif
static struct gpio_fixture { int port, pin, flags; } chg = {
    TEST_CHG_PORT, TEST_CHG_PIN, TEST_CHG_FLAGS,
}, stby = {0, 3, GPIO_ACTIVE_LOW | GPIO_PULL_UP};
#if CHARGER_STAT_EXISTS
static struct gpio_fixture charger_stat = {TEST_STAT_PORT, TEST_STAT_PIN, TEST_STAT_FLAGS};
static int charger_stat_lock;
static bool charger_stat_sense_armed, stat_locked;
static int stat_mode = 2, stat_bias = GPIO_PULL_UP, stat_sense;
static unsigned stat_reads, stat_configs, stat_waits;
static unsigned stat_fail_read, stat_fail_config;
static int stat_read_error = -EIO, stat_config_error = -EIO;
static unsigned stat_ignore_config, stat_corrupt_config, stat_corrupt_wait;
static int stat_corruption;
static int stat_dir = NRF_GPIO_PIN_DIR_INPUT, stat_input = NRF_GPIO_PIN_INPUT_CONNECT;
static nrf_gpio_pin_sense_t charger_stat_wake_sense = NRF_GPIO_PIN_NOSENSE;
static bool stat_latched;
static unsigned stat_latch_clears;
static void stat_corrupt_input(void)
{
    if (stat_corruption == 1) stat_dir = NRF_GPIO_PIN_DIR_OUTPUT;
    if (stat_corruption == 2) stat_input = NRF_GPIO_PIN_INPUT_DISCONNECT;
    if (stat_corruption == 3) stat_bias = stat_bias == GPIO_PULL_UP ? GPIO_PULL_DOWN : GPIO_PULL_UP;
}
static int nrf_gpio_pin_dir_get(unsigned pin)
{ assert(pin == TEST_STAT_PORT * 32 + TEST_STAT_PIN); return stat_dir; }
static int nrf_gpio_pin_input_get(unsigned pin)
{ assert(pin == TEST_STAT_PORT * 32 + TEST_STAT_PIN); return stat_input; }
static int nrf_gpio_pin_pull_get(unsigned pin)
{ assert(pin == TEST_STAT_PORT * 32 + TEST_STAT_PIN); return stat_bias; }
static void nrf_gpio_pin_latch_clear(unsigned pin)
{
    assert(pin == TEST_STAT_PORT * 32 + TEST_STAT_PIN && irq_depth);
    stat_latched = false; ++stat_latch_clears;
}
static int stat_sources[2], stat_source_count;
static void k_mutex_lock(int *lock, int timeout)
{
    assert(lock == &charger_stat_lock && !stat_locked);
    stat_locked = true;
}
static void k_mutex_unlock(int *lock)
{
    assert(lock == &charger_stat_lock && stat_locked);
    stat_locked = false;
}
static int gpio_pin_configure(int port, int pin, int flags)
{
    assert(port == TEST_STAT_PORT && pin == TEST_STAT_PIN);
    assert((flags & GPIO_OUTPUT) == 0 && (flags & GPIO_INPUT)); /* Never drive STAT. */
    assert(stat_sense == NRF_GPIO_PIN_NOSENSE);
    assert(!irq_depth); /* GPIO driver calls must not hold the IRQ mask. */
    ++stat_configs;
    if (stat_configs == stat_fail_config) return stat_config_error;
    if (stat_configs == stat_ignore_config) return 0; /* SDK false-success model. */
    stat_dir = NRF_GPIO_PIN_DIR_INPUT;
    stat_input = NRF_GPIO_PIN_INPUT_CONNECT;
    stat_bias = flags & (GPIO_PULL_UP | GPIO_PULL_DOWN);
    if (stat_configs == stat_corrupt_config) stat_corrupt_input();
    return 0;
}
static int gpio_pin_get_raw(int port, int pin)
{
    assert(port == TEST_STAT_PORT && pin == TEST_STAT_PIN && stat_locked);
    assert(stat_sense == NRF_GPIO_PIN_NOSENSE);
    ++stat_reads;
    if (stat_reads == stat_fail_read) return stat_read_error;
    unsigned index = stat_reads - 1;
    if (stat_source_count && index >= (unsigned)stat_source_count)
        index = (unsigned)stat_source_count - 1;
    int source = stat_source_count ? stat_sources[index] : stat_mode;
    return source == 2 ? stat_bias == GPIO_PULL_UP : source;
}
static void k_busy_wait(unsigned us)
{
    assert(us == 100 && stat_locked && !irq_depth);
    ++stat_waits;
    if (stat_waits == stat_corrupt_wait) stat_corrupt_input();
    if (stat_latched) stat_sense = NRF_GPIO_PIN_SENSE_LOW; /* Global GPIOTE NONE re-arm. */
}
#endif
static void nrf_gpio_cfg_sense_set(int pin, int sense)
{
#if CHARGER_STAT_EXISTS
    assert(pin == TEST_STAT_PORT * 32 + TEST_STAT_PIN && (stat_locked || irq_depth));
    stat_sense = sense;
#endif
}
static void nrf_gpio_cfg_input(int pin, int pull) {}
bool vbus_read(void);
bool vin_read(void);
static void k_usleep(int us) { assert(power_init); }
static int sys_user_shutdown(void) { ++boot_shutdowns; return 0; }
#if PLUG_EXISTS
static struct gpio_fixture charger_plug = {
    TEST_PLUG_PORT, TEST_PLUG_PIN, TEST_PLUG_FLAGS,
};
#endif
#if CHARGER_FULL_ON_PLUG
static int charger_plug_error = -ENODEV;
#endif
#if CONFIG_SENSOR_TCAL_HEATED
#define heater_plug charger_plug
#endif
static bool gpio_is_ready_dt(const struct gpio_fixture *spec)
{
#if PLUG_EXISTS
    if (spec == &charger_plug) return plug_available;
#endif
    return gpio_available;
}
static int gpio_pin_configure_dt(const struct gpio_fixture *spec, int flags)
{
    (void)flags;
#if PLUG_EXISTS
    if (spec == &charger_plug) return plug_config_error;
#endif
    return 0;
}
static int gpio_pin_get_dt(const struct gpio_fixture *spec)
{
#if PLUG_EXISTS
    if (spec == &charger_plug) {
        return plug_error ? plug_error : raw_plug_level ^ !!(spec->flags & GPIO_ACTIVE_LOW);
    }
#endif
    assert(spec == &chg || spec == &stby);
    if (spec == &chg) ++charge_reads; else ++standby_reads;
    if (gpio_error) return gpio_error;
    if (spec == &chg && charge_read_error) return charge_read_error;
    if (spec == &stby && standby_error) return standby_error;
    if (gpio_use_raw) {
        int raw = spec == &chg ? raw_charge_level : raw_standby_level;
        return raw ^ !!(spec->flags & GPIO_ACTIVE_LOW);
    }
    return spec == &chg ? input_charging : input_charged;
}
#if TEST_PMIC
#define BIT(n) (1U << (n))
struct device { int unused; };
struct sensor_value { int32_t val1, val2; };
enum {
    SENSOR_CHAN_GAUGE_VOLTAGE,
    SENSOR_CHAN_NPM13XX_CHARGER_VBUS_STATUS,
    SENSOR_ATTR_NPM13XX_CHARGER_VBUS_PRESENT,
    SENSOR_CHAN_NPM13XX_CHARGER_STATUS,
};
static const struct device charger_device;
static const struct device *const charger = &charger_device;
static bool battery_ok = true, charger_sample_valid;
static int64_t charger_sample_ms;
static unsigned fetched_status, pmic_fetches;
static int64_t k_uptime_get(void);
int battery_sample(void);
static int sensor_sample_fetch(const struct device *device)
{
    assert(device == charger);
    ++pmic_fetches;
    if (input_soc < 0) return input_soc;
    fetched_status = (input_charged ? BIT(1) : 0) | (input_charging ? BIT(3) : 0);
    return 0;
}
static int sensor_channel_get(const struct device *device, int channel, struct sensor_value *value)
{
    assert(device == charger);
    if (channel == SENSOR_CHAN_GAUGE_VOLTAGE) {
        *value = (struct sensor_value){input_mv / 1000, (input_mv % 1000) * 1000};
        return 0;
    }
    assert(channel == SENSOR_CHAN_NPM13XX_CHARGER_STATUS);
    if (charger_error) return charger_error;
    *value = (struct sensor_value){fetched_status, 0};
    return 0;
}
static int sensor_attr_get(const struct device *device, int channel, int attribute,
                           struct sensor_value *value)
{
    assert(device == charger);
    assert(channel == SENSOR_CHAN_NPM13XX_CHARGER_VBUS_STATUS);
    assert(attribute == SENSOR_ATTR_NPM13XX_CHARGER_VBUS_PRESENT);
    *value = (struct sensor_value){input_pmic, 0};
    return 0;
}
static int sensor_value_to_milli(const struct sensor_value *value)
{
    return value->val1 * 1000 + value->val2 / 1000;
}
#endif
static int last_voltage;
static bool dock_read(void) { return input_dock; }
#if !TEST_PMIC
static int battery_charger_state(bool *pmic, bool *charging, bool *charged)
{
    *pmic = input_pmic;
    return charger_error ? charger_error : -ENOTSUP;
}
#endif
static int16_t read_batt_mV(int *mv)
{
    assert(attempts < 128);
    attempts_at[attempts++] = clock_ms;
    *mv = input_mv;
#if TEST_PMIC
    int sampled_mv = battery_sample();
    if (sampled_mv < 0) return sampled_mv;
    *mv = sampled_mv;
#endif
    return input_soc;
}
static int64_t k_uptime_get(void) { return clock_ms; }
static void set_status(int status, bool enabled)
{
    if (status == SYS_STATUS_PLUGGED) status_plugged = enabled;
}
static void set_regulator(int regulator) {}
static void sys_update_battery_tracker_voltage(int mv, bool charging)
{
    ++voltage_updates;
    last_voltage = mv;
}
static void sys_update_battery_tracker(int16_t soc, bool charging) { ++soc_updates; }
static int16_t sys_get_calibrated_battery_pptt(int16_t soc) { return soc; }
struct k_spinlock { bool held; };
typedef int k_spinlock_key_t;
static k_spinlock_key_t k_spin_lock(struct k_spinlock *lock)
{
    assert(!lock->held);
    lock->held = true;
    return 0;
}
static void k_spin_unlock(struct k_spinlock *lock, k_spinlock_key_t key)
{
    (void)key;
    assert(lock->held);
    lock->held = false;
}
static bool sys_system_off(void) { ++shutdowns; return true; }
/* The live serializer, not a battery-flag echo, supplies these wire bytes. */
static uint8_t batt, batt_v;
#define MAX(a, b) ((a) > (b) ? (a) : (b))
static void sys_poweroff(void)
{
    assert(irq_depth);
#if CHARGER_STAT_EXISTS
    if (charger_stat_sense_armed)
        assert(stat_sense == charger_stat_wake_sense && !stat_latched);
#endif
    ++physical_offs;
    irq_depth = 0; /* Host-only return from the terminal hardware leaf. */
}
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
    reported_plugged = batt_v >= 186; /* Actual wire charging-voltage threshold. */
    reported_charged = batt == 255;
}
static void feed(int16_t soc)
{
    power_battery_feed_and_track(true, false, soc, true, 3800);
}
#if CHARGER_STAT_EXISTS
static void stat_reset_counts(void)
{
    stat_reads = stat_configs = stat_waits = 0;
}
#endif
static void expect_snapshot_error(int error)
{
    for (int sentinel = 0; sentinel < 2; ++sentinel) {
        bool charging = sentinel, charged = !sentinel;
#if CHARGER_STAT_EXISTS
        stat_reset_counts();
#endif
        assert(sys_charger_snapshot(&charging, &charged) == error);
        assert(charging == !!sentinel && charged == !sentinel);
    }
}
static void expect_feedback(enum led_power_state expected, bool full)
{
    assert(led_test_power == expected);
    assert(power_battery_device_charged() == full);
    assert(reported_charged == full);
}
static void set_charge_source(int source)
{
#if CHARGER_STAT_EXISTS
    stat_mode = source;
#else
    input_charging = source == 0;
    input_charged = source == 1;
#endif
}
static void expect_battery_wire(void)
{
    expect_feedback(LED_POWER_BATTERY, false);
    assert(!power_battery_device_plugged() && !status_plugged && !reported_plugged);
    assert(batt == (0x80 | 100) && batt_v == 171); /* 100%, genuine 4.16 V. */
}
static void feedback_pulses(void)
{
    input_mv = 4160; input_soc = 10000;
    set_charge_source(2);
    for (int time = 0; time <= 5000; time += 500) tick(time);
    expect_battery_wire();
    for (int source = 0; source < 4; ++source) {
        int64_t origin = 6000 + source * 1000;
        if (source < 2) set_charge_source(source);
        if (source == 2) NRF_POWER->USBREGSTATUS = 1;
        if (source == 3) input_pmic = true;
#if CHARGER_STAT_EXISTS
        unsigned before = stat_reads;
#endif
        tick(origin); expect_battery_wire();
#if CHARGER_STAT_EXISTS
        assert(stat_reads - before == (source == 0 ? 1U : 2U));
#endif
        tick(origin + 399); expect_battery_wire();
        set_charge_source(2); NRF_POWER->USBREGSTATUS = 0; input_pmic = false;
        tick(origin + 400); expect_battery_wire();
        tick(origin + 900); expect_battery_wire();
    }
}
static void feedback_presence(void)
{
    input_mv = 4160; input_soc = 10000;
    set_charge_source(2);
    tick(0); expect_battery_wire();
    /* Generic STAT/CHG is valid evidence without USB or a high VIN voltage. */
    set_charge_source(0);
    tick(100); expect_battery_wire();
    tick(599); expect_battery_wire();
    tick(600); expect_feedback(LED_POWER_CHARGING, false);
    assert(reported_plugged && status_plugged && batt_v == 186);
    set_charge_source(1);
    tick(700); expect_feedback(LED_POWER_CHARGED, true);
    assert(reported_plugged && batt == 255);
    set_charge_source(2);
    tick(800); expect_feedback(LED_POWER_EXTERNAL_UNKNOWN, false);
    assert(reported_plugged && status_plugged);
    tick(1299); expect_feedback(LED_POWER_EXTERNAL_UNKNOWN, false);
    tick(1300); expect_battery_wire();
    set_charge_source(1);
    tick(1400); expect_battery_wire();
    tick(1900); expect_feedback(LED_POWER_CHARGED, true);
    set_charge_source(2);
    tick(2000); expect_feedback(LED_POWER_EXTERNAL_UNKNOWN, false);
    tick(2500); expect_battery_wire();
}
static void feedback_partial_case(bool failed_charge, bool boot_error)
{
    input_mv = boot_error ? 3200 : 4160;
    input_soc = boot_error ? 0 : 10000;
    NRF_POWER->USBREGSTATUS = 0;
    input_charging = !failed_charge;
    input_charged = failed_charge;
    enum led_power_state qualified = failed_charge ? LED_POWER_CHARGED : LED_POWER_CHARGING;
    if (!boot_error) {
        tick(0); expect_feedback(qualified, failed_charge);
        assert(charge_reads == 1 && standby_reads == 1); /* No extra reads on success. */
        tick(500); expect_feedback(qualified, failed_charge);
    }
    int *read_error = failed_charge ? &charge_read_error : &standby_error;
    *read_error = -EIO;
    expect_snapshot_error(-EIO); /* Public outputs remain all-or-nothing. */
    int64_t error_at = boot_error ? 0 : 600;
    tick(error_at); expect_feedback(LED_POWER_EXTERNAL_UNKNOWN, false);
    assert(power_battery_device_plugged() && status_plugged && reported_plugged);
    assert((batt_v + 245) * 10 == 4310); /* Live serializer's external-power marker. */
    assert(batt == (boot_error ? 0x80 : (0x80 | 100)));
    tick(error_at + 500); expect_feedback(LED_POWER_EXTERNAL_UNKNOWN, false);
    assert(power_battery_device_plugged() && status_plugged && reported_plugged);
    assert(shutdowns == 0); /* Also protects startup at a genuine 0% SOC. */
    *read_error = 0;
    tick(error_at + 600); expect_feedback(qualified, failed_charge);
    if (boot_error) return;

    /* Neither failed pin proves power. Normal removal debounce still applies. */
    gpio_error = -EIO;
    expect_snapshot_error(-EIO);
    tick(1300); expect_feedback(LED_POWER_EXTERNAL_UNKNOWN, false);
    tick(1799); expect_feedback(LED_POWER_EXTERNAL_UNKNOWN, false);
    assert(power_battery_device_plugged() && reported_plugged);
    tick(1800); expect_battery_wire();
    gpio_error = 0;
    tick(1900); expect_battery_wire();
    tick(2399); expect_battery_wire();
    tick(2400); expect_feedback(qualified, failed_charge);
    assert(power_battery_device_plugged() && status_plugged && reported_plugged);
    assert(shutdowns == 0);
}
static void feedback_partial_edges(void)
{
    input_mv = 4160; input_soc = 10000;
    standby_error = -EIO; input_charged = true;
    tick(0);
    input_charging = true; tick(100);
    tick(200); assert(attempts == 2);
    input_charging = false; tick(300);
    tick(400); assert(attempts == 3);
    charge_read_error = -EIO; standby_error = 0;
    tick(450); /* An independent STBY positive edge also samples immediately. */
    input_charged = false; tick(480);
    tick(979); assert(attempts == 5);
    tick(980); assert(attempts == 6);
    const int expected[] = {0, 100, 300, 450, 480, 980};
    for (unsigned i = 0; i < attempts; ++i) assert(attempts_at[i] == expected[i]);
    expect_battery_wire(); /* Short error-evidence pulses do not bypass debounce. */
}
#if CHARGER_STAT_EXISTS
static int raw_stat_snapshot(bool *charging, bool *charged)
{
    k_mutex_lock(&charger_stat_lock, K_FOREVER);
    int err = charger_stat_snapshot_locked(charging, charged);
    k_mutex_unlock(&charger_stat_lock);
    return err;
}
static bool stat_scenario(const char *scenario)
{
    if (!strcmp(scenario, "stat_raw_levels")) {
        for (stat_mode = 0; stat_mode <= 2; ++stat_mode) {
            bool charging = true, charged = true;
            stat_reset_counts();
            stat_sense = NRF_GPIO_PIN_SENSE_HIGH;
            assert(raw_stat_snapshot(&charging, &charged) == 0);
            assert(charging == (stat_mode == 0) && charged == (stat_mode == 1));
            assert(stat_reads == (stat_mode == 0 ? 1U : 2U));
            assert(stat_configs == (stat_mode == 0 ? 1U : 3U) && stat_waits == stat_reads);
            assert(stat_waits <= 2 && stat_bias == GPIO_PULL_UP);
            assert(stat_sense == NRF_GPIO_PIN_NOSENSE);
            assert(!stat_locked);
        }
    } else if (!strcmp(scenario, "stat_pulses")) {
        feedback_pulses();
    } else if (!strcmp(scenario, "stat_presence")) {
        feedback_presence();
    } else if (!strcmp(scenario, "stat_config_readback")) {
        stat_mode = 1;
        for (stat_ignore_config = 1; stat_ignore_config <= 3; ++stat_ignore_config) {
            for (int sentinel = 0; sentinel < 2; ++sentinel) {
                bool charging = sentinel, charged = !sentinel;
                stat_bias = GPIO_PULL_DOWN;
                stat_reset_counts();
                assert(sys_charger_snapshot(&charging, &charged) == -EIO);
                assert(charging == !!sentinel && charged == !sentinel);
                assert(stat_reads == stat_ignore_config - 1);
                assert(stat_waits == (stat_ignore_config == 1 ? 1U : 2U));
                if (stat_ignore_config != 3) assert(stat_bias == GPIO_PULL_UP);
                else assert(stat_bias == GPIO_PULL_DOWN); /* Restore was ignored. */
            }
        }
        stat_ignore_config = 0;
        for (stat_corruption = 1; stat_corruption <= 3; ++stat_corruption) {
            for (stat_corrupt_config = 1; stat_corrupt_config <= 3; ++stat_corrupt_config) {
                expect_snapshot_error(-EIO);
                assert(stat_reads == stat_corrupt_config - 1);
            }
            stat_corrupt_config = 0;
            for (stat_corrupt_wait = 1; stat_corrupt_wait <= 2; ++stat_corrupt_wait) {
                expect_snapshot_error(-EIO);
                assert(stat_reads == stat_corrupt_wait - 1 && stat_waits == stat_corrupt_wait);
                assert(stat_bias == GPIO_PULL_UP); /* Restore after readback failure. */
            }
            stat_corrupt_wait = 0;
            bool charging = true, charged = false;
            stat_reset_counts();
            assert(sys_charger_snapshot(&charging, &charged) == 0);
            assert(!charging && charged && stat_reads == 2 && stat_bias == GPIO_PULL_UP);
        }
    } else if (!strcmp(scenario, "stat_latch")) {
        stat_mode = 0; stat_latched = true;
        bool charging = false, charged = true;
        stat_reset_counts();
        unsigned before = stat_latch_clears;
        assert(sys_charger_snapshot(&charging, &charged) == 0);
        assert(charging && !charged && stat_latch_clears == before + 1);
        assert(!stat_latched && !irq_depth && stat_sense == NRF_GPIO_PIN_NOSENSE);
    } else if (!strcmp(scenario, "stat_transitions")) {
        /* Model physical sources, not preset bits: UP-high/DOWN-low may be
         * high-Z or a transition, so neither observation establishes charge. */
        const int transitions[][2] = {{1, 2}, {1, 0}, {2, 1}, {2, 0}};
        stat_source_count = 2;
        for (unsigned i = 0; i < sizeof(transitions) / sizeof(transitions[0]); ++i) {
            stat_sources[0] = transitions[i][0];
            stat_sources[1] = transitions[i][1];
            bool charging = true, charged = true;
            stat_reset_counts();
            assert(sys_charger_snapshot(&charging, &charged) == 0);
            /* HIZ->high establishes full at the last observation; no claim of
             * stability or knowledge of changes after that observation. */
            assert(!charging && charged == (stat_sources[1] == 1));
            assert(stat_reads == 2 && stat_waits == 2 && stat_configs == 3);
            assert(stat_bias == GPIO_PULL_UP);
        }
        stat_sources[0] = 0; stat_sources[1] = 1;
        bool charging = false, charged = true;
        stat_reset_counts();
        assert(sys_charger_snapshot(&charging, &charged) == 0);
        assert(charging && !charged && stat_reads == 1 && stat_waits == 1 && stat_configs == 1);
        assert(stat_bias == GPIO_PULL_UP); /* Positive low already has the safe idle bias. */
    } else if (!strcmp(scenario, "stat_errors")) {
        NRF_POWER->USBREGSTATUS = 1;
        stat_mode = 1;
        for (stat_fail_read = 1; stat_fail_read <= 2; ++stat_fail_read) {
            expect_snapshot_error(-EIO);
            assert(stat_reads == stat_fail_read && stat_configs == stat_fail_read + 1);
            assert(stat_waits == stat_fail_read);
            assert(stat_bias == GPIO_PULL_UP && stat_sense == NRF_GPIO_PIN_NOSENSE);
        }
        stat_fail_read = 0;
        for (stat_fail_config = 1; stat_fail_config <= 3; ++stat_fail_config) {
            expect_snapshot_error(-EIO);
            assert(stat_configs == (stat_fail_config < 3 ? stat_fail_config + 1 : 3));
            assert(stat_reads == stat_fail_config - 1 && stat_waits == stat_reads);
            assert(stat_bias == (stat_fail_config == 3 ? GPIO_PULL_DOWN : GPIO_PULL_UP));
        }
        stat_fail_config = 0;
        /* A restore error wins over an earlier raw read error. Recovery remains
         * possible because ordinary sampling has not transferred pin ownership. */
        stat_fail_read = 2; stat_read_error = -EINVAL;
        stat_fail_config = 3; stat_config_error = -EPERM;
        expect_snapshot_error(-EPERM);
        assert(stat_reads == 2 && stat_configs == 3 && stat_bias == GPIO_PULL_DOWN);
        stat_fail_config = 0;
        expect_snapshot_error(-EINVAL);
        assert(stat_bias == GPIO_PULL_UP);
        stat_fail_read = 0; stat_read_error = stat_config_error = -EIO;
        /* False-success restoration must outrank a configuration error too. */
        stat_fail_config = 1; stat_config_error = -EINVAL;
        stat_ignore_config = 2; stat_bias = GPIO_PULL_DOWN;
        expect_snapshot_error(-EIO);
        assert(stat_reads == 0 && stat_waits == 0 && stat_configs == 2);
        assert(stat_bias == GPIO_PULL_DOWN);
        stat_ignore_config = 0;
        expect_snapshot_error(-EINVAL);
        assert(stat_bias == GPIO_PULL_UP);
        stat_fail_config = 0;
        /* The same restore precedence applies to post-wait readback errors. */
        stat_corruption = 3; stat_corrupt_wait = 1;
        stat_fail_config = 2; stat_config_error = -EPERM;
        expect_snapshot_error(-EPERM);
        assert(stat_reads == 0 && stat_waits == 1 && stat_configs == 2);
        stat_fail_config = 0;
        expect_snapshot_error(-EIO);
        assert(stat_bias == GPIO_PULL_UP);
        stat_corrupt_wait = 0; stat_config_error = -EIO;
        gpio_available = false;
        expect_snapshot_error(-ENODEV);
        assert(stat_reads == 0 && stat_configs == 0);
        gpio_available = true;
        stat_fail_config = 1;
        stat_reset_counts();
        assert(sys_gpio_init() == 0); /* A failed init is not a false full fact. */
        expect_snapshot_error(-EIO);
        stat_fail_config = 0;
        stat_reset_counts();
        bool charging = true, charged = false;
        assert(sys_charger_snapshot(&charging, &charged) == 0);
        assert(!charging && charged && !charger_stat_sense_armed && !stat_locked);
        /* A failed ordinary sample never freezes acquisition before the
         * irreversible sleep gate (including an aborted WOM proposal). */
    } else if (!strcmp(scenario, "stat_owner")) {
        NRF_POWER->USBREGSTATUS = 1;
        for (int mode = 0; mode <= 2; ++mode) {
            stat_mode = mode;
            stat_reset_counts();
            tick(mode * 100);
            expect_feedback(mode == 0 ? LED_POWER_CHARGING :
                            mode == 1 ? LED_POWER_CHARGED : LED_POWER_EXTERNAL_UNKNOWN,
                            mode == 1);
            assert(stat_reads == (mode == 0 ? 1U : 2U)); /* One shared snapshot. */
            assert(attempts == (unsigned)(mode + 1)); /* Same snapshot drives ADC edges. */
        }
        stat_mode = 1; tick(300); expect_feedback(LED_POWER_CHARGED, true);
        stat_reset_counts(); stat_fail_read = 2;
        tick(400); expect_feedback(LED_POWER_EXTERNAL_UNKNOWN, false);
        stat_fail_read = 0; stat_mode = 2; NRF_POWER->USBREGSTATUS = 0;
        tick(500); expect_feedback(LED_POWER_EXTERNAL_UNKNOWN, false);
        tick(1100); expect_feedback(LED_POWER_BATTERY, false); assert(!reported_plugged);
        input_mv = 4400;
        tick(1600); expect_feedback(LED_POWER_BATTERY, false);
        assert(vin_read()); /* Generic VIN remains independent source evidence. */
        stat_mode = 0; tick(1700); expect_feedback(LED_POWER_BATTERY, false);
        stat_mode = 1; tick(1800); expect_feedback(LED_POWER_BATTERY, false);
        tick(2100); expect_feedback(LED_POWER_CHARGED, true); assert(reported_plugged);
    } else if (!strcmp(scenario, "stat_boot")) {
        power_init = true;
        const int voltages[] = {3800, 4400, 6200};
        for (unsigned i = 0; i < sizeof(voltages) / sizeof(voltages[0]); ++i) {
            input_mv = voltages[i];
            for (stat_mode = 0; stat_mode <= 2; ++stat_mode) {
                NRF_POWER->USBREGSTATUS = 0;
                plugged = false;
                tick(i * 3000 + stat_mode * 1000);
                boot_shutdowns = 0; reset_mode = 0;
                stat_reset_counts(); boot_shutdown_check();
                assert(boot_shutdowns == (unsigned)(stat_mode == 2 && voltages[i] != 4400));
                assert(stat_reads == (stat_mode == 0 ? 1U : 2U));
                plugged = true; boot_shutdowns = 0;
                boot_shutdown_check();
                assert(boot_shutdowns == 0);
                NRF_POWER->USBREGSTATUS = 1; boot_shutdowns = 0;
                boot_shutdown_check(); assert(boot_shutdowns == 0);
            }
        }
        NRF_POWER->USBREGSTATUS = 0; gpio_available = false; plugged = true;
        boot_shutdowns = 0;
        boot_shutdown_check(); assert(boot_shutdowns == 0);
    } else if (!strncmp(scenario, "stat_wake_", 10)) {
        stat_mode = !strcmp(scenario, "stat_wake_low") ? 0 :
                    !strcmp(scenario, "stat_wake_high") ||
                    !strcmp(scenario, "stat_wake_bias_error") ? 1 : 2;
        input_dock = !strcmp(scenario, "stat_wake_dock");
        host_ignore_charge_wake = !strcmp(scenario, "stat_wake_ignore");
        if (host_ignore_charge_wake) NRF_POWER->USBREGSTATUS = 1;
        if (!strcmp(scenario, "stat_wake_error")) stat_fail_read = 1;
        if (!strcmp(scenario, "stat_wake_bias_error")) stat_fail_config = 4;
        stat_reset_counts();
        unsigned expected_reads = input_dock || host_ignore_charge_wake ? 0U :
                                  stat_mode == 0 || stat_fail_read ? 1U : 2U;
        unsigned expected_configs = input_dock || host_ignore_charge_wake ? 0U :
                                    stat_fail_config ? 5U : stat_fail_read ? 2U :
                                    stat_mode == 0 ? 1U : stat_mode == 1 ? 4U : 3U;
        configure_sense_pins();
        bool skipped = input_dock || host_ignore_charge_wake || stat_fail_read || stat_fail_config;
        assert(stat_reads == expected_reads && stat_waits == expected_reads);
        assert(stat_configs == expected_configs);
        assert(stat_sense == (skipped ? NRF_GPIO_PIN_NOSENSE :
                              stat_mode == 0 ? NRF_GPIO_PIN_SENSE_HIGH : NRF_GPIO_PIN_SENSE_LOW));
        if (!skipped) {
            int physical = stat_mode == 2 ? stat_bias == GPIO_PULL_UP : stat_mode;
            assert(stat_sense != (physical ? NRF_GPIO_PIN_SENSE_HIGH : NRF_GPIO_PIN_SENSE_LOW));
        }
        assert(stat_bias == (!skipped && stat_mode == 1 ? GPIO_PULL_DOWN : GPIO_PULL_UP));
        unsigned reads_before = stat_reads, configs_before = stat_configs;
        int sense_before = stat_sense, bias_before = stat_bias;
        for (int sentinel = 0; sentinel < 2; ++sentinel) {
            bool charging = sentinel, charged = !sentinel;
            assert(sys_charger_snapshot(&charging, &charged) == -EBUSY);
            assert(charging == !!sentinel && charged == !sentinel);
        }
        assert(stat_reads == reads_before && stat_configs == configs_before);
        assert(stat_sense == sense_before && stat_bias == bias_before);
        assert(charger_stat_sense_armed && !stat_locked);
        /* Model the global GPIOTE ISR changing the live SENSE after ownership.
         * SYSTEMOFF must restore saved intent, including skipped/failed wake. */
        stat_sense = sense_before == NRF_GPIO_PIN_SENSE_LOW ?
                     NRF_GPIO_PIN_SENSE_HIGH : NRF_GPIO_PIN_SENSE_LOW;
        stat_latched = true;
        sys_enter_poweroff();
        assert(physical_offs == 1 && stat_sense == sense_before && !stat_latched && !irq_depth);
        assert(sys_charger_snapshot(&(bool){false}, &(bool){false}) == -EBUSY);
    } else {
        return false;
    }
    return true;
}
#endif
int main(int argc, char **argv)
{
    assert(argc == 2);
    const char *scenario = argv[1];
    assert(sys_gpio_init() == 0);
    reset_mode = -1;
    power_init = true;
    boot_shutdown_check();
    assert(boot_shutdowns == 0);
    charge_reads = standby_reads = 0; /* Measure owner reads separately from boot policy. */
    power_init = false;
#if CHARGER_STAT_EXISTS
    if (stat_scenario(scenario)) return 0;
#endif
    if (!strcmp(scenario, "poweroff")) {
        sys_enter_poweroff();
        assert(physical_offs == 1 && !irq_depth);
        return 0;
    }
    if (!strcmp(scenario, "feedback_pulses")) {
        feedback_pulses();
        return 0;
    }
    if (!strcmp(scenario, "feedback_presence")) {
        feedback_presence();
        return 0;
    }
#if TEST_PMIC
    if (!strcmp(scenario, "feedback_pmic_freshness")) {
        input_pmic = input_charged = true;
        tick(0); expect_feedback(LED_POWER_CHARGED, true);
        input_charging = true;
        tick(100); expect_feedback(LED_POWER_CHARGING, false);
        assert(pmic_fetches == attempts && reported_plugged);
        input_charging = false;
        tick(200); expect_feedback(LED_POWER_CHARGED, true);
        input_soc = -EIO;
        tick(300); expect_feedback(LED_POWER_EXTERNAL_UNKNOWN, false);
        assert(reported_plugged && !charger_sample_valid);
        input_soc = 10000; charger_error = -EIO;
        tick(400); expect_feedback(LED_POWER_EXTERNAL_UNKNOWN, false);
        assert(reported_plugged && charger_sample_valid);
        charger_error = 0;
        tick(500); expect_feedback(LED_POWER_CHARGED, true);
        assert(pmic_fetches == attempts); /* Snapshot never fetches again. */
        assert(charge_reads == 0 && standby_reads == 0); /* PMIC errors never fall back to GPIO. */
        return 0;
    }
#endif
    if (!strcmp(scenario, "feedback_no_gpio")) {
        NRF_POWER->USBREGSTATUS = 1; input_soc = 10000;
        expect_snapshot_error(-ENOTSUP);
        tick(0); expect_feedback(LED_POWER_EXTERNAL_UNKNOWN, false);
        tick(600); assert(reported_plugged);
        NRF_POWER->USBREGSTATUS = 0;
        tick(700); expect_feedback(LED_POWER_EXTERNAL_UNKNOWN, false);
        assert(reported_plugged);
        tick(1300); expect_feedback(LED_POWER_BATTERY, false); assert(!reported_plugged);
        return 0;
    }
    if (!strcmp(scenario, "feedback_unknown")) {
        input_soc = 10000; NRF_POWER->USBREGSTATUS = 1;
        gpio_available = false; charger_error = -EIO;
        tick(0);
        assert(led_test_power == LED_POWER_EXTERNAL_UNKNOWN && !led_test_low);
        input_charging = input_charged = true;
        tick(100);
        assert(led_test_power == LED_POWER_EXTERNAL_UNKNOWN);
    } else if (!strcmp(scenario, "feedback_charge")) {
        input_charging = input_charged = true;
        tick(0); assert(led_test_power == LED_POWER_CHARGING);
        input_charging = false;
        tick(100); assert(led_test_power == LED_POWER_CHARGED);
        gpio_error = -EIO; charger_error = -EIO;
        NRF_POWER->USBREGSTATUS = 1; /* Independent evidence survives status failure. */
        tick(200); assert(led_test_power == LED_POWER_EXTERNAL_UNKNOWN);
    } else if (!strcmp(scenario, "feedback_low")) {
        input_soc = 500;
        tick(0); assert(led_test_low && led_test_power == LED_POWER_BATTERY);
        NRF_POWER->USBREGSTATUS = 1; gpio_available = false; charger_error = -EIO;
        tick(100); assert(led_test_low && led_test_power == LED_POWER_BATTERY);
        gpio_available = true; charger_error = 0; input_charging = true;
        tick(200); assert(led_test_low && led_test_power == LED_POWER_BATTERY);
        tick(600); assert(led_test_power == LED_POWER_CHARGING);
    } else if (!strcmp(scenario, "feedback_partial") ||
               !strcmp(scenario, "feedback_partial_charge") ||
               !strcmp(scenario, "feedback_partial_boot_chg") ||
               !strcmp(scenario, "feedback_partial_boot_stby")) {
        bool failed_charge = !strcmp(scenario, "feedback_partial_charge") ||
                             !strcmp(scenario, "feedback_partial_boot_stby");
        feedback_partial_case(failed_charge, strstr(scenario, "_boot_") != NULL);
    } else if (!strcmp(scenario, "feedback_partial_edges")) {
        feedback_partial_edges();
    } else if (!strcmp(scenario, "feedback_board")) {
        gpio_use_raw = true;
        raw_plug_level = raw_charge_level = 0;
        tick(0); expect_feedback(LED_POWER_CHARGING, false);
        raw_charge_level = 1;
        tick(100); expect_feedback(LED_POWER_CHARGED, true);
        tick(600);
        assert(reported_plugged); /* Same full fact reaches battery telemetry. */
        raw_charge_level = 0;
        tick(700); expect_feedback(LED_POWER_CHARGING, false);
        raw_plug_level = 1; /* CHG wins even an inactive PLUG conflict. */
        tick(800); expect_feedback(LED_POWER_CHARGING, false);
        raw_charge_level = 1;
        tick(900); expect_feedback(LED_POWER_EXTERNAL_UNKNOWN, false);
        NRF_POWER->USBREGSTATUS = 1;
        tick(1000); expect_feedback(LED_POWER_EXTERNAL_UNKNOWN, false);
        raw_plug_level = 0; input_soc = 5000;
        tick(1100); expect_feedback(LED_POWER_CHARGED, true);
    } else if (!strcmp(scenario, "feedback_board_errors")) {
        gpio_use_raw = true;
        NRF_POWER->USBREGSTATUS = 1;
        raw_plug_level = 0; raw_charge_level = 1;
        tick(0); expect_feedback(LED_POWER_CHARGED, true);
        gpio_error = -EIO;
        expect_snapshot_error(-EIO);
        tick(100); expect_feedback(LED_POWER_EXTERNAL_UNKNOWN, false);
        gpio_error = 0; plug_error = -EIO;
        expect_snapshot_error(-EIO);
        tick(200); expect_feedback(LED_POWER_EXTERNAL_UNKNOWN, false);
        raw_charge_level = 0; /* No partial CHG commit if the PLUG read fails. */
        expect_snapshot_error(-EIO);
        tick(300); expect_feedback(LED_POWER_EXTERNAL_UNKNOWN, false);
        plug_error = 0; gpio_available = false;
        expect_snapshot_error(-ENODEV);
        tick(400); expect_feedback(LED_POWER_EXTERNAL_UNKNOWN, false);
        gpio_available = true; plug_available = false;
        expect_snapshot_error(-ENODEV);
        tick(500); expect_feedback(LED_POWER_EXTERNAL_UNKNOWN, false);
        plug_available = true;
        tick(600); expect_feedback(LED_POWER_CHARGING, false);
        raw_charge_level = 1;
        tick(700); expect_feedback(LED_POWER_CHARGED, true);
    } else if (!strcmp(scenario, "feedback_board_initfail")) {
        gpio_use_raw = true;
        NRF_POWER->USBREGSTATUS = 1;
        raw_plug_level = 0; raw_charge_level = 1;
        plug_config_error = -EIO;
        assert(sys_gpio_init() == 0);
        expect_snapshot_error(-EIO);
        tick(0); expect_feedback(LED_POWER_EXTERNAL_UNKNOWN, false);
        plug_config_error = 0; plug_available = false;
        assert(sys_gpio_init() == 0);
        expect_snapshot_error(-ENODEV);
        tick(100); expect_feedback(LED_POWER_EXTERNAL_UNKNOWN, false);
        plug_available = true;
        assert(sys_gpio_init() == 0);
        tick(200); expect_feedback(LED_POWER_CHARGED, true);
    } else if (!strcmp(scenario, "feedback_chg_only")) {
        gpio_use_raw = true;
        raw_plug_level = raw_charge_level = 0;
        tick(0); expect_feedback(LED_POWER_CHARGING, false);
        raw_charge_level = 1;
        expect_snapshot_error(-ENOTSUP);
        tick(100); expect_feedback(LED_POWER_EXTERNAL_UNKNOWN, false);
        NRF_POWER->USBREGSTATUS = 1; raw_plug_level = 1;
        expect_snapshot_error(-ENOTSUP);
        tick(200); expect_feedback(LED_POWER_EXTERNAL_UNKNOWN, false);
    } else
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
        /* Charging-to-full advances ADC deadline without changing raw plugged. */
        input_charging = false; input_charged = true; tick(550);
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
