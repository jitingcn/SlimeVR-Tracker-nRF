#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#define USE_PMIC_CHARGER 1
#define BIT(n) (1U << (n))
#define ARG_UNUSED(value) ((void)(value))
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
static int64_t clock_ms;
static unsigned fetches;
static bool external_present = true;
static unsigned hardware_status, fetched_status;
static int fetch_error, voltage_error, vbus_error, status_error;
static int64_t k_uptime_get(void) { return clock_ms; }
static int sensor_sample_fetch(const struct device *device)
{
    assert(device == charger);
    ++fetches;
    if (fetch_error) return fetch_error;
    fetched_status = hardware_status;
    return 0;
}
static int sensor_channel_get(const struct device *device, int channel, struct sensor_value *value)
{
    assert(device == charger);
    if (channel == SENSOR_CHAN_GAUGE_VOLTAGE) {
        if (voltage_error) return voltage_error;
        *value = (struct sensor_value){3, 800000};
        return 0;
    }
    assert(channel == SENSOR_CHAN_NPM13XX_CHARGER_STATUS);
    if (status_error) return status_error;
    *value = (struct sensor_value){fetched_status, 0};
    return 0;
}
static int sensor_attr_get(const struct device *device, int channel, int attribute,
                           struct sensor_value *value)
{
    assert(device == charger);
    assert(channel == SENSOR_CHAN_NPM13XX_CHARGER_VBUS_STATUS);
    assert(attribute == SENSOR_ATTR_NPM13XX_CHARGER_VBUS_PRESENT);
    if (vbus_error) return vbus_error;
    *value = (struct sensor_value){external_present, 0};
    return 0;
}
static int sensor_value_to_milli(const struct sensor_value *value)
{
    return value->val1 * 1000 + value->val2 / 1000;
}
#include "pmic-production.inc"

static void expect_snapshot(bool external, bool active, bool full)
{
    bool plugged = !external, charging = !active, charged = !full;
    unsigned before = fetches;
    assert(battery_charger_snapshot(&plugged, &charging, &charged) == 0);
    assert(plugged == external && charging == active && charged == full);
    assert(fetches == before); /* Reading facts must not move the owner cadence. */
}
static void expect_error(int error)
{
    /* Exercise both sentinel patterns: no partial tuple may escape on error. */
    for (int sentinel = 0; sentinel <= 1; ++sentinel) {
        bool plugged = sentinel, charging = !sentinel, charged = sentinel;
        assert(battery_charger_snapshot(&plugged, &charging, &charged) == error);
        assert(plugged == !!sentinel && charging == !sentinel && charged == !!sentinel);
    }
}
static void sample(unsigned status)
{
    hardware_status = status;
    assert(battery_sample() == 3800);
}
int main(int argc, char **argv)
{
    assert(argc == 2);
    const char *scenario = argv[1];
    battery_ok = true;
    if (!strcmp(scenario, "charging")) {
        /* Trickle, constant-current and constant-voltage all mean active. */
        for (unsigned bit = 2; bit <= 4; ++bit) {
            sample(BIT(bit));
            expect_snapshot(true, true, false);
            sample(BIT(bit) | BIT(1));
            expect_snapshot(true, true, false); /* Active wins completion conflict. */
        }
    } else if (!strcmp(scenario, "full")) {
        sample(BIT(1));
        expect_snapshot(true, false, true);
        sample(BIT(3));
        expect_snapshot(true, true, false);
        sample(BIT(1));
        expect_snapshot(true, false, true);
    } else if (!strcmp(scenario, "unknown")) {
        sample(0);
        expect_snapshot(true, false, false); /* Supply alone is not completion. */
        external_present = false;
        sample(0);
        expect_snapshot(false, false, false);
    } else if (!strcmp(scenario, "readfail")) {
        sample(BIT(1));
        hardware_status = BIT(3);
        voltage_error = -EIO;
        assert(battery_sample() == -EIO);
        expect_error(-EAGAIN); /* A partial new fetch must not certify freshness. */
        voltage_error = 0;
        sample(BIT(3));
        expect_snapshot(true, true, false);
        fetch_error = -EIO;
        assert(battery_sample() == -EIO);
        expect_error(-EAGAIN);
        fetch_error = 0;
        sample(BIT(1));
        vbus_error = -EIO;
        expect_error(-EIO);
        vbus_error = 0; status_error = -EIO;
        expect_error(-EIO); /* VBUS was readable but the rest of the tuple was not. */
        status_error = 0;
        expect_snapshot(true, false, true);
        battery_ok = false;
        assert(battery_sample() == -ENODEV);
        expect_error(-EAGAIN);
        battery_ok = true;
        sample(BIT(3));
        expect_snapshot(true, true, false);
    } else if (!strcmp(scenario, "freshness")) {
        expect_error(-EAGAIN);
        clock_ms = 100;
        sample(BIT(3));
        clock_ms = 1600;
        expect_snapshot(true, true, false);
        clock_ms = 1601;
        expect_error(-EAGAIN);
        clock_ms = 10000;
        expect_error(-EAGAIN);
        sample(BIT(1));
        expect_snapshot(true, false, true);
    } else {
        assert(!"unknown scenario");
    }
    printf("actual PMIC snapshot %s passed\n", scenario);
    return 0;
}
