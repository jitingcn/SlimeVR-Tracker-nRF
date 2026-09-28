/* Appended to the controller fixture: GPIO is adapted, admission is real. */
#define DT_ALIAS(name) DT_ALIAS_##name
#define DT_ALIAS_sw0 1
#define DT_ALIAS_sw1 TEST_SW1
#define DT_ALIAS_heater_button TEST_HEATER_ALIAS
#define DT_NODE_HAS_PROP(node, prop) (node)
#undef CONFIG_SENSOR_TCAL_HEATED
#define CONFIG_SENSOR_TCAL_HEATED TEST_HEAT
#define CONFIG_SENSOR_TCAL_HEATED_DEFAULT_TARGET_C 44

#if TEST_HEAT && TEST_HEATER_ALIAS
struct device { int unused; };
struct gpio_callback { int unused; };
struct gpio_dt_spec { const struct device *port; unsigned pin; };
#define GPIO_DT_SPEC_GET(node, prop) {NULL, 13}
#define GPIO_INPUT 0
#define GPIO_INT_EDGE_BOTH 1
#define ARG_UNUSED(x) (void)(x)
#define BIT(x) (1U << (x))
#define SYS_INIT(fn, level, priority)
static int gpio_level, gpio_error;
static unsigned gpio_reads;
static bool interrupt_during_read;
static void heated_button_interrupt(const struct device *, struct gpio_callback *, uint32_t);
static bool gpio_is_ready_dt(const struct gpio_dt_spec *spec) { (void)spec; return true; }
static int gpio_pin_configure_dt(const struct gpio_dt_spec *spec, int flags)
{ (void)spec; (void)flags; return gpio_error; }
static void gpio_init_callback(struct gpio_callback *cb,
    void (*fn)(const struct device *, struct gpio_callback *, uint32_t), unsigned mask)
{ (void)cb; (void)fn; (void)mask; }
static int gpio_add_callback(const struct device *dev, struct gpio_callback *cb)
{ (void)dev; (void)cb; return 0; }
static int gpio_remove_callback(const struct device *dev, struct gpio_callback *cb)
{ (void)dev; (void)cb; return 0; }
static int gpio_pin_interrupt_configure_dt(const struct gpio_dt_spec *spec, int flags)
{ (void)spec; (void)flags; return 0; }
static int gpio_pin_get_dt(const struct gpio_dt_spec *spec)
{
    (void)spec;
    gpio_reads++;
    if (interrupt_during_read) {
        interrupt_during_read = false;
        heated_button_interrupt(NULL, NULL, 0); /* release */
        heated_button_interrupt(NULL, NULL, 0); /* repress */
    }
    return gpio_level;
}
#endif

/* PRODUCTION_BUTTON */

#if TEST_HEAT && TEST_HEATER_ALIAS
static void poll_button(int level, int64_t dt, bool edge)
{
    now_ms += dt;
    sample.sampled_at_ms = now_ms;
    sample.sequence++;
    gpio_level = level;
    if (edge) heated_button_interrupt(NULL, NULL, 0);
    heated_button_poll(ota_active || ota_suppressed);
}
static void release_button(void)
{
    poll_button(0, 0, true);
    poll_button(0, 50, false);
}
static void long_press(void)
{
    poll_button(1, 0, true);
    poll_button(1, 3000, false);
}
static void finish_session(void)
{
    assert(sensor_tcal_heated_abort(TCAL_HEATED_STOP_TIMEOUT) == 0);
    assert(!sensor_tcal_heated_busy());
}
#endif

int main(void)
{
#if TEST_HEAT && TEST_HEATER_ALIAS
    sensor_tcal_heated_set_ready(true);
    gpio_error = -EIO;
    assert(sys_heated_button_init() == -EIO);
    heated_button_poll(false);
    assert(gpio_reads == 0);
    gpio_error = 0;
    assert(sys_heated_button_init() == 0);

    sensor_calibration_identity_accel(retained->accBAinv);
    sensor_calibration_imu_load();
    sensor_calibration_set_consumer_ready(true);
    /* Boot-held, regardless of uptime: require a stable release. */
    long_press();
    poll_button(1, 10000, false);
    assert(!sensor_tcal_heated_busy());
    poll_button(0, 0, true);
    poll_button(0, 49, false);
    long_press();
    assert(!sensor_tcal_heated_busy());
    release_button();
    poll_button(1, 0, true);
    poll_button(1, 2999, false);
    assert(!sensor_tcal_heated_busy());
    poll_button(1, 1, false);
    assert(sensor_tcal_heated_busy() && imu_reserved && heat.target == 44.0f);
    uint32_t epoch = heat.epoch;
    poll_button(1, 5000, false);
    assert(heat.epoch == epoch);
    finish_session();
    poll_button(1, 5000, false);
    assert(!sensor_tcal_heated_busy());

    /* ISR release/repress inside GPIO read cannot reuse the old deadline. */
    release_button();
    poll_button(1, 0, true);
    interrupt_during_read = true;
    poll_button(1, 3000, false);
    assert(!sensor_tcal_heated_busy());
    poll_button(1, 2999, false);
    assert(!sensor_tcal_heated_busy());
    poll_button(1, 1, false);
    assert(sensor_tcal_heated_busy());
    finish_session();

    /* A brief release/repress between polls invalidates continuous hold. */
    release_button();
    poll_button(1, 0, true);
    poll_button(1, 2999, true);
    poll_button(1, 2999, false);
    assert(!sensor_tcal_heated_busy());
    poll_button(1, 1, false);
    assert(sensor_tcal_heated_busy());
    finish_session();

    /* Both OTA vetoes consume the gesture, even after the veto disappears. */
    for (int suppression = 0; suppression < 2; suppression++) {
        release_button();
        ota_active = !suppression;
        ota_suppressed = suppression;
        long_press();
        assert(!sensor_tcal_heated_busy());
        ota_active = ota_suppressed = false;
        poll_button(1, 5000, false);
        assert(!sensor_tcal_heated_busy());
    }
    /* An active session isn't stopped/restarted, nor retried after completion. */
    release_button();
    assert(sensor_tcal_heated_start(44) == 0);
    epoch = heat.epoch;
    long_press();
    assert(sensor_tcal_heated_busy() && heat.epoch == epoch);
    finish_session();
    poll_button(1, 5000, false);
    assert(!sensor_tcal_heated_busy());

    /* Canonical guard failure consumes the press too. */
    release_button();
    external_power = false;
    long_press();
    assert(!sensor_tcal_heated_busy());
    external_power = true;
    poll_button(1, 5000, false);
    assert(!sensor_tcal_heated_busy());

    /* GPIO errors cannot extend a hold or rearm a consumed gesture. */
    release_button();
    poll_button(1, 0, true);
    poll_button(-EIO, 2999, false);
    poll_button(1, 5000, false);
    poll_button(1, 5000, false);
    assert(!sensor_tcal_heated_busy());
    release_button();
    long_press();
    assert(sensor_tcal_heated_busy());
#else
    /* Compile the actual gate without GPIO adapters: P10 has no sw1 owner. */
    assert(!sensor_tcal_heated_busy() && !imu_reserved);
#endif
    puts("Heated button behavior passed");
    return 0;
}
