#include "model.h"
#include "production.inc"

static void reset_model(void)
{
	memset(&timer_registers, 0, sizeof(timer_registers));
	memset(&pwm_registers, 0, sizeof(pwm_registers));
	memset(ppi, 0, sizeof(ppi));
	memset(&state, 0, sizeof(state));
	memset(&heater_lock, 0, sizeof(heater_lock));
	fatal_error = 0;
	timer_ready = connection_ready = have_sample = false;
	last_sequence = 0;
	last_sample_ms = 0;
	clock_us = 100000;
	pwm_registers.stop_at_us = -1;
	gpio_high = irq_masked = clock_failed = pm_busy = false;
	ready = true;
	init_error = alloc_fail_at = allocations = set_error = pm_error = 0;
	extra_pin = UINT32_MAX;
	power_state = PM_DEVICE_STATE_ACTIVE;
	before_pwm_start = after_pwm_start = before_cc_write = NULL;
	after_timer_clear = after_timer_enable = NULL;
}

static void boot(void)
{
	reset_model();
	assert(heater_init() == 0);
	assert(heater_hw_available());
	assert(!pin_heating());
}

static void energize(void)
{
	assert(heater_hw_arm(1) == 0);
	assert(!pin_heating());
	assert(heater_hw_write(1, 1, k_uptime_get(), 25.0f, 3000) == 0);
	assert(pin_heating());
}

static void lease_and_freshness(void)
{
	boot();
	assert(heater_hw_arm(1) == 0);
	assert(heater_hw_write(1, UINT32_MAX, k_uptime_get(), 25.0f, 3000) == 0);
	advance_ms(1000);
	/* Same numeric temperature and sequence wrap are both valid fresh reads. */
	assert(heater_hw_write(1, 0, k_uptime_get(), 25.0f, 4000) == 0);
	advance_ms(1500);
	struct heater_hw_status status;
	heater_hw_get_status(&status);
	assert(status.armed && !status.expired && status.deadline_ms == 3100);
	assert(pin_heating());
	advance_ms(520); /* No CPU callbacks: peripheral cutoff only. */
	assert(!pin_heating());
	assert(heater_hw_expired());
	assert(heater_hw_write(1, 1, k_uptime_get(), 25.0f, 3000) == -ETIMEDOUT);
	assert(heater_hw_force_off() == 0);
	assert(heater_hw_arm(1) == -ESTALE);
	assert(heater_hw_arm(2) == 0);
	assert(!heater_hw_expired() && !pin_heating());
	assert(heater_hw_write(1, 2, k_uptime_get(), 25.0f, 1000) == -ESTALE);
	assert(!pin_heating());

	boot(); energize();
	advance_ms(1000);
	assert(heater_hw_write(1, 1, k_uptime_get(), 25.0f, 3000) == -ESTALE);
	assert(!pin_heating());
	boot(); energize();
	advance_ms(1000);
	assert(heater_hw_write(1, 0, k_uptime_get(), 25.0f, 3000) == -ESTALE);
	assert(!pin_heating());

	boot(); energize();
	advance_ms(1000);
	/* New sequence cannot make the old timestamp younger. */
	assert(heater_hw_write(1, 2, 100, 25.0f, 3000) == 0);
	advance_ms(1020);
	assert(!pin_heating() && heater_hw_expired());
}

static void rejected_inputs(void)
{
	const uint32_t bad_bits[] = {0x7fc00000U, 0x7f800000U, 0xff800000U};
	for (unsigned i = 0; i < 3; ++i) {
		float value; memcpy(&value, &bad_bits[i], sizeof(value));
		boot(); energize();
		assert(heater_hw_write(1, 2, k_uptime_get(), value, 3000) == -ERANGE);
		assert(!pin_heating());
	}
	const uint16_t duties[] = {7001, 9999, 10000, UINT16_MAX};
	for (unsigned i = 0; i < 4; ++i) {
		boot(); energize();
		assert(heater_hw_write(1, 2, k_uptime_get(), 25.0f, duties[i]) == -ERANGE);
		assert(!pin_heating());
	}
	boot(); energize();
	assert(heater_hw_write(1, 2, k_uptime_get(), 50.0f, 3000) == -ERANGE);
	assert(!pin_heating());
	boot(); energize();
	assert(heater_hw_write(1, 2, k_uptime_get() + 1, 25.0f, 3000) == -ESTALE);
	assert(!pin_heating());
	boot(); energize(); advance_ms(1000);
	assert(heater_hw_write(1, 2, -1, 25.0f, 3000) == -ESTALE);
	assert(!pin_heating());
	boot(); energize();
	assert(heater_hw_write(1, 2, k_uptime_get(), 25.0f, 0) == 0);
	assert(!pin_heating() && state.armed);
	assert(heater_hw_write(1, 3, k_uptime_get(), 25.0f, 7000) == 0);
	assert(pin_heating());
	assert(heater_hw_force_off() == 0 && !pin_heating());
}

static void stall_past_expiry(void)
{
	advance_ms(2020);
	assert(!pin_heating());
}
static void observe_late_restart(void)
{
	/* STOP is not a latch: the paused CPU did restart. Repeated STOP bounds it. */
	assert(pin_heating());
	advance_ms(2020);
	assert(!pin_heating());
}
static void expire_during_clear(void)
{
	advance_ms(2020);
	assert(!pin_heating());
}
static void delayed_cc_write(void)
{
	advance_ms(1900); /* CC0=500 ms will be behind the 1900 ms counter. */
}
static void off_racing_writer(void)
{
	assert(heater_hw_force_off() == -EBUSY);
	assert(!pin_heating());
	assert(timer_registers.running);
}

static void expiry_races(void)
{
	boot(); energize();
	before_pwm_start = stall_past_expiry;
	after_pwm_start = observe_late_restart;
	assert(heater_hw_write(1, 2, k_uptime_get(), 25.0f, 5000) == -ETIMEDOUT);
	assert(!pin_heating() && heater_hw_expired());

	boot(); energize();
	after_timer_clear = expire_during_clear;
	assert(heater_hw_write(1, 2, k_uptime_get(), 25.0f, 4000) == -ETIMEDOUT);
	assert(!pin_heating() && heater_hw_expired());

	boot(); energize(); advance_ms(1500);
	before_cc_write = delayed_cc_write;
	assert(heater_hw_write(1, 2, 100, 25.0f, 4000) == -ETIMEDOUT);
	assert(!pin_heating());

	boot();
	after_timer_enable = stall_past_expiry;
	assert(heater_hw_arm(1) == -ETIMEDOUT);
	assert(!pin_heating() && heater_hw_expired());

	boot(); energize();
	before_pwm_start = off_racing_writer;
	after_pwm_start = observe_late_restart;
	assert(heater_hw_write(1, 2, k_uptime_get(), 25.0f, 4000) == -EBUSY);
	assert(!heater_hw_available() && !pin_heating());
	assert(timer_registers.running);
	assert(heater_hw_arm(2) == -EBUSY);
}

static void independent_cutoff(void)
{
	boot(); energize();
	irq_masked = true;
	advance_ms(2020);
	/* PWM IRQ, sensor, workqueue and all firmware are absent from advance_us.
	 * Physical output already stopped despite ENABLE/driver state remaining on. */
	assert(!pin_heating() && pwm_registers.enabled);
	assert(heater_hw_expired());
	int64_t before = clock_us;
	assert(heater_hw_force_off() == -ETIMEDOUT);
	assert(clock_us - before <= 25000);
	assert(!pin_heating() && !heater_hw_available());
	assert(timer_registers.running);

	/* Direct register scenario: a CC update behind the counter must not defeat
	 * the permanently configured backstop. This is the same HAL write used by
	 * production renewal, isolated from its immediate software fail-off. */
	boot(); energize();
	nrfx_timer_clear(&lease_timer);
	advance_ms(1500);
	nrf_timer_cc_set(NRF_TIMER4, NRF_TIMER_CC_CHANNEL0, 500000);
	advance_ms(520);
	assert(!pin_heating());
	assert(timer_registers.event[1]);
}

static void errors_latch_and_keep_guard(void)
{
	boot(); energize();
	set_error = -EIO;
	assert(heater_hw_force_off() == -EIO);
	assert(!pin_heating() && !heater_hw_available());
	assert(timer_registers.running && ppi[0].enabled && ppi[1].enabled);
	set_error = 0;
	assert(heater_hw_force_off() == -EIO);
	assert(heater_hw_arm(2) == -EIO);
	struct heater_hw_status status;
	heater_hw_get_status(&status);
	assert(status.faulted && status.last_error == -EIO && !status.available);

	boot(); energize(); clock_failed = true;
	int64_t before = clock_us;
	assert(heater_hw_force_off() == -ETIMEDOUT);
	assert(clock_us - before <= 25000 && !pin_heating());
	assert(!heater_hw_available() && timer_registers.running);

	boot(); energize(); set_error = -EIO;
	assert(heater_hw_write(1, 2, k_uptime_get(), 25.0f, 3000) == -EIO);
	assert(!pin_heating() && !heater_hw_available());
	boot(); energize(); power_state = PM_DEVICE_STATE_SUSPENDED;
	assert(heater_hw_write(1, 2, k_uptime_get(), 25.0f, 3000) == -EHOSTDOWN);
	assert(!pin_heating());
}

static void initialization_failures(void)
{
	for (unsigned failure = 0; failure < 6; ++failure) {
		reset_model();
		if (failure == 0) { ready = false; }
		if (failure == 1) { init_error = -EBUSY; }
		if (failure == 2) { alloc_fail_at = 1; }
		if (failure == 3) { alloc_fail_at = 2; }
		if (failure == 4) { extra_pin = 30; }
		if (failure == 5) { pm_error = -EIO; }
		assert(heater_init() == 0);
		assert(!heater_hw_available() && !pin_heating());
		assert(heater_hw_arm(1) < 0);
		assert(!ppi[0].allocated && !ppi[1].allocated);
	}
}

int main(void)
{
	lease_and_freshness();
	rejected_inputs();
	expiry_races();
	independent_cutoff();
	errors_latch_and_keep_guard();
	initialization_failures();
	puts("heater_hw: production wrapper/register-model scenarios passed");
	return 0;
}
