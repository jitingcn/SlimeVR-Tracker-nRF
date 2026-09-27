#include <zephyr/devicetree.h>

#define HEATER_NODE DT_PATH(zephyr_user)

#ifdef CONFIG_SYSTEM_IMU_HEATER
#include "heater.h"
#include "util.h"

#include <errno.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/atomic.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_pwm.h>
#include <nrfx_timer.h>
#include <helpers/nrfx_gppi.h>

BUILD_ASSERT(DT_NODE_HAS_PROP(HEATER_NODE, heater_lease_timer),
	     "Heated T-Cal requires a board-defined heater lease timer");
BUILD_ASSERT(DT_SAME_NODE(DT_PHANDLE(HEATER_NODE, heater_lease_timer), DT_NODELABEL(timer4)));
BUILD_ASSERT(DT_NODE_HAS_STATUS(DT_NODELABEL(timer4), reserved));
BUILD_ASSERT(DT_SAME_NODE(DT_PWMS_CTLR(HEATER_NODE), DT_NODELABEL(pwm1)));
BUILD_ASSERT(DT_PWMS_CHANNEL(HEATER_NODE) == 0);
BUILD_ASSERT(DT_PWMS_PERIOD(HEATER_NODE) == HEATER_HW_PERIOD_NS);
BUILD_ASSERT(DT_PWMS_FLAGS(HEATER_NODE) == PWM_POLARITY_NORMAL);
BUILD_ASSERT(DT_GPIO_PIN(HEATER_NODE, heat_en_gpios) == 31);
BUILD_ASSERT(DT_SAME_NODE(DT_GPIO_CTLR(HEATER_NODE, heat_en_gpios), DT_NODELABEL(gpio0)));
BUILD_ASSERT(DT_GPIO_FLAGS(HEATER_NODE, heat_en_gpios) == 0);
BUILD_ASSERT(CONFIG_ESB_SYS_TIMER_INSTANCE != 4, "TIMER4 belongs exclusively to the heater");
BUILD_ASSERT(CONFIG_SYSTEM_IMU_HEATER_MAX_DUTY_PPTT < 10000);

/* This target has no system PM: stopping the HF timer clock while armed is unsafe.
 * Device PM is allowed, but PWM1 is marked busy for the entire armed session. */
BUILD_ASSERT(!IS_ENABLED(CONFIG_PM));
BUILD_ASSERT(!IS_ENABLED(CONFIG_PM_DEVICE_RUNTIME));

static const struct pwm_dt_spec heater_pwm = PWM_DT_SPEC_GET(HEATER_NODE);
static nrfx_timer_t lease_timer = NRFX_TIMER_INSTANCE(NRF_TIMER4);
static nrfx_gppi_handle_t lease_connection;
static nrfx_gppi_handle_t backstop_connection;
static K_MUTEX_DEFINE(heater_lock);
static atomic_t fatal_error;
static struct heater_hw_status state;
static bool timer_ready;
static bool connection_ready;
static bool have_sample;
static uint32_t last_sequence;
static int64_t last_sample_ms;

#define HEATER_PIN 31U
#define OFF_WAIT_US 25000U
#define LOCK_WAIT_MS 30

/* Never clear the GPIO latch to HIGH, even transiently: STOP falls back to it.
 * Direct disable is an emergency action, not normal driver state management. */
static void emergency_off(int error)
{
	atomic_cas(&fatal_error, 0, error);
	nrf_gpio_pin_clear(HEATER_PIN);
	nrf_pwm_task_trigger(NRF_PWM1, NRF_PWM_TASK_STOP);
	nrf_pwm_disable(NRF_PWM1);
	nrf_gpio_cfg_output(HEATER_PIN);
}

static bool pwm_inactive(void)
{
	return !nrf_pwm_enable_check(NRF_PWM1);
}

static int wait_inactive(void)
{
	/* Bounded even if peripheral clocks fail. IRQs stay enabled; the PWM driver
	 * may consume STOPPED and disable the peripheral while this loop waits. */
	for (unsigned elapsed = 0; elapsed < OFF_WAIT_US; elapsed += 50) {
		if (pwm_inactive()) {
			return 0;
		}
		k_busy_wait(50);
	}
	return pwm_inactive() ? 0 : -ETIMEDOUT;
}

static bool check_expired(void)
{
	if (state.armed && (nrf_timer_event_check(NRF_TIMER4, NRF_TIMER_EVENT_COMPARE0) ||
			   nrf_timer_event_check(NRF_TIMER4, NRF_TIMER_EVENT_COMPARE1) ||
			   k_uptime_get() >= state.deadline_ms)) {
		state.expired = true;
	}
	return state.expired;
}

static int off_locked(void)
{
	nrf_gpio_pin_clear(HEATER_PIN);
	int error = pwm_is_ready_dt(&heater_pwm) ?
		pwm_set_dt(&heater_pwm, HEATER_HW_PERIOD_NS, 0) : -ENODEV;
	/* Do not trust successful zero-duty submission as a completion signal. */
	nrf_pwm_task_trigger(NRF_PWM1, NRF_PWM_TASK_STOP);
	int stopped = wait_inactive();
	if (error || stopped) {
		error = error ? error : stopped;
		emergency_off(error);
	} else if (!atomic_get(&fatal_error)) {
		/* Only retire the independent guard after confirmed peripheral idle.
		 * An emergency leaves it running to catch any late in-flight restart. */
		if (timer_ready) {
			nrfx_timer_disable(&lease_timer);
		}
		if (connection_ready) {
			nrfx_gppi_conn_disable(lease_connection);
			nrfx_gppi_conn_disable(backstop_connection);
		}
		pm_device_busy_clear(heater_pwm.dev);
	}
	state.armed = false;
	state.duty_pptt = 0;
	if (atomic_get(&fatal_error)) {
		error = (int)atomic_get(&fatal_error);
	}
	if (error) {
		state.last_error = error;
	}
	return error;
}

static int fail_locked(int error)
{
	state.last_error = error;
	int off_error = off_locked();
	return off_error ? off_error : error;
}

static int take_lock(void)
{
	int error = k_mutex_lock(&heater_lock, K_MSEC(LOCK_WAIT_MS));
	if (error) {
		/* The owner may be paused inside a peripheral write. Retain TIMER/GPPI
		 * and latch failure; its post-write check must also force off. */
		emergency_off(-EBUSY);
		return -EBUSY;
	}
	return 0;
}

static int device_active(void)
{
	enum pm_device_state power_state;
	int error = pm_device_state_get(heater_pwm.dev, &power_state);
	if (error && error != -ENOSYS) {
		return error;
	}
	return power_state == PM_DEVICE_STATE_ACTIVE ? 0 : -EHOSTDOWN;
}

static int heater_init(void)
{
	nrf_gpio_pin_clear(HEATER_PIN);
	if (!pwm_is_ready_dt(&heater_pwm)) {
		emergency_off(-ENODEV);
		return 0;
	}
	/* The entire PWM instance is exclusive, not just channel zero. */
	for (unsigned channel = 0; channel < NRF_PWM_CHANNEL_COUNT; ++channel) {
		uint32_t expected = channel == 0 ? HEATER_PIN : NRF_PWM_PIN_NOT_CONNECTED;
		if (nrf_pwm_pin_get(NRF_PWM1, channel) != expected) {
			emergency_off(-EINVAL);
			return 0;
		}
	}
	int error = device_active();
	if (!error) {
		error = off_locked();
	}
	if (error) {
		emergency_off(error);
		return 0;
	}
	nrfx_timer_config_t config = NRFX_TIMER_DEFAULT_CONFIG(1000000);
	config.bit_width = NRF_TIMER_BIT_WIDTH_32;
	error = nrfx_timer_init(&lease_timer, &config, NULL);
	if (error) {
		emergency_off(error);
		return 0;
	}
	timer_ready = true;
	error = nrfx_gppi_conn_alloc(
		nrf_timer_event_address_get(NRF_TIMER4, NRF_TIMER_EVENT_COMPARE0),
		nrf_pwm_task_address_get(NRF_PWM1, NRF_PWM_TASK_STOP), &lease_connection);
	if (error) {
		nrfx_timer_uninit(&lease_timer);
		timer_ready = false;
		emergency_off(error);
		return 0;
	}
	error = nrfx_gppi_conn_alloc(
		nrf_timer_event_address_get(NRF_TIMER4, NRF_TIMER_EVENT_COMPARE1),
		nrf_pwm_task_address_get(NRF_PWM1, NRF_PWM_TASK_STOP), &backstop_connection);
	if (error) {
		nrfx_gppi_conn_free(
			nrf_timer_event_address_get(NRF_TIMER4, NRF_TIMER_EVENT_COMPARE0),
			nrf_pwm_task_address_get(NRF_PWM1, NRF_PWM_TASK_STOP), lease_connection);
		nrfx_timer_uninit(&lease_timer);
		timer_ready = false;
		emergency_off(error);
		return 0;
	}
	connection_ready = true;
	state.available = true;
	return 0;
}
SYS_INIT(heater_init, APPLICATION, 90);

int heater_hw_arm(uint32_t generation)
{
	int error = take_lock();
	if (error) {
		return error;
	}
	if (atomic_get(&fatal_error) || !state.available) {
		error = atomic_get(&fatal_error) ? (int)atomic_get(&fatal_error) : -ENODEV;
	} else if (state.armed) {
		error = fail_locked(-EBUSY);
	} else if (!generation || (state.generation &&
		   (uint32_t)(generation - state.generation) >= 0x80000000U) ||
		   generation == state.generation) {
		error = -ESTALE;
	} else {
		error = device_active();
		if (!error) {
			error = off_locked();
		}
		if (!error) {
			pm_device_busy_set(heater_pwm.dev);
			state.generation = generation;
			state.deadline_ms = k_uptime_get() + HEATER_HW_LEASE_MS;
			state.expired = false;
			state.last_error = 0;
			have_sample = false;
			nrfx_timer_clear(&lease_timer);
			nrf_timer_event_clear(NRF_TIMER4, NRF_TIMER_EVENT_COMPARE0);
			nrf_timer_event_clear(NRF_TIMER4, NRF_TIMER_EVENT_COMPARE1);
			nrfx_timer_extended_compare(&lease_timer, NRF_TIMER_CC_CHANNEL0,
				HEATER_HW_LEASE_MS * 1000U, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
			/* Fixed backstop cannot move behind a live counter during renewal. */
			nrfx_timer_extended_compare(&lease_timer, NRF_TIMER_CC_CHANNEL1,
				HEATER_HW_LEASE_MS * 1000U, NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK, false);
			nrfx_gppi_conn_enable(lease_connection);
			nrfx_gppi_conn_enable(backstop_connection);
			nrfx_timer_enable(&lease_timer);
			state.armed = true;
			if (atomic_get(&fatal_error) || check_expired()) {
				error = fail_locked(-ETIMEDOUT);
			}
		} else {
			error = fail_locked(error);
		}
	}
	if (error) {
		state.last_error = error;
	}
	k_mutex_unlock(&heater_lock);
	return error;
}

int heater_hw_write(uint32_t generation, uint32_t sequence, int64_t sampled_at_ms,
		    float raw_temperature_c, uint16_t duty_pptt)
{
	int error = take_lock();
	if (error) {
		return error;
	}
	int64_t now = k_uptime_get();
	uint32_t advance = sequence - last_sequence;
	if (atomic_get(&fatal_error)) {
		error = (int)atomic_get(&fatal_error);
	} else if (!state.armed || generation != state.generation) {
		error = -ESTALE;
	} else if (check_expired()) {
		error = -ETIMEDOUT;
	} else if (!v_finite(&raw_temperature_c, 1) || raw_temperature_c < -40.0f ||
		   raw_temperature_c >= CONFIG_SENSOR_TCAL_HEATED_MAX_TEMP_C ||
		   duty_pptt > CONFIG_SYSTEM_IMU_HEATER_MAX_DUTY_PPTT) {
		error = -ERANGE;
	} else if (sampled_at_ms < 0 || sampled_at_ms > now ||
		   now - sampled_at_ms >= HEATER_HW_LEASE_MS ||
		   (have_sample && (!advance || advance >= 0x80000000U ||
				   sampled_at_ms < last_sample_ms))) {
		error = -ESTALE;
	} else {
		error = device_active();
	}
	if (error) {
		error = fail_locked(error);
		goto done;
	}
	/* The event stays sticky across CLEAR and renewal. Never stop TIMER or
	 * disable PPI here: a stalled owner must still lose heat independently.
	 * CC0 shortens the interval to the sample's absolute deadline. CC1's fixed
	 * 2s backstop still fires if a paused CC0 update lands behind the counter.
	 * A late CPU restart remains bounded, but this is not a latched interlock. */
	state.deadline_ms = sampled_at_ms + HEATER_HW_LEASE_MS;
	nrfx_timer_clear(&lease_timer);
	now = k_uptime_get();
	if (check_expired() || now >= state.deadline_ms) {
		error = fail_locked(-ETIMEDOUT);
		goto done;
	}
	nrf_timer_cc_set(NRF_TIMER4, NRF_TIMER_CC_CHANNEL0,
			(uint32_t)(state.deadline_ms - now) * 1000U);
	last_sequence = sequence;
	last_sample_ms = sampled_at_ms;
	have_sample = true;
	/* Off's bounded wait is essential: pwm_nrfx's next nonzero write has an
	 * unbounded stopped_check loop after zero duty. Enter it only after stop
	 * completion, without masking interrupts. No other PWM1 users are allowed. */
	if (check_expired() || atomic_get(&fatal_error)) {
		error = fail_locked(-ETIMEDOUT);
		goto done;
	}
	error = pwm_set_dt(&heater_pwm, HEATER_HW_PERIOD_NS,
			   (HEATER_HW_PERIOD_NS / 10000U) * duty_pptt);
	if (error) {
		emergency_off(error);
		error = fail_locked(error);
	} else if (check_expired() || atomic_get(&fatal_error)) {
		error = fail_locked(-ETIMEDOUT);
	} else if (!duty_pptt) {
		/* Preserve the armed lease; zero duty is not a session release. */
		error = wait_inactive();
		if (error) {
			emergency_off(error);
			error = fail_locked(error);
		}
		if (!error && (check_expired() || atomic_get(&fatal_error))) {
			error = fail_locked(-ETIMEDOUT);
		}
	}
	if (!error) {
		state.duty_pptt = duty_pptt;
	}
done:
	k_mutex_unlock(&heater_lock);
	return error;
}

int heater_hw_force_off(void)
{
	int error = take_lock();
	if (error) {
		return error;
	}
	check_expired();
	error = off_locked();
	k_mutex_unlock(&heater_lock);
	return error;
}

void heater_hw_get_status(struct heater_hw_status *out)
{
	if (!out) {
		return;
	}
	int error = take_lock();
	if (error) {
		*out = (struct heater_hw_status){.faulted = true, .last_error = error};
		return;
	}
	check_expired();
	*out = state;
	out->faulted = atomic_get(&fatal_error) != 0;
	out->available = state.available && !out->faulted;
	if (out->faulted) {
		out->last_error = (int)atomic_get(&fatal_error);
	}
	k_mutex_unlock(&heater_lock);
}

bool heater_hw_expired(void)
{
	struct heater_hw_status status;
	heater_hw_get_status(&status);
	return status.expired || status.faulted;
}

bool heater_hw_available(void)
{
	struct heater_hw_status status;
	heater_hw_get_status(&status);
	return status.available;
}
#endif
