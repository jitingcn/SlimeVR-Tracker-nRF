/* Actual production bodies are extracted at build time by run.py. These leaves
 * model hardware/storage only; native Zephyr tests cover the real mailbox lock. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "system/power.h"
#include "system/power_request.h"
#include <zephyr/sys/atomic.h>
#include "connection/tracker_event_protocol.h"

#define LOG_INF(...) ((void)0)
#define LOG_DBG(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define LOG_ERR(...) ((void)0)
#define CONFIG_BUILD_OUTPUT_UF2 1
#define CONFIG_SENSOR_USE_TCAL 0
#define CONFIG_DELAY_SLEEP_ON_STATUS 1
#define ADAFRUIT_BOOTLOADER 0
#define CONFIG_DISABLE_SENSOR_GPIOS_ON_SHUTDOWN 0
#define OTA_FLASH_PAGE_SIZE 4096
#define OTA_SUPPORTED 1
#define __aligned(n) __attribute__((aligned(n)))
#define SYS_REGULATOR_LDO 0
#define SYS_REBOOT_COLD 0
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define CLAMP(x, low, high) ((x) < (low) ? (low) : ((x) > (high) ? (high) : (x)))
#define CONFIG_DYNAMIC_ACTIVE_TIMEOUT 0
#define CONFIG_SENSOR_LP_TIMEOUT 500
#define CONFIG_USE_IMU_TIMEOUT 1
#define CONFIG_USE_IMU_WAKE_UP 1
#define CONFIG_IMU_TIMEOUT_RAMP_MIN 5000
#define CONFIG_IMU_TIMEOUT_RAMP_MAX 15000
#define CONFIG_USE_ACTIVE_TIMEOUT 1
#define CONFIG_ACTIVE_TIMEOUT_THRESHOLD 15000
#define CONFIG_SLEEP_ON_ACTIVE_TIMEOUT 1
#define SYS_STATUS_CALIBRATION_RUNNING 1
static bool test_active, calibration_active, ota_suppressed;
static atomic_t main_suspended;
static int64_t last_data_time, last_suspend_attempt_time;
static bool test_mode_get(void) { return test_active; }
static bool get_status(int status) { (void)status; return calibration_active; }
#define ADAFRUIT_DFU_MAGIC_UF2_RESET 0x57
static struct { uint32_t GPREGRET; } power_registers;
#define NRF_POWER (&power_registers)

static int preparation_result, preparations, physical_offs, physical_reboots, copies;
static int status_sends;
static enum sys_power_request in_flight;
static uint32_t in_flight_generation;
static bool finish_during_preparation;
static bool observe_abort_clear;
static int abort_gap_observations;
static unsigned notices, shutdown_prepares;
static uint8_t notice_phase, notice_detail, wom_pin;
static bool link_ready = true;
static bool esb_ready(void) { return link_ready; }
static bool status_ready(void) { return link_ready; }
#if IMU_INT_EXISTS
static void sensor_calibration_online_mag_retained_save(void) {}
static void sensor_record_wom_sleep(void) {}
static uint8_t sensor_setup_WOM(void) { return wom_pin; }
#define NRF_DT_GPIOS_TO_PSEL(a,b) 0
#define NRF_GPIO_PIN_DIR_INPUT 0
#define NRF_GPIO_PIN_INPUT_DISCONNECT 0
#define NRF_GPIO_PIN_PULLDOWN 0
#define NRF_GPIO_PIN_S0S1 0
#define NRF_GPIO_PIN_NOSENSE 0
static void nrf_gpio_cfg_input(int pin, int config) { (void)pin; (void)config; }
static void nrf_gpio_cfg_sense_set(int pin, int config) { (void)pin; (void)config; }
static void nrf_gpio_cfg(int a,int b,int c,int d,int e,int f)
{ (void)a;(void)b;(void)c;(void)d;(void)e;(void)f; }
#endif
static struct { uint8_t phase, detail; int64_t time; unsigned prepares; } notice_log[32];
static void tracker_event_notice(uint8_t kind, uint8_t phase, uint8_t detail)
{
	assert(kind == TRACKER_EVENT_KIND_POWER);
	assert(notices < 32);
	notice_log[notices].phase = phase;
	notice_log[notices].detail = detail;
	notice_log[notices].time = now_ms;
	notice_log[notices].prepares = shutdown_prepares;
	notices++;
	notice_phase = phase;
	notice_detail = detail;
}
static void tracker_events_notify(void) {}
static void *observe_memset(void *destination, int value, size_t size);
static void power_iteration(void);
static bool sys_system_reboot(void);
bool esb_ota_is_active(void);
static int prepare_upgrade(void);

static bool connection_get_ota_suppressed(void) { return ota_suppressed; }
static void configure_system_off(void)
{
	assert(notices > 0);
	int64_t lead = now_ms - notice_log[notices - 1].time;
	assert(lead >= (notice_phase == POWER_WILL_WOM ?
		TRACKER_EVENT_WOM_ADVANCE_MS : TRACKER_EVENT_POWER_FLUSH_MS));
	shutdown_prepares++;
	/* Production teardown suspends the sensor, which attempts cancellation
	 * after the physical gate. It must not publish a false withdrawal. */
	unsigned before_cancel = notices;
	sys_cancel_WOM();
	assert(notices == before_cancel);
}
static void sys_flush_warm(void) {}
static void sensor_calibration_online_mag_cold_start(void) {}
static void sensor_retained_write(void) {}
static void set_regulator(int regulator) { (void)regulator; }
static void sys_disconnect_interface_pins(void) {}
static int power_battery_current_pptt(void) { return 5000; }
static bool power_battery_device_plugged(void) { return false; }
static void sys_update_battery_tracker(int pptt, bool plugged) { (void)pptt; (void)plugged; }
static void wait_for_logging(void) {}
static void sys_poweroff(void) { physical_offs++; }
static void sys_reboot(int mode) { (void)mode; physical_reboots++; }
#if OTA_USE_MCUBOOT
static int esb_ota_flash_request_mcuboot_upgrade(void) { return prepare_upgrade(); }
#else
static int esb_ota_flash_prepare_bootloader_settings(uint32_t base, uint32_t size, uint8_t *buffer)
{
	(void)base; (void)size; (void)buffer;
	return prepare_upgrade();
}
static void esb_ota_flash_copy_and_reset(uint32_t base, uint32_t target, uint32_t size)
{
	(void)base; (void)target; (void)size;
	copies++;
}
#endif
static int esb_ota_flash_compute_crc32(uint32_t base, uint32_t size, uint8_t *buffer,
				     uint32_t *result)
{
	(void)base; (void)size; (void)buffer;
	*result = 0x12345678;
	return 0;
}
static void ota_send_status(void) { status_sends++; }

#define memset observe_memset
#include "production.inc"
#undef memset

static void *observe_memset(void *destination, int value, size_t size)
{
	void *result = memset(destination, value, size);
	if (observe_abort_clear && destination == &ota) {
		/* Exact preemption point: session is now IDLE, and the next statement
		 * in the old implementation had not yet republished its cleared flag. */
		uint8_t begin[OTA_BEGIN_PACKET_SIZE] = {0};
		assert(esb_ota_is_active());
		assert(esb_ota_handle_begin(begin, sizeof(begin)) == -EALREADY);
		abort_gap_observations++;
	}
	return result;
}

static int prepare_upgrade(void)
{
	preparations++;
	/* Flash work must not hold the lock or permit shutdown, even if an owner
	 * already claimed a reboot or an OFF handler completes concurrently. */
	assert(!power_requests.lock.locked);
	assert(!sys_system_reboot());
	assert(!sys_system_off());
#if IMU_INT_EXISTS
	assert(sys_plan_WOM(false, now_ms + 5000) == -EBUSY);
#else
	assert(sys_plan_WOM(false, now_ms + 5000) == -ENOTSUP);
#endif
	if (finish_during_preparation) {
		assert(in_flight != SYS_POWER_REQ_NONE);
		power_request_finish(&power_requests, in_flight, in_flight_generation,
				     in_flight == SYS_POWER_REQ_WOM);
		in_flight = SYS_POWER_REQ_NONE;
	}
	power_iteration();
	assert(physical_offs == 0 && physical_reboots == 0);
	return preparation_result;
}

static void fixture(void)
{
	memset(&ota, 0, sizeof(ota));
	atomic_set(&ota_reboot_pending, 0);
	observe_abort_clear = false;
	abort_gap_observations = 0;
	memset(&power_requests, 0, sizeof(power_requests));
	power_wake_sem.count = 0;
	power_registers.GPREGRET = 0;
	preparations = physical_offs = physical_reboots = copies = status_sends = 0;
	preparation_result = 0;
	in_flight = SYS_POWER_REQ_NONE;
	finish_during_preparation = false;
	notices = shutdown_prepares = 0;
	notice_phase = notice_detail = wom_pin = 0;
	link_ready = true;
	wom_planned = wom_announced = wom_ready_timeout_initialized = false;
	wom_deadline = wom_commit_at = wom_ready_timeout = wom_last_eligible = 0;
	test_active = calibration_active = ota_suppressed = false;
	atomic_set(&main_suspended, 0);
	sensor_timeout = SENSOR_SENSOR_TIMEOUT_IMU;
	sensor_mode = SENSOR_SENSOR_MODE_LOW_NOISE;
	was_ota_suppressed = false;
	last_data_time = last_suspend_attempt_time = 0;
	before_mutex_lock = NULL;
	sleep_observer = NULL;
	now_ms = 1000;
	ota.state = OTA_STATE_RECEIVING;
	ota.image_size = ota.bytes_written = 4;
	ota.image_crc32 = 0x12345678;
	ota.last_data_time = now_ms;
	assert(esb_ota_handle_verify() == 0);
	assert(esb_ota_get_status() == OTA_STATUS_VERIFY_OK);
}

static void activation_with_competitor(enum sys_power_request request, int phase)
{
	fixture();
	if (request != SYS_POWER_REQ_NONE) {
		assert(sys_power_state_request(request) == 0);
		if (phase == 1) {
			power_iteration(); /* OFF enters RETRY. */
		} else if (phase >= 2) {
			in_flight = power_request_begin(&power_requests, &in_flight_generation);
			assert(in_flight == request);
			finish_during_preparation = phase == 2;
		}
	}
	assert(esb_ota_handle_activate() == 0);
	assert(preparations == 1);
	assert(esb_ota_get_status() == OTA_STATUS_COMPLETE);
	assert(esb_ota_is_active());
	if (in_flight != SYS_POWER_REQ_NONE) {
		/* The producer commits first; owner finish must not erase its reboot. */
		power_request_finish(&power_requests, in_flight, in_flight_generation,
				     in_flight == SYS_POWER_REQ_WOM);
	}
	power_iteration();
	assert(physical_reboots == 1 && physical_offs == 0);
	assert(copies == !OTA_USE_MCUBOOT);
	/* The old reproducer advanced time and dispatched 1000 times forever. */
	for (int i = 0; i < 1000; i++) {
		now_ms += OTA_TIMEOUT_MS + 1;
		esb_ota_check_timeout();
		power_iteration();
	}
	assert(physical_reboots == 1 && physical_offs == 0);
}

static void recovery_after_preparation_failure(void)
{
	fixture();
	assert(sys_request_system_off() == 0);
	power_iteration();
	preparation_result = -EIO;
	assert(esb_ota_handle_activate() == -EIO);
	assert(esb_ota_get_status() == OTA_STATUS_FLASH_ERROR);
	assert(physical_reboots == 0 && copies == 0);
	/* Failed preparation cancels its reservation, preserving ordinary work. */
	assert(sys_request_system_off() == 0);
	assert(sys_request_system_reboot() == -EBUSY);
	power_iteration();
	now_ms += OTA_TIMEOUT_MS + 1;
	esb_ota_check_timeout();
	assert(esb_ota_get_status() == OTA_STATUS_TIMEOUT);
	assert(esb_ota_is_active());
	power_iteration();
	assert(physical_reboots == 1 && physical_offs == 0);
}

static void abort_preserves_recovery_ownership(void)
{
	fixture();
	assert(sys_request_system_off() == 0);
	power_iteration();
	observe_abort_clear = true;
	esb_ota_handle_abort();
	observe_abort_clear = false;
	assert(abort_gap_observations == 1);
	assert(esb_ota_get_status() == OTA_STATUS_IDLE); /* Existing wire status. */
	assert(esb_ota_is_active()); /* But no premature physical OFF admission. */
	assert(!sys_system_off());
	power_iteration();
	assert(physical_reboots == 1 && physical_offs == 0);
}

static void physical_shutdown_wins(void)
{
	fixture();
	/* Model the opposite race order: shutdown committed before OTA admission. */
	assert(sys_request_system_off() == 0);
	assert(power_request_begin(&power_requests, &in_flight_generation) == SYS_POWER_REQ_SYSTEM_OFF);
	assert(power_request_start_physical(&power_requests, false));
	assert(esb_ota_handle_activate() == -EBUSY);
	assert(preparations == 0);
	assert(esb_ota_get_status() == OTA_STATUS_VERIFY_OK);
	esb_ota_handle_abort();
	assert(esb_ota_get_status() == OTA_STATUS_VERIFY_OK);
	now_ms += OTA_TIMEOUT_MS + 1;
	esb_ota_check_timeout();
	assert(esb_ota_get_status() == OTA_STATUS_VERIFY_OK);
	assert(physical_reboots == 0);
}

static void observe_airtime(int milliseconds)
{
	assert(milliseconds == TRACKER_EVENT_POWER_FLUSH_MS);
	assert(!power_requests.lock.locked && !power_plan_lock.locked);
	assert(shutdown_prepares == 0);
	assert(sys_ota_reboot_reserve() == -EBUSY);
#if IMU_INT_EXISTS
	assert(sys_plan_WOM(true, now_ms) == -EBUSY);
#endif
}

static void power_notices(void)
{
	fixture();
	assert(!sys_system_off() && notices == 0); /* OTA rejection */
	fixture(); memset(&ota, 0, sizeof(ota));
	link_ready = false; /* No usable radio must not make shutdown unbounded. */
	sleep_observer = observe_airtime;
	int64_t before = now_ms;
	assert(sys_request_system_off() == 0);
	power_iteration();
	assert(notices == 1 && notice_phase == POWER_WILL_SHUTDOWN);
	assert(notice_detail == POWER_REASON_UNKNOWN && physical_offs == 1);
	assert(shutdown_prepares == 1 && now_ms - before == TRACKER_EVENT_POWER_FLUSH_MS);
	fixture(); memset(&ota, 0, sizeof(ota));
	assert(power_request_ota_reserve(&power_requests) == 0);
	assert(!sys_system_off() && notices == 0);
	fixture(); memset(&ota, 0, sizeof(ota));
	sleep_observer = observe_airtime;
	before = now_ms;
	assert(sys_request_system_reboot() == 0);
	power_iteration();
	assert(notices == 1 && notice_phase == POWER_WILL_REBOOT);
	assert(physical_reboots == 1 && now_ms - before == TRACKER_EVENT_POWER_FLUSH_MS);
	/* Owner-private battery/dock entry must use the same pre-teardown window. */
	fixture(); memset(&ota, 0, sizeof(ota));
	sleep_observer = observe_airtime;
	assert(sys_system_off());
	assert(physical_offs == 1 && notice_phase == POWER_WILL_SHUTDOWN);
#if !IMU_INT_EXISTS
	fixture(); memset(&ota, 0, sizeof(ota));
	assert(sys_plan_WOM(false, now_ms) == -ENOTSUP);
	sensor_update_sensor_state(true, 0, 0);
	assert(notices == 0 && physical_offs == 0);
#endif
}

#if IMU_INT_EXISTS
static void idle_until(int64_t deadline)
{
	while (now_ms < deadline) {
		now_ms += MIN(100, deadline - now_ms);
		sensor_update_sensor_state(true, 0, 0);
		power_iteration();
	}
}

static void sensor_deadlines_and_cancellation(void)
{
	/* Long ramp preserves original deadline; short ramp/debounce extends only
	 * enough to give a full five-second announced lead. */
	fixture(); memset(&ota, 0, sizeof(ota));
	last_data_time = 10000; last_suspend_attempt_time = 0; now_ms = 14999;
	sensor_update_sensor_state(true, 0, 0);
	assert(notices == 0);
	idle_until(15000);
	assert(notices == 1 && notice_phase == POWER_WILL_WOM);
	idle_until(19999);
	assert(physical_offs == 0);
	idle_until(20000);
	assert(physical_offs == 1 && notice_log[0].time == 15000);
	fixture(); memset(&ota, 0, sizeof(ota)); now_ms = 1500;
	sensor_update_sensor_state(true, 0, 0);
	idle_until(6499);
	assert(physical_offs == 0);
	idle_until(6500);
	assert(physical_offs == 1 && notice_log[0].time == 1500);

	/* Each interruption withdraws an already advertised plan and prevents the
	 * old mailbox from becoming physical after the original deadline. */
	for (int interruption = 0; interruption < 5; interruption++) {
		fixture(); memset(&ota, 0, sizeof(ota));
		sensor_update_sensor_state(true, 0, 0);
		assert(notices == 1);
		now_ms += 100;
		test_active = interruption == 1;
		calibration_active = interruption == 2;
		ota_suppressed = interruption == 3;
		atomic_set(&main_suspended, interruption == 4);
		sensor_update_sensor_state(interruption != 0, 0, 0);
		assert(notices == 2 && notice_phase == POWER_WOM_CANCELLED);
		assert(notice_detail == POWER_WOM_NORMAL);
		now_ms += 10000;
		power_iteration();
		assert(physical_offs == 0);
	}

	fixture(); memset(&ota, 0, sizeof(ota));
	sensor_timeout = SENSOR_SENSOR_TIMEOUT_ACTIVITY;
	now_ms = CONFIG_ACTIVE_TIMEOUT_DELAY - TRACKER_EVENT_WOM_ADVANCE_MS;
	sensor_update_sensor_state(true, 0, 0);
	assert(notice_detail == POWER_WOM_FORCED);
	idle_until(CONFIG_ACTIVE_TIMEOUT_DELAY - 1);
	assert(physical_offs == 0);
	idle_until(CONFIG_ACTIVE_TIMEOUT_DELAY);
	assert(physical_offs == 1);
}

static void readiness_cancel_and_rearm(void)
{
	fixture(); memset(&ota, 0, sizeof(ota)); link_ready = false;
	sensor_update_sensor_state(true, 0, 0);
	power_iteration();
	assert(notices == 0 && physical_offs == 0);
	now_ms += 100;
	sensor_update_sensor_state(false, 0, 0); /* historical stale retry defect */
	link_ready = true; now_ms += 10000;
	power_iteration();
	assert(notices == 0 && physical_offs == 0);
	last_data_time = now_ms;
	last_suspend_attempt_time = now_ms;
	sensor_update_sensor_state(true, 0, 0);
	assert(notices == 1);
	int64_t fresh_notice = now_ms;
	idle_until(fresh_notice + 4999);
	assert(physical_offs == 0);
	idle_until(fresh_notice + 5000);
	assert(physical_offs == 1);

	/* Losing readiness after announcement cancels; renewed readiness earns a
	 * new lead rather than resurrecting the old nearly-expired countdown. */
	fixture(); memset(&ota, 0, sizeof(ota));
	sensor_update_sensor_state(true, 0, 0);
	idle_until(now_ms + 4000);
	link_ready = false;
	sensor_update_sensor_state(true, 0, 0);
	assert(notice_phase == POWER_WOM_CANCELLED);
	link_ready = true; now_ms += 100;
	sensor_update_sensor_state(true, 0, 0);
	fresh_notice = now_ms;
	assert(notice_phase == POWER_WILL_WOM && notices == 3);
	idle_until(fresh_notice + 4999);
	assert(physical_offs == 0);
	idle_until(fresh_notice + 5000);
	assert(physical_offs == 1);

	/* Readiness timeout itself is never advertised: first notice appears when
	 * the timeout permits sleep, followed by a new full lead window. */
	fixture(); memset(&ota, 0, sizeof(ota)); link_ready = false;
	sensor_timeout = SENSOR_SENSOR_TIMEOUT_IMU;
	sensor_update_sensor_state(true, 0, 0);
	/* Exercise the plan directly to keep the normal policy (activity normally
	 * supersedes it at15s) and refresh its continuous-eligibility lease. */
	int64_t original_deadline = wom_deadline;
	int64_t ready_at = original_deadline + 30000;
	while (now_ms < ready_at) {
		now_ms += MIN(100, ready_at - now_ms);
		assert(sys_plan_WOM(false, original_deadline) == 0);
		power_iteration();
		if (now_ms < ready_at) { assert(notices == 0); }
	}
	assert(notices == 1 && physical_offs == 0);
	fresh_notice = now_ms;
	for (int i = 0; i < 50; i++) {
		now_ms += 100;
		assert(sys_plan_WOM(false, original_deadline) == 0);
		power_iteration();
	}
	assert(physical_offs == 1 && now_ms - fresh_notice == 5000);
}

static void replace_before_commit(void)
{
	sys_cancel_WOM();
	assert(sys_plan_WOM(false, now_ms + 5000) == 0);
}

static void stale_generation_and_veto(void)
{
	fixture(); memset(&ota, 0, sizeof(ota));
	assert(sys_plan_WOM(false, now_ms + 5000) == 0);
	uint32_t generation;
	enum sys_power_request claimed = power_request_begin(&power_requests, &generation);
	now_ms += 5000;
	/* Cancellation/rearm runs at the final owner-mutex acquisition boundary. */
	before_mutex_lock = replace_before_commit;
	assert(sys_WOM(false, generation));
	power_request_finish(&power_requests, claimed, generation, true);
	assert(physical_offs == 0 && notices == 3);
	int64_t fresh = now_ms;
	for (int i = 0; i < 50; i++) {
		now_ms += 100;
		assert(sys_plan_WOM(false, fresh + 5000) == 0);
		power_iteration();
	}
	assert(physical_offs == 1);

	/* A resumed sensor cannot refresh away a missed eligibility interval even
	 * if the power owner was also delayed and never observed the expiry. */
	fixture(); memset(&ota, 0, sizeof(ota));
	sensor_update_sensor_state(true, 0, 0);
	now_ms += WOM_ELIGIBILITY_LEASE_MS;
	sensor_update_sensor_state(true, 0, 0);
	assert(notices == 3 && notice_log[1].phase == POWER_WOM_CANCELLED);
	int64_t resumed = now_ms;
	idle_until(resumed + 4999);
	assert(physical_offs == 0);
	idle_until(resumed + 5000);
	assert(physical_offs == 1);

	/* Test/calibration/OTA can begin after the sensor's last publication. */
	for (int veto = 0; veto < 5; veto++) {
		fixture(); memset(&ota, 0, sizeof(ota));
		assert(sys_plan_WOM(false, now_ms + 5000) == 0);
		for (int i = 0; i < 50; i++) {
			now_ms += 100;
			assert(sys_plan_WOM(false, wom_deadline) == 0);
		}
		test_active = veto == 0;
		calibration_active = veto == 1;
		ota_suppressed = veto == 2;
		if (veto == 3) { now_ms += WOM_ELIGIBILITY_LEASE_MS; }
		atomic_set(&main_suspended, veto == 4);
		power_iteration();
		assert(physical_offs == 0 && notice_phase == POWER_WOM_CANCELLED);
	}
}

static void wom_supersession_and_failure(void)
{
	for (int replacement = 0; replacement < 3; replacement++) {
		fixture(); memset(&ota, 0, sizeof(ota));
		assert(sys_plan_WOM(true, now_ms + 5000) == 0);
		if (replacement == 0) {
			assert(sys_request_system_off() == 0);
		} else if (replacement == 1) {
			assert(sys_request_system_reboot() == 0);
		} else {
			assert(sys_ota_reboot_reserve() == 0);
			sys_ota_reboot_resolve(true);
		}
		assert(notices == 2 && notice_phase == POWER_WOM_CANCELLED);
		assert(notice_detail == POWER_WOM_FORCED);
		power_iteration();
		assert(physical_offs == (replacement == 0));
		assert(physical_reboots == (replacement != 0));
	}
	fixture(); memset(&ota, 0, sizeof(ota)); wom_pin = 255;
	sensor_update_sensor_state(true, 0, 0);
	idle_until(now_ms + 5000);
	assert(physical_offs == 0 && physical_reboots == 1);
	assert(notices == 3 && notice_log[1].phase == POWER_WOM_CANCELLED);
	assert(notice_log[2].phase == POWER_WILL_REBOOT);
	assert(notice_log[2].prepares == 1); /* best effort after WOM setup failure */
}

static void consumed_intent_cleanup(void)
{
	for (int replacement = 0; replacement < 3; replacement++) {
		fixture(); memset(&ota, 0, sizeof(ota));
		assert(sys_plan_WOM(true, now_ms + 5000) == 0);
		uint32_t generation;
		enum sys_power_request claimed = power_request_begin(&power_requests, &generation);
		power_request_finish(&power_requests, claimed, generation, true);
		/* A private owner may have consumed WOM without the policy cleanup. */
		if (replacement == 1) {
			assert(power_request_submit(&power_requests, SYS_POWER_REQ_SYSTEM_OFF,
						    &power_wake_sem) == 0);
		} else if (replacement == 2) {
			assert(power_request_ota_reserve(&power_requests) == 0);
			power_request_ota_resolve(&power_requests, true, &power_wake_sem);
			assert(power_request_begin(&power_requests, &generation) == SYS_POWER_REQ_REBOOT);
		}
		sys_cancel_WOM();
		sys_cancel_WOM();
		assert(notices == 2 && notice_phase == POWER_WOM_CANCELLED);
		assert(notice_detail == POWER_WOM_FORCED);
		if (replacement == 0) {
			/* Cleared intent cannot be resurrected with an expired lead. */
			now_ms += 10000;
			assert(sys_plan_WOM(true, now_ms) == 0);
			power_iteration();
			assert(notices == 3 && physical_offs == 0);
		} else if (replacement == 1) {
			power_iteration();
			assert(physical_offs == 1);
		} else {
			assert(sys_system_reboot());
			assert(physical_reboots == 1);
		}
	}
}

static void boot_readiness_budget(void)
{
	/* An early plan (including one cancelled before due) must not consume the
	 * boot's readiness budget. Ready and forced plans must not start it either. */
	fixture(); memset(&ota, 0, sizeof(ota));
	assert(sys_plan_WOM(false, 10000) == 0);
	sys_cancel_WOM();
	link_ready = false;
	assert(sys_plan_WOM(true, 10000) == 0);
	sys_cancel_WOM();
	now_ms = 5000;
	assert(sys_plan_WOM(false, 10000) == 0);
	now_ms = 9000;
	sys_cancel_WOM();
	assert(sys_plan_WOM(false, 10000) == 0);
	assert(notice_phase == POWER_WOM_CANCELLED);
	unsigned previous_notices = notices;
	for (; now_ms < 20000; now_ms += 100) {
		assert(sys_plan_WOM(false, 10000) == 0);
		power_iteration();
	}
	sys_cancel_WOM(); /* budget began at10000; interruption cannot renew it */
	now_ms = 30000;
	for (; now_ms <= 40000; now_ms += 100) {
		assert(sys_plan_WOM(false, 35000) == 0);
		power_iteration();
		if (now_ms < 40000) { assert(notices == previous_notices); }
	}
	assert(notices == previous_notices + 1 && notice_phase == POWER_WILL_WOM);
	assert(notice_log[notices - 1].time == 40000 && physical_offs == 0);
	for (; now_ms <= 45000; now_ms += 100) {
		assert(sys_plan_WOM(false, 35000) == 0);
		power_iteration();
	}
	assert(physical_offs == 1);

	/* First blocked attempt at uptime zero still gets exactly one budget. */
	fixture(); memset(&ota, 0, sizeof(ota)); link_ready = false; now_ms = 0;
	assert(sys_plan_WOM(false, 0) == 0);
	sys_cancel_WOM();
	now_ms = 30000;
	assert(sys_plan_WOM(false, now_ms) == 0);
	assert(notices == 1 && notice_log[0].time == 30000);
	power_iteration();
	assert(physical_offs == 0);
}

static void ramp_anchor_tracks_due_attempts(void)
{
	fixture(); memset(&ota, 0, sizeof(ota));
	last_data_time = 10000; now_ms = 15000;
	sensor_update_sensor_state(true, 0, 0); /* early10s-ramp notice */
	assert(notices == 1);
	now_ms = 16000;
	sensor_update_sensor_state(false, 0, 0);
	last_data_time = now_ms; /* real publication follows the policy pass */
	sensor_update_sensor_state(true, 0, 0);
	/* The old anchor still yields a15s ramp, not5s from the early notice. */
	idle_until(25999);
	assert(notices == 2);
	idle_until(26000);
	assert(notices == 3 && notice_phase == POWER_WILL_WOM);
	idle_until(30999);
	assert(physical_offs == 0);
	idle_until(31000);
	assert(physical_offs == 1);

	/* Due accepted attempt preserves the old ramp reset even without LP2. */
	fixture(); memset(&ota, 0, sizeof(ota)); link_ready = false;
	last_data_time = 10000; now_ms = 20001;
	sensor_update_sensor_state(true, 0, 0);
	now_ms = 20100;
	sensor_update_sensor_state(false, 0, 0);
	last_data_time = now_ms;
	link_ready = true;
	sensor_update_sensor_state(true, 0, 0);
	assert(notices == 1 && notice_log[0].time == 20100);
	idle_until(25099);
	assert(physical_offs == 0);
	idle_until(25100);
	assert(physical_offs == 1);
}
#endif

int main(void)
{
	activation_with_competitor(SYS_POWER_REQ_NONE, 0);
	for (int phase = 0; phase < 4; phase++) {
		activation_with_competitor(SYS_POWER_REQ_SYSTEM_OFF, phase);
	}
	activation_with_competitor(SYS_POWER_REQ_WOM, 2);
	activation_with_competitor(SYS_POWER_REQ_WOM, 3);
	activation_with_competitor(SYS_POWER_REQ_REBOOT, 2);
	activation_with_competitor(SYS_POWER_REQ_REBOOT, 3);
	recovery_after_preparation_failure();
	abort_preserves_recovery_ownership();
	physical_shutdown_wins();
	power_notices();
#if IMU_INT_EXISTS
	sensor_deadlines_and_cancellation();
	readiness_cancel_and_rearm();
	stale_generation_and_veto();
	wom_supersession_and_failure();
	consumed_intent_cleanup();
	boot_readiness_budget();
	ramp_anchor_tracks_due_attempts();
#endif
	printf("PASS OTA/power activation, owner races, recovery and physical exclusion (MCUboot=%d)\n",
	       OTA_USE_MCUBOOT);
	return 0;
}
