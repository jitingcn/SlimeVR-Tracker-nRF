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
#define ADAFRUIT_DFU_MAGIC_UF2_RESET 0x57
static struct { uint32_t GPREGRET; } power_registers;
#define NRF_POWER (&power_registers)

static int preparation_result, preparations, physical_offs, physical_reboots, copies;
static int status_sends;
static enum sys_power_request in_flight;
static bool finish_during_preparation;
static bool observe_abort_clear;
static int abort_gap_observations;
static unsigned notices, shutdown_prepares;
static uint8_t notice_phase, notice_detail, wom_pin;
static bool link_ready = true;
#if IMU_INT_EXISTS
static int64_t system_off_timeout;
static bool esb_ready(void) { return link_ready; }
static bool status_ready(void) { return link_ready; }
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
static void tracker_event_notice(uint8_t kind, uint8_t phase, uint8_t detail)
{
	assert(kind == TRACKER_EVENT_KIND_POWER);
	assert(shutdown_prepares == 0);
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

static bool connection_get_ota_suppressed(void) { return false; }
static void configure_system_off(void) { shutdown_prepares++; }
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
	assert(sys_request_WOM(false) == -EBUSY);
	if (finish_during_preparation) {
		assert(in_flight != SYS_POWER_REQ_NONE);
		power_request_finish(&power_requests, in_flight,
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
#if IMU_INT_EXISTS
	system_off_timeout = 0;
#endif
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
			in_flight = power_request_begin(&power_requests);
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
		power_request_finish(&power_requests, in_flight, in_flight == SYS_POWER_REQ_WOM);
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
	assert(power_request_begin(&power_requests) == SYS_POWER_REQ_SYSTEM_OFF);
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

static void power_notices(void)
{
	fixture();
	assert(sys_WOM(false) && notices == 0); /* OTA rejection */
	assert(!sys_system_off() && notices == 0);
	fixture(); memset(&ota, 0, sizeof(ota));
	assert(sys_request_system_off() == 0);
	power_iteration();
	assert(notices == 1 && notice_phase == POWER_WILL_SHUTDOWN);
	assert(notice_detail == POWER_REASON_UNKNOWN && physical_offs == 1);
	assert(shutdown_prepares == 1);
	fixture(); memset(&ota, 0, sizeof(ota));
	assert(power_request_ota_reserve(&power_requests) == 0);
	assert(!sys_system_off() && notices == 0); /* physical gate, no active OTA session */
#if IMU_INT_EXISTS
	assert(!sys_WOM(false) && notices == 0);
#endif
	fixture(); memset(&ota, 0, sizeof(ota));
	assert(sys_request_system_reboot() == 0); power_iteration();
	assert(notices == 0 && physical_reboots == 1);
	for (int forced = 0; forced < 2; forced++) {
		fixture(); memset(&ota, 0, sizeof(ota));
		assert(sys_request_WOM(forced) == 0);
		int64_t before = now_ms;
		power_iteration();
#if IMU_INT_EXISTS
		assert(notices == 1 && notice_phase == POWER_WILL_WOM);
		assert(notice_detail == (forced ? POWER_WOM_FORCED : POWER_WOM_NORMAL));
		assert(physical_offs == 1 && shutdown_prepares == 1);
#else
		assert(notices == 0 && physical_offs == 0);
#endif
		assert(now_ms == before); /* no event-induced flush/wait */
	}
#if IMU_INT_EXISTS
	fixture(); memset(&ota, 0, sizeof(ota)); link_ready = false;
	assert(sys_request_WOM(false) == 0); power_iteration();
	assert(notices == 0 && shutdown_prepares == 0); /* readiness retry */
	link_ready = true; power_iteration();
	assert(notices == 1 && physical_offs == 1);
	fixture(); memset(&ota, 0, sizeof(ota)); wom_pin = 255;
	assert(sys_request_WOM(true) == 0); power_iteration();
	assert(notices == 1 && notice_phase == POWER_WILL_WOM);
	assert(physical_offs == 0 && physical_reboots == 1); /* intention, not outcome */
#endif
}

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
	printf("PASS OTA/power activation, owner races, recovery and physical exclusion (MCUboot=%d)\n",
	       OTA_USE_MCUBOOT);
	return 0;
}
