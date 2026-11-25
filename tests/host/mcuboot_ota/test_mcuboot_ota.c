#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define IS_ENABLED(x) (x)
#define LOG_ERR(...) ((void)0)
#define LOG_ERR_RATELIMIT(...) ((void)0)
#define LOG_INF(...) ((void)0)

enum pm_device_action {
	PM_DEVICE_ACTION_SUSPEND = 1,
	PM_DEVICE_ACTION_RESUME = 2,
};

struct device {
	const char *name;
};

static const struct device bus_spi0 = {.name = "spi0"};
static const struct device bus_i2c1 = {.name = "i2c1"};

/* Synthetic DT nodes:
 *   1 = imu_spi (child of bus_spi0 = 10)
 *   2 = mag     (child of bus_i2c1 = 20)
 *   3 = pmic    (foreign okay child of bus_i2c1 when HOST_SHARED_MAG_BUS == 1)
 */
#define NODE_imu_spi 1
#define NODE_mag     2
#define NODE_pmic    3
#define NODE_imu     0
#define NODE_mag_spi 0

#define DT_NODELABEL(label) NODE_##label
#define DT_NODE_EXISTS(node) ((node) != 0)
#define DT_SAME_NODE(a, b) ((a) == (b))

#define PARENT_1 10
#define PARENT_2 20
#define PARENT_0 0
#define DT_PARENT_1(n) PARENT_##n
#define DT_PARENT(n) DT_PARENT_1(n)

#define DT_NODE_HAS_STATUS_OKAY(bus) ((bus) == 10 || (bus) == 20)

#define DEV_10 (&bus_spi0)
#define DEV_20 (&bus_i2c1)
#define DEVICE_DT_GET_1(bus) DEV_##bus
#define DEVICE_DT_GET(bus) DEVICE_DT_GET_1(bus)

#define CHILDREN_10(fn) fn(1)
#if HOST_SHARED_MAG_BUS
#define CHILDREN_20(fn) fn(2) fn(3)
#else
#define CHILDREN_20(fn) fn(2)
#endif
#define DT_FOREACH_CHILD_STATUS_OKAY_1(bus, fn) CHILDREN_##bus(fn)
#define DT_FOREACH_CHILD_STATUS_OKAY(bus, fn) DT_FOREACH_CHILD_STATUS_OKAY_1(bus, fn)

static int spi_suspend_calls;
static int spi_resume_calls;
static int i2c_suspend_calls;
static int i2c_resume_calls;

static int pm_device_action_run(const struct device *dev, enum pm_device_action action)
{
	if (dev == &bus_spi0) {
		if (action == PM_DEVICE_ACTION_SUSPEND) {
			spi_suspend_calls++;
		} else {
			spi_resume_calls++;
		}
		return 0;
	}
	if (dev == &bus_i2c1) {
		if (action == PM_DEVICE_ACTION_SUSPEND) {
			i2c_suspend_calls++;
		} else {
			i2c_resume_calls++;
		}
		return 0;
	}
	return -ENODEV;
}

/* Each scenario runs in a fresh process, like a fresh power-thread boot. */
static int boot_success_calls;
static char events[16];
static size_t event_count;

static void record_event(char event)
{
	assert(event_count + 1 < sizeof(events));
	events[event_count++] = event;
	events[event_count] = '\0';
}

#if defined(CONFIG_BOOTLOADER_MCUBOOT)
#define BOOT_SWAP_TYPE_NONE   1
#define BOOT_SWAP_TYPE_TEST   2
#define BOOT_SWAP_TYPE_PERM   3
#define BOOT_SWAP_TYPE_REVERT 4

static bool stub_img_confirmed;
static int stub_write_error;
static int write_calls;

static bool boot_is_img_confirmed(void)
{
	return stub_img_confirmed;
}

static int boot_write_img_confirmed(void)
{
	write_calls++;
	if (stub_write_error) {
		record_event('F');
		return stub_write_error;
	}
	stub_img_confirmed = true;
	record_event('C');
	return 0;
}
#endif

#if defined(CONFIG_BOOTLOADER_MCUBOOT) && IS_ENABLED(CONFIG_MCUBOOT_CONFIRM_REQUIRE_LINK)
#define SYS_REBOOT_COLD 0

struct led_connection_facts {
	bool healthy;
	bool radio_required;
	bool paired;
	bool pairing;
};

static int stub_swap_type = BOOT_SWAP_TYPE_NONE;
static int swap_type_calls;
static int link_query_calls;
static bool stub_link_healthy;
static int log_flush_calls;
static int reboot_calls;
static int last_reboot_type = -1;

static int mcuboot_swap_type(void)
{
	swap_type_calls++;
	return stub_swap_type;
}

static void esb_led_connection_facts(struct led_connection_facts *facts)
{
	link_query_calls++;
	facts->healthy = stub_link_healthy;
}

static void wait_for_logging(void)
{
	log_flush_calls++;
}

static void sys_reboot(int type)
{
	reboot_calls++;
	last_reboot_type = type;
	record_event('R');
}
#endif

static void watchdog_mark_boot_success(void)
{
#if defined(CONFIG_BOOTLOADER_MCUBOOT)
	assert(stub_img_confirmed);
#endif
	boot_success_calls++;
	record_event('M');
}

#include "production.inc"

static void test_shared_bus_suspend(void)
{
	spi_suspend_calls = 0;
	spi_resume_calls = 0;
	i2c_suspend_calls = 0;
	i2c_resume_calls = 0;

	assert(sys_interface_suspend() == 0);
	assert(sys_interface_resume() == 0);

	assert(spi_suspend_calls == 1);
	assert(spi_resume_calls == 1);
#if HOST_SHARED_MAG_BUS
	/* Shared bus with foreign PMIC child must stay powered across OTA/sleep. */
	assert(i2c_suspend_calls == 0);
	assert(i2c_resume_calls == 0);
#else
	/* Exclusive sensor bus is still suspended and resumed normally. */
	assert(i2c_suspend_calls == 1);
	assert(i2c_resume_calls == 1);
#endif
}

static void test_boot_success(void)
{
#if defined(CONFIG_BOOTLOADER_MCUBOOT)
	stub_img_confirmed = true;
#endif
	power_check_boot_success(59999);
	assert(boot_success_calls == 0);
	power_check_boot_success(60000);
	power_check_boot_success(180000);
	assert(boot_success_calls == 1);
	assert(strcmp(events, "M") == 0);
#if defined(CONFIG_BOOTLOADER_MCUBOOT)
	assert(write_calls == 0);
#if IS_ENABLED(CONFIG_MCUBOOT_CONFIRM_REQUIRE_LINK)
	assert(swap_type_calls == 0 && link_query_calls == 0 && reboot_calls == 0);
#endif
#endif
}

#if defined(CONFIG_BOOTLOADER_MCUBOOT)
static void test_nonrevertible(int type)
{
	(void)type;
#if IS_ENABLED(CONFIG_MCUBOOT_CONFIRM_REQUIRE_LINK)
	stub_swap_type = type;
#endif
	stub_write_error = -EIO;
	power_check_boot_success(60000);
	assert(!stub_img_confirmed && boot_success_calls == 0);
	power_check_boot_success(180000);
	assert(write_calls == 2 && boot_success_calls == 0);
#if IS_ENABLED(CONFIG_MCUBOOT_CONFIRM_REQUIRE_LINK)
	assert(swap_type_calls == 1 && link_query_calls == 0 && reboot_calls == 0);
#endif
	stub_write_error = 0;
	power_check_boot_success(180100);
	power_check_boot_success(180200);
	assert(stub_img_confirmed && write_calls == 3 && boot_success_calls == 1);
	assert(strcmp(events, "FFCM") == 0);
}

static void test_confirmed_late(void)
{
	stub_img_confirmed = true;
	power_check_boot_success(180000);
	power_check_boot_success(180100);
	assert(write_calls == 0 && boot_success_calls == 1);
	assert(strcmp(events, "M") == 0);
#if IS_ENABLED(CONFIG_MCUBOOT_CONFIRM_REQUIRE_LINK)
	assert(swap_type_calls == 0 && link_query_calls == 0 && reboot_calls == 0);
#endif
}
#endif

#if defined(CONFIG_BOOTLOADER_MCUBOOT) && IS_ENABLED(CONFIG_MCUBOOT_CONFIRM_REQUIRE_LINK)
static void test_trial_retry(void)
{
	stub_swap_type = BOOT_SWAP_TYPE_REVERT;
	stub_link_healthy = true;
	stub_write_error = -EIO;
	power_check_boot_success(59999);
	assert(swap_type_calls == 0 && write_calls == 0 && boot_success_calls == 0);
	power_check_boot_success(60000);
	assert(write_calls == 1 && boot_success_calls == 0 && !stub_img_confirmed);

	/* A previously healthy link must not authorize a later retry. */
	stub_link_healthy = false;
	stub_write_error = 0;
	power_check_boot_success(60100);
	assert(write_calls == 1 && boot_success_calls == 0);
	stub_link_healthy = true;
	power_check_boot_success(60200);
	power_check_boot_success(60300);
	assert(swap_type_calls == 1 && link_query_calls == 3);
	assert(write_calls == 2 && boot_success_calls == 1 && stub_img_confirmed);
	assert(reboot_calls == 0 && strcmp(events, "FCM") == 0);
}

static void test_unknown_reclassification(bool revertible)
{
	/* Positive PANIC, negative errno and an unknown zero must not be cached. */
	stub_swap_type = 0xff;
	power_check_boot_success(60000);
	power_check_boot_success(60100);
	stub_swap_type = -EIO;
	power_check_boot_success(60200);
	stub_swap_type = 0;
	power_check_boot_success(60300);
	assert(swap_type_calls == 4 && link_query_calls == 4);
	assert(write_calls == 0 && boot_success_calls == 0);

	stub_swap_type = revertible ? BOOT_SWAP_TYPE_REVERT : BOOT_SWAP_TYPE_NONE;
	power_check_boot_success(60400);
	assert(swap_type_calls == 5);
	if (revertible) {
		assert(write_calls == 0 && boot_success_calls == 0);
		/* The known REVERT result is cached and still needs a live link. */
		stub_swap_type = BOOT_SWAP_TYPE_NONE;
		power_check_boot_success(60500);
		assert(swap_type_calls == 5 && write_calls == 0);
		stub_link_healthy = true;
		power_check_boot_success(60600);
	}
	assert(write_calls == 1 && boot_success_calls == 1 && stub_img_confirmed);
	assert(reboot_calls == 0 && strcmp(events, "CM") == 0);
}

static void test_unknown_healthy(int type)
{
	stub_swap_type = type;
	stub_link_healthy = true;
	power_check_boot_success(60000);
	assert(swap_type_calls == 1 && link_query_calls == 1);
	assert(write_calls == 1 && boot_success_calls == 1 && stub_img_confirmed);
	assert(reboot_calls == 0 && strcmp(events, "CM") == 0);
}

static void test_trial_deadline(int type, bool healthy)
{
	stub_swap_type = type;
	stub_link_healthy = healthy;
	stub_write_error = -EIO;
	power_check_boot_success(60000);
	power_check_boot_success(179999);
	assert(reboot_calls == 0 && boot_success_calls == 0 && !stub_img_confirmed);
	assert(write_calls == (healthy ? 2 : 0));
	power_check_boot_success(180000);
	assert(swap_type_calls == (type == BOOT_SWAP_TYPE_REVERT ? 1 : 3));
	assert(reboot_calls == 1 && log_flush_calls == 1);
	assert(last_reboot_type == SYS_REBOOT_COLD);
	assert(write_calls == (healthy ? 2 : 0) && boot_success_calls == 0);
	assert(!stub_img_confirmed && strcmp(events, healthy ? "FFR" : "R") == 0);
}

static void test_confirmed_after_failure(void)
{
	stub_swap_type = BOOT_SWAP_TYPE_REVERT;
	stub_link_healthy = true;
	stub_write_error = -EIO;
	power_check_boot_success(60000);
	assert(write_calls == 1 && boot_success_calls == 0);
	stub_img_confirmed = true;
	stub_link_healthy = false;
	power_check_boot_success(180000);
	assert(reboot_calls == 0 && write_calls == 1 && boot_success_calls == 1);
	assert(strcmp(events, "FM") == 0);
}

static void test_minimum_deadline(void)
{
	assert(CONFIG_MCUBOOT_CONFIRM_LINK_DEADLINE_MS == 61000);
	stub_swap_type = BOOT_SWAP_TYPE_REVERT;
	stub_link_healthy = true;
	stub_write_error = -EIO;
	power_check_boot_success(59999);
	assert(write_calls == 0);
	power_check_boot_success(60000);
	power_check_boot_success(60900);
	assert(write_calls == 2 && reboot_calls == 0 && boot_success_calls == 0);
	power_check_boot_success(61000);
	assert(reboot_calls == 1 && write_calls == 2 && boot_success_calls == 0);
	assert(strcmp(events, "FFR") == 0);
}
#endif

int main(int argc, char **argv)
{
	assert(argc == 2);
	if (strcmp(argv[1], "shared_bus") == 0) {
		test_shared_bus_suspend();
	} else if (strcmp(argv[1], "boot_success") == 0) {
		test_boot_success();
#if defined(CONFIG_BOOTLOADER_MCUBOOT)
	} else if (strcmp(argv[1], "none") == 0) {
		test_nonrevertible(BOOT_SWAP_TYPE_NONE);
	} else if (strcmp(argv[1], "test") == 0) {
		test_nonrevertible(BOOT_SWAP_TYPE_TEST);
	} else if (strcmp(argv[1], "perm") == 0) {
		test_nonrevertible(BOOT_SWAP_TYPE_PERM);
	} else if (strcmp(argv[1], "confirmed_late") == 0) {
		test_confirmed_late();
#if IS_ENABLED(CONFIG_MCUBOOT_CONFIRM_REQUIRE_LINK)
	} else if (strcmp(argv[1], "trial_retry") == 0) {
		test_trial_retry();
	} else if (strcmp(argv[1], "unknown_revert") == 0) {
		test_unknown_reclassification(true);
	} else if (strcmp(argv[1], "unknown_none") == 0) {
		test_unknown_reclassification(false);
	} else if (strcmp(argv[1], "unknown_healthy_panic") == 0) {
		test_unknown_healthy(0xff);
	} else if (strcmp(argv[1], "unknown_healthy_eio") == 0) {
		test_unknown_healthy(-EIO);
	} else if (strcmp(argv[1], "deadline_trial_no_link") == 0) {
		test_trial_deadline(BOOT_SWAP_TYPE_REVERT, false);
	} else if (strcmp(argv[1], "deadline_trial_link") == 0) {
		test_trial_deadline(BOOT_SWAP_TYPE_REVERT, true);
	} else if (strcmp(argv[1], "deadline_panic_no_link") == 0) {
		test_trial_deadline(0xff, false);
	} else if (strcmp(argv[1], "deadline_panic_link") == 0) {
		test_trial_deadline(0xff, true);
	} else if (strcmp(argv[1], "deadline_eio_link") == 0) {
		test_trial_deadline(-EIO, true);
	} else if (strcmp(argv[1], "deadline_eio_no_link") == 0) {
		test_trial_deadline(-EIO, false);
	} else if (strcmp(argv[1], "confirmed_after_failure") == 0) {
		test_confirmed_after_failure();
	} else if (strcmp(argv[1], "minimum_deadline") == 0) {
		test_minimum_deadline();
#endif
#endif
	} else {
		fprintf(stderr, "Unknown test case: %s\n", argv[1]);
		return 2;
	}
	printf("PASS mcuboot_ota %s (shared_mag_bus=%d)\n", argv[1], HOST_SHARED_MAG_BUS);
	return 0;
}
