#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define IS_ENABLED(x) (x)
#define LOG_ERR(...) ((void)0)

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

/* MCUboot and receiver link stubs for power_mcuboot_trial_confirm_ready(). */
#define BOOT_SWAP_TYPE_NONE   1
#define BOOT_SWAP_TYPE_REVERT 3
#define BOOT_SWAP_TYPE_PANIC  0xff
#define SYS_REBOOT_COLD       0

struct led_connection_facts {
	bool healthy;
	bool radio_required;
	bool paired;
	bool pairing;
};

static bool stub_img_confirmed;
static int stub_swap_type;
static int swap_type_calls;
static int link_query_calls;
static bool stub_link_healthy;
static int log_flush_calls;
static int reboot_calls;
static int last_reboot_type = -1;

static bool boot_is_img_confirmed(void)
{
	return stub_img_confirmed;
}

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
}

#include "production.inc"

static void reset_confirm_stubs(void)
{
	stub_img_confirmed = false;
	stub_swap_type = BOOT_SWAP_TYPE_NONE;
	swap_type_calls = 0;
	link_query_calls = 0;
	stub_link_healthy = false;
	log_flush_calls = 0;
	reboot_calls = 0;
	last_reboot_type = -1;
}

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

static void test_trial_confirm_gating(void)
{
	int cached_swap = -1;

	/* 1. Already-confirmed image: ready immediately, no flash trailer read. */
	reset_confirm_stubs();
	stub_img_confirmed = true;
	stub_swap_type = BOOT_SWAP_TYPE_REVERT;
	assert(power_mcuboot_trial_confirm_ready(61000, &cached_swap) == true);
	assert(swap_type_calls == 0);
	assert(link_query_calls == 0);
	assert(reboot_calls == 0);

	/* 2. Freshly programmed unconfirmed image (not revertible): confirms without link. */
	reset_confirm_stubs();
	cached_swap = -1;
	stub_swap_type = BOOT_SWAP_TYPE_NONE;
	assert(power_mcuboot_trial_confirm_ready(61000, &cached_swap) == true);
	assert(swap_type_calls == 1);
	assert(cached_swap == BOOT_SWAP_TYPE_NONE);
	assert(link_query_calls == 0);
	assert(reboot_calls == 0);

	/* 3. Revertible trial: waits until healthy link; caches swap type. */
	reset_confirm_stubs();
	cached_swap = -1;
	stub_swap_type = BOOT_SWAP_TYPE_REVERT;
	assert(power_mcuboot_trial_confirm_ready(61000, &cached_swap) == false);
	assert(power_mcuboot_trial_confirm_ready(180000, &cached_swap) == false);
	assert(swap_type_calls == 1);
	assert(link_query_calls == 2);
	assert(reboot_calls == 0);

	/* Link becomes healthy before deadline -> ready to confirm. */
	stub_link_healthy = true;
	assert(power_mcuboot_trial_confirm_ready(180000, &cached_swap) == true);
	assert(swap_type_calls == 1);
	assert(reboot_calls == 0);

	/* 4. Revertible trial past deadline with no link -> cold reboot to revert. */
	reset_confirm_stubs();
	cached_swap = -1;
	stub_swap_type = BOOT_SWAP_TYPE_REVERT;
	assert(power_mcuboot_trial_confirm_ready(180001, &cached_swap) == false);
	assert(swap_type_calls == 1);
	assert(log_flush_calls == 1);
	assert(reboot_calls == 1);
	assert(last_reboot_type == SYS_REBOOT_COLD);

	/* 5. Failed swap-state read: not cached, needs a link, retried next pass. */
	reset_confirm_stubs();
	cached_swap = -1;
	stub_swap_type = BOOT_SWAP_TYPE_PANIC;
	assert(power_mcuboot_trial_confirm_ready(61000, &cached_swap) == false);
	assert(cached_swap == -1);
	assert(link_query_calls == 1);
	assert(reboot_calls == 0);

	/* Read recovers as a revertible trial: still waits for the link. */
	stub_swap_type = BOOT_SWAP_TYPE_REVERT;
	assert(power_mcuboot_trial_confirm_ready(62000, &cached_swap) == false);
	assert(swap_type_calls == 2);
	assert(cached_swap == BOOT_SWAP_TYPE_REVERT);

	/* Persistent failure past the deadline reboots rather than confirming. */
	cached_swap = -1;
	stub_swap_type = BOOT_SWAP_TYPE_PANIC;
	assert(power_mcuboot_trial_confirm_ready(180001, &cached_swap) == false);
	assert(reboot_calls == 1);
}

int main(void)
{
	test_shared_bus_suspend();
	test_trial_confirm_gating();
	printf("PASS mcuboot_ota (shared_mag_bus=%d)\n", HOST_SHARED_MAG_BUS);
	return 0;
}
