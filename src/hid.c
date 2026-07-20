/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include "globals.h"
#include <zephyr/devicetree.h>

#define USB_EXISTS 0
#if CONFIG_USB_DEVICE_STACK_NEXT
#undef USB_EXISTS
#define USB DT_NODELABEL(zephyr_udc0)
#define USB_EXISTS (DT_NODE_HAS_STATUS(USB, okay) && CONFIG_UART_CONSOLE)
#endif

#if CONFIG_CONNECTION_OVER_HID
#if !USB_EXISTS
#error "CONFIG_CONNECTION_OVER_HID requires USB_DEVICE_STACK_NEXT, UART_CONSOLE, and zephyr_udc0"
#endif

#include <hal/nrf_ficr.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/usb/class/usbd_hid.h>

struct tracker_report {
	uint8_t data[16];
} __packed;

#define REPORT_SIZE sizeof(struct tracker_report)
#define REPORT_COUNT (3 * 4)

/*
 * SPSC packet ring: connection thread produces, HID thread consumes.
 * No irq_lock — only these two threads touch the indices.
 */
static uint8_t reports[REPORT_COUNT * REPORT_SIZE] __aligned(sizeof(void *));
static atomic_t hid_prod; /* next write index (monotonic) */
static atomic_t hid_cons; /* next consume index (monotonic) */
static uint8_t submit_stage[REPORT_SIZE * 4] __aligned(sizeof(void *));

static const struct device *hdev;
static ATOMIC_DEFINE(hid_ep_in_busy, 1);
static bool hid_ready;
static const uint8_t *submitted_report;

#define HID_EP_BUSY_FLAG 0

LOG_MODULE_REGISTER(hid_event, LOG_LEVEL_INF);

static void hid_thread(void);
static struct k_thread hid_thread_id;
static K_THREAD_STACK_DEFINE(hid_thread_id_stack, 256);
static bool hid_thread_running;
static bool hid_thread_joinable;
static bool hid_start_pending;
static void hid_lifecycle_work_handler(struct k_work *work);
static K_WORK_DEFINE(hid_lifecycle_work, hid_lifecycle_work_handler);

static void hid_thread_start_now(void)
{
	k_thread_create(
		&hid_thread_id,
		hid_thread_id_stack,
		K_THREAD_STACK_SIZEOF(hid_thread_id_stack),
		(k_thread_entry_t)hid_thread,
		NULL,
		NULL,
		NULL,
		HID_THREAD_PRIORITY,
		0,
		K_NO_WAIT
	);
	hid_thread_running = true;
}

static void hid_lifecycle_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	if (hid_thread_joinable) {
		(void)k_thread_join(&hid_thread_id, K_FOREVER);
		hid_thread_joinable = false;
	}
	if (hid_start_pending && !hid_thread_running) {
		hid_start_pending = false;
		hid_thread_start_now();
	}
}

static const uint8_t hid_report_desc[] = {
	HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP),
	HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
	HID_COLLECTION(HID_COLLECTION_APPLICATION),
	HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
	HID_REPORT_SIZE(8),
	HID_REPORT_COUNT(64),
	HID_INPUT(0x02),
	HID_END_COLLECTION,
};

void hid_thread_create(void)
{
	if (hid_thread_running) {
		return;
	}
	if (hid_thread_joinable) {
		/* Join+start on workqueue — USB status_cb must not block. */
		hid_start_pending = true;
		k_work_submit(&hid_lifecycle_work);
		return;
	}
	hid_thread_start_now();
}

void hid_thread_abort(void)
{
	if (!hid_thread_running) {
		return;
	}
	/* Stop touching USB before abort — unplug path needs this. */
	hid_ready = false;
	submitted_report = NULL;
	atomic_clear_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG);
	hid_start_pending = false;
	k_thread_abort(&hid_thread_id);
	hid_thread_running = false;
	hid_thread_joinable = true;
	/* Join on system workqueue — USB status_cb must not block forever. */
	k_work_submit(&hid_lifecycle_work);
}

static void packet_device_addr(uint8_t *report) // associate id and tracker address
{
	report[0] = 255; // receiver packet 0
	report[1] = 0;   // for hid on tracker, id is always 0
	uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR;
	memcpy(&report[2], addr, 6);
	memset(&report[8], 0, 8); // last 8 bytes unused for now
}

static uint32_t dropped_reports = 0;
static uint16_t max_dropped_reports = 0;

int64_t last_registration_sent = 0;

#define REGISTRATION_INTERVAL 100

static void send_report(void)
{
	if (!hid_ready) {
		return;
	}

	uint32_t cons = (uint32_t)atomic_get(&hid_cons);
	uint32_t prod = (uint32_t)atomic_get(&hid_prod);
	uint32_t pending = prod - cons;

	if (pending == 0
		&& k_uptime_get() - REGISTRATION_INTERVAL < last_registration_sent) {
		return;
	}

	if (atomic_test_and_set_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG)) {
		return;
	}

	for (uint32_t i = 0; i < 4; i++) {
		uint8_t *dst = &submit_stage[REPORT_SIZE * i];
		if (i < pending) {
			uint32_t idx = (cons + i) % REPORT_COUNT;
			memcpy(dst, &reports[REPORT_SIZE * idx], REPORT_SIZE);
		} else {
			packet_device_addr(dst);
		}
	}

	/* Drop anything beyond one USB frame; publish consume before submit. */
	if (pending > 4) {
		dropped_reports += pending - 4;
		if (dropped_reports > max_dropped_reports) {
			max_dropped_reports = dropped_reports;
		}
	}
	atomic_set(&hid_cons, (atomic_val_t)prod);

	submitted_report = submit_stage;
	int ret = hid_device_submit_report(hdev, REPORT_SIZE * 4, submitted_report);

	if (ret != 0) {
		submitted_report = NULL;
		atomic_clear_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG);
		if (ret != -EACCES) {
			LOG_ERR("Failed to submit report: %d", ret);
		}
		return;
	}

	last_registration_sent = k_uptime_get();
}

int64_t last_dropped_report_log = 0;

#define DROPPED_REPORT_LOG_INTERVAL 5000

void hid_thread(void)
{
	while (1) {
		send_report();

		if (k_uptime_get() - DROPPED_REPORT_LOG_INTERVAL > last_dropped_report_log) {
			if (dropped_reports) {
				LOG_INF("Dropped reports: %u (max: %u)", dropped_reports, max_dropped_reports);
			}
			dropped_reports = 0;
			max_dropped_reports = 0;
			last_dropped_report_log = k_uptime_get();
		}

		k_msleep(1);
	}
}

static void int_in_ready_cb(const struct device *dev)
{
	if (dev != hdev) {
		return;
	}

	if (!atomic_test_and_clear_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG) && hid_ready) {
		LOG_WRN("IN endpoint callback without preceding buffer write");
	}
}

static void input_report_done_cb(const struct device *dev, const uint8_t *const report)
{
	if (dev != hdev) {
		return;
	}

	if (report != submitted_report && hid_ready) {
		LOG_WRN("IN endpoint callback for unexpected report buffer");
	}
	submitted_report = NULL;
	int_in_ready_cb(dev);
}

static void iface_ready_cb(const struct device *dev, const bool ready)
{
	if (dev != hdev) {
		return;
	}

	hid_ready = ready;
	if (!ready) {
		submitted_report = NULL;
		atomic_clear_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG);
	}
}

static int
get_report_cb(const struct device *dev, const uint8_t type, const uint8_t id, const uint16_t len, uint8_t *const buf)
{
	if (dev != hdev) {
		return -ENODEV;
	}

	if (type != HID_REPORT_TYPE_INPUT || id != 0U) {
		return -ENOTSUP;
	}

	if (len < REPORT_SIZE * 4 || buf == NULL) {
		return -EINVAL;
	}

	memset(buf, 0, REPORT_SIZE * 4);
	for (int i = 0; i < 4; i++) {
		packet_device_addr(&buf[REPORT_SIZE * i]);
	}

	return REPORT_SIZE * 4;
}

static const struct hid_device_ops ops = {
	.iface_ready = iface_ready_cb,
	.get_report = get_report_cb,
	.input_report_done = input_report_done_cb,
};

static int composite_pre_init(void)
{
	hdev = DEVICE_DT_GET(DT_NODELABEL(hid_dev_0));
	if (!device_is_ready(hdev)) {
		LOG_ERR("Cannot get USB HID Device");
		return -ENODEV;
	}

	LOG_INF("HID Device: dev %p", hdev);

	int ret = hid_device_register(hdev, hid_report_desc, sizeof(hid_report_desc), &ops);
	if (ret != 0) {
		LOG_ERR("Failed to register HID device: %d", ret);
		return ret;
	}

	return 0;
}

SYS_INIT(composite_pre_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

void hid_write_packet_n(const uint8_t *data)
{
	uint32_t prod = (uint32_t)atomic_get(&hid_prod);
	uint32_t cons = (uint32_t)atomic_get(&hid_cons);

	if ((prod - cons) >= REPORT_COUNT) {
		return;
	}

	uint8_t *report = &reports[REPORT_SIZE * (prod % REPORT_COUNT)];

	memcpy(report, data, REPORT_SIZE);
	if (data[0] != 1 && data[0] != 4) {
		report[15] = 0;
	}
	report[1] = 0;
	atomic_set(&hid_prod, (atomic_val_t)(prod + 1));
}

#endif
