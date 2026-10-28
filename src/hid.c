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

#define PACKET_SIZE_BYTES sizeof(struct tracker_report)
#define PACKETS_PER_USB_REPORT 4
#define USB_REPORT_SIZE_BYTES (PACKET_SIZE_BYTES * PACKETS_PER_USB_REPORT)
#define RING_PACKET_CAPACITY (3 * PACKETS_PER_USB_REPORT)

/*
 * The connection thread produces and the HID thread consumes this packet
 * ring. The state lock also excludes producers while a session is retired.
 */
static uint8_t reports[RING_PACKET_CAPACITY * PACKET_SIZE_BYTES] __aligned(sizeof(void *));
static atomic_t hid_prod; /* next write index (monotonic) */
static atomic_t hid_cons; /* next consume index (monotonic) */
static uint8_t submit_stage[USB_REPORT_SIZE_BYTES] __aligned(sizeof(void *));

static const struct device *hdev;
/* Protects configuration, interface readiness, report ownership and ring
 * publication. USB callbacks never wait for the submitting thread. */
static struct k_spinlock hid_state_lock;
static bool hid_configured;
static bool hid_ready;
static bool hid_report_inflight;
static K_MUTEX_DEFINE(hid_submit_lock);
static K_SEM_DEFINE(hid_wake, 0, 1);

LOG_MODULE_REGISTER(hid_event, LOG_LEVEL_INF);

static void hid_thread(void *p1, void *p2, void *p3);
K_THREAD_DEFINE(hid_thread_id, 256, hid_thread, NULL, NULL, NULL, HID_THREAD_PRIORITY, 0, 0);

static const uint8_t hid_report_desc[] = {
	HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP),
	HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
	HID_COLLECTION(HID_COLLECTION_APPLICATION),
	HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
	HID_REPORT_SIZE(8),
	HID_REPORT_COUNT(USB_REPORT_SIZE_BYTES),
	HID_INPUT(0x02),
	HID_END_COLLECTION,
};

void hid_set_configured(bool configured)
{
	k_spinlock_key_t key = k_spin_lock(&hid_state_lock);

	if (hid_configured != configured) {
		hid_configured = configured;
		if (!configured) {
			/* The producer is excluded by this lock, so no old-session
			 * packet can be published after the ring is discarded. */
			atomic_set(&hid_cons, atomic_get(&hid_prod));
		}
	}
	k_spin_unlock(&hid_state_lock, key);
	k_sem_give(&hid_wake);
}

void hid_wait_for_idle(void)
{
	/* Called by the USB control thread only after submissions are stopped.
	 * Do not wait for the report completion: its callback may require the
	 * USB stack to finish teardown first. */
	k_mutex_lock(&hid_submit_lock, K_FOREVER);
	k_mutex_unlock(&hid_submit_lock);
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
	int64_t now = k_uptime_get();
	k_mutex_lock(&hid_submit_lock, K_FOREVER);
	k_spinlock_key_t key = k_spin_lock(&hid_state_lock);

	if (!hid_configured || !hid_ready || hid_report_inflight) {
		k_spin_unlock(&hid_state_lock, key);
		k_mutex_unlock(&hid_submit_lock);
		return;
	}

	uint32_t cons = (uint32_t)atomic_get(&hid_cons);
	uint32_t prod = (uint32_t)atomic_get(&hid_prod);
	uint32_t pending = prod - cons;

	if (pending == 0 && now - REGISTRATION_INTERVAL < last_registration_sent) {
		k_spin_unlock(&hid_state_lock, key);
		k_mutex_unlock(&hid_submit_lock);
		return;
	}

	for (uint32_t i = 0; i < PACKETS_PER_USB_REPORT; i++) {
		uint8_t *dst = &submit_stage[PACKET_SIZE_BYTES * i];
		if (i < pending) {
			uint32_t idx = (cons + i) % RING_PACKET_CAPACITY;
			memcpy(dst, &reports[PACKET_SIZE_BYTES * idx], PACKET_SIZE_BYTES);
		} else {
			packet_device_addr(dst);
		}
	}

	/* Drop anything beyond one USB frame; publish consume before submit. */
	if (pending > PACKETS_PER_USB_REPORT) {
		dropped_reports += pending - PACKETS_PER_USB_REPORT;
		if (dropped_reports > max_dropped_reports) {
			max_dropped_reports = dropped_reports;
		}
	}
	atomic_set(&hid_cons, (atomic_val_t)prod);
	hid_report_inflight = true;
	k_spin_unlock(&hid_state_lock, key);

	int ret = hid_device_submit_report(hdev, USB_REPORT_SIZE_BYTES, submit_stage);
	if (ret != 0) {
		/* A failed submission does not transfer buffer ownership. */
		key = k_spin_lock(&hid_state_lock);
		hid_report_inflight = false;
		k_spin_unlock(&hid_state_lock, key);
		if (ret != -EACCES) {
			LOG_ERR("Failed to submit report: %d", ret);
		}
	} else {
		last_registration_sent = k_uptime_get();
	}
	k_mutex_unlock(&hid_submit_lock);
}

int64_t last_dropped_report_log = 0;

#define DROPPED_REPORT_LOG_INTERVAL 5000

static void hid_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		k_spinlock_key_t key = k_spin_lock(&hid_state_lock);
		bool configured = hid_configured;
		k_spin_unlock(&hid_state_lock, key);
		if (!configured) {
			(void)k_sem_take(&hid_wake, K_FOREVER);
			continue;
		}

		send_report();

		if (k_uptime_get() - DROPPED_REPORT_LOG_INTERVAL > last_dropped_report_log) {
			if (dropped_reports) {
				LOG_INF("Dropped reports: %u (max: %u)", dropped_reports, max_dropped_reports);
			}
			dropped_reports = 0;
			max_dropped_reports = 0;
			last_dropped_report_log = k_uptime_get();
		}

		(void)k_sem_take(&hid_wake, K_MSEC(1));
	}
}

static void input_report_done_cb(const struct device *dev, const uint8_t *const report)
{
	if (dev != hdev) {
		return;
	}

	k_spinlock_key_t key = k_spin_lock(&hid_state_lock);
	bool unexpected = report != submit_stage || !hid_report_inflight;
	bool ready = hid_ready;
	if (!unexpected) {
		hid_report_inflight = false;
	}
	k_spin_unlock(&hid_state_lock, key);

	if (unexpected && ready) {
		LOG_WRN("IN endpoint callback for unexpected report buffer");
	}
	if (!unexpected) {
		k_sem_give(&hid_wake);
	}
}

static void iface_ready_cb(const struct device *dev, const bool ready)
{
	if (dev != hdev) {
		return;
	}

	k_spinlock_key_t key = k_spin_lock(&hid_state_lock);
	hid_ready = ready;
	/* A disabled interface does not return ownership of a submitted buffer.
	 * Reuse it only after input_report_done, even across a new configuration. */
	k_spin_unlock(&hid_state_lock, key);
	k_sem_give(&hid_wake);
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

	if (len < USB_REPORT_SIZE_BYTES || buf == NULL) {
		return -EINVAL;
	}

	memset(buf, 0, USB_REPORT_SIZE_BYTES);
	for (int i = 0; i < PACKETS_PER_USB_REPORT; i++) {
		packet_device_addr(&buf[PACKET_SIZE_BYTES * i]);
	}

	return USB_REPORT_SIZE_BYTES;
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

bool hid_write_packet_n(const uint8_t *data)
{
	k_spinlock_key_t key = k_spin_lock(&hid_state_lock);
	uint32_t prod = (uint32_t)atomic_get(&hid_prod);
	uint32_t cons = (uint32_t)atomic_get(&hid_cons);

	if (!hid_configured || (prod - cons) >= RING_PACKET_CAPACITY) {
		k_spin_unlock(&hid_state_lock, key);
		return false;
	}

	uint8_t *report = &reports[PACKET_SIZE_BYTES * (prod % RING_PACKET_CAPACITY)];

	memcpy(report, data, PACKET_SIZE_BYTES);
	if (data[0] != 1 && data[0] != 4) {
		report[15] = 0;
	}
	report[1] = 0;
	atomic_set(&hid_prod, (atomic_val_t)(prod + 1));
	k_spin_unlock(&hid_state_lock, key);
	k_sem_give(&hid_wake);
	return true;
}

#endif
