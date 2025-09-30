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

#if CONFIG_USB_DEVICE_STACK
#define USB DT_NODELABEL(usbd)
#define USB_EXISTS (DT_NODE_HAS_STATUS(USB, okay) && CONFIG_UART_CONSOLE)
#endif

#if USB_EXISTS
#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>

static struct tracker_report {
	uint8_t data[16];
} __packed report = {
	.data = {0}
};;

#define REPORT_SIZE sizeof(report)
#define REPORT_COUNT (3 * 4)
#define REPORT_THRESHOLD (REPORT_COUNT - 4) // buffer loop point

uint8_t reports[REPORT_COUNT * REPORT_SIZE]; // 3 report buffer (and 4 "packets" each)
uint8_t report_count = 0;
uint8_t report_sent = 0;

static const struct device *hdev;
static ATOMIC_DEFINE(hid_ep_in_busy, 1);

#define HID_EP_BUSY_FLAG 0

LOG_MODULE_REGISTER(hid_event, LOG_LEVEL_INF);

static void hid_thread(void);
static struct k_thread hid_thread_id;
static K_THREAD_STACK_DEFINE(hid_thread_id_stack, 256);

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
	k_thread_create(&hid_thread_id, hid_thread_id_stack, K_THREAD_STACK_SIZEOF(hid_thread_id_stack), (k_thread_entry_t)hid_thread, NULL, NULL, NULL, 6, 0, K_NO_WAIT);
}

void hid_thread_abort(void)
{
	k_thread_abort(&hid_thread_id);
}

static void packet_device_addr(uint8_t *report) // associate id and tracker address
{
	report[0] = 255; // receiver packet 0
	report[1] = 0; // for hid on tracker, id is always 0
	uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
	memcpy(&report[2], addr, 6);
	memset(&report[8], 0, 8); // last 8 bytes unused for now
}

static uint32_t dropped_reports = 0;
static uint16_t max_dropped_reports = 0;

int64_t last_registration_sent = 0;

#define REGISTRATION_INTERVAL 100

static void send_report()
{
	if (report_count == 0 && k_uptime_get() - REGISTRATION_INTERVAL < last_registration_sent) // send registrations only every 100ms (also acting as a keep alive)
		return;

	int ret, wrote;

	last_registration_sent = k_uptime_get();

	if (!atomic_test_and_set_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG))
	{
		// fill report; send associated address for server to register
		for (int i = report_count; i < 4; i++)
			packet_device_addr(&reports[REPORT_SIZE * (report_sent + i)]);

		ret = hid_int_ep_write(hdev, &reports[REPORT_SIZE * report_sent], REPORT_SIZE * 4, &wrote);
		report_sent += 4;

		// track if more reports were waiting than sent
		if (report_count > 4)
		{
			dropped_reports += report_count - 4;
			if (dropped_reports > max_dropped_reports)
				max_dropped_reports = dropped_reports;
		}

		// loop counters
		if (report_sent >= REPORT_THRESHOLD)
			report_sent = 0;
		report_count = 0;

		if (ret != 0)
		{
			/*
			 * Do nothing and wait until host has reset the device
			 * and hid_ep_in_busy is cleared.
			 */
			LOG_ERR("Failed to submit report");
		}
	}
}

int64_t last_dropped_report_log = 0;

#define DROPPED_REPORT_LOG_INTERVAL 5000

void hid_thread(void)
{
	while (1)
	{
		send_report();

		if (k_uptime_get() - DROPPED_REPORT_LOG_INTERVAL > last_dropped_report_log)
		{
			if (dropped_reports)
    	        LOG_INF("Dropped reports: %u (max: %u)", dropped_reports, max_dropped_reports);
			dropped_reports = 0;
			max_dropped_reports = 0;
			last_dropped_report_log = k_uptime_get();
		}

		k_msleep(1);
	}
}

static void int_in_ready_cb(const struct device *dev)
{
	ARG_UNUSED(dev);
	if (!atomic_test_and_clear_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG))
		LOG_WRN("IN endpoint callback without preceding buffer write");
}

void hid_int_in_ready(void)
{
	int_in_ready_cb(hdev);
}

/*
 * On Idle callback is available here as an example even if actual use is
 * very limited. In contrast to report_event_handler(),
 * report value is not incremented here.
 */
static void on_idle_cb(const struct device *dev, uint16_t report_id)
{
	LOG_DBG("On idle callback");
}

static void protocol_cb(const struct device *dev, uint8_t protocol)
{
	LOG_INF("New protocol: %s", protocol == HID_PROTOCOL_BOOT ? "boot" : "report");
}

static const struct hid_ops ops = {
	.int_in_ready = int_in_ready_cb,
	.on_idle = on_idle_cb,
	.protocol_change = protocol_cb,
};

static int composite_pre_init(void)
{
	hdev = device_get_binding("HID_0");
	if (hdev == NULL)
	{
		LOG_ERR("Cannot get USB HID Device");
		return -ENODEV;
	}

	LOG_INF("HID Device: dev %p", hdev);

	usb_hid_register_device(hdev, hid_report_desc, sizeof(hid_report_desc), &ops);

	atomic_set_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG);

	if (usb_hid_set_proto_code(hdev, HID_BOOT_IFACE_CODE_NONE))
		LOG_WRN("Failed to set Protocol Code");

	return usb_hid_init(hdev);
}

SYS_INIT(composite_pre_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
#endif

//|b0      |b1      |b2      |b3      |b4      |b5      |b6      |b7      |b8      |b9      |b10     |b11     |b12     |b13     |b14     |b15     |
//|type    |id      |packet data                                                                                                                  |
//|0       |id      |proto   |batt    |batt_v  |temp    |brd_id  |mcu_id  |imu_id  |mag_id  |fw_date          |major   |minor   |patch   |rssi    |
//|1       |id      |q0               |q1               |q2               |q3               |a0               |a1               |a2               |
//|2       |id      |batt    |batt_v  |temp    |q_buf                              |a0               |a1               |a2               |rssi    |
//|3	   |id      |svr_stat|status  |resv                                                                                              |rssi    |
//|4       |id      |q0               |q1               |q2               |q3               |m0               |m1               |m2               |
//|255     |id      |addr                                                 |resv                                                                   |

void hid_write_packet_n(uint8_t *data)
{
#if USB_EXISTS && CONFIG_CONNECTION_OVER_HID
	memcpy(&report.data, data, 16); // all data can be passed through
	if (data[0] != 1 && data[0] != 4) // packet 1 and 4 are full precision quat and accel/mag, no room for rssi
		report.data[15] = 0; // rssi is always -0dBm for hid from tracker
	report.data[1] = 0; // id is always 0 for hid from tracker
	if (report_sent + report_count >= REPORT_COUNT) // overflow
		return;
	memcpy(&reports[sizeof(report) * (report_sent + report_count)], &report, sizeof(report));
	report_count++;
#endif
}
