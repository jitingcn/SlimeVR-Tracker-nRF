#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/ztest.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include "thread_priority.h"

/* Production HID/USB functions and the SDK Nordic enqueue/lock functions are
 * extracted at build time. Hardware, USB message metadata and endpoint events
 * are modeled; all mutexes, threads, semaphores and workqueue scheduling are
 * real Zephyr kernel operations. */
#define CONFIG_CONNECTION_OVER_HID 1
#define USB_CONSOLE_IS_CDC 0
#undef LOG_DBG
#undef LOG_ERR
#undef LOG_WRN
#define LOG_DBG(...) ((void)0)
#define LOG_ERR(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define USB_CONTROL_EP_OUT 0
#define USB_CONTROL_EP_IN 0x80
#define USB_EP_DIR_IS_IN(ep) ((ep) & 0x80)

struct udc_buf_info { unsigned int ep; bool setup; };
struct net_buf { size_t len, size; struct udc_buf_info info; };
struct udc_ep_config { unsigned int addr; struct { bool enabled; } stat; };
struct udc_data { struct k_mutex mutex; bool setup_pending, setup_valid; unsigned char setup[8]; };
struct udc_api {
	void (*lock)(const struct device *);
	void (*unlock)(const struct device *);
	int (*ep_enqueue)(const struct device *, struct udc_ep_config *, struct net_buf *);
};
static struct udc_data controller_data;
static struct udc_ep_config endpoint = { .addr = 0x81, .stat.enabled = true };
static bool controller_enabled, stack_enabled;
static struct udc_buf_info *udc_get_buf_info(struct net_buf *buf) { return &buf->info; }
static bool udc_is_enabled(const struct device *dev) { ARG_UNUSED(dev); return controller_enabled; }
static struct udc_ep_config *udc_get_ep_cfg(const struct device *dev, unsigned int ep)
{
	ARG_UNUSED(dev);
	return ep == endpoint.addr ? &endpoint : NULL;
}
static void net_buf_add_mem(struct net_buf *buf, const void *data, size_t n)
{
	ARG_UNUSED(buf); ARG_UNUSED(data); ARG_UNUSED(n);
	zassert_unreachable("unexpected control transfer");
}
static int udc_submit_ep_event(const struct device *dev, struct net_buf *buf, int error)
{
	ARG_UNUSED(dev); ARG_UNUSED(buf); ARG_UNUSED(error);
	zassert_unreachable("unexpected control transfer");
	return 0;
}
#include "udc_enqueue.inc"

static struct { uint32_t DEVICEADDR[2]; } ficr __aligned(sizeof(uint64_t));
#define NRF_FICR (&ficr)
static const struct device hid_device = { .name = "modeled-hid" };
static int hid_device_submit_report(const struct device *dev, uint16_t size, const uint8_t *report);
#include "hid_lifecycle.inc"

static atomic_t enqueue_calls;
static bool block_enqueue;
static bool complete_during_enqueue;
static int enqueue_error;
static const uint8_t *last_report;
static K_SEM_DEFINE(enqueue_entered, 0, 1);
static K_SEM_DEFINE(release_enqueue, 0, 1);
static K_SEM_DEFINE(sender_done, 0, 1);
static K_SEM_DEFINE(control_started, 0, 1);
static K_SEM_DEFINE(control_done, 0, 1);
static K_SEM_DEFINE(callback_done, 0, 1);
static K_SEM_DEFINE(release_work_done, 0, 1);

static int hardware_enqueue(const struct device *dev, struct udc_ep_config *cfg, struct net_buf *buf)
{
	ARG_UNUSED(dev); ARG_UNUSED(cfg); ARG_UNUSED(buf);
	atomic_inc(&enqueue_calls);
	if (block_enqueue) {
		k_sem_give(&enqueue_entered);
		(void)k_sem_take(&release_enqueue, K_FOREVER);
	}
	if (complete_during_enqueue && enqueue_error == 0) {
		/* The callback can run before the submission API returns. */
		input_report_done_cb(&hid_device, last_report);
	}
	return enqueue_error;
}
static const struct udc_api controller_api = {
	.lock = udc_nrf_lock, .unlock = udc_nrf_unlock, .ep_enqueue = hardware_enqueue,
};
static const struct device controller = {
	.name = "modeled-controller", .data = &controller_data, .api = &controller_api,
};
static int hid_device_submit_report(const struct device *dev, uint16_t size, const uint8_t *report)
{
	zassert_equal_ptr(dev, &hid_device);
	last_report = report;
	struct net_buf buf = { .len = size, .size = size, .info.ep = 0x81 };
	return udc_ep_enqueue(&controller, &buf);
}

/* The actual USB transition code calls these modeled hardware/status leaves. */
struct usbd_context { int unused; };
struct usbd_msg { int type; int status; const struct device *dev; };
enum {
	USBD_MSG_RESET, USBD_MSG_CONFIGURATION, USBD_MSG_VBUS_READY, USBD_MSG_VBUS_REMOVED,
	USBD_MSG_CDC_ACM_CONTROL_LINE_STATE, USBD_MSG_CDC_ACM_LINE_CODING,
};
#define SYS_STATUS_USB_CONNECTED 1
static int status_bits;
static void set_status(int status, bool set)
{
	if (set) { status_bits |= status; } else { status_bits &= ~status; }
}
static struct usbd_context slimevr_usbd;
static const struct device console_device = { .name = "modeled-console" };
#undef DEVICE_DT_GET
#define DEVICE_DT_GET(node) (&console_device)
static atomic_t disable_calls, enable_calls;
static int disable_error, enable_error;
static char actions[8];
static unsigned int action_count;
static int setup_error, setup_event;
static bool can_detect_vbus, physical_vbus;
static bool configure_during_enable;
static void status_cb(struct usbd_context *const ctx, const struct usbd_msg *const msg);

static int usbd_setup(void (*callback)(struct usbd_context *, const struct usbd_msg *))
{
	if (setup_error) {
		return setup_error;
	}
	if (setup_event >= 0) {
		const struct usbd_msg msg = { .type = setup_event };
		callback(&slimevr_usbd, &msg);
	}
	return 0;
}
static bool usbd_can_detect_vbus(struct usbd_context *ctx)
{
	ARG_UNUSED(ctx);
	return can_detect_vbus;
}
static bool vbus_read(void) { return physical_vbus; }

static int usbd_disable(struct usbd_context *ctx)
{
	zassert_equal_ptr(ctx, &slimevr_usbd);
	atomic_inc(&disable_calls);
	actions[action_count++] = 'D';
	if (!stack_enabled) {
		return -EALREADY;
	}
	/* usbd_config_set(0) disables the class before UDC disable. */
	iface_ready_cb(&hid_device, false);
	/* The pinned SDK clears both software enabled flags even when the
	 * controller driver's disable operation returns an error. */
	udc_nrf_lock(&controller);
	controller_enabled = false;
	stack_enabled = false;
	udc_nrf_unlock(&controller);
	return disable_error;
}
static int usbd_enable(struct usbd_context *ctx)
{
	zassert_equal_ptr(ctx, &slimevr_usbd);
	atomic_inc(&enable_calls);
	actions[action_count++] = 'E';
	if (configure_during_enable) {
		const struct usbd_msg msg = { .type = USBD_MSG_CONFIGURATION, .status = 1 };
		iface_ready_cb(&hid_device, true);
		status_cb(ctx, &msg);
	}
	if (enable_error) {
		return enable_error;
	}
	controller_enabled = true;
	stack_enabled = true;
	return 0;
}
#include "usb_lifecycle.inc"

static struct k_thread sender_thread, control_thread;
K_THREAD_STACK_DEFINE(sender_stack, 2048);
K_THREAD_STACK_DEFINE(control_stack, 2048);

static void send_entry(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	send_report();
	k_sem_give(&sender_done);
}
static void control_entry(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	k_sem_give(&control_started);
	usb_hid_service_device();
	k_sem_give(&control_done);
}
static void notify(int type, int status)
{
	struct usbd_msg msg = { .type = type, .status = status };
	status_cb(&slimevr_usbd, &msg);
}
static void disconnect_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	notify(USBD_MSG_VBUS_REMOVED, 0);
	notify(USBD_MSG_VBUS_READY, 0);
	/* An already queued old CONFIGURATION must not bypass teardown. */
	notify(USBD_MSG_CONFIGURATION, 1);
	k_sem_give(&callback_done);
}
static void release_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	k_sem_give(&release_enqueue);
	k_sem_give(&release_work_done);
}
K_WORK_DEFINE(disconnect_work, disconnect_work_handler);
K_WORK_DEFINE(release_work, release_work_handler);

static void fixture_start(void)
{
	k_mutex_init(&controller_data.mutex);
	k_mutex_init(&hid_submit_lock);
	k_mutex_init(&usb_serial_transition_lock);
	k_sem_reset(&enqueue_entered); k_sem_reset(&release_enqueue); k_sem_reset(&sender_done);
	k_sem_reset(&control_started); k_sem_reset(&control_done); k_sem_reset(&callback_done);
	k_sem_reset(&release_work_done); k_sem_reset(&hid_wake); k_sem_reset(&usb_ctrl_event);
	atomic_set(&hid_prod, 0); atomic_set(&hid_cons, 0);
	atomic_set(&enqueue_calls, 0); atomic_set(&disable_calls, 0); atomic_set(&enable_calls, 0);
	memset(reports, 0, sizeof(reports));
	memset(submit_stage, 0, sizeof(submit_stage));
	memset(actions, 0, sizeof(actions));
	action_count = 0;
	dropped_reports = 0; max_dropped_reports = 0; last_registration_sent = 0;
	block_enqueue = false; complete_during_enqueue = false; enqueue_error = 0;
	disable_error = 0; enable_error = 0; configure_during_enable = false; last_report = NULL;
	hdev = &hid_device; hid_ready = false; hid_configured = false; hid_report_inflight = false;
	configured = false; usb_config_epoch = 0; status_bits = 0;
	usb_hid_initialized = true; usb_hid_vbus_seen = true; usb_hid_vbus_wanted = true;
	usb_hid_disable_pending = false; usb_hid_disable_failed = false;
	usb_hid_enabled = true; usb_hid_enabling = false; usb_hid_enable_failed = false;
	controller_enabled = true; stack_enabled = true;
	iface_ready_cb(&hid_device, true);
	notify(USBD_MSG_CONFIGURATION, 1);
	zassert_true(hid_configured);
}
static void queue_packet(uint8_t value)
{
	uint8_t packet[PACKET_SIZE_BYTES];
	memset(packet, value, sizeof(packet));
	zassert_true(hid_write_packet_n(packet));
}

ZTEST(hid_lifecycle, test_packet_order_across_index_wrap)
{
	const uint32_t starts[] = { 100U, UINT32_MAX - 3U, UINT32_MAX };
	for (size_t n = 0; n < ARRAY_SIZE(starts); n++) {
		fixture_start();
		atomic_set(&hid_prod, (atomic_val_t)starts[n]);
		atomic_set(&hid_cons, (atomic_val_t)starts[n]);
		for (uint8_t i = 1; i <= 5; i++) {
			queue_packet(i);
		}
		send_report();
		for (uint8_t i = 0; i < PACKETS_PER_USB_REPORT; i++) {
			zassert_equal(submit_stage[i * PACKET_SIZE_BYTES], i + 1U);
		}
		zassert_equal(dropped_reports, 1);
		zassert_equal((uint32_t)atomic_get(&hid_cons), starts[n] + 5U);
		zassert_equal((uint32_t)atomic_get(&hid_prod), starts[n] + 5U);
		input_report_done_cb(&hid_device, submit_stage);
	}

	fixture_start();
	for (uint8_t i = 1; i <= 16; i++) {
		queue_packet(i);
	}
	uint8_t packet[PACKET_SIZE_BYTES] = {17};
	zassert_false(hid_write_packet_n(packet));
	send_report();
	for (uint8_t i = 0; i < PACKETS_PER_USB_REPORT; i++) {
		zassert_equal(submit_stage[i * PACKET_SIZE_BYTES], i + 1U);
	}
	zassert_equal(dropped_reports, 12);
	input_report_done_cb(&hid_device, submit_stage);
	zassert_true(hid_write_packet_n(packet));
}

ZTEST(hid_lifecycle, test_unplug_during_enqueue_reconnect_and_delayed_completion)
{
	fixture_start();
	queue_packet(0x11);
	block_enqueue = true;
	k_thread_create(&sender_thread, sender_stack, K_THREAD_STACK_SIZEOF(sender_stack),
			send_entry, NULL, NULL, NULL, HID_THREAD_PRIORITY, 0, K_NO_WAIT);
	zassert_ok(k_sem_take(&enqueue_entered, K_SECONDS(1)));
	zassert_equal_ptr(controller_data.mutex.owner, &sender_thread);
	queue_packet(0x22); /* This old-session packet must be discarded. */

	k_work_submit(&disconnect_work);
	zassert_ok(k_sem_take(&callback_done, K_SECONDS(1)));
	zassert_false(hid_configured);
	zassert_false(configured);
	zassert_true(usb_hid_disable_pending);
	uint8_t rejected[PACKET_SIZE_BYTES] = {0};
	zassert_false(hid_write_packet_n(rejected));

	k_thread_create(&control_thread, control_stack, K_THREAD_STACK_SIZEOF(control_stack),
			control_entry, NULL, NULL, NULL, CONSOLE_THREAD_PRIORITY, 0, K_NO_WAIT);
	zassert_ok(k_sem_take(&control_started, K_SECONDS(1)));
	k_sleep(K_MSEC(10));
	zassert_equal(atomic_get(&disable_calls), 0, "teardown preceded submit API exit");
	/* The control owner may wait, but the shared workqueue must still run. */
	k_work_submit(&release_work);
	zassert_ok(k_sem_take(&release_work_done, K_SECONDS(1)));
	zassert_ok(k_sem_take(&sender_done, K_SECONDS(1)));
	zassert_ok(k_sem_take(&control_done, K_SECONDS(1)));
	zassert_ok(k_thread_join(&sender_thread, K_SECONDS(1)));
	zassert_ok(k_thread_join(&control_thread, K_SECONDS(1)));
	zassert_is_null(controller_data.mutex.owner);
	zassert_equal(action_count, 2);
	zassert_equal(actions[0], 'D');
	zassert_equal(actions[1], 'E');
	zassert_true(hid_report_inflight, "disable is not report completion");

	uint8_t old_stage[USB_REPORT_SIZE_BYTES];
	memcpy(old_stage, submit_stage, sizeof(old_stage));
	iface_ready_cb(&hid_device, true);
	notify(USBD_MSG_CONFIGURATION, 1);
	queue_packet(0x33);
	block_enqueue = false;
	send_report();
	zassert_equal(atomic_get(&enqueue_calls), 1);
	zassert_mem_equal(submit_stage, old_stage, sizeof(old_stage));
	input_report_done_cb(&hid_device, rejected);
	zassert_true(hid_report_inflight, "unexpected callback returned another buffer");
	input_report_done_cb(&hid_device, submit_stage);
	send_report();
	zassert_equal(atomic_get(&enqueue_calls), 2);
	zassert_equal(submit_stage[0], 0x33, "old-session packet survived reconnect");
	input_report_done_cb(&hid_device, submit_stage);
}

static void check_config_loss(int type)
{
	fixture_start();
	queue_packet(0x41);
	send_report();
	queue_packet(0x42);
	uint8_t old_stage[USB_REPORT_SIZE_BYTES];
	memcpy(old_stage, submit_stage, sizeof(old_stage));
	/* The SDK class disable callback precedes its configuration message. */
	iface_ready_cb(&hid_device, false);
	notify(type, 0);
	zassert_false(hid_configured);
	zassert_true(hid_report_inflight);
	zassert_equal(atomic_get(&hid_cons), atomic_get(&hid_prod));
	iface_ready_cb(&hid_device, true);
	notify(USBD_MSG_CONFIGURATION, 1);
	queue_packet(0x43);
	send_report();
	zassert_equal(atomic_get(&enqueue_calls), 1);
	zassert_mem_equal(submit_stage, old_stage, sizeof(old_stage));
	input_report_done_cb(&hid_device, submit_stage);
	send_report();
	zassert_equal(atomic_get(&enqueue_calls), 2);
	zassert_equal(submit_stage[0], 0x43);
	input_report_done_cb(&hid_device, submit_stage);
}
ZTEST(hid_lifecycle, test_config_zero_retains_buffer_until_completion)
{
	check_config_loss(USBD_MSG_CONFIGURATION);
}
ZTEST(hid_lifecycle, test_reset_retains_buffer_until_completion)
{
	check_config_loss(USBD_MSG_RESET);
}

ZTEST(hid_lifecycle, test_submission_failure_counts_only_real_packets)
{
	const uint8_t counts[] = { 0, 1, 2, 3, 4, 5, 16 };
	const int errors[] = { -ENOMEM, -EACCES, -EIO };
	for (size_t e = 0; e < ARRAY_SIZE(errors); e++) {
		for (size_t n = 0; n < ARRAY_SIZE(counts); n++) {
			fixture_start();
			enqueue_error = errors[e];
			last_registration_sent = k_uptime_get() - REGISTRATION_INTERVAL;
			for (uint8_t i = 0; i < counts[n]; i++) {
				queue_packet(i + 1U);
			}
			send_report();
			zassert_equal(atomic_get(&enqueue_calls), 1);
			zassert_equal(dropped_reports, counts[n]);
			zassert_equal(max_dropped_reports, counts[n]);
			zassert_equal(atomic_get(&hid_cons), atomic_get(&hid_prod));
			zassert_false(hid_report_inflight);

			last_registration_sent = k_uptime_get();
			send_report();
			zassert_equal(atomic_get(&enqueue_calls), 1);
			enqueue_error = 0;
			queue_packet(0x71);
			send_report();
			zassert_equal(atomic_get(&enqueue_calls), 2);
			zassert_equal(submit_stage[0], 0x71);
			zassert_equal(dropped_reports, counts[n]);
			input_report_done_cb(&hid_device, submit_stage);
		}
	}
}

ZTEST(hid_lifecycle, test_disconnect_during_failed_submit_counts_staged_packets)
{
	fixture_start();
	enqueue_error = -EACCES;
	block_enqueue = true;
	queue_packet(0x61);
	queue_packet(0x62);
	k_thread_create(&sender_thread, sender_stack, K_THREAD_STACK_SIZEOF(sender_stack),
			send_entry, NULL, NULL, NULL, HID_THREAD_PRIORITY, 0, K_NO_WAIT);
	zassert_ok(k_sem_take(&enqueue_entered, K_SECONDS(1)));
	uint8_t stage[USB_REPORT_SIZE_BYTES];
	memcpy(stage, submit_stage, sizeof(stage));
	notify(USBD_MSG_VBUS_REMOVED, 0);
	zassert_false(hid_configured);
	zassert_true(hid_report_inflight);
	zassert_mem_equal(submit_stage, stage, sizeof(stage));
	k_sem_give(&release_enqueue);
	zassert_ok(k_sem_take(&sender_done, K_SECONDS(1)));
	zassert_ok(k_thread_join(&sender_thread, K_SECONDS(1)));
	zassert_false(hid_report_inflight);
	zassert_mem_equal(submit_stage, stage, sizeof(stage));
	zassert_equal(dropped_reports, 2);
	zassert_equal(max_dropped_reports, 2);
	zassert_equal(atomic_get(&hid_cons), atomic_get(&hid_prod));
}
ZTEST(hid_lifecycle, test_submission_error_and_early_completion)
{
	fixture_start();
	enqueue_error = -ENOMEM;
	queue_packet(0x51);
	send_report();
	zassert_false(hid_report_inflight);
	zassert_equal(dropped_reports, 1);
	enqueue_error = 0;
	complete_during_enqueue = true;
	queue_packet(0x52);
	send_report();
	zassert_false(hid_report_inflight);
	zassert_equal(dropped_reports, 1);
	complete_during_enqueue = false;
	queue_packet(0x53);
	send_report();
	zassert_true(hid_report_inflight);
	zassert_equal(atomic_get(&enqueue_calls), 3);
	zassert_equal(submit_stage[0], 0x53);
	input_report_done_cb(&hid_device, submit_stage);
}
ZTEST(hid_lifecycle, test_failed_disable_remains_fail_closed)
{
	fixture_start();
	notify(USBD_MSG_VBUS_REMOVED, 0);
	notify(USBD_MSG_VBUS_READY, 0);
	disable_error = -EIO;
	usb_hid_service_device();
	zassert_true(usb_hid_disable_pending);
	zassert_equal(atomic_get(&enable_calls), 0);
	notify(USBD_MSG_CONFIGURATION, 1);
	zassert_false(hid_configured);
	/* The next call returns -EALREADY even after the injected error is
	 * cleared; software-disabled does not prove hardware stopped. */
	zassert_false(stack_enabled);
	zassert_false(controller_enabled);
	disable_error = 0;
	usb_hid_service_device();
	zassert_true(usb_hid_disable_pending);
	zassert_equal(atomic_get(&enable_calls), 0);
	notify(USBD_MSG_VBUS_READY, 0);
	usb_hid_service_device();
	zassert_true(usb_hid_disable_pending);
	zassert_equal(atomic_get(&disable_calls), 3);
	zassert_equal(atomic_get(&enable_calls), 0);
	zassert_equal(actions[0], 'D');
	zassert_equal(actions[1], 'D');
	zassert_equal(actions[2], 'D');
}
ZTEST(hid_lifecycle, test_failed_enable_does_not_accept_configuration)
{
	fixture_start();
	notify(USBD_MSG_VBUS_REMOVED, 0);
	notify(USBD_MSG_VBUS_READY, 0);
	enable_error = -EIO;
	configure_during_enable = true;
	usb_hid_service_device();
	zassert_false(usb_hid_disable_pending);
	zassert_false(usb_hid_enabled);
	zassert_true(usb_hid_enable_failed);
	zassert_false(configured);
	zassert_false(hid_configured);
	usb_hid_service_device();
	zassert_equal(atomic_get(&enable_calls), 1, "failed enable retried without a VBUS event");
	notify(USBD_MSG_CONFIGURATION, 1);
	zassert_false(hid_configured);
	enable_error = 0;
	configure_during_enable = false;
	notify(USBD_MSG_VBUS_READY, 0);
	usb_hid_service_device();
	zassert_true(usb_hid_enabled);
	zassert_equal(atomic_get(&disable_calls), 1);
	zassert_equal(atomic_get(&enable_calls), 2);
}
ZTEST(hid_lifecycle, test_initial_vbus_event_wins_over_later_sample)
{
	fixture_start();
	usb_configure(false);
	usb_hid_initialized = false; usb_hid_vbus_seen = false; usb_hid_vbus_wanted = false;
	usb_hid_enabled = false; controller_enabled = false; stack_enabled = false;
	atomic_set(&enable_calls, 0); atomic_set(&disable_calls, 0); action_count = 0;
	setup_error = 0; setup_event = USBD_MSG_VBUS_READY;
	can_detect_vbus = true; physical_vbus = false;
	usb_init_thread();
	zassert_true(usb_hid_initialized);
	zassert_true(usb_hid_vbus_wanted);
	usb_hid_service_device();
	zassert_equal(atomic_get(&enable_calls), 1);

	fixture_start();
	usb_configure(false);
	usb_hid_initialized = false; usb_hid_vbus_seen = false; usb_hid_vbus_wanted = false;
	usb_hid_enabled = false; controller_enabled = false; stack_enabled = false;
	atomic_set(&enable_calls, 0); atomic_set(&disable_calls, 0); action_count = 0;
	setup_event = USBD_MSG_VBUS_REMOVED; physical_vbus = true;
	usb_init_thread();
	zassert_false(usb_hid_vbus_wanted);
	usb_hid_service_device();
	zassert_equal(atomic_get(&disable_calls), 1);
	zassert_equal(atomic_get(&enable_calls), 0);
	zassert_false(usb_hid_disable_pending);

	fixture_start();
	usb_configure(false);
	usb_hid_initialized = false; usb_hid_enabled = false; controller_enabled = false; stack_enabled = false;
	atomic_set(&enable_calls, 0); action_count = 0;
	setup_error = -ENODEV; setup_event = -1;
	usb_init_thread();
	usb_hid_service_device();
	zassert_false(usb_hid_initialized);
	zassert_equal(atomic_get(&enable_calls), 0);
}
ZTEST_SUITE(hid_lifecycle, NULL, NULL, NULL, NULL, NULL);
