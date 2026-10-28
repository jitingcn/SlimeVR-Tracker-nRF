#include "globals.h"
#include "console.h"
#include "hid.h"
#include "system/power.h"
#include "system/status.h"
#include "system/system.h"

#define USB_EXISTS 0
#if CONFIG_USB_DEVICE_STACK_NEXT
#undef USB_EXISTS
#define USB DT_NODELABEL(zephyr_udc0)
#define USB_EXISTS (DT_NODE_HAS_STATUS(USB, okay) && CONFIG_UART_CONSOLE)
#endif

#if USB_EXISTS
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/pm/device.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/uart.h>

#define USB_CONSOLE_IS_CDC DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart)

static int64_t usb_start_time_ms;
static bool configured;
static uint32_t usb_config_epoch;

static K_MUTEX_DEFINE(usb_serial_transition_lock);
static K_SEM_DEFINE(usb_ctrl_event, 0, 1);

#if CONFIG_CONNECTION_OVER_HID
/* The control thread alone enables/disables the controller. Status callbacks
 * publish VBUS changes under the same lock as configuration changes. */
static bool usb_hid_initialized;
static bool usb_hid_vbus_seen;
static bool usb_hid_vbus_wanted;
static bool usb_hid_disable_pending;
static bool usb_hid_disable_failed;
static bool usb_hid_enabled;
static bool usb_hid_enabling;
static bool usb_hid_enable_failed;
#endif

LOG_MODULE_REGISTER(usb, LOG_LEVEL_INF);

static void usb_init_thread(void);
struct usb_ctrl_service_state;
static void usb_ctrl_thread(void *p1, void *p2, void *p3);
static void usb_ctrl_service_step(
	const struct device *dev_console,
	const struct log_backend *backend,
	struct usb_ctrl_service_state *state
);
K_THREAD_DEFINE(usb_ctrl_thread_id, CONFIG_USB_CTRL_THREAD_STACK_SIZE, usb_ctrl_thread, NULL, NULL, NULL, CONSOLE_THREAD_PRIORITY, 0, 0);
K_THREAD_DEFINE(
	usb_init_thread_id,
	CONFIG_USB_INIT_THREAD_STACK_SIZE,
	usb_init_thread,
	NULL,
	NULL,
	NULL,
	USB_INIT_THREAD_PRIORITY,
	0,
	500
); // Wait before enabling USB

#define SLIMEVR_USB_STRING_MANUFACTURER_IDX 1U
#define SLIMEVR_USB_STRING_PRODUCT_IDX 2U
#define SLIMEVR_USB_STRING_SERIAL_NUMBER_IDX COND_CODE_1(CONFIG_HWINFO, (3U), (0U))

#define SLIMEVR_USBD_DEVICE_DEFINE(device_name, udc_dev, vid, pid)	\
	static struct usb_device_descriptor				\
	fs_desc_##device_name = {					\
		.bLength = sizeof(struct usb_device_descriptor),	\
		.bDescriptorType = USB_DESC_DEVICE,			\
		.bcdUSB = sys_cpu_to_le16(USB_SRN_2_0),			\
		.bDeviceClass = USB_BCC_MISCELLANEOUS,			\
		.bDeviceSubClass = 2,					\
		.bDeviceProtocol = 1,					\
		.bMaxPacketSize0 = USB_CONTROL_EP_MPS,			\
		.idVendor = vid,					\
		.idProduct = pid,					\
		.bcdDevice = sys_cpu_to_le16(USB_BCD_DRN),		\
		.iManufacturer = SLIMEVR_USB_STRING_MANUFACTURER_IDX,	\
		.iProduct = SLIMEVR_USB_STRING_PRODUCT_IDX,		\
		.iSerialNumber = SLIMEVR_USB_STRING_SERIAL_NUMBER_IDX,	\
		.bNumConfigurations = 0,				\
	};								\
	IF_ENABLED(USBD_SUPPORTS_HIGH_SPEED, (				\
	static struct usb_device_descriptor				\
	hs_desc_##device_name = {					\
		.bLength = sizeof(struct usb_device_descriptor),	\
		.bDescriptorType = USB_DESC_DEVICE,			\
		.bcdUSB = sys_cpu_to_le16(USB_SRN_2_0),			\
		.bDeviceClass = USB_BCC_MISCELLANEOUS,			\
		.bDeviceSubClass = 2,					\
		.bDeviceProtocol = 1,					\
		.bMaxPacketSize0 = 64,					\
		.idVendor = vid,					\
		.idProduct = pid,					\
		.bcdDevice = sys_cpu_to_le16(USB_BCD_DRN),		\
		.iManufacturer = SLIMEVR_USB_STRING_MANUFACTURER_IDX,	\
		.iProduct = SLIMEVR_USB_STRING_PRODUCT_IDX,		\
		.iSerialNumber = SLIMEVR_USB_STRING_SERIAL_NUMBER_IDX,	\
		.bNumConfigurations = 0,				\
	};								\
	))								\
	static STRUCT_SECTION_ITERABLE(usbd_context, device_name) = {	\
		.name = STRINGIFY(device_name),				\
		.dev = udc_dev,						\
		.fs_desc = &fs_desc_##device_name,			\
		IF_ENABLED(USBD_SUPPORTS_HIGH_SPEED, (			\
		.hs_desc = &hs_desc_##device_name,			\
		))							\
	}

SLIMEVR_USBD_DEVICE_DEFINE(slimevr_usbd, DEVICE_DT_GET(USB),
			   CONFIG_SLIMEVR_USB_DEVICE_VID, CONFIG_SLIMEVR_USB_DEVICE_PID);

USBD_DESC_LANG_DEFINE(slimevr_lang);
USBD_DESC_MANUFACTURER_DEFINE(slimevr_mfr, CONFIG_SLIMEVR_USB_DEVICE_MANUFACTURER);
USBD_DESC_PRODUCT_DEFINE(slimevr_product, CONFIG_SLIMEVR_USB_DEVICE_PRODUCT);
IF_ENABLED(CONFIG_HWINFO, (USBD_DESC_SERIAL_NUMBER_DEFINE(slimevr_sn)));

USBD_DESC_CONFIG_DEFINE(slimevr_fs_cfg_desc, "FS Configuration");
USBD_DESC_CONFIG_DEFINE(slimevr_hs_cfg_desc, "HS Configuration");
USBD_CONFIGURATION_DEFINE(slimevr_fs_config, 0, CONFIG_SLIMEVR_USB_DEVICE_MAX_POWER,
			  &slimevr_fs_cfg_desc);
USBD_CONFIGURATION_DEFINE(slimevr_hs_config, 0, CONFIG_SLIMEVR_USB_DEVICE_MAX_POWER,
			  &slimevr_hs_cfg_desc);

static int usbd_set_code_triple(enum usbd_speed speed)
{
	if (!IS_ENABLED(CONFIG_USBD_CDC_ACM_CLASS)) {
		return 0;
	}

	return usbd_device_set_code_triple(&slimevr_usbd, speed,
					   USB_BCC_MISCELLANEOUS, 0x02, 0x01);
}

static int usbd_add_string_descriptors(void)
{
	int ret;

	ret = usbd_add_descriptor(&slimevr_usbd, &slimevr_lang);
	if (ret != 0) {
		return ret;
	}

	ret = usbd_add_descriptor(&slimevr_usbd, &slimevr_mfr);
	if (ret != 0) {
		return ret;
	}

	ret = usbd_add_descriptor(&slimevr_usbd, &slimevr_product);
	if (ret != 0) {
		return ret;
	}

	IF_ENABLED(CONFIG_HWINFO, (ret = usbd_add_descriptor(&slimevr_usbd, &slimevr_sn);))

	return ret;
}

static int usbd_setup(usbd_msg_cb_t msg_cb)
{
	int ret;

	ret = usbd_add_string_descriptors();
	if (ret != 0) {
		LOG_ERR("Failed to add USB string descriptors: %d", ret);
		return ret;
	}

	if (USBD_SUPPORTS_HIGH_SPEED &&
	    usbd_caps_speed(&slimevr_usbd) == USBD_SPEED_HS) {
		ret = usbd_add_configuration(&slimevr_usbd, USBD_SPEED_HS,
					     &slimevr_hs_config);
		if (ret != 0) {
			LOG_ERR("Failed to add USB HS configuration: %d", ret);
			return ret;
		}

		ret = usbd_register_all_classes(&slimevr_usbd, USBD_SPEED_HS, 1, NULL);
		if (ret != 0) {
			LOG_ERR("Failed to register USB HS classes: %d", ret);
			return ret;
		}

		ret = usbd_set_code_triple(USBD_SPEED_HS);
		if (ret != 0) {
			LOG_ERR("Failed to set USB HS code triple: %d", ret);
			return ret;
		}
	}

	ret = usbd_add_configuration(&slimevr_usbd, USBD_SPEED_FS, &slimevr_fs_config);
	if (ret != 0) {
		LOG_ERR("Failed to add USB FS configuration: %d", ret);
		return ret;
	}

	ret = usbd_register_all_classes(&slimevr_usbd, USBD_SPEED_FS, 1, NULL);
	if (ret != 0) {
		LOG_ERR("Failed to register USB classes: %d", ret);
		return ret;
	}

	ret = usbd_set_code_triple(USBD_SPEED_FS);
	if (ret != 0) {
		LOG_ERR("Failed to set USB FS code triple: %d", ret);
		return ret;
	}

	ret = usbd_msg_register_cb(&slimevr_usbd, msg_cb);
	if (ret != 0) {
		LOG_ERR("Failed to register USB message callback: %d", ret);
		return ret;
	}

	ret = usbd_init(&slimevr_usbd);
	if (ret != 0) {
		LOG_ERR("Failed to initialize USB device: %d", ret);
	}

	return ret;
}

static void usb_serial_stop_locked(bool invalidate)
{
#if USB_CONSOLE_IS_CDC
	const struct log_backend *const backend = log_backend_get_by_name("log_backend_uart");

	if (invalidate) {
		console_serial_stop();
	} else {
		console_serial_close();
	}
	log_backend_disable(backend);
	if (get_status(SYS_STATUS_SERIAL_ACTIVE)) {
		set_status(SYS_STATUS_SERIAL_ACTIVE, false);
	}
#else
	(void)invalidate;
#endif
}

static void usb_configure_locked(bool new_configured)
{
#if USB_CONSOLE_IS_CDC
	const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
#endif
	bool was_configured;

#if CONFIG_CONNECTION_OVER_HID
	/* A queued old CONFIGURATION message cannot restart HID while an unplug
	 * is waiting for the control thread to disable the controller. */
	if (new_configured && (!usb_hid_vbus_wanted || usb_hid_disable_pending ||
			       (!usb_hid_enabled && !usb_hid_enabling))) {
		new_configured = false;
	}
#endif
	was_configured = configured;
	if (was_configured != new_configured) {
		usb_config_epoch++;
	}
	configured = new_configured;
	if (new_configured) {
		if (!was_configured) {
			set_status(SYS_STATUS_USB_CONNECTED, true);
#if USB_CONSOLE_IS_CDC
			pm_device_action_run(cons, PM_DEVICE_ACTION_RESUME);
#endif
#if CONFIG_CONNECTION_OVER_HID
			hid_set_configured(true);
#endif
		}
	} else {
#if CONFIG_CONNECTION_OVER_HID
		/* Stop admission even for a duplicate reset/unconfigure message. */
		hid_set_configured(false);
#endif
		if (was_configured) {
			set_status(SYS_STATUS_USB_CONNECTED, false);
		}
		/* Close serial even when an earlier notification already cleared
		 * the configuration state. */
		usb_serial_stop_locked(true);
		if (was_configured) {
#if USB_CONSOLE_IS_CDC
			pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);
#endif
		}
	}
}

static void usb_configure(bool new_configured)
{
	k_mutex_lock(&usb_serial_transition_lock, K_FOREVER);
	usb_configure_locked(new_configured);
	k_mutex_unlock(&usb_serial_transition_lock);
	k_sem_give(&usb_ctrl_event);
}

static bool usb_config_snapshot(uint32_t *epoch)
{
	bool is_configured;

	k_mutex_lock(&usb_serial_transition_lock, K_FOREVER);
	is_configured = configured;
	*epoch = usb_config_epoch;
	k_mutex_unlock(&usb_serial_transition_lock);
	return is_configured;
}

static int usb_enable_device(struct usbd_context *ctx)
{
	int ret = usbd_enable(ctx);

	if (ret == -EALREADY) {
		ret = 0;
	}

	return ret;
}

#if CONFIG_CONNECTION_OVER_HID
static void usb_hid_vbus_changed(bool present)
{
	k_mutex_lock(&usb_serial_transition_lock, K_FOREVER);
	usb_hid_vbus_seen = true;
	usb_hid_vbus_wanted = present;
	if (present) {
		usb_hid_enable_failed = false;
	} else {
		/* Publish the stop before the control thread sees pending teardown.
		 * A later READY must not cancel this required disable. */
		hid_set_configured(false);
		usb_hid_disable_pending = true;
	}
	k_mutex_unlock(&usb_serial_transition_lock);
	k_sem_give(&usb_ctrl_event);
}

static void usb_hid_service_device(void)
{
	while (1) {
		bool disable;
		int ret;

		k_mutex_lock(&usb_serial_transition_lock, K_FOREVER);
		if (!usb_hid_initialized) {
			k_mutex_unlock(&usb_serial_transition_lock);
			return;
		}
		disable = usb_hid_disable_pending;
		if (!disable && (!usb_hid_vbus_wanted || usb_hid_enabled || usb_hid_enable_failed)) {
			k_mutex_unlock(&usb_serial_transition_lock);
			return;
		}
		if (!disable) {
			usb_hid_enabling = true;
		}
		k_mutex_unlock(&usb_serial_transition_lock);

		if (disable) {
			/* Only the USB API call must leave; completion may depend on
			 * the stack and is deliberately not awaited here. */
			hid_wait_for_idle();
			ret = usbd_disable(&slimevr_usbd);
			if (ret == -EALREADY && !usb_hid_disable_failed) {
				ret = 0;
			}
		} else {
			ret = usb_enable_device(&slimevr_usbd);
		}

		k_mutex_lock(&usb_serial_transition_lock, K_FOREVER);
		if (disable) {
			if (ret == 0) {
				usb_hid_enabled = false;
				usb_hid_disable_pending = false;
				usb_hid_disable_failed = false;
			} else {
				/* usbd_disable can mark the stack disabled even when
				 * udc_disable failed; later -EALREADY is not proof
				 * that the controller was actually stopped. */
				usb_hid_disable_failed = true;
			}
		} else {
			usb_hid_enabling = false;
			if (ret == 0) {
				usb_hid_enabled = true;
			} else {
				usb_hid_enable_failed = true;
				/* Configuration may have been reported while enable
				 * was in progress. Retire it without posting another
				 * control event and spinning on a failed enable. */
				usb_configure_locked(false);
			}
		}
		k_mutex_unlock(&usb_serial_transition_lock);

		if (ret != 0) {
			LOG_ERR("Failed to %s USB device: %d", disable ? "disable" : "enable", ret);
			/* A failed disable keeps admission stopped and never falls
			 * through to enable; a subsequent event can retry it. */
			return;
		}
	}
}
#endif

static void status_cb(struct usbd_context *const ctx, const struct usbd_msg *const msg)
{
	const struct device *const dev_console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
#if CONFIG_CONNECTION_OVER_HID
	ARG_UNUSED(ctx);
#else
	int ret;
#endif

	switch (msg->type) {
	case USBD_MSG_RESET:
		usb_configure(false);
		break;
	case USBD_MSG_CONFIGURATION:
		usb_configure(msg->status != 0);
		break;
	case USBD_MSG_VBUS_READY:
#if CONFIG_CONNECTION_OVER_HID
		usb_hid_vbus_changed(true);
#else
		ret = usb_enable_device(ctx);
		if (ret != 0) {
			LOG_ERR("Failed to enable USB device: %d", ret);
		}
#endif
		break;
	case USBD_MSG_VBUS_REMOVED:
#if CONFIG_CONNECTION_OVER_HID
		usb_hid_vbus_changed(false);
		usb_configure(false);
#else
		usb_configure(false);
		if (usbd_disable(ctx) != 0) {
			LOG_ERR("Failed to disable USB device");
		}
#endif
		break;
	case USBD_MSG_CDC_ACM_CONTROL_LINE_STATE:
	case USBD_MSG_CDC_ACM_LINE_CODING:
		if (msg->dev == dev_console) {
			k_sem_give(&usb_ctrl_event);
		}
		break;
	default:
		LOG_DBG("USBD message %s unhandled", usbd_msg_type_string(msg->type));
		break;
	}
}

#define DFU_EXISTS (CONFIG_BUILD_OUTPUT_UF2 || CONFIG_BOARD_HAS_NRF5_BOOTLOADER || CONFIG_BOOTLOADER_MCUBOOT)
#define ADAFRUIT_BOOTLOADER (CONFIG_BUILD_OUTPUT_UF2 && !CONFIG_BOOTLOADER_MCUBOOT)

static void usb_init_thread(void)
{
	int ret;
	usb_start_time_ms = k_uptime_get();

	ret = usbd_setup(status_cb);
	if (ret != 0) {
		return;
	}

#if CONFIG_CONNECTION_OVER_HID
	bool vbus_present = !usbd_can_detect_vbus(&slimevr_usbd) || vbus_read();
	k_mutex_lock(&usb_serial_transition_lock, K_FOREVER);
	/* Do not overwrite a newer event observed while initialization ran. */
	if (!usb_hid_vbus_seen) {
		usb_hid_vbus_wanted = vbus_present;
	}
	usb_hid_initialized = true;
	k_mutex_unlock(&usb_serial_transition_lock);
	k_sem_give(&usb_ctrl_event);
#else
	if (!usbd_can_detect_vbus(&slimevr_usbd) || vbus_read()) {
		ret = usb_enable_device(&slimevr_usbd);
		if (ret != 0) {
			LOG_ERR("Failed to enable USB device: %d", ret);
		}
	}
#endif
}

struct usb_ctrl_service_state {
	uint32_t epoch;
	uint32_t last_dtr;
	bool dtr_known;
#if DFU_EXISTS
	bool button_checked;
	bool button_raw_recheck_pending;
#endif
};

static void usb_ctrl_service_step(
	const struct device *dev_console,
	const struct log_backend *backend,
	struct usb_ctrl_service_state *state
)
{
	uint32_t dtr;
	bool is_configured;
	bool dtr_high;
	bool serial_active;
#if ADAFRUIT_BOOTLOADER
	uint32_t baudrate = 0;
	bool dtr_falling;
#endif
	bool request_reboot = false;

	k_mutex_lock(&usb_serial_transition_lock, K_FOREVER);
	is_configured = configured;
	if (!is_configured) {
		state->dtr_known = false;
		state->last_dtr = 0;
#if DFU_EXISTS
		state->button_checked = false;
		state->button_raw_recheck_pending = false;
#endif
		k_mutex_unlock(&usb_serial_transition_lock);
		return;
	}

	if (state->epoch != usb_config_epoch) {
		state->epoch = usb_config_epoch;
		state->dtr_known = false;
		state->last_dtr = 0;
#if DFU_EXISTS
		state->button_checked = false;
		state->button_raw_recheck_pending = false;
#endif
	}

#if DFU_EXISTS
	int64_t since_start_ms = k_uptime_get() - usb_start_time_ms;
	if (!state->button_checked) {
		bool enter_dfu = since_start_ms < 100 ? button_read_filtered() : button_read();
		state->button_checked = true;
		state->button_raw_recheck_pending = since_start_ms < 100;
		if (enter_dfu) {
			state->button_raw_recheck_pending = false;
			k_mutex_unlock(&usb_serial_transition_lock);
			sys_enter_dfu(false);
			return;
		}
	} else if (state->button_raw_recheck_pending && since_start_ms >= 100) {
		state->button_raw_recheck_pending = false;
		if (button_read()) {
			k_mutex_unlock(&usb_serial_transition_lock);
			sys_enter_dfu(false);
			return;
		}
	}
#endif

	if (!USB_CONSOLE_IS_CDC || uart_line_ctrl_get(dev_console, UART_LINE_CTRL_DTR, &dtr) != 0) {
		k_mutex_unlock(&usb_serial_transition_lock);
		return;
	}
	dtr_high = dtr != 0U;
	serial_active = get_status(SYS_STATUS_SERIAL_ACTIVE);
	if (state->dtr_known && dtr == state->last_dtr && (dtr_high == serial_active)) {
		k_mutex_unlock(&usb_serial_transition_lock);
		return;
	}
#if ADAFRUIT_BOOTLOADER
	dtr_falling = state->dtr_known && state->last_dtr != 0U && !dtr_high;
#endif
	state->dtr_known = true;
	state->last_dtr = dtr;

	if (dtr_high) {
		if (console_serial_start() == 0) {
			set_status(SYS_STATUS_SERIAL_ACTIVE, true);
			log_backend_enable(backend, backend->cb->ctx, CONFIG_LOG_MAX_LEVEL);
		}
	} else {
#if ADAFRUIT_BOOTLOADER
		if (dtr_falling && uart_line_ctrl_get(dev_console, UART_LINE_CTRL_BAUD_RATE, &baudrate) == 0
			&& baudrate == 1200) {
			NRF_POWER->GPREGRET = ADAFRUIT_DFU_MAGIC_SERIAL_ONLY_RESET;
			request_reboot = true;
		}
#endif
		/* Ordinary close preserves accepted lines; 1200-touch retires them
		 * before requesting the deliberate reboot. */
		usb_serial_stop_locked(request_reboot);
	}
	k_mutex_unlock(&usb_serial_transition_lock);

	if (request_reboot) {
		sys_request_system_reboot();
	}
}

static void usb_ctrl_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	const struct log_backend *const backend = log_backend_get_by_name("log_backend_uart");
	const struct device *const dev_console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	struct usb_ctrl_service_state state = {0};

	while (1) {
		uint32_t epoch;
		k_timeout_t timeout = K_FOREVER;
		if (usb_config_snapshot(&epoch)) {
			int64_t deadline_ms = k_uptime_get() + 1000;
#if DFU_EXISTS
			if (state.button_raw_recheck_pending) {
				int64_t button_deadline = usb_start_time_ms + 100;
				if (button_deadline < deadline_ms) {
					deadline_ms = button_deadline;
				}
			}
#endif
			timeout = K_TIMEOUT_ABS_MS(deadline_ms);
		}
		(void)k_sem_take(&usb_ctrl_event, timeout);
#if CONFIG_CONNECTION_OVER_HID
		usb_hid_service_device();
#endif
		usb_ctrl_service_step(dev_console, backend, &state);
	}
}

#endif
