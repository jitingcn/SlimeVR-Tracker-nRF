#include "globals.h"
#include "console.h"
#include "hid.h"
#include "system/power.h"
#include "system/status.h"

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

static bool connected;
static bool configured;

LOG_MODULE_REGISTER(usb, LOG_LEVEL_INF);

static void usb_init_thread(void);
K_THREAD_DEFINE(usb_init_thread_id, 256, usb_init_thread, NULL, NULL, NULL, 6, 0, 500); // Wait before enabling USB

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
USBD_CONFIGURATION_DEFINE(slimevr_fs_config, 0, CONFIG_SLIMEVR_USB_DEVICE_MAX_POWER,
			  &slimevr_fs_cfg_desc);

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

	if (IS_ENABLED(CONFIG_USBD_CDC_ACM_CLASS)) {
		ret = usbd_device_set_code_triple(&slimevr_usbd, USBD_SPEED_FS,
						  USB_BCC_MISCELLANEOUS, 0x02, 0x01);
		if (ret != 0) {
			LOG_ERR("Failed to set USB code triple: %d", ret);
			return ret;
		}
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

static int usb_enable_device(struct usbd_context *ctx)
{
	int ret = usbd_enable(ctx);

	if (ret == -EALREADY) {
		ret = 0;
	}
	if (ret == 0) {
		configured = false;
	}

	return ret;
}

static void usb_connected(void)
{
	const struct log_backend *backend = log_backend_get_by_name("log_backend_uart");
	const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

	if (connected) {
		return;
	}

	connected = true;
	set_status(SYS_STATUS_USB_CONNECTED, true);
	pm_device_action_run(cons, PM_DEVICE_ACTION_RESUME);
	log_backend_enable(backend, backend->cb->ctx, CONFIG_LOG_MAX_LEVEL);
	console_thread_create();
#if CONFIG_CONNECTION_OVER_HID
	hid_thread_create();
#endif
}

static void usb_disconnected(void)
{
	const struct log_backend *backend = log_backend_get_by_name("log_backend_uart");
	const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

	if (!connected) {
		return;
	}

	connected = false;
	set_status(SYS_STATUS_USB_CONNECTED, false);
#if CONFIG_CONNECTION_OVER_HID
	hid_thread_abort();
#endif
	console_thread_abort();
	log_backend_disable(backend);
	pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);
}

static void status_cb(struct usbd_context *const ctx, const struct usbd_msg *const msg)
{
	int ret;

	switch (msg->type) {
	case USBD_MSG_RESET:
		configured = false;
		usb_disconnected();
		break;
	case USBD_MSG_CONFIGURATION:
		if (msg->status != 0) {
			if (!configured) {
				configured = true;
				usb_connected();
			}
		} else {
			configured = false;
			usb_disconnected();
		}
		break;
	case USBD_MSG_VBUS_READY:
		ret = usb_enable_device(ctx);
		if (ret != 0) {
			LOG_ERR("Failed to enable USB device: %d", ret);
		}
		break;
	case USBD_MSG_VBUS_REMOVED:
		configured = false;
		usb_disconnected();
		if (usbd_disable(ctx) != 0) {
			LOG_ERR("Failed to disable USB device");
		}
		break;
	default:
		LOG_DBG("USBD message %s unhandled", usbd_msg_type_string(msg->type));
		break;
	}
}

static void usb_init_thread(void)
{
	int ret = usbd_setup(status_cb);

	if (ret != 0) {
		return;
	}

	if (!usbd_can_detect_vbus(&slimevr_usbd) || vbus_read()) {
		ret = usb_enable_device(&slimevr_usbd);
		if (ret != 0) {
			LOG_ERR("Failed to enable USB device: %d", ret);
			return;
		}
	}
}

#endif
