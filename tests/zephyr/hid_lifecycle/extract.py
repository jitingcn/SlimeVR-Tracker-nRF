from pathlib import Path
import re
import sys

tracker, zephyr, output = map(Path, sys.argv[1:])
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "host/harness/python"))
from c_extract import extract_block


def function(source, name):
    pattern = rf"^(?:static\s+)?(?:inline\s+)?(?:int|void|bool)\s+{re.escape(name)}\s*\([^;{{]*\)\s*\{{"
    return extract_block(source, pattern)


hid = (tracker / "src/hid.c").read_text()
usb = (tracker / "src/usb.c").read_text()
parts = [hid[hid.index("struct tracker_report {"):hid.index("LOG_MODULE_REGISTER(hid_event")],
         hid[hid.index("static uint32_t dropped_reports"):hid.index("static void send_report")]]
parts += [function(hid, name) for name in ("hid_set_configured", "hid_wait_for_idle",
          "packet_device_addr", "send_report", "input_report_done_cb", "iface_ready_cb",
          "hid_write_packet_n")]
(output / "hid_lifecycle.inc").write_text("\n\n".join(parts) + "\n")

parts = [usb[usb.index("static int64_t usb_start_time_ms;"):usb.index("LOG_MODULE_REGISTER(usb")]]
parts += [function(usb, name) for name in ("usb_serial_stop_locked", "usb_configure_locked", "usb_configure",
          "usb_enable_device", "usb_hid_vbus_changed", "usb_hid_service_device", "status_cb",
          "usb_init_thread")]
(output / "usb_lifecycle.inc").write_text("\n\n".join(parts) + "\n")

parts = []
for path, names in (("drivers/usb/udc/udc_common.h", ("udc_lock_internal", "udc_unlock_internal")),
                    ("drivers/usb/udc/udc_nrf.c", ("udc_nrf_lock", "udc_nrf_unlock")),
                    ("drivers/usb/udc/udc_common.c", ("udc_ep_enqueue",))):
    source = (zephyr / path).read_text()
    parts.extend(function(source, name) for name in names)
(output / "udc_enqueue.inc").write_text("\n\n".join(parts) + "\n")
