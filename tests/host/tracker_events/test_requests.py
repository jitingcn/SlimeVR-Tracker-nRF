"""Exercise real request owners; event and RTOS operations are injected leaves."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[3]


def function(name):
    source = (ROOT / "src/sensor/calibration/calibration.c").read_text()
    match = re.search(rf"^(?:static )?(?:void|int|uint8_t|uint16_t) {name}\([^;]*?\)\s*\{{", source, re.M)
    if not match:
        raise RuntimeError(name)
    depth = 0
    for token in re.finditer(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]', source[source.index("{", match.start()):], re.S):
        if token.group() == "{":
            depth += 1
        elif token.group() == "}":
            depth -= 1
            if depth == 0:
                return source[match.start():source.index("{", match.start()) + token.end()] + "\n"
    raise RuntimeError(name)


PRELUDE = r'''
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include "sensor/calibration/calibration.h"
#include "connection/tracker_event_protocol.h"
#define CONFIG_SENSOR_SENS_REV 5
#define K_FOREVER 0
#define SYS_STATUS_CALIBRATION_RUNNING 1
#define LOG_ERR(...) ((void)0)
#define LOG_INF(...) ((void)0)
static int calibration_request_lock, requested_calibration;
static uint16_t requested_operation;
static bool mag_cal_led_pending, running;
static uint8_t magneto_progress, sens_cal_axis;
static uint16_t sens_cal_revolutions;
static unsigned accepted, rejected, sample_ends, wakes;
static uint8_t last_kind, last_reason;
static unsigned owner_depth;
static void k_mutex_lock(int *lock, int wait) { (void)lock; (void)wait; owner_depth++; }
static void k_mutex_unlock(int *lock) { (void)lock; assert(owner_depth == 1); owner_depth--; }
static uint16_t cal_event_accept(uint8_t kind) { assert(owner_depth); last_kind=kind; return ++accepted; }
static void cal_event_reject(uint8_t kind,uint8_t reason) { rejected++; last_kind=kind; last_reason=reason; }
static void tracker_events_notify(void) { assert(!owner_depth); }
static void calibration_signal_wake(void) { wakes++; }
static void sensor_calibration_samples_end(void) { sample_ends++; }
static bool get_status(int status) { (void)status; return running; }
static void set_status(int status,bool value) { (void)status; running=value; }
'''

TEST = r'''
int main(void) {
 assert(sensor_calibration_request(CAL_REQUEST_IMU,CAL_REQUEST_USER)==0);
 uint16_t first=sensor_calibration_current_operation(); assert(first && accepted==1);
 for(unsigned i=0;i<1000;i++) {
  assert(sensor_calibration_request(CAL_REQUEST_TCAL_BOOT,CAL_REQUEST_AUTO)==-1);
  assert(sensor_calibration_request(CAL_REQUEST_IMU,CAL_REQUEST_AUTO_SILENT)==-1);
 }
 assert(accepted==1 && rejected==0 && sensor_calibration_current_operation()==first);
 assert(sensor_calibration_request(CAL_REQUEST_IMU,CAL_REQUEST_USER)==-1);
 assert(rejected==1 && last_reason==CAL_REASON_BUSY && sensor_calibration_current_operation()==first);
 assert(sensor_calibration_request(CAL_REQUEST_CLEAR,CAL_REQUEST_USER)==0);
 assert(sensor_calibration_current_operation()==0 && sample_ends==1);
 assert(sensor_calibration_request(CAL_REQUEST_IMU,CAL_REQUEST_AUTO_SILENT)==0);
 assert(sensor_calibration_current_operation()==0 && accepted==1);
 sensor_calibration_request(CAL_REQUEST_CLEAR,CAL_REQUEST_USER);
 assert(sensor_calibration_request(CAL_REQUEST_TCAL_BOOT,CAL_REQUEST_AUTO)==0);
 assert(last_kind==(CAL_KIND_TCAL_BOOT|CAL_EVENT_ORIGIN_AUTO));
 sensor_calibration_request(CAL_REQUEST_CLEAR,CAL_REQUEST_USER);
 sensor_request_calibration_mag(); first=sensor_calibration_current_operation();
 assert(first && requested_calibration==CAL_REQUEST_MAG && mag_cal_led_pending);
 unsigned before=accepted; unsigned refused=rejected;
 sensor_request_calibration_mag();
 assert(accepted==before && rejected==refused+1 && sensor_calibration_current_operation()==first);
 sensor_calibration_request(CAL_REQUEST_CLEAR,CAL_REQUEST_USER);
 sensor_request_calibration_mag();
 assert(sensor_calibration_current_operation()!=first && requested_calibration==CAL_REQUEST_MAG);
 sensor_calibration_request(CAL_REQUEST_CLEAR,CAL_REQUEST_USER);
 assert(sensor_request_calibration_sens(3,5)==-EINVAL);
 assert(last_reason==CAL_REASON_INVALID_ARGUMENT && sensor_calibration_current_operation()==0);
 assert(sensor_request_calibration_sens(2,0)==0);
 assert(sens_cal_axis==2 && sens_cal_revolutions==CONFIG_SENSOR_SENS_REV);
 assert(sensor_calibration_current_operation()!=0);
 return 0;
}
'''


def main():
    names = ("calibration_request_kind", "sensor_calibration_current_operation", "sensor_calibration_request", "sensor_request_calibration_sens", "sensor_request_calibration_mag")
    source = PRELUDE + "\n".join(function(name) for name in names) + TEST
    with tempfile.TemporaryDirectory(prefix="cal-request-") as directory:
        path = Path(directory)
        (path / "test.c").write_text(source)
        subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-DCONFIG_SENSOR_USE_SENS_CALIBRATION=1", "-I", str(ROOT / "src"), str(path / "test.c"), "-o", str(path / "test")], check=True)
        subprocess.run([str(path / "test")], check=True)
    print("calibration request ownership: PASS")


if __name__ == "__main__":
    main()
