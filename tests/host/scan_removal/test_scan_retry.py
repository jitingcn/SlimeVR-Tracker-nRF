"""Exercise the production failed-discovery loop without a sensor or scheduler."""
from pathlib import Path
import os, subprocess, tempfile
src = (Path(__file__).resolve().parents[3] / 'src/sensor/sensor.c').read_text()
start = src.index('\tfor (unsigned retry = 0; imu_id < 0 && retry < 2; retry++)')
end = src.index('\n\tif (imu_id >= (int)ARRAY_SIZE(dev_imu_names))', start)
boot_start = src.index('\tunsigned delay_ms = 2000;')
boot_end = src.index('\n}', boot_start)
boot = src[boot_start:boot_end]
code = r'''
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <setjmp.h>
typedef int atomic_t;
static atomic_t sensor_progress, sensor_recovery_pending, main_suspended;
static int atomic_get(atomic_t *p) { return *p; }
static int atomic_set(atomic_t *p, int v) { int old=*p; *p=v; return old; }
static jmp_buf done;
static unsigned sleeps, stop_after, healthy, suspend;
static unsigned pending_at, resume_at;
#define LOG_ERR(...) ((void)0)
#define LOG_WRN(...) ((void)0)
static struct { unsigned addr; } sensor_imu_dev;
static unsigned sensor_imu_dev_reg, calls, delay, succeed;
static void k_msleep(unsigned ms) {
 delay += ms;
 if (stop_after && ++sleeps == stop_after) longjmp(done, 1);
 if (resume_at && sleeps == resume_at) {
  assert(main_suspended && sensor_recovery_pending && sensor_progress);
  suspend = 0;
 }
 if (healthy) sensor_progress = 1;
 main_suspended = suspend;
 if (pending_at && sleeps == pending_at) sensor_recovery_pending = 1;
}
static int sensor_scan_imu_once(void) {
 assert(sensor_imu_dev.addr == 0 && sensor_imu_dev_reg == 255);
 return ++calls == succeed ? 7 : -1;
}
static int retry(int imu_id) {
''' + src[start:end] + r'''
return imu_id;
}
static unsigned scan_calls, scan_success;
static int scan_error = -EIO;
static int sensor_request_scan(bool force, bool feedback) {
 assert(force && !feedback);
 assert(!main_suspended);
 if (++scan_calls >= scan_success) { sensor_progress = 1; sensor_recovery_pending = 0; return 0; }
 return scan_error;
}
static void startup(void) {
''' + boot + r'''
}
int main(void) {
 assert(retry(7) == 7 && calls == 0 && delay == 0);
 succeed = 1;
 assert(retry(-1) == 7 && calls == 1 && delay == 250);
 calls = delay = 0; succeed = 2;
 assert(retry(-1) == 7 && calls == 2 && delay == 1250);
 calls = delay = 0; succeed = 3;
 assert(retry(-1) == -1 && calls == 2 && delay == 1250);
 /* Missing sensor continues far beyond the old six-attempt limit. */
 delay = scan_calls = sleeps = 0; scan_success = 1000; stop_after = 12;
 if (!setjmp(done)) startup();
 assert(scan_calls == 12 && delay == 270000);
 /* Late discovery recovers, then healthy data prevents any more scans. */
 delay = scan_calls = sleeps = 0; scan_success = 8; healthy = 1; stop_after = 20;
 if (!setjmp(done)) startup();
 assert(scan_calls == 8);
 /* Pending recovery cannot be starved by continuing decoded data. */
 delay = scan_calls = sleeps = 0; scan_success = 1; healthy = 1; suspend = 0;
 sensor_progress = sensor_recovery_pending = main_suspended = 0;
 pending_at = 3; resume_at = 0; stop_after = 12;
 if (!setjmp(done)) startup();
 assert(scan_calls == 2 && !sensor_recovery_pending);
 /* Recovery waits through suspension, then runs once after resume. */
 delay = scan_calls = sleeps = 0; scan_success = 1; healthy = 1; suspend = 1;
 sensor_progress = sensor_recovery_pending = main_suspended = 0;
 pending_at = 3; resume_at = 7; stop_after = 12;
 if (!setjmp(done)) startup();
 assert(scan_calls == 2 && !sensor_recovery_pending);
 pending_at = resume_at = 0;
 /* Stalled decoded data invokes the same full scan repeatedly. */
 delay = scan_calls = sleeps = 0; scan_success = 1; healthy = 0; stop_after = 12;
 sensor_progress = sensor_recovery_pending = 0;
 if (!setjmp(done)) startup();
 assert(scan_calls == 3);
 /* Intentional sensor suspension must not be interpreted as failure. */
 delay = scan_calls = sleeps = 0; suspend = 1; stop_after = 20;
 if (!setjmp(done)) startup();
 assert(scan_calls == 1);

}
'''
with tempfile.TemporaryDirectory() as d:
 p = Path(d); (p/'test.c').write_text(code)
 subprocess.run([os.environ.get('CC','cc'), '-std=c11','-Wall','-Wextra','-Werror',str(p/'test.c'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
print('Scan recovery: bounded probes, indefinite capped retry, late discovery, pending recovery, stall and suspension passed')
