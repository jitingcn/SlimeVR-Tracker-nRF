"""Run the real connection broadcast branch through repeated grantless RX."""
from pathlib import Path
import sys,os,shlex,subprocess,tempfile
root=Path(os.environ.get('SOURCE_ROOT',Path(__file__).resolve().parents[3]))
sys.path.insert(0,str(root/'tests/host/harness/python'))
from c_extract import extract_block
src=(root/'src/connection/connection.c').read_text();src=src[src.index('static void connection_thread(void)\n{'):]
branch=extract_block(src,r'^\s*if \(broadcast\)\s*\{')
prefix=r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#define ESB_PING_LEN 16
struct {unsigned due,attempts,queue_ok,queue_fail;} ping_sched_stats;
static unsigned iterations,serviced,sleeps,workers;
static unsigned grant_at;
static bool received;
static bool esb_relay_control_window(uint32_t t) {(void)t;return received;}
static bool relay_next_maintenance(uint32_t *t) {(void)t;return false;}
static bool relay_next_repair(uint32_t *t,uint8_t *slot) {(void)t;(void)slot;return false;}
static void ping_build(uint8_t *p) {(void)p;}
static void atomic_inc(unsigned *p) {(*p)++;}
static void ping_stats_attempt(uint32_t n) {(void)n;}
static int esb_write_maintenance(uint8_t *p) {(void)p;return 0;}
static int esb_write_relay(void) {return 0;}
static void relay_control_serviced(void) {serviced++;}
static bool esb_layout_granted(void) {return iterations>=grant_at;}
static void k_msleep(unsigned ms) {
 assert(ms==1 && serviced==iterations);sleeps++;workers++;
}
static void run(unsigned count,unsigned grant,bool rx) {
 serviced=sleeps=workers=0;grant_at=grant;received=rx;
 for(iterations=1;iterations<=count;iterations++) {
  bool broadcast=true;uint32_t target=0;int64_t now=0;
'''
test=r'''
 }
 assert(serviced==count);
}
int main(void) {
 /* A joiner can fail indefinitely; every attempt must let other owners run. */
 run(20000,20001,false);assert(workers==20000 && sleeps==20000);
 /* A usable B can still exclude this member: acquisition must also rest. */
 run(10,11,true);assert(sleeps==10);
 /* A failed normal B uses existing TDMA scheduling, without a new delay. */
 run(10,1,false);assert(sleeps==0);
 run(10,1,true);assert(sleeps==0);
 /* The first granting B immediately restores ordinary timing. */
 run(10,3,true);assert(sleeps==2);
 return 0;
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d);(p/'test.c').write_text(prefix+branch+test)
 subprocess.run(shlex.split(os.environ.get('CC','cc'))+['-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',str(p/'test.c'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
print('connection acquisition scheduling: PASS')
