"""A quiesce timeout must retain the raw owner and unwind its block token."""
from pathlib import Path
import os, shlex, subprocess, tempfile, sys
h=Path(__file__).resolve().parent
sys.path.insert(0,str(h.parent/'harness/python'))
from c_extract import extract_block
s=(h.parents[2]/'src/connection/esb.c').read_text()
f=extract_block(s,r'^int esb_raw_rx_quiesce\(void\)\s*\{')
c='''#include <assert.h>
#include <stdbool.h>
#include <errno.h>
static bool raw_rx_active; static int raw_rx_blocked, waits, finish;
static void atomic_inc(int *p){++*p;} static void atomic_dec(int *p){--*p;}
static unsigned irq_lock(void){return 0;} static void irq_unlock(unsigned k){(void)k;}
static void k_busy_wait(int n){(void)n;if(++waits==finish)raw_rx_active=false;}
'''+f+'''
int main(void){
 raw_rx_active=true;assert(esb_raw_rx_quiesce()==-ETIMEDOUT);
 assert(raw_rx_active && !raw_rx_blocked && waits==2000);
 waits=0;finish=3;assert(!esb_raw_rx_quiesce());assert(!raw_rx_active && raw_rx_blocked==1);
 raw_rx_blocked=0;waits=0;assert(!esb_raw_rx_quiesce());assert(!waits && raw_rx_blocked==1);
}
'''
with tempfile.TemporaryDirectory() as t:
 p=Path(t)/'q.c';p.write_text(c);exe=Path(t)/'q'
 subprocess.run(shlex.split(os.environ.get('CC','cc'))+['-Wall','-Wextra','-Werror',str(p),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
print('Quiesce: timeout, late completion and idle ownership passed')
