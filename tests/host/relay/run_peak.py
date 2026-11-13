"""A newly observed peak after unlock must survive the current report."""
from pathlib import Path
import subprocess,tempfile
h=Path(__file__).resolve().parent
s=(h.parents[2]/'src/connection/relay.c').read_text()
a=s.index('\tunsigned peak_key = irq_lock();');b=s.index('\tconst uint32_t counters',a)
c='''#include <stdint.h>
#include <assert.h>
static struct {uint16_t start_late_max;} stats={7};
static unsigned irq_lock(void){return 0;}
static void irq_unlock(unsigned key){(void)key;assert(!stats.start_late_max);stats.start_late_max=9;}
int main(void){
'''+s[a:b]+'''assert(start_late==7 && stats.start_late_max==9);}
'''
with tempfile.TemporaryDirectory() as t:
 p=Path(t)/'p.c';p.write_text(c);e=Path(t)/'p'
 subprocess.run(['cc','-Wall','-Wextra','-Werror',str(p),'-o',str(e)],check=True);subprocess.run([str(e)],check=True)
print('Peak handoff: snapshot contains old maximum; next maximum survives unlock')
