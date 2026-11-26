"""Execute the source-builder's actual layout gate with a poisoned failed output."""
from pathlib import Path
import subprocess,tempfile
h=Path(__file__).resolve().parent
s=(h.parents[2]/'src/connection/connection.c').read_text()
a=s.index('\t\tstruct tdma_layout layout;',s.index('private tests select from'))
b=s.index('\n\t\tbool test_send_due',a)
block=s[a:b]
cap_line=next(line for line in s.splitlines() if 'builder.sub_max = relay_direct_payload_max(' in line)
c='''#include <stdbool.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>
struct tdma_layout {unsigned profile,slot_index,slot_ticks,epoch;};
static bool available;
static bool tdma_layout_get(struct tdma_layout *p){memset(p,0xff,sizeof(*p));if(!available)return false;p->profile=1;p->slot_index=10;return true;}
static unsigned relay_direct_payload_max(unsigned p,unsigned s){assert(p==1 && s==10);return 36;}
static bool test_rate_stream_due(uint32_t t,uint8_t e){(void)t;(void)e;assert(false);return false;}
static bool tdma_wait_network_tick(uint32_t t){(void)t;assert(false);return false;}
static void k_msleep(int ms){(void)ms;assert(false);}
int main(void){struct {unsigned sub_max;} builder={0};unsigned begins=0;
for(int i=0;i<2;i++){available=i;
bool stream=true,in_test_mode=false;uint32_t stream_target=0;
'''+block+cap_line+'''
begins++;assert(builder.sub_max==32);}
assert(begins==1);}
'''
with tempfile.TemporaryDirectory() as t:
 p=Path(t)/'l.c';p.write_text(c);e=Path(t)/'l'
 subprocess.run(['cc','-Wall','-Wextra','-Werror',str(p),'-o',str(e)],check=True);subprocess.run([str(e)],check=True)
print('Source builder: failed layout skips construction; valid layout retains payload cap')
