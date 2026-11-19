"""Execute the source-builder's actual layout gate with a poisoned failed output."""
from pathlib import Path
import subprocess,tempfile
h=Path(__file__).resolve().parent
s=(h.parents[2]/'src/connection/connection.c').read_text()
a=s.index('\t\t\t\tstruct tdma_layout layout;',s.index('bool quat_ready ='))
b=s.index('\n\t\t\t}',a)
block=s[a:b]
c='''#include <stdbool.h>
#include <assert.h>
#include <string.h>
struct tdma_layout {unsigned profile,slot_index;};
static bool available;
static bool tdma_layout_get(struct tdma_layout *p){memset(p,0xff,sizeof(*p));if(!available)return false;p->profile=1;p->slot_index=10;return true;}
static unsigned relay_direct_payload_max(unsigned p,unsigned s){assert(p==1 && s==10);return 36;}
int main(void){struct {unsigned sub_max;} builder={0};unsigned begins=0;
for(int i=0;i<2;i++){available=i;
'''+block+'''
begins++;assert(builder.sub_max==32);}
assert(begins==1);}
'''
with tempfile.TemporaryDirectory() as t:
 p=Path(t)/'l.c';p.write_text(c);e=Path(t)/'l'
 subprocess.run(['cc','-Wall','-Wextra','-Werror',str(p),'-o',str(e)],check=True);subprocess.run([str(e)],check=True)
print('Source builder: failed layout skips construction; valid layout retains payload cap')
