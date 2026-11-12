#!/usr/bin/env python3
"""Exercise the production B RX window, including IRQ delivery after claim."""
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get('SOURCE_ROOT', HERE.parents[2]))
sys.path.insert(0, str(HERE.parent / 'harness/python'))
from c_extract import extract_block
source = (ROOT / 'src/connection/esb.c').read_text()
window = "#define RELAY_CONTROL_RX_SETUP_TICKS 2U\n" + extract_block(source, r'^bool esb_relay_control_window\([^;{]*\)\s*\{')
prefix = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define CONFIG_RELAY_LAYOUT_FOLLOW_RECEIVER 1
#include "relay_protocol.h"
static struct relay_geometry geometry={11,5,16,120,4096,15,0};
const struct relay_geometry *relay_geometry_active=&geometry;
static unsigned two_case,copy_index;
/* Non-two cases: frames the radio completes, one END per frame, each at a tick. */
static unsigned frames_total=1,taken;
static uint32_t frame_tick[2];
static uint8_t frame_type[2]={0xe8,0xe8};
static struct relay_runtime_anchor relay_direct_anchor={.valid=true,.third_start=1000,.direct=11,.slot_ticks=16};
#define BIT(n) (1U<<(n))
#define MAX(a,b) ((a)>(b)?(a):(b))
#define MIN(a,b) ((a)<(b)?(a):(b))
static unsigned control_window_fail[6], control_open_late, control_open_lag_max;
static unsigned control_crc_fail,control_missed,control_open_hist[16][2];
static unsigned control_no_address,control_address_timeout,radio_channel;
static unsigned depth,claimed,prepared,started,stopped,clock_users,attempts;
static unsigned tick=96,busy_attempts,delay_calls;
static bool clock_fail,init_fail,wait_fail,network_fail,timeout,take_fail,stamp_fail,reject,completed_late,edge_wait_fail,prepare_irq,setup_work;
struct relay_rx_frame { uint8_t data[72];uint32_t end_kernel_ticks;};
struct esb_network_clock {uint32_t receiver_ticks;};
static unsigned irq_lock(void){return depth++;}
static void irq_unlock(unsigned key){
 assert(depth==key+1);depth=key;
 /* Model a pending interrupt stretching the unprotected claim-to-RXEN gap. */
 if(!depth && claimed && !prepared) tick+=20;
 if(!depth && prepared && !started && prepare_irq){tick+=20;prepare_irq=false;}
}
static int esb_relay_clock(bool on){
 if(on){if(clock_fail)return -1;clock_users++;}else{assert(clock_users);clock_users--;}
 return 0;
}
static int radio_capture_init(void){return init_fail ? -1 : 0;}
static bool tdma_wait_network_tick(uint32_t t){assert((t==98 || t==100) && !depth);if(tick<t)tick=t;return !wait_fail && !(edge_wait_fail && t==100);}
static bool tdma_network_now(uint32_t *t){*t=tick;return !network_fail;}
static bool esb_raw_rx_begin(void){
 unsigned key=irq_lock();bool ok=attempts++>=busy_attempts;
 if(ok)claimed=1;irq_unlock(key);return ok;
}
static void k_busy_wait(unsigned t){assert(!depth);tick+=t;delay_calls++;}
static bool radio_capture_rx_end(uint32_t *t){*t=completed_late ? 111 : tick;return !stamp_fail;}
static void radio_capture_set_enabled(bool enabled){assert(enabled && claimed);}
static bool relay_radio_prepare(unsigned channel,unsigned pipes,bool rx){
 (void)channel;assert(pipes==BIT(1) && rx && claimed && depth);prepared++;if(setup_work)tick++;return true;
}
static bool relay_radio_receive(void){assert(prepared && claimed && depth && tick>=100);if(setup_work)assert(tick==100);started++;return true;}
static inline __attribute__((unused)) bool relay_radio_start(unsigned channel,unsigned pipes,bool rx){return relay_radio_prepare(channel,pipes,rx) && relay_radio_receive();}
static bool esb_get_network_clock(struct esb_network_clock *c){if(completed_late && started)tick=125;c->receiver_ticks=tick;return true;}
static bool relay_radio_ended(void){if(two_case){unsigned at=copy_index==0?111:111+(geometry.second_b==1?16:(geometry.repairs+1)*16);return copy_index<2 && tick>=at && two_case!=3;}return !timeout && taken<frames_total && tick>=frame_tick[taken];}
static bool relay_radio_take(struct relay_rx_frame *f){if(two_case){f->data[0]=copy_index++?0xec:0xe8;return true;}unsigned i=taken++;f->data[0]=frame_type[i];return !(take_fail && i==0);}
static void relay_radio_peek(uint8_t *len,bool *crc,uint8_t *rssi){*len=1;*crc=false;*rssi=0;}
static bool relay_radio_stop(void){assert(prepared && !depth);stopped++;return true;}
static void esb_raw_rx_end(void){assert(claimed);claimed=0;}
static bool esb_relay_control_receive(struct relay_rx_frame *f){if(completed_late)assert(f->end_kernel_ticks==111);assert(!depth && !claimed);return two_case ? (two_case!=3 && (two_case!=2 || f->data[0]==0xec)) : f->data[0]==0xe8 && !reject;}
static bool relay_radio_addressed(void){return false;}
'''
cases = r'''
int main(int argc,char **argv){
 assert(argc==2);const char *c=argv[1];
 clock_fail=!strcmp(c,"clock");init_fail=!strcmp(c,"init");wait_fail=!strcmp(c,"wait");
 timeout=!strcmp(c,"no-end") || !strcmp(c,"scan") || !strcmp(c,"scan-delay");
 if(!strcmp(c,"scan") || !strcmp(c,"scan-delay"))relay_direct_anchor.valid=false;take_fail=!strcmp(c,"crc");stamp_fail=!strcmp(c,"stamp");reject=!strcmp(c,"reject");
 if(!strcmp(c,"busy"))busy_attempts=2;
 /* An ordinary pipe-1 packet ends just before B; B must still be taken. */
 bool ordinary=!strcmp(c,"ordinary-then-b");
 if(ordinary){frames_total=2;frame_type[0]=0x01;frame_tick[1]=105;}
 if(!strcmp(c,"deadline") || !strcmp(c,"no-clock"))busy_attempts=100;
 edge_wait_fail=!strcmp(c,"wait-edge");prepare_irq=!strcmp(c,"prepare-irq") || !strcmp(c,"scan-delay");setup_work=!strcmp(c,"setup-work");
 network_fail=!strcmp(c,"no-clock");completed_late=!strcmp(c,"completed-late");
 if(!strncmp(c,"two-",4)){
  geometry.repairs=strstr(c,"short")?2:4;if(strstr(c,"short"))relay_direct_anchor.next_start=164;geometry.second_b=strstr(c,"tail")?2:1;
  two_case=strstr(c,"reject")?2:strstr(c,"both")?3:1;
  if(two_case==1)copy_index=1;
  bool ok=esb_relay_control_window(100);
  assert(ok==(two_case!=3));assert(control_missed==(two_case==3));
  assert(!claimed && !depth && !clock_users);puts(c);return 0;
 }
 bool early=clock_fail||init_fail||wait_fail||busy_attempts==100||edge_wait_fail;
 bool result=esb_relay_control_window(100);
 assert(result==!(early||timeout||take_fail||stamp_fail||reject));
 assert(!depth && !claimed && !clock_users);
 /* A corrupt, unstamped or ordinary frame re-arms RX once. */
 unsigned rearms=(take_fail||stamp_fail||ordinary)?1:0;
 assert(started==(early?0:1+rearms) && stopped==prepared);
 if(ordinary)assert(result && taken==2 && !control_missed);
 if(edge_wait_fail)assert(prepared && !started);
 if(!strcmp(c,"prepare-irq"))assert(prepared && started && tick==118);
 if(!strcmp(c,"busy"))assert(attempts==3 && delay_calls==2);
 if(timeout)assert(control_window_fail[5]==1 && control_no_address==1);
 if(!strcmp(c,"scan"))assert(tick>=100+274+16);
 if(take_fail)assert(control_window_fail[3]==1 && control_crc_fail==1);
 if(stamp_fail)assert(control_window_fail[4]==1);
 if(clock_fail)assert(control_window_fail[0]==1);
 if(init_fail)assert(control_window_fail[2]==1);
 if(wait_fail||busy_attempts==100)assert(control_window_fail[1]==1);
 printf("PASS control window %s\n",c);return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='relay-control-', dir='/tmp') as tmp:
    path = Path(tmp) / 'control.c'
    path.write_text(prefix + window + cases)
    binary = Path(tmp) / 'control'
    subprocess.run(shlex.split(os.environ.get('CC', 'cc')) + [
        '-std=c11', '-Wall', '-Wextra', '-Werror', '-g', '-O1',
        '-fsanitize=address,undefined', '-I', str(ROOT / 'src/connection'),
        str(path), '-o', str(binary)], check=True)
    for case in sys.argv[1:] or ('setup-work','scan-delay','wait-edge','prepare-irq','completed-late','scan','valid','busy','deadline','no-clock','clock','init','wait','no-end','crc','stamp','reject','ordinary-then-b'):
        subprocess.run([str(binary), case], check=True)

    for case in ('two-adjacent','two-tail','two-reject-adjacent','two-reject-tail','two-both-adjacent','two-both-tail','two-short-tail','two-both-short-tail'):
        subprocess.run([str(binary),case],check=True)
