#!/usr/bin/env python3
"""Exercise the source transaction with IRQ injection and setup/teardown faults."""
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
functions = '\n'.join(extract_block(source, rf'^(?:void|bool|int) {name}\([^;{{]*\)\s*\{{')
                      for name in ('esb_source_end', 'esb_source_begin', 'esb_source_write'))
prefix = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#define K_FOREVER 0
#define ESB_ST_PAIRING 0
#define LEDGER_SOURCE_REASON(x) ((void)0)
struct esb_source_guard { unsigned key; bool active; };
static unsigned depth, esb_radio_lock, tx_clock_users, reports, writes, pending, delivered;
static unsigned tick, published_pose, captured_pose;
static bool esb_initialized=true,radio_user_disabled,clock_status,profile=true,enabled=true;
static unsigned esb_conn_state=1;
static int clock_error,write_error;
static bool wait_ok=true;
static unsigned irq_lock(void) { return depth++; }
static void irq_unlock(unsigned key) { assert(depth==key+1);depth=key; if(!depth){delivered+=pending;pending=0;} }
static void interrupt_work(void) { if(depth)pending++;else tick+=8; }
static void k_mutex_lock(unsigned *p,int timeout) { (void)timeout;assert(!depth && !*p);*p=1; }
static void k_mutex_unlock(unsigned *p) { assert(!depth && *p);*p=0; }
static void atomic_inc(unsigned *p) { ++*p; }
static int clocks_start(void) { assert(!depth && esb_radio_lock && tx_clock_users);return clock_error; }
static void drop_failed_tx_payload_if_pending(void) { assert(!depth); }
static bool tdma_wait_network_tick(uint32_t at) {
 assert(!depth && esb_radio_lock && tx_clock_users && at==96);
 tick=at;published_pose=42;return wait_ok;
}
static bool tdma_profile_get(void) { return profile; }
static bool tdma_is_enabled(void) { return enabled; }
static int esb_write_clocked(uint8_t *data,bool noack,size_t length,bool stream_source) {
 assert(stream_source && depth && esb_radio_lock && tx_clock_users && noack && length==1 && *data==42);
 interrupt_work();tick+=2;assert(tick<=100);writes++;return write_error;
}
static void esb_write_rate_tick(void) { assert(!depth);reports++; }
static void esb_release_tx_clock(bool keep) { assert(!depth && !keep && tx_clock_users);tx_clock_users--; }
'''
cases = r'''
int main(int argc,char **argv) {
 assert(argc==2);const char *c=argv[1];
 clock_error=!strcmp(c,"clock") ? -EIO : 0;
 wait_ok=strcmp(c,"wait");esb_initialized=strcmp(c,"init");
 radio_user_disabled=!strcmp(c,"disabled");
 if(!strcmp(c,"pairing"))esb_conn_state=ESB_ST_PAIRING;
 clock_status=!strcmp(c,"warm");
 bool early=clock_error || !wait_ok || !esb_initialized || radio_user_disabled || !esb_conn_state;
 struct esb_source_guard guard={0};
 assert(esb_source_begin(&guard,100)==!early);
 if(!early) {
  /* An interrupt at capture/encoding/setup cannot consume the slot margin;
   * publication during the unchanged wait is included in the fresh pose. */
  interrupt_work();captured_pose=published_pose;assert(captured_pose==42);
  interrupt_work();tick+=1;
  profile=strcmp(c,"profile");enabled=strcmp(c,"tdma-off");
  if(!strcmp(c,"refused"))write_error=-EAGAIN;
  if(strcmp(c,"abandon")) {
   uint8_t packet=captured_pose;
   assert(esb_source_write(&guard,&packet,1)==(profile && enabled ? write_error : -EAGAIN));
   assert(writes==(profile && enabled));
  }
 }
 esb_source_end(&guard);esb_source_end(&guard);
 assert(!depth && !esb_radio_lock && !tx_clock_users && !guard.active);
 assert(reports==!early && !pending);
 printf("PASS source guard %s\n",c);
}
'''
with tempfile.TemporaryDirectory(prefix='source-guard-',dir='/tmp') as tmp:
    c=Path(tmp)/'source.c';c.write_text(prefix+functions+cases);exe=Path(tmp)/'source'
    subprocess.run(shlex.split(os.environ.get('CC','cc'))+['-std=c11','-Wall','-Wextra','-Werror','-O1','-g','-fsanitize=address,undefined',str(c),'-o',str(exe)],check=True)
    for case in ('valid','warm','clock','wait','init','disabled','pairing','profile','tdma-off','refused','abandon'):
        subprocess.run([str(exe),case],check=True)
