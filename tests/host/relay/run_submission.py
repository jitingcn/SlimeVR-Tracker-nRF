#!/usr/bin/env python3
"""Production raw-air claim and relay-only TX submission under fault injection.

The ESB and TDMA leaves record calls; production TDMA admission/revalidation
is tested separately by run_tdma.py.
"""
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile

HERE=Path(__file__).resolve().parent
ROOT=Path(os.environ.get("SOURCE_ROOT", HERE.parents[2]))
sys.path.insert(0,str(HERE.parent/"harness/python"))
from c_extract import extract_block
source=(ROOT/"src/connection/esb.c").read_text()
claim=extract_block(source,r'^static int esb_air_claim\([^;{]*\)\s*\{')
guarded=extract_block(source,r'^static int esb_submit_guarded\([^;{]*\)\s*\{')
writer=extract_block(source,r'^int esb_write_relay\([^;{]*\)\s*\{')
maintenance=extract_block(source,r'^int esb_write_maintenance\([^;{]*\)\s*\{')
PREFIX=r'''
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "relay_protocol.h"
#include "../frame_ledger.h"
#define K_FOREVER 0
#define ESB_PING_LEN 13
#define ESB_PING_TYPE 0xf0
static unsigned maintenance_sent;
static bool maintenance_valid=true;
static bool prepared, waited, profile, repair_built, peer_report;
static bool relay_next_maintenance(uint32_t *t){*t=194;return maintenance_valid;}
static bool tdma_wait_network_tick(uint32_t t){assert(prepared && t==(profile ? 190 : 194));waited=true;return true;}
#define ESB_ST_PAIRING 0
#define ESB_CREATE_PAYLOAD(...) {0}
struct esb_payload{uint8_t pipe,noack,length,data[72];};
static bool esb_initialized=true,radio_user_disabled,channel_wait_normal,layout_stale;
static bool raw_rx_active,writer_air,collection,final_valid=true,granted=true;
static bool esb_layout_granted(void){assert(!profile);return granted;}
static unsigned esb_conn_state=1,tracker_id=2,esb_radio_lock,tx_clock_users;
static unsigned irq_depth,queued,started,busy_delays,popped;
static unsigned direct_fail[3],esb_write_queued,esb_write_dropped,consecutive_enomem_errors;
static bool inject_claim_irq,source_payload;
static int queue_error;
static uint32_t last_tx_time;
static struct {uint8_t type,noack,length;uint32_t timestamp;} last_tx;
static struct {unsigned air_waits,air_timeouts;} presence_stats;
static unsigned irq_lock(void){return irq_depth++;}
static void irq_unlock(unsigned key){assert(irq_depth==key+1);irq_depth=key;if(inject_claim_irq && !key && writer_air && !queued) final_valid=false;}
static void k_busy_wait(unsigned n){busy_delays+=n;}
static void k_mutex_lock(unsigned *p,int t){(void)t;(*p)++;}
static void k_mutex_unlock(unsigned *p){assert(*p);(*p)--;}
static void atomic_inc(unsigned *p){(*p)++;}
static unsigned k_uptime_get(void){if(profile && !queued) assert(!waited);return 1000;}
static bool esb_ota_is_active(void){return false;}
static bool connection_get_data_collection(void){return collection;}
static bool tdma_is_enabled(void){return true;}
static uint8_t tdma_profile_get(void){return profile;}
static bool relay_next_repair(uint32_t *t,uint8_t *s){*t=194;*s=RELAY_V3_REPAIR_FIRST_SLOT;return profile;}
static void relay_repair_result(int result){(void)result;}
static size_t relay_build_repair(uint8_t *p,uint8_t s){assert(s==RELAY_V3_REPAIR_FIRST_SLOT);p[0]=peer_report?RELAY_PEER_REPORT_V3:RELAY_REPAIR_V3;prepared=repair_built=true;return 22;}
static bool relay_head_active(void){return true;}
static bool relay_records_pending(void){return true;}
static bool clock_status;
static unsigned clock_starts;
static int clocks_start(void){assert(tx_clock_users);clock_starts++;return 0;}
static int relay_forward_slot(void){assert(!profile);return 5;}
static bool tdma_wait_for_forward_slot(uint8_t s,uint8_t n){(void)s;(void)n;assert(!profile || (repair_built && waited && n==22));return granted;}
static bool tdma_wait_for_slot(uint8_t n){(void)n;return true;}
static bool esb_is_idle(void){return true;}
static bool tdma_forward_tx_valid(uint8_t s,uint8_t n){(void)s;(void)n;assert(irq_depth);return final_valid;}
static bool tdma_slot_tx_valid(uint8_t n){(void)n;assert(irq_depth);return final_valid;}
static size_t relay_build_bundle(uint8_t *p,size_t own,size_t cap,uint8_t id){
 (void)own;(void)cap;p[0]=RELAY_BUNDLE_TYPE;p[1]=id;return 22;
}
static int esb_write_payload(const struct esb_payload *p){
 assert(p->data[0]==(source_payload ? 0x77 : profile ? (peer_report ? RELAY_PEER_REPORT_V3 : RELAY_REPAIR_V3) : RELAY_BUNDLE_TYPE) && writer_air);if(queue_error)return queue_error;queued++;return 0;
}
static bool relay_take_maintenance(void){assert(irq_depth && writer_air);return maintenance_valid;}
static void esb_prepare_ping(uint8_t *p){p[2]=42;prepared=true;}
static int esb_queue_ping(const struct esb_payload *p){
 assert(waited && writer_air && irq_depth && !p->noack && p->pipe==3 && p->length==13);
 assert(p->data[0]==ESB_PING_TYPE && p->data[2]==42);queued++;return 0;
}
static void esb_start_queued_tx(void){started++;}
static void esb_air_release(void){writer_air=false;}
static void esb_release_tx_clock(bool keep){(void)keep;assert(tx_clock_users);tx_clock_users--;}
static void drop_failed_tx_payload_if_pending(void){popped++;}
'''
CASES=r'''
int main(int argc,char **argv){
 assert(argc==2);
 if(!strncmp(argv[1],"source-",7)){
  struct esb_payload payload={.length=22,.data={0x77}};source_payload=true;
  inject_claim_irq=true;
  if(!strcmp(argv[1],"source-expired"))final_valid=false;
  if(!strcmp(argv[1],"source-busy"))raw_rx_active=true;
  if(!strcmp(argv[1],"source-full"))queue_error=-ENOMEM;
  int expected=raw_rx_active ? -EBUSY : !final_valid ? -EAGAIN : queue_error;
  assert(esb_submit_guarded(&payload,0)==expected);
  assert(queued==(expected==0) && started==(expected==0));
  assert(direct_fail[1]==raw_rx_active && direct_fail[2]==(expected==-EAGAIN));
  assert(esb_write_dropped==(expected==-ENOMEM));
 }else if(!strcmp(argv[1],"maintenance") || !strcmp(argv[1],"maintenance-late")){
  uint8_t ping[ESB_PING_LEN]={ESB_PING_TYPE};
  maintenance_valid=strcmp(argv[1],"maintenance-late")!=0;
  assert(esb_write_maintenance(ping)==(maintenance_valid ? 0 : -EAGAIN));
  assert(queued==maintenance_valid && started==maintenance_valid && maintenance_sent==maintenance_valid);
 }else if(!strcmp(argv[1],"repair") || !strcmp(argv[1],"repair-warm") || !strcmp(argv[1],"peer-report")){
  clock_status=!strcmp(argv[1],"repair-warm");
  peer_report=!strcmp(argv[1],"peer-report");
  profile=true;assert(esb_write_relay()==0);
  assert(last_tx.type==(peer_report?RELAY_PEER_REPORT_V3:RELAY_REPAIR_V3));assert(repair_built && waited && queued==1 && started==1);
  assert(clock_starts==!clock_status);
 }else if(!strcmp(argv[1],"repair-ungranted")){
  profile=true;granted=false;assert(esb_write_relay()==-EAGAIN);
  assert(!queued && !started);
 }else if(!strcmp(argv[1],"air")){
  raw_rx_active=true;assert(esb_air_claim()==-EBUSY);
  assert(!writer_air && !busy_delays);
 }else if(!strcmp(argv[1],"expired")){
  final_valid=false;assert(esb_write_relay()==-EAGAIN);assert(!queued && !started);
 }else if(!strcmp(argv[1],"collection")){
  collection=true;assert(esb_write_relay()==-EAGAIN);assert(!queued && !started);
 }else if(!strcmp(argv[1],"no-grant")){
  granted=false;assert(esb_write_relay()==-EAGAIN);assert(!queued && !started);
 }else if(!strcmp(argv[1],"valid")){
  assert(esb_write_relay()==0);assert(queued==1 && started==1 && popped==1);
 }else assert(false);
 assert(!irq_depth && !esb_radio_lock && !tx_clock_users && !writer_air);
 printf("PASS relay submission %s\n",argv[1]);return 0;
}
'''
with tempfile.TemporaryDirectory(prefix="relay-submit-",dir="/tmp") as tmp:
    path=Path(tmp)/"submission.c";path.write_text(PREFIX+claim+'\n'+guarded+'\n'+writer+'\n'+maintenance+CASES)
    binary=Path(tmp)/"submission"
    subprocess.run(shlex.split(os.environ.get("CC","cc"))+[
        "-std=c11","-Wall","-Wextra","-Werror","-Wno-unused-function",
        "-g","-O1","-fsanitize=address,undefined","-I",str(ROOT/"src/connection"),
        str(path),"-o",str(binary)],check=True)
    for case in sys.argv[1:] or ("source-valid","source-expired","source-busy","source-full","maintenance","maintenance-late","repair","repair-warm","peer-report","repair-ungranted","air","expired","collection","no-grant","valid"):
        subprocess.run([str(binary),case],check=True)
