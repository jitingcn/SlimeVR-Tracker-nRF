#!/usr/bin/env python3
"""Production raw-air claim and relay-only TX submission under fault injection.

The ESB and TDMA leaves record calls; production TDMA admission/revalidation
is tested separately by run_tdma.py.
"""
import os
from pathlib import Path
import re
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
tdma = (ROOT / "src/connection/tdma.c").read_text()
relay = (ROOT / "src/connection/relay.c").read_text()
quiet_defines = '\n'.join(re.findall(r'^#define TDMA_(?:STAGED_BIT|ACTIVATION_QUIET_TICKS)\b[^\n]*', tdma, re.M))
quiet = extract_block(tdma, r'^bool tdma_window_quiet\([^;{]*\)\s*\{')
maintenance_grant = '\n'.join(extract_block(relay, rf'^bool relay_{name}_maintenance\([^;{{]*\)\s*\{{')
                              for name in ("next", "take"))
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
static bool prepared, waited, profile, repair_built, peer_report;
static bool announce_during_wait, delay_maintenance;
static uint32_t tdma_staged_pack, tdma_staged_activation, tdma_recent_activation;
static bool tdma_recent_activation_valid;
static uint32_t atomic_get(const uint32_t *p){return *p;}
static uint32_t maintenance_now, maintenance_start;
static bool control_valid, maintenance_attempted;
static uint32_t relay_maintenance_late, relay_maintenance_lag_max;
static struct relay_control control_plan;
static uint8_t control_helper = 2;
static bool tdma_wait_network_tick(uint32_t t){
 assert(prepared && t==(repair_built ? 190 : maintenance_start+RELAY_TX_OFFSET));
 waited=true;
 maintenance_now=t+2U*delay_maintenance;
 if(announce_during_wait)tdma_staged_pack=TDMA_STAGED_BIT;
 return true;
}
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
static bool relay_next_repair(uint32_t *t,uint8_t *s){*t=194;*s=RELAY_REPAIR_FIRST_SLOT;return profile;}
static void relay_repair_result(int result){(void)result;}
static size_t relay_build_repair(uint8_t *p,uint8_t s){assert(s==RELAY_REPAIR_FIRST_SLOT);p[0]=peer_report?RELAY_PEER_REPORT:RELAY_REPAIR;prepared=repair_built=true;return 22;}
static bool relay_head_active(void){return true;}
static bool clock_status;
static unsigned clock_starts;
static int clocks_start(void){assert(tx_clock_users);clock_starts++;return 0;}
static bool tdma_wait_for_repair_slot(uint8_t s,uint8_t n){(void)s;(void)n;assert(!profile || (repair_built && waited && n==22));return granted;}
static bool tdma_wait_for_slot(uint8_t n){(void)n;return true;}
static uint32_t admit_now=194;
static bool tdma_network_now(uint32_t *t){*t=prepared && !repair_built ? maintenance_now : admit_now;return true;}
static bool esb_is_idle(void){return true;}
static bool tdma_repair_tx_valid(uint8_t s,uint8_t n){(void)s;(void)n;assert(irq_depth);return final_valid;}
static bool tdma_slot_tx_valid(uint8_t n){(void)n;assert(irq_depth);return final_valid;}

static int esb_write_payload(const struct esb_payload *p){
 assert(p->data[0]==(source_payload ? 0x77 : profile ? (peer_report ? RELAY_PEER_REPORT : RELAY_REPAIR) : RELAY_REPAIR) && writer_air);if(queue_error)return queue_error;queued++;return 0;
}
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
 assert(argc==2);profile=true;
 if(!strncmp(argv[1],"source-",7)){
  struct esb_payload payload={.length=22,.data={0x77}};source_payload=true;
  inject_claim_irq=true;
  if(!strcmp(argv[1],"source-expired"))final_valid=false;
  if(!strcmp(argv[1],"source-grant-expired")){granted=false;profile=false;}
  if(!strcmp(argv[1],"source-busy"))raw_rx_active=true;
  if(!strcmp(argv[1],"source-full"))queue_error=-ENOMEM;
  int expected=raw_rx_active ? -EBUSY : (!final_valid || !granted) ? -EAGAIN : queue_error;
  assert(esb_submit_guarded(&payload,0)==expected);
  assert(queued==(expected==0) && started==(expected==0));
  assert(direct_fail[1]==raw_rx_active && direct_fail[2]==(expected==-EAGAIN));
  assert(esb_write_dropped==(expected==-ENOMEM));
 }else if(!strncmp(argv[1],"maintenance",11)){
  uint8_t ping[ESB_PING_LEN]={ESB_PING_TYPE};
  control_plan=(struct relay_control){.preamble_ticks=300,.maintenance_owner=2,
      .flags=RELAY_CONTROL_MAINTENANCE};
  control_valid=true;
  maintenance_start=relay_repair_start(control_plan.preamble_ticks,0);
  maintenance_now=maintenance_start-1;
  delay_maintenance=!strcmp(argv[1],"maintenance-late");
  bool overlap=!strcmp(argv[1],"maintenance-fence");
  bool adjacent=!strcmp(argv[1],"maintenance-fence-edge");
  announce_during_wait=overlap || adjacent;
  /* No quiet ticks in R0, and the first quiet tick may be exactly at
   * the exclusive end of R1. Announce after the producer waits. */
  tdma_staged_activation=maintenance_start+2U*RELAY_SLOT_TICKS+TDMA_ACTIVATION_QUIET_TICKS-!adjacent;
  bool send=!delay_maintenance && !overlap;
  assert(esb_write_maintenance(ping)==(send ? 0 : -EAGAIN));
  assert(maintenance_attempted && queued==send && started==send && maintenance_sent==send);
 }else if(!strcmp(argv[1],"repair") || !strcmp(argv[1],"repair-warm") || !strcmp(argv[1],"peer-report")){
  clock_status=!strcmp(argv[1],"repair-warm");
  peer_report=!strcmp(argv[1],"peer-report");
  profile=true;assert(esb_write_relay()==0);
  assert(last_tx.type==(peer_report?RELAY_PEER_REPORT:RELAY_REPAIR));assert(repair_built && waited && queued==1 && started==1);
  assert(clock_starts==!clock_status);
 }else if(!strcmp(argv[1],"repair-ungranted")){
  profile=true;granted=false;assert(esb_write_relay()==-EAGAIN);
  assert(!queued && !started);
 }else if(!strcmp(argv[1],"repair-next-frame")){
  /* Admission resumed in the same R slot of a later frame: not this grant. */
  profile=true;admit_now=194+RELAY_SLOT_TICKS;assert(esb_write_relay()==-EAGAIN);
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
    path=Path(tmp)/"submission.c"
    path.write_text('\n'.join((quiet_defines, PREFIX, quiet, maintenance_grant, claim, guarded, writer, maintenance, CASES)))
    binary=Path(tmp)/"submission"
    subprocess.run(shlex.split(os.environ.get("CC","cc"))+[
        "-std=c11","-Wall","-Wextra","-Werror","-Wno-unused-function",
        "-g","-O1","-fsanitize=address,undefined","-I",str(ROOT/"src/connection"),
        str(path),"-o",str(binary)],check=True)
    for case in sys.argv[1:] or ("source-valid","source-expired","source-grant-expired","source-busy","source-full","maintenance","maintenance-late","maintenance-fence","maintenance-fence-edge","repair","repair-warm","peer-report","repair-ungranted","repair-next-frame","air","expired","collection","no-grant","valid"):
        subprocess.run([str(binary),case],check=True)
