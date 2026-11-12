#!/usr/bin/env python3
"""Production passive-PONG parser, reboot fence and schedule adoption.

Clock filtering and kernel/radio leaves are modeled; parser, CRC gates,
presence accounting and recovery decisions are taken from esb.c unchanged.
"""
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2]))
sys.path.insert(0, str(HERE.parent / "harness/python"))
from c_extract import extract_block
esb = (ROOT / "src/connection/esb.c").read_text()
header = (ROOT / "src/connection/esb.h").read_text()
constants = '\n'.join(re.findall(r'^#define ESB_(?:PING|PONG)\w*[^\n]*', header, re.MULTILINE))
constants += '\n' + re.search(r'^#define RECEIVER_RESTART_BACKWARD_TICKS[^\n]*', esb, re.MULTILINE)[0]
note = extract_block(esb, r'^static void esb_note_trailer_locked\([^;{]*\)\s*\{')
function = extract_block(esb, r'^bool esb_presence_frame\([^;{]*\)\s*\{')
control = extract_block(esb, r'^bool esb_relay_control_receive\([^;{]*\)\s*\{')
lease = extract_block(esb, r'^static inline bool esb_v3_lease_at\([^;{]*\)\s*\{')
grant_at = extract_block(esb, r'^bool esb_relay_granted_at\([^;{]*\)\s*\{')
grant = extract_block(esb, r'^bool esb_layout_granted\([^;{]*\)\s*\{')
stage = extract_block(esb, r'^static void esb_stage_from_trailer\([^;{]*\)\s*\{')
constants += '\n' + re.search(r'^#define TDMA_SLOT_UNASSIGNED[^\n]*',
                              (ROOT / "src/connection/tdma.h").read_text(), re.MULTILINE)[0]
PREFIX = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define CONFIG_RELAY_RUNTIME_LAYOUT 1
#include "relay_protocol.h"
static struct relay_geometry geometry={11,5,16,120,4096,15, 0};
const struct relay_geometry *relay_geometry_active=&geometry;
struct relay_timeline relay_timeline_active;
struct relay_runtime_anchor relay_direct_anchor;
#include "link_supervision.h"
#include "relay_radio.h"
#define TDMA_MAX_TRACKERS 16
#define ESB_ST_PAIRING 0
#define ESB_ST_PAIRED 1
#define ESB_ST_RECOVERING 2
#define SYS_STATUS_CONNECTION_ERROR 1
static unsigned ping_failures,ping_success_streak;
static int64_t connection_error_start_time;
static bool shutdown_requested,status_error;
static void set_status(unsigned mask,bool on){(void)mask;status_error=on;}
static unsigned irq_lock(void){return 0;}
static void irq_unlock(unsigned k){(void)k;}
static bool esb_initialized=true,channel_search,channel_wait_normal,server_time_synced=true,layout_stale;
static bool channel_confirmed=true,channel_legacy_peer,receiver_heard_seen,relayed_confirm_seen,ping_pending;
static unsigned esb_conn_state=1,channel_redirect_pending,tracker_id=2,channel_found;
static unsigned presence_epoch=7,own_pong_length=23,receiver_heard_time,g_last_rx_raw_ticks=100000;
static unsigned g_last_sync_timestamp,receiver_restart_detections,recovery_requests,filter_updates;
static bool tdma_enabled=true;
static struct {uint32_t offset,last_local_ticks;int32_t skew_ppb;bool initialized;} sync_filter={.initialized=true};
static struct {unsigned pongs,sync_accepted,sync_rejected,layout_stale,layout_followed,relayed_confirms;} presence_stats;
static int atomic_get(unsigned *p){return *p;}
static void atomic_set(unsigned *p,unsigned n){*p=n;}
static uint32_t sys_get_be32(const uint8_t *p){return (uint32_t)p[0]<<24|(uint32_t)p[1]<<16|(uint32_t)p[2]<<8|p[3];}
static uint8_t crc8_ccitt(uint8_t v,const uint8_t *p,unsigned n){return relay_crc8(v,p,n);}
static uint64_t kernel_ticks_extend32(uint32_t v){return v;}
static uint64_t net_ticks_from_kernel64(uint64_t v){return v;}
static uint32_t k_uptime_get_32(void){return 3000;}
static uint32_t k_uptime_get(void){return 3000;}
static int64_t net_tick_difference(uint32_t a,uint32_t b){return (int32_t)(a-b);}
static uint32_t relayed_confirm_time;
struct esb_sync_sample{uint32_t local_ticks,offset;};
static int esb_sync_filter_update(void *p,void *s){(void)p;(void)s;filter_updates++;return 0;}
static uint8_t tdma_get_config_epoch(void){return presence_epoch;}
static bool tdma_is_enabled(void){return tdma_enabled;}
static bool tdma_update_config(uint8_t slot,uint8_t total,uint8_t width,uint8_t epoch){
 if(slot>=total || total>16 || width<16)return false;
 presence_epoch=epoch;return true;
}
static void esb_clear_time_sync_state(void){server_time_synced=false;sync_filter.initialized=false;}
static void esb_wom_reset(void){}
static void relay_reset(void){}
static void relay_on_control(const struct relay_control *c,uint8_t h){(void)c;(void)h;}
static void tdma_set_enabled(bool on){tdma_enabled=on;}
static void tdma_profile_drop(void){tdma_enabled=false;}
static void connection_request_ping_resync(void){recovery_requests++;}
static uint32_t layout_confirm_time;
static bool layout_confirm_seen;
static bool control_seen, stream_profile;
static unsigned control_received;
static uint32_t control_first_received, control_repeat_received, control_repeat_seen, control_repeat_reject[6];
static uint32_t control_reject[6];
static int32_t control_err_min, control_err_max, control_err_last;
static uint32_t control_session, control_frame, control_tick, current_tick;
struct esb_network_clock { uint32_t receiver_ticks; };
static bool esb_get_network_clock(struct esb_network_clock *c){c->receiver_ticks=current_tick;return true;}
static bool esb_receiver_relay_aware(void){return true;}
static bool tdma_layout_withdrawn(void){return false;}
static uint8_t tdma_profile_get(void){return stream_profile;}
struct tdma_layout {uint8_t profile,epoch,slot_index,slot_ticks;};
static bool tdma_layout_get(struct tdma_layout *l){*l=(struct tdma_layout){1,7,2,16};return tdma_enabled;}
static void tdma_stream_control(const struct relay_control *c){
 relay_geometry_make(&geometry,c->direct_count,c->repair_count,c->slot_ticks,c->tps_max);
 relay_runtime_anchor_set(&relay_direct_anchor,c->frame,c->preamble_ticks,c->direct_count,c->repair_count,relay_control_next_repairs(c),relay_control_next2_repairs(c),c->slot_ticks,c->tps_max);
 relay_timeline_announce(&relay_timeline_active,&relay_direct_anchor);
}
#define BIT(x) (1U<<(x))
#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))
static struct {unsigned calls,clears;uint8_t slot,total,width,epoch;uint32_t activation;} staged;
static bool tdma_stage_config(uint8_t slot,uint8_t total,uint8_t width,uint8_t epoch,uint32_t act){
 staged.calls++;staged.slot=slot;staged.total=total;staged.width=width;staged.epoch=epoch;staged.activation=act;
 return true;
}
static void tdma_stage_clear(void){staged.clears++;}
'''
CASES = r'''
static struct relay_rx_frame pong_with(uint32_t t2,uint8_t epoch,uint16_t members,bool pending) {
 struct relay_rx_frame f={.pipe=2,.end_kernel_ticks=100000};
 f.data[0]=ESB_PONG_TYPE;f.data[1]=1;
 f.data[3]=t2>>24;f.data[4]=t2>>16;f.data[5]=t2>>8;f.data[6]=t2;
 f.data[7]=ESB_PONG_FLAG_NORMAL;f.data[9]=3;f.data[10]=16;f.data[11]=epoch;
 f.data[12]=relay_crc8(7,f.data,12);
 struct relay_pong_trailer tr={.epoch=epoch,.forward_slot=255,.member_mask=members,.pending=pending,
  .pending_epoch=9,.pending_total=4,.pending_slot_ticks=16,.pending_member_mask=0xF,.activation_units=100};
 f.length=13+relay_trailer_encode(f.data+13,&tr);return f;
}
static struct relay_rx_frame pong(uint32_t t2,uint8_t epoch){return pong_with(t2,epoch,7,false);}
int main(int argc,char **argv) {
 assert(argc==2);
 struct relay_rx_frame f;
 if(!strcmp(argv[1],"reboot")) {
  f=pong(1000,8);assert(esb_presence_frame(&f));
  assert(channel_wait_normal && !server_time_synced && !tdma_enabled);
  assert(!channel_confirmed && recovery_requests==1 && receiver_restart_detections==1);
  assert(presence_epoch==7);
 } else if(!strcmp(argv[1],"rejected")) {
  f=pong_with(100010,7,7,true);assert(esb_presence_frame(&f));
  assert(!filter_updates && presence_epoch==7);
  assert(!layout_confirm_seen && !staged.calls && !staged.clears); /* no clock agreement */
  f=pong(100010,8);assert(esb_presence_frame(&f));
  assert(!filter_updates && layout_stale && presence_epoch==7);
 } else if(!strcmp(argv[1],"valid")) {
  f=pong(100000,8);assert(esb_presence_frame(&f));
  assert(filter_updates==1 && !layout_stale && presence_epoch==8);
  assert(layout_confirm_seen && layout_confirm_time==3000 && staged.clears==1);
 } else if(!strcmp(argv[1],"grant")) {
  f=pong_with(100000,7,3,false);assert(esb_presence_frame(&f)); /* lists 0,1 only */
  assert(!layout_confirm_seen);
  /* Epochs are unordered: a different epoch listing us is followed and confirms. */
  f=pong_with(100000,6,7,false);assert(esb_presence_frame(&f));
  assert(layout_confirm_seen && presence_epoch==6);layout_confirm_seen=false;
  f=pong_with(100000,6,7,false);assert(esb_presence_frame(&f));
  assert(layout_confirm_seen && layout_confirm_time==3000);
 } else if(!strcmp(argv[1],"pending")) {
  f=pong_with(100000,7,7,true);assert(esb_presence_frame(&f));
  assert(staged.calls==1 && staged.slot==2 && staged.total==4 && staged.width==16 && staged.epoch==9);
  assert(staged.activation==100000+100*RELAY_ACTIVATION_UNIT_TICKS);
  assert(layout_confirm_seen && presence_epoch==7);
  layout_confirm_seen=false;f=pong_with(100000,7,7,true);
  f.data[13+12]^=1; /* corrupt a pending field: the trailer CRC rejects it all */
  assert(esb_presence_frame(&f));assert(staged.calls==1 && !layout_confirm_seen);
 } else if(!strcmp(argv[1],"invalid-tuple")) {
  f=pong(100000,8);f.data[10]=0;f.data[12]=relay_crc8(7,f.data,12);
  assert(esb_presence_frame(&f));assert(layout_stale && presence_epoch==7);
 } else if(!strcmp(argv[1],"relay-recovery")) {
  esb_conn_state=ESB_ST_RECOVERING;status_error=true;ping_failures=300;
  connection_error_start_time=100;shutdown_requested=true;
  struct relay_pong_trailer tr={.relayed_mask=4};
  esb_note_trailer_locked(&tr,3000);
  assert(!ping_failures && !status_error && !connection_error_start_time && !shutdown_requested);
  assert(esb_conn_state==ESB_ST_PAIRED);
 } else if(!strcmp(argv[1],"control") || !strcmp(argv[1],"control-demand")) {
  stream_profile=true;relay_geometry_make(&geometry,3,5,16,120);
  struct relay_control b={.epoch=7,.repair_count=5,.session=42,.frame=10,
   .preamble_ticks=100000,.members=7,.expected=7,.profile=1,.maintenance_owner=255,
 .on_demand=!strcmp(argv[1],"control-demand")};
 if(b.on_demand){relay_control_set_next_repairs(&b,0);relay_control_set_next2_repairs(&b,2);}
  for(unsigned i=0;i<5;i++) b.grants[i].owner=RELAY_OWNER_IDLE;
  f=(struct relay_rx_frame){.pipe=1,.length=72,.end_kernel_ticks=100011};
  relay_control_encode(f.data,&b);
  assert(!esb_layout_granted());
  /* B sets the offset directly: preamble = END - B airtime. */
  assert(esb_relay_control_receive(&f) && control_seen && sync_filter.last_local_ticks==100000);
  assert(layout_confirm_seen && receiver_heard_seen && relayed_confirm_seen);
  uint32_t cutoff=relay_direct_anchor.third_start+3U*16U;
  current_tick=cutoff-1;assert(esb_layout_granted() && esb_relay_granted_at(current_tick));
  current_tick=cutoff;assert(!esb_layout_granted() && !esb_relay_granted_at(current_tick));
  relay_runtime_anchor_set(&relay_direct_anchor,10,0xfffffff0U,3,5,0,2,16,120);
  cutoff=relay_direct_anchor.third_start+3U*16U;
  assert(esb_v3_lease_at(cutoff-1));assert(!esb_v3_lease_at(cutoff));

  assert(!esb_relay_control_receive(&f)); /* duplicate/stale frame */
  b.frame++; relay_control_encode(f.data,&b); f.end_kernel_ticks+=10;
  assert(!esb_relay_control_receive(&f) && sync_filter.last_local_ticks==100000); /* late delivery */
  f.end_kernel_ticks=100011;b.session++;relay_control_encode(f.data,&b);
  assert(!esb_relay_control_receive(&f) && !tdma_enabled && !control_seen);
 } else if(!strcmp(argv[1],"profile")) {
  struct relay_pong_trailer tr={.epoch=7,.member_mask=7,.pending=true,
   .pending_epoch=8,.pending_total=0x95,.pending_slot_ticks=16,
   .pending_member_mask=7,.activation_units=20000};
  uint8_t bytes[17];assert(relay_trailer_encode(bytes,&tr)==17 && bytes[0]==0x5c);
  struct relay_pong_trailer decoded;assert(relay_trailer_decode_len(bytes,17,&decoded));
  esb_stage_from_trailer(&decoded,100000);
  assert(staged.total==relay_geometry_code(3,5) && staged.slot==2 && staged.activation==180000);
  bytes[10]=relay_geometry_code(3,3);bytes[11]=15;bytes[16]=relay_crc8(7,bytes,16);
  assert(relay_trailer_decode_len(bytes,17,&decoded));
  bytes[10]=relay_geometry_code(3,6);bytes[16]=relay_crc8(7,bytes,16);
  assert(!relay_trailer_decode_len(bytes,17,&decoded));
 } else assert(false);
 printf("PASS passive PONG %s\n",argv[1]);return 0;
}
'''
with tempfile.TemporaryDirectory(prefix="relay-presence-", dir="/tmp") as tmp:
    path = Path(tmp) / "presence.c"
    path.write_text(PREFIX + constants + '\n' + note + '\n' + stage + '\n' + function + '\n' + control + '\n' + lease + '\n' + grant_at + '\n' + grant + CASES)
    binary = Path(tmp) / "presence"
    subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
        "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function", "-Wno-unused-variable",
        "-g", "-O1", "-fsanitize=address,undefined", "-I", str(ROOT / "src/connection"),
        str(path), "-o", str(binary)], check=True)
    for case in sys.argv[1:] or ("reboot", "rejected", "valid", "grant", "pending", "invalid-tuple",
                                  "relay-recovery", "control", "control-demand", "profile"):
        subprocess.run([str(binary), case], check=True)
