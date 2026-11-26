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
note = extract_block(esb, r'^static void esb_note_delivery_locked\([^;{]*\)\s*\{')
function = extract_block(esb, r'^bool esb_presence_frame\([^;{]*\)\s*\{')
control = extract_block(esb, r'^bool esb_relay_control_receive\([^;{]*\)\s*\{')
profile_lost = extract_block(esb, r'^bool esb_relay_profile_lost\([^;{]*\)\s*\{')
lease = extract_block(esb, r'^static inline bool esb_lease_at\([^;{]*\)\s*\{')
grant_at = extract_block(esb, r'^bool esb_relay_granted_at\([^;{]*\)\s*\{')
grant = extract_block(esb, r'^bool esb_layout_granted\([^;{]*\)\s*\{')
stage = extract_block(esb, r'^static void esb_stage_from_trailer\([^;{]*\)\s*\{')
reset = '\n'.join(extract_block(esb, rf'^static void {name}\([^;{{]*\)\s*\{{')
                  for name in ('esb_presence_reset', 'esb_reset_receiver'))
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
static bool get_status(unsigned mask){(void)mask;return status_error;}
static unsigned irq_lock(void){return 0;}
static void irq_unlock(unsigned k){(void)k;}
static bool esb_initialized=true,channel_search,channel_wait_normal,server_time_synced=true,layout_stale;
static bool channel_confirmed=true,channel_legacy_peer,receiver_heard_seen,relayed_confirm_seen,ping_pending;
static bool control_seen, stream_profile;
static bool own_trailer_valid;
static uint8_t control_member_history, control_epoch;
static unsigned esb_conn_state=1,channel_redirect_pending,tracker_id=2,channel_found;
static unsigned presence_epoch=7,own_pong_length=23,receiver_heard_time,g_last_rx_raw_ticks=100000;
static unsigned g_last_sync_timestamp,receiver_restart_detections,recovery_requests;
static bool tdma_enabled=true;
static struct {uint32_t offset,last_local_ticks;int32_t skew_ppb;bool initialized;int8_t receiver_lag;int16_t ack_delay16;} sync_filter={.initialized=true};
static struct {unsigned pongs,sync_accepted,sync_rejected,layout_stale,layout_followed,relayed_confirms;} presence_stats;
static int atomic_get(unsigned *p){return *p;}
static void atomic_set(unsigned *p,unsigned n){*p=n;}
static uint32_t sys_get_be32(const uint8_t *p){return (uint32_t)p[0]<<24|(uint32_t)p[1]<<16|(uint32_t)p[2]<<8|p[3];}
static uint8_t crc8_ccitt(uint8_t v,const uint8_t *p,unsigned n){return relay_crc8(v,p,n);}
static uint64_t kernel_ticks_extend32(uint32_t v){return v;}
static uint64_t net_ticks_from_kernel64(uint64_t v){return v;}
static uint32_t now_ms=3000;
static uint32_t k_uptime_get_32(void){return now_ms;}
static uint32_t k_uptime_get(void){return now_ms;}
static int64_t net_tick_difference(uint32_t a,uint32_t b){return (int32_t)(a-b);}
static uint32_t relayed_confirm_time;
struct esb_sync_sample{uint32_t local_ticks,offset;};

static uint8_t tdma_get_config_epoch(void){return presence_epoch;}
static bool tdma_is_enabled(void){return tdma_enabled;}
static bool tdma_update_config(uint8_t slot,uint8_t total,uint8_t width,uint8_t epoch){
 if(slot>=total || total>16 || width<16)return false;
 presence_epoch=epoch;return true;
}
static void esb_clear_time_sync_state(void){
 server_time_synced=false;sync_filter.initialized=false;
 control_seen=false;control_member_history=0;relayed_confirm_seen=false;
}
static void esb_wom_reset(void){}
static void relay_reset(void){}
static void relay_on_control(const struct relay_control *c,uint8_t h){(void)c;(void)h;}
static void tdma_set_enabled(bool on){tdma_enabled=on;}
static void tdma_session_reset(void){
 stream_profile=false;
 tdma_enabled=false;relay_direct_anchor.valid=false;relay_timeline_clear(&relay_timeline_active);
}
static void connection_request_ping_resync(void){recovery_requests++;}
static uint32_t layout_confirm_time;
static bool layout_confirm_seen;
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
static bool tdma_layout_get(struct tdma_layout *l){*l=(struct tdma_layout){1,presence_epoch,2,16};return tdma_enabled;}
static void tdma_stream_control(const struct relay_control *c){
 relay_geometry_make_control(&geometry,c->direct_count,c->repair_count,c->slot_ticks,c->tps_max,c->control_repeat_mode);
 relay_runtime_anchor_set(&relay_direct_anchor,c->frame,c->preamble_ticks,c->direct_count,c->repair_count,relay_control_next_repairs(c),relay_control_next2_repairs(c),c->slot_ticks,c->tps_max,c->control_repeat_mode);
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
timeout = extract_block(esb, r'^static void esb_ping_timeout\([^;{]*\)\s*\{')
inputs = extract_block(esb, r'^static struct link_inputs esb_link_inputs_locked\([^;{]*\)\s*\{')
PREFIX += r'''
#define LOG_WRN(...) ((void)0)
#define TX_ERROR_THRESHOLD 300
static uint32_t own_pong_time;
static bool own_pong_seen,ping_failed;
static int64_t ping_send_time;
static unsigned get_ping_interval_ms(void){return 997;}
static uint32_t esb_age_ms(uint32_t now,uint32_t then,bool seen){return seen?now-then:LINK_AGE_NEVER;}
'''
CASES = r'''
static struct relay_rx_frame pong_with(uint32_t t2,uint8_t epoch,uint16_t members,bool pending) {
 struct relay_rx_frame f={.pipe=2,.end_kernel_ticks=100000};
 f.data[0]=ESB_PONG_TYPE;f.data[1]=1;
 f.data[3]=t2>>24;f.data[4]=t2>>16;f.data[5]=t2>>8;f.data[6]=t2;
 f.data[7]=ESB_PONG_FLAG_NORMAL;f.data[9]=3;f.data[10]=16;f.data[11]=epoch;
 f.data[12]=relay_crc8(7,f.data,12);
 struct relay_pong_trailer tr={.epoch=epoch,.member_mask=members,.pending=pending,
  .pending_epoch=9,.pending_total=4,.pending_slot_ticks=16,.pending_member_mask=0xF,.activation_units=100};
 f.length=13+relay_trailer_encode(f.data+13,&tr);return f;
}
static struct relay_rx_frame pong(uint32_t t2,uint8_t epoch){return pong_with(t2,epoch,7,false);}
static void reset_history_case(void) {
 control_seen=false;control_member_history=0;control_epoch=0;
 receiver_heard_seen=relayed_confirm_seen=layout_confirm_seen=false;
 relay_direct_anchor.valid=false;relay_timeline_clear(&relay_timeline_active);
 sync_filter.initialized=true;sync_filter.offset=sync_filter.last_local_ticks=0;sync_filter.skew_ppb=0;
 server_time_synced=tdma_enabled=stream_profile=true;
 channel_search=channel_wait_normal=layout_stale=false;presence_epoch=7;now_ms=3000;
 ping_failures=0;ping_pending=status_error=shutdown_requested=false;
 esb_conn_state=ESB_ST_PAIRED;presence_stats.relayed_confirms=0;
 relay_geometry_make(&geometry,3,5,16,120);
}
static struct relay_control history_control(uint32_t frame,uint32_t tick,bool demand) {
 struct relay_control c={.epoch=7,.direct_count=3,.repair_count=5,.slot_ticks=16,.tps_max=120,
  .session=42,.frame=frame,.preamble_ticks=tick,.members=7,.expected=7,.profile=1,
  .maintenance_owner=RELAY_OWNER_IDLE,.on_demand=demand};
 for(unsigned age=0;age<RELAY_CACHE_MAX_FRAMES;age++) relay_control_missing(&c,age)=7;
 for(unsigned i=0;i<RELAY_REPAIR_MAX;i++) c.grants[i].owner=RELAY_OWNER_IDLE;
 relay_control_set_next_repairs(&c,5);relay_control_set_next2_repairs(&c,5);
 return c;
}
static bool receive_history_control(const struct relay_control *c) {
 struct relay_rx_frame frame={.pipe=1,.length=RELAY_CONTROL_LEN,
  .end_kernel_ticks=c->preamble_ticks+RELAY_CONTROL_AIR_TICKS};
 assert(relay_control_encode(frame.data,c)==RELAY_CONTROL_LEN);
 return esb_relay_control_receive(&frame);
}
int main(int argc,char **argv) {
 assert(argc==2);
 uint32_t ping_end = 100000;
 struct relay_rx_frame f;
 if(!strcmp(argv[1],"reboot")) {
  f=pong(1000,8);assert(esb_presence_frame(&f, &ping_end));
  assert(channel_wait_normal && !server_time_synced && !tdma_enabled);
  assert(!channel_confirmed && recovery_requests==1 && receiver_restart_detections==1);
  assert(presence_epoch==7);
 } else if(!strcmp(argv[1],"rejected")) {
  f=pong_with(100010,7,7,true);assert(esb_presence_frame(&f, &ping_end));
  assert(!presence_stats.sync_accepted && presence_epoch==7);
  assert(!layout_confirm_seen && !staged.calls && !staged.clears); /* no clock agreement */
  f=pong(100010,8);assert(esb_presence_frame(&f, &ping_end));
  assert(!presence_stats.sync_accepted && layout_stale && presence_epoch==7);
 } else if(!strcmp(argv[1],"valid")) {
  /* Own filtered time lags the receiver by 3 ticks. Radio ACK END
   * lags it by 9. The measured peer sample agrees after conversion. */
  sync_filter.receiver_lag=3;sync_filter.ack_delay16=9*16;
  f=pong(100003,8);f.end_kernel_ticks = ping_end+21;
  assert(esb_presence_frame(&f, NULL) && !presence_stats.sync_accepted);
  assert(esb_presence_frame(&f, &ping_end));
  assert(sync_filter.offset==0 && sync_filter.last_local_ticks==100000);
  assert(presence_stats.sync_accepted==1 && !layout_stale && presence_epoch==8);
  assert(layout_confirm_seen && layout_confirm_time==3000 && staged.clears==1);
 } else if(!strcmp(argv[1],"grant")) {
  f=pong_with(100000,7,3,false);assert(esb_presence_frame(&f, &ping_end)); /* lists 0,1 only */
  assert(!layout_confirm_seen);
  /* Epochs are unordered: a different epoch listing us is followed and confirms. */
  f=pong_with(100000,6,7,false);assert(esb_presence_frame(&f, &ping_end));
  assert(layout_confirm_seen && presence_epoch==6);layout_confirm_seen=false;
  f=pong_with(100000,6,7,false);assert(esb_presence_frame(&f, &ping_end));
  assert(layout_confirm_seen && layout_confirm_time==3000);
 } else if(!strcmp(argv[1],"pending")) {
  f=pong_with(100000,7,7,true);assert(esb_presence_frame(&f, &ping_end));
  assert(staged.calls==1 && staged.slot==2 && staged.total==4 && staged.width==16 && staged.epoch==9);
  assert(staged.activation==100000+100*RELAY_ACTIVATION_UNIT_TICKS);
  assert(layout_confirm_seen && presence_epoch==7);
  layout_confirm_seen=false;f=pong_with(100000,7,7,true);
  f.data[13+12]^=1; /* corrupt a pending field: the trailer CRC rejects it all */
  assert(esb_presence_frame(&f, &ping_end));assert(staged.calls==1 && !layout_confirm_seen);
 } else if(!strcmp(argv[1],"invalid-tuple")) {
  f=pong(100000,8);f.data[10]=0;f.data[12]=relay_crc8(7,f.data,12);
  assert(esb_presence_frame(&f, &ping_end));assert(layout_stale && presence_epoch==7);
 } else if(!strcmp(argv[1],"relay-recovery")) {
  esb_conn_state=ESB_ST_RECOVERING;status_error=true;ping_failures=300;
  connection_error_start_time=100;shutdown_requested=true;
  esb_note_delivery_locked(3000);
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
  /* The control frame sets the offset directly: preamble = END - control frame airtime. */
  assert(esb_relay_control_receive(&f) && control_seen && sync_filter.last_local_ticks==100000);
  assert(layout_confirm_seen && receiver_heard_seen && relayed_confirm_seen);
  uint32_t cutoff=relay_direct_anchor.third_start+3U*16U;
  current_tick=cutoff-1;assert(esb_layout_granted() && esb_relay_granted_at(current_tick));
  current_tick=cutoff;assert(!esb_layout_granted() && !esb_relay_granted_at(current_tick));
  relay_runtime_anchor_set(&relay_direct_anchor,10,0xfffffff0U,3,5,0,2,16,120,0);
  cutoff=relay_direct_anchor.third_start+3U*16U;
  assert(esb_lease_at(cutoff-1));assert(!esb_lease_at(cutoff));

  assert(!esb_relay_control_receive(&f)); /* duplicate/stale frame */
  b.frame++; relay_control_encode(f.data,&b); f.end_kernel_ticks+=10;
  assert(!esb_relay_control_receive(&f) && sync_filter.last_local_ticks==100000); /* late delivery */
  f.end_kernel_ticks=100011;b.session++;relay_control_encode(f.data,&b);
  assert(!esb_relay_control_receive(&f) && !tdma_enabled && !control_seen);
 } else if(!strcmp(argv[1],"history-health")) {
  stream_profile=true;relay_geometry_make(&geometry,3,5,16,120);
  struct relay_control b={.epoch=7,.repair_count=5,.session=42,.frame=10,
   .preamble_ticks=100000,.members=7,.expected=7,.profile=1,.maintenance_owner=255};
  for(unsigned i=0;i<5;i++) b.grants[i].owner=RELAY_OWNER_IDLE;
  for(unsigned n=0;n<50000;n++) {
   now_ms=3000+n*8;b.frame=10+n;b.preamble_ticks=100000+n*273;
   b.missing[0]=b.missing[1]=7;b.missing[2]=3;
   f=(struct relay_rx_frame){.pipe=1,.length=72,.end_kernel_ticks=b.preamble_ticks+11};
   relay_control_encode(f.data,&b);assert(esb_relay_control_receive(&f));
   if(n<2) assert(!relayed_confirm_seen); /* no invented pre-session receipt */
   if(n%125==0){ping_pending=true;ping_send_time=now_ms-998;esb_ping_timeout(now_ms);}
   assert(ping_failures<300 && esb_conn_state==ESB_ST_PAIRED && !status_error);
  }
  assert(relayed_confirm_time==now_ms && !ping_failures);
  /* esb_thread() raised the error after a recovery had cleared the failures */
  status_error=true;b.frame++;b.preamble_ticks+=273;now_ms+=8;
  f=(struct relay_rx_frame){.pipe=1,.length=72,.end_kernel_ticks=b.preamble_ticks+11};
  relay_control_encode(f.data,&b);assert(esb_relay_control_receive(&f) && !status_error);
 } else if(!strcmp(argv[1],"history-gaps") || !strcmp(argv[1],"history-gaps-demand")) {
  bool demand=!strcmp(argv[1],"history-gaps-demand");
  unsigned depth=demand?5:3,attempts=0,repairs=0;
  struct {uint32_t frame;bool valid;} receipt[RELAY_CACHE_MAX_FRAMES]={0};
  struct relay_control previous={0};
  uint32_t start=100000;
  reset_history_case();
  for(unsigned n=0;n<45000;n++) {
   uint32_t frame=10+n;
   current_tick=start+2U*16U+2U; /* child 2's direct slot */
   if(n) assert(relay_runtime_direct_granted(&relay_direct_anchor,current_tick));
   struct relay_control c=history_control(frame,start+3U*16U+2U,demand);
   c.primary_valid=4;c.helpers[2]=0xf0; /* helper 0 watches nonadjacent child 2 */
   c.grants[2]=(struct relay_grant){.owner=0,.children=4};
   if(n%120==0) {
    c.flags|=RELAY_CONTROL_MAINTENANCE;c.maintenance_owner=2;
    c.grants[0].owner=c.grants[1].owner=RELAY_OWNER_MAINTENANCE;
   }
   for(unsigned age=0;age<depth;age++) {
    unsigned index=(frame-age)%depth;
    if(receipt[index].valid && receipt[index].frame==frame-age) relay_control_missing(&c,age)=3;
   }
   now_ms=(uint64_t)(c.preamble_ticks+RELAY_CONTROL_AIR_TICKS)*1000U/32768U;
   if(n%3==0) { /* helper hears every control; child misses two of every three */
    assert(receive_history_control(&c));
    if(n%120==0) {
     assert(c.maintenance_owner==2 && c.grants[0].owner==RELAY_OWNER_MAINTENANCE);
     ping_pending=true;ping_send_time=now_ms-998;esb_ping_timeout(now_ms);attempts++;
    }
    assert(esb_conn_state==ESB_ST_PAIRED && !status_error && ping_failures<300);
   }
   if(n) {
    assert(previous.frame+1U==frame && (previous.primary_valid&4));
    assert((previous.helpers[2]&15U)==0 && relay_can_overhear(previous.members,0,2));
    assert(c.grants[2].owner==0 && c.grants[2].children==4 && (c.missing[0]&4));
    /* A granted helper repair is received after this control, never in its age 0 mask. */
    receipt[frame%depth].frame=frame;receipt[frame%depth].valid=true;repairs++;
   }
   previous=c;
   start+=relay_runtime_period(frame,3,5,16,120,0);
  }
  assert(attempts>300 && repairs>300 && !ping_failures && !status_error);
  assert(presence_stats.relayed_confirms==1); /* no full recovery on every receipt */
  now_ms+=LINK_RELAYED_TIMEOUT_MS+1;ping_pending=true;ping_send_time=now_ms-998;
  esb_ping_timeout(now_ms);assert(ping_failures==1); /* old repairs eventually expire */
 } else if(!strcmp(argv[1],"history-bounds")) {
  for(unsigned demand=0;demand<2;demand++) for(unsigned wrap=0;wrap<2;wrap++)
   for(unsigned advance=1;advance<=8;advance++) for(unsigned age=1;age<5;age++) {
    reset_history_case();
    uint32_t base=wrap?UINT32_MAX-1U:100U, tick=wrap?UINT32_MAX-100U:100000U;
    struct relay_control c=history_control(base,tick,demand);
    assert(receive_history_control(&c) && !relayed_confirm_seen);
    c.frame+=advance;c.preamble_ticks+=273U*advance;now_ms+=10;
    relay_control_missing(&c,age)=3;
    assert(receive_history_control(&c));
    unsigned depth=demand?5:3;
    bool covered=age<depth && age<=advance && advance-age<=2U;
    assert(relayed_confirm_seen==covered);
    assert((esb_link_inputs_locked(now_ms).relayed_age_ms!=LINK_AGE_NEVER)==covered);
   }
 } else if(!strcmp(argv[1],"history-fences")) {
  reset_history_case();
  struct relay_control c=history_control(10,100000,true);
  c.missing[0]=3;assert(receive_history_control(&c) && relayed_confirm_seen);
  presence_epoch=8; /* layout activation fences even before its first control */
  assert(esb_link_inputs_locked(now_ms).relayed_age_ms==LINK_AGE_NEVER);
  c.epoch=8;c.frame++;c.preamble_ticks+=273;c.missing[0]=7;c.missing[1]=3;now_ms++;
  assert(receive_history_control(&c) && !relayed_confirm_seen);
  c.frame++;c.preamble_ticks+=273;now_ms++;
  assert(receive_history_control(&c) && relayed_confirm_seen);
  c.session++;c.frame++;c.preamble_ticks+=273;now_ms++;
  assert(!receive_history_control(&c) && !tdma_enabled && !relayed_confirm_seen);
  assert(esb_link_inputs_locked(now_ms).relayed_age_ms==LINK_AGE_NEVER);
  tdma_enabled=stream_profile=server_time_synced=sync_filter.initialized=true;
  channel_wait_normal=false; /* Fresh channel and time proof on the new session. */
  assert(receive_history_control(&c) && !relayed_confirm_seen);
  c.frame++;c.preamble_ticks+=273;now_ms++;
  assert(receive_history_control(&c) && relayed_confirm_seen);

  reset_history_case();c=history_control(10,100000,true);c.missing[0]=3;
  assert(receive_history_control(&c) && relayed_confirm_seen);
  relay_direct_anchor.valid=false;relay_timeline_clear(&relay_timeline_active); /* profile re-entry */
  assert(esb_link_inputs_locked(now_ms).relayed_age_ms==LINK_AGE_NEVER);
  c.frame++;c.preamble_ticks+=273;c.missing[0]=7;c.missing[1]=3;now_ms++;
  assert(receive_history_control(&c) && !relayed_confirm_seen);

  reset_history_case();c=history_control(10,100000,true);c.missing[0]=3;
  assert(receive_history_control(&c) && relayed_confirm_seen);
  now_ms+=RELAY_LAYOUT_GRANT_MS;
  assert(esb_relay_profile_lost() && !relayed_confirm_seen);
  assert(esb_link_inputs_locked(now_ms).relayed_age_ms==LINK_AGE_NEVER);
  channel_wait_normal=false;c.frame++;c.preamble_ticks+=273;c.missing[0]=7;c.missing[1]=3;
  assert(receive_history_control(&c) && !relayed_confirm_seen);

  reset_history_case();c=history_control(10,100000,true);c.missing[0]=3;
  assert(receive_history_control(&c) && relayed_confirm_seen);
  f=(struct relay_rx_frame){.pipe=1,.length=RELAY_CONTROL_LEN,
   .end_kernel_ticks=c.preamble_ticks+RELAY_CONTROL_AIR_TICKS};
  assert(relay_control_encode(f.data,&c)==RELAY_CONTROL_LEN);f.data[71]^=1;
  assert(!esb_relay_control_receive(&f) && tdma_enabled && relayed_confirm_seen);
  tdma_enabled=true;c.frame++;c.preamble_ticks+=273;c.missing[0]=7;c.missing[1]=3;now_ms++;
  assert(receive_history_control(&c) && relayed_confirm_seen);

  reset_history_case();c=history_control(10,100000,true);c.expected=3;c.missing[0]=3;
  assert(receive_history_control(&c) && !relayed_confirm_seen); /* not expected, not a receipt */

  reset_history_case();c=history_control(10,100000,true);c.missing[0]=3;
  assert(receive_history_control(&c) && relayed_confirm_seen);
  f=pong_with(100000,8,3,false); /* a new layout does not assign this tracker */
  assert(esb_presence_frame(&f, &ping_end) && layout_stale && !relayed_confirm_seen);
  layout_stale=false;c.frame++;c.preamble_ticks+=273;c.missing[0]=7;c.missing[1]=3;now_ms++;
  assert(receive_history_control(&c) && !relayed_confirm_seen);

  reset_history_case();c=history_control(10,100000,true);
  assert(receive_history_control(&c) && !relayed_confirm_seen);
  c.missing[0]=3;assert(!receive_history_control(&c) && !relayed_confirm_seen); /* duplicate */
  c.frame++;c.preamble_ticks+=273;c.missing[0]=7;
  f=(struct relay_rx_frame){.pipe=1,.length=RELAY_CONTROL_LEN,
   .end_kernel_ticks=c.preamble_ticks+RELAY_CONTROL_AIR_TICKS+10};
  assert(relay_control_encode(f.data,&c)==RELAY_CONTROL_LEN);
  assert(!esb_relay_control_receive(&f) && !relayed_confirm_seen); /* rejected clock sample */
  c.frame+=2;c.preamble_ticks+=2*273;c.missing[1]=3;now_ms++;
  assert(receive_history_control(&c) && relayed_confirm_seen); /* original grant still applies */

  reset_history_case();now_ms=UINT32_MAX-500U;c=history_control(10,100000,true);c.missing[0]=3;
  assert(receive_history_control(&c) && relayed_confirm_seen);
  now_ms=499;assert(esb_link_inputs_locked(now_ms).relayed_age_ms==1000);
  now_ms=2000;ping_pending=true;ping_send_time=now_ms-998;
  esb_ping_timeout(now_ms);assert(ping_failures==1); /* expiry across uptime wrap */
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
    path.write_text(PREFIX + constants + '\n' + note + '\n' + stage + '\n' + reset + '\n' + function + '\n' + control + '\n' + profile_lost + '\n' + inputs + '\n' + timeout + '\n' + lease + '\n' + grant_at + '\n' + grant + CASES)
    binary = Path(tmp) / "presence"
    subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
        "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function", "-Wno-unused-variable",
        "-g", "-O1", "-fsanitize=address,undefined", "-I", str(ROOT / "src/connection"),
        str(path), "-o", str(binary)], check=True)
    for case in sys.argv[1:] or ("reboot", "rejected", "valid", "grant", "pending", "invalid-tuple",
                                  "history-health", "history-gaps", "history-gaps-demand", "history-bounds",
                                  "history-fences", "relay-recovery", "control", "control-demand", "profile"):
        subprocess.run([str(binary), case], check=True)
