#!/usr/bin/env python3
"""Compile the complete production relay worker with modeled radio/kernel leaves.

This exercises storage, expiry, bundle construction, assignment leases, child
capture, and presence-clock ownership. It does not simulate RADIO registers.
"""
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2]))
source = (ROOT / "src/connection/relay.c").read_text()
source = re.sub(r'^#include[^\n]*\n', '', source, flags=re.MULTILINE)

source += r'''
static void test_store(uint8_t child, const struct relay_rx_frame *f, uint8_t epoch, uint32_t expiry) {
 struct tdma_layout l; if(!tdma_layout_get(&l))return;
 relay_store(child,f,epoch,expiry,&l,(uint16_t)relay_frame_near(control_plan.frame,control_plan.preamble_ticks-RELAY_V3_B_PREAMBLE,expiry-l.frame_ticks),relay_generation);
}
'''

PREFIX = r'''
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define CONFIG_CONNECTION_RELAY 1
#include "relay.h"
#include "relay_radio.h"
#include "link_supervision.h"
#include "tdma.h"
#define LOG_LEVEL_INF 0
#define LOG_MODULE_REGISTER(...)
#define RELAY_THREAD_PRIORITY 3
#define K_THREAD_DEFINE(...)
#define K_SEM_DEFINE(n,a,b) int n
#define K_MSEC(x) (x)
#define K_USEC(x) (x)
static uint32_t now_ticks;
static int64_t now_ms;
static struct tdma_layout test_layout;
static enum link_state test_link = LINK_HEALTHY;
static bool layout_failed;
static bool radio_owned, radio_busy, capture_available;
static int hf_refs, coarse_waits_with_hf;
static uint32_t next_ping, hf_acquire_tick, busy_until;
static uint8_t captured_pipe_mask;
static struct relay_rx_frame incoming;
static unsigned irq_depth;
static void (*irq_hook)(void);
static unsigned irq_lock(void) { if(irq_hook){void (*hook)(void)=irq_hook;irq_hook=NULL;hook();} return irq_depth++; }
static void irq_unlock(unsigned k) { assert(irq_depth==k+1); irq_depth=k; }
static int64_t k_uptime_get(void) { return now_ms; }
static void k_sem_give(int *s) { *s = 1; }
static int k_sem_take(int *s, int delay) { (void)s; (void)delay; return 0; }
static void k_msleep(unsigned n) { now_ms += n; }
static void k_sem_reset(int *s) { *s=0; }
static void k_busy_wait(unsigned n) { (void)n; now_ticks++; }
static int printk(const char *f, ...) { (void)f; return 0; }
static enum link_state esb_link_state(void) { return test_link; }
static bool esb_link_wants_presence(void) { return false; }
static bool esb_receiver_relay_aware(void) { return true; }
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
uint8_t tdma_profile_get(void) { return test_layout.profile; }
bool tdma_layout_get(struct tdma_layout *l) { if(layout_failed) return false; *l = test_layout; return true; }
bool tdma_network_now(uint32_t *n) { *n = now_ticks; return true; }
bool tdma_wait_network_tick(uint32_t target) {
    if ((int32_t)(target-now_ticks) < 0) return false;
    if (target-now_ticks > 100 && hf_refs) coarse_waits_with_hf++;
    now_ticks = target; return true;
}
bool tdma_slot_data_window(uint8_t slot, uint32_t after, uint32_t *start) {
    uint32_t f = test_layout.frame_ticks;
    uint32_t t = after/f*f + slot*test_layout.slot_ticks;
    if ((int32_t)(t-after) <= 0) t += f;
    *start = t; return true;
}
bool tdma_slot_ping_window(uint8_t s, uint32_t after, uint32_t *start, uint16_t *length) {
    (void)s; (void)after; *start = next_ping; *length = 40; return true;
}
static int esb_relay_clock(bool hold) {
    hf_refs += hold ? 1 : -1; assert(hf_refs >= 0 && hf_refs <= 1);
    if (hold) hf_acquire_tick = now_ticks;
    return 0;
}
static bool esb_raw_rx_begin(void) {
    if (radio_busy || !hf_refs || now_ticks < busy_until) return false;
    assert(!radio_owned); radio_owned = true; return true;
}
static void esb_raw_rx_end(void) { assert(radio_owned); radio_owned = false; }
static uint8_t esb_get_radio_channel(void) { return 43; }
bool relay_radio_start(uint8_t ch, uint8_t mask, bool single) {
    assert(!single || irq_depth); assert(ch == 43 && radio_owned); captured_pipe_mask = mask; return true;
}
bool relay_radio_busy(void) { return false; }
bool relay_radio_take(struct relay_rx_frame *f) {
    *f = incoming; return capture_available;
}
bool relay_radio_poll(struct relay_rx_frame *f) { *f = incoming; return true; }
static bool esb_presence_frame(const struct relay_rx_frame *f) { (void)f; return true; }
bool relay_radio_stop(void) { assert(radio_owned); return true; }
bool relay_radio_addressed(void) { return false; }
bool relay_radio_ended(void) { return false; }
void relay_radio_peek(uint8_t *length, bool *crc_ok, uint8_t *rssi) { *length = 0; *crc_ok = false; *rssi = 0; }
static int radio_capture_init(void){return 0;}
static bool radio_capture_rx_end(uint32_t *t){*t=now_ticks;return true;}
static void radio_capture_set_enabled(bool e){(void)e;}
static void connection_relay_wake(void) {}
static void esb_print_link_status(void) {}
'''

CASES = r'''
static void reset(void) {
    relay_clock(false); relay_reset(); memset(&stats, 0, sizeof(stats));
    now_ticks = 130; now_ms = 1000; test_link = LINK_HEALTHY;
    radio_busy = radio_owned = false; capture_available = true;
    hf_refs = coarse_waits_with_hf = 0; busy_until=0;
    test_layout = (struct tdma_layout){.slot_index=0,.total_slots=2,.slot_ticks=64,.frame_ticks=128,.epoch=7};
    incoming = (struct relay_rx_frame){.length=17,.rssi=65,.pipe=2};
    incoming.data[0]=1; incoming.data[1]=1; incoming.data[16]=5;
}
static void adjacent_child(void) {
    reset(); test_store(1,&incoming,7,192);
    uint8_t out[72] = {1,0}; out[16]=3;
    size_t n=relay_build_bundle(out,17,sizeof(out),0);
    assert(n && "adjacent child was discarded before the own opportunity");
    struct relay_bundle_view view; assert(relay_bundle_decode(out,n,&view));
    assert(view.record_count==1 && view.records[0].frame[1]==1);
    test_store(1,&incoming,7,192); now_ticks=190;
    assert(!relay_records_pending());
    puts("PASS runtime: adjacent child survives until a usable forwarding opportunity");
}
static void no_fit_request(void) {
    reset();
    /* Valid 60-byte composite: quaternion, magnetometer, status, runtime, info. */
    incoming.length=60; incoming.data[0]=0xFE; incoming.data[1]=1; incoming.data[2]=5;
    const uint8_t types[]={1,4,3,5,0}, sizes[]={14,14,2,8,13};
    unsigned pos=3; for(unsigned i=0;i<5;i++){incoming.data[pos++]=types[i];pos+=sizes[i];}
    incoming.data[pos++]=9; assert(pos==60);
    test_store(1,&incoming,7,1000);
    uint8_t out[72]={1,0}; assert(!relay_build_bundle(out,17,72,0));
    assert(host_info_caps() & 4U);
    assert(relay_records_pending());
    now_ms+=2001; assert(!(host_info_caps() & 4U));
    puts("PASS runtime: zero-record overflow requests capacity through info traffic");
}
static void lease_and_capture(void) {
    reset();
    struct relay_pong_trailer tr={.epoch=7,.children_mask=2,.forward_slot=255,.member_mask=3};
    relay_on_own_trailer(&tr); assert(relay_head_active());
    relay_clock(true); now_ticks=140;
    assert(relay_head_window(&tr,&test_layout,relay_generation)); assert(stats.head_captures==1);
    assert(captured_pipe_mask==(1U<<2)); assert(relay_records_pending());
    test_layout.epoch++; assert(!relay_head_active()); assert(!relay_records_pending());
    test_layout.epoch--; now_ms+=2501; assert(!relay_head_active());
    relay_clock(false); relay_reset(); assert(!assignment.valid);
    puts("PASS runtime: capture uses the child pipe; epoch, lease and reset retire state");
}
static void presence_clock(void) {
    reset(); now_ticks=1000; next_ping=21000;
    relay_clock(true); relay_presence_window(&test_layout);
    assert(!coarse_waits_with_hf && "HFCLK was held through the long PING wait");
    assert(next_ping-hf_acquire_tick <= 66);
    assert(!hf_refs && !radio_owned);
    puts("PASS runtime: presence holds HFCLK only near its RX window");
}
static void late_own_tx(void) {
    reset(); now_ticks=140; busy_until=193;
    struct relay_pong_trailer tr={.epoch=7,.children_mask=2,.forward_slot=255,.member_mask=3};
    relay_clock(true); assert(relay_head_window(&tr,&test_layout,relay_generation));
    assert(stats.head_captures==1 && stats.head_busy==0);
    relay_clock(false);
    puts("PASS runtime: RX retries while a long own TX finishes before the child preamble");
}


static unsigned cache_seed=0x12345678;
static unsigned cache_random(void) {cache_seed^=cache_seed<<13;cache_seed^=cache_seed>>17;cache_seed^=cache_seed<<5;return cache_seed;}
static void cache_equivalence(void) {
 for(unsigned trial=0;trial<10000;trial++) {
  reset();test_layout.profile=1;
  control_helper=0;
  control_plan=(struct relay_control){.frame=100,.repair_count=1+trial%5,.members=0xffff};
  unsigned grant=cache_random()%control_plan.repair_count;
  control_plan.grants[grant]=(struct relay_grant){0,(uint16_t)cache_random()};
  for(unsigned age=0;age<3;age++) relay_control_missing(&control_plan, age)=cache_random();
  for(unsigned i=0;i<RELAY_CACHE_RECORDS;i++) {
   struct relay_record *r=&records[i];
   r->valid=cache_random()%3; r->child=1+cache_random()%15;
   r->capture_frame=100-cache_random()%5; r->length=17; r->rssi=20+cache_random()%50;
   memset(r->data,0,sizeof(r->data));r->data[0]=1;r->data[1]=r->child;r->data[16]=i;
  }
  uint64_t selected=((uint64_t)cache_random()<<32)|cache_random();
  repair_selected=selected;
  uint8_t expected[72]={0},actual[72]={0};
  size_t n=reference_build_repair(expected,RELAY_V3_REPAIR_FIRST_SLOT+grant);
  uint64_t expected_selected=repair_selected,expected_pending=repair_pending_mask;
  uint8_t pending[RELAY_MAX_TRACKERS];memcpy(pending,repair_pending,sizeof(pending));
  repair_selected=selected;
  size_t m=relay_build_repair(actual,RELAY_V3_REPAIR_FIRST_SLOT+grant);
  assert(n==m && !memcmp(expected,actual,n));
  assert(expected_selected==repair_selected && expected_pending==repair_pending_mask && !memcmp(pending,repair_pending,sizeof(pending)));
 }
 puts("PASS runtime: 10000 independent cache-builder wire and selection comparisons, all five nonempty pool sizes");
}

static void final_cache_gap(void) {
 for(unsigned repairs=1;repairs<=5;repairs++) {
  reset();test_layout.profile=RELAY_PROFILE_120HZ;
  struct relay_control c={.session=42,.epoch=7,.frame=0,.preamble_ticks=500+RELAY_V3_B_PREAMBLE,.members=7,.repair_count=repairs};
  relay_on_control(&c,0);
  uint32_t start=relay_frame_start(500,3)+RELAY_V3_SLOT_TICKS;
  now_ticks=start;incoming.data[16]=3;
  test_store(1,&incoming,7,start+RELAY_V3_FRAME_TICKS_MAX);
  c.frame=3;c.missing[0]=c.missing[2]=2;
  for(unsigned i=0;i<repairs;i++)c.grants[i]=(struct relay_grant){0,2};
  relay_on_control(&c,0);
  uint8_t bytes[72];struct relay_repair parsed;
  if(repairs>1)assert(!relay_build_repair(bytes,RELAY_V3_REPAIR_FIRST_SLOT+repairs-2));
  size_t n=relay_build_repair(bytes,RELAY_V3_REPAIR_FIRST_SLOT+repairs-1);
  if(repairs<=3)assert(relay_repair_decode(bytes,n,&parsed) && parsed.count==1 && parsed.records[0].capture_frame==3);
  else assert(!n);
  relay_repair_result(-EAGAIN);
  c.missing[1]=2;relay_on_control(&c,0);
  assert(!relay_build_repair(bytes,RELAY_V3_REPAIR_FIRST_SLOT+repairs-1));
 }
 puts("PASS runtime: actual final narrow slot permits an age-two gap for R1/R2/R3; earlier, age-one and R4/R5 guards stay fixed");
}

static void fast_cache(void) {
 reset(); test_layout.profile=RELAY_PROFILE_120HZ;
 test_layout.total_slots=RELAY_V3_TOTAL_SLOTS;test_layout.slot_ticks=RELAY_V3_SLOT_TICKS;
 test_layout.frame_ticks=RELAY_V3_FRAME_TICKS_MAX;
 struct relay_control c={.session=42,.epoch=7,.frame=0,.preamble_ticks=500+RELAY_V3_B_PREAMBLE,
  .primary_valid=2,.members=7,.repair_count=5};
 relay_on_control(&c,0);
 for(unsigned f=1;f<=3;f++) {
  uint32_t start=relay_frame_start(500,f)+RELAY_V3_SLOT_TICKS;
  now_ticks=start;incoming.data[16]=f;
  test_store(1,&incoming,7,start+test_layout.frame_ticks);
 }
 c.frame=3;c.preamble_ticks=relay_frame_start(500,3)+RELAY_V3_B_PREAMBLE;
 c.missing[0]=c.missing[1]=c.missing[2]=2;
 c.grants[0]=(struct relay_grant){.owner=0,.children=2};
 c.grants[1]=c.grants[2]=c.grants[0];
 relay_on_control(&c,0);
 uint8_t buf[72];size_t n=relay_build_repair(buf,RELAY_V3_REPAIR_FIRST_SLOT);
 struct relay_repair r;assert(relay_repair_decode(buf,n,&r) && r.count==3);
 assert(r.records[0].capture_frame==1 && r.records[2].capture_frame==3);
 unsigned forwarded_before=stats.forwarded;
 relay_repair_result(-EAGAIN);
 assert(stats.forwarded==forwarded_before);
 /* A refused submission may use a later grant in this same B frame. */
 n=relay_build_repair(buf,RELAY_V3_REPAIR_FIRST_SLOT+1);
 assert(relay_repair_decode(buf,n,&r) && r.count==3);
 assert(r.records[0].capture_frame==1 && r.records[2].capture_frame==3);
 relay_repair_result(0);
 assert(!relay_build_repair(buf,RELAY_V3_REPAIR_FIRST_SLOT+2));
 /* A later B can grant still-missing records again. */
 relay_on_control(&c,0);assert(relay_build_repair(buf,RELAY_V3_REPAIR_FIRST_SLOT));
 /* A gap must not let frame 3 obsolete frame 2 held by another helper. */
 for(unsigned i=0;i<RELAY_CACHE_RECORDS;i++)
  if(records[i].valid && records[i].capture_frame==2) records[i].valid=false;
 relay_on_control(&c,0);
 n=relay_build_repair(buf,RELAY_V3_REPAIR_FIRST_SLOT);
 assert(relay_repair_decode(buf,n,&r) && r.count==1 && r.records[0].capture_frame==1);
 relay_repair_result(0);
 assert(!relay_build_repair(buf,RELAY_V3_REPAIR_FIRST_SLOT+1));
 /* A later B confirms the two older frames: the fresh record is now safe. */
 c.missing[1]=c.missing[2]=0;relay_on_control(&c,0);
 n=relay_build_repair(buf,RELAY_V3_REPAIR_FIRST_SLOT);
 assert(relay_repair_decode(buf,n,&r) && r.count==1 && r.records[0].capture_frame==3);
 relay_repair_result(0);
 /* If an older frame never arrives, the unchanged age-2 limit still permits frame 3. */
 c.missing[1]=2;relay_on_control(&c,0);
 assert(!relay_build_repair(buf,RELAY_V3_REPAIR_FIRST_SLOT));
 c.frame=5;c.missing[0]=c.missing[1]=0;c.missing[2]=2;relay_on_control(&c,0);
 n=relay_build_repair(buf,RELAY_V3_REPAIR_FIRST_SLOT);
 assert(relay_repair_decode(buf,n,&r) && r.count==1 && r.records[0].capture_frame==3);
 c.missing[0]=c.missing[1]=c.missing[2]=0;relay_on_control(&c,0);
 assert(!relay_records_pending());
 now_ticks=c.preamble_ticks+2*RELAY_V3_FRAME_TICKS_MAX;
 struct relay_pong_trailer tr;struct tdma_layout l;
 assert(relay_assignment_get(&tr,&l,NULL));
 uint32_t target;uint8_t slot;assert(!relay_next_repair(&target,&slot));
 now_ticks=c.preamble_ticks+(RELAY_V3_CACHE_FRAMES+1)*RELAY_V3_FRAME_TICKS_MAX;
 assert(!relay_assignment_get(&tr,&l,NULL));
 c.session++;relay_on_control(&c,0);assert(!records[0].valid);
 puts("PASS runtime: three-frame cache, grant selection, failed queue retention and confirmation");
}

static void d10_b_ownership(void) {
 for (unsigned busy=0; busy<2; busy++) {
 reset(); test_layout.profile=1; test_layout.total_slots=RELAY_V3_TOTAL_SLOTS;
 test_layout.slot_ticks=RELAY_V3_SLOT_TICKS;test_layout.frame_ticks=relay_frame_start(0,1);
 struct relay_control c={.session=42,.epoch=7,.frame=0,.preamble_ticks=RELAY_V3_B_PREAMBLE,
  .members=0x7ff,.repair_count=5,.survey=1U<<10};
 relay_on_control(&c,0);
 struct relay_pong_trailer tr;struct tdma_layout l;
 uint32_t d10=test_layout.frame_ticks+10*RELAY_V3_SLOT_TICKS;
 now_ticks=d10-RELAY_RX_LEAD_TICKS;capture_available=false;
 radio_busy=busy;
 assert(relay_assignment_get(&tr,&l,NULL));
 relay_clock(true);assert(relay_head_window(&tr,&l,relay_generation));
 assert(!radio_owned);
 assert(now_ticks <= test_layout.frame_ticks+RELAY_V3_B_START-RELAY_RX_LEAD_TICKS);
 relay_clock(false);
 }
 puts("PASS runtime: missing or busy D10 yields at the unchanged B RX lead");
}

static void reset_before_insert(void) { relay_reset();struct relay_control c={.session=42,.epoch=7};relay_on_control(&c,0); }
static void failed_store(void) {
 for(unsigned i=0;i<RELAY_CACHE_RECORDS;i++) {
  memset(&records[i],0xa5,sizeof(records[i])); records[i].valid=true;
 }
 relay_reset();
 for(unsigned i=0;i<RELAY_CACHE_RECORDS;i++) assert(!records[i].valid);
 layout_failed=true;
 struct relay_rx_frame f={.length=17,.data={1,2}};
 test_store(2,&f,7,100);
 for(unsigned i=0;i<RELAY_CACHE_RECORDS;i++)assert(!records[i].valid);
 layout_failed=false;
 test_layout.profile=1;test_layout.frame_ticks=273;
 struct relay_control fresh={.epoch=7,.session=42};relay_on_control(&fresh,0);
 test_store(2,&f,7,1000);
 assert(records[0].valid && records[0].child==2 && records[0].epoch==7);
 assert(records[0].length==17 && records[0].expiry==1000 && records[0].rssi==f.rssi);
 assert(!memcmp(records[0].data,f.data,17));
 uint32_t generation=relay_generation;
 irq_hook=reset_before_insert;
 relay_store(2,&f,7,1000,&test_layout,3,generation);
 for(unsigned i=0;i<RELAY_CACHE_RECORDS;i++)assert(!records[i].valid);
 generation=relay_generation;
 struct relay_pong_trailer tr={0};struct tdma_layout l=test_layout;
 relay_reset();relay_on_control(&fresh,0);
 assert(!relay_head_window(&tr,&l,generation));
 relay_reset();relay_on_control(&fresh,0); /* same session and epoch */
 relay_store(2,&f,7,1000,&test_layout,3,generation);
 for(unsigned i=0;i<RELAY_CACHE_RECORDS;i++)assert(!records[i].valid);
 relay_reset();
}
static void learned_reports(void) {
 reset(); test_layout.profile=1; test_layout.total_slots=RELAY_V3_TOTAL_SLOTS;
 test_layout.slot_ticks=RELAY_V3_SLOT_TICKS;test_layout.frame_ticks=RELAY_V3_FRAME_TICKS_MAX;
 struct relay_control c={.session=42,.epoch=7,.frame=0,.preamble_ticks=RELAY_V3_B_PREAMBLE,
  .members=0x7ff,.repair_count=5,.survey=7};
 relay_on_control(&c,0);
 struct relay_pong_trailer tr;struct tdma_layout l;
 now_ticks=280;
 assert(relay_assignment_get(&tr,&l,NULL) && tr.children_mask==4); /* self and adjacent excluded */
 incoming.data[1]=2;incoming.pipe=3;
 relay_clock(true);
 assert(relay_head_window(&tr,&l,relay_generation));
 assert(peer_counts[2].expected==1 && peer_counts[2].heard==1 && peer_counts[2].rssi_x8==520);
 capture_available=false; now_ticks+=test_layout.frame_ticks;
 assert(relay_head_window(&tr,&l,relay_generation));
 assert(peer_counts[2].expected==2 && peer_counts[2].heard==1);
 relay_clock(false);
 c.frame=2;c.preamble_ticks=relay_frame_start(0,2)+RELAY_V3_B_PREAMBLE;
 c.grants[3]=(struct relay_grant){.owner=0,.children=0};
 for(unsigned slot=0;slot<5;slot++)if(slot!=3)c.grants[slot].owner=RELAY_OWNER_IDLE;
 relay_on_control(&c,0);
 /* Negative entries count, highest denominators first, eight entries only. */
 for(unsigned child=1;child<11;child++) {
  peer_counts[child].expected=child*10;peer_counts[child].heard=0;
 }
 now_ticks=c.preamble_ticks;
 assert(relay_records_pending());
 uint32_t target;uint8_t slot;
 assert(relay_next_repair(&target,&slot) && slot==RELAY_V3_REPAIR_FIRST_SLOT+3);
 uint8_t buf[72];size_t n=relay_build_repair(buf,slot);
 struct relay_peer_report report;
 assert(relay_peer_report_decode(buf,n,&report));
 assert(report.helper==0 && report.session==42 && report.epoch==7 && report.count==8);
 for(unsigned i=0;i<8;i++) {
  assert(report.entries[i].child==10-i && report.entries[i].expected==(10-i)*10 && !report.entries[i].heard);
  assert(peer_counts[10-i].expected==(10-i)*10);
 }
 assert(peer_counts[1].expected==10 && peer_counts[2].expected==20);
 unsigned forwarded_before=stats.forwarded;
 relay_repair_result(-EAGAIN);
 assert(peer_counts[10].expected==100);
 repair_attempted=0;
 n=relay_build_repair(buf,RELAY_V3_REPAIR_FIRST_SLOT+3);
 relay_repair_result(-ENOSPC);
 assert(peer_counts[10].expected==100);
 repair_attempted=0;
 n=relay_build_repair(buf,RELAY_V3_REPAIR_FIRST_SLOT+3);
 peer_count(10,false,0); peer_count(10,true,60);
 relay_repair_result(0);
 assert(peer_counts[10].expected==1 && peer_counts[10].heard==1);
 assert(!peer_counts[9].expected);
 assert(stats.forwarded==forwarded_before);
 assert(!relay_next_repair(&target,&slot));
 c.session++;relay_on_control(&c,0);assert(!peer_counts[1].expected);
 relay_reset();
 puts("PASS runtime: survey exclusions, capture counters, negative reports, top eight and report grant TX");
}

static void maintenance(void) {
 test_layout.profile=1;now_ticks=100;
 struct relay_control c={.frame=40,.preamble_ticks=1000,.repair_count=5,
  .maintenance_owner=2,.flags=RELAY_CONTROL_MAINTENANCE};
 relay_on_control(&c,1);
 uint32_t target;
 assert(!relay_next_maintenance(&target));
 c.maintenance_owner=1;c.flags=0;relay_on_control(&c,1);
 assert(!relay_next_maintenance(&target));
 c.flags=RELAY_CONTROL_MAINTENANCE;relay_on_control(&c,1);
 uint32_t start=c.preamble_ticks-RELAY_V3_B_PREAMBLE+RELAY_V3_REPAIR_START;
 assert(relay_next_maintenance(&target) && target==start+RELAY_V3_TX_OFFSET);
 now_ticks=target;assert(relay_take_maintenance());assert(!relay_next_maintenance(&target));
 c.frame++;relay_on_control(&c,1);now_ticks=start+3;
 assert(relay_take_maintenance());
 c.frame++;relay_on_control(&c,1);now_ticks=start+4;
 unsigned late=relay_maintenance_late;
 assert(!relay_next_maintenance(&target) && relay_maintenance_late==late+1);
 assert(!relay_next_maintenance(&target) && relay_maintenance_late==late+1);
 relay_reset();
}

static void control_accounting(void) {
 reset();test_layout.profile=1;
 unsigned before=repair_diag[1].grants;
 struct relay_control c={.frame=100,.session=42,.epoch=7,.repair_count=1};
 c.grants[0]=(struct relay_grant){.owner=0,.children=2};c.missing[0]=2;
 relay_on_control(&c,0);
 assert(repair_diag[1].grants==before);
 records[0]=(struct relay_record){.valid=true,.child=1,.epoch=7,.capture_frame=100,.length=17,.rssi=50};
 records[0].data[0]=1;records[0].data[1]=1;
 uint8_t buf[72];size_t n=relay_build_repair(buf,RELAY_V3_REPAIR_FIRST_SLOT);
 struct relay_repair decoded;
 assert(n && relay_repair_decode(buf,n,&decoded) && decoded.count==1);
 assert(decoded.records[0].capture_frame==100 && decoded.records[0].source.frame[1]==1);
 assert(repair_diag[1].grants==before);
 relay_control_serviced();assert(repair_diag[1].grants==before+1 && relay_control_done);
 relay_control_serviced();assert(repair_diag[1].grants==before+1);
 c.frame++;relay_on_control(&c,0);c.frame++;relay_on_control(&c,0);
 assert(repair_diag[1].grants==before+2);
 relay_control_serviced();assert(repair_diag[1].grants==before+3);
 c.frame++;relay_on_control(&c,0);relay_reset();relay_control_serviced();
 assert(repair_diag[1].grants==before+4);
 puts("PASS runtime: B accounting follows repair construction, runs once, and flushes on reset");
}
static void demand_cache(void) {
 reset();test_layout.profile=RELAY_PROFILE_120HZ;
 struct relay_control full={.session=42,.epoch=7,.frame=100,.members=0xffff,.on_demand=true};relay_on_control(&full,0);
 for(unsigned age=0;age<5;age++)for(unsigned ch=1;ch<16;ch++) {
 incoming.data[1]=ch;relay_store(ch,&incoming,7,0,&test_layout,100-age,relay_generation);
 }
 unsigned count=0;for(unsigned i=0;i<80;i++)count+=records[i].valid;assert(count==75);

 reset();test_layout.profile=RELAY_PROFILE_120HZ;
 struct relay_control c={.session=42,.epoch=7,.frame=100,.members=7,.repair_count=3,.on_demand=true};
 c.grants[0]=c.grants[1]=c.grants[2]=(struct relay_grant){.owner=0,.children=2};
 for(unsigned age=0;age<5;age++)relay_control_missing(&c, age)=2;
 relay_on_control(&c,0);
 for(unsigned age=0;age<5;age++) {
 struct relay_record *r=&records[64+age];*r=(struct relay_record){.valid=true,.child=1,.epoch=7,.capture_frame=100-age,.length=17};r->data[0]=1;r->data[1]=1;
 }
 uint8_t buf[72];struct relay_repair r;
 size_t n=relay_build_repair(buf,RELAY_V3_REPAIR_FIRST_SLOT);
 assert(n && relay_repair_decode(buf,n,&r) && r.count==3 && r.records[0].capture_frame==96 && r.records[2].capture_frame==98);
 relay_repair_result(-EAGAIN);
 n=relay_build_repair(buf,RELAY_V3_REPAIR_FIRST_SLOT+1);assert(n && relay_repair_decode(buf,n,&r) && r.records[0].capture_frame==96);
 relay_repair_result(0);
 c.missing[2]=relay_control_missing(&c, 3)=relay_control_missing(&c, 4)=0;relay_on_control(&c,0);
 n=relay_build_repair(buf,RELAY_V3_REPAIR_FIRST_SLOT);assert(n && relay_repair_decode(buf,n,&r) && r.count==2 && r.records[0].capture_frame==99);
 relay_repair_result(0);
 records[68].valid=records[67].valid=true;relay_control_missing(&c, 4)=2;
 c.frame=101;relay_on_control(&c,0);assert(!records[68].valid && records[67].valid);
 c.on_demand=false;relay_on_control(&c,0);assert(!records[67].valid && !records[66].valid);
 puts("PASS runtime: five-age high-index cache, failed-submit retry, age-five expiry and static three-age fence");
}
int main(int argc,char **argv) {
    demand_cache();
    cache_equivalence();
    if(argc==1 || !strcmp(argv[1],"accounting")) control_accounting();
    if(argc==1 || !strcmp(argv[1],"d10")) d10_b_ownership();
    if(argc==1 || !strcmp(argv[1],"learned")) {failed_store(); learned_reports();}
    if(argc==1 || !strcmp(argv[1],"maintenance")) maintenance();
    if(argc==1 || !strcmp(argv[1],"fast")) {fast_cache(); final_cache_gap();}
    if(argc==1 || !strcmp(argv[1],"adjacent")) adjacent_child();
    if(argc==1 || !strcmp(argv[1],"overflow")) no_fit_request();
    if(argc==1 || !strcmp(argv[1],"lease")) lease_and_capture();
    if(argc==1 || !strcmp(argv[1],"clock")) presence_clock();
    if(argc==1 || !strcmp(argv[1],"late-tx")) late_own_tx();
    assert(irq_depth==0);
    return 0;
}
'''

caps = "relay_info_caps()" if "uint8_t relay_info_caps(" in source else "RELAY_INFO_CAPS"
source += (HERE / "repair_reference.inc").read_text()
source += f"\nstatic uint8_t host_info_caps(void) {{ return {caps}; }}\n"
with tempfile.TemporaryDirectory(prefix="relay-runtime-", dir="/tmp") as tmp:
    path = Path(tmp) / "runtime.c"
    path.write_text(PREFIX + source + CASES)
    binary = Path(tmp) / "runtime"
    subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
        "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function", "-Wno-unused-variable",
        "-g", "-O1", "-fsanitize=address,undefined", "-I", str(ROOT / "src/connection"),
        str(path), "-o", str(binary)], check=True)
    import sys
    subprocess.run([str(binary), *sys.argv[1:]], check=True)
