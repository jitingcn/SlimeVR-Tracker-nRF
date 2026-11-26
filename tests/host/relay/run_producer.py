#!/usr/bin/env python3
"""Actual tracker encoder/producer with relay + USB HID enabled.

Reuse the real connection producer and exercise its grant pacing with the real
TDMA target selector and control-announced frame timeline.
"""
import importlib.util
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

HERE=Path(__file__).resolve().parent
ROOT=Path(os.environ.get("SOURCE_ROOT",HERE.parents[2]))
fixture_path=HERE.parent/"tracker_events/transport.py"
spec=importlib.util.spec_from_file_location("relay_producer_transport",fixture_path)
transport=importlib.util.module_from_spec(spec);spec.loader.exec_module(transport)
transport.ROOT=ROOT
prefix='''#define MIN(a,b) ((a)<(b)?(a):(b))
#define CONFIG_RELAY_RUNTIME_LAYOUT 1
#include "connection/relay_protocol.h"
static uint8_t relay_info_caps(void) { return RELAY_INFO_CAPS; }
static struct relay_geometry tdma_active_geometry = {11,4,16,120,4096,15,0};
const struct relay_geometry *relay_geometry_active = &tdma_active_geometry;
struct relay_timeline relay_timeline_active;
struct relay_runtime_anchor relay_direct_anchor;
'''
legacy_tests=transport.TESTS.replace('int main(void)','static int unused_direct_suite(void)')
tdma=(ROOT/"src/connection/tdma.c").read_text()
selectors="\n".join(re.findall(
    r"^#define (?:TDMA_UNPACK_SLOT|TDMA_LEN\w+_RESERVE_TICKS|TDMA_SLOT_TARGET_OFFSET)[^\n]*",
    tdma,re.MULTILINE))
selectors+=r'''
static uint32_t tdma_anchor_frame, tdma_anchor_start, tdma_profile_origin;
static uint32_t tdma_last_admitted_pack, tdma_last_admitted_frame, tdma_stream_slots;
static atomic_t tdma_cfg_pack;
static uint32_t network_tick;
struct esb_network_clock { uint32_t receiver_ticks; };
static bool esb_get_network_clock(struct esb_network_clock *clock) {
    clock->receiver_ticks=network_tick;
    return true;
}
'''
selectors+="\n"+"\n".join(transport.function(tdma,name) for name in (
    "tdma_stream_anchor","tdma_stream_control","tdma_reserve_ticks","tdma_stream_at"))
selectors+="\n#define tdma_stream_next fixture_stream_next\n"+transport.function(tdma,"tdma_stream_next")
selectors+="\n#undef tdma_stream_next\n"
CASES=r'''
static void assert_hid_info_zero(void) {
 unsigned infos=0;
 for(unsigned i=0;i<mirrors;i++) if(mirrored[i][0]==0){infos++;assert(mirrored[i][7]==0);}
 assert(infos);
}

static uint32_t network_origin;
static unsigned controls;
static uint32_t frame_start(unsigned frame) {
 return network_origin+(uint64_t)frame*32768U/relay_geometry_current()->tps;
}
static void announce(unsigned frame) {
 const uint8_t repairs=4;
 const struct relay_control control={
  .direct_count=11,.slot_ticks=16,.tps_max=relay_geometry_current()->tps,
  .frame=frame,.repair_count=repairs,.preamble_ticks=frame_start(frame)+RELAY_CONTROL_PREAMBLE,
  .flags=(repairs << RELAY_CONTROL_NEXT_R_SHIFT)|(repairs << RELAY_CONTROL_NEXT2_R_SHIFT),
 };
 tdma_stream_control(&control);
}
static void wait_for_network(uint32_t target) {
 if ((int32_t)(target-network_tick)>0) network_tick=target;
}
static bool service_control(uint32_t target) {
 assert(target==frame_start(controls)+RELAY_CONTROL_START-RELAY_RX_LEAD_TICKS);
 wait_for_network(target+RELAY_RX_LEAD_TICKS+RELAY_CONTROL_AIR_TICKS);
 announce(controls++);
 return true;
}
static void admit_source(size_t len) {
 uint32_t target=frame_start(controls)+stream_slot*RELAY_SLOT_TICKS+RELAY_TX_OFFSET;
 assert(len<=relay_direct_payload_max(1,stream_slot));
 assert((int32_t)(network_tick-(target-4U))>=0);
 wait_for_network(target+RELAY_AIR_TICKS(len));
 tdma_last_admitted_pack=tdma_cfg_pack;
 tdma_last_admitted_frame=controls;
 tdma_stream_slots=BIT(stream_slot);
}
static void start_stream(unsigned cap) {
 assert(relay_geometry_make(&tdma_active_geometry,11,4,16,cap));
 stream_enabled=true; stream_slot=RELAY_DIRECT_SLOTS-1;
 stream_next_hook=fixture_stream_next;
 stream_control_hook=service_control;
 stream_wait_hook=wait_for_network;
 stream_send_hook=admit_source;
 tdma_cfg_pack=stream_slot; tdma_stream_slots=0; controls=0;
 network_origin=network_tick=UINT32_MAX-500;
 relay_timeline_clear(&relay_timeline_active);
 announce(0);
}
static void test_private_rate(unsigned requested, unsigned cap, unsigned expected) {
 reset(); event_due=false; test_enabled=true; sensor_ids_set=true; target_tps=requested;
 start_stream(cap);
 unsigned types[6]={0};
 for (unsigned i=0;controls<2U*cap;i++) {
  assert(i<4U*cap);
  /* Deliberately offset and slower local clock; only the network grid paces DATA. */
  set_clock(10000003ULL+(uint32_t)(network_tick-network_origin)*30ULL);
  publish_a(); publish_m();
  unsigned before=sends;
  iteration();
  if (sends==before) continue;
  assert(sends==before+1 && radio_rotations==sends);
  for (unsigned type=0;type<ARRAY_SIZE(types);type++) types[type]+=!!body_for(type);
  const uint8_t *info=body_for(SUB_PACKET_INFO);
  if (info) assert(!!(info[5]&RELAY_CAP_NATIVE_TEST)==(expected<cap));
 }
 assert(sends==2U*expected && radio_rotations==sends);
 assert(types[SUB_PACKET_QUAT_MAG] && types[SUB_PACKET_INFO] && types[SUB_PACKET_STATUS] && types[SUB_PACKET_RUNTIME]);
 assert(types[SUB_PACKET_QUAT_ACCEL]+types[SUB_PACKET_COMPACT_QUAT]>0);
}
static void test_private_stationary_metadata(void) {
 reset(); event_due=false; sensor_ids_set=true; start_stream(120);
 last_mag_time=last_info_time=last_status_time=last_runtime_time=0;
 unsigned infos=0,mags=0,statuses=0,runtimes=0;
 for (unsigned i=0;controls<240;i++) {
  assert(i<480);
  set_clock(10000000ULL+(uint32_t)(network_tick-network_origin)*1000000ULL/32768U);
  float q[]={1,0,0,0}, a[]={0,0,0};
  connection_update_sensor_data(q,a); publish_m();
  unsigned before=sends;
  iteration();
  if (sends==before) continue;
  if (body_for(SUB_PACKET_INFO)) {
   infos++;
   assert(body_for(SUB_PACKET_COMPACT_QUAT) && sent_len==RELAY_LAST_DIRECT_MAX);
  }
  mags+=!!body_for(SUB_PACKET_QUAT_MAG);
  statuses+=!!body_for(SUB_PACKET_STATUS);
  runtimes+=!!body_for(SUB_PACKET_RUNTIME);
 }
 assert(infos>=19 && mags>=19 && statuses>=2 && runtimes>=2);
 assert(sends==240 && radio_rotations==sends);
}
static void test_private_fallback(void) {
 reset(); event_due=false; test_enabled=true; sensor_ids_set=true; target_tps=0;
 next_ping_deadline_ms=1000000;
 publish_a(); iteration();
 assert(sends==1 && test_rate_schedule.next_due_us==10010000);
 start_stream(120);
 stream_granted=false;
 connection_update_native_test_request(true,now_ms);
 assert(!relay_native_test_requested);
 stream_granted=true;
 send_error=-EAGAIN; iteration();
 assert(sends==2 && radio_rotations==1 && relay_native_test_requested);
 /* Local expiry can retire the private path before it gets to send the INFO. */
 set_clock(10010000); stream_granted=false; stream_lost=true; send_error=0; publish_a(); iteration();
 assert(!stream_enabled && sends==3 && test_rate_schedule.next_due_us==10020000);
 assert(body_for(SUB_PACKET_INFO) && (body_for(SUB_PACKET_INFO)[5]&RELAY_CAP_NATIVE_TEST));
 set_clock(10019999); publish_a(); iteration();
 assert(sends==3);
 set_clock(10020000); iteration();
 assert(sends==4 && relay_native_test_requested);
 target_tps=120;
 set_clock(10021000); publish_a(); iteration();
 assert(sends==5 && body_for(SUB_PACKET_INFO) && !(body_for(SUB_PACKET_INFO)[5]&RELAY_CAP_NATIVE_TEST));

 target_tps=20; stream_granted=true; start_stream(120);
 for (unsigned i=0;controls<12;i++) { assert(i<24); iteration(); }
 assert(sends==7 && relay_native_test_requested);
 stream_enabled=false; iteration();
 assert(sends==8 && test_rate_schedule.next_due_us==10071000);
 test_enabled=false; publish_a(); iteration();
 assert(sends==9 && body_for(SUB_PACKET_INFO) && !(body_for(SUB_PACKET_INFO)[5]&RELAY_CAP_NATIVE_TEST));
}
static void test_private_narrow_events(void) {
 reset(); test_enabled=true; target_tps=120; sensor_ids_set=true; start_stream(120);
 unsigned info_frame=0,mag_frame=0,events=0;
 /* INFO and mag have 100 ms periods; allow two additional pose
  * opportunities when continuously ready events take every other slot. */
 const unsigned max_metadata_gap_frames=16;
 bool last_event=false;
 for (unsigned i=0;controls<240;i++) {
  assert(i<480);
  set_clock(10000000ULL+(uint32_t)(network_tick-network_origin)*1000000ULL/32768U);
  publish_a(); publish_m();
  unsigned before=sends;
  iteration();
  if (sends==before) continue;
  assert(sends==before+1 && controls==before);
  assert(controls-info_frame<=max_metadata_gap_frames && controls-mag_frame<=max_metadata_gap_frames);
  bool event=sent[0]==TRACKER_EVENT_ESB_TYPE;
  assert(!event || !last_event);
  last_event=event; events+=event;
  if (body_for(SUB_PACKET_INFO)) info_frame=controls;
  if (body_for(SUB_PACKET_QUAT_MAG)) mag_frame=controls;
 }
 assert(sends==240 && events==120 && completes==events && radio_rotations==sends-events);
 assert(controls-info_frame<=max_metadata_gap_frames && controls-mag_frame<=max_metadata_gap_frames);
}
int main(int argc,char **argv){
 assert(argc==2);struct composite_builder b;
 if(!strcmp(argv[1],"hid")){
  reset();event_due=false;composite_builder_reset(&b);assert(composite_try_add(&b,0));
  assert(send_builder(&b)==BIT(0));assert(sent[7]==RELAY_INFO_CAPS);assert_hid_info_zero();
  reset();event_due=false;publish_a();composite_builder_reset(&b);
  assert(composite_try_add(&b,1) && composite_try_add(&b,0));
  assert(send_builder(&b)==(BIT(0)|BIT(1)));assert(body_for(0)[5]==RELAY_INFO_CAPS);assert_hid_info_zero();
  reset();radio=false;event_due=false;composite_builder_reset(&b);assert(composite_try_add(&b,0));
  assert(send_builder(&b)==BIT(0));assert(!sends);assert_hid_info_zero();
 }else if(!strcmp(argv[1],"fresh-wake")){
  reset();event_due=false;stream_enabled=true;stream_target=100;publish_a();
  iteration();assert(stream_wait_target==96 && sends);
  stream_enabled=false;
 }else if(!strcmp(argv[1],"narrow-primary")){
  reset();stream_enabled=true;stream_slot=RELAY_DIRECT_SLOTS-1;publish_a();
  send_error=-EAGAIN;iteration();assert(sends==1 && completes==1 && !completion_success && qa_pending());
  send_error=0;iteration();assert(sends==2 && completion_success && sent_len==17 && sent[0]==TRACKER_EVENT_ESB_TYPE && qa_pending());
  reset();stream_enabled=true;stream_slot=RELAY_DIRECT_SLOTS-1;publish_a();publish_m();last_mag_time=0;
  iteration();assert(sends==1 && sent_len<=32 && (sent[0]==SUB_PACKET_QUAT_MAG || body_for(SUB_PACKET_QUAT_MAG)) && qa_pending());
  assert(last_mag_time==now_ms);
  iteration();assert(sends==2 && sent[0]==TRACKER_EVENT_ESB_TYPE && completion_success);
  stream_enabled=false;stream_slot=0;
 }else if(!strcmp(argv[1],"event-prefetch")){
  reset();publish_a();composite_builder_reset(&b);composite_prepare_event(&b);
  assert(selects==1 && !completes && !b.has_event);
  assert(composite_try_add(&b,1));assert(send_builder(&b)==BIT(1));
  assert(selects==1 && completes==1 && completion_success);
  reset();publish_a();composite_builder_reset(&b);b.sub_max=28;
  composite_prepare_event(&b);assert(composite_try_add(&b,1));
  assert(send_builder(&b)==BIT(1));assert(selects==1 && !completes && !b.has_event);
  reset();event_due=false;publish_a();composite_builder_reset(&b);
  composite_prepare_event(&b);assert(composite_try_add(&b,1));
  assert(send_builder(&b)==BIT(1));assert(selects==1 && !completes);
 }else if(!strcmp(argv[1],"rates")){
  test_exact_rates();
  test_refusal_retry_and_newer_publication();
  test_normal_service_and_bounds();
 }else if(!strcmp(argv[1],"private-rates")){
  test_private_rate(10,120,10);
  test_private_rate(20,120,20);
  test_private_rate(120,120,120);
  test_private_rate(0,100,100);
 }else if(!strcmp(argv[1],"stationary-metadata")){
  test_private_stationary_metadata();
 }else if(!strcmp(argv[1],"private-fallback")){
  test_private_fallback();
 }else if(!strcmp(argv[1],"private-events")){
  test_private_narrow_events();
 }else if(!strcmp(argv[1],"record-size")){
  reset();publish_a();publish_m();composite_builder_reset(&b);
  assert(composite_try_add(&b,2) && composite_try_add(&b,4)
         && composite_try_add(&b,5) && composite_try_add(&b,0));
  assert(send_builder(&b)==(BIT(2)|BIT(4)|BIT(5)|BIT(0)));
  assert(sent_len<=RELAY_RECORD_MAX_LEN && "producer emitted an unrelayable composite");
 }else if(!strcmp(argv[1],"relay-size")){
  for(unsigned slot=0;slot<11;slot++){
   reset();event_due=true;publish_a();publish_m();
   unsigned cap=relay_direct_payload_max(1,slot);
   composite_builder_reset(&b);b.sub_max=cap-4;
   uint8_t pose=composite_pose_type(&b,false,true,true);
   assert(pose==(slot==10 ? SUB_PACKET_COMPACT_QUAT : SUB_PACKET_QUAT_ACCEL));
   assert(composite_try_add(&b,pose));
   assert(composite_try_add(&b,SUB_PACKET_INFO));composite_try_add(&b,4);composite_try_add(&b,5);
   assert(send_builder(&b)&BIT(pose));assert(sent_len<=cap);
  }
 }else assert(false);
 printf("PASS relay producer %s\n",argv[1]);return 0;
}
'''
with tempfile.TemporaryDirectory(prefix="relay-producer-",dir="/tmp") as tmp:
    path=Path(tmp)/"producer.c"
    path.write_text(prefix+transport.build_source()+transport.legacy_parser_source()+legacy_tests+selectors+CASES)
    binary=Path(tmp)/"producer"
    subprocess.run(shlex.split(os.environ.get("CC","cc"))+[
        "-std=c11","-Wall","-Wextra","-Werror","-Wno-unused-function","-Wno-sign-compare",
        "-g","-O1","-fsanitize=address,undefined","-fno-pie","-no-pie","-DCONFIG_ESB_MAX_PAYLOAD_LENGTH=72",
        "-DCONFIG_CONNECTION_RELAY=1","-DCONFIG_CONNECTION_RAW_COLLECTION=1",
        "-DCONFIG_CONNECTION_TDMA=1",
        "-DCONFIG_SENSOR_USE_MAGNETOMETER=1","-I",str(HERE.parent/"concurrency"),
        "-I",str(ROOT/"src"),str(path),"-lm","-o",str(binary)],check=True)
    for case in sys.argv[1:] or ("hid","fresh-wake","narrow-primary","event-prefetch","rates","private-rates",
                                 "stationary-metadata","private-fallback","private-events","record-size","relay-size"):
        subprocess.run([str(binary),case],check=True)
