#!/usr/bin/env python3
"""Actual tracker encoder/producer with relay + USB HID enabled.

Reuse the direct transport fixture's real snapshot and encoding functions;
the relay scheduler remains a leaf, covered by the worker and TDMA tests.
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
#include "connection/relay_protocol.h"
static uint8_t relay_info_caps(void) { return RELAY_INFO_CAPS; }
'''
legacy_tests=transport.TESTS.replace('int main(void)','static int unused_direct_suite(void)')
CASES=r'''
static void assert_hid_info_zero(void) {
 unsigned infos=0;
 for(unsigned i=0;i<mirrors;i++) if(mirrored[i][0]==0){infos++;assert(mirrored[i][7]==0);}
 assert(infos);
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
  reset();event_due=false;stream=true;stream_target=100;publish_a();
  iteration();assert(stream_wait_target==96 && sends);
  stream=false;
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
 }else if(!strcmp(argv[1],"record-size")){
  reset();publish_a();publish_m();composite_builder_reset(&b);
  assert(composite_try_add(&b,2) && composite_try_add(&b,4)
         && composite_try_add(&b,5) && composite_try_add(&b,0));
  assert(send_builder(&b)==(BIT(2)|BIT(4)|BIT(5)|BIT(0)));
  assert(sent_len<=RELAY_RECORD_MAX_LEN && "producer emitted an unrelayable composite");
 }else if(!strcmp(argv[1],"v3-size")){
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
    path.write_text(prefix+transport.build_source()+transport.legacy_parser_source()+legacy_tests+CASES)
    binary=Path(tmp)/"producer"
    subprocess.run(shlex.split(os.environ.get("CC","cc"))+[
        "-std=c11","-Wall","-Wextra","-Werror","-Wno-unused-function","-Wno-sign-compare",
        "-g","-O1","-fsanitize=address,undefined","-DCONFIG_ESB_MAX_PAYLOAD_LENGTH=72",
        "-DCONFIG_CONNECTION_RELAY=1","-DCONFIG_CONNECTION_RAW_COLLECTION=1",
        "-DCONFIG_SENSOR_USE_MAGNETOMETER=1","-I",str(HERE.parent/"concurrency"),
        "-I",str(ROOT/"src"),str(path),"-lm","-o",str(binary)],check=True)
    for case in sys.argv[1:] or ("hid","fresh-wake","event-prefetch","rates","record-size","v3-size"):
        subprocess.run([str(binary),case],check=True)
