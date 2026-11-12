#!/usr/bin/env python3
"""The own-PONG NORMAL schedule decision, extracted without editing its body."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

HERE=Path(__file__).resolve().parent
ROOT=Path(os.environ.get("SOURCE_ROOT",HERE.parents[2]))
sys.path.insert(0,str(HERE.parent/"harness/python"))
from c_extract import extract_block
source=(ROOT/"src/connection/esb.c").read_text()
header=(ROOT/"src/connection/esb.h").read_text()
pong=extract_block(source,r'^static void esb_receive_pong\([^;{]*\)\s*\{')
schedule=extract_block(pong,r'^\t\tif \(pong_flags == ESB_PONG_FLAG_NORMAL\)\s*\{')
constant=re.search(r'^#define ESB_PONG_FLAG_NORMAL[^\n]*',header,re.MULTILINE)[0]
PREFIX=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define TDMA_MAX_TRACKERS 16
struct esb_payload {uint8_t data[23];};
static uint8_t epoch=7;
static bool profile;
static uint8_t tdma_profile_get(void){return profile;}
static bool layout_stale,enabled=true,channel_wait_normal,server_time_synced=true;
static unsigned channel_found,resync,updates;
static uint8_t tdma_get_config_epoch(void){return epoch;}
static bool tdma_is_enabled(void){return enabled;}
static void tdma_set_enabled(bool on){enabled=on;}
static bool tdma_update_config(uint8_t slot,uint8_t total,uint8_t width,uint8_t e){
 if(!total || total>16 || slot>=total || width<16)return false;
 epoch=e;enabled=true;updates++;return true;
}
static void relay_reset(void){}
static void connection_request_ping_resync(void){resync++;}
static void atomic_set(unsigned *p,unsigned v){*p=v;}
#include "relay_protocol.h"
static struct relay_pong_trailer own_trailer;
static bool own_trailer_valid=true,layout_confirm_seen;
static uint32_t layout_confirm_time,staged_t2,stage_calls;
static uint32_t k_uptime_get_32(void){return 4242;}
static uint32_t sys_get_be32(const uint8_t *p){return (uint32_t)p[0]<<24|(uint32_t)p[1]<<16|(uint32_t)p[2]<<8|p[3];}
/* Production staging is covered by run_presence.py; here only its call. */
static void esb_stage_from_trailer(const struct relay_pong_trailer *t,uint32_t t2){(void)t;staged_t2=t2;stage_calls++;}
'''
CASES=r'''
int main(int argc,char **argv){
 assert(argc==2);struct esb_payload p={0};p.data[8]=2;p.data[9]=3;p.data[10]=16;p.data[11]=7;
 if(!strcmp(argv[1],"profile")){
  profile=true;layout_stale=true;p.data[11]=8;receive_schedule(&p,ESB_PONG_FLAG_NORMAL);
  assert(updates==0 && stage_calls==1 && enabled && epoch==7);
  p.data[11]=7;receive_schedule(&p,ESB_PONG_FLAG_NORMAL);
  assert(!layout_stale && layout_confirm_seen);
  p.data[8]=255;receive_schedule(&p,ESB_PONG_FLAG_NORMAL);
  assert(!enabled && layout_stale && resync==1 && updates==0);
 }else if(!strcmp(argv[1],"private-bootstrap")){
  const struct relay_geometry cases[]={{11,5,16,120,4096,15, 0},{16,3,15,120,300,1, 0}};
  for(unsigned row=0;row<sizeof(cases)/sizeof(cases[0]);row++){
   channel_wait_normal=true;channel_found=0;enabled=false;profile=false;updates=0;
   p.data[9]=11;p.data[10]=cases[row].slot_ticks;
   own_trailer.pending=true;own_trailer.pending_total=relay_geometry_code(cases[row].direct,cases[row].repairs);
   own_trailer.pending_slot_ticks=cases[row].slot_ticks;own_trailer.pending_epoch=7;
   layout_confirm_time=0;layout_confirm_seen=false;
   receive_schedule(&p,ESB_PONG_FLAG_NORMAL);
   assert(channel_found && stage_calls && updates==0 && resync==0);
   /* Boot took longer than one layout lease. Acquisition must remain alive
    * without confirming any pose grant before the first B. */
   assert(layout_confirm_time==4242 && !layout_confirm_seen && layout_stale);
  }
 }else if(!strcmp(argv[1],"private-width")){
  p.data[10]=15;own_trailer.pending=true;own_trailer.pending_total=relay_geometry_code(3,3);own_trailer.pending_slot_ticks=15;own_trailer.pending_epoch=7;
  receive_schedule(&p,ESB_PONG_FLAG_NORMAL);assert(resync==0);
  own_trailer.pending=false;receive_schedule(&p,ESB_PONG_FLAG_NORMAL);assert(resync==1);
 }else if(!strcmp(argv[1],"unassigned")){
  p.data[8]=255;p.data[11]=8;receive_schedule(&p,ESB_PONG_FLAG_NORMAL);
  assert(layout_stale && !enabled && resync==1);
 }else if(!strcmp(argv[1],"same-epoch-recovery")){
  enabled=false;layout_stale=true;receive_schedule(&p,ESB_PONG_FLAG_NORMAL);
  assert(enabled && !layout_stale && updates==1);
  assert(layout_confirm_seen && layout_confirm_time==4242);
 }else if(!strcmp(argv[1],"stage-before-withdraw")){
  /* A joiner's unassigned tuple still stages its announced first slot. */
  p.data[3]=0x12;p.data[4]=0x34;p.data[5]=0x56;p.data[6]=0x78;p.data[8]=255;
  receive_schedule(&p,ESB_PONG_FLAG_NORMAL);
  assert(stage_calls==1 && staged_t2==0x12345678 && !enabled && !layout_confirm_seen);
 }else if(!strcmp(argv[1],"no-trailer-no-grant")){
  own_trailer_valid=false;receive_schedule(&p,ESB_PONG_FLAG_NORMAL);
  assert(!stage_calls && !layout_stale && !layout_confirm_seen);
 }else assert(false);
 printf("PASS own-PONG schedule %s\n",argv[1]);return 0;
}
'''
with tempfile.TemporaryDirectory(prefix="relay-own-schedule-",dir="/tmp") as tmp:
    c=Path(tmp)/"schedule.c";exe=Path(tmp)/"schedule"
    c.write_text(PREFIX+constant+'\nstatic void receive_schedule(const struct esb_payload *payload,uint8_t pong_flags){\n'+schedule+'\n}\n'+CASES)
    subprocess.run(shlex.split(os.environ.get("CC","cc"))+[
        "-std=c11","-Wall","-Wextra","-Werror","-Wno-unused-function",
        "-I"+str(ROOT/"src/connection"),"-g","-O1","-fsanitize=address,undefined",str(c),"-o",str(exe)],check=True)
    for case in sys.argv[1:] or ("profile","private-bootstrap","private-width","unassigned","same-epoch-recovery","stage-before-withdraw","no-trailer-no-grant"):
        subprocess.run([str(exe),case],check=True)
