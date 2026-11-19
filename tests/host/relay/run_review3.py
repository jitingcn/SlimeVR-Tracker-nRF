#!/usr/bin/env python3
"""Query reply lifecycle and radio-off stream ownership, using production bodies."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile
HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get('SOURCE_ROOT', HERE.parents[2]))
sys.path.insert(0, str(HERE.parent / 'harness/python'))
from c_extract import extract_block

def fn(s, name):
    return extract_block(s, rf'^(?:static )?(?:int|void) {name}\([^;{{]*\)\s*\{{')

esb = (ROOT/'src/connection/esb.c').read_text()
conn = (ROOT/'src/connection/connection.c').read_text()
ota = (ROOT/'src/system/esb_ota.c').read_text()
clock_gate = extract_block(conn, r'^\t\tif \(stream_clock_held &&[^\n]*\) \{')
service = fn(ota, 'ota_service_fw_info') if 'static void ota_service_fw_info(void)' in ota else 'static void ota_service_fw_info(void) {}'
query = fn(ota, 'esb_ota_handle_query_info')
# Compile the actual packet encoder as well as the deferred reply state machine.
send = fn(ota, 'ota_send_fw_info')
prefix = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#define LOG_DBG(...) ((void)0)
#define LOG_ERR(...) ((void)0)
#define LOG_INF(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define IS_ENABLED(x) (x)
#define CONFIG_BOOTLOADER_MCUBOOT 0
#define CONFIG_BUILD_OUTPUT_UF2 0
#define CONFIG_BOARD_HAS_NRF5_BOOTLOADER 0
#define OTA_FW_INFO_PACKET_SIZE 66
#define ESB_OTA_FW_INFO_TYPE 0x25
#define FW_VERSION_MAJOR 1
#define FW_VERSION_MINOR 2
#define FW_VERSION_PATCH 3
#define BUILD_YEAR 2026
#define BUILD_MONTH 11
#define BUILD_DAY 15
#define BUILD_HOUR 1
#define BUILD_MIN 2
#define BUILD_SEC 4
#define OTA_BOOTLOADER_MCUBOOT 1
#define OTA_BOOTLOADER_NONE 0
#define OTA_PROTOCOL_VERSION 1
#define BOARD_TARGET_STRING "host"
#define OTA_BOARD_TARGET_MAX 48
#define OTA_FLASH_BASE 4096
static char _flash_used[1];
static void sys_put_be32(uint32_t n,uint8_t*p){(void)n;(void)p;}
static void sys_put_le32(uint32_t n,uint8_t*p){(void)n;(void)p;}
static void sys_put_be16(uint16_t n,uint8_t*p){(void)n;(void)p;}
static uint8_t connection_get_id(void){return 2;}
static uint8_t esb_ota_crc8(const uint8_t*p,unsigned n){(void)p;(void)n;return 0;}
static bool profile=true,ready=true,fw_info_pending;
static int64_t now_ms=1000,fw_info_retry_time;
static unsigned writes,queued,tx_clock_users=1;
static int write_error;
static bool clock_status=true;
static bool esb_ready(void){return ready;}
static unsigned tdma_profile_get(void){return profile;}
static int64_t k_uptime_get(void){return now_ms;}
static int esb_write(uint8_t*p,bool noack,size_t len){
 assert(p[0]==ESB_OTA_FW_INFO_TYPE && !noack && len==66);writes++;
 if(profile)return -EAGAIN;
 if(write_error)return write_error;
 queued++;return 0;
}
static void atomic_inc(unsigned*p){++*p;}
static void atomic_dec(unsigned*p){--*p;}
static bool esb_is_idle(void){return true;}
#define ESB_ST_PAIRING 0
static unsigned esb_conn_state=1;
static bool connection_get_data_collection(void){return false;}
static int clocks_start(void){clock_status=true;return 0;}
static bool lf_synth;
#define CONFIG_CLOCK_USE_LF_SYNTH lf_synth
static unsigned clocks_stop_generation,clk_mgr;
static bool clocks_hf_owned=true;
static unsigned irq_lock(void){return 0;}
static void irq_unlock(unsigned key){(void)key;}
static unsigned atomic_get(unsigned*p){return *p;}
static int onoff_release(unsigned manager){(void)manager;return 0;}
'''
cases = r'''
int main(int argc,char**argv){
 assert(argc==2);
 if(!strcmp(argv[1],"query")){
  esb_ota_handle_query_info();ota_service_fw_info();assert(!writes);
  profile=false;write_error=-EAGAIN;ota_service_fw_info();assert(writes==1 && !queued);
  ota_service_fw_info();assert(writes==1);
  now_ms+=100;write_error=0;ota_service_fw_info();assert(queued==1);
  now_ms+=100;ota_service_fw_info();assert(queued==1);
 }else{
  for(unsigned synth=0;synth<2;synth++){
   bool stream_clock_held=true;tx_clock_users=1;clock_status=true;clocks_hf_owned=true;lf_synth=synth;
   ready=false;profile=true;
   CLOCK_GATE
   /* This is the following radio-off/HID-off sleep iteration. */
   assert(!stream_clock_held && !tx_clock_users);
   assert(clock_status==lf_synth);
  }
 }
 return 0;
}
'''.replace('CLOCK_GATE', clock_gate)
with tempfile.TemporaryDirectory(prefix='review3-') as tmp:
    c=Path(tmp)/'test.c'; binary=Path(tmp)/'test'
    c.write_text(prefix+fn(esb,'clocks_stop')+fn(esb,'esb_release_tx_clock')+fn(esb,'esb_relay_clock')+send+query+service+cases)
    subprocess.run(shlex.split(os.environ.get('CC','cc'))+['-std=c11','-Wall','-Wextra','-Werror','-Wno-unused-variable','-Wno-unused-function','-Wno-pointer-to-int-cast','-O1','-g','-fsanitize=address,undefined',str(c),'-o',str(binary)],check=True)
    for case in sys.argv[1:] or ('query','clock'):
        subprocess.run([str(binary),case],check=True)
        print('PASS review3',case)
