#!/usr/bin/env python3
"""Run production relay_radio.c against a RADIO register/DMA model.

The two builds cover the nRF52 END and nRF54L PHYEND branches. This is a
software ownership test, not an RF interoperability or silicon timing test.
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
source = (ROOT / "src/connection/relay_radio.c").read_text()
source = re.sub(r'^#include[^\n]*\n', '', source, flags=re.MULTILINE)
PREFIX = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "relay_radio.h"
#define CONFIG_ESB_MAX_PAYLOAD_LENGTH 72
#define __aligned(n) __attribute__((aligned(n)))
#define __DMB() ((void)0)
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
#define RADIO_IRQn 1
#define RADIO_0_IRQn 2
#define NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK 1U
#define NRF_RADIO_SHORT_READY_START_MASK 2U
#define NRF_RADIO_SHORT_END_DISABLE_MASK 4U
#define NRF_RADIO_SHORT_END_START_MASK 8U
#define NRF_RADIO_SHORT_PHYEND_DISABLE_MASK 16U
#define NRF_RADIO_SHORT_PHYEND_START_MASK 32U
enum {NRF_RADIO_EVENT_READY,NRF_RADIO_EVENT_ADDRESS,NRF_RADIO_EVENT_PAYLOAD,
 NRF_RADIO_EVENT_END,NRF_RADIO_EVENT_DISABLED,NRF_RADIO_EVENT_CRCOK,NRF_RADIO_EVENT_CRCERROR,
 NRF_RADIO_EVENT_PHYEND};
typedef unsigned nrf_radio_event_t;
enum {NRF_RADIO_STATE_DISABLED,NRF_RADIO_STATE_RXIDLE,NRF_RADIO_STATE_RX};
enum {NRF_RADIO_TASK_DISABLE,NRF_RADIO_TASK_RXEN,NRF_RADIO_TASK_START};
static struct {uint32_t shorts,inten,state,frequency;void *ptr;uint8_t addresses;bool event[8];} radio;
#define NRF_RADIO (&radio)
static uint32_t now_us, starts, rxen_us;
static bool irq_locked, inject_irq;
static bool inject_dma;
static unsigned irq_lock(void) {unsigned old=irq_locked;irq_locked=true;return old;}
static void irq_unlock(unsigned k) {irq_locked=k;}
static void NVIC_ClearPendingIRQ(unsigned n) {(void)n;}
static void k_busy_wait(unsigned n) {now_us+=n;}
static int reboot_requests;
static int sys_request_system_reboot(void) {return ++reboot_requests, 0;}
static uint32_t sys_clock_tick_get_32(void) {return now_us;}
static uint32_t k_ticks_to_us_floor32(uint32_t n) {return n;}
#define nrf_radio_shorts_get(r) ((r)->shorts)
#define nrf_radio_int_enable_check(r,m) ((r)->inten&(m))
#define nrf_radio_packetptr_get(r) ((r)->ptr)
#define nrf_radio_rxaddresses_get(r) ((r)->addresses)
#define nrf_radio_int_disable(r,m) ((r)->inten&=~(m))
#define nrf_radio_int_enable(r,m) ((r)->inten|=(m))
#define nrf_radio_shorts_set(r,v) ((r)->shorts=(v))
static void model_frequency(uint32_t frequency) {
 if(inject_irq && !irq_locked) now_us+=200; /* unrelated USB/sensor IRQ */
 radio.frequency=frequency;
}
#define nrf_radio_frequency_set(r,v) model_frequency(v)
#define nrf_radio_rxaddresses_set(r,v) ((r)->addresses=(v))
#define nrf_radio_packetptr_set(r,v) ((r)->ptr=(v))
#define nrf_radio_state_get(r) ((r)->state)
#define nrf_radio_event_check(r,v) ((r)->event[(v)])
#define nrf_radio_crc_status_check(r) (true)
#define nrf_radio_rxmatch_get(r) (2)
#define nrf_radio_rssi_sample_get(r) (55)
static void nrf_radio_event_clear(void *unused,unsigned e) {
 (void)unused;radio.event[e]=false;
 if(inject_dma && (e==NRF_RADIO_EVENT_END || e==NRF_RADIO_EVENT_PHYEND)
    && (radio.shorts&(8U|32U))) {
  /* A CPU IRQ lock does not prevent the next packet's EasyDMA write. */
  ((uint8_t *)radio.ptr)[2]=0x99;
 }
}
static bool stuck;
static void nrf_radio_task_trigger(void *unused,unsigned t) {
 (void)unused;
 if(t==NRF_RADIO_TASK_DISABLE && !stuck) {radio.state=0;radio.event[NRF_RADIO_EVENT_DISABLED]=true;}
 else if(t==NRF_RADIO_TASK_RXEN) {rxen_us=now_us;radio.state=NRF_RADIO_STATE_RX;}
 else {starts++;radio.state=NRF_RADIO_STATE_RX;}
}
'''
CASES = r'''
static void setup(void) {
 memset(&radio,0,sizeof(radio));now_us=starts=0;inject_dma=inject_irq=irq_locked=false;
 radio.shorts=0x100;radio.inten=0x400;radio.addresses=4;radio.frequency=2443;
 radio.ptr=(void *)(uintptr_t)0x1234;
 relay_radio_start(43,0xFE,false);
 assert(radio.inten==0 && radio.addresses==0xFE);
}
static void completed(void) {
 rx_buffer[0]=13;rx_buffer[1]=0;rx_buffer[2]=0xF1;
 radio.state=NRF_RADIO_STATE_RXIDLE;
 radio.event[RR_EVENT_END]=true;
}
int main(int argc,char **argv) {
 struct relay_rx_frame frame;
 if(argc==1 || !strcmp(argv[1],"startup")) {
  setup();relay_radio_stop();now_us=0;inject_irq=true;
  relay_radio_start(43,0xFE,true);
  assert(rxen_us==0 && "an unrelated IRQ delayed the configured RXEN edge");
  assert(!irq_locked);relay_radio_stop();assert(!irq_locked);
  puts("PASS radio: raw configuration stays atomic through RXEN");
 }
 if(argc==1 || !strcmp(argv[1],"prepare")) {
  setup();relay_radio_stop();now_us=rxen_us=0;
  assert(relay_radio_prepare(43,0xFE,true));
  assert(radio.state==NRF_RADIO_STATE_DISABLED && rxen_us==0 && !irq_locked);
  now_us=40;assert(relay_radio_receive() && rxen_us==40);
  assert(!irq_locked);relay_radio_stop();
  assert(relay_radio_prepare(43,0xFE,true));relay_radio_stop();
  assert(!relay_radio_receive() && radio.state==NRF_RADIO_STATE_DISABLED);
  puts("PASS radio: scheduled prepare stays disabled until RXEN; unstarted prepare cancels");
 }
 if(argc==1 || !strcmp(argv[1],"dma")) {
  setup();completed();now_us=5;inject_dma=true;
  assert(relay_radio_poll(&frame));
  assert(frame.data[0]==0xF1 && "DMA overwrote the completed packet while copying");
  assert(starts==1);relay_radio_stop();
  assert(radio.shorts==0x100 && radio.inten==0x400 && radio.addresses==4);
  assert(radio.ptr==(void *)(uintptr_t)0x1234 && radio.state==0);
  puts("PASS radio: completed DMA packet stays stable until copied; ESB state restored");
 }
 if(argc==1 || !strcmp(argv[1],"timestamp")) {
  setup();completed();now_us=200;
  assert(!relay_radio_poll(&frame) && "delayed polling timestamp was accepted as END");
  assert(starts==1);relay_radio_stop();
  puts("PASS radio: delayed END observation is dropped and RX rearms");
 }
 if(argc==1 || !strcmp(argv[1],"cleanup")) {
  setup();radio.event[NRF_RADIO_EVENT_ADDRESS]=true;
  assert(relay_radio_busy() && !relay_radio_take(&frame));
  relay_radio_stop();relay_radio_stop();
  assert(radio.state==0 && radio.shorts==0x100 && radio.inten==0x400);
  setup();completed();now_us=7;
  assert(relay_radio_take(&frame) && frame.end_kernel_ticks==7);
  relay_radio_stop();
  puts("PASS radio: incomplete and completed single-window cleanup");
 }
 setup();stuck=true;void *raw=radio.ptr;
 assert(!relay_radio_stop() && relay_radio_faulted() && reboot_requests==1);
 assert(radio.ptr==raw && !radio.inten && saved.active);
 assert(!relay_radio_start(43,0xFE,true));
 puts("PASS radio: disable timeout fences DMA and interrupts and requests reboot");
 return 0;
}
'''
with tempfile.TemporaryDirectory(prefix="relay-radio-", dir="/tmp") as tmp:
    path = Path(tmp) / "radio.c"
    path.write_text(PREFIX + source + CASES)
    for family in (52, 54):
        binary = Path(tmp) / f"radio-{family}"
        subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function",
            "-g", "-O1", "-fsanitize=address,undefined", "-I", str(ROOT / "src/connection"),
            *(["-DCONFIG_SOC_SERIES_NRF54L=1"] if family == 54 else []),
            str(path), "-o", str(binary)], check=True)
        subprocess.run([str(binary), *sys.argv[1:]], check=True)
