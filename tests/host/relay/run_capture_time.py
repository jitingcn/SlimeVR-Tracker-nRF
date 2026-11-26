#!/usr/bin/env python3
"""Exercise production nRF52 capture ownership and hardware END conversion."""
from pathlib import Path
import os, shlex, subprocess, tempfile
root = Path(__file__).resolve().parents[3]
src = (root / 'src/connection/radio_capture.c').read_text()
platform = src[src.index('static nrfx_gppi_handle_t capture_conn[3];'):
               src.index('#elif defined(CONFIG_SOC_SERIES_NRF54L)\nstatic nrfx_gppi_handle_t capture_conn[2];')]
a = src.index('static int capture_init_locked(')
b = src.index('\nvoid radio_capture_record', a)
code = r'''
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define CONFIG_SOC_SERIES_NRF52 1
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
struct capture_record { uint64_t ready_us, address_us, end_us; };
static bool capture_initialized, diagnostics_available, rx_capture_owned, user_capture_active, locked;
static int capture_enabled, ping_transaction_pending;
static unsigned irq_lock(void) { unsigned old=locked; locked=true; return old; }
static void irq_unlock(unsigned old) { assert(locked); locked=old; }
static int atomic_get(const int *value) { return *value; }
static void atomic_set(int *value, int new_value) { *value=new_value; }
static bool radio_capture_is_enabled(void);
#define led_request_event(...) ((void)0)
static void connection_feedback_maintenance_update(void) {}

#define NRF_RADIO 0
#define NRF_TIMER3 0
typedef unsigned nrfx_gppi_handle_t, nrf_radio_event_t, nrf_timer_task_t;
enum { NRF_RADIO_EVENT_READY, NRF_RADIO_EVENT_ADDRESS, NRF_RADIO_EVENT_END };
enum { NRF_TIMER_TASK_CAPTURE0, NRF_TIMER_TASK_CAPTURE1, NRF_TIMER_TASK_CAPTURE2,
       NRF_TIMER_TASK_CAPTURE3, NRF_TIMER_TASK_STOP, NRF_TIMER_TASK_CLEAR, NRF_TIMER_TASK_START };
enum { NRF_TIMER_CC_CHANNEL0, NRF_TIMER_CC_CHANNEL1, NRF_TIMER_CC_CHANNEL2, NRF_TIMER_CC_CHANNEL3 };
enum { NRF_TIMER_MODE_TIMER, NRF_TIMER_BIT_WIDTH_32, NRF_TIMER_FREQ_1MHz };
static unsigned allocated, enabled, starts, stops;
static uint32_t cc[4], timer_us;
static bool timer_running, fail_end_allocation;
static void nrf_timer_task_trigger(unsigned timer, unsigned task) {
    assert(locked && timer == NRF_TIMER3);
    if (task == NRF_TIMER_TASK_START) { assert(!timer_running); timer_running=true; ++starts; }
    if (task == NRF_TIMER_TASK_STOP) { timer_running=false; ++stops; }
    if (task == NRF_TIMER_TASK_CLEAR) timer_us=0;
    if (task == NRF_TIMER_TASK_CAPTURE3) { assert(timer_running); cc[3]=timer_us; }
}
static uint32_t nrf_timer_cc_get(unsigned timer, unsigned channel) {
    assert(locked && timer_running && timer == NRF_TIMER3); return cc[channel];
}
static void nrf_timer_mode_set(unsigned timer, unsigned mode) { assert(locked && timer==NRF_TIMER3 && mode==NRF_TIMER_MODE_TIMER); }
static void nrf_timer_bit_width_set(unsigned timer, unsigned width) { assert(locked && timer==NRF_TIMER3 && width==NRF_TIMER_BIT_WIDTH_32); }
static void nrf_timer_prescaler_set(unsigned timer, unsigned freq) { assert(locked && timer==NRF_TIMER3 && freq==NRF_TIMER_FREQ_1MHz); }
static uint32_t nrf_radio_event_address_get(unsigned radio, unsigned event) { assert(radio==NRF_RADIO); return event; }
static uint32_t nrf_timer_task_address_get(unsigned timer, unsigned task) { assert(timer==NRF_TIMER3); return task; }
static int nrfx_gppi_conn_alloc(unsigned event, unsigned task, unsigned *handle) {
    assert(locked && event==task && !(allocated & (1U<<event)));
    if (event==2 && fail_end_allocation) return -ENOMEM;
    allocated |= 1U<<event; *handle=event; return 0;
}
static void nrfx_gppi_conn_enable(unsigned handle) {
    assert(locked && (allocated & (1U<<handle))); enabled |= 1U<<handle;
}
static void nrfx_gppi_conn_disable(unsigned handle) {
    assert(locked && (allocated & (1U<<handle))); enabled &= ~(1U<<handle);
}
static void nrfx_gppi_conn_free(unsigned event, unsigned task, unsigned handle) {
    assert(locked && event==task && task==handle && !(enabled & (1U<<handle)));
    allocated &= ~(1U<<handle);
}
/* A seven-tick interrupt at the TIMER/kernel sampling boundary. */
static uint64_t k_uptime_ticks(void) { return 32+(locked ? 0 : 7); }
static uint32_t k_us_to_ticks_floor32(uint32_t us) { return (uint64_t)us*32768/1000000; }
''' + platform + src[a:b] + r'''
static void received(void) {
    assert(timer_running && allocated==7 && enabled==7);
    cc[0]=800;cc[1]=850;cc[2]=900;timer_us=1000;
    uint32_t ticks=0;
    assert(radio_capture_rx_end(&ticks) && ticks==29 && !locked);
    cc[2]=0;assert(!radio_capture_rx_end(&ticks) && !locked);
    cc[2]=900;timer_us=2001;assert(!radio_capture_rx_end(&ticks) && !locked);
}
static void released(void) {
    uint32_t ticks;
    assert(!timer_running && !allocated && !enabled && !capture_initialized);
    assert(!radio_capture_rx_end(&ticks) && !locked);
}
int main(void) {
    fail_end_allocation=true;
    assert(radio_capture_rx_acquire()==-ENOMEM); released();
    fail_end_allocation=false;
    assert(radio_capture_rx_acquire()==0 && !radio_capture_is_enabled());
    received();
    unsigned initial_starts=starts;
    assert(radio_capture_rx_acquire()==0 && starts==initial_starts);
    radio_capture_rx_release(); released();
    assert(radio_capture_rx_acquire()==0 && starts==initial_starts+1);
    received();
    radio_capture_deinit(); released(); /* Teardown in a relay-only build. */

    assert(radio_capture_init()==0 && !radio_capture_user_active());
    assert(radio_capture_rx_acquire()==0 && radio_capture_is_enabled());
    radio_capture_rx_release();
    received(); /* Leaving the profile cannot stop diagnostics. */
    assert(radio_capture_rx_acquire()==0);
    assert(radio_capture_user_set_enabled(false)==0 && !radio_capture_is_enabled());
    received(); /* Disabling diagnostics cannot stop raw timestamps. */
    radio_capture_rx_release(); released();
    assert(radio_capture_user_set_enabled(true)==0 && radio_capture_user_active());
    received();
    ping_transaction_pending=1;
    radio_capture_deinit(); released();
    assert(!radio_capture_user_active() && !ping_transaction_pending);
    assert(radio_capture_user_set_enabled(true)==-ENODEV);
    return 0;
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d); (p/'test.c').write_text(code)
 subprocess.run(shlex.split(os.environ.get('CC','cc'))+['-std=c11','-Wall','-Wextra','-Werror',str(p/'test.c'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
print('nRF52 capture survives IRQ injection, releases both owners independently, and restarts')
