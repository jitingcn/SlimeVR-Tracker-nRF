#!/usr/bin/env python3
"""Real snapshot, numerical encoders, transport and bounded connection iterations."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'harness/python'))
from c_extract import extract_block as block

ROOT = Path(os.environ.get("SOURCE_ROOT", Path(__file__).resolve().parents[3]))
HOST = Path(__file__).resolve().parents[1]


def function(source, name):
    return block(source, rf"^(?:static\s+)?(?:const struct sub_packet_desc \*|bool|void|int|uint8_t|uint32_t)\s*{re.escape(name)}\([^;{{]*\)\s*\{{")


PREFIX = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include "connection/tracker_events.h"
#include "connection/sensor_data_snapshot.h"
#include "util.h"
#define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))
#define BIT(x) (1U << (x))
#define CONFIG_CONNECTION_OVER_HID 1
#define SYS_STATUS_CONNECTION_ERROR 1
#define SYS_STATUS_USB_CONNECTED 2
#define LOG_WRN(...) ((void)0)
#define OTA_SUPPRESS_TIMEOUT_MS 600000
#define PING_QUEUE_RETRY_MS 10
#define FW_BOARD 7
#define FW_MCU 8
#define BUILD_YEAR 2026
#define BUILD_MONTH 10
#define BUILD_DAY 21
#define FW_VERSION_MAJOR 1
#define FW_VERSION_MINOR 2
#define FW_VERSION_PATCH 3
static inline void sys_put_le16(uint16_t value, uint8_t dst[2]) {
    dst[0]=value; dst[1]=value>>8;
}
static inline void sys_put_le32(uint32_t value, uint8_t dst[4]) {
    sys_put_le16(value,dst); sys_put_le16(value>>16,&dst[2]);
}
static bool radio=true, usb=true, test_enabled, ota, error, event_due=true, raw, raw_pending;
static bool sensor_ids_set, ota_suppressed;
static sensor_data_snapshot_t sensor_data_snapshot;
static float sensor_last_q[4];
static unsigned publication_wakes;
static void connection_signal_wake(void) { publication_wakes++; }
static int64_t ota_suppress_start_time;
static uint8_t tracker_id=2, packet_sequence=19;
static uint8_t batt=60, batt_v=94, sensor_temp=133, imu_id=9, mag_id=10;
static uint8_t tracker_svr_status=11, tracker_status=12;
static bool no_ack=true;
static uint16_t target_tps=128;
static uint32_t now_ms=10000;
static uint64_t clock_us=10000000;
static int64_t last_mag_time=10000, last_info_time=10000, last_status_time=10000, last_runtime_time=10000;
static unsigned sends, mirrors, selects, completes, raw_calls, ping_calls, sleeps;
static unsigned radio_rotations, hid_rotations, captures;
static uint8_t radio_rotation_types[1024], hid_rotation_types[1024];
static bool completion_success, sent_noack;
static int send_error;
static uint8_t sent[CONFIG_ESB_MAX_PAYLOAD_LENGTH], mirrored[1024][16];
static uint8_t hid_accept_mask=63;
static size_t sent_len;
static void (*radio_hook)(void), (*hid_hook)(void), (*capture_hook)(void);
static void capture_with_hook(sensor_data_snapshot_t *state, sensor_data_capture_t *value) {
    captures++;
    if (capture_hook) { void (*hook)(void)=capture_hook; capture_hook=NULL; hook(); }
    sensor_data_snapshot_capture(state,value);
}
static uint32_t k_uptime_get_32(void) { return now_ms; }
static uint64_t k_uptime_ticks(void) { return clock_us; }
static uint64_t k_ticks_to_us_near64(uint64_t x) { return x; }
static uint64_t k_ticks_to_us_floor64(uint64_t x) { return x; }
static uint64_t sys_get_battery_remaining_time_estimate(void) { return 123456789; }
static int last_sleep_ms;
static void k_msleep(int x) { last_sleep_ms=x; sleeps++; }
static void k_usleep(int x) { (void)x; sleeps++; }
static bool esb_ready(void) { return radio; }
static bool esb_ota_is_active(void) { return ota; }
static bool get_status(int x) { return x==SYS_STATUS_USB_CONNECTED ? usb : error; }
static bool test_mode_get(void) { return test_enabled; }
static bool rotation_type(uint8_t type) { return type==1 || type==2 || type==4; }
static unsigned wire_body_len(uint8_t type) {
    switch (type) {
    case 0: case 2: return 13;
    case 1: case 4: return 14;
    case 3: return 2;
    case 5: return 8;
    case TRACKER_EVENT_ESB_TYPE: return 15;
    default: assert(!"unknown consumer packet type"); return 0;
    }
}
static void record_radio_rotations(const uint8_t *p, size_t n) {
    if (p[0]==ESB_COMPOSITE_TYPE) {
        size_t pos=3;
        for (unsigned i=0; i<p[2]; ++i) {
            assert(pos<n-1); uint8_t type=p[pos++];
            if (rotation_type(type)) {
                assert(radio_rotations<ARRAY_SIZE(radio_rotation_types));
                radio_rotation_types[radio_rotations++]=type;
            }
            pos+=wire_body_len(type);
        }
        assert(pos==n-1);
    } else if (rotation_type(p[0])) {
        assert(n==17 && radio_rotations<ARRAY_SIZE(radio_rotation_types));
        radio_rotation_types[radio_rotations++]=p[0];
    }
}
static int esb_write(uint8_t *p, bool noack, size_t n) {
    assert(n<=sizeof(sent)); memcpy(sent,p,n); sent_len=n; sent_noack=noack; sends++;
    if (radio_hook) { void (*hook)(void)=radio_hook; radio_hook=NULL; hook(); }
    if (send_error==0) record_radio_rotations(p,n);
    return send_error;
}
static bool hid_write_packet_n(const uint8_t *p) {
    assert(mirrors<ARRAY_SIZE(mirrored)); memcpy(mirrored[mirrors++],p,16);
    if (hid_hook) { void (*hook)(void)=hid_hook; hid_hook=NULL; hook(); }
    bool accepted=(hid_accept_mask & BIT(p[0]))!=0;
    if (accepted && rotation_type(p[0])) {
        assert(hid_rotations<ARRAY_SIZE(hid_rotation_types));
        hid_rotation_types[hid_rotations++]=p[0];
    }
    return accepted;
}
bool tracker_events_select(uint32_t now, uint8_t id, struct tracker_event_tx *out) {
    (void)now; selects++; if(!event_due) return false;
    struct tracker_event e={.nonce=0x12345678,.event_seq=42,.tracker_id=id,.kind=TRACKER_EVENT_KIND_TRACKER_REST,.event=CAL_EVENT_STATE,.phase=TRACKER_REST_REST};
    out->token=91; assert(tracker_event_encode(out->packet,&e)); return true;
}
void tracker_events_complete(uint32_t token, bool success, uint32_t now) {
    assert(token==91 && now==now_ms); completes++; completion_success=success;
}
static atomic_t next_ping_deadline_ms=20000, ping_resync_requested, ping_server_phase_aligned=1;
static uint32_t ping_aligned_interval_ms=1000;
static struct { atomic_t due, attempts, queue_ok, queue_fail, retry_deferred; } ping_sched_stats;
static void atomic_inc(atomic_t *v) { __atomic_fetch_add(v,1,__ATOMIC_SEQ_CST); }
static bool atomic_cas(atomic_t *v,int a,int b) { return __atomic_compare_exchange_n(v,&a,b,false,__ATOMIC_SEQ_CST,__ATOMIC_SEQ_CST); }
static uint32_t get_ping_interval_ms(void) { return 1000; }
static bool tdma_ping_wake_delay_ms(uint32_t *p) { (void)p; return false; }
static bool tdma_stalled;
static bool tdma_admission_stalled(void) { return tdma_stalled; }
static uint32_t ping_server_phase_delay_ms(uint32_t x) { (void)x; return 0; }
static uint8_t connection_get_id(void) { return tracker_id; }
static uint8_t esb_get_ping_ack_flag(void) { return 0; }
static void esb_get_ping_request_data(uint8_t p[4]) { memset(p,0,4); }
static int esb_write_ping(uint8_t *p,bool force) { (void)force; assert(p[0]==ESB_PING_TYPE); ping_calls++; return send_error; }
static void ping_stats_attempt(uint32_t n) { (void)n; }
static uint32_t ping_next_periodic_deadline(uint32_t p,uint32_t n,uint32_t i) { (void)p;return n+i; }
static void esb_ota_periodic_status(void) {}
static void connection_set_ota_suppressed(bool x) { ota_suppressed=x; }
static bool connection_raw_collection_active(void) { return raw; }
static bool connection_process_raw_data(void) { raw_calls++; return raw_pending; }
static uint16_t test_mode_effective_tps(void) { return test_enabled ? target_tps : 0; }
static void connection_idle_wait(int64_t x) { (void)x; }
'''

TESTS = r'''
static void set_clock(uint64_t us) { clock_us=us; now_ms=us/1000; }
static void publish_a(void) {
    const float q[]={0.5f,0.5f,-0.5f,0.5f}, a[]={1.25f,-2.5f,0.125f};
    sensor_data_snapshot_publish_qa(&sensor_data_snapshot,q,a,true);
}
static void publish_zero(void) {
    const float q[]={1,0,0,0}, a[]={0,0,0};
    sensor_data_snapshot_publish_qa(&sensor_data_snapshot,q,a,true);
}
static void publish_m(void) {
    const float m[]={0.25f,-0.5f,1.5f};
    sensor_data_snapshot_publish_m(&sensor_data_snapshot,m);
}
static void publish_m_b(void) {
    const float m[]={2,3,4}; sensor_data_snapshot_publish_m(&sensor_data_snapshot,m);
}
static sensor_data_capture_t capture(void) {
    sensor_data_capture_t value; sensor_data_snapshot_capture(&sensor_data_snapshot,&value); return value;
}
static bool qa_pending(void) { return sensor_data_snapshot_qa_pending(&sensor_data_snapshot); }
static bool m_pending(void) { return sensor_data_snapshot_m_pending(&sensor_data_snapshot); }
static void reset(void) {
    radio=usb=event_due=true; test_enabled=ota=error=raw=raw_pending=false;
    sensor_ids_set=ota_suppressed=tdma_stalled=false; ota_suppress_start_time=0;
    memset(&sensor_data_snapshot,0,sizeof(sensor_data_snapshot));
    publish_zero(); sensor_data_capture_t value=capture();
    sensor_data_snapshot_commit(&sensor_data_snapshot,&value,SENSOR_STREAM_QA);
    memset(&test_rate_schedule,0,sizeof(test_rate_schedule));
    memset(&ping_sched_stats,0,sizeof(ping_sched_stats));
    set_clock(10000000); target_tps=128;
    sends=mirrors=selects=completes=raw_calls=ping_calls=sleeps=0;
    radio_rotations=hid_rotations=captures=0; completion_success=sent_noack=false;
    send_error=last_sleep_ms=0; hid_accept_mask=63; radio_hook=hid_hook=capture_hook=NULL;
    packet_sequence=19; next_ping_deadline_ms=20000; ping_resync_requested=0;
    ping_server_phase_aligned=1; ping_aligned_interval_ms=1000;
    last_mag_time=last_info_time=last_status_time=last_runtime_time=now_ms;
    batt=60; batt_v=94; sensor_temp=133; tracker_svr_status=11; tracker_status=12;
    sent_len=0; memset(sent,0,sizeof(sent)); memset(mirrored,0,sizeof(mirrored));
}
static void test_sensor_update_generation(void) {
    reset();
    memset(sensor_last_q, 0, sizeof(sensor_last_q));
    publication_wakes = 0;
    sensor_data_capture_t before = capture();
    float q[4] = {1, 0, 0, 0}, a[3] = {1, 2, 3};
    connection_update_sensor_data(q, a);
    sensor_data_capture_t first = capture();
    assert(first.qa_generation == before.qa_generation + 1 && qa_pending());
    assert(!first.precise && !memcmp(first.q, q, sizeof(q)) && !memcmp(first.a, a, sizeof(a)));
    assert(publication_wakes == 1);
    connection_update_sensor_data(q, a);
    sensor_data_capture_t second = capture();
    assert(second.qa_generation == first.qa_generation + 1 && second.precise && publication_wakes == 2);
    sensor_data_snapshot_commit(&sensor_data_snapshot, &first, SENSOR_STREAM_QA);
    assert(qa_pending());
    float invalid[4] = {NAN, 0, 0, 0};
    connection_update_sensor_data(invalid, a);
    sensor_data_capture_t after_invalid = capture();
    assert(after_invalid.qa_generation == second.qa_generation && publication_wakes == 2);
    assert(!memcmp(after_invalid.q, q, sizeof(q)));
    sensor_data_snapshot_commit(&sensor_data_snapshot, &second, SENSOR_STREAM_QA);
    assert(!qa_pending());
}
static const uint8_t *body_for(uint8_t wanted) {
    if (sent[0]!=ESB_COMPOSITE_TYPE) return sent[0]==wanted ? sent+2 : NULL;
    size_t pos=3;
    for (unsigned i=0;i<sent[2];++i) {
        uint8_t type=sent[pos++]; const uint8_t *body=sent+pos;
        pos+=wire_body_len(type); assert(pos<sent_len);
        if (type==wanted) return body;
    }
    return NULL;
}
static void assert_a_bytes(void) {
    const uint8_t expected[]={0,64,0,192,0,64,0,64,160,0,192,254,16,0};
    const uint8_t *body=body_for(SUB_PACKET_QUAT_ACCEL);
    assert(body && memcmp(body,expected,sizeof(expected))==0);
}
static void assert_zero_bytes(void) {
    const uint8_t expected[]={0,0,0,0,0,0,255,127,0,0,0,0,0,0};
    const uint8_t *body=body_for(SUB_PACKET_QUAT_ACCEL);
    assert(body && memcmp(body,expected,sizeof(expected))==0);
}
static uint8_t send_builder(struct composite_builder *b) {
    sensor_data_capture_t value=capture(); return send_composite_or_single(b,&value);
}
static void test_protocol_events(void) {
    struct composite_builder b;
    reset(); composite_builder_reset(&b);
    b.has_event=tracker_events_select(now_ms,tracker_id,&b.event);
    assert(b.has_event && send_builder(&b)==0);
    assert(sends==0 && completes==0 && packet_sequence==19);
    reset(); composite_builder_reset(&b);
    assert(send_builder(&b)==0);
    assert(sends==0 && selects==0 && completes==0 && packet_sequence==19);
    assert(!composite_try_add(&b,TRACKER_EVENT_ESB_TYPE));
    b.n=1; b.types[0]=255;
    assert(send_builder(&b)==0 && selects==0 && completes==0 && sends==0);
    composite_builder_reset(&b);
    publish_a();
    assert(composite_try_add(&b,SUB_PACKET_QUAT_ACCEL));
    assert(composite_try_add(&b,SUB_PACKET_INFO));
    assert(send_builder(&b)==(BIT(SUB_PACKET_QUAT_ACCEL)|BIT(SUB_PACKET_INFO)));
    assert(sent_len==49 && sent[0]==ESB_COMPOSITE_TYPE && sent[2]==3 && sent_noack);
    assert(sent[3]==SUB_PACKET_QUAT_ACCEL && sent[18]==SUB_PACKET_INFO && sent[32]==TRACKER_EVENT_ESB_TYPE);
    assert_a_bytes();
    struct tracker_event decoded;
    assert(tracker_event_decode_body(tracker_id,sent+33,15,&decoded));
    assert(decoded.event_seq==42 && sent[48]==19 && packet_sequence==20);
    assert(completes==1 && completion_success && mirrors==2);
    assert(mirrored[0][0]==SUB_PACKET_QUAT_ACCEL && mirrored[1][0]==SUB_PACKET_INFO);
#ifdef HAVE_LEGACY_PARSER
    legacy_count=0; legacy_parse(); assert(legacy_count==2);
    assert(memcmp(legacy_packets[0],mirrored[0],16)==0);
    assert(memcmp(legacy_packets[1],mirrored[1],16)==0);
#endif
    reset(); composite_builder_reset(&b);
    assert(composite_try_add(&b,1) && composite_try_add(&b,0) && composite_try_add(&b,4));
    assert(b.used==44 && send_builder(&b)==(BIT(1)|BIT(0)|BIT(4)));
    assert(sent_len==64 && sent[47]==TRACKER_EVENT_ESB_TYPE && sent[63]==19);
    reset(); composite_builder_reset(&b);
    assert(composite_try_add(&b,1) && composite_try_add(&b,0) && composite_try_add(&b,4) && composite_try_add(&b,3));
    assert(send_builder(&b)==(BIT(1)|BIT(0)|BIT(4)|BIT(3)));
    if (CONFIG_ESB_MAX_PAYLOAD_LENGTH==64) {
        assert(selects==0 && completes==0 && sent[2]==4 && sent_len==51);
    } else {
        assert(selects==1 && completes==1 && completion_success && sent[2]==5 && sent_len==67);
        assert(body_for(TRACKER_EVENT_ESB_TYPE) && sent[66]==19);
    }
    reset(); publish_a(); composite_builder_reset(&b); assert(composite_try_add(&b,1));
    send_error=-EAGAIN; assert(send_builder(&b)==0);
    assert(packet_sequence==19 && mirrors==0 && completes==1 && !completion_success && qa_pending());
    reset(); assert(connection_send_tracker_event());
    assert(sent_len==17 && sent[0]==TRACKER_EVENT_ESB_TYPE && sent_noack);
    assert(packet_sequence==19 && mirrors==0 && completion_success && radio_rotations==0);
    reset(); send_error=-EAGAIN; assert(connection_send_tracker_event());
    assert(packet_sequence==19 && !completion_success && sleeps==1);
    reset(); test_enabled=true; assert(!connection_send_tracker_event()); assert(sends==0 && selects==0);
    reset(); ota=true; assert(!connection_send_tracker_event()); assert(selects==0);
    reset(); error=true; assert(!connection_send_tracker_event()); assert(selects==0);
}
static void test_numerical_encoders(void) {
    reset(); event_due=false; publish_a(); publish_m();
    struct composite_builder b; composite_builder_reset(&b);
    assert(composite_try_add(&b,1) && composite_try_add(&b,4));
    assert(send_builder(&b)==(BIT(1)|BIT(4))); assert_a_bytes();
    const uint8_t mag[]={0,64,0,192,0,64,0,64,0,1,0,254,0,6};
    assert(memcmp(body_for(4),mag,sizeof(mag))==0);
    reset(); event_due=false; const float q[]={1,0,0,0}, a[]={1.25f,-2.5f,0.125f};
    sensor_data_snapshot_publish_qa(&sensor_data_snapshot,q,a,false);
    sensor_ids_set=true; last_info_time=0; radio_hook=publish_zero; iteration();
    const uint8_t compact[]={60,94,133,0,2,16,128,160,0,192,254,16,0};
    assert(body_for(2) && memcmp(body_for(2),compact,sizeof(compact))==0);
    assert(body_for(0) && last_info_time==now_ms && qa_pending());
    iteration(); assert_zero_bytes(); assert(!qa_pending());
}
static void test_capture_after_readiness_hint(void) {
    reset(); event_due=false; iteration(); assert(captures==0 && radio_rotations==0);
    last_status_time=0; capture_hook=publish_a;
    iteration(); assert_a_bytes();
    assert(radio_rotations==1 && radio_rotation_types[0]==1 && body_for(3));
    assert(!qa_pending() && last_status_time==now_ms && captures==1);
}
static void test_refusal_retry_and_newer_publication(void) {
    reset(); event_due=false; publish_zero(); send_error=-EAGAIN;
    iteration(); assert_zero_bytes();
    assert(qa_pending() && radio_rotations==0 && packet_sequence==19);
    send_error=0; iteration(); assert_zero_bytes();
    assert(!qa_pending() && radio_rotations==1 && packet_sequence==20);
    for (unsigned failed=0;failed<2;++failed) {
        reset(); event_due=false; publish_a(); radio_hook=publish_zero;
        send_error=failed ? -EAGAIN : 0; iteration(); assert_a_bytes();
        assert(qa_pending() && packet_sequence==(failed ? 19 : 20));
        send_error=0; iteration(); assert_zero_bytes(); assert(!qa_pending());
        assert(radio_rotations==(failed ? 1 : 2));
    }
}
static void test_independent_streams(void) {
    struct composite_builder b;
    reset(); event_due=false; publish_a(); publish_m();
    composite_builder_reset(&b); assert(composite_try_add(&b,4));
    assert(send_builder(&b)==BIT(4)); assert(qa_pending() && !m_pending());
    composite_builder_reset(&b); assert(composite_try_add(&b,3));
    publish_m(); assert(send_builder(&b)==BIT(3)); assert(qa_pending() && m_pending());
    composite_builder_reset(&b); assert(composite_try_add(&b,1));
    radio_hook=publish_m; assert(send_builder(&b)==BIT(1)); assert(!qa_pending() && m_pending());
    reset(); event_due=false; publish_m(); composite_builder_reset(&b); assert(composite_try_add(&b,4));
    radio_hook=publish_a; assert(send_builder(&b)==BIT(4)); assert(qa_pending() && !m_pending());
    for (unsigned failed=0;failed<2;++failed) {
        reset(); event_due=false; publish_m(); composite_builder_reset(&b);
        assert(composite_try_add(&b,4)); radio_hook=publish_m_b; send_error=failed ? -EAGAIN : 0;
        assert(send_builder(&b)==(failed ? 0 : BIT(4)));
        const uint8_t old_m[]={0,1,0,254,0,6};
        assert(memcmp(body_for(4)+8,old_m,sizeof(old_m))==0 && m_pending());
        send_error=0; assert(send_builder(&b)==BIT(4));
        const uint8_t new_m[]={0,8,0,12,0,16};
        assert(memcmp(body_for(4)+8,new_m,sizeof(new_m))==0 && !m_pending());
    }
    for (unsigned qa_success=0;qa_success<2;++qa_success) {
        reset(); radio=false; event_due=false; publish_a(); publish_m(); last_mag_time=0;
        hid_accept_mask=BIT(qa_success ? 1 : 4); iteration();
        assert(qa_pending()==!qa_success && m_pending()==!!qa_success);
        assert(last_mag_time==(qa_success ? 0 : (int64_t)now_ms));
        assert(hid_rotations==1 && hid_rotation_types[0]==(qa_success ? 1 : 4));
        assert(packet_sequence==19 && sleeps==1 && last_sleep_ms==SEND_RETRY_MS);
    }
    reset(); radio=usb=event_due=false; publish_a(); publish_m(); composite_builder_reset(&b);
    assert(composite_try_add(&b,1) && composite_try_add(&b,4));
    assert(send_builder(&b)==0 && qa_pending() && m_pending());
    assert(sends==0 && mirrors==0 && packet_sequence==19);
}
static void mutate_publications_and_metadata(void) {
    publish_zero(); publish_m_b();
    batt=1; sensor_temp=2; tracker_status=3;
}
static void remove_radio(void) { radio=false; }
static void add_radio(void) { radio=true; }
static void disconnect_usb(void) { usb=false; }
static void test_immutable_bytes_and_radio_authority(void) {
    struct composite_builder b;
    reset(); publish_a(); publish_m(); composite_builder_reset(&b);
    assert(composite_try_add(&b,1) && composite_try_add(&b,4) && composite_try_add(&b,0));
    radio_hook=mutate_publications_and_metadata;
    assert(send_builder(&b)==(BIT(1)|BIT(4)|BIT(0))); assert_a_bytes();
    for (unsigned i=0;i<3;++i) {
        uint8_t type=mirrored[i][0]; unsigned len=wire_body_len(type);
        assert(type==b.types[i] && mirrored[i][1]==tracker_id);
        assert(memcmp(mirrored[i]+2,body_for(type),len)==0);
        if (len==13) assert(mirrored[i][15]==0);
    }
    assert(body_for(0)[0]==60 && body_for(0)[2]==133 && qa_pending() && m_pending());
    reset(); radio=false; event_due=false; publish_a(); publish_m(); composite_builder_reset(&b);
    assert(composite_try_add(&b,1) && composite_try_add(&b,4) && composite_try_add(&b,0));
    hid_hook=mutate_publications_and_metadata;
    assert(send_builder(&b)==(BIT(1)|BIT(4)|BIT(0)));
    const uint8_t qa[]={0,64,0,192,0,64,0,64,160,0,192,254,16,0};
    const uint8_t qm[]={0,64,0,192,0,64,0,64,0,1,0,254,0,6};
    assert(memcmp(mirrored[0]+2,qa,sizeof(qa))==0 && memcmp(mirrored[1]+2,qm,sizeof(qm))==0);
    assert(mirrored[2][2]==60 && mirrored[2][4]==133 && qa_pending() && m_pending());
    assert(sends==0 && selects==0 && completes==0 && packet_sequence==19);
    reset(); event_due=false; publish_a(); hid_accept_mask=0; iteration();
    assert(radio_rotations==1 && hid_rotations==0 && !qa_pending() && packet_sequence==20 && sleeps==0);
    reset(); event_due=false; publish_a(); send_error=-EAGAIN; iteration();
    assert(radio_rotations==0 && hid_rotations==0 && mirrors==0 && qa_pending() && packet_sequence==19);
    reset(); event_due=false; publish_a(); publish_m(); composite_builder_reset(&b);
    assert(composite_try_add(&b,1) && composite_try_add(&b,4)); radio_hook=remove_radio;
    assert(send_builder(&b)==(BIT(1)|BIT(4)));
    assert(!qa_pending() && !m_pending() && packet_sequence==20 && radio_rotations==2);
    reset(); radio=false; publish_a(); publish_m(); composite_builder_reset(&b);
    assert(composite_try_add(&b,1) && composite_try_add(&b,4)); hid_hook=add_radio;
    assert(send_builder(&b)==(BIT(1)|BIT(4)));
    assert(!qa_pending() && !m_pending() && sends==0 && packet_sequence==19 && hid_rotations==2);
}
static void test_hid_disconnect_after_first_acceptance(void) {
    struct composite_builder b;
    reset(); radio=false; publish_a(); publish_m(); last_mag_time=last_status_time=0;
    composite_builder_reset(&b); assert(composite_try_add(&b,1));
    assert(composite_try_add_due(&b,4,true,&last_mag_time,now_ms));
    assert(composite_try_add_due(&b,3,true,&last_status_time,now_ms));
    hid_hook=disconnect_usb; assert(send_builder(&b)==BIT(1));
    assert(hid_rotations==1 && hid_rotation_types[0]==1 && mirrors==1);
    assert(!qa_pending() && m_pending() && last_mag_time==0 && last_status_time==0);
    assert(sends==0 && selects==0 && completes==0 && packet_sequence==19);

    reset(); radio=false; test_enabled=true; target_tps=1;
    publish_a(); last_status_time=last_runtime_time=0; hid_hook=disconnect_usb;
    iteration();
    assert(hid_rotations==1 && hid_rotation_types[0]==1 && mirrors==1 && !qa_pending());
    assert(last_status_time==0 && last_runtime_time==0 && test_rate_schedule.next_due_us==11000000);
    assert(sleeps==1 && last_sleep_ms==SEND_RETRY_MS);
    usb=true; iteration(); assert(hid_rotations==1 && mirrors==1);
}
static void test_normal_service_and_bounds(void) {
    reset(); publish_a(); publish_m(); sensor_ids_set=true;
    last_mag_time=last_info_time=last_status_time=last_runtime_time=0; iteration();
    assert_a_bytes(); assert(body_for(4) && body_for(0) && body_for(3) && body_for(5));
    assert(radio_rotations==2 && radio_rotation_types[0]==1 && radio_rotation_types[1]==4);
    assert(sent_len==60 && sent_len<=CONFIG_ESB_MAX_PAYLOAD_LENGTH && sent[2]==5);
    assert(selects==0 && completes==0 && !qa_pending() && !m_pending());
    assert(last_mag_time==now_ms && last_info_time==now_ms && last_status_time==now_ms && last_runtime_time==now_ms);
    reset(); event_due=false;
    for (unsigned i=0;i<4;++i) {
        set_clock(10000000ULL+100000ULL*i); publish_a(); publish_m(); last_mag_time=0;
        iteration(); assert_a_bytes(); assert(body_for(4));
        assert(radio_rotations==2*(i+1) && !qa_pending() && !m_pending());
    }
}
static void test_hid_partial_test_slot(void) {
    reset(); radio=false; event_due=false; test_enabled=true; target_tps=1;
    publish_a(); publish_m(); last_mag_time=last_status_time=0;
    hid_accept_mask=BIT(1); iteration();
    assert(hid_rotations==1 && hid_rotation_types[0]==1 && !qa_pending() && m_pending());
    assert(last_status_time==0 && last_mag_time==0 && test_rate_schedule.next_due_us==11000000);
    assert(sleeps==1 && last_sleep_ms==SEND_RETRY_MS && packet_sequence==19);
    unsigned attempts=mirrors; iteration(); assert(mirrors==attempts && hid_rotations==1);
    set_clock(11000000); publish_a(); publish_m(); hid_accept_mask=BIT(4)|BIT(3); iteration();
    assert(hid_rotations==2 && hid_rotation_types[1]==4 && qa_pending() && !m_pending());
    assert(last_status_time==11000 && last_mag_time==11000 && test_rate_schedule.next_due_us==12000000);
    reset(); radio=false; event_due=false; test_enabled=true; target_tps=1;
    publish_a(); publish_m(); last_mag_time=last_status_time=0;
    hid_accept_mask=BIT(3); iteration();
    assert(hid_rotations==0 && qa_pending() && m_pending() && last_status_time==10000);
    assert(test_rate_schedule.next_due_us==10000000 && sleeps==1);
    hid_accept_mask=BIT(1)|BIT(3); iteration();
    assert(hid_rotations==1 && hid_rotation_types[0]==1 && !qa_pending());
    assert(test_rate_schedule.next_due_us==11000000);
    set_clock(11000000); publish_a(); publish_m(); hid_accept_mask=63; iteration();
    assert(hid_rotations==2 && hid_rotation_types[1]==4);
}
static void test_low_rate_fair_sharing(void) {
    const unsigned rates[]={1,3};
    for (unsigned r=0;r<ARRAY_SIZE(rates);++r) {
        reset(); test_enabled=true; target_tps=rates[r]; sensor_ids_set=true; next_ping_deadline_ms=1000000;
        last_mag_time=last_info_time=last_status_time=last_runtime_time=0;
        for (unsigned i=0;i<6;++i) {
            uint64_t due=10000000ULL+(uint64_t)i*1000000/rates[r];
            if (i) {
                set_clock(due-1); publish_a(); publish_m(); unsigned attempts=sends; iteration();
                assert(sends==attempts && radio_rotations==i);
            }
            set_clock(due); publish_a(); publish_m();
            if (i==1) {
                unsigned remainder=test_rate_schedule.remainder;
                send_error=-EAGAIN; iteration();
                assert(radio_rotations==i && qa_pending() && m_pending());
                assert(test_rate_schedule.next_due_us==due && test_rate_schedule.remainder==remainder);
                assert(body_for(4) && !body_for(1) && !body_for(2));
                send_error=0;
            }
            iteration();
            assert(radio_rotations==i+1 && radio_rotation_types[i]==(i%2 ? 4 : 1));
            assert(test_rate_schedule.next_due_us==10000000ULL+(uint64_t)(i+1)*1000000/rates[r]);
            assert(!!body_for(1)+!!body_for(2)+!!body_for(4)==1);
            assert(sent_len<=CONFIG_ESB_MAX_PAYLOAD_LENGTH);
            if (i==0) assert(sent_len==61 && body_for(0) && body_for(3) && body_for(5) && body_for(TRACKER_EVENT_ESB_TYPE));
        }
    }
}
static void delay_acceptance(void) { set_clock(clock_us+25000); }
static void test_exact_rates(void) {
    const unsigned rates[]={3,128};
    for (unsigned r=0;r<ARRAY_SIZE(rates);++r) {
        reset(); test_enabled=true; target_tps=rates[r]; next_ping_deadline_ms=1000000; event_due=false;
        publish_a(); test_rate_schedule_sync(clock_us);
        for (unsigned i=0;i<rates[r]+1;++i) {
            uint64_t due=10000000ULL+(uint64_t)i*1000000/rates[r];
            if (i) {
                set_clock(due-1); unsigned attempts=sends; iteration();
                assert(sends==attempts && radio_rotations==i);
            }
            set_clock(due); iteration();
            assert(radio_rotations==i+1 && radio_rotation_types[i]==1);
            assert(test_rate_schedule.next_due_us==10000000ULL+(uint64_t)(i+1)*1000000/rates[r]);
            assert(test_rate_schedule.remainder==((uint64_t)(i+1)*1000000)%rates[r]);
        }
    }
    reset(); test_enabled=true; event_due=false; publish_a(); radio_hook=delay_acceptance; iteration();
    assert(clock_us==10025000 && radio_rotations==1);
    assert(test_rate_schedule.next_due_us==10031250 && test_rate_schedule.remainder==0);
}
static void test_in_mode_reanchors_and_mode_reset(void) {
    for (unsigned late=0;late<2;++late) {
        reset(); test_enabled=true; target_tps=1; event_due=false; next_ping_deadline_ms=1000000;
        publish_a(); publish_m(); last_mag_time=0; iteration();
        assert(radio_rotations==1 && radio_rotation_types[0]==1);
        if (late) set_clock(13000001); else target_tps=3;
        publish_a(); publish_m(); last_mag_time=0; iteration();
        assert(radio_rotations==2 && radio_rotation_types[1]==4 && qa_pending());
        assert(!!body_for(1)+!!body_for(2)+!!body_for(4)==1);
        assert(test_rate_schedule.next_due_us==(late ? 14000001ULL : 10333333ULL));
    }
    reset(); test_enabled=true; target_tps=1; event_due=false;
    publish_a(); publish_m(); last_mag_time=0; iteration();
    test_enabled=false; iteration();
    test_enabled=true; publish_a(); publish_m(); last_mag_time=0;
    unsigned rotations=radio_rotations; iteration();
    assert(radio_rotations==rotations+1 && radio_rotation_types[rotations]==1);
}
static void test_backoff_and_control_priority(void) {
    reset(); publish_a(); event_due=false; send_error=-EAGAIN; tdma_stalled=true;
    iteration(); assert(sends==1 && sleeps==1 && last_sleep_ms==SEND_RETRY_STALLED_MS && qa_pending());
    tdma_stalled=false; iteration(); assert(sends==2 && sleeps==2 && last_sleep_ms==SEND_RETRY_MS);
    reset(); last_status_time=0; event_due=false; send_error=-EAGAIN; tdma_stalled=true;
    iteration(); assert(sends==1 && sleeps==1 && last_sleep_ms==SEND_RETRY_STALLED_MS && last_status_time==0);
    tdma_stalled=false; iteration(); assert(sends==2 && sleeps==2 && last_sleep_ms==SEND_RETRY_MS);
    reset(); raw=raw_pending=true; iteration(); assert(sends==1 && raw_calls==0 && packet_sequence==19);
    reset(); raw=raw_pending=true; next_ping_deadline_ms=now_ms; iteration();
    assert(ping_calls==1 && sends==0 && selects==0 && raw_calls==0 && radio_rotations==0);
    assert(ping_sched_stats.due==1 && ping_sched_stats.attempts==1 && ping_sched_stats.queue_ok==1);
    reset(); raw=raw_pending=true; next_ping_deadline_ms=now_ms; send_error=-EAGAIN; iteration();
    assert(ping_calls==1 && sends==0 && selects==0 && raw_calls==0 && radio_rotations==0);
    assert(ping_sched_stats.due==0 && ping_sched_stats.attempts==0 && next_ping_deadline_ms==now_ms);
    reset(); event_due=false; test_enabled=true; publish_a(); send_error=-EAGAIN; iteration();
    assert(test_rate_schedule.next_due_us==clock_us && radio_rotations==0 && packet_sequence==19);
    send_error=0; iteration(); assert(test_rate_schedule.next_due_us==10007812 && radio_rotations==1 && packet_sequence==20);
}
int main(void) {
    test_sensor_update_generation();
    test_protocol_events(); test_numerical_encoders(); test_refusal_retry_and_newer_publication();
    test_capture_after_readiness_hint();
    test_independent_streams(); test_immutable_bytes_and_radio_authority(); test_normal_service_and_bounds();
    test_hid_disconnect_after_first_acceptance();
    test_hid_partial_test_slot(); test_low_rate_fair_sharing(); test_exact_rates();
    test_in_mode_reanchors_and_mode_reset(); test_backoff_and_control_priority();
    puts("PASS real snapshot/encoders, latest-state ownership, immutable mirrors, partial receipts, actual consumer pose pacing and fair sharing");
    return 0;
}
'''


def build_source(source=None):
    """Assemble production bodies without compiling or adding a test entrypoint."""
    if source is None:
        source = (ROOT / "src/connection/connection.c").read_text()
    header = (ROOT / "src/connection/esb.h").read_text()
    constants = "\n".join(re.findall(r"^#define (?:ESB_|SUB_PACKET_)[^\n]*", header, re.MULTILINE))
    constants += "\n" + "\n".join(re.findall(r"^#define SEND_RETRY_[^\n]*", source, re.MULTILINE))
    utility = (ROOT / "src/util.c").read_text()
    parts = [constants, PREFIX, function(utility, "q_fem"), function(utility, "q_epsilon"),
             function(source, "connection_update_sensor_data")]
    start = source.index("enum sub_packet_type {")
    end = source.index("\nvoid connection_update_sensor_ids(", start)
    parts.append(source[start:end])
    parts.append(block(source, r"^static struct \{(?=\n\s*uint64_t next_due_us;)") + " test_rate_schedule;")
    for name in ("test_rate_schedule_sync", "test_rate_due", "test_rate_advance"):
        parts.append(function(source, name))
    start = source.index("#define COMPOSITE_LOOKAHEAD_MS")
    end = source.index("\nstatic void connection_signal_wake(", start)
    parts.append(source[start:end])
    for name in ("connection_send_retry_ms", "connection_send_tracker_event"):
        parts.append(function(source, name))
    parts.append(function(source, "connection_service_ping"))
    thread = function(source, "connection_thread")
    start = thread.index("\t\tif (radio_ready) {")
    end = thread.rindex("\n\t}")
    parts.append("#define sensor_data_snapshot_capture capture_with_hook\n"
                 + "static void iteration(void) { int64_t now=now_ms; bool radio_ready=esb_ready(); do {\n"
                 + thread[start:end] + "\n} while (0); }\n#undef sensor_data_snapshot_capture")
    return "\n\n".join(parts)


def legacy_parser_source():
    legacy = Path(__file__).with_name("legacy_composite.inc").read_text()
    return r'''
#define HAVE_LEGACY_PARSER 1
#define MIN(a,b) ((a)<(b)?(a):(b))
#define LOG_ERR(...) ((void)0)
static unsigned legacy_count;
static uint8_t legacy_packets[3][16];
static struct packet_stats { uint8_t status_received, status_lost; } tracker_stats[16];
static void legacy_hid(const uint8_t *p,int rssi) {
    (void)rssi; assert(legacy_count<3); memcpy(legacy_packets[legacy_count++],p,16);
}
#define hid_write_packet_n legacy_hid
static void legacy_parse(void) {
    struct { uint8_t data[CONFIG_ESB_MAX_PAYLOAD_LENGTH]; int length, rssi; } rx_payload;
    memcpy(rx_payload.data,sent,sent_len); rx_payload.length=sent_len; rx_payload.rssi=-40;
    uint8_t tracker_id=sent[1], sub_count=sent[2];
''' + legacy + "\n}\n#undef hid_write_packet_n\n"


def run():
    generated = build_source() + legacy_parser_source() + TESTS
    with tempfile.TemporaryDirectory(prefix="tracker-event-transport-") as directory:
        c = Path(directory) / "transport.c"
        c.write_text(generated)
        for payload_length in (64, 72):
            binary = Path(directory) / f"transport-{payload_length}"
            subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
                "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-sign-compare", "-g", "-O1",
                "-fsanitize=address,undefined", "-fno-sanitize-recover=all", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie",
                f"-DCONFIG_ESB_MAX_PAYLOAD_LENGTH={payload_length}",
                "-DCONFIG_CONNECTION_RAW_COLLECTION=1", "-DCONFIG_SENSOR_USE_MAGNETOMETER=1",
                "-I", str(HOST / "concurrency"),
                "-I", str(ROOT / "src"), str(c), "-lm", "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)
    env = dict(os.environ, RADIO_CASE="payload")
    env.pop("RADIO_SCENARIO", None)
    subprocess.run(["python3", str(ROOT / "tests/host/radio_sessions/run.py")], env=env, check=True)


if __name__ == "__main__":
    run()
