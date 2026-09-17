#!/usr/bin/env python3
"""Actual connection encoder and one production scheduler iteration; hardware leaves only."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[3]


def block(source, pattern, semicolon=False):
    match = re.search(pattern, source, re.MULTILINE)
    if match is None:
        raise ValueError(f"Missing production construct: {pattern}")
    start = source.index("{", match.start())
    depth = 0
    for token in re.finditer(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]', source[start:], re.DOTALL):
        if token.group() == "{":
            depth += 1
        elif token.group() == "}":
            depth -= 1
            if depth == 0:
                return source[match.start():start + token.end() + int(semicolon)]
    raise ValueError(pattern)


def function(source, name):
    return block(source, rf"^(?:static\s+)?(?:const struct sub_packet_desc \*|bool|void|int)\s*{name}\([^;{{]*\)\s*\{{")


PREFIX = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "connection/tracker_events.h"
#define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))
#define CONFIG_CONNECTION_OVER_HID 1
#define CONFIG_ESB_MAX_PAYLOAD_LENGTH 64
#define SYS_STATUS_CONNECTION_ERROR 1
#define SYS_STATUS_USB_CONNECTED 2
#define LOG_WRN(...) ((void)0)
#define OTA_SUPPRESS_TIMEOUT_MS 600000
#define PING_QUEUE_RETRY_MS 10
#define COMPOSITE_MAX_SUB_DATA (ESB_MAX_PAYLOAD_LEN - 4)
#define COMPOSITE_LOOKAHEAD_MS 10
static bool radio=true, usb=true, test_enabled, ota, error, event_due=true, raw, raw_pending;
static bool pose_pending, mag_pending, sensor_ids_set;
static bool ota_suppressed;
static int64_t ota_suppress_start_time, dc_conn_error_start;
static uint8_t tracker_id=2, packet_sequence=19;
static bool no_ack=true;
static uint32_t now_ms=10000;
static uint64_t clock_us=10000000;
static int64_t last_mag_time=10000, last_info_time=10000, last_status_time=10000, last_runtime_time=10000;
static unsigned sends, mirrors, selects, completes, raw_calls, ping_calls, sleeps;
static bool completion_success, sent_noack;
static int send_error;
static uint8_t sent[64], mirrored[8][16];
static size_t sent_len;
static uint32_t k_uptime_get_32(void) { return now_ms; }
static uint64_t k_uptime_ticks(void) { return clock_us; }
static uint64_t k_ticks_to_us_near64(uint64_t x) { return x; }
static void k_msleep(int x) { (void)x; sleeps++; }
static void k_usleep(int x) { (void)x; sleeps++; }
static bool esb_ready(void) { return radio; }
static bool esb_ota_is_active(void) { return ota; }
static bool get_status(int x) { return x==SYS_STATUS_USB_CONNECTED ? usb : error; }
static bool test_mode_get(void) { return test_enabled; }
static int esb_write(uint8_t *p, bool noack, size_t n) {
    assert(n<=sizeof(sent)); memcpy(sent,p,n); sent_len=n; sent_noack=noack; sends++; return send_error;
}
static bool hid_write_packet_n(const uint8_t *p) {
    assert(mirrors<8); memcpy(mirrored[mirrors++],p,16); return true;
}
bool tracker_events_select(uint32_t now, uint8_t id, struct tracker_event_tx *out) {
    (void)now; selects++; if(!event_due) return false;
    struct tracker_event e={.nonce=0x12345678,.event_seq=42,.tracker_id=id,.kind=TRACKER_EVENT_KIND_TRACKER_REST,.event=CAL_EVENT_STATE,.phase=TRACKER_REST_REST};
    out->token=91; assert(tracker_event_encode(out->packet,&e)); return true;
}
void tracker_events_complete(uint32_t token, bool success, uint32_t now) {
    assert(token==91 && now==now_ms); completes++; completion_success=success;
}
typedef int64_t atomic_t;
typedef int64_t atomic_val_t;
static atomic_t next_ping_deadline_ms=20000, ping_resync_requested, ping_server_phase_aligned=1;
static uint32_t ping_aligned_interval_ms=1000;
static struct { atomic_t due, attempts, queue_ok, queue_fail, retry_deferred; } ping_sched_stats;
static int64_t atomic_get(atomic_t *v) { return *v; }
static void atomic_set(atomic_t *v, int64_t x) { *v=x; }
static void atomic_inc(atomic_t *v) { ++*v; }
static bool atomic_cas(atomic_t *v,int64_t a,int64_t b) { if(*v!=a)return false;*v=b;return true; }
static uint32_t get_ping_interval_ms(void) { return 1000; }
static bool tdma_ping_wake_delay_ms(uint32_t *p) { (void)p; return false; }
static uint32_t ping_server_phase_delay_ms(uint32_t x) { (void)x; return 0; }
static uint8_t connection_get_id(void) { return tracker_id; }
static uint8_t esb_get_ping_ack_flag(void) { return 0; }
static uint16_t test_mode_get_target_tps(void) { return 100; }
static uint16_t connection_get_data_collection_batch_rate(void) { return 0; }
static void esb_get_ping_request_data(uint8_t p[4]) { memset(p,0,4); }
static int esb_write_ping(uint8_t *p,bool force) { (void)force; assert(p[0]==ESB_PING_TYPE); ping_calls++; return 0; }
static void ping_stats_attempt(uint32_t n) { (void)n; }
static uint32_t ping_next_periodic_deadline(uint32_t p,uint32_t n,uint32_t i) { (void)p;return n+i; }
static void esb_ota_check_timeout(void) {}
static void esb_ota_periodic_status(void) {}
static void connection_set_ota_suppressed(bool x) { ota_suppressed=x; }
static bool connection_raw_collection_active(void) { return raw; }
static void connection_set_data_collection(bool x) { raw=x; }
static int connection_set_data_collection_batch(bool x,uint16_t y) { (void)y;raw=x;return 0; }
static void test_mode_set(bool x) { test_enabled=x; }
static bool connection_process_raw_data(void) { raw_calls++; return raw_pending; }
static int sensor_data_snapshot;
static bool sensor_data_snapshot_qa_pending(void *p) { (void)p;return pose_pending; }
static bool sensor_data_snapshot_m_pending(void *p) { (void)p;return mag_pending; }
static uint16_t test_mode_effective_tps(void) { return test_enabled ? 128 : 0; }
static bool connection_sensor_get_precise_quat(void) { return true; }
static void connection_idle_wait(int64_t x) { (void)x; }
'''

TESTS = r'''
static void reset(void) {
    radio=usb=event_due=true; test_enabled=ota=error=raw=raw_pending=false;
    pose_pending=mag_pending=sensor_ids_set=false;
    now_ms=10000; clock_us=10000000;
    memset(&test_rate_schedule,0,sizeof(test_rate_schedule));
    sends=mirrors=selects=completes=raw_calls=ping_calls=sleeps=0;
    send_error=0; packet_sequence=19; next_ping_deadline_ms=20000;
    last_mag_time=last_info_time=last_status_time=last_runtime_time=now_ms;
}
static void set_clock(uint64_t us) {
    clock_us=us; now_ms=us/1000;
}
static void assert_lowfreq_times(int64_t expected) {
    assert(last_mag_time==expected && last_info_time==expected
        && last_status_time==expected && last_runtime_time==expected);
}
static void assert_lowfreq_wire(bool with_event) {
    assert(sent_noack && sent[0]==ESB_COMPOSITE_TYPE && sent[1]==tracker_id);
    assert(sent[2]==(with_event ? 5 : 4) && sent_len==(with_event ? 61 : 45));
    /* Decode the actual FE layout: full quaternion+mag, status, runtime, IDs. */
    assert(sent[3]==SUB_PACKET_QUAT_MAG && sent[18]==SUB_PACKET_STATUS
        && sent[21]==SUB_PACKET_RUNTIME && sent[30]==SUB_PACKET_INFO);
    if (with_event) {
        struct tracker_event e;
        assert(sent[44]==TRACKER_EVENT_ESB_TYPE);
        assert(tracker_event_decode_body(tracker_id,sent+45,15,&e));
        assert(e.event_seq==42 && e.kind==TRACKER_EVENT_KIND_TRACKER_REST);
    }
    assert(sent[sent_len-1]==19);
}
static void test_lowfreq_target(bool with_event, bool fail_first) {
    reset(); test_enabled=pose_pending=mag_pending=sensor_ids_set=true;
    event_due=with_event;
    last_mag_time=last_info_time=last_status_time=last_runtime_time=8000;
    test_rate_schedule_sync(clock_us);
    test_rate_advance(clock_us);
    const uint64_t target=10007812;
    assert(test_rate_schedule.next_due_us==target && test_rate_schedule.remainder==64);
    set_clock(target-1); iteration();
    assert(sends==0 && selects==0 && completes==0 && mirrors==0);
    assert_lowfreq_times(8000);
    assert(test_rate_schedule.next_due_us==target && test_rate_schedule.remainder==64);
    set_clock(target);
    if (fail_first) {
        send_error=-EAGAIN; iteration();
        assert_lowfreq_wire(with_event);
        assert(sends==1 && packet_sequence==19 && mirrors==0 && sleeps==1);
        assert(completes==(with_event ? 1 : 0));
        if (with_event) assert(!completion_success);
        assert_lowfreq_times(8000);
        assert(test_rate_schedule.next_due_us==target && test_rate_schedule.remainder==64);
        send_error=0;
    }
    iteration();
    assert_lowfreq_wire(with_event);
    assert(sends==(fail_first ? 2 : 1) && packet_sequence==20 && mirrors==4);
    assert(completes==(with_event ? (fail_first ? 2 : 1) : 0));
    if (with_event) assert(completion_success);
    assert_lowfreq_times(10007);
    /* The second 128-TPS interval consumes the fractional microsecond once. */
    assert(test_rate_schedule.next_due_us==10015625 && test_rate_schedule.remainder==0);
    set_clock(10015624); iteration();
    assert(sends==(fail_first ? 2 : 1) && packet_sequence==20 && mirrors==4);
    assert_lowfreq_times(10007);
    set_clock(10015625); iteration();
    assert(sends==(fail_first ? 3 : 2) && packet_sequence==21 && mirrors==5);
    assert(test_rate_schedule.next_due_us==10023437 && test_rate_schedule.remainder==64);
    assert_lowfreq_times(10007);
}
int main(void) {
    struct composite_builder b;
    reset(); composite_builder_reset(&b);
    /* An event-only composite cannot consume a pose sequence or its event. */
    b.has_event=tracker_events_select(now_ms,tracker_id,&b.event);
    assert(!send_composite(&b));
    assert(sends==0 && completes==0 && packet_sequence==19);
    composite_builder_reset(&b);
    assert(!composite_try_add(&b,TRACKER_EVENT_ESB_TYPE));
    assert(composite_try_add(&b,SUB_PACKET_QUAT_ACCEL));
    assert(composite_try_add(&b,SUB_PACKET_INFO));
    assert(send_composite_or_single(&b,SUB_PACKET_QUAT_ACCEL));
    assert(sent_len==49 && sent[0]==ESB_COMPOSITE_TYPE && sent[2]==3);
    assert(sent[3]==SUB_PACKET_QUAT_ACCEL && sent[18]==SUB_PACKET_INFO && sent[32]==TRACKER_EVENT_ESB_TYPE);
    struct tracker_event decoded;
    assert(tracker_event_decode_body(tracker_id,sent+33,15,&decoded));
    assert(decoded.event_seq==42 && sent[48]==19 && packet_sequence==20);
    assert(completes==1 && completion_success && mirrors==2);
    assert(mirrored[0][0]==SUB_PACKET_QUAT_ACCEL && mirrored[1][0]==SUB_PACKET_INFO);
#ifdef HAVE_LEGACY_PARSER
    legacy_parse();
    assert(legacy_count==2);
    assert(memcmp(legacy_packets[0],mirrored[0],16)==0);
    assert(memcmp(legacy_packets[1],mirrored[1],16)==0);
#endif
    reset(); composite_builder_reset(&b);
    /* Exact capacity includes type byte, body, and trailing pose sequence. */
    assert(composite_try_add(&b,1) && composite_try_add(&b,0) && composite_try_add(&b,4));
    assert(b.used==44 && send_composite_or_single(&b,1));
    assert(sent_len==64 && sent[47]==TRACKER_EVENT_ESB_TYPE && sent[63]==19);
    reset(); composite_builder_reset(&b);
    assert(composite_try_add(&b,1) && composite_try_add(&b,0) && composite_try_add(&b,4) && composite_try_add(&b,3));
    assert(send_composite_or_single(&b,1));
    assert(selects==0 && completes==0 && sent[2]==4 && sent_len==51);
    reset(); composite_builder_reset(&b); assert(composite_try_add(&b,1));
    send_error=-EAGAIN; assert(!send_composite_or_single(&b,1));
    assert(packet_sequence==19 && mirrors==0 && completes==1 && !completion_success);
    reset(); assert(connection_send_tracker_event());
    assert(sent_len==17 && sent[0]==TRACKER_EVENT_ESB_TYPE && sent_noack);
    assert(packet_sequence==19 && mirrors==0 && completion_success);
    reset(); send_error=-EAGAIN; assert(connection_send_tracker_event());
    assert(packet_sequence==19 && !completion_success && sleeps==1);
    reset(); test_enabled=true; assert(!connection_send_tracker_event());
    assert(sends==0 && selects==0);
    reset(); radio=false; composite_builder_reset(&b); assert(composite_try_add(&b,1));
    assert(send_composite_or_single(&b,1)); assert(mirrors==1 && selects==0 && sends==0);
    reset(); raw=raw_pending=true; iteration(); assert(sends==1 && raw_calls==0 && packet_sequence==19);
    reset(); raw=raw_pending=true; next_ping_deadline_ms=now_ms; iteration();
    assert(ping_calls==1 && sends==0 && selects==0 && raw_calls==0);
    reset(); test_enabled=pose_pending=true;
    test_rate_schedule_sync(clock_us); test_rate_advance(clock_us);
    for (unsigned wait=0; wait<3; ++wait) {
        iteration();
        assert(sends==0 && selects==0 && completes==0 && test_rate_schedule.next_due_us==10007812);
    }
    set_clock(10007812); iteration();
    assert(sends==1 && test_rate_schedule.next_due_us==10015625 && completion_success);
    assert(sent[0]==ESB_COMPOSITE_TYPE && sent[2]==2
        && sent[3]==SUB_PACKET_QUAT_ACCEL && sent[18]==TRACKER_EVENT_ESB_TYPE);
    reset(); test_enabled=pose_pending=true; send_error=-EAGAIN; iteration();
    assert(sends==1 && test_rate_schedule.next_due_us==clock_us && packet_sequence==19 && !completion_success);
    send_error=0; iteration();
    assert(sends==2 && test_rate_schedule.next_due_us==10007812 && packet_sequence==20 && completion_success);
    assert(sent[0]==ESB_COMPOSITE_TYPE && sent[2]==2 && mirrors==1);
    test_lowfreq_target(false,false);
    test_lowfreq_target(true,false);
    test_lowfreq_target(false,true);
    test_lowfreq_target(true,true);
    reset(); ota=true; assert(!connection_send_tracker_event()); assert(selects==0);
    reset(); error=true; assert(!connection_send_tracker_event()); assert(selects==0);
    puts("PASS transport tail/capacity/failure/mirror/sequence/raw fairness/PING priority/target pose admission");
    puts("PASS test-mode lowfreq wire 4/3/5/0 (45B), optional event (61B), exact 128-TPS deadlines, enqueue failure accounting");
}
'''


def run():
    source = (ROOT / "src/connection/connection.c").read_text()
    enum = block(source, r"^enum sub_packet_type \{", True)
    header = (ROOT / "src/connection/esb.h").read_text()
    constants = "\n".join(re.findall(r"^#define (?:ESB_|SUB_PACKET_)[^\n]*", header, re.MULTILINE))
    constants += "\n" + "\n".join(re.findall(r"^#define SUB_DATA_LEN_[^\n]*", source, re.MULTILINE))
    parts = [constants, enum, PREFIX]
    # Numerical sensor serializers are leaves; actual production type/length table is retained.
    for name, size in (("info", "INFO"), ("quat_accel", "QUAT"), ("compact_quat", "COMPACT"), ("status", "STATUS"), ("mag", "MAG"), ("runtime", "RUNTIME")):
        parts.append(f"static int fill_sub_{name}(uint8_t *p) {{ memset(p, 0x5a, SUB_DATA_LEN_{size}); return SUB_DATA_LEN_{size}; }}")
    parts += [block(source, r"^struct composite_builder \{", True),
              "typedef int (*sub_fill_fn)(uint8_t *buf);",
              block(source, r"^struct sub_packet_desc \{", True),
              block(source, r"^static const struct sub_packet_desc sub_packet_table\[\] = \{", True)]
    for name in ("sub_packet_get", "sub_data_len", "connection_hid_output_ready", "fill_normal_packet", "write_normal_packet", "write_hid_packet_type", "write_hid_composite_as_normal_packets", "connection_write_packet_type", "send_composite", "composite_builder_reset", "composite_try_add", "composite_try_add_due", "composite_commit_timestamps", "send_composite_or_single", "connection_send_tracker_event"):
        parts.append(function(source, name))
    parts.append(block(source, r"^static struct \{\n\s*uint64_t next_due_us;", True)
                 + " test_rate_schedule;")
    for name in ("test_rate_schedule_sync", "test_rate_due", "test_rate_advance"):
        parts.append(function(source, name))
    thread = function(source, "connection_thread")
    start = thread.index("\t\tif (radio_ready) {")
    end = thread.rindex("\n\t}")
    # The production body is unchanged; do/while bounds its continues to one turn.
    parts.append("static void iteration(void) { int64_t now=now_ms; bool radio_ready=esb_ready(); do {\n" + thread[start:end] + "\n} while (0); }")
    legacy = Path(__file__).with_name("legacy_composite.inc").read_text()
    parts.append(r'''
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
    struct { uint8_t data[64]; int length, rssi; } rx_payload;
    memcpy(rx_payload.data,sent,sent_len); rx_payload.length=sent_len; rx_payload.rssi=-40;
    uint8_t tracker_id=sent[1], sub_count=sent[2];
''' + legacy + "\n}\n#undef hid_write_packet_n\n")
    parts.append(TESTS)
    with tempfile.TemporaryDirectory(prefix="tracker-event-transport-") as directory:
        c = Path(directory) / "transport.c"
        binary = Path(directory) / "transport"
        c.write_text("\n\n".join(parts))
        subprocess.run(shlex.split(os.environ.get("CC", "cc")) + ["-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-sign-compare", "-g", "-O1", "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie", "-I", str(ROOT / "src"), str(c), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
    env = dict(os.environ, RADIO_CASE="payload")
    env.pop("RADIO_SCENARIO", None)
    subprocess.run(["python3", str(ROOT / "tests/host/radio_sessions/run.py")], env=env, check=True)


if __name__ == "__main__":
    run()
