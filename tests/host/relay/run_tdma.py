#!/usr/bin/env python3
"""Production TDMA relay windows and forwarding-slot admission on a modelled
clock: tdma_slot_data_window, tdma_slot_ping_window, tdma_wait_for_forward_slot
and their independence from the own-slot admission record."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[0] / "harness/python"))
from c_extract import extract_block  # noqa: E402

root = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2]))
tdma = (root / "src/connection/tdma.c").read_text()
esb = (root / "src/connection/esb.c").read_text()
esb_header = (root / "src/connection/esb.h").read_text()
tdma_header = (root / "src/connection/tdma.h").read_text()
network_ticks = (root / "src/connection/network_ticks.h").read_text()


def function(source, name):
    return extract_block(source, rf"^(?:static )?(?:inline )?(?:enum tdma_ping_admission|void|bool|int64_t|uint64_t|uint32_t|uint16_t|uint8_t) {re.escape(name)}\([^;{{]*\)\s*\{{")


PREFIX = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define LOG_WRN(...) ((void)0)
#define LOG_INF(...) ((void)0)
#define ARG_UNUSED(x) ((void)(x))
#define PING_INTERVAL_MS 997
#define TIME_SYNC_TIMEOUT_MS 15000
#define TDMA_MAX_TRACKERS 16
#define TDMA_NUM_TRACKERS 10
#define TDMA_SLOT_TICKS 22
#define TDMA_FRAME_TICKS (TDMA_NUM_TRACKERS * TDMA_SLOT_TICKS)
#define CONFIG_CONNECTION_TDMA 1
#define CONFIG_RELAY_RUNTIME_LAYOUT 1
#define K_USEC(x) ((uint64_t)(x))
typedef int64_t atomic_t;
typedef int64_t atomic_val_t;
static int64_t atomic_get(const atomic_t *p) { return *p; }
static int64_t atomic_set(atomic_t *p, int64_t v) { int64_t old = *p; *p = v; return old; }
enum tdma_ping_admission { TDMA_PING_UNAVAILABLE, TDMA_PING_DEFERRED, TDMA_PING_ADMITTED };
static atomic_t tdma_cfg_pack;
static uint64_t virtual_us;
static uint32_t sleeps, sleep_entry_delay;
#define CONFIG_SYS_CLOCK_TICKS_PER_SEC 1000000U
#define K_TIMEOUT_ABS_TICKS(t) ((uint64_t)(t) | (1ULL << 63))
static unsigned irq_lock(void) { return 0; }
static void irq_unlock(unsigned key) { (void)key; }
static bool server_time_synced = true;
static int64_t g_last_sync_timestamp;
struct esb_sync_filter {
	uint32_t offset; uint32_t last_local_ticks; uint32_t reference_offset;
	uint32_t reference_local_ticks; int32_t skew_ppb; uint32_t update_count; bool initialized;
};
static struct esb_sync_filter sync_filter;
static uint64_t k_uptime_ticks(void) { g_last_sync_timestamp = (int64_t)(virtual_us / 1000); return virtual_us; }
static uint64_t k_ticks_to_ms_floor64(uint64_t ticks) { return ticks / 1000; }
static uint64_t net_ticks_from_kernel64(uint64_t ticks) { return ticks * 32768 / 1000000; }
static uint64_t k_ticks_to_us_floor64(uint64_t ticks) { return ticks; }
static uint64_t k_us_to_ticks_floor64(uint64_t us) { return us; }
static void k_sleep(uint64_t us) { sleeps++; virtual_us += sleep_entry_delay; if (us >> 63) { us &= ~(1ULL << 63); if (virtual_us < us) virtual_us = us; } else virtual_us += us + 31; }
static void k_busy_wait(uint32_t us) { virtual_us += us; }
static uint32_t now_ticks(void) { return (uint32_t)(virtual_us * 32768 / 1000000); }
static void set_ticks(uint32_t ticks) { virtual_us = ((uint64_t)ticks * 1000000 + 32767) / 32768; }
'''


def main():
    parts = [PREFIX, "#include \"frame_ledger.h\"\nint16_t ledger_window_detail, ledger_wait_lateness;",
             (root / "src/connection/relay_protocol.h").read_text(),
             "static bool granted = true; static bool esb_layout_granted(void) { return granted; } bool esb_relay_granted_at(uint32_t tick);",
             function(network_ticks, "net_ticks_to_us_ceil64"),
             function(network_ticks, "net_ticks_to_ms_ceil32"),
             function(network_ticks, "net_ticks_to_ms_floor32"),
             extract_block(esb_header, r"^struct esb_network_clock \{", True),
             extract_block(tdma_header, r"^struct tdma_layout \{", True),
             function(esb, "esb_project_network_ticks"),
             function(esb, "esb_get_network_clock"),
             tdma[tdma.index("#define TDMA_ENABLED_BIT"):tdma.index("\nvoid tdma_init(")],
             tdma[tdma.index("#define TDMA_LEN17_RESERVE_TICKS"):tdma.index("\nvoid tdma_note_radio_busy")],
             re.search(r"^#define TDMA_SLOT_UNASSIGNED[^\n]*", tdma_header, re.MULTILINE)[0],
             function(tdma, "tdma_update_config"),
             tdma[tdma.index("bool tdma_stage_config("):tdma.index("bool tdma_layout_get(")],
             tdma[tdma.index("bool tdma_layout_get("):]]
    parts.append("bool esb_relay_granted_at(uint32_t tick) { return granted && relay_runtime_direct_granted(&relay_direct_anchor, tick); }")
    parts.append((HERE / "tdma_cases.c").read_text())
    with tempfile.TemporaryDirectory() as tmp:
        src = Path(tmp) / "tdma.c"
        src.write_text("\n".join(parts))
        exe = Path(tmp) / "tdma"
        cc = shlex.split(os.environ.get("CC", "cc"))
        for diagnostic in (0, 1):
            flags = ["-DCONFIG_RELAY_FRAME_LEDGER=1"] if diagnostic else []
            subprocess.run(cc + flags + ["-I", str(root / "src"), "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function",
                             "-Wno-unused-variable", "-Wno-unused-parameter", "-Wno-unused-but-set-variable",
                             "-g", "-O1", "-fsanitize=address,undefined", str(src), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
