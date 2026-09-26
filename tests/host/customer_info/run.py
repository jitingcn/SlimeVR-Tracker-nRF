#!/usr/bin/env python3
"""Exercise production decoders/readers with host SDK/MMIO adapters.

The board CUSTOMER hook is extracted verbatim, excluding unrelated board boot
code. CRC is independently pinned to standard and frozen upstream vectors.
"""
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
SRC = HERE.parents[2] / "src/system"
BOARDS = HERE.parents[2] / "boards/kemopati"
SK_COMMON = BOARDS / "sk_common"

ADAPTERS = {
    "zephyr/sys/crc.h": r"""
#ifndef HOST_CRC_H
#define HOST_CRC_H
#include <stdint.h>
#include <stddef.h>
static inline uint32_t crc32_ieee(const uint8_t *data, size_t length)
{
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ ((crc & 1) ? UINT32_C(0xedb88320) : 0);
        }
    }
    return ~crc;
}
#endif
""",
    "hal/nrf_uicr.h": r"""
#ifndef HOST_NRF_H
#define HOST_NRF_H
#include <stdint.h>
typedef struct {
    uint32_t reserved[32];
    volatile uint32_t CUSTOMER[32];
} NRF_UICR_Type;
extern NRF_UICR_Type host_uicr;
#define NRF_UICR (&host_uicr)
#endif
""",
    "zephyr/sys/util.h": r"""
#pragma once
#include <stddef.h>
#define BUILD_ASSERT(cond, ...) _Static_assert(cond, #cond)
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
""",
    "zephyr/sys/byteorder.h": r"""
#pragma once
#include <stdint.h>
static inline void sys_put_le32(uint32_t value, uint8_t *dst)
{
    for (unsigned i = 0; i < 4; ++i) { dst[i] = value >> (i * 8); }
}
""",
    "zephyr/sys/printk.h": r"""
#pragma once
int host_print(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
#define printk host_print
#define snprintk snprintf
""",
    "zephyr/logging/log.h": r"""
#pragma once
#include <zephyr/sys/printk.h>
#define LOG_LEVEL_INF 3
#define LOG_MODULE_REGISTER(...)
#define LOG_INF(fmt, ...) host_print(fmt "\n", ##__VA_ARGS__)
""",
}


def board_hook(board, temporary):
    """Compile the actual CUSTOMER hook, not a reimplementation of its logic."""
    source = BOARDS / f"sk_cheesecake_nrf_{board}" / "board.c"
    text = source.read_text()
    match = re.search(
        r"^void\s+customer_info_read_board_variant\s*\([^;{}]*\)\s*\{",
        text,
        re.MULTILINE,
    )
    if match is None:
        raise RuntimeError(f"Cannot locate CUSTOMER hook in {source}")
    depth = 0
    end = None
    # Ignore braces in comments/string literals; preserve the original bytes.
    tokens = re.finditer(
        r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]',
        text[match.end() - 1:],
        re.DOTALL,
    )
    for token in tokens:
        if token.group() == "{":
            depth += 1
        elif token.group() == "}":
            depth -= 1
            if depth == 0:
                end = match.end() - 1 + token.end()
                break
    if end is None:
        raise RuntimeError(f"Unterminated CUSTOMER hook in {source}")
    target = temporary / f"board-{board}.c"
    target.write_text(
        '#include <hal/nrf_uicr.h>\n'
        '#include <zephyr/sys/byteorder.h>\n'
        '#include "customer_info.h"\n'
        '#include "customer_info_skt0.h"\n'
        + text[match.start():end] + "\n"
    )
    return str(target)

with tempfile.TemporaryDirectory(prefix="tracker-customer-info-") as directory:
    temporary = Path(directory)
    for name, source in ADAPTERS.items():
        target = temporary / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(source)
    compiler = shlex.split(os.environ.get("CC") or "cc")
    common = compiler + [
        "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-g", "-O1",
        "-fsanitize=undefined", "-fno-sanitize-recover=all",
        f"-I{temporary}", f"-I{SRC}", str(SRC / "customer_info_parser.c"),
    ]
    smoke = "--smoke" in sys.argv[1:]
    legacy = [f"-I{SK_COMMON}", str(SK_COMMON / "customer_info_skt0.c")]
    if not smoke:
        for name, extra in (("parser", []), ("skt0", legacy)):
            binary = temporary / name
            subprocess.run(common + extra + [
                str(HERE / f"test_{name}.c"), "-o", str(binary),
            ], check=True)
            subprocess.run([str(binary)], check=True)

    # Generic identity settings are deliberately also enabled on SK targets;
    # those settings must never replace SK-local numeric identity (or vice versa).
    variants = [
        ("disabled", None, -1, -1),
        ("blank", None, -1, -1),
        ("unspecified", None, -1, -1),
        ("match", None, -1, -1),
        ("mismatch", None, -1, -1),
    ]
    for board, hardware in (("p00", 0), ("p10", 10)):
        variants.extend([
            ("match", board, 1, hardware),
            ("match", board, 2, hardware),
            ("match", board, -1, hardware),
            ("match", board, 1, -1),
        ])
    if smoke:
        variants = [("disabled", None, -1, -1), ("match", "p00", 1, 0), ("match", "p10", 1, 10)]
    hooks = {}
    for identity, board, sk_product, sk_hardware in variants:
        flags = ["-DCONFIG_CUSTOMER_INFO=1", "-DCONFIG_SOC_SERIES_NRF52=1"]
        if identity != "disabled":
            manufacturer = "" if identity == "blank" else "ACME-7"
            product = -1 if identity == "unspecified" else 0 if identity == "mismatch" else 0x1234
            flags += [
                "-DCONFIG_CUSTOMER_INFO_CHECK_IDENTITY=1",
                f'-DCONFIG_CUSTOMER_INFO_EXPECTED_MANUFACTURER="{manufacturer}"',
                f"-DCONFIG_CUSTOMER_INFO_EXPECTED_PRODUCT_ID={product}",
                "-DCONFIG_CUSTOMER_INFO_EXPECTED_HARDWARE_REVISION=22136",
            ]
        extra = []
        if board is not None:
            if board not in hooks:
                hooks[board] = board_hook(board, temporary)
            flags += [
                "-DCONFIG_CUSTOMER_INFO_SKT0=1",
                f"-DCONFIG_CUSTOMER_INFO_SKT0_PRODUCT_ID={sk_product}",
                f"-DCONFIG_CUSTOMER_INFO_SKT0_HARDWARE_REVISION={sk_hardware}",
            ]
            extra = legacy + [hooks[board]]
        label = f"{identity}-{board or 'generic'}-{sk_product}-{sk_hardware}"
        binary = temporary / f"reader-{label}"
        subprocess.run(common + flags + extra + [
            str(SRC / "customer_info.c"), str(HERE / "test_reader.c"),
            "-o", str(binary),
        ], check=True)
        if smoke:
            print(f"=== {label}: production reader and report ===", flush=True)
        subprocess.run([str(binary), "--smoke" if smoke else label], check=True)
