#!/usr/bin/env python3
"""Exercise actual flash CRC, VERIFY and ACTIVATE bodies with injected flash I/O.

OTA_SOURCE_ROOT selects a saved src tree; OTA_LEGACY_CRC32=1 supports replaying
pre-fix source with its old helper signature. No production logic is rewritten.
"""

import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile
import zlib

HERE = Path(__file__).resolve().parent
SRC = Path(os.environ.get("OTA_SOURCE_ROOT", HERE.parents[2] / "src"))


def block(source, pattern, semicolon=False):
    """Use the host OTA/power convention: skip comments and string braces."""
    match = re.search(pattern, source, re.MULTILINE)
    if match is None:
        raise ValueError(f"Production construct not found: {pattern}")
    start = source.index("{", match.start())
    tokens = re.compile(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]', re.DOTALL)
    depth = 0
    for token in tokens.finditer(source, start):
        if token.group() == "{":
            depth += 1
        elif token.group() == "}":
            depth -= 1
            if depth == 0:
                return source[match.start():token.end() + int(semicolon)]
    raise ValueError(f"Unclosed production construct: {pattern}")


def function(source, name):
    return block(source, rf"^(?:static )?(?:int|uint8_t|uint32_t) {name}\([^;{{]*\)\s*\{{")


ota = (SRC / "system/esb_ota.c").read_text()
flash = (SRC / "system/esb_ota_flash.c").read_text()
header = (SRC / "system/esb_ota.h").read_text()
parts = ["\n".join(re.findall(r"^#define OTA_.*$", header, re.MULTILINE)),
         block(ota, r"^enum ota_state \{", True),
         block(ota, r"^struct ota_context \{", True),
         "static struct ota_context ota;", "static atomic_t ota_reboot_pending;",
         function(flash, "esb_ota_flash_compute_crc32")]
for name in ("esb_ota_get_status", "esb_ota_handle_verify", "esb_ota_handle_activate"):
    parts.append(function(ota, name))

with tempfile.TemporaryDirectory(prefix="tracker-ota-crc32-") as directory:
    temporary = Path(directory)
    (temporary / "production.inc").write_text("\n\n".join(parts))
    # Independent known vectors check the host CRC primitive, including chunking.
    vectors = f"#define LONG_IMAGE_CRC UINT32_C(0x{zlib.crc32(bytes([0xFF]) * 4100):08x})\n"
    (temporary / "vectors.h").write_text(vectors)
    for mcuboot in (1, 0):
        binary = temporary / f"ota-crc32-{mcuboot}"
        command = shlex.split(os.environ.get("CC", "cc")) + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-g", "-O1",
            "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie",
            f"-DOTA_USE_MCUBOOT={mcuboot}",
            f"-DOTA_LEGACY_CRC32={int(os.environ.get('OTA_LEGACY_CRC32', '0'))}",
            "-I", str(temporary), "-I", str(SRC), str(HERE / "test_ota_crc32.c"),
            "-o", str(binary),
        ]
        subprocess.run(command, check=True)
        subprocess.run([str(binary)], check=True)
