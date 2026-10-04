#!/usr/bin/env python3
"""Execute production radio init, deadline and CRC16 bodies with failure injection."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'harness/python'))
from c_extract import extract_block as block

HERE = Path(__file__).resolve().parent
SRC = Path(os.environ.get("RADIO_OTA_SOURCE_ROOT", HERE.parents[2] / "src"))


def function(source, name):
    return block(source, rf"^(?:static )?(?:int64_t|uint16_t|int|void) {re.escape(name)}\([^;{{]*\)\s*\{{")


esb = (SRC / "connection/esb.c").read_text()
connection = (SRC / "connection/connection.c").read_text()
flash = (SRC / "system/esb_ota_flash.c").read_text()
header = (SRC / "system/esb_ota.h").read_text()
parts = [function(esb, "esb_deinitialize"), function(esb, "esb_initialize"),
         function(connection, "connection_next_deadline_ms"),
         function(connection, "connection_idle_wait"),
         block(header, r"^struct bootloader_settings \{") + ";",
         "static struct bootloader_settings prepared_bl_settings; static bool bl_settings_prepared;",
         function(flash, "esb_ota_flash_compute_crc16_nordic"),
         function(flash, "esb_ota_flash_prepare_bootloader_settings")]
legacy = "uint16_t esb_ota_flash_compute_crc16_nordic(" in flash
if not legacy:
    parts += [block(flash, r"^struct flash_copy_params \{") + ";",
              "static void ota_flash_copy_from_ram(const struct flash_copy_params *p);",
              function(flash, "esb_ota_flash_copy_and_reset")]
with tempfile.TemporaryDirectory(prefix="radio-ota-findings-") as directory:
    temporary = Path(directory)
    (temporary / "production.inc").write_text("\n\n".join(parts))
    for diagnostics in (0, 1):
        binary = temporary / f"test-{diagnostics}"
        command = shlex.split(os.environ.get("CC", "cc")) + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
            "-g", "-O1", "-fsanitize=address,undefined", "-fno-pie", "-no-pie",
            f"-DLEGACY_CRC16={int(legacy)}", "-I", str(temporary),
            str(HERE / "test.c"), "-o", str(binary)]
        if diagnostics:
            command.append("-DCONFIG_TDMA_DIAGNOSTICS=1")
        subprocess.run(command, check=True)
        cases = os.environ.get("RADIO_OTA_CASES", "init deadline crc16" + ("" if legacy else " copy")).split()
        for case in cases:
            subprocess.run([str(binary), case], check=True)
