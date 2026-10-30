#!/usr/bin/env python3
"""Extract the connection owner's IMU mapping for the scanner fixture."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "harness/python"))
from c_extract import extract_block

source = Path(sys.argv[1]).read_text()
mapping = extract_block(source, r"^static uint8_t get_server_constant_imu_id\([^;{}]*\)\s*\{")
Path(sys.argv[2]).write_text(mapping + "\n")
