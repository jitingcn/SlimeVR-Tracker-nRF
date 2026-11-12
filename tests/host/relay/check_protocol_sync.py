"""Tracker and receiver must carry byte-identical relay wire headers."""
from pathlib import Path

root = Path(__file__).resolve().parents[3]
peer = root.parent / ("pr-receiver" if root.name == "pr-tracker" else "receiver")
header = Path("src/connection/relay_protocol.h")
if not (peer / header).is_file():
    print("Relay protocol parity: receiver checkout absent; local tests still run")
elif (root / header).read_bytes() != (peer / header).read_bytes():
    raise SystemExit(f"Relay protocol headers differ: {root / header} != {peer / header}")
else:
    print("Relay protocol parity: tracker and receiver headers are byte-identical")
