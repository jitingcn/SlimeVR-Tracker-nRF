"""Compare private wire headers when the companion checkout is available."""
from pathlib import Path


root = Path(__file__).resolve().parents[3]
peer = root.parent / "SlimeVR-Tracker-nRF-Receiver"
header = Path("src/connection/tracker_event_protocol.h")
if not peer.is_dir():
    print("Protocol parity: companion checkout absent; local golden tests still run")
elif (root / header).read_bytes() != (peer / header).read_bytes():
    raise SystemExit(f"Protocol headers differ: {root / header} != {peer / header}")
else:
    print("Protocol parity: tracker and receiver headers are byte-identical")
