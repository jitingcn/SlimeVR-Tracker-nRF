"""Compile actual lifecycle bodies, preserving the production scheduler calls."""
from pathlib import Path
import re
import sys


def function(source, name):
    match = re.search(rf"^(?:static )?(?:void|int) {name}\([^;]*?\)\s*\{{", source, re.M)
    if not match:
        raise RuntimeError(f"Cannot find {name}")
    body = source.index("{", match.start())
    depth = 0
    tokens = re.compile(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]', re.S)
    for token in tokens.finditer(source, body):
        if token.group() == "{":
            depth += 1
        elif token.group() == "}":
            depth -= 1
            if depth == 0:
                return source[match.start():token.end()] + "\n"
    raise RuntimeError(f"Unclosed function {name}")


source = Path(sys.argv[1]).read_text()
Path(sys.argv[2]).write_text(
    function(source, "sensor_loop_wait") + function(source, "main_imu_resume")
)
