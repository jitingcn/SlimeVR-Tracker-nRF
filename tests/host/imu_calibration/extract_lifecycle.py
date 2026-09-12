"""Compile actual caller bodies with hardware leaves stubbed by the host tests."""
from pathlib import Path
import re
import sys

root = Path(__file__).resolve().parents[3]


def function(path, name):
    source = (root / path).read_text()
    match = re.search(rf"^(?:static )?(?:void|int) {name}\([^;]*?\)\s*\{{", source, re.M)
    if not match:
        raise RuntimeError(f"Cannot find {name} in {path}")
    start = match.start()
    body = source.index("{", match.start())
    depth = 0
    # Ignore comments and literals when counting braces; preserve the source verbatim.
    tokens = re.compile(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]', re.S)
    for token in tokens.finditer(source, body):
        if token.group() == "{":
            depth += 1
        elif token.group() == "}":
            depth -= 1
            if depth == 0:
                return source[start:token.end()] + "\n"
    raise RuntimeError(f"Unclosed function {name}")


out = Path(sys.argv[1])
out.mkdir(parents=True, exist_ok=True)
(out / "clear_resume.inc").write_text(
    function("src/system/system.c", "sys_clear")
    + function("src/sensor/sensor.c", "main_imu_resume")
)
(out / "button.inc").write_text(
    function("src/system/system.c", "sys_user_shutdown")
    + function("src/system/system.c", "button_thread")
)
