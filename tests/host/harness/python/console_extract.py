"""Extract the production command table and feedback classifier for host workers."""
import re

from c_extract import extract_block


def console_table(source, handlers, fallback="console_unrelated_command"):
    start = source.index("typedef void (*console_cmd_fn)")
    declaration = source[start:source.index("\n};", start) + 3]
    start = source.index("static const struct console_cmd console_cmds[]")
    table = source[start:source.index("\n};", start) + 3]
    names = set(re.findall(r'\{"[^"]+",\s*(console_cmd_\w+)', table))
    redirects = [f"#define {name} {fallback}" for name in sorted(names - set(handlers))]
    lookup = extract_block(source, r"^static const struct console_cmd \*console_find_command\([^;{]*\)\s*\{")
    classify = extract_block(source, r"^static bool console_command_mutates\([^;{]*\)\s*\{")
    return "\n\n".join([declaration, *redirects, table, lookup, classify])
