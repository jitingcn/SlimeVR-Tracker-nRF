"""Small lexical C block extractor for host fixtures, not a C parser.

Callers own the definition/struct selector regex (including its opening brace).
No preprocessing is performed: duplicate conditional definitions are ambiguous.
Returned text is always an unchanged slice of the supplied source.
"""
import re


def _code_mask(source):
    """Hide comments/literals while preserving offsets and physical newlines."""
    masked = list(source)
    i = 0
    size = len(source)
    while i < size:
        start = i
        if source.startswith("/*", i):
            end = source.find("*/", i + 2)
            if end < 0:
                raise ValueError(f"Unterminated C comment at offset {start}")
            i = end + 2
        elif source.startswith("//", i):
            i += 2
            while i < size:
                if source.startswith("\\\r\n", i):
                    i += 3
                elif source.startswith("\\\n", i):
                    i += 2
                elif source[i] in "\r\n":
                    break
                else:
                    i += 1
        elif source[i] in "\"'":
            quote = source[i]
            i += 1
            while i < size:
                if source[i] == quote:
                    i += 1
                    break
                if source[i] in "\r\n":
                    raise ValueError(f"Unterminated C literal at offset {start}")
                if source[i] == "\\":
                    if source.startswith("\\\r\n", i):
                        i += 3
                    else:
                        i += 2
                else:
                    i += 1
            else:
                raise ValueError(f"Unterminated C literal at offset {start}")
        else:
            i += 1
            continue
        for offset in range(start, i):
            if source[offset] not in "\r\n":
                masked[offset] = " "
    return "".join(masked)


def extract_block(source, pattern, semicolon=False):
    """Extract one regex-selected balanced block, failing closed on mismatch.

    The selector must end at the construct's opening ``{``. ``semicolon``
    requires and includes a following semicolon, allowing intervening trivia.
    Signatures and return types deliberately remain a caller responsibility.
    """
    code = _code_mask(source)
    matches = list(re.finditer(pattern, code, re.MULTILINE))
    if len(matches) != 1:
        raise ValueError(f"Expected one C construct, found {len(matches)}: {pattern}")
    match = matches[0]
    opening = match.end() - 1
    if opening < match.start() or code[opening] != "{" or "{" in code[match.start():opening]:
        raise ValueError(f"Selector must end at the first opening brace: {pattern}")
    depth = 0
    for end in range(opening, len(code)):
        if code[end] == "{":
            depth += 1
        elif code[end] == "}":
            depth -= 1
            if depth == 0:
                end += 1
                if semicolon:
                    while end < len(code) and code[end].isspace():
                        end += 1
                    if end == len(code) or code[end] != ";":
                        raise ValueError(f"Missing C construct semicolon: {pattern}")
                    end += 1
                return source[match.start():end]
    raise ValueError(f"Unclosed C construct: {pattern}")
