"""Behavioral lexical fixtures; deliberately independent of production source."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent / "python"))
from c_extract import extract_block


FUNCTION = r"^static int selected\(void\)\s*\{"


class ExtractBlockTests(unittest.TestCase):
    def test_preserves_nested_body_and_lexical_braces(self):
        body = r'''static int selected(void)
{
    char close = '}';
    char open = '{';
    char quote = '\'';
    char slash = '\\';
    const char *text = "escaped \" } /* and {";
    /* } { */
    // } {
    if (open) { return close; }
    return 0;
}'''
        self.assertEqual(extract_block("int before;\n" + body + "\nint after;", FUNCTION), body)

    def test_ignores_fake_definitions_in_comments_and_literals(self):
        body = "static int selected(void) { return 1; }"
        source = '/*\nstatic int selected(void) { }\n*/\n'
        source += 'const char *s = "static int selected(void) { }";\n'
        self.assertEqual(extract_block(source + body, FUNCTION), body)

    def test_escaped_newline_in_literal_and_line_comment(self):
        body = 'static int selected(void) {\nchar *s = "a\\\n}b";\n// } \\\n{ ignored\nreturn 0;\n}'
        self.assertEqual(extract_block(body, FUNCTION), body)

    def test_crlf_and_escaped_crlf(self):
        body = 'static int selected(void) {\r\n// } \\\r\n{ ignored\r\nreturn 0;\r\n}'
        self.assertEqual(extract_block(body, FUNCTION), body)

    def test_semicolon_preserves_intervening_trivia(self):
        body = 'struct item { int value; } /* comment } */\n ;'
        self.assertEqual(extract_block(body + '\nint other;', r'^struct item \{', True), body)
        self.assertEqual(extract_block(body, r'^struct item \{'), 'struct item { int value; }')

    def test_signature_variants_are_caller_selected(self):
        body = 'static inline unsigned selected(\n int value\n)\n{ return value; }'
        pattern = r'^static inline unsigned selected\([^;{]*\)\s*\{'
        self.assertEqual(extract_block(body, pattern), body)
        with self.assertRaises(ValueError):
            extract_block(body, FUNCTION)

    def test_explicit_signature_excludes_call_conditions(self):
        body = 'static int selected(void) { return 1; }'
        source = body + '\nvoid caller(void) {\n    if (selected()) {}\n    while (selected()) {}\n}'
        self.assertEqual(extract_block(source, FUNCTION), body)
        # A broad return-type selector is not silently resolved to its first hit.
        with self.assertRaises(ValueError):
            extract_block(source, r'^(?:static )?[^;\n]+\bselected\([^;{]*\)\s*\{')

    def test_missing_ambiguous_and_prototype(self):
        for source in ('', 'static int selected(void);',
                       'static int selected(void) {}\nstatic int selected(void) {}',
                       '#if A\nstatic int selected(void) {}\n#else\nstatic int selected(void) {}\n#endif'):
            with self.subTest(source=source), self.assertRaises(ValueError):
                extract_block(source, FUNCTION)

    def test_unclosed_lexical_constructs(self):
        for suffix in ('/* missing', '"missing', "'missing", '"escape\\', "'escape\\", '"raw\nnewline"'):
            with self.subTest(suffix=suffix), self.assertRaises(ValueError):
                extract_block('static int selected(void) { ' + suffix + ' }', FUNCTION)

    def test_unclosed_nested_block(self):
        with self.assertRaises(ValueError):
            extract_block('static int selected(void) { if (1) { }', FUNCTION)

    def test_required_semicolon_is_not_arbitrary_next_byte(self):
        for suffix in ('', ' x', ' /* missing'):
            with self.subTest(suffix=suffix), self.assertRaises(ValueError):
                extract_block('struct item { int x; }' + suffix, r'^struct item \{', True)

    def test_selector_requires_opening_brace(self):
        for pattern in (r'^static int selected', r'^static int selected.*\{.*\{'):
            with self.subTest(pattern=pattern), self.assertRaises(ValueError):
                extract_block('static int selected(void) { if (1) { } }', pattern)


if __name__ == '__main__':
    unittest.main()
