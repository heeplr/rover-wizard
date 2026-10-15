"""Code-quality rules that keep holding: no unexplained number literals, no trailing comments.

Running this file lists the offending lines (pytest tests/test_code_quality.py).
"""

from pathlib import Path

from util.magic_numbers import findings

ROOT = Path(__file__).resolve().parent.parent


def test_no_unexplained_number_literals():
    unexplained = [
        f"{path.relative_to(ROOT)}:{line}: {value}"
        for path in sorted((ROOT / "rover_wizard").rglob("*.py"))
        for line, value in findings(path)
    ]
    assert not unexplained, "name these values or explain them in a comment:\n" + "\n".join(unexplained)


def test_comments_stand_on_their_own_line():
    # Comments go on the line above the code (formatters re-space trailing comments); lint pragmas stay inline.
    import io
    import tokenize

    trailing = []
    # dev_tools/ is optional: a checkout may contain only the package and the tests
    for root in (ROOT / name for name in ("rover_wizard", "tests", "dev_tools")):
        for path in sorted(root.rglob("*.py")) if root.is_dir() else []:
            source = path.read_text(encoding="utf-8")
            lines = source.split("\n")
            for token in tokenize.generate_tokens(io.StringIO(source).readline):
                row, column = token.start
                if (
                    token.type == tokenize.COMMENT
                    and lines[row - 1][:column].strip()
                    and not any(pragma in token.string for pragma in ("noqa", "type:", "pragma"))
                ):
                    trailing.append(f"{path.relative_to(ROOT)}:{row}")
    assert not trailing, "move these comments to the line above:\n" + "\n".join(trailing)
