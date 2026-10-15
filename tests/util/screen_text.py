"""Turn a Textual SVG screenshot into plain text (for reading layouts without an SVG viewer)."""

import html
import re

# Textual's SVG export grid:
# pixels per character column
CHAR_WIDTH = 12.2
# pixels per text row
LINE_HEIGHT = 24.4
# y of the first row's baseline
FIRST_BASELINE = 20.0


def svg_to_text(svg: str) -> str:
    """Place every text run of the screenshot on a character grid (column = x / 12.2, row = (y - 20) / 24.4)."""
    rows: dict = {}
    for x, y, text in re.findall(r'<text class="[^"]*" x="([\d.]+)" y="([\d.]+)"[^>]*>([^<]*)</text>', svg):
        row = round((float(y) - FIRST_BASELINE) / LINE_HEIGHT)
        column = round(float(x) / CHAR_WIDTH)
        line = rows.setdefault(row, {})
        for offset, char in enumerate(html.unescape(text).replace("\xa0", " ")):
            line[column + offset] = char
    width = max((max(line) + 1 for line in rows.values() if line), default=0)
    return "\n".join(
        "".join(rows.get(row, {}).get(column, " ") for column in range(width)).rstrip()
        for row in range(max(rows) + 1 if rows else 0)
    )


if __name__ == "__main__":
    import pathlib
    import sys

    print(svg_to_text(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8")))
