"""rover_wizard.ui.markdown - the Markdown parser used for help and intro texts."""

from __future__ import annotations

from markdown_it import MarkdownIt


def markdown_parser() -> MarkdownIt:
    """Textual's default "gfm-like" parser without link auto-detection.

    Auto-detecting URLs needs the optional linkify-it-py package; the wizard's texts don't need it, so switching it
    off lets the help texts render with or without that package installed.
    """
    return MarkdownIt("gfm-like", {"linkify": False})
