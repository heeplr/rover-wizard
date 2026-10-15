"""List unexplained number literals inside functions: not 0/1/-1/2, not self-describing (a keyword argument like
min_value=... or a dictionary value under a string key like {"SERVO1_MIN": 1100}), and without a comment on the
same line or the line above."""

import ast
import sys
from pathlib import Path

# (0.0, 1.0, ... are equal to these and match too)
TRIVIAL = {0, 1, -1, 2}


def findings(path: Path) -> list:
    """(line, value) of each unexplained literal in one file."""
    source = path.read_text(encoding="utf-8")
    lines = source.splitlines()
    tree = ast.parse(source)
    keyword_values = {
        id(keyword.value)
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        for keyword in node.keywords
        if keyword.arg
    }
    # A dictionary value under a string key is labelled by that key (e.g. {"SERVO1_MIN": 1100}).
    keyword_values |= {
        id(value)
        for node in ast.walk(tree)
        if isinstance(node, ast.Dict)
        for key, value in zip(node.keys, node.values)
        if isinstance(key, (ast.Constant, ast.JoinedStr))
    }
    # A negative literal (-90) is "minus" applied to 90: it is labelled like the value it belongs to.
    keyword_values |= {
        id(node.operand) for node in ast.walk(tree) if isinstance(node, ast.UnaryOp) and id(node) in keyword_values
    }
    results = []
    for function in (n for n in ast.walk(tree) if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda))):
        for node in ast.walk(function):
            if not (isinstance(node, ast.Constant) and type(node.value) in (int, float)) or node.value in TRIVIAL:
                continue
            if id(node) in keyword_values:
                continue
            line = node.lineno
            explained = "#" in lines[line - 1] or (line >= 2 and lines[line - 2].strip().startswith("#"))
            if not explained:
                results.append((line, node.value))
    return sorted(set(results))


# Also usable on its own: python tests/magic_numbers.py rover_wizard
if __name__ == "__main__":
    total = 0
    target = Path(sys.argv[1])
    for path in [target] if target.is_file() else sorted(target.rglob("*.py")):
        found = findings(path)
        total += len(found)
        for line, value in found:
            print(f"{path}:{line}: {value}")
    print("unexplained literals:", total)
