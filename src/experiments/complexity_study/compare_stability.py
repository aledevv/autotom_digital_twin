"""Compare headless ``*.stability.json`` reports of optimized stage variants.

Usage::

    uv run python src/experiments/complexity_study/compare_stability.py \
      full=/path/d160_pet.stability.json load40=/path/l160_b40.stability.json
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROLES = ("internode", "petiole", "leaf_rachis", "truss_rachis", "pedicel")


def summarize(path: Path) -> dict:
    report = json.loads(path.read_text())
    sag = report.get("dynamic_sag_by_role", {})
    return {
        "passed": not report.get("errors"),
        "max_displacement_m": report.get("max_displacement_m"),
        "max_endpoint_displacement_m": report.get("max_endpoint_displacement_m"),
        **{f"sag_{role}": (sag.get(role) or {}).get("ratio") for role in ROLES},
    }


def main(argv: list[str]) -> int:
    rows = {}
    for item in argv:
        label, _, raw = item.partition("=")
        rows[label] = summarize(Path(raw))
    columns = list(next(iter(rows.values())))
    print("| variant | " + " | ".join(columns) + " |")
    print("|---" * (len(columns) + 1) + "|")
    for label, row in rows.items():
        cells = [
            f"{value:.3f}" if isinstance(value, float) else str(value)
            for value in row.values()
        ]
        print(f"| {label} | " + " | ".join(cells) + " |")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
