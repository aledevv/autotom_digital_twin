"""Run in a Python 3.12 environment with real_leaves dependencies installed."""

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "external/real_leaves/src"))
from tomato_leaf_generator import generate_leaf

p = argparse.ArgumentParser()
p.add_argument("--output", type=Path, required=True)
p.add_argument("--seed", type=int, default=42)
a = p.parse_args()
if a.output.exists():
    raise FileExistsError(a.output)
leaf = generate_leaf(seed=a.seed, length_m=0.09, pose="flat")
# +Y tip -> +X by a proper rotation; preserve face winding and SI dimensions.
points = np.column_stack(
    (leaf.vertices[:, 1], -leaf.vertices[:, 0], leaf.vertices[:, 2] + 0.15)
)
a.output.parent.mkdir(parents=True, exist_ok=True)
np.savez_compressed(a.output, points=points, faces=leaf.faces)
a.output.with_suffix(".json").write_text(
    json.dumps(
        {
            "seed": a.seed,
            "pose": "flat",
            "length_m": 0.09,
            "generator": "tomato_leaf_generator.generate_leaf",
            "sha256": hashlib.sha256(a.output.read_bytes()).hexdigest(),
            "shape_provenance": dict(leaf.metadata["shape"]),
        },
        indent=2,
        default=str,
    )
    + "\n"
)
print(a.output)
