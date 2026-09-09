"""One configuration for both production leaflet generators."""
from dataclasses import dataclass
from pathlib import Path
import math

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "profiles/leaf_shape.yaml"
RESOURCES = PROJECT_ROOT / "external/real_leaves/src/tomato_leaf_generator/resources"
BACKENDS = ("gaussian", "i3", "legacy")
ROLE_MAP = {"petiolule_left": "left", "petiolule_right": "right", "rachis_terminal": "terminal"}


@dataclass(frozen=True)
class LeafShapeConfig:
    backend: str
    seed: int
    timeout_seconds: float
    generator: dict

    def __post_init__(self):
        if self.backend not in BACKENDS:
            raise ValueError(f"leaf shape backend must be one of {BACKENDS}")
        if type(self.seed) is not int or self.seed < 0:
            raise ValueError("leaf shape seed must be a non-negative integer")
        if isinstance(self.timeout_seconds, bool) or not isinstance(self.timeout_seconds, (int, float)):
            raise ValueError("leaf shape timeout_seconds must be a positive finite number")
        if not math.isfinite(self.timeout_seconds) or self.timeout_seconds <= 0:
            raise ValueError("leaf shape timeout_seconds must be positive and finite")
        if not isinstance(self.generator, dict):
            raise ValueError("leaf shape generator must be a mapping")
        for key in ("model", "bank"):
            if not isinstance(self.generator.get(key), str) or not self.generator[key]:
                raise ValueError(f"leaf shape {key} must be a non-empty resource path")
        for key in ("neighbours", "maximum_weight_draws", "maximum_geometry_attempts", "variation_seed_offset"):
            value = self.generator.get(key)
            minimum = 0 if key == "variation_seed_offset" else 1
            if type(value) is not int or value < minimum:
                raise ValueError(f"leaf shape {key} must be an integer >= {minimum}")
        for key in ("dirichlet_alpha", "maximum_raw_weight", "maximum_expansion"):
            value = self.generator.get(key)
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f"leaf shape {key} must be a finite number")
        # Backend-specific admissible values are checked by the production factory.


def load_leaf_shape_config(path=None, *, backend=None, seed=None):
    defaults = yaml.safe_load(DEFAULT_CONFIG.read_text())
    if path is not None:
        try:
            selected = yaml.safe_load(Path(path).expanduser().read_text())
        except yaml.YAMLError as exc:
            raise ValueError(f"invalid leaf shape YAML: {exc}") from exc
        if not isinstance(selected, dict):
            raise ValueError("leaf shape YAML must be a mapping")
        unknown = selected.keys() - defaults.keys()
        if unknown:
            raise ValueError(f"unknown leaf shape settings: {sorted(unknown)}")
        generator = selected.get("generator", {})
        if not isinstance(generator, dict):
            raise ValueError("leaf shape generator must be a mapping")
        unknown = generator.keys() - defaults["generator"].keys()
        if unknown:
            raise ValueError(f"unknown generator settings: {sorted(unknown)}")
        defaults = {**defaults, **selected, "generator": {**defaults["generator"], **generator}}
    if backend is not None:
        defaults["backend"] = backend
    if seed is not None:
        defaults["seed"] = seed
    return LeafShapeConfig(**defaults)
