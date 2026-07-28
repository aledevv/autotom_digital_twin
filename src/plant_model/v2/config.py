# plant_model/v2/config.py

from dataclasses import dataclass, field


# ── World settings (global, non per-organo) ──────────────────────────
PLANT_ROOT_PATH = "/World/Plant"
GLOBAL_SCALE = 10.0   # scale up the plant by 10x, mitigates small-number instability


@dataclass
class OrganConfig:
    """Per-organ toggles for physics + segmentation, used for stability testing."""
    physics_enabled: bool = False
    max_segments: int = 1


@dataclass
class SimulationConfig:
    """Single knob panel for the whole build — one section per organ type."""
    stem: OrganConfig = field(default_factory=lambda: OrganConfig(physics_enabled=True, max_segments=2))
    leaf: OrganConfig = field(default_factory=lambda: OrganConfig(physics_enabled=True, max_segments=2))
    fruit: OrganConfig = field(default_factory=lambda: OrganConfig(physics_enabled=False, max_segments=2))
    # branch, truss: aggiungi qui quando sai cosa ti serve, stesso pattern


DEFAULT_CONFIG = SimulationConfig()