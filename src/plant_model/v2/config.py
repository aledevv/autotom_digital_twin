# plant_model/v2/config.py

from dataclasses import dataclass


# ── World settings ────────────────────────────────────────────────────
PLANT_ROOT_PATH = "/World/Plant"
GLOBAL_SCALE = 10.0   # scale up the plant by 10x, mitigates small-number instability


@dataclass
class StemPhysicsConfig:
    physics_enabled: bool = False   # anchor-only; enables joints on attached children
    mass_per_segment: float = 1.0


@dataclass
class LeafPhysicsConfig:
    physics_enabled: bool = True
    num_petiole_segments: int = 1     # 1 = rigid petiole, >1 = articulated chain
    stiffness_base: float = 50.0
    stiffness_tip: float = 10.0
    damping_ratio: float = 0.7
    max_bend_angle: float = 30.0      # per-joint bend limit (deg)
    twist_limit: float = 15.0         # per-joint twist limit (deg)
    density: float = 200.0            # petiole cylinder density (kg/m³)


@dataclass
class SimulationConfig:
    stem: StemPhysicsConfig = StemPhysicsConfig()
    leaf: LeafPhysicsConfig = LeafPhysicsConfig()


DEFAULT_CONFIG = SimulationConfig()
