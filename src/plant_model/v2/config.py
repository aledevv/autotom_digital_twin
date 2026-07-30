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
    num_petiole_segments: int = 8     # 1 = rigid petiole, >1 = articulated chain
    youngs_modulus: float = 2.0e7     # Young's Modulus E (N/m^2) for herbaceous tomato stems (softer than wood)
    damping_ratio: float = 0.2        # Slightly higher damping so it settles smoothly
    wood_density: float = 800.0       # Mass density (kg/m³)
    max_bend_angle: float = 60.0      # Maximum bending angle [degrees] before hard limits kick in
    # ── Debug toggles ────────────────────────────────────────────────
    petiole_collision: bool = True     # set False to disable CollisionAPI on petiole cylinders (Test A)
    debug_physics: bool = True        # print mass/stiffness/damping for every segment at build time


@dataclass
class SimulationConfig:
    stem: StemPhysicsConfig = StemPhysicsConfig()
    leaf: LeafPhysicsConfig = LeafPhysicsConfig()


DEFAULT_CONFIG = SimulationConfig()
