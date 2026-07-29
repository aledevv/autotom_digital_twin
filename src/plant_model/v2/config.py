# plant_model/v2/config.py

from dataclasses import dataclass, field


# ── World settings ────────────────────────────────────────────────────
PLANT_ROOT_PATH = "/World/Plant"
GLOBAL_SCALE = 10.0   # scale up the plant by 10x, mitigates small-number instability


@dataclass
class LeafPhysicsConfig:
    # ── Petiole / rachis chain ────────────────────────────────────────
    physics_enabled: bool = True
    num_petiole_segments: int = 3     # 1 = rigid petiole, >1 = articulated chain
    stiffness_base: float = 50.0
    stiffness_tip: float = 10.0
    damping_ratio: float = 0.7
    max_bend_angle: float = 30.0      # per-joint bend limit (deg)
    twist_limit: float = 15.0         # per-joint twist limit (deg)
    density: float = 200.0            # petiole cylinder density (kg/m³)

    # ── Compound leaf blade geometry ──────────────────────────────────
    blade_enabled: bool = True                    # attach blade meshes to rachis
    # Inclination override for lateral leaflets.
    # If None → use the per-leaflet CSV angle (leaf_inclination_segments).
    # If float (e.g. 50.0) → override all leaflets with this fixed angle (deg).
    blade_inclination_override: float | None = 50.0
    blade_collision: bool = True     # enable mesh collision (off = more stable)
    petiolule_length_m: float = 0.01  # visual-only petiolule cylinder length (m)


@dataclass
class BranchPhysicsConfig:
    physics_enabled: bool = True
    max_segments: int = 3          # number of physical segments per order>0 branch chain
    stiffness: float = 184_000.0
    damping: float = 5_000.0
    max_bend_angle: float = 30.0
    twist_limit: float = 15.0


@dataclass
class FruitPhysicsConfig:
    physics_enabled: bool = True
    stiffness: float = 0.001
    damping: float = 0.0001
    collisions: bool = False


@dataclass
class StemPhysicsConfig:
    physics_enabled: bool = False   # anchor-only; enables joints on attached children
    mass_per_segment: float = 1.0


@dataclass
class SimulationConfig:
    stem: StemPhysicsConfig = StemPhysicsConfig()
    leaf: LeafPhysicsConfig = LeafPhysicsConfig()
    branch: BranchPhysicsConfig = BranchPhysicsConfig()
    fruit: FruitPhysicsConfig = FruitPhysicsConfig()


DEFAULT_CONFIG = SimulationConfig()

