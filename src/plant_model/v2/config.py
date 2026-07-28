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
class LeafPhysicsConfig:
    physics_enabled: bool = True
    max_segments: int = 5
    stiffness_base: float = 50.0
    stiffness_tip: float = 10.0
    damping_base: float = 0.0
    damping_tip: float = 0.0
    damping_ratio: float = 0.7
    max_bend_angle: float = 10.0
    twist_limit: float = 15.0
    density: float = 200.0


@dataclass
class BranchPhysicsConfig:
    physics_enabled: bool = True
    max_segments: int = 3          # <-- se i rami order>0 avranno anche loro una catena multi-segmento
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
    physics_enabled: bool = False   # anchor-only, per abilitare i joint dei figli
    mass_per_segment: float = 1.0


@dataclass
class SimulationConfig:
    stem: StemPhysicsConfig = StemPhysicsConfig()
    leaf: LeafPhysicsConfig = LeafPhysicsConfig()
    branch: BranchPhysicsConfig = BranchPhysicsConfig()
    fruit: FruitPhysicsConfig = FruitPhysicsConfig()


DEFAULT_CONFIG = SimulationConfig()