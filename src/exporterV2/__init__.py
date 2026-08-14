"""
exporterV2 - Recursive Tree Model Exporter

Production-ready tree model generator with modular architecture:
- core: Generic tree builder (works with any BRANCHES config)
- adapters: Data source adapters (groIMP CSV, manual config, etc.)
- profiles: Cultivar-specific configurations

Quick Start:
    from exporterV2.core.usd import build_stage
    stage, stem_path = build_stage("tree.usda")
"""

# Re-export core functionality for convenience (tree_config only, no pxr)
from .core import (
    BRANCHES,
    GLOBAL_SCALE,
    MAX_N_JOINTS,
    MIN_LINK_RADIUS_WORLD,
    TrussGeometryConfig,
    print_tree_summary,
    validate_branches,
)

# Lazy imports for USD/physics (require pxr)
def __getattr__(name):
    if name == "build_stage":
        from .core.usd import build_stage
        return build_stage
    elif name == "apply_physx_scene_settings":
        from .core.physics import apply_physx_scene_settings
        return apply_physx_scene_settings
    raise AttributeError(f"module '{__name__}' has no attribute '{name}'")

__all__ = [
    "build_stage",
    "BRANCHES",
    "GLOBAL_SCALE",
    "MAX_N_JOINTS",
    "MIN_LINK_RADIUS_WORLD",
    "TrussGeometryConfig",
    "print_tree_summary",
    "validate_branches",
    "apply_physx_scene_settings",
]

__version__ = "2.2.0"
