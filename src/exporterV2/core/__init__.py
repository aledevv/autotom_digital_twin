"""
core - Generic Tree Builder

Generic tree generation from BRANCHES configuration.
Works with any plant/tree structure.
"""

from .tree_config import (
    BRANCHES,
    GLOBAL_SCALE,
    MAX_N_LINK,
    MIN_LINK_RADIUS_WORLD,
    PHYLLOTAXIS,
    print_tree_summary,
    validate_branches,
    clamp_radius,
)

from .physics import apply_physx_scene_settings

from . import usd

__all__ = [
    "BRANCHES",
    "GLOBAL_SCALE",
    "MAX_N_LINK",
    "MIN_LINK_RADIUS_WORLD",
    "PHYLLOTAXIS",
    "print_tree_summary",
    "validate_branches",
    "clamp_radius",
    "apply_physx_scene_settings",
    "usd",
]
