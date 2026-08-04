"""
exporterV2 - Recursive Tree Model Exporter

Production-ready tree model generator using recursive branch structures.
Supports articulated physics with flexible joints based on Euler-Bernoulli beam theory.

Main components:
- tree_config: Configuration and physics parameters
- csv_data: CSV parsing and leaf construction
- usd: USD generation with articulated physics
- physics: PhysX configuration for Isaac Sim
- main: Unified entry point

Usage:
    from exporterV2 import build_stage, tree_config
    from exporterV2.tree_config import BRANCHES, BioConfig
    
    # Generate USD
    stage, stem_path = build_stage("output.usda")
    
    # Or customize configuration
    custom_branches = [...]
    stage, stem_path = build_stage("output.usda", branches=custom_branches)

For more details, see README.md
"""

# Lazy imports - only import when accessed
def __getattr__(name):
    if name == 'build_stage':
        from .usd import build_stage
        return build_stage
    elif name == 'build_stage_locked':
        from .usd import build_stage_locked
        return build_stage_locked
    elif name == 'get_output_usd_path':
        from .usd import get_output_usd_path
        return get_output_usd_path
    elif name == 'apply_physx_scene_settings':
        from .physics import apply_physx_scene_settings
        return apply_physx_scene_settings
    elif name == 'apply_physx_articulation_settings':
        from .physics import apply_physx_articulation_settings
        return apply_physx_articulation_settings
    elif name == 'csv_data':
        from . import csv_data
        return csv_data
    elif name == 'usd':
        from . import usd
        return usd
    raise AttributeError(f"module '{__name__}' has no attribute '{name}'")


# Only import tree_config (no external deps)
from . import tree_config

__all__ = [
    'build_stage',
    'build_stage_locked',
    'get_output_usd_path',
    'tree_config',
    'csv_data',
    'usd',
    'apply_physx_scene_settings',
    'apply_physx_articulation_settings',
]

__version__ = '2.1.0'
