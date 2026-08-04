"""
exporterV2 - Recursive Tree Model Exporter

Production-ready tree model generator using recursive branch structures.
Supports articulated physics with flexible joints based on Euler-Bernoulli beam theory.

Main components:
- tree_config: Configuration and physics parameters
- generate_tree: USD generation with articulated physics
- load_tree: Isaac Sim integration and simulation

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

from .generate_tree import build_stage, build_stage_locked, get_output_usd_path
from . import tree_config

__all__ = [
    'build_stage',
    'build_stage_locked',
    'get_output_usd_path',
    'tree_config',
]

__version__ = '2.0.0'
