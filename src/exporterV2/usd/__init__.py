"""
usd - USD Generation Module

Handles USD stage creation, geometry, joints, and collision filtering.

Main components:
- stage: Stage setup and top-level orchestration
- geometry: Link and cylinder creation
- joints: Joint creation (flexible and locked)
- collision: Collision filtering logic
"""

from .stage import build_stage, build_stage_locked, get_output_usd_path

__all__ = [
    'build_stage',
    'build_stage_locked',
    'get_output_usd_path',
]
