"""
profiles - Cultivar Configuration Profiles

Each profile contains cultivar-specific filtering and orientation parameters.
"""

from .tomato_default import TOMATO_PROFILE
from .simple_plant import SIMPLE_PLANT_PROFILE

__all__ = ["TOMATO_PROFILE", "SIMPLE_PLANT_PROFILE"]
