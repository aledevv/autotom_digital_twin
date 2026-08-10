"""
thin_link_lock.py - Thin Link Lock Optimization Technique

Converts joints of branches with radius < MIN_LINK_RADIUS_WORLD to Fixed.
This is a safety mechanism to prevent PhysX solver instabilities on very thin segments.
"""

from typing import Dict

try:
    from .lock_base import BaseLockTechnique
except ImportError:
    import sys
    import os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from lock_base import BaseLockTechnique

# Import the threshold and scale from tree_config
try:
    from ...tree_config import MIN_LINK_RADIUS_WORLD, GLOBAL_SCALE
except ImportError:
    # Fallback if imported directly
    import sys
    import os
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
    from tree_config import MIN_LINK_RADIUS_WORLD, GLOBAL_SCALE


class ThinLinkLockTechnique(BaseLockTechnique):
    """
    Thin Link Lock: Convert joints of very thin branches to Fixed.
    
    Priority: 1 (highest - safety feature)
    Impact: Reduces DOF for thin branches to prevent physics instability
    """
    
    def __init__(self, params: Dict = None):
        """Initialize thin link lock technique."""
        super().__init__(params)
        self._name = "thin_link_lock"
        self._priority = 1
    
    @property
    def name(self) -> str:
        """Technique identifier."""
        return self._name
    
    @property
    def priority(self) -> int:
        """Execution priority."""
        return self._priority
        
    def _is_target(self, branch: dict) -> bool:
        """
        Check if a branch is below the minimum safe radius threshold.
        """
        radius_world = branch.get("radius", 0.0) * GLOBAL_SCALE
        # Consideriamo anche i rami che sono stati "clampati" esattamente al valore minimo
        return radius_world <= (MIN_LINK_RADIUS_WORLD + 1e-6)
