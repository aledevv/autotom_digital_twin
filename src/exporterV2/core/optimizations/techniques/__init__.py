"""
Optimization Techniques

Plugin-based optimization techniques for joint budget reduction.
Each technique implements the OptimizationTechnique abstract base class.

Available Techniques (by priority):
    1. PetioleLockTechnique - Convert D6 joints to Fixed ✅
    1.5 ThinLinkLockTechnique - Convert thin segment joints to Fixed ✅
    2. LateralBranchReductionTechnique - Reduce lateral branch segments
    3. StemCollapseTechnique - Collapse main stem with remapping
    4. TrussStaticTechnique - Pre-bent static truss geometry
    5. LeafBranchReductionTechnique - Merge petiole+rachis
"""

from .base import OptimizationTechnique, OptimizationReport, ValidationResult, count_d6_joints
from .petiole_lock import PetioleLockTechnique
from .thin_link_lock import ThinLinkLockTechnique

# Exports - only implemented techniques
from .lateral_reduce import LateralBranchReductionTechnique
from .stem_collapse import StemCollapseTechnique
# from .truss_static import TrussStaticTechnique  # TODO: Task 7 - skipped (truss not in codebase)
from .leaf_branch_reduce import LeafBranchReductionTechnique

__all__ = [
    "OptimizationTechnique",
    "OptimizationReport",
    "ValidationResult",
    "count_d6_joints",
    "PetioleLockTechnique",
    "ThinLinkLockTechnique",
    "LateralBranchReductionTechnique",
    "StemCollapseTechnique",
    # "TrussStaticTechnique",
    "LeafBranchReductionTechnique",
]
