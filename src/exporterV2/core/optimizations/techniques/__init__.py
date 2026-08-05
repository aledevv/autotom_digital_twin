"""
Optimization Techniques

Plugin-based optimization techniques for joint budget reduction.
Each technique implements the OptimizationTechnique abstract base class.

Available Techniques (by priority):
    1. PetioleLockTechnique - Convert D6 joints to Fixed ✅
    2. LateralBranchReductionTechnique - Reduce lateral branch segments
    3. StemCollapseTechnique - Collapse main stem with remapping
    4. TrussStaticTechnique - Pre-bent static truss geometry
    5. LeafBranchReductionTechnique - Merge petiole+rachis
"""

from .base import OptimizationTechnique, OptimizationReport, ValidationResult
from .petiole_lock import PetioleLockTechnique

# Exports (will be implemented in subsequent tasks)
# from .lateral_reduce import LateralBranchReductionTechnique
# from .stem_collapse import StemCollapseTechnique
# from .truss_static import TrussStaticTechnique
# from .leaf_branch_reduce import LeafBranchReductionTechnique

__all__ = [
    "OptimizationTechnique",
    "OptimizationReport",
    "ValidationResult",
    "PetioleLockTechnique",
    # "LateralBranchReductionTechnique",
    # "StemCollapseTechnique",
    # "TrussStaticTechnique",
    # "LeafBranchReductionTechnique",
]
