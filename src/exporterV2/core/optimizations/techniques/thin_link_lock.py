"""
thin_link_lock.py - Thin Link Lock Optimization Technique

Converts joints of branches with radius < MIN_LINK_RADIUS_WORLD to Fixed.
This is a safety mechanism to prevent PhysX solver instabilities on very thin segments.
"""

from typing import List, Dict, Tuple
from dataclasses import dataclass

try:
    from .base import OptimizationTechnique, OptimizationReport, ValidationResult, count_d6_joints
except ImportError:
    import sys
    import os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from base import OptimizationTechnique, OptimizationReport, ValidationResult, count_d6_joints

# Import the threshold and scale from tree_config
try:
    from ...tree_config import MIN_LINK_RADIUS_WORLD, GLOBAL_SCALE
except ImportError:
    # Fallback if imported directly
    import sys
    import os
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
    from tree_config import MIN_LINK_RADIUS_WORLD, GLOBAL_SCALE


@dataclass
class ThinLinkLockReport:
    """Detailed report for thin link lock technique."""
    thin_links_found: int
    thin_links_locked: int
    dof_reduced: int


class ThinLinkLockTechnique(OptimizationTechnique):
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
        
    def _is_thin_link(self, branch: dict) -> bool:
        """
        Check if a branch is below the minimum safe radius threshold.
        """
        radius_world = branch.get("radius", 0.0) * GLOBAL_SCALE
        # Consideriamo anche i rami che sono stati "clampati" esattamente al valore minimo
        return radius_world <= (MIN_LINK_RADIUS_WORLD + 1e-6)
        
    def _has_fixed_joint(self, branch: dict) -> bool:
        """
        Check if branch already has a fixed joint.
        """
        return branch.get("joint_type", "d6").lower() == "fixed"
    
    def can_apply(self, branches: List[dict], current_joints: int = 0, budget: int = 0) -> bool:
        """
        Check if technique can be applied.
        """
        for branch in branches:
            if self._is_thin_link(branch) and not self._has_fixed_joint(branch):
                return True
        return False
    
    def estimate_reduction(self, branches: List[dict]) -> int:
        """
        Estimate joint reduction.
        Since this converts D6 to Fixed, the total number of physical joints 
        doesn't change structurally, but D6 budget decreases.
        We return 0 for joint reduction if we consider structural joints,
        but for the budget, each conversion saves n_links D6 joints.
        Wait, in `petiole_lock`, it says `joints_saved=0` and only returns DOF reduction.
        Let's return 0 to be consistent with petiole_lock, as this is mainly a safety technique.
        Actually, let's just return 0 for estimate_reduction as it doesn't change `n_links`.
        Wait, `optimizer.py` relies on `estimate_reduction` to calculate `minimum_achievable`.
        Actually, `count_d6_joints` skips fixed joints, so changing a joint to fixed DOES reduce 
        the D6 count by `branch["n_links"]`! Let's return the sum of n_links.
        """
        count = 0
        for branch in branches:
            if self._is_thin_link(branch) and not self._has_fixed_joint(branch):
                count += branch.get("n_links", 1)
        return count
    
    def apply(self, branches: List[dict]) -> Tuple[List[dict], OptimizationReport]:
        """
        Apply thin link lock technique.
        """
        modified = []
        thin_links_locked = 0
        thin_links_found = 0
        
        joints_before_total = count_d6_joints(branches)
        
        for branch in branches:
            branch_copy = branch.copy()
            
            if self._is_thin_link(branch_copy):
                thin_links_found += 1
                
                if not self._has_fixed_joint(branch_copy):
                    branch_copy["joint_type"] = "fixed"
                    thin_links_locked += 1
            
            modified.append(branch_copy)
            
        joints_after_total = count_d6_joints(modified)
        joints_saved = joints_before_total - joints_after_total
        
        detailed_report = ThinLinkLockReport(
            thin_links_found=thin_links_found,
            thin_links_locked=thin_links_locked,
            dof_reduced=joints_saved * 6
        )
        
        report = OptimizationReport(
            technique_name=self.name,
            joints_before=joints_before_total,
            joints_after=joints_after_total,
            joints_saved=joints_saved,
            details={
                "thin_links_found": detailed_report.thin_links_found,
                "thin_links_locked": detailed_report.thin_links_locked,
                "dof_reduced": detailed_report.dof_reduced,
            }
        )
        
        return modified, report
        
    def validate(self, original: List[dict], modified: List[dict]) -> ValidationResult:
        """
        Validate that optimization preserved topology and geometry.
        """
        errors = []
        warnings = []
        
        if len(original) != len(modified):
            errors.append(f"Branch count mismatch: {len(original)} → {len(modified)}")
            return ValidationResult(False, errors, warnings)
            
        orig_dict = {b["id"]: b for b in original}
        mod_dict = {b["id"]: b for b in modified}
        
        for branch_id in orig_dict:
            if branch_id not in mod_dict:
                errors.append(f"Branch {branch_id} missing after optimization")
                
        if errors:
            return ValidationResult(False, errors, warnings)
            
        for branch_id, orig_branch in orig_dict.items():
            mod_branch = mod_dict[branch_id]
            
            if orig_branch.get("parent") != mod_branch.get("parent"):
                errors.append(f"Branch {branch_id}: parent changed")
                
            for key in ["n_links", "height", "radius"]:
                if key in orig_branch and orig_branch[key] != mod_branch.get(key):
                    errors.append(f"Branch {branch_id}: {key} changed")
                    
            if "attach_link" in orig_branch and orig_branch["attach_link"] != mod_branch.get("attach_link"):
                errors.append(f"Branch {branch_id}: attachment changed")
                
        success = len(errors) == 0
        return ValidationResult(success, errors, warnings)
