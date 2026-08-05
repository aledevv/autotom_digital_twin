"""
petiole_lock.py - Petiole Lock Optimization Technique

Converts petiolule joints from D6 (articulated, 6 DOF) to Fixed (static, 0 DOF).
This reduces computational cost without changing geometry or visual appearance.

Rationale:
- Petiolules (leaflet stems) are small and have limited movement in practice
- Converting to fixed joints maintains visual fidelity while reducing DOF
- Minimal impact on simulation realism for plant modeling

Usage:
    technique = PetioleLockTechnique()
    if technique.can_apply(branches):
        reduction = technique.estimate_reduction(branches)
        modified, report = technique.apply(branches)
"""

from typing import List, Dict, Tuple
from dataclasses import dataclass

try:
    from .base import OptimizationTechnique, OptimizationReport, ValidationResult
except ImportError:
    import sys
    import os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from base import OptimizationTechnique, OptimizationReport, ValidationResult


@dataclass
class PetioleLockReport:
    """Detailed report for petiole lock technique."""
    petiolules_found: int
    petiolules_locked: int
    dof_reduced: int  # Each D6 has 6 DOF, Fixed has 0


class PetioleLockTechnique(OptimizationTechnique):
    """
    Petiole Lock: Convert petiolule joints from D6 to Fixed.
    
    Priority: 1 (highest - minimal visual impact)
    Impact: Reduces DOF without geometry changes
    """
    
    def __init__(self):
        """Initialize petiole lock technique."""
        self._name = "petiole_lock"
        self._priority = 1
    
    @property
    def name(self) -> str:
        """Technique identifier."""
        return self._name
    
    @property
    def priority(self) -> int:
        """Execution priority (lower = earlier)."""
        return self._priority
    
    def _is_petiolule(self, branch: dict) -> bool:
        """
        Check if a branch is a petiolule.
        
        Petiolules are identified by naming pattern: "Petiolule_*"
        or by having parent that is a rachis.
        
        Args:
            branch: Branch configuration dict
        
        Returns:
            True if branch is a petiolule
        """
        branch_id = branch.get("id", "")
        
        # Check naming pattern
        if branch_id.startswith("Petiolule_"):
            return True
        
        # Check if it's a small terminal branch attached to rachis
        # (Alternative identification if naming is inconsistent)
        parent_id = branch.get("parent", "")
        if parent_id and parent_id.startswith("Rachis_"):
            # Additional check: petiolules are typically single-link
            n_links = branch.get("n_links", 1)
            if n_links <= 2:  # Allow up to 2 links for flexibility
                return True
        
        return False
    
    def _has_fixed_joint(self, branch: dict) -> bool:
        """
        Check if branch already has a fixed joint.
        
        Args:
            branch: Branch configuration dict
        
        Returns:
            True if already fixed
        """
        return branch.get("joint_type", "d6").lower() == "fixed"
    
    def can_apply(self, branches: List[dict]) -> bool:
        """
        Check if technique can be applied.
        
        Args:
            branches: List of branch configurations
        
        Returns:
            True if there are petiolules with D6 joints
        """
        for branch in branches:
            if self._is_petiolule(branch) and not self._has_fixed_joint(branch):
                return True
        return False
    
    def estimate_reduction(self, branches: List[dict]) -> int:
        """
        Estimate DOF reduction.
        
        Note: This counts DOF, not joints. Each D6→Fixed conversion
        reduces 6 DOF but doesn't change joint count.
        
        Args:
            branches: List of branch configurations
        
        Returns:
            Estimated DOF reduction (6 per petiolule)
        """
        count = 0
        for branch in branches:
            if self._is_petiolule(branch) and not self._has_fixed_joint(branch):
                count += 1
        
        # Each D6 has 6 DOF, Fixed has 0
        return count * 6
    
    def apply(self, branches: List[dict]) -> Tuple[List[dict], OptimizationReport]:
        """
        Apply petiole lock technique.
        
        Converts petiolule joints from D6 to Fixed by adding metadata.
        
        Args:
            branches: List of branch configurations
        
        Returns:
            (modified_branches, report) tuple
        """
        modified = []
        petiolules_locked = 0
        petiolules_found = 0
        
        for branch in branches:
            branch_copy = branch.copy()
            
            if self._is_petiolule(branch_copy):
                petiolules_found += 1
                
                # Check if not already fixed
                if not self._has_fixed_joint(branch_copy):
                    # Add joint_type metadata
                    branch_copy["joint_type"] = "fixed"
                    petiolules_locked += 1
            
            modified.append(branch_copy)
        
        # Create detailed report
        detailed_report = PetioleLockReport(
            petiolules_found=petiolules_found,
            petiolules_locked=petiolules_locked,
            dof_reduced=petiolules_locked * 6
        )
        
        # Create standard report
        report = OptimizationReport(
            technique_name=self.name,
            joints_before=len(branches),
            joints_after=len(modified),
            joints_saved=0,  # No joints removed, only DOF reduced
            details={
                "petiolules_found": detailed_report.petiolules_found,
                "petiolules_locked": detailed_report.petiolules_locked,
                "dof_reduced": detailed_report.dof_reduced,
            }
        )
        
        return modified, report
    
    def validate(self, original: List[dict], modified: List[dict]) -> ValidationResult:
        """
        Validate that optimization preserved topology.
        
        Checks:
        - Same number of branches
        - Same branch IDs
        - Same parent-child relationships
        - Same geometry (n_links, heights, radii)
        
        Args:
            original: Original branch configuration
            modified: Modified branch configuration
        
        Returns:
            ValidationResult with success flag and messages
        """
        errors = []
        warnings = []
        
        # Check branch count
        if len(original) != len(modified):
            errors.append(
                f"Branch count mismatch: {len(original)} → {len(modified)}"
            )
            return ValidationResult(False, errors, warnings)
        
        # Create lookup dicts
        orig_dict = {b["id"]: b for b in original}
        mod_dict = {b["id"]: b for b in modified}
        
        # Check all branches exist
        for branch_id in orig_dict:
            if branch_id not in mod_dict:
                errors.append(f"Branch {branch_id} missing after optimization")
        
        if errors:
            return ValidationResult(False, errors, warnings)
        
        # Check geometry preservation
        for branch_id, orig_branch in orig_dict.items():
            mod_branch = mod_dict[branch_id]
            
            # Check parent relationship
            if orig_branch.get("parent") != mod_branch.get("parent"):
                errors.append(
                    f"Branch {branch_id}: parent changed "
                    f"{orig_branch.get('parent')} → {mod_branch.get('parent')}"
                )
            
            # Check geometry
            for key in ["n_links", "height", "radius"]:
                if key in orig_branch:
                    orig_val = orig_branch[key]
                    mod_val = mod_branch.get(key)
                    if orig_val != mod_val:
                        errors.append(
                            f"Branch {branch_id}: {key} changed {orig_val} → {mod_val}"
                        )
            
            # Check attachment (if applicable)
            if "attach_link" in orig_branch:
                if orig_branch["attach_link"] != mod_branch.get("attach_link"):
                    errors.append(
                        f"Branch {branch_id}: attachment changed"
                    )
        
        # Check that only petiolules got joint_type metadata
        for branch_id, mod_branch in mod_dict.items():
            if "joint_type" in mod_branch:
                if not self._is_petiolule(mod_branch):
                    warnings.append(
                        f"Non-petiolule {branch_id} has joint_type metadata"
                    )
        
        success = len(errors) == 0
        if success and not warnings:
            return ValidationResult(
                valid=True,
                errors=[],
                warnings=[]
            )
        
        return ValidationResult(success, errors, warnings)
