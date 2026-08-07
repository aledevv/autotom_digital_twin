"""
leaf_branch_reduce.py - Leaf Branch Reduction Optimization Technique

Merges petiole and rachis into a single segment to save joint budget.
Petiolules are remapped along the merged segment using exact geometry matching (attach_frac).

Usage:
    technique = LeafBranchReductionTechnique()
    if technique.can_apply(branches):
        modified, report = technique.apply(branches)
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


class LeafBranchReductionTechnique(OptimizationTechnique):
    """
    Leaf Branch Reduction: Merge petiole and rachis into one single segment.
    
    Priority: 5 (highest visual impact - leaves become stiff)
    Impact: Saves N joints (where N is the number of rachis links) per leaf.
    """
    
    def __init__(self):
        """Initialize leaf branch reduction technique."""
        self._name = "leaf_branch_reduce"
        self._priority = 5
    
    @property
    def name(self) -> str:
        return self._name
    
    @property
    def priority(self) -> int:
        return self._priority
    
    def _is_petiole(self, branch: dict) -> bool:
        return branch.get("id", "").endswith("_petiole") or "Petiole_" in branch.get("id", "")
        
    def _is_rachis(self, branch: dict) -> bool:
        return branch.get("id", "").endswith("_rachis") or "Rachis_" in branch.get("id", "")

    def can_apply(self, branches: List[dict]) -> bool:
        """Check if any petiole+rachis pair exists."""
        petiole_ids = {b["id"] for b in branches if self._is_petiole(b)}
        for b in branches:
            if self._is_rachis(b) and b.get("parent") in petiole_ids:
                return True
        return False
    
    def estimate_reduction(self, branches: List[dict]) -> int:
        """Estimate how many joints will be saved."""
        petioles = {b["id"]: b for b in branches if self._is_petiole(b)}
        saved = 0
        for b in branches:
            if self._is_rachis(b) and b.get("parent") in petioles:
                # Merge petiole (1 link) and rachis (M links) -> 1 link total
                saved += b.get("n_links", 1)
        return saved
    
    def apply(self, branches: List[dict]) -> Tuple[List[dict], OptimizationReport]:
        """Apply petiole+rachis merge - merges ONE pair per call."""
        petioles = {b["id"]: b for b in branches if self._is_petiole(b)}
        rachis_list = [b for b in branches if self._is_rachis(b) and b.get("parent") in petioles]
        
        if not rachis_list:
            report = OptimizationReport(
                technique_name=self.name,
                joints_before=count_d6_joints(branches),
                joints_after=count_d6_joints(branches),
                joints_saved=0,
                details={"pairs_merged": 0, "links_removed": 0, "petiolules_remapped": 0}
            )
            return branches, report
        
        # CHANGE: Process only the FIRST mergeable pair (not all at once)
        rachis = rachis_list[0]
        
        modified = []
        links_removed = 0
        petiolules_remapped = 0
        
        # Build lookup for all branches to ease modification
        branch_dict = {b["id"]: b.copy() for b in branches}
        
        petiole_id = rachis["parent"]
        petiole = branch_dict[petiole_id]
        rachis_id = rachis["id"]
        
        # Merge into petiole
        base_name = petiole_id.replace("_petiole", "").replace("Petiole_", "Leaf_")
        merged_id = f"{base_name}_merged" if not petiole_id.endswith("_merged") else petiole_id
        
        petiole_len = petiole.get("height", 0.0) * petiole.get("n_links", 1)
        rachis_len = rachis.get("height", 0.0) * rachis.get("n_links", 1)
        total_len = petiole_len + rachis_len
        
        # Update petiole to become the merged segment
        branch_dict[petiole_id]["id"] = merged_id
        branch_dict[petiole_id]["height"] = total_len
        branch_dict[petiole_id]["n_links"] = 1
        branch_dict[petiole_id]["radius"] = (petiole.get("radius", 0.01) + rachis.get("radius", 0.01)) / 2.0
        
        # Count savings (we removed the rachis n_links)
        links_removed += rachis.get("n_links", 1)
        
        # Remap petiolules that were attached to the rachis
        for b_id, b in branch_dict.items():
            if b.get("parent") == rachis_id:
                # It was attached to the rachis
                old_attach_link = b.get("attach_link", 1)
                old_rachis_n = rachis.get("n_links", 1)
                
                # Fraction along rachis
                rachis_fraction = old_attach_link / old_rachis_n
                
                # Distance from base of merged leaf
                absolute_dist = petiole_len + (rachis_fraction * rachis_len)
                
                # New attach_frac
                new_frac = absolute_dist / total_len if total_len > 0 else 1.0
                
                b["parent"] = merged_id
                b["attach_link"] = 1
                b["attach_frac"] = new_frac
                petiolules_remapped += 1
        
        # Remove the rachis from the dictionary
        del branch_dict[rachis_id]
        
        # If any other branches were attached to the petiole directly, update their parent
        for b_id, b in branch_dict.items():
            if b.get("parent") == petiole_id and b_id != merged_id:
                b["parent"] = merged_id

        modified = list(branch_dict.values())
        
        report = OptimizationReport(
            technique_name=self.name,
            joints_before=count_d6_joints(branches),
            joints_after=count_d6_joints(modified),
            joints_saved=links_removed,
            details={
                "pairs_merged": 1,  # Only 1 pair per call now
                "links_removed": links_removed,
                "petiolules_remapped": petiolules_remapped
            }
        )
        
        return modified, report

    def validate(self, original: List[dict], modified: List[dict]) -> ValidationResult:
        """Validate merged geometry."""
        # Simple validation: ensure no broken parent references
        mod_ids = {b["id"] for b in modified}
        errors = []
        for b in modified:
            parent = b.get("parent")
            if parent is not None and parent not in mod_ids:
                errors.append(f"Branch {b['id']} has missing parent {parent}")
                
        return ValidationResult(len(errors) == 0, errors, [])
