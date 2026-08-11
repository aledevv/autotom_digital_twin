"""
optimizer.py - Main Orchestrator for Joint-Budget Optimization

Coordinates the application of optimization techniques to reduce joint count
within a specified budget while maintaining structural integrity.

Example:
    >>> from exporterV2.core.optimizations import BudgetOptimizer
    >>> optimizer = BudgetOptimizer()
    >>> optimized_branches, report = optimizer.optimize(branches)
    >>> print(report)
"""

import os
import yaml
from pathlib import Path
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass, field

# Support both package and standalone imports
try:
    from .techniques.base import OptimizationTechnique, OptimizationReport, ValidationResult
except ImportError:
    from techniques.base import OptimizationTechnique, OptimizationReport, ValidationResult


@dataclass
class BudgetConfig:
    """Parsed configuration from budget_config.yaml."""
    max_joints: int
    warning_threshold: int
    structural_limits: Dict[str, Dict]
    techniques: List[Dict]
    logging: Dict
    
    @classmethod
    def load(cls, config_path: str) -> 'BudgetConfig':
        """Load configuration from YAML file."""
        path = Path(config_path)
        if not path.exists():
            raise FileNotFoundError(f"Config file not found: {config_path}")
        
        with open(path, 'r') as f:
            config = yaml.safe_load(f)
        
        # Validate required sections
        required_sections = ['budget', 'structural_limits', 'techniques']
        for section in required_sections:
            if section not in config:
                raise ValueError(f"Missing required section '{section}' in config")
        
        # Extract and validate budget section
        budget = config['budget']
        if budget['max_joints'] <= 0:
            raise ValueError("max_joints must be positive")
        
        return cls(
            max_joints=budget['max_joints'],
            warning_threshold=budget.get('warning_threshold', budget['max_joints'] - 20),
            structural_limits=config['structural_limits'],
            techniques=sorted(config['techniques'], key=lambda t: t['priority']),
            logging=config.get('logging', {'level': 'INFO'})
        )


@dataclass
class FullOptimizationReport:
    """Complete optimization report with all techniques applied."""
    original_joints: int
    final_joints: int
    budget: int
    lower_bound: int
    technique_reports: List[OptimizationReport] = field(default_factory=list)
    success: bool = False
    error_message: Optional[str] = None
    minimum_achievable: Optional[int] = None  # Minimum if all techniques fully applied
    
    @property
    def total_reduction(self) -> int:
        """Total joints saved across all techniques."""
        return self.original_joints - self.final_joints
    
    @property
    def reduction_percentage(self) -> float:
        """Percentage reduction in joints."""
        if self.original_joints == 0:
            return 0.0
        return (self.total_reduction / self.original_joints) * 100.0
    
    @property
    def max_reduction_percentage(self) -> float:
        """Maximum achievable reduction percentage (if fully optimized)."""
        if self.original_joints == 0 or self.minimum_achievable is None:
            return 0.0
        max_reduction = self.original_joints - self.minimum_achievable
        return (max_reduction / self.original_joints) * 100.0
    
    def __str__(self) -> str:
        """Human-readable full report."""
        lines = [
            "=" * 60,
            "  Joint-Budget Optimization Report",
            "=" * 60,
            f"Original joints: {self.original_joints}",
            f"Budget: {self.budget}",
            f"Lower bound: {self.lower_bound}",
        ]
        
        # Add minimum achievable info if available
        if self.minimum_achievable is not None and self.minimum_achievable != self.final_joints:
            lines.append(f"Minimum achievable: {self.minimum_achievable} "
                        f"(max {self.max_reduction_percentage:.1f}% reduction)")
        
        lines.append("")
        
        if self.technique_reports:
            lines.append("Techniques applied:")
            for i, report in enumerate(self.technique_reports, 1):
                lines.append(f"  {i}. {report.technique_name}: "
                           f"{report.joints_before} → {report.joints_after} "
                           f"(-{report.joints_saved} joints)")
                if report.details:
                    for key, value in report.details.items():
                        lines.append(f"      - {key}: {value}")
            lines.append("")
        
        if self.success:
            status = "✓"
            lines.append(f"Final joints: {self.final_joints} {status}")
            lines.append(f"Total reduction: -{self.total_reduction} joints "
                        f"({self.reduction_percentage:.1f}%)")
        else:
            status = "✗"
            lines.append(f"Status: FAILED {status}")
            if self.error_message:
                lines.append(f"Error: {self.error_message}")
        
        lines.append("=" * 60)
        return "\n".join(lines)


class BudgetOptimizer:
    """
    Main orchestrator for joint-budget optimization.
    
    Loads configuration, calculates lower bound, and applies techniques
    sequentially by priority until budget is met or techniques exhausted.
    
    Example:
        >>> optimizer = BudgetOptimizer()
        >>> optimized, report = optimizer.optimize(branches)
        >>> if report.success:
        ...     print(f"Reduced from {report.original_joints} to {report.final_joints} joints")
    """
    
    def __init__(self, config_path: Optional[str] = None):
        """
        Initialize optimizer with configuration.
        
        Args:
            config_path: Path to budget_config.yaml (default: auto-detect)
        """
        if config_path is None:
            # Auto-detect config path (same directory as this file)
            config_path = Path(__file__).parent / "budget_config.yaml"
        
        self.config = BudgetConfig.load(str(config_path))
    
    def calculate_total_joints(self, branches: List[Dict]) -> int:
        """
        Calculate total number of D6 joints in current configuration.
        
        Args:
            branches: Branches configuration list
        
        Returns:
            Total D6 joint count (excludes Fixed joints from petiolules)
        
        Note:
            Only D6 joints count toward the budget. Fixed joints (locked petiolules)
            are excluded because they don't contribute to simulation complexity.
            
            In PhysX articulations, each link represents a rigid body,
            and joints are between consecutive links.
        """
        total = 0
        for branch in branches:
            # Skip Fixed joints (locked petiolules)
            if branch.get("joint_type", "d6").lower() == "fixed":
                continue
            total += branch.get("n_links", 1)
        return total
    
    def calculate_lower_bound(self, branches: List[Dict]) -> int:
        """
        Calculate structural lower bound - minimum joints needed.
        
        Args:
            branches: Branches configuration list
        
        Returns:
            Minimum number of joints required for structural integrity
        
        Raises:
            ValueError: If lower bound calculation fails
        
        Note:
            Lower bound is based on structural_limits config:
            - trunk: min_links per trunk
            - lateral_branch: min_links per lateral
            - petiole: min_links per petiole
            - truss: min_links per truss
            - petiolules and rachis can be reduced to 0
        """
        lower_bound = 0
        limits = self.config.structural_limits
        
        # Component type identification helpers
        def is_trunk(b: Dict) -> bool:
            return (
                b.get("parent") is None
                and b.get("joint_type", "d6").lower() != "fixed"
            )
        
        def is_lateral_branch(b: Dict) -> bool:
            # Lateral branches have parent = trunk and specific naming
            return (b.get("parent") in ["trunk"] and 
                   "Branch_" in b.get("id", ""))
        
        def is_petiole(b: Dict) -> bool:
            return "petiole" in b.get("id", "").lower()
        
        def is_truss(b: Dict) -> bool:
            branch_id = b.get("id", "").lower()
            return (
                "truss" in branch_id
                and "_pedicel_" not in branch_id
                and "_static_curve_" not in branch_id
            )
        
        # Count by component type
        trunk_count = sum(1 for b in branches if is_trunk(b))
        lateral_count = sum(1 for b in branches if is_lateral_branch(b))
        petiole_count = sum(1 for b in branches if is_petiole(b))
        truss_count = sum(1 for b in branches if is_truss(b))
        
        # Calculate lower bound based on minimums
        lower_bound += trunk_count * limits["trunk"]["min_links"]
        lower_bound += lateral_count * limits["lateral_branch"]["min_links"]
        lower_bound += petiole_count * limits["petiole"]["min_links"]
        lower_bound += truss_count * limits["truss"]["min_links"]
        
        # Note: rachis and petiolules have min_links=0, so they don't contribute
        
        return lower_bound
    
    def _get_technique(self, technique_config: Dict) -> OptimizationTechnique:
        # Import with fallback for both package and standalone use
        try:
            from .techniques import (
                PetioleLockTechnique,
                ThinLinkLockTechnique,
                LateralBranchReductionTechnique,
                StemCollapseTechnique,
                TrussStaticTechnique,
                LeafBranchReductionTechnique,
            )
        except ImportError:
            from techniques import (
                PetioleLockTechnique,
                ThinLinkLockTechnique,
                LateralBranchReductionTechnique,
                StemCollapseTechnique,
                TrussStaticTechnique,
                LeafBranchReductionTechnique,
            )
        
        tech_id = technique_config["id"]
        
        if tech_id == "petiole_lock":
            return PetioleLockTechnique()
        elif tech_id == "thin_link_lock":
            return ThinLinkLockTechnique()
        elif tech_id == "lateral_reduce":
            min_segments = technique_config.get("params", {}).get("min_segments", 1)
            return LateralBranchReductionTechnique(min_segments=min_segments)
        elif tech_id == "stem_collapse":
            target_segments = technique_config.get("params", {}).get("target_segments", 3)
            return StemCollapseTechnique(target_segments=target_segments)
        elif tech_id == "truss_static":
            return TrussStaticTechnique(params=technique_config.get("params", {}))
        elif tech_id == "leaf_branch_reduce":
            return LeafBranchReductionTechnique()
        else:
            # Skip undefined techniques while preserving config compatibility.
            # Return dummy technique that does nothing
            class DummyTechnique(OptimizationTechnique):
                def __init__(self): self._name = tech_id; self._priority = 99
                @property
                def name(self): return self._name
                @property
                def priority(self): return self._priority
                def can_apply(self, branches): return False
                def estimate_reduction(self, branches): return 0
                def apply(self, branches): return branches, None
                def validate(self, orig, mod): 
                    try:
                        from .techniques.base import ValidationResult
                    except ImportError:
                        from techniques.base import ValidationResult
                    return ValidationResult(True, [], [])
            return DummyTechnique()

    def optimize(self, branches: List[Dict]) -> Tuple[List[Dict], FullOptimizationReport]:
        """
        Apply optimization techniques until budget met or techniques exhausted.

        Args:
            branches: Original branches configuration

        Returns:
            Tuple (optimized_branches, report):
                optimized_branches: Modified branches configuration
                report: FullOptimizationReport with details

        Raises:
            ValueError: If budget is impossible to meet (below lower bound)

        Algorithm:
            1. Calculate total joints and lower bound
            2. If already within budget → return unchanged
            3. If lower bound > budget → raise ValueError
            4. Apply enabled techniques by priority until budget met
            5. Validate after each technique application
            6. Return optimized config + full report
        """
        # Calculate initial state
        original_joints = self.calculate_total_joints(branches)
        lower_bound = self.calculate_lower_bound(branches)
        budget = self.config.max_joints
        
        # Check if already within budget
        if original_joints <= budget:
            report = FullOptimizationReport(
                original_joints=original_joints,
                final_joints=original_joints,
                budget=budget,
                lower_bound=lower_bound,
                success=True,
                technique_reports=[]
            )
            return (branches, report)
        
        # Check if budget is achievable
        if lower_bound > budget:
            error_msg = (
                f"Budget impossible to meet: lower bound ({lower_bound} joints) "
                f"exceeds budget ({budget} joints). "
                f"Reduce plant complexity or increase budget."
            )
            report = FullOptimizationReport(
                original_joints=original_joints,
                final_joints=original_joints,
                budget=budget,
                lower_bound=lower_bound,
                success=False,
                error_message=error_msg,
                technique_reports=[]
            )
            raise ValueError(error_msg)
        
        # Apply techniques sequentially
        current_branches = branches.copy()
        current_joints = original_joints
        technique_reports = []
        
        for technique_config in self.config.techniques:
            if not technique_config.get("enabled", True):
                continue
            
            technique = self._get_technique(technique_config)
            
            # Apply technique iteratively until one of:
            # 1. Budget is met (current_joints <= budget)
            # 2. Technique cannot reduce further (can_apply returns False)
            # 3. Safety limit reached (prevent infinite loops)
            iteration = 0
            max_iterations = 1000
            prev_joints = current_joints
            
            while technique.can_apply(current_branches) and iteration < max_iterations:
                modified, tech_report = technique.apply(current_branches)
                validation = technique.validate(current_branches, modified)
                
                if validation.valid:
                    current_branches = modified
                    if tech_report:  # Skip dummy reports
                        technique_reports.append(tech_report)
                    
                    prev_joints = current_joints
                    current_joints = self.calculate_total_joints(current_branches)
                    
                    # Stop if no progress (stuck)
                    if current_joints == prev_joints:
                        break
                    
                    # Stop if budget met
                    if current_joints <= budget:
                        break  # Exit inner loop (budget met)
                else:
                    # Validation failed, stop this technique
                    break
                
                iteration += 1
            
            # Stop outer loop if budget met
            if current_joints <= budget:
                break  # Exit outer loop (budget met)
        
        final_joints = self.calculate_total_joints(current_branches)
        success = (final_joints <= budget)
        
        # Calculate minimum achievable (if all techniques were fully applied)
        # This is done by checking if any technique can still be applied
        minimum_achievable = final_joints
        temp_branches = current_branches.copy()
        for technique_config in self.config.techniques:
            if not technique_config.get("enabled", True):
                continue
            technique = self._get_technique(technique_config)
            # Estimate how many more joints could be saved if this technique was fully applied
            if technique.can_apply(temp_branches):
                estimated_reduction = technique.estimate_reduction(temp_branches)
                minimum_achievable -= estimated_reduction
        
        # Ensure minimum doesn't go below lower bound
        if minimum_achievable < lower_bound:
            minimum_achievable = lower_bound
        
        # Check if we reached the minimum possible (no more techniques can apply)
        reached_minimum = True
        for technique_config in self.config.techniques:
            if not technique_config.get("enabled", True):
                continue
            technique = self._get_technique(technique_config)
            if technique.can_apply(current_branches):
                reached_minimum = False
                break
        
        # Determine error message if budget not met
        error_message = None
        if not success:
            if reached_minimum:
                error_message = (
                    f"Optimization reached minimum possible ({final_joints} joints) "
                    f"but could not meet budget ({budget} joints). "
                    f"Budget is {final_joints - budget} joints too aggressive. "
                    f"Consider increasing budget or reducing plant complexity."
                )
            else:
                potential_reduction = final_joints - minimum_achievable
                error_message = (
                    f"Optimization stopped at {final_joints} joints (budget: {budget} joints). "
                    f"Minimum achievable: ~{minimum_achievable} joints "
                    f"({potential_reduction} more joints could be saved with full optimization)."
                )
        
        report = FullOptimizationReport(
            original_joints=original_joints,
            final_joints=final_joints,
            budget=budget,
            lower_bound=lower_bound,
            technique_reports=technique_reports,
            success=success,
            error_message=error_message,
            minimum_achievable=minimum_achievable
        )
        
        return (current_branches, report)
