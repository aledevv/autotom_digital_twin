"""
Joint-Budget Optimization System

Incremental optimization system to reduce joint count in USD plant models
for Isaac Sim/PhysX hardware constraints.

Main Entry Point:
    BudgetOptimizer - Main orchestrator class

Example:
    >>> from exporterV2.core.optimizations import BudgetOptimizer
    >>> optimizer = BudgetOptimizer()
    >>> optimized_branches, report = optimizer.optimize(branches)
    >>> print(report)

Documentation:
    See docs/OPTIMIZATION_README.md for complete documentation index.
"""

# Main API exports (will be implemented in subsequent tasks)
# from .optimizer import BudgetOptimizer, OptimizationReport

__version__ = "0.1.0"
__all__ = [
    # "BudgetOptimizer",
    # "OptimizationReport",
]
