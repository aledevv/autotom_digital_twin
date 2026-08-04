"""
csv_data - CSV Parsing Module

Handles groIMP CSV data parsing and conversion to BRANCHES format.

Main components:
- parser: Generic CSV loading (trunk, leaves)
- leaf_builder: Leaf-specific branch construction (petiole, rachis, petiolules)
"""

from .parser import (
    load_trunk_internodes,
    load_leaves,
    parse_csv_to_branches,
)
from .leaf_builder import (
    calculate_leaf_orientation,
    leaf_to_petiole_rachis_branches,
    create_lateral_petiolules,
    create_terminal_petiolule,
)

__all__ = [
    'load_trunk_internodes',
    'load_leaves',
    'parse_csv_to_branches',
    'calculate_leaf_orientation',
    'leaf_to_petiole_rachis_branches',
    'create_lateral_petiolules',
    'create_terminal_petiolule',
]
