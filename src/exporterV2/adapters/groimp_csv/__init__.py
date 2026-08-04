"""
groimp_csv - groIMP CSV Adapter

Parses groIMP CSV export files and converts to generic BRANCHES format.
"""

from .parser import parse_csv_to_branches

__all__ = ["parse_csv_to_branches"]
