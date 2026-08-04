"""
adapters - Data Source Adapters

Adapters convert various data sources to the generic BRANCHES format.
"""

from . import groimp_csv

__all__ = ["groimp_csv"]
