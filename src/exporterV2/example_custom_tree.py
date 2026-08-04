"""
example_custom_tree.py - Custom Tree Configuration Example

Shows how to create a custom tree configuration and run it.

Run with:
    ~/isaacsim/python.sh src/exporterV2/example_custom_tree.py
"""

# Define custom configuration BEFORE importing main
import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(SCRIPT_DIR))

from exporterV2 import tree_config

# Customize configuration
tree_config.GLOBAL_SCALE = 3.0  # Scale up for better stability

# Define custom tree structure
tree_config.BRANCHES = [
    {
        "id": "main_stem",
        "parent": None,
        "attach_link": None,
        "n_links": 8,
        "radius": 0.08,
        "height": 0.15,
        "tilt": 0.0,
        "rot": 0.0,
    },
    {
        "id": "branch_left",
        "parent": "main_stem",
        "attach_link": 4,
        "n_links": 5,
        "radius": 0.04,
        "height": 0.12,
        "tilt": 35.0,
        "rot": 90.0,
    },
    {
        "id": "branch_right",
        "parent": "main_stem",
        "attach_link": 4,
        "n_links": 5,
        "radius": 0.04,
        "height": 0.12,
        "tilt": 35.0,
        "rot": 270.0,
    },
    {
        "id": "branch_front",
        "parent": "main_stem",
        "attach_link": 6,
        "n_links": 4,
        "radius": 0.03,
        "height": 0.10,
        "tilt": 40.0,
        "rot": 0.0,
    },
    {
        "id": "subbranch_left",
        "parent": "branch_left",
        "attach_link": 3,
        "n_links": 3,
        "radius": 0.02,
        "height": 0.08,
        "tilt": 30.0,
        "rot": 45.0,
    },
]

# Print configuration summary
print("\n" + "=" * 80)
print("  Custom Tree Configuration")
print("=" * 80)
tree_config.print_tree_summary()

# Now run main
from exporterV2.main import main

if __name__ == "__main__":
    main()
