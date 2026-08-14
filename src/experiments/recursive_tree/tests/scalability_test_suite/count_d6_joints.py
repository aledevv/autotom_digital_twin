"""
count_d6_joints.py - Real D6 Joint Counter from GroIMP CSV data

This script computes the EXACT number of D6 (non-fixed) joints that exporterV2
would generate for each day of simulation. It imports the actual V2 parsing
logic (including TOMATO_PROFILE for lateral opposite pairs) to guarantee 100%
fidelity without the massive overhead of generating 160 USDA files via Isaac Sim.

Usage:
    uv run src/experiments/recursive_tree/tests/scalability_test_suite/count_d6_joints.py
"""

import os
import sys
import glob
import pandas as pd
import re
import tempfile

# Add project root to path so we can import exporterV2
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", "..", "..", ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from exporterV2.adapters.groimp_csv.parser import parse_csv_to_branches, _count_d6_joints
from exporterV2.profiles.tomato_default import TOMATO_PROFILE
from exporterV2.core.optimizations.techniques.base import count_d6_joints

def load_all_days(csv_dir: str, plant_id: int = 1):
    pattern = os.path.join(csv_dir, "graph_day_*.csv")
    files = sorted(glob.glob(pattern))

    if not files:
        raise FileNotFoundError(f"No CSV files found matching: {pattern}")

    results = []
    print(f"Found {len(files)} CSV files in {csv_dir}")
    
    # Use a temporary directory for the JSON exports to avoid polluting the workspace
    with tempfile.TemporaryDirectory() as tmp_dir:
        for fpath in files:
            match = re.search(r"graph_day_(\d+)\.csv", os.path.basename(fpath))
            if not match:
                continue
            day = int(match.group(1))

            try:
                # Call the EXACT SAME pipeline V2 uses for production!
                # Including TOMATO_PROFILE which filters non-opposite lateral pairs
                branches, terminal_bodies, _ = parse_csv_to_branches(
                    day=day,
                    plant_id=plant_id,
                    profile=TOMATO_PROFILE,
                    include_terminal_bodies=True
                )
                
                # Count total D6 joints
                total_d6 = count_d6_joints(branches)
                
                # Breakdown by organ type
                trunk_d6 = sum(b.get("n_links", 1) for b in branches if "trunk" in b["id"].lower() and b.get("joint_type", "d6").lower() != "fixed")
                lateral_d6 = sum(b.get("n_links", 1) for b in branches if "branch_" in b["id"].lower() and b.get("joint_type", "d6").lower() != "fixed")
                leaves_d6 = sum(b.get("n_links", 1) for b in branches if "leaf_" in b["id"].lower() and b.get("joint_type", "d6").lower() != "fixed")
                trusses_d6 = sum(b.get("n_links", 1) for b in branches if "truss" in b["id"].lower() and b.get("joint_type", "d6").lower() != "fixed")
                
                counts = {
                    "day": day,
                    "total": total_d6,
                    "trunk": trunk_d6,
                    "lateral": lateral_d6,
                    "leaves": leaves_d6,
                    "trusses": trusses_d6
                }
                
                results.append(counts)
                print(f"  Day {day:3d}: total={total_d6:4d} (trunk={trunk_d6}, lateral={lateral_d6}, leaves={leaves_d6}, trusses={trusses_d6})")
                      
            except Exception as e:
                print(f"  [WARNING] Day {day}: {e}")

    if not results:
        raise ValueError("No valid data extracted from CSVs")

    result_df = pd.DataFrame(results).sort_values("day").reset_index(drop=True)
    return result_df

if __name__ == "__main__":
    CSV_DIR = os.path.join(PROJECT_ROOT, "data", "simulation_output", "dynamic_output", "graphs")
    OUTPUT_CSV = os.path.join(SCRIPT_DIR, "d6_joints_per_day.csv")

    print("=" * 60)
    print("  D6 Joint Counter using ExporterV2 API")
    print("=" * 60)
    
    result_df = load_all_days(CSV_DIR)
    
    print("\n" + "=" * 60)
    print(f"  Summary: {len(result_df)} days processed")
    print(f"  D6 joints range: {result_df['total'].min()} – {result_df['total'].max()}")
    print("=" * 60)

    result_df.to_csv(OUTPUT_CSV, index=False)
    print(f"\nResults saved to: {OUTPUT_CSV}")
