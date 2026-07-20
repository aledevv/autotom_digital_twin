import os
import csv
import sys
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

# Add the articulation_subbranch folder to python path
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(script_dir, "src/experiments/articulation_subbranch"))
from generate_generalized_articulation_usda import build_stage_from_csv_data

csv_path = os.path.join(script_dir, "data/usd_models/generated_generalized_articulation_config.csv")
csv_data = []
with open(csv_path, "r") as f:
    reader = csv.DictReader(f)
    for row in reader:
        csv_data.append(row)

output_path = "/tmp/test_leaf_gen.usda"
stage, stem_path = build_stage_from_csv_data(output_path, csv_data)
stage.GetRootLayer().Save()

print(f"SUCCESS: USD generated at {output_path}")
