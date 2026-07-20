import sys
import os

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

SCRIPT_DIR = "/home/alessandro/isaacsim/autotom_digital_twin/src/experiments/articulation_subbranch"
sys.path.insert(0, SCRIPT_DIR)

from generate_generalized_articulation_usda import build_stage_from_csv_data, get_output_usd_path
import csv

USD_PATH = get_output_usd_path()
CSV_PATH = USD_PATH.replace(".usda", "_config.csv")

csv_data = []
with open(CSV_PATH, "r") as f:
    reader = csv.DictReader(f)
    for row in reader:
        csv_data.append(row)

stage, stem_path = build_stage_from_csv_data(USD_PATH, csv_data)
stage.GetRootLayer().Save()

print(f"[OK] Deterministic stage built from CSV successfully!")

simulation_app.close()
