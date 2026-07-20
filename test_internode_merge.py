import sys
import os

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

SCRIPT_DIR = "/home/alessandro/isaacsim/autotom_digital_twin/src/experiments/articulation_subbranch"
sys.path.insert(0, SCRIPT_DIR)

from generate_generalized_articulation_usda import build_stage, get_output_usd_path

USD_PATH = get_output_usd_path()

stage, stem_path, csv_data = build_stage(USD_PATH)

print(f"[OK] Deterministic stage built successfully with {len(csv_data)} CSV rows.")

simulation_app.close()
