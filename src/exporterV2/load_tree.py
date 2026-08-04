"""
load_tree.py - Isaac Sim Loader

Single entry point: generates the tree USD stage, applies PhysX
configuration, and starts interactive Isaac Sim simulation.

Run with:
    ~/isaacsim/python.sh src/exporterV2/load_tree.py
"""

import os
import sys

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from pxr import UsdPhysics, PhysxSchema, Gf
import omni.usd
import omni.kit.actions.core
from isaacsim.core.api import World

# Add src to path for imports
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_DIR = os.path.dirname(SCRIPT_DIR)
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from exporterV2.usd import build_stage, get_output_usd_path
from exporterV2.physics import apply_physx_scene_settings, apply_physx_articulation_settings

USD_PATH = get_output_usd_path()


# ==============================================================================
# MAIN
# ==============================================================================

print("[INFO] Generating tree USD stage...")
stage, stem_path = build_stage(USD_PATH)

apply_physx_scene_settings(stage)
apply_physx_articulation_settings(stage, stem_path)

stage.GetRootLayer().Save()
print(f"[OK] Stage saved with PhysX config: {USD_PATH}")

omni.usd.get_context().open_stage(USD_PATH)
print(f"[OK] Stage opened in Isaac Sim.")

# Set camera lighting mode if available
try:
    reg = omni.kit.actions.core.get_action_registry()
    action = reg.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera")
    if action:
        action.execute()
except Exception as e:
    print(f"[WARN] Lighting action not available: {e}")

my_world = World(stage_units_in_meters=1.0)
my_world.reset()
print("[OK] Simulation running — close the window to exit.")

while simulation_app.is_running():
    my_world.step(render=True)

print("[INFO] Simulation finished.")
simulation_app.close()
