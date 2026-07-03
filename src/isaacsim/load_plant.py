import os
from isaacsim import SimulationApp

# Start Isaac Sim app with GUI enabled (set True for headless/server runs)
simulation_app = SimulationApp({"headless": False})

# Import Omniverse / Isaac Sim modules only AFTER SimulationApp is created
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects.ground_plane import GroundPlane
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Sdf, UsdLux
import omni.kit.actions.core

# ---------------------------------------------------------------------------
# Path setup
# ---------------------------------------------------------------------------
# This script lives at: /src/isaacsim/load_plant.py
# The plant asset lives at: /output/day_1/plant_day1.usda
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(os.path.dirname(SCRIPT_DIR))
PLANT_USD_PATH = os.path.join(PROJECT_ROOT, "output", "day_1", "plant_day1.usda")
PLANT_PRIM_PATH = "/World/Tomato_Plant"

# ---------------------------------------------------------------------------
# World setup
# ---------------------------------------------------------------------------
# Create the World object: manages stage, physics context and simulation stepping
my_world = World(stage_units_in_meters=1.0)

# Add a default ground plane at z = 0
GroundPlane(prim_path="/World/GroundPlane", z_position=0)

action_registry = omni.kit.actions.core.get_action_registry()

# Available modes include: set_lighting_mode_stage, set_lighting_mode_camera, set_lighting_mode_default (rig)
action = action_registry.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera")
action.execute()

# ---------------------------------------------------------------------------
# Load the plant mesh as a USD reference
# ---------------------------------------------------------------------------
# add_reference_to_stage creates a prim at PLANT_PRIM_PATH and references
# the external .usda file into the current stage, instead of merging/importing
# the geometry directly. This keeps the plant asset as a separate, reusable layer.
if os.path.exists(PLANT_USD_PATH):
    add_reference_to_stage(usd_path=PLANT_USD_PATH, prim_path=PLANT_PRIM_PATH)
    print(f"Plant mesh loaded from '{PLANT_USD_PATH}' at prim '{PLANT_PRIM_PATH}'.")
else:
    print(f"WARNING: plant USD file not found at '{PLANT_USD_PATH}'. Skipping reference.")

# Reset the world once before stepping (required to initialize physics handles)
my_world.reset()
print("Stage ready: ground plane, default light and plant reference set up.")

# ---------------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------------
# Runs until the app window is closed by the user (or the app signals a stop),
# instead of a hardcoded number of steps or an unconditional infinite loop.
while simulation_app.is_running():
    my_world.step(render=True)

print("Simulation stopped, shutting down.")

# Cleanly close the Isaac Sim app
simulation_app.close()
