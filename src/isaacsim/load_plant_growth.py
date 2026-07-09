import os
import time
from isaacsim import SimulationApp

# Start Isaac Sim app with GUI enabled (set True for headless/server runs)
simulation_app = SimulationApp({"headless": False})

# Import Omniverse / Isaac Sim modules only AFTER SimulationApp is created
import omni.usd
import omni.kit.actions.core
from isaacsim.core.api import World
from isaacsim.core.api.objects.ground_plane import GroundPlane
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.prims import delete_prim, is_prim_path_valid
from pxr import Sdf, UsdLux

# ---------------------------------------------------------------------------
# Path setup
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(os.path.dirname(SCRIPT_DIR))
PLANT_PRIM_PATH = "/World/Tomato_Plant"

FIRST_DAY = 1
LAST_DAY = 160
SECONDS_PER_DAY = 0.25  # wall-clock seconds between day swaps


def build_plant_path(day: int) -> str:
    day_folder = f"day_{day}"
    day_file = f"plant_day{day}.usda"
    return os.path.join(PROJECT_ROOT, "output", day_folder, day_file)


def load_plant_day(day: int) -> bool:
    """
    Replace the plant prim with the USD reference for the given day.

    IMPORTANT: this function must only ever be called from the MAIN LOOP,
    never from inside a physics callback / event subscription. PhysX does not
    allow subscriptions (which World.reset() re-creates) to be changed while
    an event callback is still executing -- doing so causes a native crash
    (segfault), not a catchable Python exception.
    """
    plant_path = build_plant_path(day)

    if not os.path.exists(plant_path):
        print(f"WARNING: plant USD file not found for day {day}: '{plant_path}'. Skipping.")
        return False

    if is_prim_path_valid(PLANT_PRIM_PATH):
        delete_prim(PLANT_PRIM_PATH)

    add_reference_to_stage(usd_path=plant_path, prim_path=PLANT_PRIM_PATH)
    print(f"Day {day}: loaded '{plant_path}' at '{PLANT_PRIM_PATH}'.")
    return True


# ---------------------------------------------------------------------------
# World setup
# ---------------------------------------------------------------------------
my_world = World(stage_units_in_meters=1.0)

GroundPlane(prim_path="/World/GroundPlane", z_position=0)

action_registry = omni.kit.actions.core.get_action_registry()
action = action_registry.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera")
action.execute()

# ---------------------------------------------------------------------------
# Initial plant load
# ---------------------------------------------------------------------------
current_day = FIRST_DAY
load_plant_day(current_day)

my_world.reset()
print("Stage ready: ground plane, default lighting and initial plant reference set up.")

# ---------------------------------------------------------------------------
# Simulation loop with growth swap handled OUTSIDE any physics event
# ---------------------------------------------------------------------------
# NOTE: no physics callback is used for the swap logic anymore. Instead, we
# track elapsed wall-clock time in plain Python variables in the main loop,
# and only call load_plant_day() / my_world.reset() AFTER my_world.step()
# has fully returned, i.e. outside of any PhysX event context. This is what
# avoids the "Subscription cannot be changed during the event call" crash.
last_swap_time = time.time()

while simulation_app.is_running():
    my_world.step(render=True)

    # All stage-modifying logic happens here, safely outside the physics event.
    if current_day < LAST_DAY:
        now = time.time()
        if now - last_swap_time >= SECONDS_PER_DAY:
            last_swap_time = now
            next_day = current_day + 1
            if load_plant_day(next_day):
                current_day = next_day
                # Reset AFTER the reference swap, and only in the main loop,
                # never inside a callback triggered by a physics event.
                my_world.reset()

print("Simulation stopped, shutting down.")

simulation_app.close()