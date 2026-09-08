from pathlib import Path

USD_PATH = Path(__file__).resolve().parent / "generated_leaflet.usda"

if not USD_PATH.is_file():
    raise SystemExit(
        f"Generated leaflet USD not found: {USD_PATH}\n"
        "Run generate_real_leaf_shape.py first."
    )

from isaacsim import SimulationApp


# Isaac Sim must be bootstrapped before omni / pxr imports.
simulation_app = SimulationApp({
    "headless": False,
})


import omni.usd

from isaacsim.core.api import World


def main():
    usd_path = USD_PATH

    print("[1] Opening leaflet USD")
    print("    ", usd_path)

    context = omni.usd.get_context()
    context.open_stage(str(usd_path))

    # Give Kit a few updates so the stage is loaded/rendered.
    for _ in range(10):
        simulation_app.update()

    print("[2] Stage opened")

    world = World(stage_units_in_meters=1.0)
    world.reset()

    print("[OK] Isaac Sim running")
    print("     Close the Isaac Sim window to exit.")

    while simulation_app.is_running():
        world.step(render=True)


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
