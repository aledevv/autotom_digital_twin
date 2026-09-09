from pathlib import Path

from isaacsim import SimulationApp


simulation_app = SimulationApp({
    "headless": False,
})


import omni.usd
from isaacsim.core.api import World


def main():
    usd_path = (
        Path(__file__).resolve().parent
        / "generated_leaflet_3d.usda"
    )

    if not usd_path.exists():
        raise FileNotFoundError(
            f"Missing USD: {usd_path}\n"
            "Run generate_real_leaf_shape_3d.py first."
        )

    print("[1] Opening:", usd_path)

    context = omni.usd.get_context()
    context.open_stage(str(usd_path))

    # Allow stage to finish loading.
    for _ in range(10):
        simulation_app.update()

    world = World(
        stage_units_in_meters=1.0
    )

    world.reset()

    print("[OK] Leaf loaded in Isaac Sim")
    print("Close the window to exit.")

    while simulation_app.is_running():
        world.step(render=True)


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()