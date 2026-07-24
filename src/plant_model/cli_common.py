"""
cli_common.py

Shared CLI/simulation-loop boilerplate for the v1 (main.py) and v2
(main_builder.py) Isaac Sim entry points: argparse setup, CSV/output path
resolution, and the "open stage + set lighting + run World.step loop until
window closes" viewport boilerplate that was previously duplicated almost
verbatim between the two entry points.

Each main*.py stays a thin wrapper that:
  1. Bootstraps SimulationApp (must happen before this module is imported,
     since importing pxr/omni before SimulationApp starts breaks Isaac Sim).
  2. Calls `parse_args()` here for CLI args.
  3. Calls `resolve_paths()` here for CSV/output path resolution.
  4. Calls its own pipeline function (export_plant_usd / build_plant_stage).
  5. Calls `run_viewport_loop()` here to open + simulate the result.
"""

import os
import argparse


def parse_args(description: str) -> argparse.Namespace:
    """Argument parser shared by main.py and main_builder.py.

    Both entry points previously defined an identical set of flags
    (--day, --plant, --csv, --out) with only the `description` differing.
    """
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--day", type=int, default=1, help="Simulation day.")
    parser.add_argument("--plant", type=int, default=1, help="Plant ID.")
    parser.add_argument("--csv", default=None, help="CSV path (default: resolved from day).")
    parser.add_argument("--out", default=None, help="Output .usda path (default: resolved from day).")
    return parser.parse_args()


def resolve_paths(args: argparse.Namespace, project_root: str, out_filename_template: str) -> tuple[str, str]:
    """Resolves (csv_path, out_path) from CLI args + day, creating the output directory.

    `out_filename_template` is a format string using `{day}`, e.g.
    "plant_day{day}_static.usda" (v1) or "plant_day{day}_builder.usda" (v2).
    """
    csv_path = args.csv or os.path.join(
        project_root, "data/simulation_output/dynamic_output/graphs", f"graph_day_{args.day}.csv"
    )
    out_path = args.out or os.path.join(
        project_root, "output", f"day_{args.day}", out_filename_template.format(day=args.day)
    )
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    return csv_path, out_path


def set_camera_lighting_mode():
    """Best-effort: switch viewport lighting to 'camera' mode. Never fatal."""
    import omni.kit.actions.core
    try:
        action_registry = omni.kit.actions.core.get_action_registry()
        action = action_registry.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera")
        if action:
            action.execute()
    except Exception as e:
        print(f"[WARN] Could not set lighting mode: {e}")


def run_viewport_loop(simulation_app, out_path: str):
    """Opens `out_path` in the Isaac Sim viewport and runs the sim loop until
    the window is closed. Shared tail-end of both main.py and main_builder.py.
    """
    import omni.usd
    from isaacsim.core.api import World

    omni.usd.get_context().open_stage(out_path)
    print(f"[OK] Stage opened in Isaac Sim: {out_path}")

    set_camera_lighting_mode()

    world = World(stage_units_in_meters=1.0)
    world.reset()
    print("[OK] Simulation running — close the window to exit.")

    while simulation_app.is_running():
        world.step(render=True)

    print("Simulation ended.")
    simulation_app.close()
