"""
main.py - exporterV2 Entry Point

Generates tree USD and loads it in Isaac Sim in one step.

Run with static config:
    ~/isaacsim/python.sh src/exporterV2/main.py

Run with CSV data:
    ~/isaacsim/python.sh src/exporterV2/main.py --day 1

Or use wrapper script:
    ./run_mainV2.sh [--day N]
"""

import os
import sys
import argparse

# Parse arguments BEFORE initializing SimulationApp
parser = argparse.ArgumentParser(description="exporterV2 Tree Loader")
parser.add_argument("--day", type=int, help="Load plant from CSV for specified day")
parser.add_argument("--plant-id", type=int, default=1, help="Plant ID (default: 1)")
parser.add_argument("--optimize", action="store_true", help="Apply joint-budget optimization")
parser.add_argument(
    "--branch-backend",
    choices=("legacy", "skinned"),
    default="skinned",
    help="Vegetative branch backend (default: skinned)",
)
parser.add_argument(
    "--skinning-visual-mode",
    choices=(
        "skinned",
        "static",
        "rigid-single",
        "segmented",
    ),
    default="segmented",
    help=(
        "Visual mode for the skinned backend: per-axis UsdSkel, static smooth "
        "mesh benchmark, rigid single-bone optimization, or rigid segmented organic meshes"
    ),
)
args = parser.parse_args()


# The builder reads this without changing the public build_stage API.
os.environ["AUTOTOM_SKINNING_VISUAL_MODE"] = args.skinning_visual_mode

# Bootstrap Isaac Sim
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# Now import USD and Isaac modules
from pxr import UsdPhysics, PhysxSchema, Gf
import omni.usd
import omni.kit.actions.core
from isaacsim.core.api import World

# Import our modules (need to add parent to path)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(SCRIPT_DIR))

from exporterV2.core.usd import build_stage, get_output_usd_path
from exporterV2.core.physics import apply_physx_scene_settings, apply_physx_articulation_settings
from exporterV2.core.tree_config import BRANCHES, BranchResolutionConfig, limit_branch_resolution, MAX_N_JOINTS
from exporterV2.core.optimizations.techniques.base import count_d6_joints
from exporterV2.core.skinning import SkinningRuntime
from exporterV2.core.skinning.runtime import configure_physx_mouse_interaction

# ANSI color codes for terminal output
RED = '\033[91m'
BLUE = '\033[94m'
RESET = '\033[0m'


# ==============================================================================
# MAIN
# ==============================================================================

def main():
    """Generate tree USD and load in Isaac Sim."""
    print("=" * 80)
    print("  exporterV2 - Tree Model Generator & Loader")
    print("=" * 80)

    # Determine configuration source and USD path
    if args.day is not None:
        from exporterV2.adapters.groimp_csv import parse_csv_to_branches
        print(f"\n[CONFIG] Loading plant from CSV (day {args.day}, plant_id {args.plant_id})")
        branches, terminal_bodies, json_path = parse_csv_to_branches(
            args.day,
            args.plant_id,
            include_terminal_bodies=True,
        )
        print(f"[CONFIG] Configuration saved: {json_path}")

        base_path = get_output_usd_path()
        usd_path = base_path.replace("tree_v2.usda", f"tree_v2_day_{args.day}.usda")
    else:
        print("\n[CONFIG] Using static configuration from tree_config.py")
        branches = BRANCHES
        terminal_bodies = []
        usd_path = get_output_usd_path()

    branches, resolution_changes = limit_branch_resolution(branches)
    print(
        f"[CONFIG] Branch resolution cap applied: "
        f"max={BranchResolutionConfig.MAX_LINKS_PER_BRANCH}, "
        f"capped={len(resolution_changes)}"
    )

    optimization_report = None

    if args.optimize:
        print("\n[OPTIMIZE] Applying joint-budget optimization...")
        try:
            from exporterV2.core.optimizations import BudgetOptimizer

            optimizer = BudgetOptimizer(max_joints=MAX_N_JOINTS)
            branches, optimization_report = optimizer.optimize(
                branches,
                terminal_body_count=len(terminal_bodies),
            )

            branch_ids = {b["id"] for b in branches}
            terminal_bodies = [
                body for body in terminal_bodies
                if body.get("parent_branch_id") in branch_ids
            ]
        except ValueError as e:
            print(f"\n{RED}[ERROR] Optimization failed: {e}{RESET}", file=sys.stderr)
            print(f"{BLUE}[HINT] Remove --optimize flag to generate unoptimized USD{RESET}", file=sys.stderr)
            simulation_app.close()
            sys.exit(1)
        except Exception as e:
            print(f"\n{RED}[ERROR] Unexpected optimization error: {e}{RESET}", file=sys.stderr)
            import traceback
            traceback.print_exc()
            simulation_app.close()
            sys.exit(1)

    # Step 1: Generate USD
    print("\n[STEP 1/3] Generating tree USD stage...")
    total_links = sum(b["n_links"] for b in branches)
    total_d6_joints = count_d6_joints(branches)
    print(
        f"  Configuration: {len(branches)} branches, {total_links} total links, "
        f"{total_d6_joints} D6 joints, {len(terminal_bodies)} terminal bodies"
    )
    if args.branch_backend == "skinned":
        print(f"  Skinning visual mode: {args.skinning_visual_mode}")

    stage, stem_path = build_stage(
        usd_path,
        branches=branches,
        terminal_bodies=terminal_bodies,
        branch_backend=args.branch_backend,
    )

    # Step 2: Apply PhysX settings
    print("\n[STEP 2/3] Applying PhysX configuration...")
    apply_physx_scene_settings(stage)
    apply_physx_articulation_settings(stage, stem_path)

    stage.GetRootLayer().Save()
    print(f"  ✓ Stage saved: {usd_path}")

    # Step 3: Load in Isaac Sim
    print("\n[STEP 3/3] Loading in Isaac Sim...")
    omni.usd.get_context().open_stage(usd_path)
    print("  ✓ Stage opened")
    if args.branch_backend == "skinned":
        configure_physx_mouse_interaction(simulation_app)

    try:
        reg = omni.kit.actions.core.get_action_registry()
        action = reg.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera")
        if action:
            action.execute()
            print("  ✓ Camera lighting applied")
    except Exception as e:
        print(f"  ⚠ Lighting action not available: {e}")

    opened_stage = omni.usd.get_context().get_stage()
    skinning_runtime = None
    non_runtime_modes = ("static", "segmented")
    if args.branch_backend == "skinned" and args.skinning_visual_mode not in non_runtime_modes:
        candidate = SkinningRuntime.discover(opened_stage)
        if candidate.branch_count > 0:
            skinning_runtime = candidate
            print(
                f"  ✓ Runtime skinning: {candidate.branch_count} skeleton runtime(s), "
                f"{candidate.bone_count} bones"
            )
        else:
            print("  ✓ No runtime-skinned axes remain")
    elif args.branch_backend == "skinned" and args.skinning_visual_mode == "static":
        print("  ✓ Static visual benchmark: no UsdSkel runtime")
    elif args.branch_backend == "skinned" and args.skinning_visual_mode == "segmented":
        label = "Segmented organic visuals + terminal forks"
        print(f"  ✓ {label}: no UsdSkel runtime")

    my_world = World(stage_units_in_meters=1.0)
    my_world.reset()
    if args.branch_backend == "skinned":
        configure_physx_mouse_interaction(simulation_app)
    if skinning_runtime is not None:
        skinning_runtime.sync()
        simulation_app.update()

    if optimization_report is not None:
        print(str(optimization_report))

    print("\n" + "=" * 80)
    print("  ✓ Simulation running — close the window to exit")
    print("=" * 80 + "\n")

    while simulation_app.is_running():
        if skinning_runtime is None:
            my_world.step(render=True)
        else:
            my_world.step(render=False)
            skinning_runtime.sync()
            simulation_app.update()

    print("\n[INFO] Simulation finished.")
    simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.")
        simulation_app.close()
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
        simulation_app.close()
        sys.exit(1)
