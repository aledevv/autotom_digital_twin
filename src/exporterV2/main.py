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
import time

# Parse arguments BEFORE initializing SimulationApp
parser = argparse.ArgumentParser(description="exporterV2 Tree Loader")
parser.add_argument("--day", type=int, help="Load plant from CSV for specified day")
parser.add_argument("--plant-id", type=int, default=1, help="Plant ID (default: 1)")
parser.add_argument("--optimize", action="store_true", help="Apply joint-budget optimization")
parser.add_argument(
    "--branch-backend",
    choices=("legacy", "skinned"),
    default="legacy",
    help="Vegetative branch backend (default: legacy)",
)
parser.add_argument(
    "--skinning-profile",
    action="store_true",
    help="Print detailed runtime timing diagnostics for the skinned backend",
)
parser.add_argument(
    "--skinning-no-sync",
    action="store_true",
    help="Diagnostic mode: keep skinned meshes authored but skip runtime SkelAnimation sync",
)
parser.add_argument(
    "--skinning-sync-every",
    type=int,
    default=1,
    metavar="N",
    help="Update skinning every N simulation frames (default: 1)",
)
parser.add_argument(
    "--skinning-profile-window",
    type=int,
    default=240,
    metavar="N",
    help="Number of frames per performance report (default: 240)",
)
args = parser.parse_args()

if args.skinning_sync_every < 1:
    parser.error("--skinning-sync-every must be >= 1")
if args.skinning_profile_window < 1:
    parser.error("--skinning-profile-window must be >= 1")

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
        # Load from CSV
        from exporterV2.adapters.groimp_csv import parse_csv_to_branches
        print(f"\n[CONFIG] Loading plant from CSV (day {args.day}, plant_id {args.plant_id})")
        branches, terminal_bodies, json_path = parse_csv_to_branches(
            args.day,
            args.plant_id,
            include_terminal_bodies=True,
        )
        print(f"[CONFIG] Configuration saved: {json_path}")
        
        # Use day-specific USD path
        base_path = get_output_usd_path()
        usd_path = base_path.replace("tree_v2.usda", f"tree_v2_day_{args.day}.usda")
    else:
        # Use static config
        print(f"\n[CONFIG] Using static configuration from tree_config.py")
        branches = BRANCHES
        terminal_bodies = []
        usd_path = get_output_usd_path()

    # Apply the configured upper bound before any budget optimization. CSV
    # branches are already limited before JSON export; this call is idempotent.
    branches, resolution_changes = limit_branch_resolution(branches)
    print(
        f"[CONFIG] Branch resolution cap applied: "
        f"max={BranchResolutionConfig.MAX_LINKS_PER_BRANCH}, "
        f"capped={len(resolution_changes)}"
    )
    
    optimization_report = None

    # Apply optimization if requested
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
            # Budget impossible (below lower bound)
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
    
    # Set camera lighting mode
    try:
        reg = omni.kit.actions.core.get_action_registry()
        action = reg.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera")
        if action:
            action.execute()
            print("  ✓ Camera lighting applied")
    except Exception as e:
        print(f"  ⚠ Lighting action not available: {e}")
    
    # Initialize simulation
    opened_stage = omni.usd.get_context().get_stage()
    skinning_runtime = (
        SkinningRuntime.discover(opened_stage)
        if args.branch_backend == "skinned"
        else None
    )
    if skinning_runtime is not None:
        stats = skinning_runtime.stats()
        print(
            "  ✓ Skinning runtime: "
            f"axes={stats['visual_axes']}, bones={stats['bones']}, "
            f"single_bone_axes={stats['single_bone_axes']}, "
            f"multi_bone_axes={stats['multi_bone_axes']}, "
            f"vertices={stats['mesh_vertices']}, "
            f"USD writes/sync={stats['usd_attr_writes_per_sync']}"
        )
        if args.skinning_no_sync:
            print("  ⚠ Skinning runtime sync DISABLED for diagnostics")
        elif args.skinning_sync_every > 1:
            print(f"  ⚠ Skinning runtime sync every {args.skinning_sync_every} frames")

    my_world = World(stage_units_in_meters=1.0)
    my_world.reset()
    if skinning_runtime is not None:
        configure_physx_mouse_interaction(simulation_app)
        if not args.skinning_no_sync:
            skinning_runtime.sync()
        simulation_app.update()

    # Optimization report goes to console only when optimization was actually run.
    if optimization_report is not None:
        print(str(optimization_report))
    
    print("\n" + "=" * 80)
    print("  ✓ Simulation running — close the window to exit")
    print("=" * 80 + "\n")

    profile_window = args.skinning_profile_window
    profile_frames = 0
    sync_calls = 0
    profile_wall_s = 0.0
    profile_physics_s = 0.0
    profile_sync_s = 0.0
    profile_render_s = 0.0
    sync_detail = {
        "cache_clear_s": 0.0,
        "xform_reads_s": 0.0,
        "local_matrices_s": 0.0,
        "decompose_s": 0.0,
        "usd_writes_s": 0.0,
        "total_s": 0.0,
    }
    frame_index = 0
    
    # Run simulation loop
    while simulation_app.is_running():
        if skinning_runtime is None:
            my_world.step(render=True)
            continue

        frame_start = time.perf_counter()

        physics_start = time.perf_counter()
        my_world.step(render=False)
        physics_end = time.perf_counter()

        should_sync = (
            not args.skinning_no_sync
            and frame_index % args.skinning_sync_every == 0
        )

        if should_sync:
            sync_start = time.perf_counter()
            if args.skinning_profile:
                detail = skinning_runtime.sync_profiled()
                for key in sync_detail:
                    sync_detail[key] += detail[key]
            else:
                skinning_runtime.sync()
            sync_end = time.perf_counter()
            sync_calls += 1
        else:
            sync_start = sync_end = time.perf_counter()

        render_start = time.perf_counter()
        simulation_app.update()
        render_end = time.perf_counter()

        frame_end = time.perf_counter()
        frame_index += 1

        if args.skinning_profile:
            profile_frames += 1
            profile_wall_s += frame_end - frame_start
            profile_physics_s += physics_end - physics_start
            profile_sync_s += sync_end - sync_start
            profile_render_s += render_end - render_start

            if profile_frames >= profile_window:
                frame_divisor = float(profile_frames)
                sync_divisor = float(max(sync_calls, 1))
                wall_ms = profile_wall_s / frame_divisor * 1000.0
                physics_ms = profile_physics_s / frame_divisor * 1000.0
                sync_ms_per_frame = profile_sync_s / frame_divisor * 1000.0
                sync_ms_per_call = profile_sync_s / sync_divisor * 1000.0
                render_ms = profile_render_s / frame_divisor * 1000.0
                fps = 1000.0 / wall_ms if wall_ms > 0.0 else 0.0

                print(
                    "\n[SKIN-PERF] "
                    f"frames={profile_frames} sync_calls={sync_calls} "
                    f"wall={wall_ms:.2f}ms (~{fps:.1f} FPS) | "
                    f"physics={physics_ms:.2f}ms | "
                    f"sync/frame={sync_ms_per_frame:.2f}ms | "
                    f"sync/call={sync_ms_per_call:.2f}ms | "
                    f"render={render_ms:.2f}ms"
                )

                if sync_calls > 0:
                    print(
                        "[SKIN-SYNC] "
                        f"cache={sync_detail['cache_clear_s'] / sync_divisor * 1000.0:.3f}ms | "
                        f"xforms={sync_detail['xform_reads_s'] / sync_divisor * 1000.0:.3f}ms | "
                        f"locals={sync_detail['local_matrices_s'] / sync_divisor * 1000.0:.3f}ms | "
                        f"decompose={sync_detail['decompose_s'] / sync_divisor * 1000.0:.3f}ms | "
                        f"USD-writes={sync_detail['usd_writes_s'] / sync_divisor * 1000.0:.3f}ms | "
                        f"internal-total={sync_detail['total_s'] / sync_divisor * 1000.0:.3f}ms"
                    )

                profile_frames = 0
                sync_calls = 0
                profile_wall_s = 0.0
                profile_physics_s = 0.0
                profile_sync_s = 0.0
                profile_render_s = 0.0
                for key in sync_detail:
                    sync_detail[key] = 0.0
    
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
