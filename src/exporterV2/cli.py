"""Serverless ExporterV2 command line for canonical PlantState inputs."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

from plant_state import PlantStateValidationError, load_plant_state

from .plant_state_adapter import (
    DEBUG_PROFILES,
    V2PlantStateError,
    build_v2_authoring_plan,
)
from .plant_state_usd import (
    DRIVE_SCALE_CHOICES,
    V2ExportError,
    audit_v2_stage,
    export_plant_state_v2,
    manifest_path_for,
    save_v2_manifest,
)
from .plant_state_branches import POSE_MODES
from .plant_state_legacy_backend import (
    INCREMENTAL_PROFILES,
    StemCheckpointError,
    export_incremental_checkpoint,
)


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def _positive_int(value: str) -> int:
    result = int(value)
    if result < 1:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return result


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate an articulated ExporterV2 USDA from plant_state/1.0."
    )
    parser.add_argument("--day", type=_positive_int, required=True)
    parser.add_argument("--plant-id", type=_positive_int, default=1)
    parser.add_argument("--input", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--physics-preset", choices=("locked", "flexible"), default="locked"
    )
    parser.add_argument("--optimize", action="store_true")
    parser.add_argument("--allow-near-budget", action="store_true")
    parser.add_argument(
        "--stiffness-scale", type=float, choices=(1.0, 2.0, 4.0), default=1.0
    )
    parser.add_argument(
        "--leaf-stiffness-scale",
        type=float,
        choices=DRIVE_SCALE_CHOICES,
        default=1.0,
    )
    parser.add_argument(
        "--truss-stiffness-scale",
        type=float,
        choices=DRIVE_SCALE_CHOICES,
        default=1.0,
    )
    parser.add_argument("--physics-hz", type=int, choices=(480, 960), default=480)
    parser.add_argument(
        "--debug-profile", choices=DEBUG_PROFILES, default="full"
    )
    parser.add_argument(
        "--pose-mode",
        choices=POSE_MODES,
        default="canonical",
        help="PlantState rest pose for the conservative V2 backend.",
    )
    parser.add_argument("--debug-no-colliders", action="store_true")
    parser.add_argument("--debug-no-drives", action="store_true")
    parser.add_argument("--debug-no-articulation", action="store_true")
    parser.add_argument(
        "--generate-only",
        action="store_true",
        help="Compatibility flag; module execution is always serverless.",
    )
    return parser


def generate_from_args(args: argparse.Namespace):
    source = args.input
    if source is None:
        suffix = "" if args.plant_id == 1 else f"_plant_{args.plant_id}"
        source = PROJECT_ROOT / "data" / "plant_states" / (
            f"plant_state_day_{args.day}{suffix}.json"
        )
    destination = args.output
    if destination is None:
        suffix = "" if args.plant_id == 1 else f"_plant_{args.plant_id}"
        if args.debug_profile == "full":
            destination = PROJECT_ROOT / "data" / "usd_models" / (
                f"tree_v2_day_{args.day}{suffix}.usda"
            )
        else:
            destination = Path("/tmp/autotom-phase-j-debug") / f"day_{args.day}" / (
                f"tree_v2_day_{args.day}{suffix}_{args.debug_profile}.usda"
            )
    state = load_plant_state(source)
    if state.metadata.plant_id != args.plant_id:
        raise V2PlantStateError(
            f"requested plant_id {args.plant_id}, input contains {state.metadata.plant_id}"
        )
    if state.metadata.simulation_time is None or int(state.metadata.simulation_time) != args.day:
        raise V2PlantStateError(
            f"requested day {args.day}, input metadata contains "
            f"{state.metadata.simulation_time}"
        )
    if args.debug_profile in INCREMENTAL_PROFILES:
        if args.debug_no_colliders or args.debug_no_drives or args.debug_no_articulation:
            raise V2PlantStateError(
                f"the conservative {args.debug_profile} checkpoint requires "
                "colliders, joints, and articulation"
            )
        return (
            state,
            *export_incremental_checkpoint(
                state,
                destination,
                debug_profile=args.debug_profile,
                pose_mode=args.pose_mode,
                physics_preset=args.physics_preset,
                physics_hz=args.physics_hz,
            ),
        )
    plan = build_v2_authoring_plan(
        state,
        physics_preset=args.physics_preset,
        allow_near_budget=args.allow_near_budget,
        optimize=args.optimize,
        debug_profile=args.debug_profile,
        colliders_enabled=not args.debug_no_colliders,
        drives_enabled=not args.debug_no_drives,
        articulation_enabled=not args.debug_no_articulation,
    )
    usd_path = export_plant_state_v2(
        plan,
        destination,
        stiffness_scale=args.stiffness_scale,
        leaf_stiffness_scale=args.leaf_stiffness_scale,
        truss_stiffness_scale=args.truss_stiffness_scale,
        physics_hz=args.physics_hz,
    )
    manifest = audit_v2_stage(plan, usd_path)
    manifest_path = save_v2_manifest(manifest, manifest_path_for(usd_path))
    if manifest.errors:
        raise V2ExportError("; ".join(manifest.errors))
    return state, plan, usd_path, manifest_path


def main(argv: list[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    try:
        state, plan, usd_path, manifest_path = generate_from_args(args)
    except (
        FileNotFoundError,
        PlantStateValidationError,
        V2PlantStateError,
        V2ExportError,
        StemCheckpointError,
        ValueError,
    ) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1
    physical_links = getattr(plan, "physical_link_count", None)
    if physical_links is None:
        physical_links = len(plan.physical_links)
    print(
        f"[OK] V2 PlantState stage: day={args.day}, plant_id={args.plant_id}, "
        f"axes={len(state.axes)}, physical_links={physical_links}, "
        f"d6={plan.predicted_d6_joints}, spheres={len(state.spheres)}, "
        f"profile={plan.debug_profile}, pose_mode={args.pose_mode}"
    )
    print(f"[OK] USDA: {usd_path}")
    print(f"[OK] Manifest: {manifest_path}")
    return 0
