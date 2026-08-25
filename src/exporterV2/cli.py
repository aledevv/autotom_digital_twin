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
from .plant_state_branches import (
    APPENDAGE_POSE_MODES,
    LATERAL_JOINT_POLICIES,
    LEAF_JOINT_POLICIES,
    POSE_MODES,
    TRUSS_CALIBRATION_PRESETS,
    TRUSS_DAMPING_CHOICES,
    VISUAL_QUALITY_MODES,
)
from .plant_state_legacy_backend import (
    INITIAL_OVERLAP_POLICIES,
    INCREMENTAL_PROFILES,
    StemCheckpointError,
    TERMINAL_SOLVER_PRESETS,
    TRUSS_ARMATURE_MULTIPLIERS,
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
        "--physics-preset", choices=("locked", "flexible"), default="flexible"
    )
    parser.add_argument("--optimize", action="store_true")
    parser.add_argument("--allow-near-budget", action="store_true")
    parser.add_argument(
        "--initial-overlap-policy",
        choices=INITIAL_OVERLAP_POLICIES,
        default="filter",
        help="Filter precise authored overlap pairs or fail the canonical audit.",
    )
    # WARNING: this restores one rigid body, collider set, and D6 joint per
    # petiolule.  It is intentionally off because it can substantially slow
    # down and destabilize large PlantState articulations.
    parser.add_argument("--physical-petiolules", action="store_true")
    parser.add_argument(
        "--allow-over-budget",
        action="store_true",
        help="Unsafe diagnostic override; requires --physical-petiolules.",
    )
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
        "--debug-profile", choices=DEBUG_PROFILES, default="truss-supports"
    )
    parser.add_argument(
        "--allow-experimental-fruit-physics",
        action="store_true",
        help=(
            "Required for the unsupported full profile with physical fruits. "
            "The stable default stops at dynamic truss supports."
        ),
    )
    parser.add_argument(
        "--pose-mode",
        choices=POSE_MODES,
        default="canonical",
        help="PlantState rest pose for the conservative V2 backend.",
    )
    parser.add_argument(
        "--appendage-pose-mode",
        choices=APPENDAGE_POSE_MODES,
        default="v2-aesthetic",
        help="Leaflet/pedicel authored pose: historical V2 recipe or raw GroIMP.",
    )
    parser.add_argument(
        "--leaf-joint-policy",
        choices=LEAF_JOINT_POLICIES,
        default="distributed",
        help=(
            "PlantState leaf supports: optimized fixes rachides to petioles; "
            "distributed restores D6 bending along both."
        ),
    )
    parser.add_argument(
        "--lateral-joint-policy",
        choices=LATERAL_JOINT_POLICIES,
        default="dynamic",
        help="PlantState laterals remain D6 or become a fixed support scaffold.",
    )
    parser.add_argument(
        "--truss-calibration-preset",
        choices=TRUSS_CALIBRATION_PRESETS,
        default="current",
        help="In-memory day-160 truss calibration; does not rewrite tree_config.",
    )
    parser.add_argument(
        "--truss-damping-override",
        type=float,
        choices=TRUSS_DAMPING_CHOICES,
        help="Override both truss damping ratios after selecting a preset.",
    )
    parser.add_argument(
        "--truss-armature-multiplier",
        type=float,
        choices=TRUSS_ARMATURE_MULTIPLIERS,
        default=0.0,
        help="Fallback armature: 0, 1x or 4x each truss link local inertia.",
    )
    parser.add_argument(
        "--terminal-solver-preset",
        choices=TERMINAL_SOLVER_PRESETS,
        default="current",
        help="Tomato solver iterations: current (32/1) or stabilized (64/4).",
    )
    parser.add_argument(
        "--visual-quality",
        choices=VISUAL_QUALITY_MODES,
        default="realistic",
        help=(
            "PlantState skinned mesh quality: realistic restores the original "
            "V2 sampling; performance keeps the lighter diagnostic mesh."
        ),
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
    if args.allow_experimental_fruit_physics and args.debug_profile != "full":
        raise V2PlantStateError(
            "--allow-experimental-fruit-physics is valid only with "
            "--debug-profile full"
        )
    if args.debug_profile == "full" and not args.allow_experimental_fruit_physics:
        raise V2PlantStateError(
            "full PlantState fruit physics is unsupported and requires "
            "--allow-experimental-fruit-physics; use the default "
            "truss-supports profile"
        )
    if args.debug_profile == "full":
        print(
            "[WARNING] physical PlantState fruits are experimental and "
            "unsupported on mature plants; no stability guarantee is made",
            file=sys.stderr,
        )
    source = args.input
    if source is None:
        suffix = "" if args.plant_id == 1 else f"_plant_{args.plant_id}"
        source = PROJECT_ROOT / "data" / "plant_states" / (
            f"plant_state_day_{args.day}{suffix}.json"
        )
    destination = args.output
    if destination is None:
        suffix = "" if args.plant_id == 1 else f"_plant_{args.plant_id}"
        if args.debug_profile == "truss-supports":
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
                appendage_pose_mode=args.appendage_pose_mode,
                physics_preset=args.physics_preset,
                physics_hz=args.physics_hz,
                leaf_joint_policy=args.leaf_joint_policy,
                lateral_joint_policy=args.lateral_joint_policy,
                truss_calibration_preset=args.truss_calibration_preset,
                truss_damping_override=args.truss_damping_override,
                truss_armature_multiplier=args.truss_armature_multiplier,
                terminal_solver_preset=args.terminal_solver_preset,
                visual_quality=args.visual_quality,
                physical_petiolules=args.physical_petiolules,
                initial_overlap_policy=args.initial_overlap_policy,
                allow_near_budget=args.allow_near_budget,
                allow_over_budget=args.allow_over_budget,
                allow_experimental_fruit_physics=(
                    args.allow_experimental_fruit_physics
                ),
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
        f"profile={plan.debug_profile}, pose_mode={args.pose_mode}, "
        f"appendage_pose_mode={args.appendage_pose_mode}, "
        f"leaf_joint_policy={args.leaf_joint_policy}, "
        f"lateral_joint_policy={args.lateral_joint_policy}, "
        f"truss_calibration_preset={args.truss_calibration_preset}, "
        f"truss_armature_multiplier={args.truss_armature_multiplier}, "
        f"terminal_solver_preset={args.terminal_solver_preset}, "
        f"physical_petiolules={args.physical_petiolules}, "
        f"visual_quality={args.visual_quality}"
    )
    print(f"[OK] USDA: {usd_path}")
    print(f"[OK] Manifest: {manifest_path}")
    return 0
