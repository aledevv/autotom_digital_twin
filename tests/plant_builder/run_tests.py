"""
run_tests.py

PlantBuilder unit-test suite.
Bootstraps Isaac Sim in headless mode to get access to pxr/OpenUSD.
Run with:  cd ~/isaacsim && ./python.sh <project>/tests/plant_builder/run_tests.py
"""

import os
import sys
import traceback
import math

# --- bootstrap Isaac Sim (headless) so pxr becomes available -----------------
from isaacsim import SimulationApp
_sim_app = SimulationApp({"headless": True})

# --- bootstrap project import path -------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(os.path.dirname(SCRIPT_DIR))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from pxr import Usd, UsdGeom, Gf, UsdPhysics, Sdf
from src.plant_model.plant_builder import PlantBuilder, _quatd_to_quatf, GAP


# =============================================================================
#  Helpers
# =============================================================================
_counter = [0]

def _fresh_stage():
    """Create a new in-memory stage with /World and Z-up."""
    _counter[0] += 1
    # Use CreateInMemory so no files are written
    stage = Usd.Stage.CreateInMemory(f"test_{_counter[0]}.usda")
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    return stage


def _prim_exists(stage, path: str) -> bool:
    return bool(stage.GetPrimAtPath(path))


def _get_translate(stage, path: str) -> Gf.Vec3d:
    prim = stage.GetPrimAtPath(path)
    xformable = UsdGeom.Xformable(prim)
    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            return op.Get()
    return Gf.Vec3d(0, 0, 0)


def _get_orient(stage, path: str) -> Gf.Quatf:
    prim = stage.GetPrimAtPath(path)
    xformable = UsdGeom.Xformable(prim)
    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
            return op.Get()
    return Gf.Quatf(1, 0, 0, 0)


def _approx(a, b, tol=1e-4):
    """Compare two scalars or Vec3d values."""
    if isinstance(a, (Gf.Vec3d, Gf.Vec3f)):
        for i in range(3):
            if abs(a[i] - b[i]) > tol:
                return False
        return True
    return abs(a - b) < tol


# =============================================================================
#  Test registry
# =============================================================================
_tests = []

def test(fn):
    _tests.append(fn)
    return fn


# =============================================================================
#  TESTS — Structure
# =============================================================================

@test
def test_01_root_creation():
    """Root creates a prim at /World/Stem/<id> with RigidBody + FixedJoint."""
    stage = _fresh_stage()
    b = PlantBuilder(stage, "/World/Stem")
    b.create_root("Root", radius=0.1, length=0.5)

    assert _prim_exists(stage, "/World/Stem/Root"), "Root prim missing"
    assert _prim_exists(stage, "/World/Stem/Root/Cylinder"), "Cylinder missing"
    assert _prim_exists(stage, "/World/Stem/Root/FixedJoint"), "FixedJoint missing"

    # RigidBody API applied
    prim = stage.GetPrimAtPath("/World/Stem/Root")
    assert prim.HasAPI(UsdPhysics.RigidBodyAPI), "RigidBodyAPI not applied"

    # Position at origin
    pos = _get_translate(stage, "/World/Stem/Root")
    assert _approx(pos, Gf.Vec3d(0, 0, 0)), f"Root not at origin: {pos}"


@test
def test_02_single_internode():
    """One internode stacks exactly on top of the root."""
    stage = _fresh_stage()
    b = PlantBuilder(stage, "/World/Stem")
    b.create_root("R", radius=0.1, length=0.5)
    b.add_internode("R", "I1", radius=0.1, length=0.4)

    assert _prim_exists(stage, "/World/Stem/I1"), "Internode prim missing"
    assert _prim_exists(stage, "/World/Stem/I1/Joint"), "Joint missing"

    pos = _get_translate(stage, "/World/Stem/I1")
    expected_z = 0.5 + GAP
    assert _approx(pos[2], expected_z), \
        f"Internode Z={pos[2]}, expected {expected_z}"


@test
def test_03_internode_chain():
    """A chain of 5 internodes accumulates height correctly."""
    stage = _fresh_stage()
    b = PlantBuilder(stage, "/World/Stem")
    L = 0.3
    prev = b.create_root("S0", radius=0.1, length=L)
    expected_z = 0.0
    for i in range(1, 6):
        expected_z += L + GAP
        prev = b.add_internode(prev, f"S{i}", radius=0.1, length=L)
        pos = _get_translate(stage, f"/World/Stem/S{i}")
        assert _approx(pos[2], expected_z), \
            f"S{i}: Z={pos[2]:.6f}, expected {expected_z:.6f}"


@test
def test_04_lateral_branch_position():
    """Lateral branch base is offset from the trunk surface."""
    stage = _fresh_stage()
    b = PlantBuilder(stage, "/World/Stem")
    b.create_root("T", radius=0.1, length=1.0)
    b.add_lateral_branch("T", "B1", radius=0.04, length=0.3,
                         z_offset_ratio=0.5, tilt_angle=45,
                         rot_around_parent=0.0)

    pos = _get_translate(stage, "/World/Stem/B1")
    # At rot=0, the offset is along +Y by parent_radius
    assert abs(pos[1] - 0.1) < 0.01, \
        f"Branch Y={pos[1]}, expected ~0.1 (parent radius)"
    # At z_offset_ratio=0.5, z should be around 0.5
    assert abs(pos[2] - 0.5) < 0.1, \
        f"Branch Z={pos[2]}, expected ~0.5"


@test
def test_05_lateral_branch_orientation():
    """Lateral branch gets a non-identity quaternion (tilted)."""
    stage = _fresh_stage()
    b = PlantBuilder(stage, "/World/Stem")
    b.create_root("T", radius=0.1, length=1.0)
    b.add_lateral_branch("T", "B1", radius=0.04, length=0.3,
                         z_offset_ratio=0.5, tilt_angle=45,
                         rot_around_parent=90)

    quat = _get_orient(stage, "/World/Stem/B1")
    # Should NOT be identity — it's tilted 45°
    assert abs(quat.GetReal() - 1.0) > 0.01, \
        f"Branch orientation is identity — tilt not applied: {quat}"


@test
def test_06_subbranch_off_branch():
    """Replicate the generate_subbranch_articulation topology:
    Trunk → Branch (tilt=45, rot=90) → Subbranch (tilt=30, rot=90)."""
    stage = _fresh_stage()
    b = PlantBuilder(stage, "/World/Stem")
    # Trunk: single 2m segment
    t = b.create_root("Trunk", radius=0.10, length=2.0)
    # Branch: 1.5m at z_offset=1.0, tilt=45, rot=90
    br = b.add_lateral_branch(t, "Branch_01", radius=0.04, length=1.5,
                              z_offset_ratio=1.0, tilt_angle=45.0,
                              rot_around_parent=90.0)
    # Extend the branch with an internode
    br2 = b.add_internode(br, "Branch_01_ext", radius=0.04, length=0.5)
    # Subbranch off the first branch segment
    sb = b.add_lateral_branch(br, "Sub_01", radius=0.02, length=0.8,
                              z_offset_ratio=0.5, tilt_angle=30.0,
                              rot_around_parent=90.0)

    # All prims must exist
    for name in ("Trunk", "Branch_01", "Branch_01_ext", "Sub_01"):
        assert _prim_exists(stage, f"/World/Stem/{name}"), f"{name} missing"

    # Subbranch must have a different orientation than the branch
    q_br = _get_orient(stage, "/World/Stem/Branch_01")
    q_sb = _get_orient(stage, "/World/Stem/Sub_01")
    assert abs(q_br.GetReal() - q_sb.GetReal()) > 0.001 or \
           any(abs(q_br.GetImaginary()[i] - q_sb.GetImaginary()[i]) > 0.001
               for i in range(3)), \
        "Subbranch has same orientation as branch — rotation not compounded"


@test
def test_07_replicate_subbranch_articulation():
    """Full replication of generate_subbranch_articulation.py build_stage topology:
    1 trunk (2m) → Branch_01 (1.5m, tilt=45, rot=90) →
    Subbranch_01_01 (0.8m, tilt=30, rot=90) + Subbranch_01_02 (0.8m, tilt=30, rot=-90).

    The subbranch script creates multi-segment branches internally. Here we
    replicate the same logical structure with one segment per call."""
    stage = _fresh_stage()
    b = PlantBuilder(stage, "/World/Stem")

    trunk = b.create_root("Trunk", radius=0.10, length=2.0)
    branch = b.add_lateral_branch(trunk, "Branch_01", radius=0.04, length=1.5,
                                  z_offset_ratio=1.0, tilt_angle=45.0,
                                  rot_around_parent=90.0)
    sub1 = b.add_lateral_branch(branch, "Sub_01_01", radius=0.04, length=0.8,
                                z_offset_ratio=0.5, tilt_angle=30.0,
                                rot_around_parent=90.0)
    sub2 = b.add_lateral_branch(branch, "Sub_01_02", radius=0.04, length=0.8,
                                z_offset_ratio=0.5, tilt_angle=30.0,
                                rot_around_parent=-90.0)

    # All 4 prims exist
    for name in ("Trunk", "Branch_01", "Sub_01_01", "Sub_01_02"):
        assert _prim_exists(stage, f"/World/Stem/{name}"), f"{name} missing"

    # The two subbranches have same attachment point but different azimuth
    p1 = _get_translate(stage, "/World/Stem/Sub_01_01")
    p2 = _get_translate(stage, "/World/Stem/Sub_01_02")
    # They should NOT be at the exact same position (different rot_around_parent)
    dist = math.sqrt(sum((p1[i] - p2[i])**2 for i in range(3)))
    assert dist > 0.01, \
        f"Subbranches at same position — rot_around_parent not working (dist={dist})"


@test
def test_08_internode_continues_branch_direction():
    """An internode appended to a tilted branch continues in the branch direction."""
    stage = _fresh_stage()
    b = PlantBuilder(stage, "/World/Stem")
    b.create_root("T", radius=0.1, length=1.0)
    b.add_lateral_branch("T", "B", radius=0.04, length=0.3,
                         z_offset_ratio=1.0, tilt_angle=45,
                         rot_around_parent=0)
    b.add_internode("B", "B2", radius=0.04, length=0.3)

    pos_b = _get_translate(stage, "/World/Stem/B")
    pos_b2 = _get_translate(stage, "/World/Stem/B2")
    q_b = _get_orient(stage, "/World/Stem/B")
    q_b2 = _get_orient(stage, "/World/Stem/B2")

    # Orientations must match (same direction)
    assert _approx(q_b.GetReal(), q_b2.GetReal()), \
        f"Orientations differ: {q_b} vs {q_b2}"

    # B2 must be further from trunk than B (it extends the branch)
    dist_b = math.sqrt(pos_b[0]**2 + pos_b[1]**2 + pos_b[2]**2)
    dist_b2 = math.sqrt(pos_b2[0]**2 + pos_b2[1]**2 + pos_b2[2]**2)
    assert dist_b2 > dist_b, \
        f"B2 not further than B (dist B={dist_b:.4f}, B2={dist_b2:.4f})"


# =============================================================================
#  TESTS — Security checks
# =============================================================================

@test
def test_09_duplicate_id_raises():
    """Adding a segment with an existing ID must raise ValueError."""
    stage = _fresh_stage()
    b = PlantBuilder(stage, "/World/Stem")
    b.create_root("X", radius=0.1, length=0.5)
    try:
        b.add_internode("X", "X", radius=0.1, length=0.5)
        assert False, "Should have raised ValueError for duplicate ID"
    except ValueError as e:
        assert "already exists" in str(e), f"Wrong error message: {e}"


@test
def test_10_missing_parent_raises():
    """Referencing a non-existent parent must raise KeyError."""
    stage = _fresh_stage()
    b = PlantBuilder(stage, "/World/Stem")
    b.create_root("R", radius=0.1, length=0.5)
    try:
        b.add_internode("NONEXISTENT", "X", radius=0.1, length=0.5)
        assert False, "Should have raised KeyError for missing parent"
    except KeyError:
        pass  # expected


@test
def test_11_depth_limit_raises():
    """Exceeding 64 links in a chain must raise ValueError."""
    stage = _fresh_stage()
    b = PlantBuilder(stage, "/World/Stem")
    prev = b.create_root("D0", radius=0.1, length=0.05)
    for i in range(1, 64):
        prev = b.add_internode(prev, f"D{i}", radius=0.1, length=0.05)
    # Now at depth 63 — one more should exceed
    try:
        b.add_internode(prev, "D64", radius=0.1, length=0.05)
        assert False, "Should have raised ValueError at depth 64"
    except ValueError as e:
        assert "64" in str(e), f"Wrong error: {e}"


@test
def test_12_aspect_ratio_warning(capsys=None):
    """High aspect ratio (length/radius > 25) must print a warning."""
    stage = _fresh_stage()
    b = PlantBuilder(stage, "/World/Stem")
    # Redirect stdout to capture the warning
    import io
    old_stdout = sys.stdout
    sys.stdout = buf = io.StringIO()
    try:
        b.create_root("T", radius=0.01, length=0.5)  # aspect = 50 > 25
    finally:
        sys.stdout = old_stdout
    output = buf.getvalue()
    assert "aspect ratio" in output.lower(), \
        f"No aspect-ratio warning printed. Got:\n{output}"


@test
def test_13_quatd_to_quatf_identity():
    """Identity rotation converts to identity quaternion."""
    rot = Gf.Rotation(Gf.Vec3d(0, 0, 1), 0.0)
    qf = _quatd_to_quatf(rot.GetQuat())
    assert _approx(qf.GetReal(), 1.0), f"Real part wrong: {qf.GetReal()}"
    imag = qf.GetImaginary()
    for i in range(3):
        assert _approx(imag[i], 0.0), f"Imag[{i}] wrong: {imag[i]}"


@test
def test_14_quatd_to_quatf_45deg():
    """45° rotation around X-axis converts correctly."""
    rot = Gf.Rotation(Gf.Vec3d(1, 0, 0), 45.0)
    qf = _quatd_to_quatf(rot.GetQuat())
    # cos(22.5°) ≈ 0.9239, sin(22.5°) ≈ 0.3827
    assert _approx(qf.GetReal(), 0.9239, tol=0.001), \
        f"Real={qf.GetReal()}, expected ~0.9239"
    imag = qf.GetImaginary()
    assert _approx(imag[0], 0.3827, tol=0.001), \
        f"Imag[0]={imag[0]}, expected ~0.3827"


# =============================================================================
#  Runner
# =============================================================================
def main():
    print("=" * 60)
    print(f"  PlantBuilder Test Suite  —  {len(_tests)} tests")
    print("=" * 60)

    passed = 0
    failed = 0
    errors = []

    for fn in _tests:
        name = fn.__name__
        doc = (fn.__doc__ or "").strip().split("\n")[0]
        try:
            fn()
            passed += 1
            print(f"  ✅  {name}: {doc}")
        except Exception as e:
            failed += 1
            errors.append((name, e))
            print(f"  ❌  {name}: {doc}")
            traceback.print_exc()
            print()

    print("=" * 60)
    print(f"  Results: {passed} passed, {failed} failed, "
          f"{len(_tests)} total")
    print("=" * 60)

    if errors:
        print("\nFailed tests:")
        for name, e in errors:
            print(f"  • {name}: {e}")
        _sim_app.close()
        sys.exit(1)
    else:
        print("\n🎉 All tests passed!")
        _sim_app.close()
        sys.exit(0)


if __name__ == "__main__":
    main()
