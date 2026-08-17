"""
run_bridge.py — Test 0E: runtime bridge PhysX -> UsdSkel

Pipeline ad ogni physics step:

    PhysX rigid links
          |
          v
    world transforms
          |
          v
    world_to_joint_local()
          |
          v
    decompose T + R
          |
          v
    SkelAnimation translations/rotations
          |
          v
    Skeleton -> TubeMesh

Success criteria:
  [ ] la TubeMesh parte dritta
  [ ] cade insieme alla catena PhysX
  [ ] nessun salto/offset evidente
  [ ] nessun freeze
  [ ] nessun twist inatteso

Uso:
    ~/isaacsim/python.sh run_bridge.py

NOTA:
    Non serve premere Play nella UI.
    Questo script avanza esplicitamente World.step().
"""

import os
import sys
import time

from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "width": 1280,
    "height": 720,
})

import omni.usd
from isaacsim.core.api import World
from pxr import Gf, Usd, UsdGeom, UsdSkel, Vt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import generate_bridge


NUM_LINKS = generate_bridge.NUM_LINKS
OUTPUT_USD = generate_bridge.OUTPUT_USD

LINK_PATHS = [
    f"/World/Stem/Branch_Link_{i + 1:02d}"
    for i in range(NUM_LINKS)
]

SKEL_ROOT_PATH = "/World/StemVisual/SkelRoot"
ANIM_PATH = "/World/StemVisual/SkelRoot/SkelAnim"


def get_world_mats(cache, prims):
    """
    Legge la posa corrente dei rigid links dallo stage USD.
    Isaac/PhysX aggiorna le pose USD durante la simulazione.
    """
    cache.Clear()
    return [
        Gf.Matrix4d(
            cache.GetLocalToWorldTransform(prim)
        )
        for prim in prims
    ]


def world_to_joint_local(
    link_world_mats,
    skel_root_world,
):
    """
    Converte le pose concatenate/world dei rigid links nella gerarchia locale
    richiesta da SkelAnimation.

    IMPORTANTE — convenzione Gf/OpenUSD:
        local = child_world * inverse(parent_world)

    Per la root:
        local = root_world * inverse(skel_root_world)

    Poiché ogni rigid link origin coincide con la corrispondente bone origin,
    NON esiste un offset link->bone.
    """
    result = []

    root_inv = skel_root_world.GetInverse()

    for i, world in enumerate(link_world_mats):
        if i == 0:
            local = world * root_inv
        else:
            parent_inv = link_world_mats[i - 1].GetInverse()
            local = world * parent_inv

        result.append(local)

    return result


def decompose_joint_mats(local_mats):
    translations = []
    rotations = []

    for m in local_mats:
        t = m.ExtractTranslation()
        q = m.ExtractRotationQuat()
        qi = q.GetImaginary()

        translations.append(
            Gf.Vec3f(
                float(t[0]),
                float(t[1]),
                float(t[2]),
            )
        )
        rotations.append(
            Gf.Quatf(
                float(q.GetReal()),
                Gf.Vec3f(
                    float(qi[0]),
                    float(qi[1]),
                    float(qi[2]),
                ),
            )
        )

    return (
        Vt.Vec3fArray(translations),
        Vt.QuatfArray(rotations),
    )


def translation_error(a, b):
    ta = a.ExtractTranslation()
    tb = b.ExtractTranslation()
    d = ta - tb
    return float(d.GetLength())


def main():
    print()
    print("=" * 72)
    print("TEST 0E — PhysX -> UsdSkel runtime bridge")
    print("=" * 72)

    # ------------------------------------------------------------------
    # 1. Build + open USD
    # ------------------------------------------------------------------
    generate_bridge.build_stage(OUTPUT_USD)

    ctx = omni.usd.get_context()
    ctx.open_stage(OUTPUT_USD)

    if World.instance() is not None:
        World.instance().clear_instance()

    world = World(
        stage_units_in_meters=1.0,
        physics_prim_path="/World/PhysicsScene",
    )

    stage = ctx.get_stage()

    # ------------------------------------------------------------------
    # 2. Handles
    # ------------------------------------------------------------------
    link_prims = [
        stage.GetPrimAtPath(path)
        for path in LINK_PATHS
    ]

    for path, prim in zip(LINK_PATHS, link_prims):
        if not prim.IsValid():
            raise RuntimeError(
                f"Rigid link non trovato: {path}"
            )

    skel_root_prim = stage.GetPrimAtPath(
        SKEL_ROOT_PATH
    )
    if not skel_root_prim.IsValid():
        raise RuntimeError(
            f"SkelRoot non trovato: {SKEL_ROOT_PATH}"
        )

    anim_prim = stage.GetPrimAtPath(ANIM_PATH)
    if not anim_prim.IsValid():
        raise RuntimeError(
            f"SkelAnimation non trovata: {ANIM_PATH}"
        )

    anim = UsdSkel.Animation(anim_prim)
    translations_attr = anim.GetTranslationsAttr()
    rotations_attr = anim.GetRotationsAttr()

    cache = UsdGeom.XformCache(
        Usd.TimeCode.Default()
    )

    # ------------------------------------------------------------------
    # 3. Frame-0 sanity check PRIMA della simulazione
    # ------------------------------------------------------------------
    link_world_before = get_world_mats(
        cache,
        link_prims,
    )

    cache.Clear()
    skel_root_world = Gf.Matrix4d(
        cache.GetLocalToWorldTransform(
            skel_root_prim
        )
    )

    computed_rest = world_to_joint_local(
        link_world_before,
        skel_root_world,
    )
    expected_rest = generate_bridge.rest_local_transforms()

    print()
    print("[FRAME-0 CHECK]")
    max_t_err = 0.0

    for i, (actual, expected) in enumerate(
        zip(computed_rest, expected_rest)
    ):
        e = translation_error(actual, expected)
        max_t_err = max(max_t_err, e)
        print(
            f"  Bone{i}: translation error = "
            f"{e * 1000.0:.6f} mm"
        )

    print(
        f"  max translation error = "
        f"{max_t_err * 1000.0:.6f} mm"
    )

    if max_t_err > 1e-5:
        print(
            "  [WARNING] pose PhysX e rest pose Skeleton "
            "non coincidono perfettamente."
        )
    else:
        print("  [OK] origins PhysX/bones allineate.")

    # ------------------------------------------------------------------
    # 4. Start physics
    # ------------------------------------------------------------------
    world.reset()

    # Sincronizza immediatamente la skin alla posa corrente di PhysX.
    link_world = get_world_mats(
        cache,
        link_prims,
    )
    local_mats = world_to_joint_local(
        link_world,
        skel_root_world,
    )
    translations, rotations = decompose_joint_mats(
        local_mats
    )
    translations_attr.Set(translations)
    rotations_attr.Set(rotations)

    print()
    print("[RUN]")
    print("  Non premere Play nella UI.")
    print("  Lo script sta già avanzando la simulazione.")
    print()
    print("  Expected:")
    print("    tubo inizialmente dritto lungo +Y")
    print("    -> bending verso -Z")
    print("    -> skin sempre sovrapposta alla fisica")
    print()
    print("  Ctrl+C oppure chiudi Isaac Sim per terminare.")
    print("=" * 72)

    step = 0
    start = time.time()
    last_log = 0

    # ------------------------------------------------------------------
    # 5. Runtime bridge
    # ------------------------------------------------------------------
    while simulation_app.is_running():
        # Avanza SOLO physics.
        # Renderizziamo dopo avere aggiornato lo skeleton, evitando il
        # one-frame lag del pattern step(render=True) -> update skeleton.
        world.step(render=False)

        # PhysX -> world transforms
        link_world = get_world_mats(
            cache,
            link_prims,
        )

        # world/concatenated -> joint-local
        local_mats = world_to_joint_local(
            link_world,
            skel_root_world,
        )

        # Matrices -> T/R arrays richiesti da SkelAnimation
        translations, rotations = decompose_joint_mats(
            local_mats
        )

        # Batch update
        translations_attr.Set(translations)
        rotations_attr.Set(rotations)

        # Ora renderizza la frame con la skin già sincronizzata.
        simulation_app.update()

        step += 1

        if step - last_log >= 240:
            tip = link_world[-1].ExtractTranslation()
            child_local = local_mats[1]
            q = child_local.ExtractRotationQuat()
            qi = q.GetImaginary()

            print(
                f"[frame {step:6d}] "
                f"tip=({tip[0]:+.3f},"
                f"{tip[1]:+.3f},"
                f"{tip[2]:+.3f})  "
                f"Bone1.local.q="
                f"({q.GetReal():+.3f},"
                f"{qi[0]:+.3f},"
                f"{qi[1]:+.3f},"
                f"{qi[2]:+.3f})"
            )
            last_log = step

    elapsed = time.time() - start

    print()
    print("=" * 72)
    print(
        f"Terminato: {step} frames in "
        f"{elapsed:.1f}s"
    )
    print()
    print("GO se:")
    print("  [ ] TubeMesh segue il bending")
    print("  [ ] nessun offset iniziale")
    print("  [ ] nessun freeze")
    print("  [ ] nessun twist inatteso")
    print("=" * 72)

    simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        if simulation_app.is_running():
            simulation_app.close()
