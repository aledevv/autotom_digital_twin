"""
usd_exporterV2.py

STEP 1 della ricostruzione fisica incrementale del digital twin.

Esporta SOLO lo stelo principale (main stem) della pianta di pomodoro come
un'articolazione PhysX: una catena di link rigid-body "a segmenti" collegati
da D6 joint elastici (stile generate_articulation_usda.py), invece del singolo
cilindro rigido per internodo usato in usd_exporter.py.

Cosa fa e cosa NON fa (di proposito, per andare per gradi):
    - Prende gli internodi (InternodeNode) dallo snapshot, ordinati come in
      usd_exporter.py (key.order, key.rank), e li tratta come un'unica catena
      dalla base alla cima (in questo modello lo stelo principale e' l'unica
      catena di InternodeNode: rami/foglie/frutti sono organi laterali, non
      internodi aggiuntivi).
    - Ogni internodo viene suddiviso in N segmenti rigidi di lunghezza
      ~costante ("densita'" dei segmenti, vedi SEGMENT_TARGET_LENGTH_M in
      constants.py), MA il raggio del cilindro resta quello reale
      dell'internodo (node.width_m / 2). Niente cilindri innestati, niente
      geometrie complesse: stessa logica semplice di generate_articulation_usda.py.
    - Applica UsdPhysics.ArticulationRootAPI una sola volta, sulla radice
      dello stelo, e ancora il primissimo segmento al mondo con un FixedJoint
      (stessa convenzione del file di riferimento generate_articulation_usda.py).
    - NON crea rami, foglie, frutti, radice, materiali multipli, PhysicsScene
      o impostazioni PhysX (quelle vanno iniettate a runtime dentro Isaac Sim,
      vedi load_stem_v2.py, esattamente come fa load_articulation_subbranch.py
      per generate_articulation_usda.py).

Uso tipico:
    from plant_model.usd_exporterV2 import build_stem_stage, export_stem_usd_v2

    # Solo costruzione in memoria (utile se poi devi iniettare PhysX dentro IsaacSim):
    stage, stem_path = build_stem_stage(snapshot, output_path)

    # Costruzione + salvataggio su disco (per debug/ispezione offline, senza PhysX scene):
    export_stem_usd_v2(snapshot, output_path)
"""

import math
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

from .models import PlantSnapshot, InternodeNode

from .constants import (
    STEM_DENSITY_KG_M3,
    SEGMENT_TARGET_LENGTH_M,
    MIN_SEGMENTS_PER_INTERNODE,
    SEGMENT_GAP_M,
    STEM_JOINT_STIFFNESS_BASE,
    STEM_JOINT_STIFFNESS_TIP,
    STEM_JOINT_DAMPING,
    STEM_JOINT_BEND_LIMIT_DEG,
    GLOBAL_SCALE, MAX_TOTAL_SEGMENTS
)

from .usd_helpers import _make_material, _bind_material


# ==============================================================================
# CONFIG DI PATH (nomi dei prim)
# ==============================================================================

PLANT_ROOT_PATH_TEMPLATE = "/Plant_{plant_id}_StemV2"


# ==============================================================================
# HELPERS: densita' dei segmenti
# ==============================================================================

def _segments_for_internode(length_m: float, target_length_m: float) -> int:
    """
    Decide in quanti segmenti rigidi suddividere un internodo, in base alla
    "densita'" desiderata (SEGMENT_TARGET_LENGTH_M in constants.py).

    Esempio: internodo lungo 4 cm con target 1 cm -> 4 segmenti.
    Internodi molto corti restano comunque a MIN_SEGMENTS_PER_INTERNODE segmenti
    (di default 1, cioe' nessuna suddivisione).
    """
    if length_m <= 0:
        return max(1, MIN_SEGMENTS_PER_INTERNODE)
    n = round(length_m / target_length_m)
    return max(MIN_SEGMENTS_PER_INTERNODE, n, 1)


# Fattori derivati dalla scala globale:
# - massa scala col volume -> GLOBAL_SCALE**3
# - stiffness/damping dei joint scalano -> GLOBAL_SCALE**5
#   (stessa convenzione dimensionale di PhysicsConfig in generate_articulation_usda.py)
_SCALE = GLOBAL_SCALE
_SCALE5 = _SCALE ** 5


# ==============================================================================
# HELPERS: geometria + rigid body per singolo segmento
# ==============================================================================

def _create_segment_link(stage: Usd.Stage, stem_path: str, seg_index: int,
                          radius: float, height: float, base_z: float, material) -> str:
    """
    Crea un singolo link (Xform + RigidBodyAPI + MassAPI) con dentro un
    Cylinder di collisione, esattamente come create_rigid_body_link() in
    generate_articulation_usda.py, ma con raggio/altezza/massa presi dai
    dati reali della pianta invece che da TrunkConfig.

    NOTA: radius/height qui sono GIA' scalati per GLOBAL_SCALE (vedi
    build_stem_stage). La massa viene calcolata sulle dimensioni REALI
    (non scalate) per mantenere un'inerzia fisicamente plausibile.
    """
    link_path = f"{stem_path}/Seg{seg_index:03d}"

    xform_prim = UsdGeom.Xform.Define(stage, link_path)
    xform_prim.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, base_z))

    UsdPhysics.RigidBodyAPI.Apply(xform_prim.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(xform_prim.GetPrim())
    real_radius = radius / GLOBAL_SCALE
    real_height = height / GLOBAL_SCALE
    mass_kg = math.pi * (real_radius ** 2) * real_height * STEM_DENSITY_KG_M3
    mass_api.CreateMassAttr().Set(mass_kg)

    cylinder_path = f"{link_path}/Cylinder"
    cylinder = UsdGeom.Cylinder.Define(stage, cylinder_path)
    cylinder.GetRadiusAttr().Set(radius)
    cylinder.GetHeightAttr().Set(height)
    cylinder.GetAxisAttr().Set(UsdGeom.Tokens.z)
    cylinder.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, height / 2.0))
    UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())

    if material is not None:
        _bind_material(cylinder, material)

    return link_path


def _anchor_link_to_world(stage: Usd.Stage, link_path: str) -> None:
    """Ancora il primissimo segmento al mondo con un FixedJoint (base dello stelo)."""
    joint_path = f"{link_path}/RootFixedJoint"
    joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    joint.CreateBody1Rel().SetTargets([Sdf.Path(link_path)])


# ==============================================================================
# HELPERS: joint D6 elastico tra due segmenti consecutivi
# ==============================================================================

def _configure_bend_drive(joint: UsdPhysics.Joint, stiffness: float, damping: float,
                           bend_limit_deg: float) -> None:
    """
    Blocca tutte le traslazioni + il twist (rotZ), e applica una molla+damper
    sullo swing (rotX/rotY). Stessa identica convenzione di
    configure_joint_drives(..., lock_z=True) in generate_articulation_usda.py:
    low > high sul LimitAPI = asse completamente bloccato.
    """
    for axis in ("transX", "transY", "transZ"):
        limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        limit.CreateLowAttr().Set(1.0)
        limit.CreateHighAttr().Set(-1.0)

    for axis in ("rotX", "rotY"):
        limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        limit.CreateLowAttr().Set(-bend_limit_deg)
        limit.CreateHighAttr().Set(bend_limit_deg)

        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
        drive.CreateTypeAttr().Set("force")
        drive.CreateStiffnessAttr().Set(stiffness)
        drive.CreateDampingAttr().Set(damping)
        drive.CreateTargetPositionAttr().Set(0.0)

    # Lo stelo principale non ha bisogno di torsione: rotZ sempre bloccato.
    limit_z = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), "rotZ")
    limit_z.CreateLowAttr().Set(1.0)
    limit_z.CreateHighAttr().Set(-1.0)


def _create_bend_joint(stage: Usd.Stage, parent_link: str, child_link: str, name: str,
                        parent_height: float, stiffness: float, damping: float,
                        bend_limit_deg: float) -> None:
    """
    Crea il D6 Joint tra un segmento e il successivo. L'ancoraggio e' in cima
    al parent (parent_height + gap) e alla base del child, stessa convenzione
    di create_d6_bending_joint() in generate_articulation_usda.py.
    """
    joint_path = f"{child_link}/{name}"
    joint = UsdPhysics.Joint.Define(stage, joint_path)

    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_link)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_link)])

    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, parent_height + SEGMENT_GAP_M))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    _configure_bend_drive(joint, stiffness, damping, bend_limit_deg)


# ==============================================================================
# COSTRUZIONE DELLO STAGE (stelo articolato a segmenti)
# ==============================================================================

def build_stem_stage(snapshot: PlantSnapshot, output_path: str) -> tuple:
    """
    Costruisce (ma NON salva) uno Stage USD contenente SOLO lo stelo
    principale della pianta, come catena di rigid-body segmentati con D6
    joint elastici. Nessun ramo, foglia, frutto o radice.

    Ritorna (stage, articulation_root_path) cosi' un loader dentro Isaac Sim
    puo' iniettare PhysicsScene + PhysxArticulationAPI, esattamente come fa
    load_articulation_subbranch.py con build_stage() di generate_articulation_usda.py.
    """
    stage = Usd.Stage.CreateNew(output_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    plant_path = PLANT_ROOT_PATH_TEMPLATE.format(plant_id=snapshot.plant_id)
    plant_prim = UsdGeom.Xform.Define(stage, plant_path).GetPrim()
    stage.SetDefaultPrim(plant_prim)

    stem_path = f"{plant_path}/Stem"
    stem_prim = UsdGeom.Xform.Define(stage, stem_path)
    UsdPhysics.ArticulationRootAPI.Apply(stem_prim.GetPrim())

    # Un solo materiale semplice per lo stelo (stesso colore di usd_exporter.py)
    mats_path = f"{plant_path}/Materials"
    UsdGeom.Xform.Define(stage, mats_path)
    mat_stem = _make_material(stage, f"{mats_path}/Stem", (0.45, 0.30, 0.10))

    # Stessa selezione/ordinamento di usd_exporter.py: tutti gli InternodeNode,
    # ordinati per (order, rank). In questo modello formano un'unica catena
    # dalla base alla cima dello stelo principale.
    internodes = sorted(
        [n for n in snapshot.organs if isinstance(n, InternodeNode)],
        key=lambda n: (n.key.order, n.key.rank)
    )

    if not internodes:
        print("[WARN][usd_exporterV2] Nessun InternodeNode trovato nello snapshot: stage vuoto.")
        return stage, stem_path

    n_internodes = len(internodes)
    seg_index = 0
    previous_link_path = None
    previous_link_height = None
    current_base_z = 0.0
    total_segments = 0

    total_stem_length = sum(n.length for n in internodes)
    adaptive_target_length = max(
        total_stem_length / MAX_TOTAL_SEGMENTS,
        SEGMENT_TARGET_LENGTH_M  # non scendere sotto la densita' minima "fine" per piante corte
    )

    print(f"[INFO][usd_exporterV2] Lunghezza totale stelo: {total_stem_length:.4f}m")
    print(f"[INFO][usd_exporterV2] Target segmento adattivo: {adaptive_target_length:.4f}m (budget max {MAX_TOTAL_SEGMENTS} joint)")

    print(f"[INFO][usd_exporterV2] Costruzione stelo articolato: {n_internodes} internodi...")

    for i, node in enumerate(internodes):
        length = node.length * GLOBAL_SCALE
        radius = (node.width_m / 2.0) * GLOBAL_SCALE
        n_segments = _segments_for_internode(node.length, adaptive_target_length)  # subdivisione sulla lunghezza REALE

        # Sottrai i gap interni al segmento (gap anch'esso scalato)
        n_gaps = max(n_segments - 1, 0)
        gap_scaled = SEGMENT_GAP_M * GLOBAL_SCALE
        seg_height = max((length - n_gaps * gap_scaled) / n_segments, 1e-5)

        # Rigidezza del joint decresce dalla base alla cima, poi scalata per GLOBAL_SCALE**5
        t = i / max(n_internodes - 1, 1)
        stiffness = (STEM_JOINT_STIFFNESS_BASE + t * (STEM_JOINT_STIFFNESS_TIP - STEM_JOINT_STIFFNESS_BASE)) * _SCALE5
        damping = STEM_JOINT_DAMPING * _SCALE5*3.0

        for s in range(n_segments):
            link_path = _create_segment_link(
                stage, stem_path, seg_index, radius, seg_height, current_base_z, mat_stem
            )

            if previous_link_path is None:
                _anchor_link_to_world(stage, link_path)
            else:
                joint_name = f"Joint_{seg_index-1:03d}_{seg_index:03d}"
                _create_bend_joint(
                    stage, previous_link_path, link_path, joint_name,
                    parent_height=previous_link_height,
                    stiffness=stiffness, damping=damping,
                    bend_limit_deg=STEM_JOINT_BEND_LIMIT_DEG,
                )

            previous_link_path = link_path
            previous_link_height = seg_height
            current_base_z += seg_height + gap_scaled
            seg_index += 1
            total_segments += 1

        print(f"  [internode order={node.key.order} rank={node.key.rank}] "
              f"L={length:.4f}m R={radius:.4f}m (x{GLOBAL_SCALE} scaled) -> {n_segments} segmenti (seg_h~{seg_height:.4f}m)")

    print(f"[INFO][usd_exporterV2] GLOBAL_SCALE={GLOBAL_SCALE}x — geometria/joint scalati per stabilita' PhysX")
    print(f"[INFO][usd_exporterV2] Costruzione stelo articolato: {n_internodes} internodi...")

    return stage, stem_path


def export_stem_usd_v2(snapshot: PlantSnapshot, output_path: str) -> None:
    """
    Wrapper di comodo: costruisce lo stage e lo salva su disco.
    NOTA: qui non viene creata alcuna PhysicsScene / PhysxArticulationAPI —
    quelle vanno iniettate a runtime dentro Isaac Sim (vedi load_stem_v2.py),
    esattamente come fa load_articulation_subbranch.py con generate_articulation_usda.py.
    Utile per ispezionare/validare la geometria offline (es. usdview).
    """
    stage, _ = build_stem_stage(snapshot, output_path)
    stage.GetRootLayer().Save()
    print(f"[USD] Salvato (stelo articolato v2) -> {output_path}")
