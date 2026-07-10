"""
usd_exporterV2.py

STEP 2 della ricostruzione fisica incrementale del digital twin.

Estende lo stelo articolato v2 (STEP 1) per includere anche i RAMI: ogni
valore di InternodeNode.key.order diventa una catena fisica indipendente
(stelo principale = order 1, ogni ramo successivo = order > 1), ancorata
non al mondo ma al segmento fisico giusto del suo genitore botanico
(node.parent), individuato per quota Z reale.

Cosa fa e cosa NON fa (di proposito, per andare per gradi):
    - Raggruppa tutti gli InternodeNode per key.order. Ogni gruppo, ordinato
      per key.rank, e' una catena di segmenti rigidi + D6 joint elastici
      (stessa logica di STEP 1, ora fattorizzata in _build_chain()).
    - Il primo segmento della catena order=1 (stelo) e' ancorato al mondo
      con un FixedJoint, esattamente come in STEP 1.
    - Il primo segmento di ogni catena order>1 (ramo) e' ancorato con un
      bend joint (stesso tipo di quelli interni alla catena) al segmento
      fisico del genitore la cui estensione [base_z, base_z+height]
      contiene la quota mondo del punto di attacco botanico reale
      (world_base_z del primo internodo del ramo).
    - Applica un DOPPIO budget di segmenti per restare stabile in PhysX
      anche quando i rami crescono: un budget GLOBALE (MAX_TOTAL_SEGMENTS,
      su stelo+rami insieme) e un budget LOCALE per singola catena
      (MAX_SEGMENTS_PER_CHAIN), che vince quando piu' stringente.
    - NON crea foglie, frutti, radice, PhysicsScene o impostazioni PhysX
      (quelle vanno iniettate a runtime dentro Isaac Sim, vedi main_v2.py).
    - Le foglie sono volutamente lasciate per uno STEP successivo separato.

Uso tipico:
    from plant_model.usd_exporterV2 import build_stem_stage, export_stem_usd_v2

    stage, stem_path = build_stem_stage(snapshot, output_path)
    export_stem_usd_v2(snapshot, output_path)
"""

import math
from collections import defaultdict
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

from .models import PlantSnapshot, InternodeNode, LeafNode

from .constants import (
    STEM_DENSITY_KG_M3,
    SEGMENT_TARGET_LENGTH_M,
    MIN_SEGMENTS_PER_INTERNODE,
    SEGMENT_GAP_M,
    STEM_JOINT_STIFFNESS_BASE,
    STEM_JOINT_STIFFNESS_TIP,
    STEM_JOINT_DAMPING,
    STEM_JOINT_BEND_LIMIT_DEG,
    GLOBAL_SCALE,
    MAX_TOTAL_SEGMENTS,
    MAX_SEGMENTS_PER_CHAIN,
    BRANCH_ATTACH_STIFFNESS_FACTOR,
    BRANCH_ATTACH_BEND_LIMIT_DEG,
    LEAF_MASS_KG,
    LEAF_JOINT_STIFFNESS,
    LEAF_JOINT_DAMPING,
    LEAF_CONE_ANGLE_DEG,
)

from .usd_helpers import _make_material, _bind_material, _make_leaf

# ==============================================================================
# CONFIG DI PATH (nomi dei prim)
# ==============================================================================

PLANT_ROOT_PATH_TEMPLATE = "/Plant_{plant_id}_StemV2"

# Fattori derivati dalla scala globale:
# - massa scala col volume -> non serve un fattore esplicito qui, si de-scala
#   dividendo per GLOBAL_SCALE dentro _create_segment_link
# - stiffness/damping dei joint scalano -> GLOBAL_SCALE**5
_SCALE5 = GLOBAL_SCALE ** 5


# ==============================================================================
# HELPERS: world_base_z ricorsivo (topologia reale, non ordinamento sequenziale)
# ==============================================================================

def _compute_world_base_z(node: InternodeNode) -> float:
    """
    Calcola (e mette in cache su node.world_base_z) la quota mondo della BASE
    di questo internodo, camminando la vera catena node.parent — non
    assumendo che la lista ordinata per (order, rank) sia topologicamente
    corretta. Fondamentale ora che esistono piu' catene (order diversi).
    """
    if hasattr(node, 'world_base_z'):
        return node.world_base_z
    if node.parent is None or not isinstance(node.parent, InternodeNode):
        node.world_base_z = 0.0
    else:
        node.world_base_z = _compute_world_base_z(node.parent) + node.parent.length
    return node.world_base_z


# ==============================================================================
# HELPERS: densita' dei segmenti (budget adattivo, doppio livello)
# ==============================================================================

def _segments_for_internode(length_m: float, target_length_m: float) -> int:
    """
    Decide in quanti segmenti rigidi suddividere un internodo, in base alla
    lunghezza-target del segmento (adattiva, vedi _adaptive_target_length).
    """
    if length_m <= 0:
        return max(1, MIN_SEGMENTS_PER_INTERNODE)
    n = round(length_m / target_length_m)
    return max(MIN_SEGMENTS_PER_INTERNODE, n, 1)


def _adaptive_target_length(chain_length: float, global_target: float,
                             max_segments_per_chain: int) -> float:
    """
    Lunghezza-target del segmento per QUESTA catena: il massimo tra il
    target globale (derivato da MAX_TOTAL_SEGMENTS su tutta la pianta) e il
    target locale (derivato da MAX_SEGMENTS_PER_CHAIN su questa sola
    catena). Cosi' nessuna singola catena supera mai il suo tetto locale,
    anche se il budget globale lo permetterebbe.
    """
    if chain_length <= 0:
        return global_target
    local_target = chain_length / max_segments_per_chain
    return max(global_target, local_target)


# ==============================================================================
# HELPERS: geometria + rigid body per singolo segmento
# ==============================================================================

def _create_segment_link(stage: Usd.Stage, chain_path: str, seg_index: int,
                          radius: float, height: float, base_z: float, material) -> str:
    """
    Crea un singolo link (Xform + RigidBodyAPI + MassAPI) con dentro un
    Cylinder di collisione. radius/height sono GIA' scalati per
    GLOBAL_SCALE; la massa viene calcolata sulle dimensioni REALI
    (de-scalate) per mantenere un'inerzia fisicamente plausibile.
    """
    link_path = f"{chain_path}/Seg{seg_index:04d}"

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
    """Ancora il primissimo segmento dello stelo (order=1) al mondo con un FixedJoint."""
    joint_path = f"{link_path}/RootFixedJoint"
    joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    joint.CreateBody1Rel().SetTargets([Sdf.Path(link_path)])


# ==============================================================================
# HELPERS: joint D6 elastico (sia interno a una catena, sia attacco di un ramo)
# ==============================================================================

def _configure_bend_drive(joint: UsdPhysics.Joint, stiffness: float, damping: float,
                           bend_limit_deg: float) -> None:
    """
    Blocca tutte le traslazioni + il twist (rotZ), e applica una molla+damper
    sullo swing (rotX/rotY). low > high sul LimitAPI = asse completamente
    bloccato (stessa convenzione di generate_articulation_usda.py).
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

    limit_z = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), "rotZ")
    limit_z.CreateLowAttr().Set(1.0)
    limit_z.CreateHighAttr().Set(-1.0)


def _create_bend_joint(stage: Usd.Stage, parent_link: str, child_link: str, name: str,
                        parent_local_z: float, stiffness: float, damping: float,
                        bend_limit_deg: float,
                        tilt_deg: float = 0.0, azimuth_deg: float = 0.0) -> None:
    """
    Crea il D6 Joint tra un segmento genitore e un segmento figlio.

    parent_local_z: quota locale nel frame del genitore dove ancorare il
    pivot (per un joint INTERNO a una catena: altezza del genitore + gap;
    per un joint di ATTACCO RAMO: offset locale dal base del segmento
    genitore in cui cade il punto di attacco botanico).

    tilt_deg/azimuth_deg: se != 0, ruota il frame locale del genitore prima
    di agganciare il figlio (usato SOLO per l'attacco dei rami, che partono
    lateralmente e non in prosecuzione verticale — stessa tecnica di
    create_sub_branch() in generate_articulation_usda.py). Per i joint
    interni a una catena, restano 0 (nessuna rotazione, prosecuzione dritta).
    """
    joint_path = f"{child_link}/{name}"
    joint = UsdPhysics.Joint.Define(stage, joint_path)

    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_link)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_link)])

    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, parent_local_z))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

    if tilt_deg != 0.0 or azimuth_deg != 0.0:
        rot_z = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), azimuth_deg)
        rot_total = rot_z * Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), -tilt_deg)
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(rot_total.GetQuat()))
    else:
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    _configure_bend_drive(joint, stiffness, damping, bend_limit_deg)


# ==============================================================================
# HELPER: trova il segmento fisico del genitore a una data quota mondo
# ==============================================================================

def _find_parent_segment(parent_segments: list[dict], target_world_z: float) -> dict:
    """
    Trova il segmento del genitore la cui estensione [base_z, base_z+height]
    contiene target_world_z. Se nessuno lo contiene esattamente (bordi
    numerici / arrotondamenti), ritorna il piu' vicino per base_z.
    """
    for seg in parent_segments:
        if seg['base_z'] <= target_world_z <= seg['base_z'] + seg['height'] + 1e-9:
            return seg
    return min(parent_segments, key=lambda s: abs(s['base_z'] - target_world_z))


# ==============================================================================
# COSTRUZIONE DI UNA SINGOLA CATENA (stelo O un ramo)
# ==============================================================================

def _build_chain(stage: Usd.Stage, chain_path: str,
                  internodes_of_chain: list[InternodeNode],
                  seg_index_start: int,
                  attach_to: dict | None,
                  target_segment_length: float,
                  material) -> tuple[list[dict], int]:
    """
    Costruisce UNA catena di segmenti articolati (stelo principale o un
    singolo ramo), ordinata per key.rank.

    attach_to:
        None                        -> primo segmento ancorato al MONDO (FixedJoint)
        {'link_path', 'local_z',
         'tilt_deg', 'azimuth_deg',
         'stiffness', 'damping',
         'bend_limit_deg'}          -> primo segmento ancorato con bend joint
                                        al segmento del genitore specificato

    Ritorna (lista_segmenti, nuovo_seg_index). Ogni elemento della lista è
    {'path', 'base_z', 'height'} — world_base_z assoluto, utile sia per
    trovare punti di attacco di rami figli sia (in futuro) per le foglie.
    """
    segments: list[dict] = []
    seg_index = seg_index_start

    n_internodes = len(internodes_of_chain)
    previous_link_path = None
    previous_link_height = None
    current_base_z = internodes_of_chain[0].world_base_z if internodes_of_chain else 0.0

    for i, node in enumerate(internodes_of_chain):
        length = node.length * GLOBAL_SCALE
        radius = (node.width_m / 2.0) * GLOBAL_SCALE
        n_segments = _segments_for_internode(node.length, target_segment_length)

        n_gaps = max(n_segments - 1, 0)
        gap_scaled = SEGMENT_GAP_M * GLOBAL_SCALE
        seg_height = max((length - n_gaps * gap_scaled) / n_segments, 1e-5)

        t = i / max(n_internodes - 1, 1)
        stiffness = (STEM_JOINT_STIFFNESS_BASE + t * (STEM_JOINT_STIFFNESS_TIP - STEM_JOINT_STIFFNESS_BASE)) * _SCALE5
        damping = STEM_JOINT_DAMPING * _SCALE5

        for s in range(n_segments):
            link_path = _create_segment_link(
                stage, chain_path, seg_index, radius, seg_height, current_base_z, material
            )

            if previous_link_path is None:
                if attach_to is None:
                    _anchor_link_to_world(stage, link_path)
                else:
                    _create_bend_joint(
                        stage, attach_to['link_path'], link_path,
                        name=f"BranchAttach_{seg_index:04d}",
                        parent_local_z=attach_to['local_z'],
                        stiffness=attach_to['stiffness'],
                        damping=attach_to['damping'],
                        bend_limit_deg=attach_to['bend_limit_deg'],
                        tilt_deg=attach_to.get('tilt_deg', 0.0),
                        azimuth_deg=attach_to.get('azimuth_deg', 0.0),
                    )
            else:
                joint_name = f"Joint_{seg_index-1:04d}_{seg_index:04d}"
                _create_bend_joint(
                    stage, previous_link_path, link_path, joint_name,
                    parent_local_z=previous_link_height + gap_scaled,
                    stiffness=stiffness, damping=damping,
                    bend_limit_deg=STEM_JOINT_BEND_LIMIT_DEG,
                )

            segments.append({'path': link_path, 'base_z': current_base_z, 'height': seg_height})

            previous_link_path = link_path
            previous_link_height = seg_height
            current_base_z += seg_height + gap_scaled
            seg_index += 1

        print(f"    [order={node.key.order} rank={node.key.rank}] "
              f"L={length:.4f}m R={radius:.4f}m (x{GLOBAL_SCALE} scaled) -> {n_segments} segmenti")

    return segments, seg_index


def _create_leaf_joint(stage: Usd.Stage, joint_path: str,
                        parent_link: str, rametto_path: str,
                        parent_local_z: float, tip_world_z: float,
                        stiffness: float, damping: float, cone_angle_deg: float) -> None:
    """
    Joint sferico tra il segmento del ramo/stelo (parent_link) e il rametto
    della foglia (rametto_path). Stessa convenzione di Joint_Leaf_* in V1
    (usd_exporter.py): PhysicsSphericalJoint con drive force su rotX/Y/Z e
    cono di apertura limitato — NON un D6 bloccato come i joint interni
    alla catena, perche' la foglia deve poter oscillare liberamente entro
    un cono, non solo flettersi su un piano.
    """
    joint = UsdPhysics.SphericalJoint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_link)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(rametto_path)])

    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, parent_local_z))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, tip_world_z))

    joint.GetPrim().CreateAttribute("physics:coneAngle0Limit", Sdf.ValueTypeNames.Float).Set(cone_angle_deg)
    joint.GetPrim().CreateAttribute("physics:coneAngle1Limit", Sdf.ValueTypeNames.Float).Set(cone_angle_deg)

    for axis in ("rotX", "rotY", "rotZ"):
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
        drive.CreateTypeAttr().Set("force")
        drive.CreateStiffnessAttr().Set(stiffness)
        drive.CreateDampingAttr().Set(damping)
        drive.CreateTargetPositionAttr().Set(0.0)


def _attach_leaf(stage, leaves_path, joints_path, leaf_node, parent_segments, materials):
    if leaf_node.parent is None or not isinstance(leaf_node.parent, InternodeNode):
        return

    tip_world_z = leaf_node.parent.world_base_z + leaf_node.parent.length * GLOBAL_SCALE
    parent_seg = _find_parent_segment(parent_segments, tip_world_z)
    parent_local_z = tip_world_z - parent_seg['base_z']

    leaf_id = f"o{leaf_node.key.order}_r{leaf_node.key.rank}_i{leaf_node.key.organ_index}"
    leaf_group = f"{leaves_path}/Leaf_{leaf_id}"
    UsdGeom.Xform.Define(stage, leaf_group)

    rametto_path = f"{leaf_group}/Rametto"
    rametto_xform = UsdGeom.Xform.Define(stage, rametto_path)

    # Applica GLOBAL_SCALE a TUTTA la geometria della foglia (petiolo+rachide
    # +blade), in un solo colpo, senza modificare _make_leaf. tip_world_z e'
    # gia' in coordinate scalate (world), quindi va passato "scale-neutro"
    # (0,0,0) qui e traslato SEPARATAMENTE fuori dallo scale, altrimenti la
    # posizione del pivot verrebbe anch'essa moltiplicata per GLOBAL_SCALE
    # un'altra volta.
    rametto_xform.AddScaleOp().Set(Gf.Vec3f(GLOBAL_SCALE, GLOBAL_SCALE, GLOBAL_SCALE))
    rametto_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, tip_world_z))

    _make_leaf(stage, rametto_path, leaf_node, 0.0, materials)  # tip_z locale = 0, la traslazione mondo la da' il joint

    UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(rametto_path))
    mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(rametto_path))
    mass_api.CreateMassAttr().Set(LEAF_MASS_KG)

    filtered_pairs = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(rametto_path))
    filtered_pairs.GetFilteredPairsRel().AddTarget(Sdf.Path(parent_seg['path']))

    _create_leaf_joint(
        stage, joint_path=f"{joints_path}/Joint_Leaf_{leaf_id}",
        parent_link=parent_seg['path'], rametto_path=rametto_path,
        parent_local_z=parent_local_z, tip_world_z=0.0,  # il joint posiziona il rametto, non serve offset aggiuntivo
        stiffness=LEAF_JOINT_STIFFNESS, damping=LEAF_JOINT_DAMPING,
        cone_angle_deg=LEAF_CONE_ANGLE_DEG,
    )

    print(f"  [LEAF {leaf_id}] attaccata a {parent_seg['path']} (local_z={parent_local_z:.4f}m)")


# ==============================================================================
# COSTRUZIONE DELLO STAGE (stelo + rami, orchestrazione)
# ==============================================================================

def build_stem_stage(snapshot: PlantSnapshot, output_path: str) -> tuple:
    """
    Costruisce (ma NON salva) uno Stage USD contenente lo stelo principale
    E tutti i rami (ogni InternodeNode.key.order > 1 e' un ramo), come
    catene indipendenti di rigid-body segmentati con D6 joint elastici,
    ognuna ancorata al punto giusto del suo genitore botanico reale.

    Ritorna (stage, articulation_root_path) cosi' un loader dentro Isaac Sim
    puo' iniettare PhysicsScene + PhysxArticulationAPI SOLO sulla radice
    dello stelo principale: PhysX tratta l'intero albero (stelo+rami) come
    UNA SOLA articolazione, essendo tutti i joint concatenati in una
    componente rigida connessa.
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

    branches_path = f"{plant_path}/Branches"
    UsdGeom.Xform.Define(stage, branches_path)

    mats_path = f"{plant_path}/Materials"
    UsdGeom.Xform.Define(stage, mats_path)
    mat_stem = _make_material(stage, f"{mats_path}/Stem", (0.45, 0.30, 0.10))

    internodes = [n for n in snapshot.organs if isinstance(n, InternodeNode)]

    if not internodes:
        print("[WARN][usd_exporterV2] Nessun InternodeNode trovato nello snapshot: stage vuoto.")
        return stage, stem_path

    # world_base_z reale, ricorsivo sulla vera topologia (non sull'ordinamento)
    for n in internodes:
        _compute_world_base_z(n)

    # Raggruppa per order — ogni order e' una catena fisica indipendente
    chains_by_order: dict[int, list[InternodeNode]] = defaultdict(list)
    for n in internodes:
        chains_by_order[n.key.order].append(n)
    for order in chains_by_order:
        chains_by_order[order].sort(key=lambda n: n.key.rank)

    orders_sorted = sorted(chains_by_order.keys())

    total_wood_length = sum(n.length for n in internodes)
    global_target = max(total_wood_length / MAX_TOTAL_SEGMENTS, SEGMENT_TARGET_LENGTH_M)

    print(f"\n{'='*55}")
    print(f"  Plant {snapshot.plant_id} | Day {snapshot.day}")
    print(f"  Catene (order) trovate: {len(orders_sorted)} -> {orders_sorted}")
    print(f"  Lunghezza totale legno: {total_wood_length:.4f}m | target globale: {global_target:.4f}m")
    print(f"{'='*55}\n")

    seg_index = 0
    # Segmenti creati per ogni order, per poter trovare attach point dei rami figli
    chain_segments: dict[int, list[dict]] = {}

    for order in orders_sorted:
        chain_internodes = chains_by_order[order]
        chain_length = sum(n.length for n in chain_internodes)
        target_len = _adaptive_target_length(chain_length, global_target, MAX_SEGMENTS_PER_CHAIN)

        first_node = chain_internodes[0]
        attach_to = None

        if order != orders_sorted[0]:
            # Ramo: trova il segmento del genitore alla quota reale di attacco
            parent_node = first_node.parent
            if parent_node is not None and isinstance(parent_node, InternodeNode):
                parent_order = parent_node.key.order
                target_world_z = first_node.world_base_z
                parent_segments = chain_segments.get(parent_order, [])
                if parent_segments:
                    parent_seg = _find_parent_segment(parent_segments, target_world_z)
                    local_z = target_world_z - parent_seg['base_z']

                    t0 = 0.0  # stiffness al punto di attacco: usa la base della catena figlia
                    branch_stiffness = STEM_JOINT_STIFFNESS_BASE * BRANCH_ATTACH_STIFFNESS_FACTOR * _SCALE5
                    branch_damping = STEM_JOINT_DAMPING * _SCALE5

                    attach_to = {
                        'link_path': parent_seg['path'],
                        'local_z': local_z,
                        'stiffness': branch_stiffness,
                        'damping': branch_damping,
                        'bend_limit_deg': BRANCH_ATTACH_BEND_LIMIT_DEG,
                        'tilt_deg': 45.0,       # TODO: da dati reali se disponibili (es. angolo branching)
                        'azimuth_deg': 0.0,     # TODO: da dati reali (es. phyllotaxis del ramo)
                    }
                else:
                    print(f"[WARN][usd_exporterV2] Nessun segmento trovato per genitore order={parent_order}, "
                          f"ramo order={order} verra' ancorato al mondo per sicurezza.")

        chain_path = stem_path if order == orders_sorted[0] else f"{branches_path}/Branch_o{order}"
        if order != orders_sorted[0]:
            UsdGeom.Xform.Define(stage, chain_path)

        print(f"  [Catena order={order}] {len(chain_internodes)} internodi, "
              f"target segmento={target_len:.4f}m, attach={'mondo' if attach_to is None else attach_to['link_path']}")

        segments, seg_index = _build_chain(
            stage, chain_path, chain_internodes, seg_index, attach_to, target_len, mat_stem
        )
        chain_segments[order] = segments


    leaves_path = f"{plant_path}/Leaves"
    UsdGeom.Xform.Define(stage, leaves_path)
    joints_path = f"{plant_path}/Joints"
    UsdGeom.Xform.Define(stage, joints_path)

    mat_leaf = _make_material(stage, f"{mats_path}/Leaf", (0.15, 0.55, 0.10))
    mat_pedicel = _make_material(stage, f"{mats_path}/Pedicel", (0.20, 0.50, 0.10))
    materials = {"leaf": mat_leaf, "pedicel": mat_pedicel}

    leaves = [n for n in snapshot.organs if isinstance(n, LeafNode)]
    print(f"\n[INFO][usd_exporterV2] Foglie trovate: {len(leaves)}")

    for leaf_node in leaves:
        if leaf_node.parent is None or not isinstance(leaf_node.parent, InternodeNode):
            continue
        parent_order = leaf_node.parent.key.order
        parent_segments = chain_segments.get(parent_order)
        if not parent_segments:
            print(f"[WARN][usd_exporterV2] Nessun segmento trovato per genitore order={parent_order}, "
                  f"foglia {leaf_node.key} saltata.")
            continue
        _attach_leaf(stage, leaves_path, joints_path, leaf_node, parent_segments, materials)


    total_segments = seg_index
    print(f"\n[INFO][usd_exporterV2] GLOBAL_SCALE={GLOBAL_SCALE}x")
    print(f"[INFO][usd_exporterV2] Totale segmenti creati: {total_segments} (budget globale {MAX_TOTAL_SEGMENTS})")

    return stage, stem_path


def export_stem_usd_v2(snapshot: PlantSnapshot, output_path: str) -> None:
    """
    Wrapper di comodo: costruisce lo stage (stelo + rami) e lo salva su disco.
    NOTA: nessuna PhysicsScene / PhysxArticulationAPI qui — quelle vanno
    iniettate a runtime dentro Isaac Sim (vedi main_v2.py).
    """
    stage, _ = build_stem_stage(snapshot, output_path)
    stage.GetRootLayer().Save()
    print(f"[USD] Salvato (stelo + rami articolati v2) -> {output_path}")