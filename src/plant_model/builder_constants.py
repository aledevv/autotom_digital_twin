"""
builder_constants.py

Constants specific to the PlantBuilder / usd_exporter_builder (V2) pipeline.
Kept separate from plant_model/constants.py (shared with V1) to avoid
touching shared code.
"""

from .constants import PHYLLOTAXIS
BAKED_SCALE = 10.0
MAX_STEM_SEGMENTS = 25  # Budget for the main stem
PLANT_ROOT_PATH_TEMPLATE = "/Plant_{plant_id}_StemBuilder"

# ── Visual ──────────────────────────────────────────────────────────
PLANT_COLOR = (0.35, 0.62, 0.20)   # stem, branches, petioles, rachises
LEAF_COLOR = (0.12, 0.42, 0.08)    # leaf blades
FRUIT_RIPE = (0.90, 0.17, 0.10)    # fruit Ripe
FRUIT_YOUNG = (0.45, 0.58, 0.25)   # fruit Young

# ── Scale / segment budget ─────────────────────────────────────────
BAKED_SCALE = 10.0
MAX_STEM_SEGMENTS = 25
GAP = 0.001                        # tiny gap between stacked segments

# ── PlantBuilder safety limits ──────────────────────────────────────
MAX_ARTICULATION_DEPTH = 64        # hard PhysX limit
DEPTH_WARNING_THRESHOLD = 50
ASPECT_RATIO_WARNING = 25.0        # length/radius above this → jitter risk

# ── Stem joint defaults (add_internode auto-tune) ───────────────────
TRUNK_STIFFNESS = 500_000.0
TRUNK_DAMPING = 50.0
BRANCH_STIFFNESS = 300.0
BRANCH_DAMPING = 50.0

# ── Lateral branch joint defaults (add_lateral_branch auto-tune) ────
LATERAL_STIFFNESS = 184_000.0
LATERAL_DAMPING = 5_000.0

# ── Compound leaf joint defaults (add_compound_leaf) ─────────────────
LEAF_RACHIS_STIFFNESS_BASE = 0.05
LEAF_RACHIS_STIFFNESS_TIP = 0.005
LEAF_RACHIS_DAMPING_BASE = 0.005
LEAF_RACHIS_DAMPING_TIP = 0.0005
LEAF_RACHIS_DENSITY = 200.0
LEAF_RACHIS_MAX_BEND_DEG = 45.0

PETIOLULE_STIFFNESS_BASE = 0.001
PETIOLULE_STIFFNESS_TIP = 0.0005
PETIOLULE_DAMPING_BASE = 0.0005
PETIOLULE_DAMPING_TIP = 0.0001
PETIOLULE_DENSITY = 100.0
PETIOLULE_MAX_BEND_DEG = 60.0