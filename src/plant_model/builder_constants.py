"""
builder_constants.py

Constants specific to the PlantBuilder / usd_exporter_builder (V2) pipeline.
Kept separate from plant_model/constants.py (shared with V1) to avoid
touching shared code.
"""

BAKED_SCALE = 10.0
MAX_STEM_SEGMENTS = 25  # Budget for the main stem
PLANT_ROOT_PATH_TEMPLATE = "/Plant_{plant_id}_StemBuilder"

PHYLLOTAXIS    = 137.5      # deg — azimuth of the truss w.r.t. the stem

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

# --- Stable mode / branch tuning ---
LATERAL_BRANCH_STIFFNESS_BASE = 50000.0
LATERAL_BRANCH_STIFFNESS_TIP = 10000.0
LATERAL_BRANCH_DAMPING_BASE = 8000.0
LATERAL_BRANCH_DAMPING_TIP = 2000.0
LATERAL_BRANCH_MAX_BEND_ANGLE = 10.0
LATERAL_BRANCH_DENSITY = 700.0
LATERAL_BRANCH_TILT_ANGLE = 45.0

# --- Compound leaf main rachis tuning ---
COMPOUND_LEAF_STIFFNESS_BASE = 5000.0
COMPOUND_LEAF_STIFFNESS_TIP = 1000.0
COMPOUND_LEAF_DAMPING_BASE = 1000.0
COMPOUND_LEAF_DAMPING_TIP = 600.0
COMPOUND_LEAF_MAX_BEND_ANGLE = 10.0
COMPOUND_LEAF_DENSITY = 200.0

# --- Compound leaf lateral petiolules tuning ---
LEAFLET_PETIOLULE_STIFFNESS_BASE = 5000.0
LEAFLET_PETIOLULE_STIFFNESS_TIP = 1000.0
LEAFLET_PETIOLULE_DAMPING_BASE = 800.0
LEAFLET_PETIOLULE_DAMPING_TIP = 200.0
LEAFLET_PETIOLULE_MAX_BEND_ANGLE = 10.0
LEAFLET_PETIOLULE_DENSITY = 200.0
LEAFLET_PETIOLULE_SEGMENTS = 2
