# Model constants from organs-5.rgg and auxiliary_tools_and_charts.rgg
# These are fixed values shared across all organ instances.

# --- GLOBAL SCALE (STEP 1, stem V2) ---
# Real plant data produces sub-millimeter cylinders which cause PhysX errors.
# GLOBAL_SCALE scales up the geometry to ensure stability (e.g., 10x means mm -> cm).
# Mass scales with GLOBAL_SCALE**3, joint stiffness with GLOBAL_SCALE**5.
GLOBAL_SCALE: float = 1.0

# --- Truss geometry ---
PETIOLE_LENGTH_M: float = 0.003
INTERNODE_TRUSS_LENGTH_M: float = 0.012
INTERNODE_TRUSS_ANGLE_DEG: float = 9.0
INTERNODE_TRUSS_DIAMETER_M: float = 0.0015
ANGLE_AMONG_SUBSEQUENT_FRUITS_DEG: float = 0.0
FRUIT_PAIRING: bool = False
TRUSS_LENGTH        = 0.012    # m — main peduncle
TRUSS_RADIUS        = 0.00075  # m — peduncle cylinder radius
PEDICEL_LENGTH      = 0.008    # m — lateral pedicels (fruits 1+)
PEDICEL_RADIUS      = 0.0005   # m
PEDICEL_SPREAD_DEG  = 35.0     # degrees of opening of lateral pedicels

RACHIS_SEG   = INTERNODE_TRUSS_LENGTH_M
PEDICEL_LEN  = PETIOLE_LENGTH_M
PEDICEL_R    = TRUSS_RADIUS
INITIAL_TILT = 45.0

# --- Leaf geometry ---
BASE_SPACING: float = 1 / 30  # distance between points along leaflet midvein

# --- Biomass buffer fractions ---
STRUCTURAL_FRACTION: float = 0.90
BUFFER_FRACTION: float = 0.10

# --- Static simulation ---
STATIC_RGR: float = 0.04  # assumed relative growth rate in static mode (dd⁻¹)

# --- Root and others ---
ROOT_SPHERE_RADIUS = 0.005  # m — visual marker
PHYLLOTAXIS    = 137.5      # deg — azimuth of the truss w.r.t. the stem

# --- PHYSICS: Joint chain physics ---
JOINT_STIFFNESS_BASE: float  = 80000.0  # N·m/rad — mature stem
JOINT_STIFFNESS_TIP: float   = 2000.0   # N·m/rad — young stem
JOINT_DAMPING: float         = 5.0      # N·m·s/rad
JOINT_MAX_ANGLE_DEG: float   = 25.0     # Maximum swing range
STEM_DENSITY_KG_M3: float    = 900.0    # approximate density
FRUIT_DENSITY_KG_M3: float   = 1050.0   # tomato density (~water)

# --- PHYSICS TOGGLES ---
ENABLE_STEM_PHYSICS: bool    = False  # RigidBody + Collider + Joints on internodes
ENABLE_FRUIT_PHYSICS: bool   = True   # Collider on fruit spheres
ENABLE_LEAF_PHYSICS: bool    = True   # RigidBody + SphericalJoint on leaves

# --- PHYSICS: Leaf springs ---
LEAF_MASS_KG: float          = 0.05   # kg
LEAF_JOINT_STIFFNESS: float  = 5.0    # N·m/rad
LEAF_JOINT_DAMPING: float    = 0.5    # N·m·s/rad
LEAF_CONE_ANGLE_DEG: float   = 45.0   # Cone limit for spherical joints


# ============================================================================
# STEM ARTICULATION V2
# ============================================================================
# Used by plant_model/usd_exporterV2.py and load_stem_v2.py.
# The stem is simulated as a chain of short rigid segments with elastic D6 joints.

USE_STEM_ARTICULATION_V2: bool = True

# Global segment budget for the entire plant to keep PhysX stable.
MAX_TOTAL_SEGMENTS: int = 50

# Segment density: target length of a single rigid segment.
# Internodes are subdivided based on this target length.
SEGMENT_TARGET_LENGTH_M: float   = 0.01    # m — ~1 cm per segment
MIN_SEGMENTS_PER_INTERNODE: int  = 1
SEGMENT_GAP_M: float             = 0.0002  # m — visual gap between segments

# D6 Drive parameters for joints between stem segments.
# Translations and rotZ are locked, elastic drive on rotX/rotY.
STEM_JOINT_STIFFNESS_BASE: float = 80000.0  # N·m/rad — stiffer near base
STEM_JOINT_STIFFNESS_TIP: float  = 200.0    # N·m/rad — flexible near tip
STEM_JOINT_DAMPING: float        = 0.80     # N·m·s/rad
STEM_JOINT_BEND_LIMIT_DEG: float = 20.0     # symmetric swing limit