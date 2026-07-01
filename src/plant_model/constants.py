# Model constants from organs-5.rgg and auxiliary_tools_and_charts.rgg
# These are fixed values shared across all organ instances.

# --- Truss geometry ---
PETIOLE_LENGTH_M: float = 0.003
INTERNODE_TRUSS_LENGTH_M: float = 0.012
INTERNODE_TRUSS_ANGLE_DEG: float = 9.0
INTERNODE_TRUSS_DIAMETER_M: float = 0.0015
ANGLE_AMONG_SUBSEQUENT_FRUITS_DEG: float = 0.0
FRUIT_PAIRING: bool = False
TRUSS_LENGTH        = 0.012   # m — main peduncle
TRUSS_RADIUS        = 0.00075  # m — peduncle cylinder radius
PEDICEL_LENGTH      = 0.008   # m — lateral pedicels (fruits 1+)
PEDICEL_RADIUS      = 0.0005  # m
PEDICEL_SPREAD_DEG  = 35.0    # degrees of opening of lateral pedicels

RACHIS_SEG   = INTERNODE_TRUSS_LENGTH_M   # 0.012 m — rachis segment length
PEDICEL_LEN  = PETIOLE_LENGTH_M           # 0.003 m — lateral pedicel length
PEDICEL_R    = TRUSS_RADIUS               # 0.00075 m
INITIAL_TILT = 45.0                        # GroIMP: RL(45) at start

# --- Leaf geometry ---
BASE_SPACING: float = 1 / 30  # distance between points along leaflet midvein

# --- Biomass buffer fractions ---
STRUCTURAL_FRACTION: float = 0.90
BUFFER_FRACTION: float = 0.10

# --- Static simulation ---
STATIC_RGR: float = 0.04  # assumed relative growth rate in static mode (dd⁻¹)

# --- Root and others ---
ROOT_SPHERE_RADIUS = 0.005  # m — visual marker, placed at z=-ROOT_SPHERE_RADIUS
PHYLLOTAXIS    = 137.5   # deg — azimuth of the truss w.r.t. the stem

# --- PHYSICS: Joint chain physics ---
JOINT_STIFFNESS_BASE: float  = 800.0   # N·m/rad — mature stem (low rank)
JOINT_STIFFNESS_TIP: float   = 200.0   # N·m/rad — young stem (high rank)
JOINT_DAMPING: float         = 50.0    # N·m·s/rad
JOINT_MAX_ANGLE_DEG: float   = 25.0    # Maximum range/fluctuation (symmetric)
STEM_DENSITY_KG_M3: float    = 900.0   # approximate density of plant tissue
FRUIT_DENSITY_KG_M3: float   = 1050.0  # tomato density (~water)

# --- PHYSICS TOGGLES ---
# Comment out any of these lines to disable that physics layer.
ENABLE_STEM_PHYSICS: bool    = True   # RigidBody + Collider + Joints on internodes
ENABLE_FRUIT_PHYSICS: bool   = True   # Collider on fruit spheres (for robot sensing)
ENABLE_LEAF_PHYSICS: bool    = True   # RigidBody + SphericalJoint on leaves (to make them oscillate)

# --- PHYSICS: Leaf springs ---
LEAF_MASS_KG: float          = 0.05   # kg
LEAF_JOINT_STIFFNESS: float  = 5.0    # N·m/rad
LEAF_JOINT_DAMPING: float    = 0.5    # N·m·s/rad