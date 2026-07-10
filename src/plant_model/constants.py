# Model constants from organs-5.rgg and auxiliary_tools_and_charts.rgg
# These are fixed values shared across all organ instances.

# --- SCALA GLOBALE (STEP 1, stelo V2) ---
# I dati reali della pianta producono cilindri troppo sottili per PhysX
# (raggi sub-millimetrici), che generano "Invalid PhysX transform" warnings.
# GLOBAL_SCALE ingrandisce uniformemente TUTTA la geometria dello stelo v2
# (lunghezze, raggi, gap) mantenendo le proporzioni reali della pianta,
# stessa tecnica di generate_articulation_usda.py (TrunkConfig/BranchConfig).
# La massa scala con GLOBAL_SCALE**3 (volume), la stiffness dei joint con
# GLOBAL_SCALE**5 (N*m/rad — vedi PhysicsConfig in generate_articulation_usda.py).
GLOBAL_SCALE: float = 1.0   # 10x -> mm diventano cm, piu' sicuro per PhysX

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
JOINT_STIFFNESS_BASE: float  = 80000.0   # N·m/rad — mature stem (low rank)
JOINT_STIFFNESS_TIP: float   = 2000.0   # N·m/rad — young stem (high rank)
JOINT_DAMPING: float         = 5.0    # N·m·s/rad
JOINT_MAX_ANGLE_DEG: float   = 25.0    # Maximum range/fluctuation (symmetric)
STEM_DENSITY_KG_M3: float    = 900.0   # approximate density of plant tissue
FRUIT_DENSITY_KG_M3: float   = 1050.0  # tomato density (~water)

# --- PHYSICS TOGGLES ---
# Comment out any of these lines to disable that physics layer.
ENABLE_STEM_PHYSICS: bool    = False   # RigidBody + Collider + Joints on internodes
ENABLE_FRUIT_PHYSICS: bool   = True   # Collider on fruit spheres (for robot sensing)
ENABLE_LEAF_PHYSICS: bool    = True   # RigidBody + SphericalJoint on leaves (to make them oscillate)

# --- PHYSICS: Leaf springs ---
LEAF_MASS_KG: float          = 0.05   # kg
LEAF_JOINT_STIFFNESS: float  = 5.0    # N·m/rad
LEAF_JOINT_DAMPING: float    = 0.5    # N·m·s/rad


# ============================================================================
# STEM ARTICULATION V2 — stelo principale a segmenti articolati (STEP 1)
# ============================================================================
# Usato da plant_model/usd_exporterV2.py + load_stem_v2.py.
# Qui lo stelo non e' piu' un cilindro rigido per internodo, ma una catena di
# segmenti rigidi piu' piccoli collegati da D6 joint elastici (stile
# generate_articulation_usda.py), con densita' di segmenti configurabile.

# Toggle usato da main.py per scegliere quale exporter chiamare, per restare
# modulari senza duplicare il loop principale.
USE_STEM_ARTICULATION_V2: bool = True

# Budget massimo di segmenti articolati per l'intero stelo, indipendente
# dal giorno/lunghezza totale. Invece di una densita' fissa (SEGMENT_TARGET_LENGTH_M)
# che fa crescere N_joints senza limite con la crescita della pianta, calcoliamo
# la lunghezza-target del segmento adattivamente cosi' il conteggio totale
# di joint resti sempre <= MAX_TOTAL_SEGMENTS (limite di stabilita' PhysX).
MAX_TOTAL_SEGMENTS: int = 50   # valore che hai verificato stabile al giorno 160

# Densita' dei segmenti: lunghezza-bersaglio di un singolo segmento rigido.
# Ogni internodo viene suddiviso in:
#   n_segments = max(MIN_SEGMENTS_PER_INTERNODE, round(internode_length / SEGMENT_TARGET_LENGTH_M))
# segmenti uguali, cosi' gli internodi corti restano comunque a
# MIN_SEGMENTS_PER_INTERNODE segmenti (default 1 = nessuna suddivisione).
SEGMENT_TARGET_LENGTH_M: float   = 0.01    # m — ~1 cm per segmento articolato
MIN_SEGMENTS_PER_INTERNODE: int  = 1
SEGMENT_GAP_M: float             = 0.0002  # m — piccolo gap visivo tra segmenti

# Parametri del drive D6 (molla+damper) sui joint tra segmenti dello stelo v2.
# Stessa convenzione di generate_articulation_usda.py: traslazioni e rotZ
# bloccati, drive elastico solo su rotX/rotY (bending).
STEM_JOINT_STIFFNESS_BASE: float = 80000.0   # N·m/rad — piu' rigido vicino alla base
STEM_JOINT_STIFFNESS_TIP: float  = 200.0   # N·m/rad — piu' flessibile vicino alla cima
STEM_JOINT_DAMPING: float        = 0.80    # N·m·s/rad
STEM_JOINT_BEND_LIMIT_DEG: float = 20.0    # limite di swing simmetrico su rotX/rotY

# ============================================================================
# STEM ARTICULATION V2 — budget adattivo (rami)
# ============================================================================

# Budget locale: numero massimo di segmenti per UNA SINGOLA catena
# (stelo principale OPPURE un singolo ramo, mai la somma di piu' catene).
# Protegge dal caso in cui un solo ramo sia molto piu' lungo degli altri:
# anche se il budget globale lo permetterebbe, nessuna catena singola
# supera mai questo tetto locale.
MAX_SEGMENTS_PER_CHAIN: int = 20

# Parametri del joint di attacco di un ramo al suo genitore (stelo o altro
# ramo). Stessa convenzione dimensionale di STEM_JOINT_* ma pensata per
# essere leggermente piu' permissiva del bend interno alla catena, perche'
# l'attacco di un ramo e' un punto di flessione naturale piu' pronunciato.
BRANCH_ATTACH_STIFFNESS_FACTOR: float = 1.0  # moltiplicatore su stiffness base della catena figlia
BRANCH_ATTACH_BEND_LIMIT_DEG: float = 35.0   # piu' permissivo di STEM_JOINT_BEND_LIMIT_DEG

# ============================================================================
# LEAVES V2 — rametto mobile (petiolo+rachide), lamina statica
# ============================================================================

LEAF_MASS_KG: float = 0.05          # stesso valore usato in V1 (usd_exporter.py)
LEAF_JOINT_STIFFNESS: float = 5.0   # stesso valore usato in V1
LEAF_JOINT_DAMPING: float = 0.5     # stesso valore usato in V1
LEAF_CONE_ANGLE_DEG: float = 45.0   # stesso valore usato in V1 (coneAngle0/1Limit)