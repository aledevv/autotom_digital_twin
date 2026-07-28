# Visual (placeholder colors, will change with real mesh skinning)
STEM_COLOR = (0.35, 0.62, 0.20)   # from original PLANT_COLOR
LEAF_COLOR = (0.12, 0.42, 0.08)    # leaf blades
FRUIT_RIPE = (0.90, 0.17, 0.10)    # fruit Ripe
FRUIT_YOUNG = (0.45, 0.58, 0.25)   # fruit Young

# ── Safety limits (kept from original — still relevant for future joints) ──
MAX_ARTICULATION_DEPTH = 64       # hard PhysX limit
ASPECT_RATIO_WARNING = 25.0       # length/radius above this → jitter risk

# ── Stem defaults ──
DEFAULT_STEM_MASS = 1.0

# Leaf defaults
PHYLLOTAXIS = 137.5       # Golden angle (degrees)