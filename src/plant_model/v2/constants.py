# Visual (placeholder colors, will change with real mesh skinning)
STEM_COLOR = (0.35, 0.62, 0.20)   # from original PLANT_COLOR

# ── Safety limits (kept from original — still relevant for future joints) ──
MAX_ARTICULATION_DEPTH = 64       # hard PhysX limit
ASPECT_RATIO_WARNING = 25.0       # length/radius above this → jitter risk

# ── Stem defaults ──
DEFAULT_STEM_MASS = 1.0