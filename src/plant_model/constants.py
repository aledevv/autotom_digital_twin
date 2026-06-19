# constants.py
# Model constants from organs-5.rgg and auxiliary_tools_and_charts.rgg
# These are fixed values shared across all organ instances.

# --- Truss geometry ---
PETIOLE_LENGTH_M: float = 0.003
INTERNODE_TRUSS_LENGTH_M: float = 0.012
INTERNODE_TRUSS_ANGLE_DEG: float = 9.0
INTERNODE_TRUSS_DIAMETER_M: float = 0.0015
ANGLE_AMONG_SUBSEQUENT_FRUITS_DEG: float = 0.0
FRUIT_PAIRING: bool = False

# --- Leaf geometry ---
BASE_SPACING: float = 1 / 30  # distance between points along leaflet midvein

# --- Biomass buffer fractions ---
STRUCTURAL_FRACTION: float = 0.90
BUFFER_FRACTION: float = 0.10

# --- Static simulation ---
STATIC_RGR: float = 0.04  # assumed relative growth rate in static mode (dd⁻¹)