"""
threepoint_config.py

Shared configuration for the Three-Point Bending Test.
Importable without Isaac Sim dependencies.
"""

class TrunkConfig:
    N_LINKS  = 21   # Odd number -> exact center link exists
    HEIGHT   = 0.015 * 1.0   # 1.5 cm per link
    RADIUS   = 0.005 * 1.0   # 5 mm radius (diameter 10 mm)
    GAP      = 0.0001 * 1.0  # 0.1 mm gap between links

    @classmethod
    def total_span(cls) -> float:
        """Total beam span between the two support link origins [m]."""
        return (cls.N_LINKS - 1) * (cls.HEIGHT + cls.GAP)

    @classmethod
    def center_link_index(cls) -> int:
        """0-based index of the central link (exact center for odd N_LINKS)."""
        assert cls.N_LINKS % 2 == 1, "N_LINKS should be odd for an exact center link"
        return cls.N_LINKS // 2

class PhysicsConfig:
    BEND_LIMIT_DEG = 30.0   # max bending angle per joint [deg]

class BioConfig:
    # Elastic modulus for tomato-like stem (Solanaceae):
    #   Primary tissue (turgor-supported, young): 10–50 MPa  — Anisimov et al. 2025
    #   Mature tissue with sclerenchyma:         100–200 MPa — Shah et al. 2017
    # Default: 35 MPa — center of primary tissue range.
    YOUNG_MODULUS = 3.5e7   # [Pa] = 35 MPa
    DAMPING_RATIO = 0.2     # under-critical damping
    PLANT_DENSITY = 1000.0  # [kg/m³] — water density, plausible for turgid tissue
