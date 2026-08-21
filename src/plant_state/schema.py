"""Constants for the canonical, exporter-independent plant-state schema."""

from __future__ import annotations


PLANT_STATE_SCHEMA_VERSION = "plant_state/1.0"

SUPPORTED_ORGAN_TYPES = frozenset(
    {"PlantBase", "Root", "Internode", "Leaf", "Truss", "Fruits", "Meristem"}
)
TURTLE_OPERATION_TYPES = frozenset({"RH", "RL", "RU", "RG", "Translate"})
STRUCTURAL_EDGE_KINDS = frozenset({"successor", "branch"})

DEFAULT_UNITS = {
    "angle": "degree",
    "area": "m2",
    "dry_biomass": "mg",
    "length": "m",
}

DEFAULT_CONVENTIONS = {
    "identity_scope": "stable_within_one_groimp_workbench",
    "matrix_composition": "world_at_local",
    "matrix_layout": "row_major_serialization",
    "rotation_columns": "local_x_left_local_y_up_local_z_head",
    "transform_semantics": "local_to_world_column_vectors",
}
