"""
geometry - Geometry utilities for optimization

Provides utilities for:
- Attachment point remapping when collapsing segments
- Bounding volume calculation from branch configs
- Height preservation across topology changes
"""

from .remapping import (
    remap_attachment_height,
    RemappingResult,
    calculate_absolute_height,
    find_new_attachment,
)

from .bounds import (
    link_to_cylinder_geometry,
    branch_to_cylinder_geometries,
    get_link_dimensions,
)

__all__ = [
    # Remapping
    'remap_attachment_height',
    'RemappingResult',
    'calculate_absolute_height',
    'find_new_attachment',
    
    # Bounds
    'link_to_cylinder_geometry',
    'branch_to_cylinder_geometries',
    'get_link_dimensions',
]
