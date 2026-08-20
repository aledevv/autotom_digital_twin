"""Shared topology helpers for procedural tube meshes."""


def build_open_tube_topology(ring_count: int, radial_segments: int):
    """Return the triangular side faces for a tube without end caps."""
    face_counts = []
    face_indices = []
    for ring in range(ring_count - 1):
        row0 = ring * radial_segments
        row1 = (ring + 1) * radial_segments
        for radial in range(radial_segments):
            next_radial = (radial + 1) % radial_segments
            face_counts.extend((3, 3))
            face_indices.extend(
                (
                    row0 + radial,
                    row1 + radial,
                    row1 + next_radial,
                    row0 + radial,
                    row1 + next_radial,
                    row0 + next_radial,
                )
            )
    return face_counts, face_indices
