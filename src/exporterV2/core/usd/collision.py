"""
collision.py - Collision Filtering

Manages collision filtering between rigid bodies to prevent spurious contacts.
"""

import re
from pxr import Usd, UsdPhysics, Sdf


def add_collision_filter(stage, body_a_path: str, body_b_path: str) -> None:
    """
    Add collision filtering between two bodies.
    
    Filtering is applied at RigidBody level and propagates to collision shapes.
    """
    prim_a = stage.GetPrimAtPath(body_a_path)
    prim_b = stage.GetPrimAtPath(body_b_path)
    
    if prim_a and prim_b:
        filtered_pairs = UsdPhysics.FilteredPairsAPI.Apply(prim_a)
        filtered_pairs.GetFilteredPairsRel().AddTarget(Sdf.Path(body_b_path))


def parse_link_number(link_path: str) -> tuple:
    """
    Parse link path to extract branch ID and link number.
    
    Args:
        link_path: Path like "/World/Stem/branch_Link_03"
    
    Returns:
        Tuple (branch_id, link_num) or (None, None) if parsing fails
    """
    match = re.match(r'.*/(\w+)_Link_(\d+)$', link_path)
    if match:
        return match.group(1), int(match.group(2))
    return None, None


def add_attachment_collision_filters(stage, child_link_path: str, parent_link_path: str) -> None:
    """
    Add collision filtering for branch attachment.
    
    When a branch attaches to a parent chain, filter collisions with:
    1. The parent link it attaches to
    2. The next link in the parent chain (prevents instability)
    
    Example:
        stem_Link_03 (parent attachment)
             ├─ branch_Link_01 (child)
        stem_Link_04 (next in chain - also filtered)
    """
    # Filter parent link
    add_collision_filter(stage, child_link_path, parent_link_path)
    
    # Find and filter next link in parent chain
    branch_id, link_num = parse_link_number(parent_link_path)
    if branch_id and link_num:
        stem_path = parent_link_path.rsplit('/', 1)[0]  # Get /World/Stem
        next_link_path = f"{stem_path}/{branch_id}_Link_{link_num + 1:02d}"
        next_link_prim = stage.GetPrimAtPath(next_link_path)
        
        if next_link_prim and next_link_prim.IsValid():
            add_collision_filter(stage, child_link_path, next_link_path)


def add_sibling_collision_filtering(stage, branches, branch_registry) -> None:
    """
    Add collision filtering between sibling branches.
    
    When multiple branches attach to the same parent link, filter collisions
    between them to prevent spurious contact forces.
    
    Example:
        main_petiole Link_02
             ├─ petiolule_1 ←┐
             ├─ petiolule_2 ←├─ These must filter each other
             └─ petiolule_3 ←┘
    """
    # Build map: (parent_id, attach_link_idx) → [child link paths]
    attachment_map = {}
    
    for b in branches:
        if b.get("parent") is None:
            continue  # Skip root
        
        parent_id = b["parent"]
        attach_idx = b["attach_link"] - 1  # Convert to 0-based
        key = (parent_id, attach_idx)
        
        # Get first link of this branch
        link_paths, _, _, _ = branch_registry[b["id"]]
        first_link = link_paths[0]
        
        attachment_map.setdefault(key, []).append(first_link)
    
    # Filter siblings pairwise (bidirectional)
    filtered_count = 0
    for sibling_links in attachment_map.values():
        if len(sibling_links) <= 1:
            continue
        
        for i, link_a in enumerate(sibling_links):
            for link_b in sibling_links[i+1:]:
                add_collision_filter(stage, link_a, link_b)
                add_collision_filter(stage, link_b, link_a)
                filtered_count += 2
    
    if filtered_count > 0:
        print(f"[INFO] Added {filtered_count} sibling collision filters")
