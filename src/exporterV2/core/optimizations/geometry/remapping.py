"""
remapping.py - Attachment Point Remapping

When collapsing stem segments (e.g., 5 links → 3 links), child branches
attached to the stem need their attachment points recalculated to preserve
their absolute height in the world.

Example:
    Original: 5 links @ 0.2m each, branch attached at link 3, offset 0.1m
              → absolute height = 3*0.2 + 0.1 = 0.7m
    
    After collapse to 3 links @ 0.33m each:
              → new attachment: link 2, offset 0.04m
              → absolute height = 2*0.33 + 0.04 = 0.7m ✓

Usage:
    result = remap_attachment_height(
        original_link_idx=3,
        original_offset=0.1,
        original_segment_heights=[0.2] * 5,
        new_segment_heights=[0.33] * 3
    )
    print(f"New attachment: link {result.new_link_idx}, offset {result.new_offset}")
"""

from dataclasses import dataclass
from typing import List, Optional


@dataclass
class RemappingResult:
    """Result of attachment remapping."""
    new_link_idx: int
    new_offset: float
    absolute_height: float
    height_error: float  # Difference from target (should be ~ 0)
    success: bool
    message: str


def calculate_absolute_height(link_idx: int, offset: float, segment_heights: List[float]) -> float:
    """
    Calculate absolute height of attachment point.
    
    Args:
        link_idx: Index of link (0-indexed)
        offset: Offset along link (0 = base, link_height = top)
        segment_heights: Height of each segment
    
    Returns:
        Absolute height from base
    
    Example:
        >>> calculate_absolute_height(2, 0.1, [0.2, 0.2, 0.2, 0.2, 0.2])
        0.5  # 2*0.2 + 0.1
    """
    if link_idx < 0 or link_idx >= len(segment_heights):
        raise ValueError(f"link_idx {link_idx} out of range [0, {len(segment_heights)})")
    
    # Sum heights of all links before attachment link
    height_before = sum(segment_heights[:link_idx])
    
    # Add offset along attachment link
    return height_before + offset


def find_new_attachment(
    target_height: float,
    new_segment_heights: List[float],
    tolerance: float = 1e-6
) -> tuple[int, float]:
    """
    Find new attachment point (link_idx, offset) for target absolute height.
    
    Args:
        target_height: Target absolute height to match
        new_segment_heights: Heights of new segments
        tolerance: Tolerance for height matching
    
    Returns:
        (new_link_idx, new_offset)
    
    Raises:
        ValueError: If target height is out of range
    
    Example:
        >>> find_new_attachment(0.7, [0.33, 0.33, 0.33])
        (2, 0.04)  # link 2, offset 0.04m
    """
    total_height = sum(new_segment_heights)
    
    # Check bounds
    if target_height < -tolerance:
        raise ValueError(f"Target height {target_height} is negative")
    if target_height > total_height + tolerance:
        raise ValueError(
            f"Target height {target_height} exceeds total height {total_height}"
        )
    
    # Clamp to valid range
    target_height = max(0.0, min(target_height, total_height))
    
    # Find which link contains the target height
    cumulative_height = 0.0
    for link_idx, segment_height in enumerate(new_segment_heights):
        link_top = cumulative_height + segment_height
        
        if target_height <= link_top + tolerance:
            # Target is within this link
            offset = target_height - cumulative_height
            offset = max(0.0, min(offset, segment_height))  # Clamp to link bounds
            return link_idx, offset
        
        cumulative_height = link_top
    
    # Fallback: attach to last link, top
    return len(new_segment_heights) - 1, new_segment_heights[-1]


def remap_attachment_height(
    original_link_idx: int,
    original_offset: float,
    original_segment_heights: List[float],
    new_segment_heights: List[float],
    tolerance: float = 0.01  # 1cm tolerance
) -> RemappingResult:
    """
    Remap attachment point to preserve absolute height after segment collapse.
    
    Args:
        original_link_idx: Original attachment link index
        original_offset: Original offset along link (m)
        original_segment_heights: Original segment heights (m)
        new_segment_heights: New segment heights after collapse (m)
        tolerance: Max acceptable height error (m)
    
    Returns:
        RemappingResult with new attachment and validation info
    
    Example:
        >>> result = remap_attachment_height(
        ...     original_link_idx=3,
        ...     original_offset=0.1,
        ...     original_segment_heights=[0.2] * 5,
        ...     new_segment_heights=[0.33, 0.33, 0.34]
        ... )
        >>> result.new_link_idx, result.new_offset
        (2, 0.04)
    """
    # Validate inputs
    if not original_segment_heights:
        return RemappingResult(
            new_link_idx=0,
            new_offset=0.0,
            absolute_height=0.0,
            height_error=0.0,
            success=False,
            message="Original segment heights is empty"
        )
    
    if not new_segment_heights:
        return RemappingResult(
            new_link_idx=0,
            new_offset=0.0,
            absolute_height=0.0,
            height_error=0.0,
            success=False,
            message="New segment heights is empty"
        )
    
    if original_link_idx < 0 or original_link_idx >= len(original_segment_heights):
        return RemappingResult(
            new_link_idx=0,
            new_offset=0.0,
            absolute_height=0.0,
            height_error=0.0,
            success=False,
            message=f"Original link_idx {original_link_idx} out of range"
        )
    
    # Calculate target absolute height
    try:
        target_height = calculate_absolute_height(
            original_link_idx,
            original_offset,
            original_segment_heights
        )
    except Exception as e:
        return RemappingResult(
            new_link_idx=0,
            new_offset=0.0,
            absolute_height=0.0,
            height_error=0.0,
            success=False,
            message=f"Failed to calculate absolute height: {e}"
        )
    
    # Find new attachment
    try:
        new_link_idx, new_offset = find_new_attachment(target_height, new_segment_heights)
    except Exception as e:
        return RemappingResult(
            new_link_idx=0,
            new_offset=0.0,
            absolute_height=target_height,
            height_error=0.0,
            success=False,
            message=f"Failed to find new attachment: {e}"
        )
    
    # Verify new height
    actual_height = calculate_absolute_height(new_link_idx, new_offset, new_segment_heights)
    height_error = abs(actual_height - target_height)
    
    success = height_error <= tolerance
    
    if success:
        message = f"Remapped successfully (error: {height_error:.4f}m)"
    else:
        message = f"Height error {height_error:.4f}m exceeds tolerance {tolerance}m"
    
    return RemappingResult(
        new_link_idx=new_link_idx,
        new_offset=new_offset,
        absolute_height=actual_height,
        height_error=height_error,
        success=success,
        message=message
    )


def remap_all_children(
    parent_branch: dict,
    child_branches: List[dict],
    new_n_links: int,
    tolerance: float = 0.01
) -> tuple[List[dict], List[str]]:
    """
    Remap all child branches after parent segment collapse.
    
    Args:
        parent_branch: Parent branch config (with original n_links)
        child_branches: List of child branch configs to remap
        new_n_links: New number of links for parent
        tolerance: Height tolerance for remapping
    
    Returns:
        (remapped_branches, errors) tuple:
            - remapped_branches: List of updated branch configs
            - errors: List of error messages (empty if all successful)
    
    Example:
        >>> parent = {"id": "trunk", "n_links": 5, "height": 0.2}
        >>> children = [
        ...     {"id": "branch1", "parent": "trunk", "attach_link": 3, "attach_offset": 0.1},
        ...     {"id": "branch2", "parent": "trunk", "attach_link": 4, "attach_offset": 0.05}
        ... ]
        >>> remapped, errors = remap_all_children(parent, children, new_n_links=3)
        >>> len(errors)
        0
    """
    original_n_links = parent_branch["n_links"]
    segment_height = parent_branch.get("height", 0.2)  # Default 0.2m
    
    # Create segment height arrays
    original_heights = [segment_height] * original_n_links
    
    # Calculate new segment heights (distribute total height evenly)
    total_height = sum(original_heights)
    new_segment_height = total_height / new_n_links if new_n_links > 0 else 0.0
    new_heights = [new_segment_height] * new_n_links
    
    remapped = []
    errors = []
    
    for child in child_branches:
        # Skip if not a direct child of this parent
        if child.get("parent") != parent_branch["id"]:
            remapped.append(child.copy())
            continue
        
        # Get original attachment
        orig_link_idx = child.get("attach_link", 0)
        orig_offset = child.get("attach_offset", 0.0)
        
        # Remap
        result = remap_attachment_height(
            orig_link_idx,
            orig_offset,
            original_heights,
            new_heights,
            tolerance
        )
        
        if not result.success:
            errors.append(
                f"Failed to remap {child['id']}: {result.message}"
            )
            # Keep original attachment (fallback)
            remapped.append(child.copy())
        else:
            # Update attachment
            child_copy = child.copy()
            child_copy["attach_link"] = result.new_link_idx
            child_copy["attach_offset"] = result.new_offset
            remapped.append(child_copy)
    
    return remapped, errors
