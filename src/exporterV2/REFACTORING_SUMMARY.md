# ExporterV2 Refactoring Summary

## Phase 1-2: Architecture Restructure & Configuration Extraction ✅

**Date:** August 5, 2026  
**Status:** Complete

---

## What Changed

### New Directory Structure

```
exporterV2/
├── core/                        # Generic tree builder (reusable)
│   ├── tree_config.py          # BRANCHES format & physics
│   ├── physics.py              # PhysX configuration
│   └── usd/                    # USD generation
│       ├── stage.py
│       ├── geometry.py
│       ├── joints.py
│       └── collision.py
│
├── adapters/                    # Data source adapters
│   └── groimp_csv/             # groIMP CSV adapter
│       ├── parser.py           # CSV loading & filtering
│       └── leaf_builder.py     # Leaf construction
│
├── profiles/                    # Cultivar configurations
│   └── tomato_default.py       # Tomato cultivar profile
│
├── main.py
├── load_tree.py
└── example_custom_tree.py
```

### Key Improvements

1. **Separation of Concerns**
   - `core/` = Generic (works with any plant)
   - `adapters/groimp_csv/` = CSV-specific logic
   - `profiles/` = Cultivar-specific parameters

2. **Configuration Extraction**
   - Hardcoded values moved to `profiles/tomato_default.py`
   - Easy to create new cultivar profiles
   - Profile-driven filtering and orientation

3. **Clean Interfaces**
   - `parse_csv_to_branches(day, plant_id, profile)` → BRANCHES list
   - `build_stage(usd_path, branches)` → USD stage
   - Profile defaults to tomato if not specified

---

## What Stayed the Same

- ✅ Output behavior identical (same JSON, same USD)
- ✅ Command line interface unchanged: `./run_mainV2.sh --day 1`
- ✅ All existing functionality preserved
- ✅ No breaking changes for current workflows

---

## Tomato Profile Configuration

**File:** `profiles/tomato_default.py`

```python
TOMATO_PROFILE = {
    "lateral_branches": {
        "organ_indices": [0, 1],      # Opposite pairs only
        "tilt_deg": 45.0,             # Fixed tilt from trunk
        "rot_base_deg": [0.0, 180.0], # Symmetric rotation
    },
    "trunk_leaves": {
        "filter_strategy": "opposite_pairs_180deg",
        "phyllotaxis_deg": 137.5,     # Golden angle
    },
    "lateral_leaves": {
        "organ_indices": [0, 1],
        "clone_missing": True,
        "tilt_deg": 75.0,
        "rot_range_deg": (-90.0, 90.0),
    },
}
```

---

## How to Use

### Default (Tomato Profile)
```python
from exporterV2.adapters.groimp_csv import parse_csv_to_branches

# Uses tomato profile automatically
branches, json_path = parse_csv_to_branches(day=100)
```

### Custom Profile
```python
from exporterV2.adapters.groimp_csv import parse_csv_to_branches

CUSTOM_PROFILE = {
    "lateral_branches": {"enabled": False},  # No lateral branches
    # ...
}

branches, json_path = parse_csv_to_branches(day=100, profile=CUSTOM_PROFILE)
```

### Manual BRANCHES Config (No CSV)
```python
from exporterV2.core import tree_config
from exporterV2.core.usd import build_stage

tree_config.BRANCHES = [
    {"id": "trunk", "parent": None, ...},
    # ...
]

stage, stem_path = build_stage("output.usda")
```

---

## Testing

**Verified:**
- ✅ Day 1, 50, 100, 160 - All produce identical output
- ✅ Import structure works (lazy pxr loading)
- ✅ Path resolution correct for all modules

**Test command:**
```bash
./run_mainV2.sh --day 1   # Should work as before
```

---

## Next Steps (Phase 3-4 - Future)

**Phase 3: Refactor Internals**
- Split `parser.py` → `loader.py` (generic) + `tomato_filters.py` (specific)
- Make `leaf_builder.py` accept orientation calculator callback
- Add pluggable filtering strategies

**Phase 4: Polish**
- Create second profile (e.g., simple test plant)
- Add profile validation
- Update README with architecture diagram
- Create migration guide for other cultivars

---

## Files Modified

**Moved:**
- `tree_config.py` → `core/tree_config.py`
- `physics.py` → `core/physics.py`
- `usd/*.py` → `core/usd/*.py`
- `csv_data/*.py` → `adapters/groimp_csv/*.py`

**Updated:**
- All import statements throughout codebase
- Path resolution in `parser.py` and `leaf_builder.py`
- `__init__.py` files with lazy imports

**Created:**
- `profiles/tomato_default.py` - Cultivar configuration
- `core/__init__.py` - Core module exports
- `adapters/__init__.py` - Adapter exports
- `profiles/__init__.py` - Profile exports
- `REFACTORING_SUMMARY.md` - This document

---

## Migration Notes

**For developers working on other cultivars:**

1. Copy `profiles/tomato_default.py` → `profiles/your_cultivar.py`
2. Adjust parameters for your plant
3. Pass your profile to `parse_csv_to_branches()`
4. No changes to core needed!

**For recursive_tree experiments:**
- No changes needed - core modules are unchanged
- `example_custom_tree.py` still works with manual BRANCHES config

---

## Maintainer: Alessandro
**Last Updated:** August 5, 2026
