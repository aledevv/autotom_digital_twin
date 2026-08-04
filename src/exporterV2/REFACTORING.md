# exporterV2 Refactoring (v2.1.0)

## Overview

Refactored exporterV2 from 2 large monolithic files (~1300 lines) to 10 focused modules (~200 lines each) for improved maintainability and testability.

## Changes

### Before (v2.0.0)
```
exporterV2/
├── csv_parser.py          (~600 lines - CSV + leaf logic + export)
├── generate_tree.py       (~700 lines - USD + physics + collision)
├── load_tree.py           (~100 lines - duplicated PhysX config)
├── main.py                (~150 lines - duplicated PhysX config)
└── tree_config.py         (~300 lines)
```

### After (v2.1.0)
```
exporterV2/
├── csv_data/                # CSV parsing module (renamed to avoid conflict with stdlib csv)
│   ├── __init__.py
│   ├── parser.py           (~350 lines - Generic CSV loading)
│   └── leaf_builder.py     (~250 lines - Leaf-specific logic)
│
├── usd/                     # USD generation module
│   ├── __init__.py
│   ├── stage.py            (~300 lines - Orchestration)
│   ├── geometry.py         (~60 lines - Link creation)
│   ├── joints.py           (~150 lines - Joint creation)
│   └── collision.py        (~100 lines - Collision filtering)
│
├── physics.py              (~40 lines - Shared PhysX config)
├── tree_config.py          (~300 lines - unchanged)
├── main.py                 (~150 lines - simplified)
├── load_tree.py            (~100 lines - simplified)
└── example_custom_tree.py  (~70 lines - unchanged)
```

## Key Improvements

### 1. Separation of Concerns
- **CSV parsing** separated from **leaf construction logic**
- **USD geometry** separated from **joints** and **collision filtering**
- **PhysX configuration** extracted to shared module (no duplication)

### 2. Lazy Loading
- Modules with external dependencies (pxr, pandas) are lazy-loaded
- Can import `exporterV2` without loading heavy dependencies
- Faster import times for scripts that don't need USD generation

### 3. Testability
- Each module can be tested independently
- CSV parser can run standalone: `python src/exporterV2/csv/parser.py --day 1`
- No pxr required for CSV testing

### 4. Maintainability
- Files are ~200-300 lines (down from 600-700)
- Clear module boundaries
- Easier to locate and modify specific functionality

## Backward Compatibility

All existing code continues to work without changes:

```python
# Still works exactly as before
from exporterV2 import build_stage, tree_config
from exporterV2.csv_data import parse_csv_to_branches

# New explicit imports (recommended)
from exporterV2.usd import build_stage
from exporterV2.csv_data import load_trunk_internodes, load_leaves
from exporterV2.physics import apply_physx_scene_settings
```

## Migration Guide

No changes required for existing code. The refactoring is fully backward compatible.

### Optional: Use explicit imports

Old style (still works):
```python
from exporterV2 import build_stage
```

New style (more explicit):
```python
from exporterV2.usd import build_stage
from exporterV2.csv_data import parse_csv_to_branches
from exporterV2.physics import apply_physx_scene_settings
```

## Important Notes

### Module Naming
The CSV parsing module is named `csv_data` (not `csv`) to avoid conflicts with Python's standard library `csv` module. This prevents import errors when Isaac Sim tries to import the standard `csv` module.

## Testing

All functionality verified:
- ✅ CSV parsing (day 1, 10, 50)
- ✅ Leaf filtering (opposite pairs)
- ✅ Branch construction
- ✅ JSON export
- ✅ Lazy loading
- ✅ Backward compatibility

## Files Deprecated

The following files have been renamed to `.old` and can be deleted after verification:
- `csv_parser.py.old` → replaced by `csv_data/parser.py` + `csv_data/leaf_builder.py`
- `generate_tree.py.old` → replaced by `usd/stage.py` + `usd/geometry.py` + `usd/joints.py` + `usd/collision.py`

## Version History

- **v2.0.0**: Original monolithic structure
- **v2.1.0**: Refactored to modular structure (this version)
