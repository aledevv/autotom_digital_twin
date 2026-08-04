# Plant Model Restructuring Summary

## Completed Actions

### 1. Renamed plant_model → exporterV1 ✅
**Location:** `/src/exporterV1/`

**Files preserved:**
- `loader.py` - CSV data loading
- `models.py` - Data structures (OrganNode, PlantSnapshot, etc.)
- `constants.py` - Physical parameters
- `usd_exporter.py` - USD generation
- `usd_helpers.py` - USD primitive helpers
- `main.py` - Entry point
- `debug_viz.py` - Visualization utilities
- `graph_export.py` - Graph export utilities

**Files removed:**
- `mainV2.py` - Obsolete
- `usd_exporter_v2.py` - Superseded by exporterV2
- `physx_runner.py` - Obsolete
- `physx_utils.py` - Obsolete
- `.DS_Store` - Artifact
- `__pycache__/` - Cache directory

**Added:**
- `__init__.py` - Module exports (load_snapshot, export_plant_usd)
- `README.md` - Documentation

### 2. Created exporterV2 ✅
**Location:** `/src/exporterV2/`

**Core files (from recursive_tree):**
- `tree_config.py` - Configuration and physics helpers (optimized)
- `generate_tree.py` - USD generation (optimized from generate_recursive_tree_usda.py)
- `load_tree.py` - Isaac Sim loader (from load_recursive_tree.py)

**Optimizations applied:**
1. **Simplified collision filtering:**
   - Created unified `_add_collision_filter()` helper
   - Simplified `_add_attachment_collision_filters()` with cleaner link parsing
   - Consolidated sibling filtering logic

2. **Improved code organization:**
   - Better docstrings with Args/Returns sections
   - Clearer function grouping (sections)
   - Removed experimental comments

3. **Enhanced maintainability:**
   - Extracted `_parse_link_number()` helper for link path parsing
   - More descriptive variable names
   - Consistent formatting

**Added:**
- `__init__.py` - Module exports (build_stage, tree_config, etc.)
- `README.md` - Comprehensive documentation with examples

### 3. Kept recursive_tree intact ✅
**Location:** `/src/experiments/recursive_tree/`

The experimental/alpha version remains unchanged with all tests, documentation, and measurement tools.

## Directory Structure

```
src/
├── exporterV1/              [Legacy CSV-based exporter]
│   ├── __init__.py
│   ├── README.md
│   ├── loader.py
│   ├── models.py
│   ├── constants.py
│   ├── usd_exporter.py
│   ├── usd_helpers.py
│   ├── main.py
│   ├── debug_viz.py
│   └── graph_export.py
│
├── exporterV2/              [Production tree exporter]
│   ├── __init__.py
│   ├── README.md
│   ├── tree_config.py       [Optimized config]
│   ├── generate_tree.py     [Optimized USD generator]
│   └── load_tree.py         [Isaac Sim loader]
│
└── experiments/
    └── recursive_tree/      [Unchanged alpha version]
        ├── tree_config.py
        ├── generate_recursive_tree_usda.py
        ├── load_recursive_tree.py
        ├── droop_theory.py
        ├── measure_droop.py
        ├── tests/
        └── ...
```

## Usage Examples

### exporterV1 (Legacy CSV-based)

```python
from src.exporterV1 import load_snapshot, export_plant_usd

snapshot = load_snapshot("plant_data.csv", day=10, plant_id=1)
export_plant_usd(snapshot, "plant_model.usda")
```

### exporterV2 (Tree model)

```python
from src.exporterV2 import build_stage, tree_config

# Generate with default config
stage, stem_path = build_stage("tree.usda")
stage.GetRootLayer().Save()

# Or customize
tree_config.GLOBAL_SCALE = 5.0
tree_config.BRANCHES = [...]
stage, stem_path = build_stage("custom_tree.usda")
```

**Run in Isaac Sim:**
```bash
~/isaacsim/python.sh -m src.exporterV2.load_tree
```

**Generate standalone:**
```bash
uv run python -m src.exporterV2.generate_tree
```

## Verification Results

✅ **exporterV1 structure:** All required files present, README created  
✅ **exporterV2 structure:** Core files copied and optimized  
✅ **tree_config.py:** Standalone execution successful  
✅ **generate_tree.py:** USD generation successful (tree_v2.usda created)  
✅ **recursive_tree:** Unchanged, all files intact  

## Key Improvements

1. **Cleaner separation:** V1 for CSV-based plants, V2 for recursive trees
2. **Optimized code:** Simplified collision filtering, better organization
3. **Better documentation:** READMEs with usage examples for both exporters
4. **Module structure:** Proper __init__.py with clean exports
5. **Maintained compatibility:** recursive_tree experiments unchanged

## Future Integration Plan

The two exporters will remain separate for now. In the future, you plan to:
- Implement CSV-to-BRANCHES converter in exporterV2
- This will allow recursive_tree to consume GroIMP organ data
- Both exporters will coexist for backward compatibility

## Testing

To verify everything works:

```bash
# Test tree_config
cd /home/alessandro/isaacsim/autotom_digital_twin/src/exporterV2
python tree_config.py

# Generate tree USD
cd /home/alessandro/isaacsim/autotom_digital_twin
uv run python -m src.exporterV2.generate_tree

# Check output
ls -lh data/usd_models/tree_v2.usda
```

All tests passed successfully! ✅
