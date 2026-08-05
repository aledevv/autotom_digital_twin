# ExporterV2 Changelog

## [2.2.0] - 2026-08-05

### 🎉 Major Refactoring - Architecture Restructure

**Phase 1-2: Separation of Generic vs Cultivar-Specific Logic**

#### Added
- ✨ `core/` module - Generic tree builder (reusable for any plant)
- ✨ `adapters/groimp_csv/` - CSV adapter with profile system
- ✨ `profiles/` module - Cultivar configuration system
- ✨ `profiles/tomato_default.py` - Tomato cultivar configuration
- ✨ `profiles/simple_plant.py` - Example alternative profile
- ✨ `test_refactoring.sh` - Automated test suite
- 📖 `REFACTORING_SUMMARY.md` - Detailed change documentation
- 📖 `REFACTORING_COMPLETE.md` - Final summary and sign-off
- 📖 Updated `README.md` with new architecture

#### Changed
- 🔧 Restructured directories: `core/`, `adapters/`, `profiles/`
- 🔧 All imports updated to new structure
- 🔧 Path resolution fixed for nested modules
- 🔧 Lazy imports to avoid pxr dependency outside Isaac Sim
- 🔧 `parse_csv_to_branches()` now accepts `profile` parameter

#### Fixed
- 🐛 CSV path resolution for nested adapter structure
- 🐛 tree_config.py import paths in parser and leaf_builder
- 🐛 Lazy import strategy to handle pxr unavailability

#### Migration
- ✅ No breaking changes - all existing scripts work
- ✅ Default profile = tomato (preserves current behavior)
- ✅ Output identical to pre-refactoring
- ✅ All tests passing

---

## [2.1.0] - 2026-08-04

### Lateral Branches & Leaves Implementation

#### Added
- ✨ Lateral branch support (order=1 internodes)
- ✨ Lateral leaf support (order=1 leaves)
- ✨ Opposite pair filtering for cultivar-specific branching
- ✨ Random leaf orientation on lateral branches
- ✨ Leaf cloning for incomplete pairs

#### Fixed
- 🐛 Duplicate branch IDs between trunk and lateral leaves
- 🐛 attach_link calculation for lateral branch leaves
- 🐛 Leaf orientation on lateral branches (perpendicular + random)

---

## [2.0.0] - 2026-07-29

### Initial Modular Refactoring

#### Added
- ✨ `csv_data/` module for CSV parsing
- ✨ `usd/` module for USD generation
- ✨ Modular architecture with clear separation

#### Changed
- 🔧 Split monolithic code into modules
- 🔧 Improved code organization

---

## [1.0.0] - 2026-07-08

### Initial Release

- Basic tree generation from CSV
- Trunk and leaf support
- PhysX articulation
- USD output

---

## Legend

- ✨ New feature
- 🔧 Changed
- 🐛 Bug fix
- 📖 Documentation
- ✅ Test/Verification
- 🎉 Major milestone
