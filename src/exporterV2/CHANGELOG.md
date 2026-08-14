# Exporter V2 Changelog

## [2.4.0] - 2026-08-14

### Repository Cleanup and Documentation

#### Added
- Canonical documentation under `docs/` for architecture, physics, CSV adaptation, collision checks, testing, implementation notes, and troubleshooting.
- Root README comparison between V1 and V2 with local demo videos from `assets/`.
- Cleaner project metadata and direct runtime requirements.

#### Changed
- Root now keeps only the essential V1 and V2 runners.
- Experiment runners live beside their experiments.
- Visual validation scripts live under `src/exporterV2/demos/` and are outside normal pytest collection.
- Intermediate refactoring and task-summary reports were consolidated into the canonical docs.

## [2.3.0] - 2026-08-14

### Truss Detachment and Conservative Refactor

#### Added
- CSV-derived truss rachis, one-link pedicels, terminal tomato metadata, and detachable tomato rigid bodies.
- `/World/TerminalBodies` for standalone tomato bodies connected by breakable FixedJoints.
- Runtime collision filters between terminal tomatoes, pedicels, and rachis links.
- Terminal tomato solver iteration settings in `PhysicsRuntimeConfig`.
- Tests covering FixedJoint authoring, break force, `excludeFromArticulation`, terminal-body placement, collision filters, and tomato solver iterations.

#### Changed
- V2 runtime is tuned at `480 Hz` with GPU dynamics enabled.
- Articulation and terminal-body solver iterations were tuned for smoother truss behavior.
- CSV parsing now reads the GroIMP CSV once per full pipeline run while preserving standalone loader compatibility.
- Leaf and truss builders cache `tree_config` loading and keep package and standalone import paths working.
- Stage construction reuses branch lookup maps and isolates terminal tomato authoring in a helper.

## [2.2.0] - 2026-08-05

### Modular Architecture

#### Added
- `core/` module for generic tree building, physics helpers, USD stage construction, and optimization.
- `adapters/groimp_csv/` for GroIMP CSV ingestion.
- `profiles/` for cultivar-specific behavior.
- Tomato default profile and simple alternative profile.

#### Changed
- Exporter logic moved from the initial monolithic layout to `core/adapters/profiles`.
- CSV loaders gained profile-driven filtering and organ generation.
- Lazy USD imports allow ordinary Python tests to run outside Isaac Sim when PhysX modules are unavailable.

## [2.1.0] - 2026-08-04

### Lateral Branches and Leaves

#### Added
- Lateral branch support from order-1 internodes.
- Lateral leaf support with petiole, rachis, petiolule, and blade hierarchy.
- Opposite-pair filtering and deterministic leaf orientation behavior for the tomato profile.

#### Fixed
- Duplicate branch IDs between trunk and lateral leaves.
- Attachment link calculation for lateral branch leaves.
- Leaf orientation and cloning behavior for incomplete pairs.

## [2.0.0] - 2026-07-29

### Initial V2 Pipeline

#### Added
- Modular CSV-to-USD exporter prototype.
- Basic articulated trunk and leaf output.
- PhysX-ready USD stage generation.

## [1.0.0] - 2026-07-08

### Initial Baseline

- Basic tree generation from CSV.
- Trunk and leaf support.
- Initial USD output.
