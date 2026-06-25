# AutoTom Digital Twin 🍅

[![Version](https://img.shields.io/badge/version-v1.5-blue.svg)](#)
[![Python](https://img.shields.io/badge/python-3.14%2B-blue.svg)](https://www.python.org/)
[![USD](https://img.shields.io/badge/USD-UsdCore-green.svg)](https://openusd.org/)
[![Dependency Manager](https://img.shields.io/badge/dependency_manager-uv-purple.svg)](https://github.com/astral-sh/uv)

This repository is dedicated to generating high-fidelity **Universal Scene Description (USD)** digital twins of tomato plants. The pipeline transforms outputs from the [GroIMP FSP-model-dwarf-tomato](https://github.com/Micbut/FSP-model-dwarf-tomato) growth simulator into interactive, physics-enabled 3D USD scenes.

## Pipeline Flow

```mermaid
graph LR
    A[GroIMP Growth Model] -->|Modified CSV Export| B[CSV Node Graph Data]
    B -->|USD Exporter Pipeline| C[Python USD Exporter]
    C -->|USD/USDA Scenes| D[Physics-Enabled USD Model]
```

1. **GroIMP Growth Model**: Run simulation inside GroIMP. Modifications were introduced to the GroIMP tomato model to export topological and physical organ properties.
2. **CSV Node Graph Data**: Daily CSV graph exports capturing structural data are written to the folder [model/output/dynamic_output/graphs/](https://github.com/aledevv/autotom_digital_twin/tree/main/data/dynamic_output/graphs).
3. **Python USD Exporter**: Rebuilds the hierarchical growth tree of the tomato plant and translates it into OpenUSD primitives.
4. **Physics-Enabled USD Model**: Outputs a single `.usda` scene featuring rigid-body stems, physical joint constraints, compound leaves, and fruit trusses.

---

## Demo Video

Below is the demo representing the exported USD tomato plant structure and physical dynamics under version 1.5:

<video src="https://github.com/user-attachments/assets/dea1479b-8c71-4847-a253-0b26884dee18" width="100%" controls muted autoplay loop>
</video>

---

## File Structure & Roles

The main source code is organized within the [src/plant_model](https://github.com/aledevv/autotom_digital_twin/tree/main/src/plant_model) package:

*   **[main.py](https://github.com/aledevv/autotom_digital_twin/blob/main/src/plant_model/main.py)**: Main entry point. Loads CSV data, runs the USD export pipeline, generates HTML Pyvis debug visualizer files, and exports JSON graph summaries.
*   **[models.py](https://github.com/aledevv/autotom_digital_twin/blob/main/src/plant_model/models.py)**: Defines python dataclasses for individual plant organs (internodes, leaves, fruits, roots) and growth day snapshots.
*   **[loader.py](https://github.com/aledevv/autotom_digital_twin/blob/main/src/plant_model/loader.py)**: Parses daily CSV files and reconstructs the parent-child graph topology.
*   **[usd_exporter.py](https://github.com/aledevv/autotom_digital_twin/blob/main/src/plant_model/usd_exporter.py)**: Main scene building pipeline. Maps components into USD structures, applies materials, and constructs rigid bodies/joints.
*   **[usd_helpers.py](https://github.com/aledevv/autotom_digital_twin/blob/main/src/plant_model/usd_helpers.py)**: Wraps low-level OpenUSD API operations, handling coordinate transforms, cylinder/sphere/mesh creations, material bindings, and physics setups.
*   **[constants.py](https://github.com/aledevv/autotom_digital_twin/blob/main/src/plant_model/constants.py)**: Contains simulation presets, joint limits, stiffness, damping, material colors, and organ dimensions.
*   **[debug_viz.py](https://github.com/aledevv/autotom_digital_twin/blob/main/src/plant_model/debug_viz.py)**: Utility that exports interactive plant structure graphs to HTML using Pyvis.
*   **[graph_export.py](https://github.com/aledevv/autotom_digital_twin/blob/main/src/plant_model/graph_export.py)**: Exports lightweight JSON representations of the plant topology.

---

## Installation & Execution

### Running the pipeline

You can execute the USD exporter using the provided helper shell script:
```bash
./run_main.sh
```

Or run it directly through `uv`:
```bash
uv run python src/plant_model/main.py
```

Or using standard Python (ensuring `PYTHONPATH` contains the `src` directory):
```bash
export PYTHONPATH=src
python src/plant_model/main.py
```

---

## Feature Status Matrix (v1.5)

| Category | Component / Feature | Status | Details |
| :--- | :--- | :---: | :--- |
| **Parsing** | Automated CSV topology & hierarchy mapping | ✅ | Automatically builds growth branches and relative attachments. |
| **Visuals** | Stem (Internode) rendering | ✅ | Cylindrical representation based on width and length properties. |
| **Visuals** | Root rendering | ✅ | Base grounding sphere under `z=0`. |
| **Visuals** | Compound leaf mesh generation | ✅ | Visualizes petiole, rachis, segments, and leaf blades. |
| **Visuals** | Fruits rendering | ✅ | Sphere approximations with dynamic colors based on ripening thermal age (Ripe Red vs Unripe Green-Yellow). |
| **Physics** | Internode (Stem) rigid body colliders | ✅ | Bounding volume and custom density mass estimation. |
| **Physics** | Stem Revolute Joints | ✅ | Flexible joint chains with height-interpolated stiffness & damping. |
| **Debug** | Interactive HTML topology visualizer | ✅ | Explores nodes, orders, and ranks using Pyvis. |
| **Visuals** | Improve visual appearance of leaves and stems | ❌ | Use some model like NeRF to reconstruct the plant shape/meshes from images. |
| **Physics** | Leaf physics / rigid body colliders | ❌ | Bending and collision models are not supported yet. |
| **Physics** | Fruits massAPI and complex physics | ❌ | Only static/kinematic colliders; complex dynamic weight/mass physics is missing. |
| **Physics** | Pedicel & truss rachis physics | ❌ | Treated as static/kinematic structures without dynamic joints. |
| **Simulation** | Isaac Sim simulation integration | ❌ | Output is exported to USD, but not yet integrated into Isaac Sim simulation scenes. |

---

### Python Stubs for IDE Autocomplition: 'typings' and '.vscode' folders
This folder contains **USD stubs** to easily work with 'usd-core' python module on your IDE and get **autocomplitions**. In order to make it work, DO NOT delete the .vscode folder (files are already set up).

> ⚠️ This only works in **VSCode**.

If you use another IDE, you can delete these folders. If you face any issue in VSCode, delete the .vscode folder.
