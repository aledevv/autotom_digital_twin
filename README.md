# AutoTom Digital Twin 🍅

[![Version](https://img.shields.io/badge/version-v2.0-blue.svg)](#)
[![Python](https://img.shields.io/badge/python-3.10%2B-blue.svg)](https://www.python.org/)
[![USD](https://img.shields.io/badge/USD-OpenUSD-green.svg)](https://openusd.org/)
[![Isaac Sim](https://img.shields.io/badge/Isaac%20Sim-PhysX-nvidia.svg)](https://developer.nvidia.com/isaac-sim)
[![Dependency Manager](https://img.shields.io/badge/dependency_manager-uv-purple.svg)](https://github.com/astral-sh/uv)

This repository is dedicated to generating high-fidelity **Universal Scene Description (USD)** digital twins of tomato plants. The pipeline transforms outputs from the [GroIMP FSP-model-dwarf-tomato](https://github.com/Micbut/FSP-model-dwarf-tomato) growth simulator into interactive, physics-enabled 3D USD scenes optimized for robotics simulation in **NVIDIA Isaac Sim**.

> [!IMPORTANT]
> **Exporter V2 (`src/exporterV2`) is the active, state-of-the-art exporter pipeline.**  
> Unlike V1 (which generated static/kinematic USD models), Exporter V2 builds **fully articulated, physics-driven multibody plants** equipped with PhysX D6 joint dynamics, automated collision filtering, cultivar profiles, and a **Joint-Budget LOD Optimization System** to guarantee real-time Isaac Sim simulation stability.

---

## 🏗️ Pipeline Flow

```mermaid
graph LR
    A[GroIMP Growth Model] -->|Modified CSV Export| B[CSV Node Graph Data]
    B -->|Exporter V2 Pipeline| C[Budget Optimizer & Physics Engine]
    C -->|OpenUSD Generation| D[Physics-Enabled USD Plant in Isaac Sim]
```

1. **GroIMP Growth Model**: Executes growth simulation inside GroIMP to export topological and physical organ properties.
2. **CSV Node Graph Data**: Daily CSV graph exports capturing structural data are stored under [`data/simulation_output/dynamic_output/graphs/`](data/simulation_output/dynamic_output/graphs/).
3. **Exporter V2 Pipeline & Budget Optimizer**: Reconstructs parent-child topologies, parameterizes PhysX joints using Euler-Bernoulli beam theory, and applies LOD joint reduction techniques to respect hardware budgets (~250 joints max).
4. **Physics-Enabled USD Model**: Outputs a `.usda` scene featuring rigid-body stems, physical joint constraints, compound leaves, and fruit trusses.

---

## 🎬 Demo Video

Below is a demonstration of the exported USD tomato plant structure and physical dynamics:

<video src="https://github.com/user-attachments/assets/c0e688ce-e95a-447d-8f5c-63a488afddd2" width="100%" controls muted autoplay loop>
</video>

---

## 📂 Repository Structure & Exporter Versions

> [!NOTE]
> The codebase contains two exporter generations under `src/`:

### 🌟 Exporter V2 (Active & Recommended) — `src/exporterV2`
Production-ready, modular exporter designed for Isaac Sim physics simulation.
*   **`src/exporterV2/core/`**: Generic USD builder, joint mechanics (PhysX D6 drives), and Euler-Bernoulli stiffness calculations.
*   **`src/exporterV2/core/optimizations/`**: **Joint-Budget LOD System**. Algorithmically reduces joint counts (Petiole Lock, Lateral Branch Reduction, Stem Collapse) to stay below solver limits.
*   **`src/exporterV2/adapters/`**: Converts raw GroIMP CSV data into universal branch representations.
*   **`src/exporterV2/profiles/`**: Cultivar-specific parameters (e.g., tomato branch angles, leaf cloning, phyllotaxis).
*   **`src/exporterV2/docs/`**: Comprehensive technical documentation & thesis chapters.

### 📜 Exporter V1 (Legacy Static Baseline) — `src/plant_model`
Monolithic CSV parser and static USD exporter.
*   *Scope:* Exports static/kinematic plant geometry without articulated joint physics or optimization tools. Kept for historical reference and baseline visual comparison.

---

## 🚀 Execution & Usage

> [!TIP]
> Use **`./run_mainV2.sh`** to run the active V2 pipeline.

### Running Exporter V2 (Interactive Physics & Optimization)
```bash
# Export day 100 with Joint-Budget Optimization enabled (Recommended)
./run_mainV2.sh --day 100 --optimize

# Export without optimization
./run_mainV2.sh --day 100
```

### Running Exporter V1 (Legacy Static Export)
```bash
./run_main.sh
```

---

## 📊 Feature Status Matrix (Exporter V2)

| Category | Component / Feature | Status | Details |
| :--- | :--- | :---: | :--- |
| **Architecture**| Exporter V2 Modular Pipeline | ✅ | Clean separation of generic tree building, data adapters, and profiles. |
| **Optimization**| Joint-Budget LOD System | ✅ | Reduces physics joints to stay within hardware limits (~250 joints max). |
| **Parsing** | Automated CSV topology & hierarchy mapping | ✅ | Automatically builds growth branches and relative attachments. |
| **Physics** | Stem Revolute Joints & D6 Colliders | ✅ | Flexible joint chains with height-interpolated stiffness & damping. |
| **Visuals** | Stem (Internode) & Root rendering | ✅ | Cylindrical representation and base grounding sphere. |
| **Visuals** | Compound leaf mesh generation | ✅ | Visualizes petiole, rachis, segments, and leaf blades. |
| **Visuals** | Plant texturing and coloring | ❌ | *In Progress:* Adding high-fidelity realistic colors and textures to plant geometry. |
| **Physics** | Truss & Pedicel physics | ❌ | *In Progress:* Dynamic articulated joint support for fruit trusses. |
| **Physics** | Fruits massAPI and dynamic weight | ❌ | Static/kinematic colliders present; full dynamic weight physics pending. |
| **Simulation** | Isaac Sim multi-plant scene integration | ❌ | Standalone USD models exported; multi-plant scene orchestration pending. |

---

## 🔮 Future Optimization Plans

> [!NOTE]
> **Proximity-Based Physics Activation (In Planning)**  
> In greenhouse environments containing dozens of tomato plants and a mobile harvesting robot, running physics on all plants simultaneously is computationally prohibitive. We plan to implement dynamic proximity activation: articulated physics will be enabled **only for plants within the robot's interaction radius**, while distant plants remain static/kinematic rigid bodies.

---

### Python Stubs for IDE Autocompletion: `typings` and `.vscode` folders
This folder contains **USD stubs** to easily work with `usd-core` python module on your IDE and get **autocompletions**. In order to make it work, DO NOT delete the `.vscode` folder (files are already set up).

> ⚠️ This only works in **VSCode**.
