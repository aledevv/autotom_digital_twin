# Joint-Budget Optimization System - Documentation Index

> **Incremental optimization system to reduce joint count in USD plant models for Isaac Sim/PhysX.**

## 📚 Available Documentation

### 1. [7_Comprehensive_Optimization_Report.md](./notion_pages/7_Comprehensive_Optimization_Report.md)
**Contents**: The exhaustive technical paper for the thesis, describing architecture, algorithms, and empirical results.

**When to use it**:
- Exporting to Notion for thesis integration
- Deep-dive technical understanding of the entire pipeline
- Results analysis (Day 100 benchmark)
- Study of implementation trade-offs

---

### 2. [llm_context/ (Archive)](./llm_context/)
**Contents**: Archive folder containing technical design documents (Design, Implementation Plan, Task Summaries).

**When to use it**:
- Providing context for LLMs in future conversations
- Historical debugging of design decisions
- Technical backup of individual implementation phases (Tasks 1-12)

---

### 3. [OPTIMIZATION_QUICK_START.md](./OPTIMIZATION_QUICK_START.md)
**Contents**: Quick reference guide to get started, usage examples, configuration, and troubleshooting.

**When to use it**:
- First approach to the system
- Quick reference during usage and development
- Debugging common issues
- API code examples

---

### 4. This File (README)
**Contents**: General overview and documentation index.

---

## 🎯 Quick Navigation

| Goal... | Go to... |
|-----------|----------|
| Get started using the system | [Quick Start](./OPTIMIZATION_QUICK_START.md) |
| Read Technical Paper / Thesis Report | [Comprehensive Report](./notion_pages/7_Comprehensive_Optimization_Report.md) |
| LLM Context / Design History | [llm_context/](./llm_context/) |
| Troubleshooting | [Quick Start](./OPTIMIZATION_QUICK_START.md#common-issues--solutions) |
| Configure budget | [Quick Start](./OPTIMIZATION_QUICK_START.md#configuration-quick-ref) |
| Testing Reference | [Quick Start](./OPTIMIZATION_QUICK_START.md#testing-reference) |

---

## 📋 Executive Summary

### Problem
Isaac Sim/PhysX has a hardware-imposed limit of ~250 joints per articulation. Day 100+ tomato plants with multiple branches, trusses, and fruits exceed this limit, causing physics solver instability or engine crashes.

### Solution
An incremental optimization system that applies 5 LOD-based techniques ordered by minimal visual impact, reducing joints until fitting within the budget while maintaining structural integrity.

### Techniques (Priority Order)
1. **Petiole Lock** (Priority 1): D6 → Fixed joint (no geometry change)
2. **Lateral Reduce** (Priority 2): Reduce segments in lateral branches
3. **Stem Collapse** (Priority 3): Collapse trunk segments + remap attachments
4. **Truss Static** (Priority 4): Pre-bent static geometry
5. **Leaf Branch Reduce** (Priority 5): Merge petiole + rachis

### Key Features
- ✅ **Incremental**: Applies techniques progressively, stopping immediately when budget is met
- ✅ **Safe**: Geometric validation + collision checks after each step
- ✅ **Transparent**: Detailed report with breakdown per technique
- ✅ **Configurable**: External YAML for budget, limits, parameters
- ✅ **Extensible**: Plugin architecture for new techniques

### Validation
- **Research-backed**: Approach validated by LOD/MOR literature (see `Research_Joint-Budget Optimization for USD Tomato Plant Exporter Approach Validation and SOTA Review.md`)
- **Industry standard**: Two-stage broad-phase collision detection (Sphere + AABB)

---

## 🚀 Getting Started

### For Developers & Researchers

1. **Read** [Comprehensive Report](./notion_pages/7_Comprehensive_Optimization_Report.md) for technical details and algorithms.
2. **Consult** `llm_context/` directory for historical design notes and task summaries.
3. **Use** [Quick Start](./OPTIMIZATION_QUICK_START.md) for quick references on running tests and configuring the environment.

### For Users

```python
# Basic Example
from exporterV2.core.optimizations import BudgetOptimizer

optimizer = BudgetOptimizer()
optimized_branches, report = optimizer.optimize(branches)
print(report)
```

```bash
# From CLI
./run_mainV2.sh --day 50 --optimize
```

See [Quick Start](./OPTIMIZATION_QUICK_START.md) for full examples.

---

## 📊 Project Status

✅ **Project Completed**: All phases (Infrastructure, Techniques, Integration, and Visual Validation) have been successfully implemented, tested, and the documentation has been organized for thesis work.
See [Comprehensive Report](./notion_pages/7_Comprehensive_Optimization_Report.md) for results.

---

## 🏗️ Architecture Overview

```
Optimizer (Orchestrator)
    ↓
Apply Techniques by Priority
    ↓
┌─────────────┬──────────────┬────────────┐
│  Technique  │  Collision   │  Geometry  │
│  Plugins    │  Detection   │  Remapping │
└─────────────┴──────────────┴────────────┘
    ↓
Optimized Branches Config
    ↓
build_stage() → USD Export
```

See [Comprehensive Report](./notion_pages/7_Comprehensive_Optimization_Report.md) for architecture details.

---

## 📝 Notes

### Design Principles
- **Minimal Visual Impact**: Priority to techniques preserving realistic appearance
- **Structural Integrity**: Never drop below structural lower bound
- **Fail Safe**: Clear error message if budget cannot be met
- **Transparent**: Report tracks every step

---

## 🔗 Related Documentation

- **Research Background**: `Research_Joint-Budget Optimization for USD Tomato Plant Exporter Approach Validation and SOTA Review.md`
- **Collision Recommendation**: `collision_check_recommendation.md`
- **Tree Config**: `../core/tree_config.py`
- **USD Stage Builder**: `../core/usd/stage.py`

---

**Version**: 2.0  
**Author**: Alessandro
