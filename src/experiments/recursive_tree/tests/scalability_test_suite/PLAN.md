# Refactoring Scalability Tests and Plot Generation

This plan outlines the steps to reorganize the scalability tests and create the analytical plots for the thesis, focusing on the relationship between joints, days, and constraint memory (O(n²)).

## Proposed Changes

### 1. File Reorganization (Refactoring)
Create a new directory `src/experiments/recursive_tree/tests/scalability_test_suite` and move all scalability-related files into it.

#### [NEW] `src/experiments/recursive_tree/tests/scalability_test_suite/`
#### [MODIFY] Move the following files into the new directory:
- `src/experiments/recursive_tree/tests/test_scalability.py`
- `src/experiments/recursive_tree/tests/test_stress_limits.py`
- `src/experiments/recursive_tree/tests/SCALABILITY_TEST_RESULTS.md`
- `src/experiments/recursive_tree/tests/scalability_usds/` (entire folder)
- `src/experiments/recursive_tree/SCALABILITY_ANALYSIS.md` (move to the new suite for consistency)

*Note: All import paths inside the python scripts will be updated to point to the correct parent directories.*

### 2. Analytical Plots Generation
Create a new script to generate the 5/6 plots requested.

#### [NEW] `src/experiments/recursive_tree/tests/scalability_test_suite/generate_thesis_plots.py`
This script will use `matplotlib` and `numpy` (and `pandas` if needed) to generate the following visual analyses:

**Plot 1: The "Ragnarok" Limit (Joints vs Memory)**
- **X-axis:** Number of Simulated Joints (50 to 300)
- **Y-axis:** Estimated Constraint Memory / Complexity $O(n^2)$
- **Visuals:** 
  - A quadratic trendline showing the memory/complexity load.
  - A shaded background region (e.g., light red) starting from ~260 joints, labeled as "PhysX Core Dump Zone".
  - The trendline will be solid up to 260, and dashed/interrupted inside the crash zone.
  - A text box indicating the hardware dependency (e.g., "Hardware: NVIDIA RTX XXXX, RAM XXGB - PhysX Engine Limit").

**Plots 2 & 3: Unoptimized Growth (Days vs Joints & Memory)**
- **X-axis:** Simulation Days (1 to 160)
- **Y-axis:** Number of Joints / Constraint Memory
- **Visuals:** Shows the exponential/logistic growth of the raw plant geometry. Will clearly show the curve hitting the 260-joint "Crash Zone" around a specific day.

**Plots 4 & 5: Optimized Growth (Days vs Joints & Memory)**
- **X-axis:** Simulation Days (1 to 160)
- **Y-axis:** Number of Joints / Constraint Memory
- **Visuals:** Shows the effect of `--optimize`. The curve will flatten or grow much slower, remaining in the "Safe Zone" well below 260 joints.

**Plot 6 (Bonus/Comparative): Unoptimized vs Optimized Overlay**
- An overlaid comparison plot of both Joint counts (Optimized vs Unoptimized) against Days on the same axes to clearly demonstrate the effectiveness of the `BudgetOptimizer`.

### 3. Documentation
#### [NEW] `src/experiments/recursive_tree/tests/scalability_test_suite/PLAN.md`
As requested, I will export a markdown file directly into the new folder detailing this exact plan and how the plots are calculated, so you have it persistently for your thesis write-up.

## Open Questions
> [!IMPORTANT]
> 1. **Plant Growth Data:** To generate the plots from Day 1 to 160, do you have a specific CSV file already generated that lists the "Number of Joints" per day? Or would you prefer the script to use a realistic mathematical growth function (e.g., Logistic growth reaching ~500 raw joints at day 160) to simulate the unoptimized plant, and a capped function for the optimized one based on your `BudgetOptimizer` logic?
> 2. **Hardware Specs:** What are the exact hardware specs (GPU, RAM) you want me to write in the text box of the first plot?

## Verification Plan
1. Ensure all moved Python scripts (`test_scalability.py`, etc.) still run successfully without import errors.
2. Run `generate_thesis_plots.py` to ensure it successfully saves the `.png` plots to the disk.
3. Review the generated plots to ensure the shaded crash zone and trendlines accurately reflect the findings in `SCALABILITY_ANALYSIS.md`.
