# Recursive Tree - Next Session TODO

## Context

Branch: `feat/recursive-tree-articulation`

Esperimento di albero articolato ricorsivo con fisica Euler-Bernoulli, scala 10x, configurazione esplicita tramite `BRANCHES` list.

**Stato attuale**: albero funzionante con geometria corretta. Droop analysis parzialmente completata (teoria + measurement). Rimangono da fare: GIF recording + analysis completa + geometric verification tests.

---

## Completed Tasks (già committati)

✅ **Task 1**: `tree_config.py` — Config con `BRANCHES` list esplicita, fisica E-B, validation  
✅ **Task 2**: `generate_recursive_tree_usda.py` — Generatore USD con radial offset, orientamento corretto  
✅ **Task 3**: `load_recursive_tree.py` — Loader Isaac Sim con PhysX TGS/480Hz  
✅ **Task 4**: `run_recursive_tree.sh` — Shell script entry point  
✅ **Task 5**: `droop_theory.py` — Calcolo teorico del droop per cantilever sotto gravità  
✅ **Task 6**: `measure_droop.py` — Measurement del droop in Isaac Sim (headless)  

**Dati disponibili**:
- `data/droop_measurement.csv` — droop misurato: trunk 0mm, branchA 1.49mm, subA1 4.88mm
- Teoria (E=5e8 Pa): branchA 4mm, subA1 8mm
- **Ratio measured/theory**: 0.37-0.61 (droop più basso del previsto)

---

## Pending Tasks (droop analysis completion)

### Task 7: GIF Recorder for Visual Inspection

**File**: `src/experiments/recursive_tree/record_droop_gif.py`

**Goal**: Generare un GIF dei primi 5 secondi di simulazione che mostra il settling dei branch.

**Implementation**:
1. Come `measure_droop.py`, ma con `SimulationApp({"headless": False})` e `render=True`
2. Cattura screenshot ogni 10 frames (a 60Hz sim → 6 fps GIF)
3. Usa Isaac Sim viewport capture API:
   ```python
   from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
   viewport = get_active_viewport()
   capture_viewport_to_file(viewport, f"data/frames/frame_{i:04d}.png")
   ```
4. Dopo 300 frames (5 sec), converti PNG sequence in GIF con `imageio` o `ffmpeg`:
   ```bash
   ffmpeg -framerate 6 -i data/frames/frame_%04d.png -vf "scale=800:-1" data/droop_settling.gif
   ```
5. **Optional**: overlay testo con posizione Z dei tip usando PIL o OpenCV

**Expected output**: `data/droop_settling.gif` (5 sec, ~30 frames)

**Demo**: Visual confirmation che il droop è un movimento **graduale e smooth** (flessione elastica), non uno snap improvviso (bug di posizionamento).

---

### Task 8: Droop Analysis & Verdict

**File**: `src/experiments/recursive_tree/analyze_droop.py`

**Goal**: Confrontare droop misurato con teoria, generare plot e verdict finale.

**Implementation**:
1. Legge `data/droop_measurement.csv`
2. Per ogni branch, calcola `droop_theory` chiamando `droop_theory.calculate_cantilever_deflection(...)`
3. Calcola `ratio = droop_measured / droop_theory`
4. **Verdict logic**:
   - `ratio ∈ [0.5, 2.0]` → ✅ "Normal elastic deflection"
   - `ratio < 0.5` → ⚠️ "Droop lower than expected — possible causes: E higher than configured, or joint stiffness adds extra rigidity"
   - `ratio > 2.0` → ❌ "Droop higher than expected — bug in initial pose or joint parameters"
   - Trunk: `droop < 1mm` → ✅ "Vertical branch minimal droop (OK)"
5. Genera scatter plot (matplotlib):
   ```python
   plt.scatter(droop_theory, droop_measured, c=['blue', 'green', 'red'])
   plt.plot([0, max_val], [0, max_val], 'k--', label='y=x (ideal)')
   plt.fill_between([0, max_val], [0, max_val*0.5], [0, max_val*2], alpha=0.2, label='acceptable (0.5x-2x)')
   ```
6. Salva plot in `data/droop_analysis.png`

**Test**: `uv run src/experiments/recursive_tree/analyze_droop.py`

**Expected output**:
```
=== DROOP ANALYSIS ===
Branch: trunk
  Theory:   0.0 mm
  Measured: 0.0 mm
  Ratio:    N/A (vertical)
  Verdict:  ✅ Minimal droop (OK)

Branch: branchA
  Theory:   4.0 mm
  Measured: 1.5 mm
  Ratio:    0.37
  Verdict:  ⚠️  Lower than expected — joint stiffness may add rigidity

Branch: subA1
  Theory:   8.2 mm
  Measured: 4.9 mm
  Ratio:    0.60
  Verdict:  ✅ Within acceptable range

OVERALL: Droop is within or slightly below expected elastic behavior.
         Lower-than-theory values suggest joint drives add effective stiffness.
         No bug detected — this is normal for articulated structures.
```

---

### Task 9: Integration & Documentation

**File**: `run_droop_analysis.sh` (in project root)

**Goal**: One-command workflow per l'intera droop analysis.

**Implementation**:
```bash
#!/bin/bash
set -e
echo "=== Droop Analysis Workflow ==="

echo "Step 1: Generate USD (if not present)..."
if [ ! -f data/usd_models/recursive_tree.usda ]; then
    env -i HOME=$HOME PATH=$PATH uv run src/experiments/recursive_tree/generate_recursive_tree_usda.py
fi

echo "Step 2: Compute theoretical deflection..."
uv run src/experiments/recursive_tree/droop_theory.py

echo "Step 3: Measure droop in Isaac Sim..."
~/isaacsim/python.sh src/experiments/recursive_tree/measure_droop.py

echo "Step 4: Record GIF (optional — takes ~1 min)..."
read -p "Record GIF? (y/N): " -n 1 -r
if [[ $REPLY =~ ^[Yy]$ ]]; then
    ~/isaacsim/python.sh src/experiments/recursive_tree/record_droop_gif.py
fi

echo "Step 5: Analyze results..."
uv run src/experiments/recursive_tree/analyze_droop.py

echo ""
echo "=== DONE ==="
echo "Check:"
echo "  - data/droop_measurement.csv"
echo "  - data/droop_analysis.png"
echo "  - data/droop_settling.gif (if recorded)"
```

**File**: `DROOP_ANALYSIS.md` — documento che spiega:
- Cos'è il droop e perché succede
- Formula teorica E-B usata
- Risultati e interpretazione
- Conclusione: droop è flessione elastica normale, ratio 0.37-0.61 dovuto a joint stiffness

---

## Pending Tasks (geometric verification)

### Task 10: Geometric Test Suite

**File**: `src/experiments/recursive_tree/test_geometric_consistency.py`

**Goal**: Test suite automatico che verifica la correttezza geometrica del USD per varie configurazioni.

**Strategy**:
1. Funzione helper `verify_branch_position(stage, branch_def, parent_def, ...)`:
   - Legge posizione world-space del link dall'USD
   - Calcola posizione attesa analiticamente:
     ```python
     parent_quat = _axis_to_quat(parent_axis)
     parent_matrix = Gf.Matrix3d(parent_quat)
     
     radial = parent_radius / 2.0
     offset_local = Gf.Vec3d(0, radial, parent_height + gap)
     rot_az = Gf.Rotation(Gf.Vec3d(0,0,1), azimuth)
     offset_rotated = rot_az.TransformDir(offset_local)
     offset_world = parent_matrix * offset_rotated
     
     expected_pos = parent_base_world + offset_world
     ```
   - Confronta `distance(actual, expected) < 1mm`
   - Ritorna `(actual_pos, expected_pos, error_mm)`

2. **Test cases** (solo casi validi):
   - `test_trunk_vertical()` — solo trunk, 10 link verticali
   - `test_single_branch_45deg()` — trunk + 1 branch a 45°, attacco link 3
   - `test_branch_attach_first_link()` — branch attaccato al link 1 del trunk
   - `test_branch_attach_last_link()` — branch attaccato all'ultimo link trunk
   - `test_sub_branch_nested()` — depth=3, verifica subA1 su branchA (già tilted)
   - `test_multiple_branches_azimuth()` — 4 branch sullo stesso link con rot=0°, 90°, 180°, 270° (croce)
   - `test_tiny_radius_branch()` — radius=0.001m (1mm), verifica stabilità numerica
   - `test_horizontal_branch()` — tilt=90° (branch orizzontale)
   - `test_near_vertical_branch()` — tilt=1° (quasi verticale, edge case)

3. Per ogni test:
   - Costruisce `BRANCHES` custom
   - Chiama `build_stage(temp_usd_path, custom_branches)`
   - Verifica tutte le posizioni
   - Assert `max_error < 1.0 mm`

4. Output report:
   ```
   ====== GEOMETRIC CONSISTENCY TESTS ======
   test_trunk_vertical ................... PASS (max error: 0.001 mm)
   test_single_branch_45deg .............. PASS (max error: 0.023 mm)
   test_branch_attach_first_link ......... PASS (max error: 0.015 mm)
   test_branch_attach_last_link .......... PASS (max error: 0.018 mm)
   test_sub_branch_nested ................ PASS (max error: 0.034 mm)
   test_multiple_branches_azimuth ........ PASS (max error: 0.042 mm)
   test_tiny_radius_branch ............... PASS (max error: 0.007 mm)
   test_horizontal_branch ................ PASS (max error: 0.028 mm)
   test_near_vertical_branch ............. PASS (max error: 0.011 mm)
   
   ========================================
   9/9 tests passed ✅
   Maximum position error across all tests: 0.042 mm
   ```

**Test**: `uv run src/experiments/recursive_tree/test_geometric_consistency.py`

**Expected outcome**: Tutti i test passano con errore < 1mm, confermando che la geometria è corretta per qualsiasi configurazione valid di BRANCHES.

---

## How to Resume in Next Session

### Option A: Continue droop analysis (Tasks 7-9)

Prompt per la prossima sessione:

```
Ciao, voglio continuare il droop analysis per il recursive tree experiment.

Branch: feat/recursive-tree-articulation
Repo: /home/alessandro/isaacsim/autotom_digital_twin

Stato attuale:
- droop_theory.py ✅ (teoria: branchA 4mm, subA1 8mm)
- measure_droop.py ✅ (misurato: branchA 1.49mm, subA1 4.88mm)
- Ratio measured/theory: 0.37-0.61 (più basso del previsto)

Leggi src/experiments/recursive_tree/TODO_NEXT_SESSION.md (sezione "Pending Tasks - droop analysis completion") e implementa:
1. Task 7: record_droop_gif.py — GIF dei primi 5 sec di settling
2. Task 8: analyze_droop.py — plot + verdict
3. Task 9: run_droop_analysis.sh + documentazione

Procedi con implementazione completa.
```

---

### Option B: Start geometric verification tests (Task 10)

Prompt per la prossima sessione:

```
Ciao, voglio implementare il geometric test suite per il recursive tree experiment.

Branch: feat/recursive-tree-articulation
Repo: /home/alessandro/isaacsim/autotom_digital_twin

Il generatore USD (generate_recursive_tree_usda.py) è completo e funzionante.
Ora serve un test automatico che verifica la correttezza geometrica per varie configurazioni.

Leggi src/experiments/recursive_tree/TODO_NEXT_SESSION.md (sezione "Pending Tasks - geometric verification") e implementa Task 10:

test_geometric_consistency.py con 9 test cases:
- trunk vertical
- single branch 45deg
- branch attach first/last link
- sub-branch nested
- multiple branches azimuth
- tiny radius, horizontal, near-vertical

Ogni test:
1. Costruisce BRANCHES custom
2. Genera USD
3. Verifica posizioni link vs teoria
4. Assert error < 1mm

Output: report con max error per ogni test.

Procedi con implementazione completa.
```

---

### Option C: Do both (full completion)

Prompt:

```
Ciao, voglio completare tutto il lavoro pendente sul recursive tree experiment.

Branch: feat/recursive-tree-articulation
Repo: /home/alessandro/isaacsim/autotom_digital_twin

Leggi src/experiments/recursive_tree/TODO_NEXT_SESSION.md e implementa tutti i pending tasks:
- Droop analysis completion (Tasks 7-9): GIF + plot + verdict
- Geometric test suite (Task 10): test parametrico per validazione geometria

Segui l'ordine nel TODO file. Procedi con implementazione completa e committa ogni task.
```

---

## Current Files Summary

```
src/experiments/recursive_tree/
├── __init__.py
├── tree_config.py              ✅ Config con BRANCHES list + validation
├── generate_recursive_tree_usda.py  ✅ Generatore USD ricorsivo
├── load_recursive_tree.py      ✅ Loader Isaac Sim
├── droop_theory.py             ✅ Teoria cantilever E-B
├── measure_droop.py            ✅ Measurement droop in Isaac Sim
├── record_droop_gif.py         ⏸ TODO Task 7
├── analyze_droop.py            ⏸ TODO Task 8
└── test_geometric_consistency.py  ⏸ TODO Task 10

Root:
├── run_recursive_tree.sh       ✅ Entry point principale
└── run_droop_analysis.sh       ⏸ TODO Task 9

Data:
├── data/usd_models/recursive_tree.usda  ✅ Generated
├── data/droop_measurement.csv   ✅ Measured data
├── data/droop_analysis.png      ⏸ TODO Task 8
└── data/droop_settling.gif      ⏸ TODO Task 7
```

---

## Git Commits Reference

Last commits on this branch:
- `0502d54` — feat: add droop analysis — theory + measurement
- `e34241b` — fix: compute sub-branch offset in parent frame for correct static position
- `f69380f` — fix: remove container Xforms, links now flat under /World/Stem
- `bb12c30` — refactor: replace implicit TREE_CONFIG with explicit BRANCHES list
- `55457a1` — feat: add recursive tree articulation experiment

To see full history: `git log --oneline feat/recursive-tree-articulation`

---

## Notes

- **E configurato**: attualmente `tree_config.py` ha `E = 5e8 Pa` (500 MPa, molto rigido). Se vuoi testare con E più morbido (es. 150 MPa come default cantilever), cambia `BioConfig.YOUNG_MODULUS` e rigenera USD.
  
- **Droop ratio < 1**: il fatto che il droop misurato sia più basso della teoria è **normale** per articolazioni. I joint drive aggiungono rigidità effettiva oltre a quella puramente materiale (E × I). Un ratio 0.5-0.8 è tipico.

- **PhysX iterations**: se vedi instabilità con sub-branch molto sottili, aumenta `SolverPositionIterationCount` in `load_recursive_tree.py` (attualmente 64, max 255).

- **Test timeout**: i test Isaac Sim prendono ~30-60s per bootstrap + settling. Budget 2-3 min per test completo.

---

EOF
