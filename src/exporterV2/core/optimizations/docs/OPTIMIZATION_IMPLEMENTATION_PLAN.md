# Joint-Budget Optimization - Implementation Plan

> **IMPORTANTE**: Questo documento traccia lo stato di implementazione del sistema di ottimizzazione joints.
> Aggiorna lo stato delle task man mano che vengono completate.

## Status Overview

**Ultima Modifica**: 2025-01-08  
**Stato Generale**: 🟡 In Progress (5/12 tasks complete)

### Task Status Legend
- ✅ **DONE**: Task completata e testata
- 🟡 **IN PROGRESS**: Task in corso di implementazione
- 🔴 **TODO**: Task non ancora iniziata
- ⚠️ **BLOCKED**: Task bloccata da dipendenze

---

## Implementation Checklist

### Phase 1: Infrastructure (Tasks 1-3)
- [x] **Task 1**: Setup Infrastructure - Cartella optimizations e configurazione YAML ✅
- [x] **Task 2**: Collision Detection - Sistema Broad-Phase (Sphere + AABB) ✅
- [x] **Task 3**: Geometry Remapping - Attachment Point Recalculation ✅

### Phase 2: Optimization Techniques (Tasks 4-8)
- [x] **Task 4**: Tecnica 1 - Petiole Lock (D6 → Fixed Joint) ✅
- [x] **Task 5**: Tecnica 2 - Lateral Branch Reduction ✅
- [ ] **Task 6**: Tecnica 3 - Stem Collapse con Remapping
- [ ] **Task 7**: Tecnica 4 - Truss Static Pre-bent
- [ ] **Task 8**: Tecnica 5 - Leaf Branch Reduction (Petiole+Rachis merge)

### Phase 3: Integration & Validation (Tasks 9-12)
- [ ] **Task 9**: Integration Tests - Composizione Tecniche
- [ ] **Task 10**: Visual Validation Suite
- [ ] **Task 11**: Integrazione con Parse Pipeline
- [ ] **Task 12**: Documentazione Implementazione

---

## Quick Reference

### File Structure
```
exporterV2/
└── core/
    └── optimizations/           # Sistema di ottimizzazione
        ├── __init__.py
        ├── optimizer.py         # Orchestratore principale
        ├── budget_config.yaml   # Configurazione budget e limiti
        ├── techniques/          # Tecniche di ottimizzazione
        │   ├── __init__.py
        │   ├── base.py          # Classe base astratta
        │   ├── stem_collapse.py
        │   ├── petiole_lock.py
        │   ├── lateral_reduce.py
        │   ├── truss_static.py
        │   └── leaf_branch_reduce.py
        ├── collision/           # Utilità collision check
        │   ├── __init__.py
        │   ├── sphere.py
        │   ├── aabb.py
        │   └── broad_phase.py
        ├── geometry/            # Utilità geometriche
        │   ├── __init__.py
        │   ├── remapping.py
        │   └── bounds.py
        └── tests/               # Test suite
            ├── __init__.py
            ├── test_optimizer.py
            ├── test_stem_collapse.py
            ├── test_petiole_lock.py
            ├── test_lateral_reduce.py
            ├── test_truss_static.py
            ├── test_leaf_branch_reduce.py
            ├── test_collision_detection.py
            ├── test_geometry_remapping.py
            ├── test_integration_composition.py
            └── visual_validation/
                ├── run_visual_test.py
                ├── config_baseline.py
                ├── config_optimized.py
                └── README.md
```

### Key Configuration Files
- **budget_config.yaml**: Budget limiti, structural minimums, tecniche priority
- **optimizer.py**: Entry point per ottimizzazione
- **techniques/base.py**: Interface comune per tutte le tecniche

### Testing Strategy
1. **Unit Tests**: Per ogni tecnica isolata (geometry, collision, singola tecnica)
2. **Integration Tests**: Composizione multiple tecniche
3. **Visual Validation**: Verifica manuale su IsaacSim

---

## Problem Statement

Implementare un sistema di ottimizzazione incrementale del budget di joints per l'exporter USD di piante di pomodoro in Isaac Sim, riducendo progressivamente le articolazioni attraverso tecniche LOD-based validate dalla letteratura, mantenendo l'integrità strutturale e geometrica della pianta.

### Context
- **Hardware Limit**: ~250 joints per configurazione attuale (Isaac Sim/PhysX)
- **Current State**: ~200 joints al day 160 (trunk + lateral branches + foglie)
- **Missing Components**: Truss + pomodori (aggiungeranno joints)
- **Goal**: Sistema che riduce automaticamente joints mantenendo realismo visivo

---

## Requirements Summary

### Functional Requirements
1. Sistema di ottimizzazione incrementale con tecniche applicate in ordine di impatto minimo
2. Controllo strutturale lower-bound (blocca export se budget impossibile da rispettare)
3. Collision detection broad-phase (sphere + AABB) per validare remapping
4. Configurazione esterna YAML per budget, limiti e parametri tecniche
5. Report dettagliato con breakdown per tecnica
6. 5 tecniche: petiole lock, lateral reduce, stem collapse, truss static, leaf branch reduce
7. Test suite completa: unit + integration + visual validation

### Non-Functional Requirements
- **Performance**: Ottimizzazione < 1s per piante fino a day 160
- **Robustezza**: Errori chiari se ottimizzazione insufficiente
- **Manutenibilità**: Architettura modulare, tecniche facilmente estendibili

---

## Technical Background

### Literature Validation
- LOD-based joint reduction è pratica consolidata in skeletal animation e vegetation rendering
- Structural lower-bound è standard nei sistemi di semplificazione mesh/LOD
- Incremental budget-driven optimization è approccio corretto per PhysX articulations
- Broad-phase collision checks (sphere → AABB) è standard industriale

Riferimenti completi in: `docs/Research_Joint-Budget Optimization for USD Tomato Plant Exporter Approach Validation and SOTA Review.md`

### Existing Architecture
- `tree_config.py`: configurazione branches + validation
- `usd/stage.py`: builder USD con `build_chain()` e branch registry
- `adapters/groimp_csv/parser.py`: parsing CSV → branches
- **Gap**: Nessuna logica remapping attachment o collision detection geometrica

### Structural Constraints
- Trunk: min 1 link
- Lateral branch: min 1 link
- Petiole: min 1 link
- Rachis: min 0 link (può essere merged)
- Petiolule: min 0 link (può essere fixed/static)
- Truss: min 1 link

---

## Design Overview

### Optimization Flow

```
Branches Config (CSV parsed)
    ↓
Calculate Total Joints
    ↓
Within Budget? ────YES────> Export USD
    ↓ NO
Check Lower Bound
    ↓
Below Minimum? ────YES────> ❌ BuildError
    ↓ NO
Apply Next Technique (by priority)
    ↓
Validate Geometry & Collisions
    ↓
Valid? ────NO────> Revert + Try Alternative
    ↓ YES
Update Configuration
    ↓
[Loop back to Calculate Total Joints]
```

### Architecture Components

1. **Orchestrator** (`optimizer.py`):
   - Load config YAML
   - Calculate lower bound
   - Apply techniques sequentially by priority
   - Validate after each step
   - Generate report

2. **Technique Plugin System**:
   - Base class `OptimizationTechnique`
   - Methods: `can_apply()`, `estimate_reduction()`, `apply()`, `validate()`
   - Tecniche isolate e testabili

3. **Collision System**:
   - Stage 1: Sphere overlap (fast pre-check)
   - Stage 2: AABB overlap (precision)
   - Modular components in `collision/`

4. **Geometry Utilities**:
   - Attachment remapping when segments collapse
   - Bounding volume calculation

### Technique Priority Order

1. **Petiole Lock** (Priority 1): D6 → Fixed joint (no geometry change)
2. **Lateral Reduce** (Priority 2): Reduce lateral branch segments
3. **Stem Collapse** (Priority 3): Collapse main stem + remap attachments
4. **Truss Static** (Priority 4): Pre-bent static geometry
5. **Leaf Branch Reduce** (Priority 5): Merge petiole+rachis

Order based on: impatto visivo minimo → realismo preservato

---

## Detailed Task Breakdown

### Task 1: Setup Infrastructure - Cartella optimizations e configurazione YAML

**Status**: ✅ DONE (Completed: 2025-01-08)

**Obiettivo**: Creare struttura base del modulo optimizations con configurazione YAML e orchestratore skeleton.

**Deliverables**:
- [x] Cartella `exporterV2/core/optimizations/` con struttura completa ✅
- [x] File `budget_config.yaml` completo (budget, limiti, tecniche) ✅
- [x] File `optimizer.py` con classe `BudgetOptimizer` ✅:
  - `__init__()`, `load_config()`, `calculate_total_joints()`, `calculate_lower_bound()`, `optimize()`
  - Solo loading config e calcoli (senza tecniche implementate)
- [x] File `techniques/base.py` con classe astratta `OptimizationTechnique` ✅

**Testing**:
- [x] Unit test `test_optimizer_simple.py` (6 tests passed) ✅:
  - Loading config YAML
  - Calcolo joints totali
  - Calcolo lower bound (diversi scenari)
  - Error handling config invalida
- [x] Demo script `demo_task1.py` funzionante ✅

**Demo**: ✅ Script `demo_task1.py` carica config, calcola joints/lower bound, stampa report

**Dependencies**: Nessuna

**Actual Time**: ~3 ore

**Notes**: 
- Aggiunte dipendenze: PyYAML installato con `uv add pyyaml`
- Tutti i test passano con successo
- Demo script dimostra funzionalità complete di Task 1

---

### Task 2: Collision Detection - Sistema Broad-Phase (Sphere + AABB)

**Status**: ✅ DONE (Completed: 2025-01-08)

**Obiettivo**: Implementare sistema collision detection two-stage per validare attachment remapping.

**Deliverables**:
- [x] File `collision/sphere.py` ✅:
  - `calculate_bounding_sphere(link_geometry) -> (center, radius)`
  - `check_sphere_overlap(sphere1, sphere2, margin) -> bool`
- [x] File `collision/aabb.py` ✅:
  - `calculate_aabb(link_geometry) -> (min_point, max_point)`
  - `check_aabb_overlap(aabb1, aabb2) -> bool`
- [x] File `collision/broad_phase.py` ✅:
  - `check_attachment_collision(new_link, siblings, parent, margin) -> CollisionResult`
  - Logica two-stage: sphere → AABB

**Testing**:
- [x] Unit test `test_collision_detection.py` (12 tests passed) ✅:
  - Sphere overlap (touching, overlapping, separated)
  - AABB overlap con cilindri orientati
  - Broad-phase con scenari realistici
  - Safety margin
- [ ] Fixture: geometrie sintetiche (cilindri con posizioni/orientamenti noti)

**Demo**: Visualizzazione 3D (matplotlib) collision check per attachment remappati.

**Dependencies**: Nessuna

**Estimated Time**: 3-4 ore

---

### Task 3: Geometry Remapping - Attachment Point Recalculation

**Status**: ✅ DONE (Completed: 2025-01-08)

**Obiettivo**: Implementare logica geometrica per rimappare attachment points quando si collassano segmenti.

**Deliverables**:
- [x] File `geometry/remapping.py` ✅:
  - `remap_attachment_height(original_link_idx, original_n_links, new_n_links, segment_heights) -> (new_link_idx, offset_z)`
  - Preserva altezza geometrica assoluta
  - Gestisce edge cases (primo/ultimo link, segmenti non uniformi)
- [x] File `geometry/bounds.py` ✅:
  - `link_to_cylinder_geometry(branch, link_idx) -> CylinderGeometry`
  - Helper per calcolo bounds da branches config

**Testing**:
- [x] Unit test `test_geometry_remapping.py` (8 tests passed) ✅:
  - Remapping con stem 5→3→1 links
  - Preservazione altezza assoluta (tolerance 1%)
  - Segmenti non uniformi
  - Edge cases (attach top/bottom)
  - Batch remapping multiple children

**Demo**: ✅ Tabella comparativa altezze attachment prima/dopo remapping

**Dependencies**: Nessuna

**Actual Time**: ~2.5 ore

**Notes**:
- Sub-millimeter accuracy (<0.01mm error)
- Works for any collapse ratio (5→4, 5→3, 5→2, 5→1)
- Integrates with Task 2 collision detection via CylinderGeometry
- Ready for Task 6 (Stem Collapse technique)

---

### Task 4: Tecnica 1 - Petiole Lock (D6 → Fixed Joint)

**Status**: ✅ DONE (Completed: 2025-01-08)

**Obiettivo**: Convertire petiolule joints da D6 a Fixed, riducendo DOF senza cambiare geometria.

**Deliverables**:
- [x] File `techniques/petiole_lock.py` ✅:
  - Classe `PetioleLockTechnique` extends `OptimizationTechnique`
  - `can_apply()`: controlla petiolules con D6 joints
  - `estimate_reduction()`: conta petiolules convertibili
  - `apply()`: aggiungi metadata `joint_type: "fixed"`
  - `validate()`: topologia preservata
- [x] Integrazione `stage.py`: estendi `build_chain()` per `joint_type` override ✅

**Testing**:
- [x] Unit test `test_petiole_lock.py`: 8 tests passed ✅
  - Identificazione petiolules
  - Conversione preserva geometria
  - Stima riduzione corretta
  - Validation topologia
- [x] Integration test: genera USD, verifica joints sono FixedJoint ✅

**Demo**: ✅ Script IsaacSim baseline vs petioles locked, USD generati (baseline.usda + petiole_lock.usda)

**Dependencies**: Task 1

**Actual Time**: ~3.5 ore

**Notes**:
- Aggiunto supporto `joint_type` metadata in `stage.py` (backward compatible)
- Bug fix: `_is_petiolule()` None check per parent
- USD files generati: 37KB baseline, 33KB locked (4 FixedJoint vs 1)
- Riduzione: 18 DOF (3 petiolule × 6 DOF ciascuno)

---

### Task 5: Tecnica 2 - Lateral Branch Reduction

**Status**: ✅ DONE (Completed: 2025-01-08)

**Obiettivo**: Ridurre numero di segmenti in lateral branches incrementalmente con geometry remapping.

**Deliverables**:
- [x] File `techniques/lateral_reduce.py` ✅:
  - Classe `LateralBranchReductionTechnique`
  - `can_apply()`: lateral branches con n_links > min_segments
  - `estimate_reduction()`: somma links riducibili
  - `apply()`: riduci n_links di 1, ricalcola height, remap children
  - `validate()`: min_links rispettato, geometry preserved
  - Priority strategy: smallest radius → lowest attach → alphabetical

**Testing**:
- [x] Unit test `test_lateral_reduce.py`: 12 tests passed ✅
  - Identificazione lateral branches/leaves
  - Reduction priority ordering
  - Riduzione incrementale con height recalculation
  - Child attachment remapping (usa Task 3)
  - Rispetto min_segments
  - Validation topology e geometry
  - Multiple branches con priority
- [x] Integration test: verifica USD generation ✅

**Demo**: ✅ Script genera USD (baseline: 8 lateral links → reduced: 3 lateral links, 5 links saved)

**Dependencies**: Task 1, Task 3 (geometry remapping)

**Actual Time**: ~3.5 ore

**Notes**:
- Identificazione: `Branch_r*_o*` e `LateralLeaf_r*_o*` patterns
- Height recalculation: `new_height = old_height * old_n_links / new_n_links`
- Child remapping: usa `remap_attachment_height()` da Task 3
- USD files: 45KB baseline (931 lines) → 31KB reduced (650 lines)
- Reduction: 5 links rimossi (3 branches: 3+3+2 → 1+1+1)

---

### Task 6: Tecnica 3 - Stem Collapse con Remapping

**Status**: 🔴 TODO

**Obiettivo**: Collassare main stem segments con remapping attachment points e collision check.

**Deliverables**:
- [ ] File `techniques/stem_collapse.py`:
  - Classe `StemCollapseTechnique`
  - `can_apply()`: trunk con n_links > min_segments
  - `estimate_reduction()`: n_links_trunk - 1
  - `apply()`:
    1. Riduci trunk n_links di 1
    2. Ricalcola attach_link per child branches (usa `remap_attachment_height()`)
    3. Valida collision (usa `check_attachment_collision()`)
    4. Fallback se collision irrisolvibile
  - `validate()`: tutti attachment validi, no overlaps

**Testing**:
- [ ] Unit test `test_stem_collapse.py`:
  - Remapping attachment 5→3, 5→1 links
  - Collision detection con siblings
  - Fallback collision irrisolvibile
  - Preservazione topologia
- [ ] Integration test: genera USD, verifica geometria

**Demo**: Script IsaacSim trunk 5 links vs 1 link con lateral branches remappati.

**Dependencies**: Task 1, Task 2, Task 3

**Estimated Time**: 4-5 ore

---

### Task 7: Tecnica 4 - Truss Static Pre-bent

**Status**: 🔴 TODO

**Obiettivo**: Convertire truss da articolato a geometria statica pre-piegata.

**Deliverables**:
- [ ] File `techniques/truss_static.py`:
  - Classe `TrussStaticTechnique`
  - `can_apply()`: truss con n_links > min_segments
  - `estimate_reduction()`: conta joints eliminabili
  - `apply()`:
    1. Riduci truss a 1 link
    2. Genera geometria mesh pre-bent
    3. Aggiungi metadata `prebent: true`
  - `validate()`: geometria mesh valida
- [ ] Nota: implementazione completa dipende da truss non ancora in codebase

**Testing**:
- [ ] Unit test `test_truss_static.py`:
  - Riduzione truss multi-segment → single static
  - Generazione geometria pre-bent
  - Stima riduzione
- [ ] Placeholder test con branch generico

**Demo**: Script genera USD con truss pre-bent (quando truss implementato).

**Dependencies**: Task 1

**Estimated Time**: 3-4 ore

---

### Task 8: Tecnica 5 - Leaf Branch Reduction (Petiole+Rachis merge)

**Status**: 🔴 TODO

**Obiettivo**: Ridurre petiole+rachis a singolo segmento opzionalmente pre-bent.

**Deliverables**:
- [ ] File `techniques/leaf_branch_reduce.py`:
  - Classe `LeafBranchReductionTechnique`
  - `can_apply()`: petiole+rachis riducibili
  - `estimate_reduction()`: conta links eliminabili
  - `apply()`:
    1. Identifica petiole+rachis pairs
    2. Merge in single branch (somma lunghezze, media raggi)
    3. Se `prebend: true`, calcola angle
  - `validate()`: attachment petiolules validi

**Testing**:
- [ ] Unit test `test_leaf_branch_reduce.py`:
  - Merge preserva lunghezza totale
  - Prebend angle calculation
  - Preservazione attachment petiolules
  - Diversi tipi foglie (trunk, lateral)
- [ ] Integration test: verifica leaf structure USD

**Demo**: Script IsaacSim foglia full-articulated vs single-segment pre-bent.

**Dependencies**: Task 1

**Estimated Time**: 3-4 ore

---

### Task 9: Integration Tests - Composizione Tecniche

**Status**: 🔴 TODO

**Obiettivo**: Testare applicazione sequenziale di multiple tecniche e riduzione cumulativa.

**Deliverables**:
- [ ] File `test_integration_composition.py`:
  - Scenario 1: Pianta semplice over budget → petiole lock + lateral reduce → rientra
  - Scenario 2: Pianta complessa → tutte tecniche in ordine → riduzione progressiva
  - Scenario 3: Pianta impossible (under lower bound) → errore bloccante
  - Scenario 4: Pianta border case (al budget) → nessuna tecnica applicata

**Testing**:
- [ ] Verifica joint count dopo ogni step
- [ ] Report contiene breakdown per tecnica
- [ ] Geometria USD valida
- [ ] No regression (tecniche non interferiscono)
- [ ] Snapshot testing per regression check
- [ ] Performance test: < 1s

**Demo**: Script genera 4 USD (uno per scenario), stampa report, carica IsaacSim.

**Dependencies**: Task 4, Task 5, Task 6, Task 7, Task 8

**Estimated Time**: 4-5 ore

---

### Task 10: Visual Validation Suite

**Status**: 🔴 TODO

**Obiettivo**: Suite test visuali IsaacSim con istruzioni verifica manuale.

**Deliverables**:
- [ ] File `visual_validation/run_visual_test.py`:
  - Genera N configurazioni (baseline, tech1, tech1+2, ..., full)
  - Carica ogni USD in IsaacSim automaticamente
  - Stampa istruzioni verifica manuale
- [ ] File `visual_validation/config_baseline.py`: pianta ~300 joints
- [ ] File `visual_validation/config_optimized.py`: pianta ~200 joints
- [ ] File `visual_validation/README.md`: checklist verifica per tecnica

**Testing**:
- [ ] Nessun test automatico (verifica manuale)
- [ ] Checklist README:
  - Petiole lock: petiolules statici
  - Lateral reduce: branch più rigidi
  - Stem collapse: lateral branches posizionati correttamente
  - Truss static: geometria pre-bent
  - Leaf reduce: foglie single-segment

**Demo**: Esegui suite, genera 6 USD, carica in IsaacSim, segui checklist.

**Dependencies**: Task 9

**Estimated Time**: 3-4 ore

---

### Task 11: Integrazione con Parse Pipeline

**Status**: 🔴 TODO

**Obiettivo**: Integrare optimizer nel flusso parse_csv_to_branches → build_stage.

**Deliverables**:
- [ ] Estendi `parse_csv_to_branches()` in `parser.py`:
  - Parametro `optimize: bool = False`
  - Se `optimize=True`, chiama `BudgetOptimizer.optimize(branches)`
- [ ] Estendi `main.py`:
  - Argomento `--optimize` CLI
  - Passa flag a parse
- [ ] Logging: stampa report ottimizzazione prima di build USD

**Testing**:
- [ ] Integration test: parse CSV day 50 → optimize → build USD → verifica joint count
- [ ] Test CLI: `python main.py --day 50 --optimize`
- [ ] Test fallback: CSV oltre lower bound → errore chiaro

**Demo**: `./run_mainV2.sh --day 50 --optimize`, report nel log, carica USD.

**Dependencies**: Task 9

**Estimated Time**: 2-3 ore

---

### Task 12: Documentazione Implementazione

**Status**: 🔴 TODO

**Obiettivo**: Documentazione completa per reference futura e onboarding.

**Deliverables**:
- [ ] File `optimizations/README.md`:
  - Panoramica sistema
  - Come configurare budget/tecniche (YAML)
  - Come aggiungere nuova tecnica
  - Troubleshooting
- [ ] File `optimizations/DESIGN.md`:
  - Architettura dettagliata
  - Diagrammi flusso
  - Decisioni design
- [ ] File `optimizations/TESTING.md`:
  - Guida testing nuova tecnica
  - Come eseguire visual validation
  - Interpretare report
- [ ] Aggiorna `exporterV2/README.md` con sezione optimization

**Testing**:
- [ ] Review documentazione: esempi funzionanti
- [ ] Link interni corretti

**Demo**: README completo con esempi runnable.

**Dependencies**: Tutte le altre task

**Estimated Time**: 3-4 ore

---

## Effort Estimation

| Phase | Tasks | Estimated Time |
|-------|-------|----------------|
| Phase 1: Infrastructure | 1-3 | 7-10 ore |
| Phase 2: Techniques | 4-8 | 16-21 ore |
| Phase 3: Integration | 9-12 | 12-16 ore |
| **Total** | **12 tasks** | **35-47 ore** |

---

## Notes for Implementation

### Best Practices
1. **Test First**: Scrivi test prima di implementare (TDD quando possibile)
2. **Incremental**: Ogni task deve produrre codice funzionante e testato
3. **Git Commits**: Commit dopo ogni task completata con messaggio chiaro
4. **Documentation**: Aggiorna inline docstrings + README man mano

### Common Pitfalls
- Non dimenticare collision check dopo remapping (Task 6 dipende da Task 2+3)
- Preserva sempre topologia parent-child anche dopo merge/collapse
- Testa edge cases: pianta minima (1 trunk), pianta massiva (>500 joints)

### Testing Guidelines
- Unit test devono essere veloci (< 100ms ciascuno)
- Integration test possono generare USD temporanei (usa `pytest-tmp`)
- Visual validation è manuale ma deve avere checklist chiara

---

## Resources

- **Research Document**: `docs/Research_Joint-Budget Optimization for USD Tomato Plant Exporter Approach Validation and SOTA Review.md`
- **Collision Recommendation**: `docs/collision_check_recommendation.md`
- **Existing Architecture**: `core/tree_config.py`, `core/usd/stage.py`, `adapters/groimp_csv/parser.py`

---

## Change Log

| Data | Task | Descrizione | Autore |
|------|------|-------------|--------|
| 2025-01-08 | - | Documento creato | Alessandro |
| 2025-01-08 | 1 | ✅ Completata Task 1: Setup Infrastructure | Alessandro |
| 2025-01-08 | 2 | ✅ Completata Task 2: Collision Detection | Alessandro |
| 2025-01-08 | 3 | ✅ Completata Task 3: Geometry Remapping | Alessandro |
| 2025-01-08 | 4 | ✅ Completata Task 4: Petiole Lock | Alessandro |
| 2025-01-08 | 5 | ✅ Completata Task 5: Lateral Branch Reduction | Alessandro |
| | | | |

---

**Next Steps**: Inizia con Task 1 (Setup Infrastructure). Una volta completata, aggiorna questo documento marcando la task come ✅ DONE e aggiorna il change log.
