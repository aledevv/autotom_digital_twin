# Test Suite per Recursive Tree USD Generator

Questo documento descrive la suite di test completa per il generatore USD dell'albero ricorsivo articolato.

## Overview

La test suite è composta da 3 file principali:

1. **`test_geometric_consistency.py`** - Test analitici di consistenza geometrica (9 test)
2. **`test_error_handling.py`** - Test di gestione errori (8 test)
3. **`test_isaac_sim_integration.py`** - Test di integrazione con Isaac Sim (3 test)

---

## Test Analitici (no Isaac Sim required)

### 1. Geometric Consistency Tests

**File**: `test_geometric_consistency.py`  
**Scopo**: Verifica che le posizioni dei link nel USD generato corrispondano ai calcoli analitici attesi.

**Esecuzione**:
```bash
cd /path/to/autotom_digital_twin
uv run src/experiments/recursive_tree/test_geometric_consistency.py
```

**Durata**: ~10 secondi

**Test cases** (9 totali):
1. **trunk_vertical** - Trunk verticale con 10 link
2. **single_branch_45deg** - Trunk + branch a 45°
3. **branch_attach_first_link** - Branch attaccato al primo link
4. **branch_attach_last_link** - Branch attaccato all'ultimo link
5. **sub_branch_nested** - Nesting ricorsivo (depth=3)
6. **multiple_branches_azimuth** - 4 branch formano croce (0°/90°/180°/270°)
7. **tiny_radius_branch** - Radius 1mm (stabilità numerica)
8. **horizontal_branch** - Tilt 90° (orientamento perpendicolare)
9. **near_vertical_branch** - Tilt 1° (preservazione piccoli angoli)

**Tolleranza**: < 1.0 mm per tutte le posizioni

**Output atteso**:
```
====== GEOMETRIC CONSISTENCY TESTS ======
Test 1: trunk_vertical
  trunk_Link_01             error  0.000 mm ✓
  ...
  → PASS (max error: 0.000 mm)
...
9/9 tests passed ✅
Maximum position error: 0.000 mm
```

---

### 2. Error Handling Tests

**File**: `test_error_handling.py`  
**Scopo**: Verifica che configurazioni invalide vengano correttamente respinte con messaggi chiari.

**Esecuzione**:
```bash
cd /path/to/autotom_digital_twin
uv run src/experiments/recursive_tree/test_error_handling.py
```

**Durata**: < 1 secondo

**Test cases** (8 totali):
1. **duplicate_ids** - Duplicate branch IDs
2. **no_root** - Nessun branch root
3. **multiple_roots** - Più branch root
4. **unknown_parent** - Parent ID inesistente
5. **missing_attach_link** - Branch senza attach_link
6. **attach_link_not_integer** - attach_link non intero
7. **attach_link_out_of_range** - attach_link fuori range valido
8. **too_many_links** - Total links > 64 (limite PhysX)

**Output atteso**:
```
====== ERROR HANDLING TEST SUITE ======
Test 1: duplicate_ids
  ✓ Correctly rejected with error:
    [tree_config] Duplicate branch id: 'branch'
...
8/8 tests passed ✅
```

---

## Test Integrazione Isaac Sim (require Isaac Sim)

### 3. Isaac Sim Integration Tests

**File**: `test_isaac_sim_integration.py`  
**Scopo**: Verifica consistenza geometrica quando USD viene caricato in Isaac Sim.

**Esecuzione**:
```bash
cd /path/to/autotom_digital_twin
~/isaacsim/python.sh src/experiments/recursive_tree/test_isaac_sim_integration.py
```

**Durata**: ~60-90 secondi (include 5s di simulazione)

**Test scenarios** (3 totali):

#### Test 1: Geometry after stage open
- **Verifica**: Posizioni dopo caricamento USD (no simulazione)
- **Atteso**: 0.000 mm error
- **Significato**: USD carica correttamente in Isaac Sim

#### Test 2: Geometry after world reset
- **Verifica**: Posizioni dopo inizializzazione PhysX (joint flessibili)
- **Atteso**: ~8-10 mm error (flessione sotto gravità)
- **Significato**: Joint flessibili si deflettono - **questo è normale**

#### Test 3: Simulation with locked joints
- **Verifica**: Drift durante 300 step (5s @ 60Hz) con FixedJoint
- **Atteso**: 0.000 mm drift
- **Significato**: Joint completamente rigidi mantengono geometria perfettamente

**Output atteso**:
```
====== ISAAC SIM INTEGRATION TEST SUITE ======
Test 1: Geometry after stage open (no simulation)
  ✅ PASS: All positions within tolerance (0.000 mm)

Test 2: Geometry after world reset (PhysX initialized)
  ❌ FAIL: Max error 8.721mm (flexible joints deflect - EXPECTED)

Test 3: Simulation with locked joints (300 steps @ 60Hz)
  Initial vs Final positions (drift check):
    Trunk links:  max drift 0.000 mm
    Branch links: max drift 0.000 mm
  ✅ PASS: All positions stable (no drift)

======== FINAL REPORT ========
✅ CRITICAL TESTS PASSED
VERDICT: USD geometry is consistent in Isaac Sim.
         Locked joints maintain positions during simulation.
```

**Note importanti**:
- Il Test 2 **fallisce per design** - dimostra che joint flessibili si comportano correttamente sotto gravità
- Il Test 3 è quello critico - dimostra zero drift con joint rigidi

---

## Interpretazione Risultati

### Tutti i test analitici passano (test_geometric_consistency.py)
✅ **Geometria USD corretta** - Le posizioni dei link matchano i calcoli analitici

### Tutti gli error test passano (test_error_handling.py)
✅ **Validazione robusta** - Configurazioni invalide vengono respinte con messaggi chiari

### Test Isaac Sim: Test 1 e 3 passano, Test 2 fallisce
✅ **Comportamento corretto**:
- USD carica correttamente in Isaac Sim
- Joint flessibili si deflettono sotto gravità (atteso)
- Joint rigidi mantengono geometria (0 drift)

---

## Troubleshooting

### Test analitici falliscono con "ModuleNotFoundError: No module named 'pxr'"
**Soluzione**: Usa `uv run` invece di `python` direttamente:
```bash
uv run src/experiments/recursive_tree/test_geometric_consistency.py
```

### Test Isaac Sim non mostra output
**Causa**: Isaac Sim è molto verbose, l'output viene nascosto  
**Soluzione**: Il test usa automatic flushing, l'output dovrebbe apparire. Verifica con:
```bash
~/isaacsim/python.sh test_isaac_sim_integration.py 2>&1 | grep -E "(Test|PASS|FAIL)"
```

### Test Isaac Sim fallisce con "command not found: ~/isaacsim/python.sh"
**Soluzione**: Verifica il path corretto di Isaac Sim:
```bash
which isaacsim  # o controlla dove hai installato Isaac Sim
```

---

## Esecuzione Completa

Per eseguire l'intera test suite:

```bash
cd /path/to/autotom_digital_twin

echo "=== Test 1: Geometric Consistency ==="
uv run src/experiments/recursive_tree/test_geometric_consistency.py

echo -e "\n=== Test 2: Error Handling ==="
uv run src/experiments/recursive_tree/test_error_handling.py

echo -e "\n=== Test 3: Isaac Sim Integration ==="
~/isaacsim/python.sh src/experiments/recursive_tree/test_isaac_sim_integration.py
```

**Tempo totale stimato**: ~2 minuti

---

## Summary

| Test Suite | Cosa verifica | Requisiti | Runtime |
|------------|---------------|-----------|---------|
| Geometric Consistency | Posizioni USD vs analitico | `uv` | ~10s |
| Error Handling | Validazione config invalide | `uv` | <1s |
| Isaac Sim Integration | Geometria in Isaac Sim + locked joints | Isaac Sim | ~60-90s |

**Risultati attesi**:
- Geometric: 9/9 pass, max error 0.000mm
- Error Handling: 8/8 pass
- Isaac Sim: 2/3 pass (Test 2 fallisce per design)

---

## Future Extensions

**Test parametrici** (pianificati ma non implementati):
- 50+ configurazioni random valide
- Verifiche statistiche su stabilità numerica

**Test PhysX stability** (pianificati ma non implementati):
- Simulazioni più lunghe (30s-60s) con joint flessibili
- Verifica convergenza sotto oscillazioni

**Test invalid configs con Isaac Sim** (pianificati ma non implementati):
- Verifica che Isaac Sim gestisca correttamente config limite (es. 64 links esatti)
