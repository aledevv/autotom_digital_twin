# Riepilogo Test Lateral Branches

## Due Configurazioni Generate

### Test 1: Lateral Branches Antenna
**File**: `lateral_branches_antenna.usda` (74 KB)

```
Struttura:
  Trunk: 5 links verticali
  Main branch: 6 links inclinati 50°
  Lateral subbranches: 8 rametti (2 per link ai links 2,3,4,5)
  
  Total: 27 links
```

**Caratteristiche:**
- Rametti inclinati 35° rispetto al main branch
- Pattern simmetrico: rot 0° / 180° (destra/sinistra)
- Radius rametti: 2mm → 20mm world
- Test base per antenna style

---

### Test 2: Multi-Branch Horizontal  
**File**: `multi_branch_horizontal.usda` (148 KB)

```
Struttura:
  Trunk: 10 links verticali
  Main branches: 4 rami ORIZZONTALI (tilt=90°)
    - Attaccati ai link 3, 5, 7, 9 del trunk
    - Rotazioni: 0°, 90°, 180°, 270° (distribuzione radiale)
    - 3 links per branch
  Lateral subbranches: 16 rametti ORIZZONTALI (4 per branch)
    - 2 attach points per branch (link 2, 3)
    - 2 rametti per attach point
    - 2 links per rametto
  
  Total: 54 links (10 + 12 + 32)
```

**Caratteristiche CHIAVE:**
- ✅ **Rami orizzontali**: tilt=90° (paralleli al terreno)
- ✅ **Rametti ruotati 90°**: tilt=0° + rot=90°/270°
  - I rametti seguono la direzione del parent (orizzontale)
  - MA sono ruotati di 90°/270° attorno all'asse del parent
  - Risultato: si espandono LATERALMENTE rispetto al ramo
  - Mantengono orientamento orizzontale (parallelo al terreno)
- ✅ **Radius rametti: 10mm → 100mm world (1cm come richiesto!)**
- ✅ **4 rami principali** (distribuzione radiale completa)
- ✅ **Pattern più pesante**: 54 links vs 27 del Test 1

## Geometria Rotazioni

### Come funziona la rotazione dei rametti laterali:

```
Vista laterale di un branch orizzontale:

     Trunk
       |
       |
   ----●----> Branch (orizzontale, tilt=90°, si espande verso destra)
       |
       |
    
Senza rotazione laterale (rot=0°):
   I rametti seguirebbero la stessa direzione del branch (verso destra)
   
Con rotazione laterale (rot=90°/270°):
   I rametti sono ruotati ATTORNO all'asse del branch
   
Vista dall'alto del branch:
   
              ↑ sub (rot=90°)
              |
    ←---------●--------→  Branch (orizzontale)
              |
              ↓ sub (rot=270°)
              
I rametti si espandono PERPENDICOLARMENTE al branch,
ma restano paralleli al terreno!
```

### Parametri chiave:
- **Branch**: `tilt=90°` (diventa orizzontale rispetto al trunk verticale)
- **Subbranch**: `tilt=0°` (segue direzione parent, quindi orizzontale)
- **Subbranch**: `rot=90°/270°` (ruotato attorno asse parent, espansione laterale)

## Tabella Comparativa

| Parametro | Test 1 (Antenna) | Test 2 (Horizontal) |
|-----------|------------------|---------------------|
| **Total links** | 27 | 54 |
| **Trunk links** | 5 | 10 |
| **Main branches** | 1 | 4 |
| **Links/branch** | 6 | 3 |
| **Subbranches** | 8 | 16 |
| **Links/sub** | 2 | 2 |
| **Branch tilt** | 50° | 90° (orizzontale) |
| **Sub tilt** | 35° | 0° (segue parent) |
| **Sub rot** | 0°/180° | 90°/270° ✓ |
| **Sub radius** | 20mm world | **100mm world (1cm)** ✓ |
| **Sub height** | 150mm world | 150mm world |
| **Pattern** | Lineare | Radiale |

## Test in Isaac Sim

```bash
cd ~/isaacsim/autotom_digital_twin
python3 src/experiments/recursive_tree/tests/test_manual_cli.py
```

### Per Test 1 (Antenna) - Osserva:
- [ ] Rametti si stabilizzano rapidamente?
- [ ] Nessuna oscillazione laterale?
- [ ] Pattern antenna mantiene forma?

### Per Test 2 (Horizontal) - Osserva:
- [ ] **Rami orizzontali mantengono orientamento?**
- [ ] **Rametti si espandono lateralmente (perpendicolari al ramo)?**
- [ ] **Droop verticale accettabile? (< 200mm)**
- [ ] **Pattern radiale bilanciato?**
- [ ] **Rametti più spessi (1cm) stabili?**
- [ ] Tempo di settling < 10 secondi?

## Aspettative Test 2 (Horizontal)

### ✅ STABLE (ideale)
- Rami orizzontali con droop moderato (50-100mm)
- Rametti laterali espansi perpendicolarmente
- Stabilizzazione rapida (< 5 secondi)
- Pattern radiale ben bilanciato

### ⚠️ MARGINAL (accettabile)
- Droop più marcato (100-200mm) ma stabile
- Oscillazioni verticali che si smorzano (< 10 sec)
- Rametti più spessi mostrano leggera flessione
- Raggiunge equilibrio stabile

### ❌ UNSTABLE (problematico)
- Droop eccessivo (rami toccano terreno)
- Oscillazioni persistenti
- Collisioni tra rami/rametti
- Instabilità numerica (jitter continuo)

## Sfide Specifiche Test 2

1. **Gravità perpendicolare ai rami**
   - Rami orizzontali subiscono massimo stress
   - Droop inevitabile ma deve essere controllato

2. **Rametti più spessi (1cm)**
   - Più massa → più inerzia
   - Potrebbe aiutare stabilità O causare più droop
   - Test interessante per trovare sweet spot

3. **Pattern radiale con 4 rami**
   - Possibili interazioni tra rami adiacenti
   - Bilanciamento carichi importante

4. **54 links vicino al limite**
   - Solo 10 links di margine rispetto al limite 64
   - Più stress sul solver

## File Generati

- **Script**: `test_lateral_branches.py`
- **USD Test 1**: `scalability_usds/lateral_branches_antenna.usda` (74 KB)
- **USD Test 2**: `scalability_usds/multi_branch_horizontal.usda` (148 KB)
- **Test runner**: `test_manual_cli.py`
- **Documentazione**: 
  - `LATERAL_BRANCHES_README.md`
  - `MULTI_BRANCH_HORIZONTAL_README.md`
  - `QUICK_START_LATERAL_TEST.md`
  - Questo file: `SUMMARY_LATERAL_TESTS.md`

## Rigenerare i Test

```bash
cd ~/isaacsim/autotom_digital_twin
uv run src/experiments/recursive_tree/tests/test_lateral_branches.py
```

Output: entrambi i file USD saranno rigenerati.

---

**Status**: ✅ Pronto per test Isaac Sim  
**Created**: 2026-08-04  
**Updated**: 2026-08-04 08:37 (fix rotazioni rametti + aumentato radius a 1cm + aggiunti rami)
