# Configurazione Finale - Test Lateral Branches

## ✅ Test Completati e Pronti

### Test 1: Lateral Branches Antenna
**File**: `lateral_branches_antenna.usda` (74 KB)  
**Status**: ✅ Invariato (configurazione base di riferimento)

```
Struttura:
  • Trunk: 5 links verticali
  • 1 Main branch: 6 links inclinati 50°
  • 8 Rametti laterali (2 per link)
  
  Total: 27 links
```

---

### Test 2: Multi-Branch Horizontal ⭐ CONFIGURAZIONE FINALE
**File**: `multi_branch_horizontal.usda` (175 KB)  
**Status**: ✅ Aggiornato con tutte le modifiche richieste

```
Struttura:
  • Trunk: 7 links verticali
  • 3 Main branches ORIZZONTALI (tilt=90°)
    - Distribuzione radiale: 0°, 120°, 240°
    - 7 links per branch
    - Radius: 1cm (10mm → 100mm world) ✓
  • 18 Rametti laterali (6 per branch)
    - Radius: 1mm (0.001m → 10mm world) ✓
    - Pattern INTERSECATO (lati alternati)
  
  Total: 64 links ✓✓✓ (ESATTAMENTE al limite PhysX!)
```

## 🔧 Modifiche Applicate (Test 2)

### ✅ 1. Radius Corretto
- **Rami**: 1cm (0.010m → 100mm world scale)
- **Rametti**: 1mm (0.001m → 10mm world scale)
  - Corretto per GLOBAL_SCALE=10!

### ✅ 2. Rametti Intersecati
Pattern alternato tra i link del ramo:

```
Link 2: side1 (rot=90°)     ←|
Link 3: side2 (rot=270°)      |→ INTERSECATI
Link 4: side1 (rot=90°)     ←|
Link 5: side2 (rot=270°)      |→ INTERSECATI
Link 6: side1 (rot=90°)     ←|
Link 7: side2 (rot=270°)      → INTERSECATI
```

Vista dall'alto di un branch:

```
        ↑ L2 (90°)
        |
←-------●-------● L3 (270°, lato opposto)
        |       |
     L4 ●       ● INTERSECATI!
        |       |
        ●-------● L5 (270°)
      L6 |
         ↓ L7 (270°)
```

### ✅ 3. Distribuzione Uniforme
- 6 rametti su 7 links (link 2-7, non link 1 perché è l'attacco al trunk)
- Distribuzione uniforme lungo tutto il ramo
- Pattern intersecato per massima copertura spaziale

### ✅ 4. Ottimizzazione Numero Links
- Ridotto a **esattamente 64 links** (limite PhysX)
- 3 branches invece di 4 (per avere 7 links ciascuno)
- 6 rametti per branch (distribuzione ottimale)

## 📊 Tabella Comparativa Finale

| Parametro | Test 1 (Antenna) | Test 2 (Horizontal) |
|-----------|------------------|---------------------|
| **Total links** | 27 | **64** ✓ |
| **Trunk links** | 5 | 7 |
| **Main branches** | 1 | 3 |
| **Links/branch** | 6 | 7 |
| **Branch radius** | 40mm world | **100mm world (1cm)** ✓ |
| **Subbranches** | 8 | 18 |
| **Sub radius** | 20mm world | **10mm world (1mm)** ✓ |
| **Branch tilt** | 50° | 90° (orizzontale) |
| **Sub pattern** | 0°/180° | **90°/270° intersecati** ✓ |
| **Distribution** | Lineare | Radiale (120°) |

## 🎯 Geometria Rotazioni Intersecate

### Come funziona il pattern intersecato:

```
Vista laterale del branch orizzontale con rametti intersecati:

              ↑ L2 (rot=90°)
              |
    ←---------●---------●---------●---------→  Branch (orizzontale)
         L3   |    L4   |    L5   |    L7
              ↓         ↑         ↓         ↓
           (270°)     (90°)    (270°)    (270°)
           
I rametti alternano tra side1 (90°) e side2 (270°),
creando un pattern INTERSECATO lungo il ramo.
```

### Parametri chiave:
- **Branch**: `tilt=90°` → orizzontale rispetto al trunk verticale
- **Subbranch**: `tilt=0°` → segue direzione parent (orizzontale)
- **Subbranch**: `rot=90°/270° ALTERNATI` → intersecati tra i link

## 🚀 Test in Isaac Sim

```bash
cd ~/isaacsim/autotom_digital_twin
python3 src/experiments/recursive_tree/tests/test_manual_cli.py
```

### Cosa Osservare (Test 2)

**Pattern Intersecato**
- [ ] Rametti si alternano tra i lati? (visibile intersecazione)
- [ ] Distribuzione uniforme lungo i rami?
- [ ] Pattern simmetrico tra i 3 rami principali?

**Orientamento Orizzontale**
- [ ] Rami principali orizzontali (paralleli al terreno)?
- [ ] Rametti si espandono perpendicolarmente ai rami?
- [ ] Mantengono orientamento orizzontale?

**Stabilità e Convergenza**
- [ ] Droop verticale accettabile? (< 200mm)
- [ ] Tempo di settling < 10 secondi?
- [ ] Nessuna oscillazione persistente?
- [ ] Pattern radiale bilanciato?

**Performance**
- [ ] 64 links gestibili dal solver?
- [ ] FPS > 30?
- [ ] Nessun warning PhysX?

## 📈 Aspettative

### ✅ STABLE (ideale)
- Rami orizzontali con droop controllato (< 150mm)
- Rametti intersecati mantengono pattern
- Stabilizzazione rapida (< 5 sec)
- Pattern radiale ben bilanciato

### ⚠️ MARGINAL (accettabile)
- Droop moderato (150-250mm)
- Leggere oscillazioni che si smorzano (< 10 sec)
- Pattern intersecato leggermente deformato
- Raggiunge equilibrio stabile

### ❌ UNSTABLE (problematico)
- Droop eccessivo (> 300mm o toccano terreno)
- Oscillazioni persistenti
- Collisioni tra rametti
- Instabilità numerica (64 links al limite!)

## 🔍 Sfide Specifiche

1. **64 Links = Limite Esatto**
   - Massimo stress sul solver PhysX
   - Nessun margine di errore
   - Potrebbe richiedere solver iterations elevate

2. **Pattern Intersecato**
   - Rametti vicini tra loro
   - Possibili interazioni/collisioni
   - Collision filtering critico

3. **Rametti Sottili (1mm)**
   - Molto flessibili (radius ridotto)
   - Potrebbero oscillare facilmente
   - Ma massa ridotta → meno inerzia

4. **Rami Spessi (1cm)**
   - Più massa → più droop
   - Ma più rigidi → meno oscillazioni
   - Balance critico

## 📁 File Generati

- **Script**: `test_lateral_branches.py`
- **USD Test 1**: `lateral_branches_antenna.usda` (74 KB)
- **USD Test 2**: `multi_branch_horizontal.usda` (175 KB)
- **Documentazione**:
  - `SUMMARY_LATERAL_TESTS.md`
  - `MULTI_BRANCH_HORIZONTAL_README.md`
  - `LATERAL_BRANCHES_README.md`
  - Questo file: `FINAL_LATERAL_CONFIG.md` ⭐

## 🔄 Rigenerare

```bash
cd ~/isaacsim/autotom_digital_twin
uv run src/experiments/recursive_tree/tests/test_lateral_branches.py
```

Output: entrambi i file USD saranno rigenerati con le configurazioni corrette.

---

**Status**: ✅✅✅ PRONTO PER TEST ISAAC SIM  
**Total Links Test 2**: **64** (limite esatto)  
**Created**: 2026-08-04  
**Updated**: 2026-08-04 08:45  
**Changes**:
- ✅ Radius rametti: 1mm (corretto per GLOBAL_SCALE)
- ✅ Radius rami: 1cm
- ✅ Pattern intersecato: lati alternati 90°/270°
- ✅ Distribuzione uniforme: 6 rametti su 7 links
- ✅ Ottimizzato: esattamente 64 links
