# Tre Test Lateral Branches - Riepilogo Completo

## ✅ Tutti i Test Generati con Successo!

### Test 1: Lateral Branches Antenna (BASELINE)
**File**: `lateral_branches_antenna.usda` (73 KB)  
**Total**: **27 links**

```
Struttura:
  • Trunk: 5 links verticali
  • 1 Main branch inclinato 50°
  • 8 Rametti laterali (pattern simmetrico 0°/180°)
  
Caratteristiche:
  • Test di base per confronto
  • Rami inclinati (non orizzontali)
  • Pattern semplice
```

---

### Test 2: Multi-Branch Horizontal (LIMITE PHYSX)
**File**: `multi_branch_horizontal.usda` (176 KB)  
**Total**: **64 links** ✓ (esattamente al limite!)

```
Struttura:
  • Trunk: 7 links verticali
  • 3 Main branches ORIZZONTALI (tilt=90°)
    - Distribuzione radiale: 0°, 120°, 240°
    - 7 links per branch
    - Radius: 1cm (100mm world)
  • 18 Rametti laterali INTERSECATI
    - 6 per branch
    - Radius: 1mm (10mm world)
    - Pattern alternato: 90°/270°
  
Caratteristiche:
  • ESATTAMENTE al limite PhysX (64 links)
  • Pattern intersecato ottimizzato
  • Distribuzione uniforme
```

---

### Test 3: Complex Multi-Branch Heavy ⭐ NUOVO!
**File**: `complex_multi_branch_heavy.usda` (269 KB)  
**Total**: **98 links** 🔥 (oltre il limite!)

```
Struttura:
  • Trunk: 10 links verticali (PIÙ ALTO)
  • 4 Main branches ORIZZONTALI (tilt=90°)
    - Distribuzione radiale COMPLETA: 0°, 90°, 180°, 270°
    - 8 links per branch (PIÙ LUNGHI)
    - Radius: 12mm = 120mm world (PIÙ ROBUSTI)
  • 28 Rametti laterali INTERSECATI
    - 7 per branch (distribuzione uniforme)
    - Radius: 1mm (10mm world)
    - Pattern alternato: 90°/270°
  
Caratteristiche:
  • 98 links - HEAVY TEST!
  • Usa skip_limit_check=True
  • 4 rami invece di 3 (radiale completo)
  • Rami più lunghi (8 vs 7 links)
  • Trunk più alto (10 vs 7 links)
  • Rami più robusti (12mm vs 10mm)
```

## 📊 Tabella Comparativa

| Parametro | Test 1 | Test 2 | Test 3 |
|-----------|--------|--------|--------|
| **Total links** | 27 | 64 | **98** 🔥 |
| **Trunk links** | 5 | 7 | 10 |
| **Trunk radius** | 100mm | 120mm | 150mm |
| **Main branches** | 1 | 3 | 4 |
| **Links/branch** | 6 | 7 | 8 |
| **Branch radius** | 40mm | 100mm | **120mm** |
| **Subbranches** | 8 | 18 | 28 |
| **Pattern** | Simmetrico | Intersecato | Intersecato |
| **Orientamento** | Inclinato 50° | Orizzontale 90° | Orizzontale 90° |
| **File size** | 73 KB | 176 KB | 269 KB |
| **Status PhysX** | ✅ OK | ✅ Limite | ⚠️ Oltre |

## 🎯 Progressione della Complessità

```
Test 1 (27 links)
  │
  ├─ Pattern semplice
  ├─ 1 ramo inclinato
  └─ Baseline di riferimento

Test 2 (64 links)
  │
  ├─ Pattern complesso intersecato
  ├─ 3 rami orizzontali
  ├─ Al limite PhysX
  └─ Ottimizzato per 64 links

Test 3 (98 links)  ⭐
  │
  ├─ Pattern molto complesso
  ├─ 4 rami orizzontali (radiale completo)
  ├─ Rami PIÙ LUNGHI e PIÙ ROBUSTI
  ├─ OLTRE il limite PhysX
  └─ Heavy stress test!
```

## 🚀 Test in Isaac Sim

```bash
cd ~/isaacsim/autotom_digital_twin
python3 src/experiments/recursive_tree/tests/test_manual_cli.py
```

### Ordine di Test Consigliato:

1. **Test 1** (27 links) - Verifica baseline
2. **Test 2** (64 links) - Verifica limite PhysX
3. **Test 3** (98 links) - Verifica stress test

### Aspettative per Test 3 (Heavy):

**✅ STABLE (ottimale)**
- Rami più robusti (12mm) compensano la complessità
- 4 rami radiali ben bilanciati
- Droop controllato (< 200mm)
- Stabilizzazione entro 10 secondi

**⚠️ MARGINAL (accettabile)**
- Droop moderato (200-300mm)
- Oscillazioni che si smorzano (10-15 sec)
- Possibili interazioni tra rami adiacenti
- Raggiunge equilibrio stabile

**❌ UNSTABLE (problematico)**
- 98 links troppo per il solver
- Oscillazioni persistenti
- Collisioni multiple
- Instabilità numerica
- FPS basso (< 20)

### Cosa Osservare (Test 3):

**Performance**
- [ ] FPS accettabile con 98 links? (> 20 FPS)
- [ ] Tempo di caricamento ragionevole?
- [ ] Warning PhysX nel console?

**Stabilità Strutturale**
- [ ] Trunk alto (10 links) stabile?
- [ ] 4 rami radiali ben bilanciati?
- [ ] Rami più lunghi (8 links) stabili?
- [ ] Rametti intersecati senza collisioni?

**Convergenza**
- [ ] Tempo settling < 15 secondi?
- [ ] Droop totale < 300mm?
- [ ] Oscillazioni si smorzano?
- [ ] Raggiunge equilibrio statico?

## ⚠️ Note Importanti

### Test 3 - Superamento Limite PhysX

Il Test 3 con 98 links **supera il limite PhysX di 64 links**:

- Usa `skip_limit_check=True` per bypassare la validazione
- PhysX **supporta fino a 64 links per articulation**
- **POTREBBE** funzionare comunque (PhysX può gestirlo)
- **POTREBBE** causare instabilità o crash
- **POTREBBE** avere performance ridotte

### Se il Test 3 Fallisce:

Opzioni di riduzione:
1. **Ridurre a 3 branches**: 10 + 24 + 42 = 76 links (ancora sopra)
2. **Branches con 7 links**: 10 + 28 + 42 = 80 links (ancora sopra)
3. **Trunk 8 links**: 8 + 32 + 56 = 96 links (quasi uguale)

**Per restare sotto 64**:
- Trunk 7 + 3 branches × 7 + 6 subs × 2 = 64 ✓ (Test 2)

Il Test 3 è **intenzionalmente oltre il limite** per testare i limiti di Isaac Sim!

## 📁 File Generati

- `lateral_branches_antenna.usda` (73 KB) - Test 1
- `multi_branch_horizontal.usda` (176 KB) - Test 2
- `complex_multi_branch_heavy.usda` (269 KB) - Test 3 ⭐

## 🔄 Rigenerare

```bash
cd ~/isaacsim/autotom_digital_twin
uv run src/experiments/recursive_tree/tests/test_lateral_branches.py
```

Genera tutti e tre i test in una singola esecuzione!

---

**Status**: ✅✅✅ TUTTI I TEST PRONTI  
**Created**: 2026-08-04  
**Test 1**: 27 links (baseline)  
**Test 2**: 64 links (limite PhysX)  
**Test 3**: 98 links (heavy stress test) ⭐
