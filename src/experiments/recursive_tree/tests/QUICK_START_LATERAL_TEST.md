# Quick Start - Test Lateral Branches (DUE CONFIGURAZIONI)

## 🎯 Obiettivo

Testare DUE configurazioni ad albero con **rametti laterali**:

1. **Lateral Antenna** - 1 ramo con rametti inclinati (27 links)
2. **Multi-Branch Horizontal** - 3 rami orizzontali con rametti (56 links)

## 📋 Struttura

### TEST 1: Lateral Branches Antenna (27 links)

```
        Trunk (5 links)
             |
        Link 3
            /
     main_branch (6 links, inclinato 50°)
          /
  Link 2: ←-●-→  (2 rametti inclinati 35°)
        /
  Link 3: ←-●-→  (2 rametti)
        /
  Link 4: ←-●-→  (2 rametti)
        /
  Link 5: ←-●-→  (2 rametti)
        /
      Link 6
```

**Total: 27 links** (5 trunk + 6 branch + 16 laterali)

### TEST 2: Multi-Branch Horizontal (56 links)

```
Vista dall'alto:
         Branch_3
              \
               \
                Trunk
               /    \
              /      \
        Branch_1    Branch_2

Ogni branch: ORIZZONTALE (tilt=90°, parallelo al terreno)
Ogni branch ha 6 rametti ORIZZONTALI ai lati
```

**Total: 56 links** (8 trunk + 12 branches + 36 laterali)  
**Dimensioni: 25-33% più piccole del Test 1**

## ⚡ Quick Test

### 1. Entrambi i file USD sono già stati generati! ✅

```bash
scalability_usds/lateral_branches_antenna.usda      (~74 KB)  - Test 1
scalability_usds/multi_branch_horizontal.usda       (~154 KB) - Test 2
```

### 2. Testa in Isaac Sim

```bash
cd ~/isaacsim/autotom_digital_twin
python3 src/experiments/recursive_tree/tests/test_manual_cli.py
```

### 3. Workflow (per OGNI test)

1. ✅ Il test caricherà automaticamente ciascun USD
2. 🎮 Premi **PLAY** in Isaac Sim
3. 👀 Osserva per 5-10 secondi
4. ❌ **CHIUDI** Isaac Sim
5. 📝 Classifica nel terminal

## 🔍 Cosa Osservare

### TEST 1: Lateral Branches Antenna

**✅ Comportamento Ideale (STABLE)**
- [ ] Rametti si stabilizzano in < 2 secondi
- [ ] Nessuna oscillazione visibile
- [ ] Struttura mantiene forma "antenna"

**⚠️ Comportamento Accettabile (MARGINAL)**
- [ ] Lievi oscillazioni che si smorzano (< 5 sec)
- [ ] Piccoli movimenti pendolari residui

**❌ Comportamento Problematico (UNSTABLE)**
- [ ] Oscillazioni continue o crescenti
- [ ] Collisioni tra rametti

---

### TEST 2: Multi-Branch Horizontal

**✅ Comportamento Ideale (STABLE)**
- [ ] Rami orizzontali si stabilizzano rapidamente
- [ ] Droop verticale accettabile (< 100mm)
- [ ] Pattern radiale ben bilanciato

**⚠️ Comportamento Accettabile (MARGINAL)**
- [ ] Droop moderato (normale per gravità)
- [ ] Oscillazioni verticali si smorzano (< 10 sec)
- [ ] Rametti flessibili con leggero movimento

**❌ Comportamento Problematico (UNSTABLE)**
- [ ] Droop eccessivo (toccano terreno)
- [ ] Oscillazioni verticali persistenti
- [ ] Collisioni tra rami/rametti

## 📊 Caratteristiche del Test

| Parametro | Valore | Note |
|-----------|--------|------|
| **Total links** | 27 | Sotto limite (64) |
| **Max L/D** | 7.5 | Subbranches flessibili |
| **Attachments** | 8 | Multipli attaccamenti |
| **Pattern** | Simmetrico | Rot 0° e 180° |
| **Complexity** | Medium | Test esplorativo |

## 🔧 Se Serve Rigenerare

```bash
cd ~/isaacsim/autotom_digital_twin
uv run src/experiments/recursive_tree/tests/test_lateral_branches.py
```

Output: `scalability_usds/lateral_branches_antenna.usda`

## 📚 Documentazione Completa

Per dettagli approfonditi, vedi:
- `LATERAL_BRANCHES_README.md` - Documentazione completa
- `test_lateral_branches.py` - Script generatore
- `test_manual_cli.py` - Test runner interattivo

## 🎯 Confronto con Altri Test

| Test | Links | L/D | Status |
|------|-------|-----|--------|
| baseline_tomato | 41 | 5.87 | Da testare |
| **lateral_antenna** | **27** | **7.5** | **⏳ Nuovo** |
| petiolule_ld_10 | 41 | 10.0 | Da testare |

**Vantaggi**: Meno links, pattern simmetrico, test specifico laterali  
**Sfide**: L/D alto (7.5), 8 attachment simultanei

---

**Ready to test!** 🚀  
Esegui `test_manual_cli.py` per iniziare il test in Isaac Sim.
