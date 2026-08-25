# Resoconto del branch GroIMP e limite della fisica dei frutti

Data di chiusura: 2026-08-25

Branch: `groPy`

Stato: prototipo GroIMP -> PlantState -> USD consolidato; Phase J
`PARTIALLY COMPLETED — FRUIT PHYSICS UNSUPPORTED`.

## Scopo e confine del risultato

Questo branch dimostra una pipeline riproducibile che estrae la pianta nativa
da GroIMP, ne conserva topologia e pose in un formato indipendente e alimenta
gli exporter USD. Il risultato ha due livelli distinti:

1. il builder USD V1/V2 è infrastruttura generale e riutilizzabile;
2. l'adapter PlantState e l'attuale `tree_config` descrivono questa specifica
   pianta di pomodoro GroIMP, usata come base sperimentale e non come modello
   definitivo del progetto Autotom.

Il branch non è destinato al merge in `main` in questa fase. La versione V2 di
`main`, più semplice, rimane il riferimento stabile con frutti. Qui il percorso
supportato si ferma ai rachidi e pedicelli dei truss; la fisica dei pomodori è
conservata solo come esperimento esplicito.

## Architettura implementata

```text
GroIMP ProjectGraph
        |
        v
Inspector raw (groimp_inspection/1.0)
        |
        +--> Turtle resolver e validazione contro location()/direction()/OBJ
        |
        v
Canonical extractor --> PlantState (plant_state/1.0) --> JSON deterministico
                                                    |--> ExporterV1 statico
                                                    `--> adapter BRANCHES
                                                         --> backend V2 storico
```

Le API principali introdotte nelle Phase A-F sono:

- `inspect_project(...)` e `inspect_workbench(...)`;
- `resolve_turtle(...)`;
- `build_rendered_geometry(...)` e `validate_rendered_geometry(...)`;
- `compare_representations(...)` e `validate_project(...)`;
- `extract_plant_state(...)`, `extract_workbench_state(...)` e
  `extract_project_state(...)`;
- `save_plant_state(...)`, `load_plant_state(...)`,
  `validate_plant_state(...)` e `plant_states_equivalent(...)`.

L'Inspector conserva il ProjectGraph, i codici originali degli archi, gli
organi, `RH`/`RL`/`RU`/`RG`/`Translate`, gli anchor GroIMP e la diagnostica. Il
resolver usa frame local-to-world 4x4, vettori colonna e composizione
`world @ local`. PlantState conserva il sottografo biologico della singola
pianta, attributi originali e normalizzati, pose, assi, sfere e provenienza.

## Stato delle fasi

| Fase | Stato | Risultato |
|---|---|---|
| A | `COMPLETED` | Inspector nativo e lifecycle GroPy sicuro |
| B | `COMPLETED` | Turtle resolver condiviso e validato |
| C | `COMPLETED` | Ricostruzione/validazione della geometria renderizzata |
| D | `COMPLETED` | Confronto nativo, CSV e exporter storici |
| E | `COMPLETED` | Modello canonico ed estrattore PlantState |
| F | `COMPLETED` | Persistenza JSON strict e deterministica |
| G | `SKIPPED BY DESIGN` | Nessun adapter CSV temporaneo |
| H | `COMPLETED` | ExporterV1 migrato a PlantState |
| I | `COMPLETED` | Parità e workflow V1 verificati |
| J | `PARTIALLY COMPLETED — FRUIT PHYSICS UNSUPPORTED` | V2 valido fino ai supporti dei truss; frutti fisici sperimentali |

## Estrazione batch e replay offline

Una singola giornata o un intervallo consecutivo si estraggono senza riaprire
continuamente il workbench:

```bash
./extract_plant_states.sh --day 50
./extract_plant_states.sh --from-day 1 --to-day 160 --skip-existing
```

Il runtime copia GSZ e input sotto `/tmp`, verifica il giorno osservato, salva
ogni JSON atomicamente e chiude sempre il workbench. Questo evita il noto
errore di stato/reset che compariva aprendo un progetto GroIMP non ripristinato
o riutilizzando in modo improprio il workbench. La procedura dettagliata è in
`docs/GROIMP_BATCH_EXTRACTION.md`.

I PlantState versionati permettono in seguito di generare USD senza GroIMP e
senza CSV. Gli ID nativi restano affidabili nello stesso workbench; non viene
dichiarata stabilità degli ID fra simulazioni GroIMP indipendenti.

## Migrazione degli exporter

V1 consuma PlantState, conserva la resa grafica statica storica e usa topologia
e ID canonici. Gli organi sovrapposti non vengono eliminati salvo duplicati
esatti per tipo, parentela, attributi e posa. Il workflow interattivo rimane:

```bash
./run_mainV1.sh --day 25 --isaacsim
```

V2 riusa il backend storico `skinned` in modalità `segmented`, non un nuovo
renderer a cilindri. Mantiene mesh organiche, capsule invisibili, gerarchia
leggibile e ID GroIMP. Le pose canoniche determinano stem e laterali; gli
appendici possono usare la posa estetica V2 preservando separatamente la posa
GroIMP sorgente. Petioluli e lamine sono visuali solidali per contenere il
costo; piccioli, rachidi fogliari, laterali, rachidi dei truss e pedicelli
restano articolati.

## Audit fruit-free

Il profilo supportato è `truss-supports`, con preset `flexible`, laterali
dinamici e nessuna sfera di frutto authored. Gli audit serverless del
2026-08-25 hanno prodotto:

| Giorno | Corpi | D6 | Fixed | Capsule | Sfere frutto |
|---:|---:|---:|---:|---:|---:|
| 1 | 9 | 6 | 3 | 18 | 0 |
| 25 | 51 | 43 | 8 | 102 | 0 |
| 50 | 133 | 123 | 10 | 266 | 0 |
| 80 | 216 | 206 | 10 | 432 | 0 |
| 160 | 216 | 206 | 10 | 432 | 0 |

Tutti gli audit hanno riportato zero errori. Il day 160 fruit-free ha inoltre
superato 5 secondi simulati headless a 480 Hz senza NaN/Inf, snapping, drift
dello stem o invalidazione dell'articolazione.

I comandi normali sono quindi:

```bash
uv run python -m exporterV2 --day 160
./run_mainV2.sh --day 160
./run_debugV2.sh --day 160 --headless --duration 5
```

`run_mainV2.sh` salva normalmente in
`data/usd_models/tree_v2_day_N.usda`; `run_debugV2.sh` scrive sotto `/tmp`.
`fruit-visual` resta disponibile per visualizzare pomodori statici senza
rigid body, collider o giunti terminali.

## Limite scientifico dei pomodori fisici al day 160

Il sistema completo candidato contiene 288 corpi: i 216 supporti vegetativi e
72 pomodori esterni all'articolazione, collegati da 72 FixedJoint terminali.
Il V2 tracciato su `main` contiene invece 265 corpi e 40 pomodori. Il candidato
ha 1716 relazioni di collisione filtrate contro 1248 nel legacy.

È stata verificata esplicitamente l'ipotesi che l'instabilità fosse causata da
collisioni residue. Nel test A/B:

- tutti i 504 collider del candidato sono stati disabilitati soltanto nella
  copia in memoria;
- la break force è stata portata a `1e9 N`;
- `excludeFromArticulation=true` è rimasto invariato, per non cambiare la
  topologia fisica del test;
- masse, 288 corpi e giunti sono rimasti identici al caso completo.

Dopo 12 secondi simulati il sistema non si è assestato: velocità lineare finale
`0.1128 m/s`, velocità angolare finale `4.613 rad/s`, picco angolare
`10.934 rad/s`. L'USD su disco è rimasto invariato.

Questo falsifica, per il test eseguito, l'ipotesi che collisioni non filtrate
siano la causa primaria. Il limite osservato riguarda la scala del carico e la
dinamica dei 72 FixedJoint terminali esterni. Non dimostra un difetto generale
del builder USD, né che ogni possibile modello di frutto sia instabile; indica
che questa configurazione matura specifica non ha superato i criteri dichiarati
e non deve essere presentata come supportata.

Il percorso sperimentale mantiene il comportamento storico esplicito:
pomodori esterni, `excludeFromArticulation=true` e break force `6 N`. È
accessibile solo con:

```bash
./run_mainV2.sh --day 160 --debug-profile full \
  --allow-experimental-fruit-physics
```

Il comando emette un warning forte e il manifest riporta
`fruit_physics_support_status: unsupported_experimental`. Non esegue più
soft-start, gravity ramp, cattura della posa di equilibrio o arming differito.

## Verifica finale della chiusura

La verifica offline è stata eseguita con `uv run`, separando i test storici
dell'optimizer perché alcuni di essi importano intenzionalmente moduli
top-level omonimi mediante `sys.path`:

```bash
uv run pytest -q -m "not groimp" \
  src/groimp_bridge/tests src/exporterV1/tests src/exporterV2/tests
# 167 passed, 11 deselected

uv run pytest -q src/exporterV2/core/usd/tests \
  src/exporterV2/core/skinning/tests \
  src/exporterV2/adapters/groimp_csv/tests
# 65 passed, 2 skipped

uv run pytest -q src/exporterV2/core/optimizations/tests
# 98 passed, 1 skipped
```

Il test dedicato ai truss e alle guardie fruit-free riporta `26 passed`. Le
guardie dei due wrapper rifiutano `full` senza opt-in con exit code 2. Il test
Isaac finale è stato eseguito con:

```bash
./run_debugV2.sh --day 160 --headless --duration 5 --physics-hz 480
# stability=passed; 216 corpi; max displacement 0.0617247 m
```

Gli SHA-256 del GSZ e dei PlantState versionati sono stati misurati prima e
dopo e sono rimasti invariati. In particolare:

```text
project_bridge.gsz       d646a340eb3fd57f885d4dcea8f7f207b76a35596b0a87c90e63968d30acf4d9
plant_state_day_1.json   cd71d55c4b6f739b091a6ae5d9d4e7043c978973d86016e6b4f0ce8a5ee73f40
plant_state_day_10.json  edf4d5058b43a3742b3e4ee7a9f033a5fb9df217bd65d696c78fb0c3cf46fb85
plant_state_day_25.json  ab6a476b5a6021c879b0c29b33ef54b907a2f29fd971e49a34b1dc7bed61f7
plant_state_day_50.json  0ec2f431a5e4e5e723cdd002e7ede352552a402028fce61493d91261e6c20cf9
plant_state_day_80.json  afe5e1084537bc982d90665e35b6f0df0b7617e0675d294e68224077bebda4c6
plant_state_day_160.json d6c8b490f190b793f6bec6dea7f955c506f7ff02fd2a87b5c5337d43196a87bc
```

## Limiti residui e uso futuro

- La superficie canonica delle lamine non è in PlantState; viene ricostruita
  dallo stile V1/V2 senza inventare dati biologici.
- Le pose estetiche V2 di petioluli/pedicelli sono un adattamento authored e
  non sostituiscono la posa GroIMP conservata nel manifest.
- I filtri per overlap iniziali sono permanenti per la coppia e consentono
  pass-through successivo; la modalità `error` resta disponibile per audit.
- I petioluli fisici rimangono un confronto diagnostico costoso e soggetto al
  limite di giunti.
- La fisica dei pomodori maturi necessita in futuro di un modello terminale
  diverso o di una validazione dedicata; non blocca l'uso del builder generale
  né del percorso fruit-free.
- Il bridge live in-memory GroIMP -> PlantState -> Isaac non è parte di questa
  chiusura.

Il valore del branch è quindi un confine dati solido e verificabile, insieme a
un risultato negativo documentato in modo riproducibile. Non si nasconde il
fallimento del carico completo e non lo si propaga nel percorso predefinito.
