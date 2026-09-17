# Pianta con più foglie e pomodoro in cascata

Esperimento separato dal pomodoro su una foglia già approvato. Stessi parametri:
20 g, raggio 20 mm, distanza iniziale 60 mm, CPU/PGS a 480 Hz, giunti e materiali
invariati. I launcher precedenti mantengono il loro comportamento.

## Capacità della pianta

`--drop-test scale` crea un insieme fisso di 120 sostituzioni del candidato
seed 42 nei punti di attacco della pianta, orizzontali e con yaw nativo. Le
numerosità minori attivano sottoinsiemi dello stesso insieme. Le altre foglie,
fusto e rami restano visivi. Non si tratta di 120 forme native tutte validate.
Un controllo conservativo AABB esclude sovrapposizioni iniziali delle lamine.
Collisioni fra foglie filtrate; il pomodoro può colpire solo la foglia 0.
Tutte le foglie attive conservano corpi, giunti e solver, con sleeping disabilitato.

La camera mostra la pianta intera, a 1280×720. Questa vista e la geometria
visiva differiscono dalla vecchia misura ravvicinata della singola foglia:
non confrontare direttamente quei costi con la nuova curva.

Il rendering del benchmark è a 30 Hz di tempo simulato; la fisica rimane
480 Hz. Nessun pacing durante le misure. Finestra t=9–17 s, dopo assestamento;
letture e skinning inclusi, analisi geometrica completa e scrittura offline
escluse. Tutti gli stati fisici sono comunque salvati a ogni passo.

Due criteri distinti:

- Sopra 20 FPS: frame prodotti per secondo reale >20 e p95 del lavoro per
  frame <50 ms, oltre al superamento di tutti i controlli fisici.
- Tempo reale: rapporto secondi simulati/secondi reali ≥1, riportato
  separatamente. Gli FPS da soli non garantiscono questo requisito.

La capacità è riferita alla finestra misurata, alla macchina e a questa scena.
Il percentile non garantisce un minimo assoluto per ogni singolo frame.
La campagna esegue tre prove sul candidato maggiore e conserva anche i casi
falliti. Non estrapola oltre le numerosità provate.

```bash
python3 src/experiments/leaf61/tomato_capacity.py \
  --output artifacts/leaf61/tomato-capacity-repeat \
  --counts 1 5 10 20 30 40 --render-hz 30

./run_leaf61_tomato.sh --drop-test scale --leaves 20 \
  --render-hz 30 --playback-speed 1
```

Le cartelle di output devono essere nuove. Il secondo comando apre una GUI
che resta aperta fino a Termina e salva. Le prestazioni GUI sono distinte
dalla capacità offscreen.

## Cascata

```bash
./run_leaf61_cascade.sh
```

Tre foglie inizialmente orizzontali, distanza verticale 60 mm e avanzamento
45 mm per piano. Pomodoro in vera caduta libera; nessun bersaglio cinematico
o comando di forza durante il volo. Il calice verde è solo visivo. Sono
abilitati i contatti del pomodoro con tutte le foglie e quelli fra foglie.
Primi contatti ordinati non implicano intervalli di contatto disgiunti: il
pomodoro può toccare due foglie durante il passaggio.

`--leaves 2..8`, `--leaf-gap`, `--lower-offset` e `--lower-y` permettono varianti;
non tutte sono già validate. `--cascade-layout percorso.json` permette
posizioni deterministiche, come lista di offset xyz in metri, una per foglia.
Il file viene copiato negli artefatti. Le lamine mantengono yaw zero; i centri
basali si trovano a quota `0.15 + offset_z`. Il pomodoro parte sopra la prima.
`--playback-speed 1` mostra il tempo reale, il default 0,25× facilita la
revisione dell’urto. **Ripeti caduta** ripristina il pomodoro e riavvia la prova.

Il primo tentativo `tomato-cascade-3-pilot` è fallito: l’offset del gruppo
veniva contato due volte nello spawn del pomodoro. È conservato come prova
non valida. Il pomodoro della cascata viene ora creato in coordinate mondo,
con controllo esplicito fra posizione richiesta e posizione fisica iniziale.

### Cascata di tre foglie: verifica numerica

`artifacts/leaf61/tomato-cascade-3-v2/`, esito `passed`, senza rendering.

| Foglia | Primo contatto dopo il rilascio | Spostamento massimo | Recupero residuo |
|---|---:|---:|---:|
| Superiore | 0,1125 s | 62,85 mm | 0,162 mm |
| Intermedia | 0,1896 s | 27,95 mm | 0,039 mm |
| Inferiore | 0,3104 s | 16,89 mm | 0,030 mm |

Tutte ricevono impulsi PhysX positivi; allungamento massimo 2,79%, deriva
basale zero, penetrazione geometrica massima 0,406 mm. Nessun allentamento
delle soglie precedenti. La validazione visiva e le prestazioni con rendering
sono distinte da questa prova fisica headless.

## Risultati sulla pianta, 17 settembre 2026

Evidenza: `artifacts/leaf61/tomato-capacity-r30/`. Rapporti completi,
`costs.csv`, `REPORT.md`, `results.json` e `decision.json` nella cartella.

| Foglie dinamiche | FPS senza pacing | Velocità simulata/reale | p95 lavoro/frame |
|---:|---:|---:|---:|
| 1 | 51,67 | 1,722× | 22,34 ms |
| 5 | 43,04 | 1,435× | 25,39 ms |
| 10 | 34,69 | 1,156× | 32,74 ms |
| 20 | 27,41 | 0,914× | 41,18 ms |
| 30, tre ripetizioni | 23,02–23,26 | 0,767–0,775× | 48,48–49,75 ms |
| 40 | 20,64 | 0,688× | 54,79 ms |

**Massima numerosità verificata sopra 20 FPS: 30 foglie**, confermata in tre
prove, con margine ridotto. Anche nel primo secondo dopo il rilascio il p95
delle tre prove è inferiore a 50 ms (peggiore 48,41 ms). A 40 foglie la media
è appena sopra 20 ma il p95 supera 50 ms, quindi il criterio non passa.
Non sono state provate numerosità intermedie fra 30 e 40 o superiori a 40.

**Massima numerosità provata che mantiene anche tempo reale: 10**, con una
prova nella campagna. Il limite esatto fra 10 e 20 non è stato cercato.
A 20 foglie c’è più margine sugli FPS rispetto a 30, ma già circa il 9% di
rallentamento. La prova preliminare separata `tomato-scale-20-r30` è coerente:
27,54 FPS, RTF 0,918, p95 39,89 ms.

Tutti i controlli numerici passano in tutte le otto prove, compresa quella
con 40 foglie: il fallimento a 40 è prestazionale. Le foglie indipendenti
non premute restano entro la soglia di 0,5 mm e non ricevono contatti del
pomodoro. Nessuna soglia è stata modificata. Questi dati non misurano urti
simultanei su 30 foglie né una pianta con rami dinamici.

I costi includono letture delle pose a ogni passo, controlli di finitezza e
skinning; non sono tempi del solo solver. Il rendering della GUI con pacing
sarà limitato ai 30 Hz richiesti, anche quando la capacità offscreen supera
30 FPS. La revisione e la misura GUI rimangono separate.

Nei report grezzi `target_20fps_passed` richiede anche RTF ≥1; la verifica
separata della sola soglia FPS si trova in `results.json` e `decision.json`.

## Cascata di cinque foglie

```bash
./run_leaf61_cascade_five.sh
```

Usa `configs/cascade-five.json`: quote 390/330/270/210/150 mm, avanzamenti
basali 0/45/90/155/225 mm. Le ultime due sono spostate più avanti per
intercettare la traiettoria deviata. I parametri fisici restano gli stessi.
La GUI indica **Foglie colpite: 1 → 2 → 3 → 4 → 5**, basandosi sugli impulsi
PhysX positivi; il contatore riparte a ogni Ripeti caduta. La camera GUI
include anche lo spawn sopra la prima foglia.

Evidenza `artifacts/leaf61/tomato-cascade-5-render/`: tutti i controlli numerici
superati, con rendering a 60 Hz e posizione visuale coerente con la fisica.

| Foglia | Primo contatto dopo il rilascio | Spostamento massimo | Recupero residuo |
|---|---:|---:|---:|
| 1 | 0,1125 s | 62,85 mm | 0,162 mm |
| 2 | 0,1896 s | 27,95 mm | 0,039 mm |
| 3 | 0,3104 s | 16,89 mm | 0,030 mm |
| 4 | 0,3646 s | 40,71 mm | 0,121 mm |
| 5 | 0,4083 s | 40,86 mm | 0,288 mm |

Allungamento massimo 2,79%, deriva basale zero, penetrazione geometrica
massima 0,406 mm. Tutte le foglie ricevono impulsi positivi e recuperano.
Una sola misura offscreen: RTF 1,146, capacità 68,76 FPS, p95 16,96 ms.
Supera il budget dei 20 FPS, **non** il precedente p95 di 16,667 ms a 60 Hz.
È un banco separato senza la geometria della pianta: non sommare né
estrapolare questi costi alla campagna sulla pianta. La camera GUI è stata
successivamente allargata per includere lo spawn; la revisione visiva rimane
all’utente. Il default rallentato non è una misura di tempo reale.
