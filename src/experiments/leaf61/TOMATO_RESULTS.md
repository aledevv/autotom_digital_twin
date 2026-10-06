# Pomodoro in caduta sulla pianta: prima lamina

Avvio: `./run_leaf61_tomato.sh`. Isaac Sim 6.1, CPU/PGS, una lamina dinamica
del candidato seed 42 al suo punto di attacco nella pianta day-160 congelata.
Il corpo rosso è **dinamico**, con massa e collider sferico. Il piccolo calice
verde è solo visivo. Parametri della lamina invariati, picciolo fissato al mondo.

## Configurazione consegnata

- Massa esplorativa **20 g**, raggio **20 mm**, distanza iniziale fra fondo
  del pomodoro e lamina a riposo **60 mm**. Nessuna calibrazione biologica.
- Un secondo senza gravità e otto di assestamento della foglia; il pomodoro
  attende con gravità disabilitata, poi viene liberato a t=9 s.
- Dopo il rilascio nessuna posa, velocità, forza o traiettoria prescritta.
  Solo **Ripeti caduta** riposiziona il corpo e azzera le velocità per ricominciare.
- Passo fisico **1/480 s**, CCD continua e speculativa abilitate.
- Rendering a 60 Hz di tempo simulato nelle misure offscreen; GUI rallentata
  a **0,25× dopo il rilascio** per osservare l'urto. La gravità rimane 9,81 m/s².
- Tutte le altre foglie e i rami sono visivi e senza collisioni. Il pomodoro
  può attraversarli; dopo la lamina attiva cade nel vassoio con bordi statici.

La GUI indica il conto alla rovescia e resta aperta fino a **Termina e salva**.
**Ripeti caduta** attende nuovamente otto secondi di assestamento. Il launcher
`run_leaf61_canopy.sh` rimane il precedente test con sfera cinematica: per la
caduta usare il nuovo launcher `run_leaf61_tomato.sh`.

## Risultato numerico

Evidenza: `artifacts/leaf61/tomato-drop-20g-480-render/`, esito `passed`.

| Misura | Risultato |
|---|---:|
| Primo contatto dopo il rilascio | 0,1125 s |
| Intervallo primo–ultimo contatto | 0,0979 s |
| Spostamento massimo della superficie dall'equilibrio | 65,53 mm |
| Recupero nell'ultimo secondo | 0,190 mm |
| Oscillazione residua nell'ultimo secondo | 0,000048 mm |
| Deriva della fascia fissata | 0 mm |
| Allungamento massimo degli spigoli visivi | 3,704% |
| Penetrazione massima nei contact report PhysX | 0,0356 mm |
| Penetrazione geometrica sfera–prismi convessi | 0,254 mm |
| Penetrazione sfera–mesh visiva | 0,0527 mm |

La flessione è grande, poi la foglia recupera. Non si tratta di un pomodoro
che rimane appoggiato: il contatto è transitorio e il corpo prosegue verso il
piano. I contatti PhysX registrano impulsi positivi. Nei primi 80 ms la quota
segue la caduta libera discreta a 9,81 m/s² con errore massimo di 0,00012 mm.
Questo controllo e le pose salvate verificano che il corpo cada realmente.

## Limite trovato con 30 g

Il primo pomodoro scelto pesava 30 g. Stessi raggio, altezza, lamina e giunti:

| Frequenza | Penetrazione PhysX | Penetrazione geometrica | Allungamento visivo | Esito numerico |
|---:|---:|---:|---:|---|
| 120 Hz | 1,291 mm | 1,014 mm | 5,993% | non superato |
| 240 Hz | 3,504 mm | 3,224 mm | 5,095% | non superato |
| 480 Hz | 0,0357 mm | 0,235 mm | 5,025% | non superato |
| 960 Hz | 0,0093 mm | 0,311 mm | 5,162% | non superato |

Le prove sono `tomato-drop-120`, `tomato-drop-240`, `tomato-drop-480-render`
e `tomato-drop-960-render`. Nessun errore PhysX o dato non finito; tutte
recuperano, ma nessuna supera l'intero insieme di soglie. L'allungamento
misura la superficie skinnata, non la deformazione materiale dei segmenti
rigidi. Aumentare la frequenza risolve il contatto, ma non quel limite visivo.
La soglia del 5% non è stata rilassata. Il controllo a 20 g cambia **solo la
massa** e non costituisce una validazione del caso a 30 g.

## Costo prima dell’ottimizzazione

Una prova offscreen per ciascun caso riportato; non è una promozione basata
su tre ripetizioni. Camera ravvicinata e vassoio diversi dal benchmark di
pressione lenta: non attribuire tutta la differenza di costo al solo passo.
Finestra misurata t=9–17 s, escludendo avvio, assestamento e analisi offline.

| Caso | RTF | p95 lavoro per frame | p95 primo secondo dopo rilascio |
|---|---:|---:|---:|
| 30 g, 480 Hz | 0,895 | 22,23 ms | 21,97 ms |
| 30 g, 960 Hz | 0,579 | 39,35 ms | 27,23 ms |
| 20 g, 480 Hz | 1,022 | 19,28 ms | 18,79 ms |

**Nessuna di queste misure iniziali supera il budget di 16,667 ms.** Il caso da 20 g è circa in tempo
reale medio ma non mantiene la regolarità richiesta a 60 Hz. Costi medi per
frame in quel caso: fisica 8,27 ms, letture 1,38 ms, skinning 0,30 ms,
rendering 6,21 ms; il resto è overhead. Il picco RAM di circa 7,66 GiB comprende
anche l'analisi offline delle mesh, non soltanto la simulazione.

I 60 FPS della GUI rallentata non equivalgono a tempo reale della fisica.
Queste misure riguardano una sola lamina; non dimostrano il costo di due
o cinque lamine colpite. Il comportamento pomodoro–foglia è stato approvato
visivamente dall’utente prima dell’ottimizzazione.
Le verifiche geometriche avvengono offline e sono escluse dai tempi del loop.

Riproduzione della misura senza pacing:

```bash
/usr/bin/python3 src/experiments/leaf61/run.py \
  --model skinning --shape real --mesh artifacts/leaf61/real-seed42.npz \
  --canopy-usd artifacts/branch_collisions/C-organic-leaf-pair-settle60/scene.usda \
  --layout canopy-drop --leaves 1 --scenario press --hz 480 \
  --ball-mass 0.02 --ball-radius 0.02 --drop-height 0.06 --render
```

Tutti i risultati restano locali, con sorgenti congelati, configurazione,
USD iniziale, contatti e pose in `trace.npz`, tempi in `timings.npz`, log e
report. Il giudizio visivo dell'utente resta distinto dall'accettazione numerica.

## Ottimizzazione del runtime (17 settembre 2026)

Il percorso `--drop-runtime optimized` sincronizza le pose fisiche in USD
soltanto ai frame visuali, legge insieme le pose dei segmenti e del
pomodoro tramite PhysX tensors e riusa gli attributi UsdSkel a 60 Hz. Disabilita
inoltre la copia continua delle velocità in USD: vengono già lette dai tensors
e salvate nelle tracce. Massa,
geometria, giunti, solver CPU/PGS e passo fisico a 480 Hz restano invariati.
`--drop-runtime baseline` mantiene il percorso precedente per confronti.
La GUI ottimizzata usa scadenze cumulative: il tempo del rendering viene
compensato nei sottopassi successivi, senza aggiungere attese a ogni frame.

Il confronto verifica l’uguaglianza esatta di tutti i campioni di pose,
velocità e contatti rispetto al caso approvato. Verifica inoltre la posizione
USD del collider visualizzato nel primo secondo dopo il rilascio, separata
dalle letture PhysX. La sincronizzazione usa l’API PhysX
`update_transformations(False, True, False)` a 60 Hz. Non imposta pose o target
cinematici sui corpi: pubblica lo stato simulato per il renderer.

Due tentativi Fabric non hanno superato il controllo della posa visuale,
nonostante traiettorie fisiche identiche e tempi migliori. Sono esclusi dalla
consegna e conservati in `artifacts/leaf61/tomato-optimization/` e
`artifacts/leaf61/tomato-optimization-v2/`; non costituiscono prove valide
delle prestazioni della visualizzazione.

Riproduzione dei tre confronti accoppiati, senza pacing:

```bash
/usr/bin/python3 src/experiments/leaf61/tomato_benchmark.py \
  --output artifacts/leaf61/tomato-optimization-repeat
```

Ogni percorso conserva scena, sorgenti, tracce, log e report. La cartella di
output deve essere nuova. Il benchmark si interrompe se cambiano le traiettorie
o falliscono i controlli numerici; una prestazione insufficiente resta esplicita
in `decision.json`. La revisione visiva GUI è separata dalla misura offscreen.
Nella GUI interattiva, `gui_initial_window.json` salva i tempi della prima
caduta dopo otto secondi di osservazione senza chiudere la finestra.

Il passaggio intermedio che sincronizzava ancora le velocità in USD è conservato
in `artifacts/leaf61/tomato-optimization-usd60/`: tre ripetizioni fisicamente
identiche, RTF 1,127–1,173, p95 peggiore 18,108 ms. Il budget prestazionale
non era ancora superato in tutte le ripetizioni.

### Confronto finale

Evidenza: `artifacts/leaf61/tomato-optimization-final/`, tre coppie sequenziali
baseline/optimized. Stessi input, camera, 1280×720, rendering a 60 Hz,
fisica a 480 Hz e diagnostica completa. Finestra t=9–17 s, senza pacing.

| Percorso | RTF min–max | p95 peggiore/frame | p95 peggiore nel primo secondo |
|---|---:|---:|---:|
| Prima | 1,032–1,089 | 18,904 ms | 18,938 ms |
| Ottimizzato | 1,335–1,390 | 14,997 ms | 15,820 ms |

Tutte e tre le prove ottimizzate superano RTF ≥1 e p95 ≤16,667 ms, anche
nel primo secondo dopo il rilascio. Il lavoro medio per frame scende di
circa il **22%** nei confronti accoppiati. Il picco RAM rimane circa
7,7 GiB: questa modifica riduce il tempo, non dimostra risparmi di memoria.

Tutte le pose, velocità, contatti e tempi simulati sono **identici campione
per campione** al riferimento approvato: 8.161 campioni di stato per prova.
Il controllo della posa USD visualizzata dà errore zero nelle 60 letture
del primo secondo. Tutti i controlli numerici passano. I tempi dei passi
fisici includono l’interfaccia e la sincronizzazione: il guadagno non implica
che il solver abbia risolto un problema più semplice.

Il launcher `./run_leaf61_tomato.sh` usa ora l’ottimizzazione. Per provarlo
in tempo reale, mantenendo la finestra aperta:

```bash
./run_leaf61_tomato.sh --playback-speed 1
```

Per tornare al percorso precedente: `--drop-runtime baseline`. Il default
resta 0,25× dopo il rilascio per osservare l’urto. I risultati promuovono
soltanto **una lamina dinamica**; non sono una misura di scalabilità della
pianta completa. Il nuovo percorso GUI deve ancora ricevere la revisione
visiva dell’utente, distinta dall’approvazione precedente del comportamento.

### GUI interattiva

Prova `artifacts/leaf61/tomato-optimization-gui/`, a 1×: RTF misurato 0,9998,
p95 del lavoro per frame 15,49 ms; posa visuale coerente con PhysX.
Durante la prova sono stati azionati due reset e ulteriori rilasci, poi la
sessione è terminata prima degli otto secondi successivi all’ultimo rilascio.
Il report conserva quindi `completed=false` e `status=failed`, con
`process_ok=true`: **non è una conferma GUI completa del protocollo**, né un
confronto di traiettoria equivalente alla caduta singola offscreen. Gli altri
controlli numerici passano. Il file della prima finestra automatica non è
stato scritto perché i reset hanno interrotto quella finestra.
