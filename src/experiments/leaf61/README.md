# Foglia isolata — Isaac Sim 6.1

Surface deformable oppure tre segmenti elastici con UsdSkel.
Il candidato selezionato usa una catena articolata con giunti nella sola lamina. Picciolo rigido
fissato al mondo tramite fixed joint; nessuna modifica alla pianta.

## Pianta completa: primo incremento

Per la **caduta libera di un pomodoro** su una lamina della pianta:

```bash
./run_leaf61_tomato.sh
```

Questa modalità usa un corpo rigido dinamico rosso con collider sferico:
20 g, diametro 40 mm, altezza iniziale di caduta 60 mm. Parametri esplorativi,
non calibrati biologicamente. Dopo 9 s simulati di inizializzazione e
assestamento viene attivata la gravità sul pomodoro; nessuna posa, velocità
o forza viene comandata durante la caduta. Il conto alla rovescia è visibile.
Il pomodoro colpisce la lamina e prosegue verso il vassoio sottostante.

La GUI parte rallentata a **0,25× dopo il rilascio**, per osservare il contatto;
`--playback-speed 1` richiede velocità reale senza garantirla. Il passo fisico
è 1/480 s. I 60 FPS eventualmente osservati al rallentatore non dimostrano
tempo reale della simulazione. La finestra resta aperta: **Ripeti caduta**
riporta il pomodoro sopra la lamina e attende 8 s di assestamento; **Termina e
salva** conclude la prova. Non usare il launcher `canopy` per questa demo:
quello conserva la sfera cinematica per le misure di pressione lenta.

In questo primo passo una sola lamina è dinamica. Tutta la vegetazione
restante è visiva, senza collisioni: il pomodoro può attraversarla. Il nuovo
launcher non cambia i precedenti banchi a 120 Hz. Configurazione, scena,
pose del pomodoro e delle lamine, contatti PhysX, tempi e report restano
in `artifacts/leaf61/<run>/`. La misura geometrica della penetrazione usa
la distanza sfera–prismi convessi, e la mesh visiva viene controllata a parte.
Il launcher usa `--drop-runtime optimized`: pose USD aggiornate ai frame
visuali, letture PhysX raggruppate e nessuna copia ridondante delle velocità
in USD. Per il confronto precedente usare `--drop-runtime baseline`.
Fisica e parametri del contatto restano invariati. Il confronto riproducibile
è `python3 src/experiments/leaf61/tomato_benchmark.py --output artifacts/leaf61/tomato-ab-new`.
I tempi della prima caduta GUI sono salvati in `gui_initial_window.json`
senza chiudere l’applicazione.
Risultati e limiti del carico: [TOMATO_RESULTS.md](TOMATO_RESULTS.md).

Per misurare più foglie e provare urti in sequenza, vedere
[TOMATO_CAPACITY.md](TOMATO_CAPACITY.md). Esempi:

```bash
./run_leaf61_tomato.sh --drop-test scale --leaves 20 --render-hz 30 --playback-speed 1
./run_leaf61_cascade.sh
```

Il primo isola il costo delle foglie indipendenti nella pianta; il secondo
abilita i contatti su una cascata di tre foglie in un banco separato.


```bash
./run_leaf61_canopy.sh --leaves 0   # riferimento statico
./run_leaf61_canopy.sh --leaves 1
./run_leaf61_canopy.sh             # cinque lamine dinamiche
/usr/bin/python3 src/experiments/leaf61/canopy_campaign.py \
  --output artifacts/leaf61/canopy-stage1-new --gui-benchmark
```

Il nuovo launcher usa la copia locale della pianta day-160 salvata in
`artifacts/branch_collisions/C-organic-leaf-pair-settle60/scene.usda` e Isaac 6.1.
L'originale viene copiato e tutta la sua fisica viene rimossa nella copia.
Restano **131 lamine visibili**, rami e frutti statici. Cinque lamine dello stesso
ramo vengono sostituite, anche nel caso statico, con il candidato seed 42 da
90 mm: punti di attacco originali, orientamento orizzontale e azimut del modello.
Questo primo passo isola il costo della dinamica mantenendo la geometria visiva
iniziale identica fra 0, 1 e 5 lamine attive. Non è ancora la conversione fisica
di tutte le forme e inclinazioni native della pianta.

CPU/PGS, fisica 120 Hz, aggiornamento UsdSkel e rendering 60 Hz, 1280×720.
Le collisioni sono limitate a sfera–lamine dinamiche: il resto della pianta è
visivo, senza collider, e le lamine sono filtrate reciprocamente. Sleeping
disabilitato. Il numero di lamine dinamiche si sceglie al lancio; la GUI sceglie
quale premere con **Premi / Rilascia / Ripeti prova / Termina e salva**.
Il caso 0 mantiene la sfera parcheggiata e misura il riferimento statico.

La campagna esegue per ogni numero una prova full headless e tre light con
rendering, fermandosi su errori fisici/runtime. La finestra prestazionale è
t=9–22 s, dopo l'assestamento; la GUI automatica è una verifica distinta.
`comparison.json` e `REPORT.md` confrontano gli esiti; ogni run conserva
input, sorgenti, scena, pose, log, memoria e tempi dei singoli componenti.
`timings.npz` conserva anche i campioni dei tempi. `canopy_audit.py`, eseguito
con le librerie USD della distribuzione Isaac, verifica geometria iniziale,
trasformazioni della pianta congelata e numero effettivo di corpi e giunti.
Risultati effettivamente misurati: [CANOPY_RESULTS.md](CANOPY_RESULTS.md).

## Banco della foglia isolata

Dalla radice del repository, per la foglia reale con controlli interattivi:

```bash
./run_leaf61.sh
```

Per i singoli banchi:

```bash
/usr/bin/python3 src/experiments/leaf61/run.py --model surface --scenario cycle
/usr/bin/python3 src/experiments/leaf61/run.py --model skinning --scenario cycle
/usr/bin/python3 src/experiments/leaf61/run.py --model surface --gui
```

Il launcher seleziona esplicitamente `~/isaacsim-6.1/python.sh`. `--config` accetta
un JSON di campi di `model.Config`; `--hz 60|120|240` prevale sul JSON.
`--scenario smoke|rest|press|cycle` distingue verifica runtime, sola gravità,
un contatto centrale e sei contatti (tre centrali + tre fuori centro).
`--render` abilita rendering offscreen. La GUI esegue il ciclo e rimane aperta;
**Premi / Rilascia / Ripeti prova** controllano la sfera. Le interazioni manuali
non costituiscono una prova automatica accettata. Stop/reset della timeline non
fa parte del protocollo: terminare e rilanciare per una nuova prova.

Gli output, mai sovrascritti, rimangono in `artifacts/leaf61/<run>/`: configurazione
risolta, runtime, hash dei sorgenti, scena iniziale, mesh a riposo, traccia nodale,
posa effettiva e comandata della sfera, pose dei corpi per skinning, log e report.
Ogni run esegue una copia locale dei sorgenti in `sources/`. La scena USD da sola
non riproduce la sequenza Python. `diagnostic` indica una prova smoke/rest,
**non** un'accettazione della foglia. La valutazione GUI resta umana.

## Proprietà e misure

90 × 40 mm, 30 × 14 celle, fascia fissata di 6 mm, spessore meccanico 0,5 mm,
densità 1.000 kg/m³, massa rettangolare 1,8 g. Lo spessore meccanico è distinto
da rest offset (0,25 mm) e contact offset (0,5 mm). Parametri esplorativi,
non misure biologiche. Young e Poisson controllano l'elasticità; bend stiffness
controlla la flessione. Gli override stretch/shear rimangono a zero.

Surface: gerarchia auto-cooked, materiale sperimentale, attachment PhysX,
solver TGS/GPU e lettura `DeformablePrim` su CUDA. Skinning: tre corpi rigidi,
giunti elastici con traslazioni bloccate, supporto fisso e interpolazione UsdSkel;
solver PGS/CPU. Nessuna animazione prescritta della foglia.

La sfera scende per 2 s, mantiene 1 s, risale in 2 s; seguono 8 s di recupero.
La quota iniziale viene calcolata sulla superficie assestata con distanza
sfera-triangoli. Ogni ciclo confronta il recupero con il proprio equilibrio
precedente sotto gravità. Tutte le soglie sono in `model.evaluate` e nel piano.

## Contorno reale e test

L'export richiede un ambiente Python 3.12 con le dipendenze `real_leaves`:

```bash
/home/alessandro/real_leaves/.venv/bin/python src/experiments/leaf61/export_real.py \
  --output artifacts/leaf61/real-seed42.npz
/usr/bin/python3 src/experiments/leaf61/run.py --model surface --shape real \
  --mesh artifacts/leaf61/real-seed42.npz
UV_CACHE_DIR=/tmp/uv-cache uv run --no-project --python /usr/bin/python3 \
  python -m pytest -q src/experiments/leaf61/tests
```

L'export usa il checkout `external/real_leaves`, seed 42, posa piatta e lunghezza
90 mm. Non riscrive modelli o dataset. Per una ricerca sequenziale limitata:

```bash
/usr/bin/python3 src/experiments/leaf61/campaign.py --output artifacts/leaf61/campaign-01 \
  --real-mesh artifacts/leaf61/real-seed42.npz
```

La campagna si arresta sugli errori runtime; non li interpreta come un risultato
fisico. Non avvia processi GPU concorrenti. Vedere `RESULTS.md` per l'evidenza
realmente ottenuta e i limiti ancora aperti.

## API verificate e limiti

Build locale: Isaac 6.1.0-rc.26, PhysX 110.3.2. Il runner usa `World` e
`RigidPrim` di compatibilità, ancora disponibili ma deprecati in 6.1; authoring,
materiali, attachment e letture deformable usano le nuove API.
`DeformablePrim.get_nodal_positions` richiede qui una simulation view CUDA:
la corrispondente lettura CPU restituisce un errore di funzione non implementata.
Non aggiungere la sfera cinematica alla scena gestita da `World.reset`, che tenta
di azzerarne le velocità e genera errori PhysX; viene comandata e letta dalla tensor API dopo il reset. Sulla view CPU si usano
`set_kinematic_targets`; la view GPU 110.3.2 non li implementa e usa
`set_transforms` a ogni piccolo passo quasi-statico. Questa prova non valida urti
o equivalenza dinamica dei due metodi di comando.

Fonti ufficiali confrontate con il codice installato:
- [DeformablePrim 6.1](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/py/source/extensions/isaacsim.core.experimental.prims/docs/index.html)
- [Materiali 6.1](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/py/source/extensions/isaacsim.core.experimental.materials/docs/index.html)
- [Limiti deformable PhysX](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/deformables/deformable_bodies.html)

Il contorno reale mantiene la lunghezza biologica nominale di 90 mm del generatore;
la mesh triangolata può non contenere esattamente il punto semantico della punta.
Per lo skinning reale i collider sono tre prismi convessi, ottenuti dalle porzioni
della stessa mesh. Approssimano le lobature: non dimostrano contatto preciso in
ogni concavità. Il report campiona a 10 Hz la distanza dei vertici visivi dai collider,
non una distanza di Hausdorff bidirezionale completa.

La radice della lamina skinnata consente flessione ma blocca la torsione della
fascia fissata; i due giunti interni consentono flessione e torsione. I pesi
non mescolano il supporto attraverso la prima sezione mobile: mescolare il supporto
impedirebbe una rotazione corretta del giunto e introdurrebbe stiramento artificiale.
L'articolazione è mantenuta sveglia durante le misure.

## Piantina fissa: scalabilità dello skinning

Per una **palla dinamica in caduta libera su due foglie**, usare invece:

```bash
./run_leaf61_drop.sh
```

La palla ha massa 5 g e raggio 10 mm, parte 60 mm sopra la lamina superiore
e viene liberata dopo l'assestamento. La seconda foglia è 60 mm più in basso
e 75 mm più avanti, per intercettare più internamente la palla in uscita dalla prima.
Entrambe possono collidere con la palla. Nessuna traiettoria o forza viene
imposta durante il volo: agiscono gravità, contatti e giunti elastici.
Piccioli fissi e rigidità delle lamine restano quelli del candidato.

Questa demo usa **960 Hz**, CCD e visualizzazione rallentata a 0,25× dopo il
rilascio, senza cambiare gravità o massa. Non è il benchmark prestazionale a
120 Hz. Controlli: **Lascia cadere / Ripeti caduta / Termina e salva**.
Ripeti riporta soltanto la palla alla posizione iniziale e lascia assestare
le foglie prima della nuova caduta. `--playback-speed 1` mostra il tempo reale
quando il computer riesce a mantenerlo. Massa e disposizione sono configurabili
con `--ball-mass`, `--drop-height`, `--leaf-gap`, `--lower-offset` (unità SI).
Vedere [DROP_RESULTS.md](DROP_RESULTS.md) per risultati e limiti.

```bash
./run_leaf61_plant.sh                         # 10 lamine, GUI, diagnostica light
./run_leaf61_plant.sh --leaves 20
./run_leaf61_plant.sh --layout contact-pair --pair-collisions on
/usr/bin/python3 src/experiments/leaf61/plant_campaign.py \
  --output artifacts/leaf61/plant-campaign-new --gui-benchmark
```

`--leaves 1|5|10|20` attiva il nuovo banco (solo skinning e contorno reale).
`--layout contact-pair` usa sempre due lamine, indipendentemente da `--leaves`;
`--pair-collisions on|off` controlla soltanto questo caso. La campagna confronta
entrambi. `--gui-benchmark` termina la GUI dopo il protocollo automatico;
senza questa opzione rimane aperta per la revisione umana. Il selettore sceglie
la lamina per la prossima pressione: usare **Ripeti prova** dopo aver cambiato
selezione durante un contatto. Nella coppia la sfera agisce solo sulla superiore.

La fixture sintetica ha tre rami paralleli a quote distanti 120 mm; gli attacchi
sullo stesso ramo sono distanti 80 mm. I casi piccoli usano le prime N posizioni
dello stesso layout da 20. I rami sono statici, ogni picciolo ha un fixed joint
al mondo. Non è una pianta PlantState e non simula propagazione del movimento
attraverso rami o piccioli. Le lamine indipendenti sono filtrate reciprocamente;
la coppia usa 3 mm di separazione verticale e 4 mm di scostamento laterale.

Parametri fisici invariati, CPU/PGS, sleeping disattivato, passo 1/120 s. Una
vista legge tutte le pose; UsdSkel viene aggiornato ogni due passi quando il
rendering è attivo. `full` salva le pose a 120 Hz e ricostruisce le superfici dopo
il loop; `light` salva le pose e controlla la geometria a 10 Hz. Entrambe leggono
le pose a 120 Hz, controllano valori finiti e movimento effettivo della sfera.
Non si conservano intere tracce nodali per N foglie: la mesh e le pose permettono
la ricostruzione. I massimi della modalità light sono massimi **campionati**;
la promozione richiede anche la prova full.

La finestra prestazionale inizia a t=9 s, dopo inizializzazione senza gravità e
assestamento. Ogni campione `frame_work` comprende due passi fisici, letture,
comandi, diagnostica eventualmente prevista e un rendering; il percentile 95
deve essere <= 1/60 s, e il rapporto tempo simulato/tempo reale >= 1 in tutte le
tre ripetizioni offscreen. La frequenza richiesta non implica FPS raggiunti.
Headless e rendering hanno risultati distinti. I tempi offline di analisi e
scrittura non entrano nel rapporto di simulazione. La memoria GPU riportata da
`nvidia-smi` è l'occupazione del dispositivo, non una misura esclusiva della foglia.

La campagna blocca la scalabilità se la regressione rispetto alla traccia
`skinning-real-final` supera 0,1 mm o fallisce i controlli fisici; si interrompe
anche sugli errori runtime. Una configurazione numericamente instabile o lenta
resta esplicitamente fallita, senza cambiare soglie. Ogni run conserva una copia
dei sorgenti e gli hash, `layout.json`, USD, pose, log e report per lamina.
`decision.json`, `scaling.json` e `REPORT.md` confrontano le esecuzioni. I contatti
della coppia registrano separazione e impulso PhysX; gli attraversamenti visivi
sono misurati separatamente a 1 Hz e richiedono anche revisione GUI.

Riferimenti: [prestazioni Isaac Sim 6.1](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/reference_material/sim_performance_optimization_handbook.html),
[articolazioni PhysX](https://nvidia-omniverse.github.io/PhysX/physx/5.6.1/docs/Articulations.html).
Le API dei contact report sono verificate anche sul demo `ContactReportDemo.py`
della distribuzione locale PhysX 110.3.2.

La camera della campagna usa focale 35 mm, aperture 36×20,25 mm e la stessa posa
per tutte le numerosità; un controllo del frustum verifica anche l'inviluppo delle
20 lamine nei casi più piccoli. La coppia usa una vista ravvicinata. Le immagini
`final.png` sono catturate dopo le misure, con fisica in pausa. Per il grafico:

```bash
/usr/bin/python3 src/experiments/leaf61/plant_compare.py artifacts/leaf61/plant-campaign-final
```

`plant_campaign.py --reuse-headless-from <campagna>` può riusare prove headless
concluse con successo: controlla configurazione, mesh e fingerprint AST di
costruzione della fisica, modello numerico, letture e loop. La camera e l'analisi
successiva al loop sono escluse dal fingerprint; **nessuna prova con rendering
viene riusata**. `reused_headless.json` conserva i riferimenti originali. Usare
questa opzione nella stessa installazione e macchina, dopo aver verificato che
l'ambiente prestazionale non sia cambiato. Il lancio normale ripete tutte le prove.

Nei report delle esecuzioni, `status: passed` indica i controlli **numerici**.
La prestazione è un esito distinto: `timing.performance_passed`; la campagna
richiede entrambi e tre ripetizioni per `promoted`. I riepiloghi dei componenti
sono campionati per passo fisico, con zero quando rendering/diagnostica non sono
previsti; `frame_work` aggrega sempre due passi ed è la misura usata per il budget
60 Hz. Il rapporto grafico usa i tempi totali dei componenti per frame.
