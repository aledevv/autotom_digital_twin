# Smoke piantina fissa — risultati del 16 settembre 2026

**5 lamine** sono la massima numerosità verificata che rispetta tutti i criteri
numerici e il budget 120 Hz fisica / 60 Hz visualizzazione, incluse le tre
ripetizioni offscreen, sei cicli di contatto e un benchmark GUI.
**20 lamine sono numericamente stabili**, ma non raggiungono il tempo reale
con rendering e diagnostica light nella configurazione misurata.

Isaac `6.1.0-rc.26+release.49347.2d230af4.gl`, estensione PhysX `110.3.2`,
RTX 4080 Laptop 12 GiB, driver `575.57.08`. Fisica **CPU/PGS**, rendering GPU,
risoluzione 1280×720. Parametri fisici del candidato singolo invariati.
Il contorno reale seed 42 viene ripetuto, massa 1,48984 g per lamina.
Tre rami statici, piccioli fissati al mondo, collisioni tra lamine disabilitate
nella prova di scalabilità. Sleeping disabilitato: tutte le articolazioni restano
attive anche quando sono assestate. Una sfera preme una lamina per volta.

## Prestazioni misurate

Campagna decisionale: `artifacts/leaf61/plant-campaign-final/`.
Finestra prestazionale t=9–22 s, esclusi avvio/assestamento e analisi successiva.
RTF = tempo simulato / tempo reale. La soglia p95 è **16,667 ms** per due passi
fisici, aggiornamenti, diagnostica prevista e un rendering.

| Lamine | Corpi mobili / DOF | Numerica full | RTF offscreen, 3 prove | p95 peggiore | Promossa |
|---:|---:|---|---:|---:|---|
| 1 | 3 / 5 | superata | 1,911–2,001 | 10,752 ms | sì |
| 5 | 15 / 25 | superata | 1,465–1,541 | 14,586 ms | sì |
| 10 | 30 / 50 | superata | 1,161–1,191 | 18,865 ms | **no: p95** |
| 20 | 60 / 100 | superata | 0,845–0,860 | 27,092 ms | **no: RTF e p95** |

Dieci lamine superano il tempo reale medio, ma non il criterio sulla regolarità
dei frame. Non sono state promosse e nessuna soglia è stata rilassata.
Il numero 5 vale per questa fixture e questa macchina: non stima la capacità
con rami dinamici, tutte le collisioni reciproche o una pianta PlantState.

Costi medi per frame nelle tre prove offscreen, in millisecondi:

| Lamine | Fisica | Letture | UsdSkel | Diagnostica | Rendering |
|---:|---:|---:|---:|---:|---:|
| 1 | 1,934 | 0,339 | 0,311 | 0,084 | 5,258 |
| 5 | 3,221 | 0,588 | 0,824 | 0,342 | 5,573 |
| 10 | 4,765 | 0,836 | 1,336 | 0,614 | 6,122 |
| 20 | 7,374 | 1,263 | 2,245 | 1,160 | 6,999 |

I costi medi non sono percentili; la diagnostica a 10 Hz concentra lavoro in
alcuni frame. Il tempo completo include anche comandi e altro overhead.
La prova full con 20 lamine ha richiesto 54,5 s di analisi **dopo** la simulazione:
questo tempo non entra nel RTF. Memoria del processo e GPU sono nei report;
l'occupazione GPU è riferita al dispositivo e include il resto dell'ambiente.

## Stabilità, regressione e GUI

Regressione singola: differenza massima dalla traiettoria precedente
`skinning-real-final` di **0,000276 mm**, contro soglia 0,1 mm; stessi tempi di
campionamento e accettazione numerica superata.

Nelle prove complete 1/5/10/20: deriva basale nulla, flessione gravitazionale
circa 2,699 mm, allungamento massimo 0,306%. Con 20 lamine, lo spostamento massimo
delle foglie non premute rispetto all'equilibrio è 0,000067 mm.

Cinque lamine, sei cicli (tre centrali e tre fuori centro): recupero peggiore
**0,002046 mm**, residuo finale **0,000018 mm**, allungamento massimo **0,5633%**.
Contatto effettivo, con distanza minima sfera-superficie di circa 0,293 mm.
Tutti i controlli originali sono superati.

Benchmark GUI automatico a cinque lamine: **1,355×**, p95 **15,562 ms**,
81,28 chiamate di rendering completate/s senza pacing. È una misura della GUI
in esecuzione, non della frequenza di presentazione fisica del monitor.
Il launcher interattivo è invece regolato sul tempo reale. La revisione visiva
umana e dei pulsanti resta **da completare**; non viene dedotta dalle misure.

## Coppia con contatto reciproco

Due lamine, separazione verticale iniziale 3 mm, scostamento laterale 4 mm.
La sfera collide solo con la superiore. Identico protocollo centrale con
collisioni reciproche disabilitate e abilitate.

- Nessuna penetrazione iniziale rilevata dai contatti PhysX nella prova attiva.
- 6.156 campioni di contatto tra i segmenti delle due lamine, impulso non nullo.
- Differenza massima del movimento della lamina inferiore rispetto al controllo:
  **13,345 mm**. Senza collisioni la lamina inferiore resta al proprio equilibrio.
- Penetrazione massima riportata da PhysX: **0,300 mm**, sotto il limite di 1 mm.
- Recupero superiore **0,00165 mm**, inferiore **0,01175 mm**; residui sotto soglia.
- Attraversamento visivo campionato a 1 Hz: **0 mm** con collisioni, **13,49 mm**
  nel controllo che permette alle foglie di attraversarsi.

Il confronto numerico è superato. Il controllo visivo è campionato ai vertici e
non dimostra assenza di qualsiasi intersezione in ogni istante: serve la revisione
GUI. Non sono state misurate 20 lamine tutte in contatto tra loro.

## Riproduzione ed evidenze

```bash
./run_leaf61_plant.sh --leaves 5
./run_leaf61_plant.sh --layout contact-pair --pair-collisions on
/usr/bin/python3 src/experiments/leaf61/plant_campaign.py \
  --output artifacts/leaf61/plant-campaign-new --gui-benchmark
```

Il launcher senza argomenti usa le 10 lamine richieste dal piano: è utile per
esplorare il caso che supera il budget p95. Per il candidato verificato usare 5.

Evidenze locali: `decision.json`, `scaling.json`, `scaling.png`, `REPORT.md`,
`candidate-six-cycles/`, `candidate-gui/`, `pair-off/`, `pair-on/` nella campagna
finale. Ogni esecuzione conserva configurazione, layout, USD, versioni, sorgenti,
hash, log, pose, contatti e report per lamina. Le scene USD sono istantanee iniziali;
il launcher serve per riprodurre comandi della sfera e aggiornamenti dello skinning.

La campagna preliminare `plant-campaign/` aveva una camera troppo stretta:
i suoi risultati **con rendering non sono usati** per la decisione. Le prove
headless concluse sono state riutilizzate dopo controllo di equivalenza del codice
fisico, configurazione e mesh; i riferimenti sono in `reused_headless.json`.
La correzione della camera include un controllo del frustum sull'intero inviluppo
20-foglie; nessun parametro fisico è stato cambiato per migliorare il risultato.
