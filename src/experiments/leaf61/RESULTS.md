# Risultati — 16 settembre 2026

**Candidato numerico: skinning a tre segmenti, picciolo fissato, 120 Hz.**
Revisione GUI dell’utente ancora da eseguire. Nessuna integrazione nella pianta.

Runtime: Isaac Sim `6.1.0-rc.26+release.49347.2d230af4.gl`, PhysX 110.3.2, RTX 4080 Laptop 12 GiB.

| Caso | Contatti superati | Sag mm | Allungamento massimo | Recupero peggiore mm | Esito |
|---|---:|---:|---:|---:|---|
| Skinning, rettangolo 60 Hz | 6/6 | 3.602 | 0.498% | 0.004292 | passed |
| Skinning, rettangolo 120 Hz | 6/6 | 3.408 | 0.378% | 0.001574 | passed |
| Skinning, rettangolo 240 Hz | 6/6 | 3.386 | 0.359% | 0.000703 | passed |
| Skinning, contorno reale + rendering | 6/6 | 2.699 | 0.563% | 0.001974 | passed |
| Surface, contatto corretto | 0/1 | 11.626 | 0.203% | 0.547051 | failed |

Tutte le prove riportate sono finite e senza errori PhysX. Le prove skinning hanno deriva della fascia fissata nulla alla precisione della lettura. La sfera è verificata dalla tensor API contro la traiettoria comandata.

## Decisione e confini

- La catena articolata elimina gli errori di traslazione dei giunti osservati nella prima versione a corpi separati. La fascia fissata non viene mescolata nei pesi della prima sezione mobile; il giunto radice permette flessione, i due interni anche torsione.
- Il contorno reale è prodotto da `real_leaves`, seed 42, posa piatta e lunghezza biologica nominale 90 mm. Il collider di ogni segmento è un prisma convesso della porzione di mesh; non rappresenta ogni concavità del contorno.
- Surface: cooking, attachment (45 nodi nella fixture rettangolare), materiale e lettura GPU funzionano. Le rigidezze di flessione 5e6, 5e8 e 5e10 a 64 iterazioni non superano la gravità. Con bend=5e8 e 256 iterazioni il sag scende a 11,6 mm, ma il contatto corretto lascia 0,750 mm di oscillazione residua contro il limite 0,5 mm. Il ciclo strumentato avanza a 0,197 volte il tempo reale. Nessuna promozione surface; non sono stati avviati ulteriori sweep o raffinamenti sul modello respinto.
- La prima prova surface a sei contatti (`surface-04-cycle`) usava una traiettoria USD non recepita dalla fisica e aveva un errore di fine ciclo. Non è evidenza di comportamento fisico: il confronto usa solo `surface-04-press-v2`, con readback e conteggio corretti.
- La view GPU non implementa `set_kinematic_targets`; il banco surface prescrive la posa a ogni piccolo passo con `set_transforms`. Il banco CPU usa veri target cinematici. Questo confronto vale per la pressione lenta, non per urti o identità delle forze di contatto.
- La stabilità a 60/120/240 Hz è verificata sul rettangolo. Il contorno reale è verificato a 120 Hz. Le prestazioni di una foglia non misurano quelle di una pianta.

## Riproduzione

```bash
./run_leaf61.sh
/usr/bin/python3 src/experiments/leaf61/run.py --model skinning --scenario cycle --config src/experiments/leaf61/configs/skinning_candidate.json
```

La GUI dispone di Premi, Rilascia e Ripeti prova. Il wrapper apre il contorno reale con un contatto centrale iniziale; la suite numerica completa usa `--scenario cycle`.

Output locali: `artifacts/leaf61/`. Configurazioni, scene, tracce e log rimangono separati per run. I run recenti eseguono i sorgenti archiviati in `sources/`. Il grafico comune è in `artifacts/leaf61/timestep-comparison/comparison.png`.

12 test automatici del modello geometrico, Ruff (target Python 3.10 per il supervisore), sintassi shell e `git diff --check` verificati. La GUI interattiva richiede ancora revisione umana.


## Verifica della versione finale

`skinning-real-final`: un contatto sul contorno reale con rendering, tutte le
soglie superate; 22 s simulati in 16,03 s di ciclo completo (1,37× tempo reale).
Il costo comprende passo fisico, letture, aggiornamento UsdSkel, rendering e
strumentazione durante il ciclo; esclude avvio Isaac, valutazione finale e
salvataggio. Non è una misura degli FPS della GUI o della pianta.

Il controllo della distanza visiva dai collider è campionato a 10 Hz; fisica,
traccia nodale, pose rigide e sfera rimangono a 120 Hz. I primi 2.641 campioni
nodali e della sfera coincidono esattamente con la precedente prova a sei
contatti (`skinning-real-render`), nella quale tale diagnostica era calcolata a
ogni passo. Evidenza in `artifacts/leaf61/final_equivalence.json`.

L'immagine offscreen è stata aperta e controllata: foglia, supporto e sfera
sono visibili. Questo controllo statico non sostituisce la revisione GUI.
`./run_leaf61.sh` apre la configurazione scelta con Premi / Rilascia / Ripeti
prova; ripetere ritira prima la sfera e attende l'assestamento.

## Estensione alla piantina fissa

La campagna 1/5/10/20, il benchmark GUI e la coppia in contatto sono documentati
in [PLANT_RESULTS.md](PLANT_RESULTS.md). Massimo verificato con i criteri
120 Hz fisica / 60 Hz rendering: **5 lamine**. Le 20 lamine restano numericamente
stabili; 10 e 20 non superano tutte le soglie prestazionali. La revisione visiva
umana della nuova fixture resta separata.
