# Pianta completa: primo incremento 0 / 1 / 5

Esecuzioni del 17 settembre 2026, `artifacts/leaf61/canopy-stage1/`.
La pianta contiene 131 lamine visibili. Cinque lamine vicine dello stesso
ramo `Leaf_r7_o0_g421459_rachis` vengono sostituite con il candidato da 90 mm,
negli attacchi nativi ma in posa orizzontale. Stesse cinque sostituzioni anche
nel riferimento statico; cambia soltanto il numero di lamine dinamiche.
Tutto il resto, inclusi rami e frutti, è congelato e senza collisioni.

Fonte: copia locale dell'esperimento day-160
`artifacts/branch_collisions/C-organic-leaf-pair-settle60/scene.usda`, SHA-256
`e21c37aae8d16339c69d5ed24258c11a6b6dc6c6c3339b9e3e34504086b4579a`.
Originale intatto; nessuna modifica al PlantState o all'exporter.

## Confronto misurato

Isaac 6.1.0-rc.26, PhysX 110.3.2, RTX 4080 Laptop 12 GiB. CPU/PGS, fisica
120 Hz, rendering richiesto 60 Hz, risoluzione 1280×720. Sleeping disabilitato.
Una pressione centrale lenta: 1 s senza gravità, 8 s di assestamento, 5 s
di contatto/ritiro e 8 s di recupero. Costi misurati solo nella finestra t=9–22 s.
Una prova full headless e tre light con rendering per ogni numero.

| Lamine dinamiche | DOF elastici | RTF rendering, tre prove | p95 peggiore per frame | Budget offscreen |
|---:|---:|---:|---:|---|
| 0 | 0 | 2,474–2,565 | 8,56 ms | superato |
| 1 | 5 | 1,904–1,944 | 10,66 ms | superato |
| 5 | 25 | 1,467–1,472 | 15,10 ms | superato |

RTF = tempo simulato / tempo reale. La promozione richiede RTF ≥1 e
p95 ≤16,667 ms in tutte e tre le ripetizioni. Il frame comprende due passi
fisici, letture, aggiornamenti, diagnostica e un rendering. Le prove sono
senza pacing: queste misure non sono la frequenza di presentazione del monitor.

Costi **medi** per frame, mediati sulle tre ripetizioni, in ms:

| Lamine dinamiche | Fisica | Letture | UsdSkel | Diagnostica | Rendering |
|---:|---:|---:|---:|---:|---:|
| 0 | 0,897 | 0,037 | 0,002 | 0,001 | 5,440 |
| 1 | 1,952 | 0,325 | 0,301 | 0,084 | 5,434 |
| 5 | 3,268 | 0,558 | 0,873 | 0,330 | 5,783 |

I tempi medi dei componenti non sono sommabili per ottenere il p95; il lavoro
completo comprende anche comandi e overhead. La crescita è principalmente
nella fisica e negli aggiornamenti CPU, non nel solo rendering.

Verifica GUI automatica separata (`n5-gui-auto`): RTF 1,261, p95 16,601 ms,
controlli numerici superati e uscita regolare. Il margine rispetto a 16,667 ms
è minimo: non basta per assumere che dieci lamine o altri contatti mantengano
60 Hz. La prima GUI (`n5-gui`) ha ricevuto comandi manuali ed è esclusa dalla
conferma automatica; le sue misure restano salvate come prova interattiva.
Durante il benchmark i pulsanti fisici sono ora disabilitati.

## Verifiche

- Tutte le prove full e light completate superano i controlli numerici.
- Flessione gravitazionale circa 2,699 mm, deriva basale nulla,
  allungamento massimo 0,306%.
- Lamina premuta: flessione circa 16,61 mm dall'equilibrio, recupero
  0,000663 mm e oscillazione residua 0,000044 mm; contatto effettivo.
- Le quattro lamine non premute rimangono entro 0,000032 mm dal loro equilibrio.
- Regressione della traiettoria locale rispetto al precedente banco singolo:
  differenza massima 0,00311 mm, sotto la soglia di 0,1 mm (`regression.json`).
- Audit USD (`usd_audit.json`): stesse geometrie visibili iniziali nei tre casi,
  trasformazioni originali dei 733 primitivi della pianta conservate,
  nessuna fisica residua nella pianta congelata. Tolleranza delle trasformazioni
  dei nuovi primitivi 0,2 µm, per la conversione delle pose fisiche in float32.
- Con 5 lamine: 15 corpi mobili, 5 supporti fissati al mondo, una sfera cinematica,
  15 giunti elastici e 5 fixed joint. Le 126 lamine native restano visive.
- 16 test superati, Ruff, sintassi del launcher e `git diff --check` superati.

La prima prova di integrazione `canopy-preflight-0` era fallita per view fisiche
create sui corpi poi resi statici. La costruzione ora evita quelle view e usa
una sola lettura aggregata per le lamine attive. Il caso fallito è conservato
separatamente e non entra nel confronto.

## Riproduzione e limiti

```bash
./run_leaf61_canopy.sh --leaves 0
./run_leaf61_canopy.sh --leaves 1
./run_leaf61_canopy.sh --leaves 5
/usr/bin/python3 src/experiments/leaf61/canopy_campaign.py \
  --output artifacts/leaf61/canopy-stage1-new --gui-benchmark
```

I pulsanti selezionano la lamina da premere e consentono pressione, rilascio,
ripetizione e salvataggio. I contatti riguardano solo sfera e lamine attive:
questo passo non valida collisioni con le foglie statiche, interazione fra
lamine, rami mobili o conversione delle forme/inclinazioni native.
Non estrapolare i risultati a tutte le 131 lamine dinamiche, né alla caduta
della palla a 960 Hz. Cinque è il massimo numero provato su questa pianta.
La revisione visiva dell'utente resta separata dai controlli numerici.

Il launcher GUI interattivo resta aperto fino a **Termina e salva**. Solo
`--gui-benchmark` si ferma a 22 s simulati, esegue l'analisi offline e chiude
la finestra: la pausa durante il salvataggio non è una misura del costo fisico.
La sfera è cinematica e compie una pressione lenta; **Premi** la mantiene in
pressione fino a **Rilascia**. Non è una caduta libera.
