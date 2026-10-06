# Pioggia graduale sulla pianta

```bash
./run_leaf61_rain.sh
# Esempio più breve, con un'altra sequenza:
./run_leaf61_rain.sh --rain-count 12 --rain-seed 7
```

Configurazione delle 127 lamine approvata visivamente invariata: CPU/PGS,
480 Hz, 32 iterazioni, sleeping e skinning ottimizzato. Rami e fusto restano
visivi e fissi. I pomodori sono da 20 g, raggio 20 mm.

Dopo 9 secondi di inizializzazione e assestamento partono 24 pomodori, con
intervalli casuali uniformi fra 0,6 e 1,4 secondi. I punti di caduta vengono
campionati sopra diverse lamine, con jitter laterale, a 10–18 cm sopra il
punto più alto della chioma (oltre al raggio). Il seed è salvato. Ogni pomodoro
riceve posizione e velocità solo al rilascio; durante il volo agisce la fisica.
Collider e visuale dei pomodori in attesa sono disabilitati. I pomodori possono
urtare tutte le lamine e gli altri pomodori. I contatti fra lamine restano
filtrati, come nel candidato precedente.

La GUI resta aperta. Pausa/Riprendi ferma solo i nuovi rilasci; i pomodori già
in volo continuano a cadere. Nuova pioggia rimuove i pomodori della sequenza
precedente, incrementa il seed e riparte dopo tre secondi. Termina e salva
chiude l'applicazione. Ogni sequenza salva report, schedule, contatti e pose
campionate a 10 Hz in rain-NNN; terminata la finestra di osservazione non
accumula altre pose. Una sequenza interrotta viene riportata incompleta.

## Verifica eseguita

`artifacts/leaf61/rain24-smoke-v2`: 24/24 pomodori rilasciati e caduti,
50 lamine con impulsi PhysX positivi, 49 con spostamento dei segmenti >1 mm,
deriva dei corpi basali zero, dati finiti, un passo fisico per iterazione,
nessun errore PhysX. È una prova headless senza rendering: non è una misura FPS.
23 test unitari passano, inclusi riproducibilità del calendario e rilascio
selettivo che non modifica i corpi già in volo.

**Esito diagnostico, non accettazione fisica completa:** la massima penetrazione
riportata nei contatti PhysX è 4,32 mm, sopra la soglia di 1 mm del precedente
banco controllato. Allungamento delle superfici, recupero dopo ogni singolo
urto e penetrazione visiva non sono certificati per questa pioggia. Le masse,
i giunti e le soglie precedenti non sono stati alterati per nascondere il limite.
La prima prova `rain24-smoke` conserva un errore di formato dei buffer tensor,
corretto nella v2: il backend richiede buffer di dimensione completa anche
quando gli indici selezionano un solo corpo.

## Intensità crescente

```bash
./run_leaf61_rain_ramp.sh
# Progressione personalizzata:
./run_leaf61_rain_ramp.sh --rain-waves 1,2,4,8,12
```

Default: raffiche da **1, 1, 2, 3, 4, 6, 8, 10, 12** pomodori, 47 in tutto.
La pausa casuale fra raffiche è di 2–3,5 secondi. Tutti i pomodori di una
raffica vengono rilasciati nello stesso passo fisico e alla stessa quota;
le posizioni casuali rispettano almeno 5 mm di distanza fra le superfici
delle sfere al momento del rilascio. L'intensità cresce quindi aumentando
il numero simultaneo, partendo da due rilasci singoli. Nuova pioggia riparte
dall'intensità minima con un nuovo seed. Il numero totale viene dedotto da
`--rain-waves`; massimo 12 per raffica e 100 per sequenza.

La modalità precedente `run_leaf61_rain.sh` rimane disponibile per la pioggia
singola. I contatti multipli restano una prova esplorativa, con i limiti
fisici descritti sopra.

Verifica della progressione: `artifacts/leaf61/rain-ramp-smoke`, 47/47 caduti,
71 foglie con contatti PhysX, nessun distacco o errore PhysX. `wave_audit.json`
verifica dai rilasci effettivi che ogni raffica parte nello stesso passo:
t=9,000; 11,142; 13,819; 15,915; 18,146; 21,196; 23,242; 25,694; 28,575 s.
Penetrazione massima PhysX 3,97 mm: esito ancora diagnostico, senza promuovere
il caso rispetto alla soglia controllata di 1 mm. Test unitari: 24 superati.
