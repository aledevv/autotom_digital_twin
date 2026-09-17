# Caduta dinamica su due foglie

Avvio dalla radice: `./run_leaf61_drop.sh`.

Palla dinamica da 5 g, raggio 10 mm; altezza iniziale sopra la superficie
superiore 60 mm. Due lamine reali seed 42, nominalmente 90 mm, con supporti
fissi, stessa massa e giunti del candidato skinnato. La seconda è traslata
di 75 mm in avanti e 60 mm verso il basso. Attrito statico 0,30, dinamico
0,25, restituzione 0,02. Solver CPU/PGS, 960 Hz, CCD abilitata.

Il rilascio avviene a t=9 s, dopo l'assestamento. Durante il volo non si
comandano pose, velocità o forze della palla. Il rallentamento GUI 0,25×
modifica soltanto la cadenza di esecuzione, non la dinamica. Le superfici visive
sono skinnate; il contatto avviene sui tre collider convessi della lamina.

## Verifica headless

Configurazione corrente: `artifacts/leaf61/drop-offset75/`, esito positivo.
La foglia inferiore è stata avanzata di altri 20 mm lungo base–punta.
L'intervallo fra primo e ultimo contatto passa da 22,9 ms a 53,1 ms (2,3×),
con 52 campioni di contatto a impulso positivo invece di 22. La flessione
massima dell'inferiore cresce a 25,77 mm; recupero finale 0,0270 mm,
nessuna penetrazione registrata nei suoi contatti, allungamento massimo 0,766%.
La superiore mantiene gli stessi risultati. Questa variante è verificata
headless; la revisione GUI della nuova posizione resta all'utente.

### Configurazione precedente: scostamento orizzontale 55 mm

Evidenza locale: `artifacts/leaf61/drop-03-960/`, con sorgenti congelati,
configurazione, scena USD, log, pose e contatti PhysX in `trace.npz`.

| Misura | Superiore | Inferiore |
| --- | ---: | ---: |
| Primo contatto dopo il rilascio | 0,1115 s | 0,3125 s |
| Flessione massima dall'equilibrio | 29,09 mm | 14,51 mm |
| Scostamento finale dall'equilibrio, ultimo secondo | 0,0104 mm | 0,0224 mm |
| Penetrazione massima registrata nei contatti | 0,0387 mm | 0 mm |
| Allungamento massimo degli spigoli visivi | 0,752% | 0,363% |

Contatti con impulso positivo nell'ordine richiesto, dati finiti, nessun
distacco o collasso, recupero di entrambe. Il contatto sulla seconda è breve:
la palla la urta e prosegue verso il piano, non vi rimane appoggiata.
La demo mostra quindi la risposta all'urto e al peso durante il contatto,
non un carico statico permanente. La revisione visiva dell'utente resta aperta.

La prova GUI automatica `artifacts/leaf61/drop-final-gui/` ha riprodotto
gli stessi contatti e valori numerici, con esito positivo e uscita regolare.
L'immagine finale verifica l'inquadratura delle due lamine; non documenta
da sola il passaggio della palla. Questa esecuzione non rallentata ha richiesto
27,15 s per 17 s simulati: non dimostra tempo reale a 960 Hz.
Verifiche del codice: 16 test superati, Ruff e controlli shell/diff superati.

Le prove preliminari `drop-01` e `drop-02` a 240 Hz mostravano circa 3,34 mm
di penetrazione al primo impatto. L'aggiunta della CCD speculativa non era
sufficiente; la prova a 960 Hz supera la soglia di 1 mm senza modificare
massa della palla o rigidità dei giunti. Il primo report precedeva l'aggiunta
di questa soglia e non va considerato un'accettazione della penetrazione.

Questa verifica riguarda la demo dinamica e non sostituisce il protocollo
quasi-statico né dimostra prestazioni di una pianta a 960 Hz.
