Charts good enough per l'uso qui (dati di sintesi tecnica). Procedo con il documento.
## Panoramica dei dati analizzati
Ho analizzato i tre CSV (day 1, 60, 160) che descrivono l'architettura del graph-model della pianta di pomodoro, estraendo diametri, lunghezze e rapporti dimensionali per stelo (Internode), picciolo (petiole), rachide (rachis) e frutti. Il numero di organi cresce da 9 (day 1) a 73 (day 60/160), segno che la topologia si stabilizza dopo il giorno 60 mentre le dimensioni continuano a variare marginalmente. 
## Diametri e lunghezze medie per stadio
| Organo / metrica | Day 1 | Day 60 | Day 160 |
|---|---|---|---|
| Diametro stelo (mm) | 6.00 (range 5.42–6.30) | 7.90 (range 6.44–8.74) | 7.94 (range 6.44–8.74) |
| Lunghezza internodo (mm) | 8.02 | 28.67 | 28.97 |
| Diametro picciolo (mm) | 3.13 (range 2.67–3.75) | 4.62 (range 2.74–5.70) | 4.64 (range 2.74–5.70) |
| Lunghezza picciolo (mm) | 16.69 | 27.07 | 27.22 |
| Lunghezza rachide (mm) | 28.48 | 73.55 | 77.04 |
| Raggio frutto (mm) | — | 0–13.64 (in formazione) | 11.49–13.64 (maturo) |

Il diametro dello stelo cresce del 32% tra day 1 e day 60 ma resta praticamente costante tra day 60 e day 160 (+0.5%), mentre la lunghezza degli internodi triplica nello stesso intervallo iniziale. Il picciolo segue un pattern simile, con diametro che aumenta del 47% e lunghezza del 62% nella prima fase, per poi stabilizzarsi. 
## Rapporti proporzionali (aspect ratio)
Il rapporto lunghezza/diametro (L/D) è la metrica più critica per Isaac Sim, perché determina se un organo può essere rappresentato come rigid body cilindrico senza artefatti di collisione o instabilità PhysX.

Rapporti proporzionali (aspect ratio)

Il rapporto lunghezza/diametro (L/D) è la metrica più critica per Isaac Sim, perché determina se un organo può essere rappresentato come rigid body cilindrico senza artefatti di collisione o instabilità PhysX.