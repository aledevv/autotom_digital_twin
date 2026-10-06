# Break-threshold probe (TGS/GPU)

Riproduttore minimo dell'anomalia della soglia di rottura dei giunti in PhysX TGS su GPU. Spiegazione e risultati in [REPORT.md](REPORT.md).

## Contenuto

- `probe_break_threshold.py`: una configurazione per processo; cerca per bisezione la massa che rompe un `FixedJoint` esterno e aggiunge una riga a un file jsonl.
- `run_matrix.sh`: lancia la matrice completa dei controlli (12 processi, circa 5 minuti con una GPU).
- `summarize.py`: stampa una tabella da un file di risultati.
- `results_isaac45.jsonl`: risultati ottenuti con Isaac Sim 4.5 (PhysX 5.5.1).

## Uso

Servono Isaac Sim con `python.sh` e una GPU CUDA. Per default si usa `~/isaacsim`.

```bash
export ISAACSIM_DIR="$HOME/isaacsim"   # cambia installazione se serve
src/experiments/break_threshold_probe/run_matrix.sh /tmp/mio_risultato.jsonl
python3 src/experiments/break_threshold_probe/summarize.py /tmp/mio_risultato.jsonl
```

Una singola configurazione:

```bash
~/isaacsim/python.sh src/experiments/break_threshold_probe/probe_break_threshold.py \
  --solver TGS --gpu 1 --iterations 32 --support articulation --output /tmp/uno.jsonl
```

Opzioni principali: `--solver TGS|PGS`, `--gpu 0|1`, `--iterations N`, `--support articulation|kinematic`, `--break-force`, `--hz`.

Risultato atteso su Isaac Sim 4.5: con TGS su GPU e supporto articolato la colonna `W*` vale circa `Fb/N`. In tutti gli altri casi vale circa `Fb`.

## Note

- Il giunto del frutto ha `excludeFromArticulation = 1`. Senza, il frutto viene incorporato nell'articolazione e il giunto non si rompe.
- Il launcher non adatta le API a Isaac Sim 6.1: eseguire su un'altra versione richiede di verificare che `omni.physx` e `isaacsim` espongano gli stessi moduli.
- Il file jsonl contiene anche la cronologia della bisezione (`history`).
