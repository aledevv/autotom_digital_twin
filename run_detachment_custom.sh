cd /home/alessandro/isaacsim/autotom_digital_twin

UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python src/experiments/detachable_fruit_v2/run_experiment.py \
  --run-dir "$(mktemp -d artifacts/detachable_fruit_v2/custom-25n-gui-XXXXXX)" \
  --source-usd artifacts/detachable_fruit_v2/2026-09-09/baseline-full/source.usda \
  --scenario full --gui --cpu --solver PGS --hz 60 \
  --art-position 32 --art-velocity 0 --fruit-position 255 --fruit-velocity 0 \
  --mouse-grab-mode bounded --drag-slew-rate 4.8 --drag-damping 1 \
  --retain-fruit-grip --break-force 2.5 --duration 60 --acceptance functional