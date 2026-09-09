#!/usr/bin/env bash
# Generate the leaflet, then keep Isaac Sim open until its window is closed.
set -Eeuo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$repo_dir"

fail() {
    printf '\nERRORE: %s\n' "$*" >&2
    exit 1
}
trap 'printf "\nOperazione interrotta.\n" >&2; exit 130' INT
trap 'printf "\nOperazione terminata.\n" >&2; exit 143' TERM

isaac_python="${ISAAC_PYTHON:-$HOME/isaacsim/python.sh}"
command -v uv >/dev/null || fail "uv non trovato nel PATH."
command -v timeout >/dev/null || fail "Il comando timeout non è disponibile."
[[ -x "$isaac_python" ]] || fail "Launcher Isaac Sim non trovato: $isaac_python"
[[ -x /usr/bin/python3 ]] || fail "Python di sistema non trovato."
[[ -f external/real_leaves/src/tomato_leaf_generator/shape/gaussian_efd.py ]] ||
    fail "Submodule mancante. Esegui: git submodule update --init --recursive"

# Deactivate Conda inside this script only; the calling terminal is unchanged.
if [[ -n "${CONDA_PREFIX:-}" ]]; then
    conda_base="${CONDA_EXE:-}"
    if [[ -n "$conda_base" ]]; then
        conda_setup="$(dirname -- "$(dirname -- "$conda_base")")/etc/profile.d/conda.sh"
        if [[ -f "$conda_setup" ]]; then
            set +u
            source "$conda_setup"
            while [[ ${CONDA_SHLVL:-0} -gt 0 ]]; do
                conda deactivate || fail "Impossibile disattivare Conda."
            done
            set -u
        fi
    fi
    [[ -z "${CONDA_PREFIX:-}" ]] || fail "Esegui conda deactivate e riprova."
fi

printf '\n[1/2] Generazione foglia (timeout: 180 secondi)…\n'
if UV_CACHE_DIR="${UV_CACHE_DIR:-/tmp/uv-cache}" \
    PYTHONPATH="$repo_dir/external/real_leaves/src" \
    timeout --kill-after=10s 180s uv run --no-config --no-project \
        --python /usr/bin/python3 --default-index https://pypi.org/simple \
        --with triangle==20250106 --with usd-core --with numpy \
        python src/exporterV2/demos/generate_real_leaf_shape.py; then
    [[ -s src/exporterV2/demos/generated_leaflet.usda ]] || fail "Il generatore non ha prodotto il file USD."
else
    status=$?
    fail "Generazione fallita (codice $status; 124 indica timeout). Isaac Sim non è stato avviato."
fi

printf '\n[2/2] Apertura Isaac Sim. Chiudi la finestra per terminare.\n'
if env -u PYTHONEXE -u PYTHONHOME "$isaac_python" "$repo_dir/src/exporterV2/demos/load_real_leaf_shape.py"; then
    printf '\nIsaac Sim terminato.\n'
else
    status=$?
    fail "Isaac Sim è terminato con errore (codice $status). Consulta le righe precedenti."
fi
