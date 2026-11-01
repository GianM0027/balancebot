#!/usr/bin/env bash
# Setup dell'ambiente di simulazione per il self-balancing robot (WSL2 / Ubuntu).
# Uso:  bash setup_wsl.sh
set -euo pipefail

VENV="$HOME/venvs/balancebot"

echo "== 1/4 Pacchetti di sistema =="
sudo apt-get update
sudo apt-get install -y python3 python3-venv python3-pip git build-essential \
    libgl1 libglfw3 libegl1   # librerie OpenGL per il viewer di MuJoCo (WSLg)

echo "== 2/4 Ambiente Python in $VENV =="
python3 -m venv "$VENV"
# shellcheck disable=SC1091
source "$VENV/bin/activate"
pip install --upgrade pip wheel

echo "== 3/4 Librerie =="
# Calcolo e grafici (il simulatore lo scriviamo noi con numpy)
pip install numpy scipy matplotlib jupyterlab sympy pytest
# MuJoCo: secondo simulatore, per confrontarlo con il nostro
pip install mujoco
# JAX con CUDA: per MJX e per l'RL su GPU piu' avanti
pip install -U "jax[cuda12]"

echo "== 4/4 GPU =="
if command -v nvidia-smi >/dev/null 2>&1; then
    nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv
else
    echo "ATTENZIONE: nvidia-smi non trovato. Aggiorna il driver NVIDIA su Windows (non in WSL)."
fi

grep -q "venvs/balancebot" "$HOME/.bashrc" || \
    echo "alias bb='source $VENV/bin/activate'" >> "$HOME/.bashrc"

echo
echo "Fatto. Apri un nuovo terminale, scrivi 'bb' per attivare l'ambiente, poi:"
echo "    python environment/check_env.py"
