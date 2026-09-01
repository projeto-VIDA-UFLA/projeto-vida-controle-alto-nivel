#!/usr/bin/env bash
# Inicializa os submodules do projeto (ex: ros_ws/src/robot_state_ekf) e configura este clone
# para manter os submodules atualizados automaticamente em `git pull`/`checkout`.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

echo "Inicializando submodules..."
git submodule update --init --recursive

git config submodule.recurse true

echo "Pronto. Submodules inicializados e submodule.recurse=true configurado neste clone."
