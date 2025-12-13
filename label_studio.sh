#!/usr/bin/env bash
set -euo pipefail

LS_PORT="${LS_PORT:-8080}"
DATA_DIR="${DATA_DIR:-$HOME/labelstudio_data}"
PUBLIC_ACCESS="${PUBLIC_ACCESS:-false}"

if [[ "${PUBLIC_ACCESS}" == "true" ]]; then
  HOST_BIND="0.0.0.0"
else
  HOST_BIND="127.0.0.1"
fi

mkdir -p "${DATA_DIR}"
cd "${DATA_DIR}"

echo "Starting Label Studio (pip) ..."
echo "  DATA_DIR: ${DATA_DIR}"
echo "  URL     : http://localhost:${LS_PORT}"
echo "  bind    : ${HOST_BIND}:${LS_PORT}"

# 这里用 --data-dir 固定项目目录（避免乱写到别处）
label-studio start \
  --host "${HOST_BIND}" \
  --port "${LS_PORT}" \
  --data-dir "${DATA_DIR}"
