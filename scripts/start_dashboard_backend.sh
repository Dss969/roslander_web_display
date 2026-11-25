#!/usr/bin/env bash
set -eo pipefail
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME_DIR="${WS_DIR}/.runtime/dashboard"
mkdir -p "${RUNTIME_DIR}"
chmod 700 "${RUNTIME_DIR}"
exec 9>"${RUNTIME_DIR}/instance.lock"
if ! flock -n 9; then
  echo '仿真 Dashboard 已在运行，请检查 python3 scripts/dashboard_service.py status' >&2
  exit 1
fi

source "${WS_DIR}/scripts/workspace_env.sh"
load_workspace
cd "${WS_DIR}"
exec python3 -u "${WS_DIR}/dashboard/server.py" "$@" --mode simulation --host 127.0.0.1 --port 8089
