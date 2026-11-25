#!/usr/bin/env bash
set -eo pipefail
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "${WS_DIR}/scripts/remote_web.py" "$@"
