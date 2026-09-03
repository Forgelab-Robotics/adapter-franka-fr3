#!/usr/bin/env bash

set -euo pipefail

if [[ "${PAOS_FRANKA_REAL_ACK:-}" != "I_UNDERSTAND_THIS_MOVES_FR3" ]]; then
  echo "拒绝启动 FR3 真机节点：请现场完成安全检查后设置" >&2
  echo "PAOS_FRANKA_REAL_ACK=I_UNDERSTAND_THIS_MOVES_FR3" >&2
  exit 2
fi

exec "$(dirname "$0")/run_franka_driver.sh" "$@"
