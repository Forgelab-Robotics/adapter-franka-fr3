#!/usr/bin/env bash

set -euo pipefail

project="${FRANKA_FR3_PROJECT:?必须设置 FRANKA_FR3_PROJECT，指向 franka_fr3 仓库根目录}"
python_bin="${project}/.venv/bin/python"
node="${project}/src/robots_franka_fr3/node.py"

if [[ ! -x "${python_bin}" ]]; then
  echo "FR3 Python 环境不存在或不可执行：${python_bin}" >&2
  exit 2
fi
if [[ ! -f "${node}" ]]; then
  echo "FR3 Dora 节点不存在：${node}" >&2
  exit 2
fi

exec "${python_bin}" "${node}" "$@"
