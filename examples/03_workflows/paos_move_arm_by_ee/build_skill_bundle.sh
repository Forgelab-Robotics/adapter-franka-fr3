#!/usr/bin/env bash

set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project="$(cd "${root}/../../.." && pwd)"
phyagentos="${PHYAGENTOS_ROOT:-$(cd "${root}/../../../../../PhyAgentOS" && pwd)}"
python_bin="${PAOS_PYTHON:-${phyagentos}/.venv/bin/python}"

if [[ ! -x "${python_bin}" || ! -f "${phyagentos}/scripts/package_skill.py" ]]; then
  echo "找不到 PhyAgentOS 打包环境；请设置 PHYAGENTOS_ROOT 或 PAOS_PYTHON" >&2
  exit 2
fi

exec "${python_bin}" "${phyagentos}/scripts/package_skill.py" \
  "${root}/skill" --output-dir "${project}/dist/skills" --force
