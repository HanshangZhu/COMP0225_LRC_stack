#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

echo "== Git Status =="
git status --short

echo
echo "== Artifact Sizes =="
for dir in build install log; do
  if [[ -d "${dir}" ]]; then
    du -sh "${dir}"
  else
    echo "${dir}: (missing)"
  fi
done

echo
echo "== Top Workspace Dirs (depth 1) =="
du -sh ./* 2>/dev/null | sort -h
