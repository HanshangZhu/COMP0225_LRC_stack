#!/usr/bin/env bash
set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="$(cd "${PKG_DIR}/../.." && pwd)"

python3 "${PKG_DIR}/tools/generate_ros2_tare.py" "$@"

if ! git -C "${WS_DIR}" diff --quiet -- "${PKG_DIR}/generated/tare_planner" "${PKG_DIR}/UPSTREAM_MANIFEST.json"; then
  echo "Regeneration drift detected in generated/tare_planner or UPSTREAM_MANIFEST.json" >&2
  git -C "${WS_DIR}" --no-pager diff -- "${PKG_DIR}/generated/tare_planner" "${PKG_DIR}/UPSTREAM_MANIFEST.json"
  exit 1
fi

echo "Regeneration check passed."
