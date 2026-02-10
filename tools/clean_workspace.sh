#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-safe}"
YES_FLAG="${2:-}"

usage() {
  cat <<'EOF'
Usage: tools/clean_workspace.sh [safe|build|full] [--yes]

Modes:
  safe  - remove local caches/temp/log artifacts only (default)
  build - remove build/ and log/ plus safe artifacts
  full  - remove build/, install/, and log/ plus safe artifacts (requires --yes)
EOF
}

if [[ "${MODE}" == "-h" || "${MODE}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ "${MODE}" != "safe" && "${MODE}" != "build" && "${MODE}" != "full" ]]; then
  echo "Unknown mode: ${MODE}" >&2
  usage
  exit 1
fi

cd "${ROOT_DIR}"

echo "[clean] mode=${MODE} root=${ROOT_DIR}"

echo "[clean] removing python/cache/temp artifacts"
find . -type d -name '__pycache__' -prune -exec rm -rf {} +
find . -type d -name '.pytest_cache' -prune -exec rm -rf {} +
find . -type f \( -name '*.pyc' -o -name '*.pyo' -o -name '*.tmp' -o -name '*~' \) -delete

if [[ -d log ]]; then
  rm -rf log
fi

if [[ "${MODE}" == "build" || "${MODE}" == "full" ]]; then
  echo "[clean] removing build/"
  rm -rf build
fi

if [[ "${MODE}" == "full" ]]; then
  if [[ "${YES_FLAG}" != "--yes" ]]; then
    echo "[clean] full mode is destructive. Re-run with --yes." >&2
    exit 2
  fi
  echo "[clean] removing install/"
  rm -rf install
fi

echo "[clean] done"
