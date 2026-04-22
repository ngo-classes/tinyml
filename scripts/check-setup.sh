#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

if ! command -v arduino-cli >/dev/null 2>&1; then
  echo "arduino-cli not found in PATH."
  echo "Run: source scripts/activate.sh"
  exit 1
fi

echo "=== Arduino CLI version ==="
arduino-cli version

echo
echo "=== Active config ==="
arduino-cli config dump --verbose

echo
echo "=== Repo root ==="
echo "$REPO_ROOT"
