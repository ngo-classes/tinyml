#!/usr/bin/env bash

# Detect the current script path in bash or zsh
if [ -n "${BASH_VERSION:-}" ]; then
  _SCRIPT_PATH="${BASH_SOURCE[0]}"
elif [ -n "${ZSH_VERSION:-}" ]; then
  _SCRIPT_PATH="${(%):-%x}"
else
  echo "Unsupported shell. Please use bash or zsh."
  return 1 2>/dev/null || exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "$_SCRIPT_PATH")" && pwd)"
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
