#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TOOLS_DIR="$REPO_ROOT/tools/arduino-cli"

mkdir -p \
  "$REPO_ROOT/.arduino-data" \
  "$REPO_ROOT/.arduino-downloads" \
  "$REPO_ROOT/.arduino-user" \
  "$REPO_ROOT/.arduino-build-cache"

export ARDUINO_CONFIG_FILE="$REPO_ROOT/arduino-cli.yaml"
export ARDUINO_DIRECTORIES_DATA="$REPO_ROOT/.arduino-data"
export ARDUINO_DIRECTORIES_DOWNLOADS="$REPO_ROOT/.arduino-downloads"
export ARDUINO_DIRECTORIES_USER="$REPO_ROOT/.arduino-user"
export ARDUINO_BUILD_CACHE_PATH="$REPO_ROOT/.arduino-build-cache"

if [[ -x "$TOOLS_DIR/arduino-cli" ]]; then
  export PATH="$TOOLS_DIR:$PATH"
elif [[ -f "$TOOLS_DIR/arduino-cli.exe" ]]; then
  export PATH="$TOOLS_DIR:$PATH"
fi

echo "Activated repo-local Arduino CLI settings."
echo "REPO_ROOT=$REPO_ROOT"
echo "ARDUINO_CONFIG_FILE=$ARDUINO_CONFIG_FILE"
