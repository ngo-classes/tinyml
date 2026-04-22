#!/usr/bin/env bash

# Detect the current script path in bash or zsh
if [ -n "${BASH_VERSION:-}" ]; then
  _SCRIPT_PATH="${BASH_SOURCE[0]}"
elif [ -n "${ZSH_VERSION:-}" ]; then
  _SCRIPT_PATH="${(%):-%N}"
else
  echo "Unsupported shell. Please use bash or zsh."
  return 1 2>/dev/null || exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "$_SCRIPT_PATH")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Prefer an explicit ARDUINO_CLI path, then PATH, then the repo-local tools folder.
if [[ -n "${ARDUINO_CLI:-}" ]]; then
  CLI="$ARDUINO_CLI"
elif command -v arduino-cli >/dev/null 2>&1; then
  CLI="$(command -v arduino-cli)"
elif [[ -x "$REPO_ROOT/tools/arduino-cli/arduino-cli" ]]; then
  CLI="$REPO_ROOT/tools/arduino-cli/arduino-cli"
elif [[ -x "$REPO_ROOT/tools/arduino-cli/arduino-cli.exe" ]]; then
  CLI="$REPO_ROOT/tools/arduino-cli/arduino-cli.exe"
else
  echo "ERROR: Could not find arduino-cli."
  echo "Place the binary at tools/arduino-cli/arduino-cli (or arduino-cli.exe),"
  echo "or add it to PATH, or set ARDUINO_CLI=/full/path/to/arduino-cli."
  exit 1
fi

export ARDUINO_CONFIG_FILE="$REPO_ROOT/arduino-cli.yaml"
export ARDUINO_DIRECTORIES_DATA="$REPO_ROOT/.arduino-data"
export ARDUINO_DIRECTORIES_DOWNLOADS="$REPO_ROOT/.arduino-downloads"
export ARDUINO_DIRECTORIES_USER="$REPO_ROOT/.arduino-user"
export ARDUINO_BUILD_CACHE_PATH="$REPO_ROOT/.arduino-build-cache"

mkdir -p \
  "$ARDUINO_DIRECTORIES_DATA" \
  "$ARDUINO_DIRECTORIES_DOWNLOADS" \
  "$ARDUINO_DIRECTORIES_USER" \
  "$ARDUINO_BUILD_CACHE_PATH"

CORE="arduino:mbed_nano"
LIBRARIES=(
  "ArduinoBLE"
  "Arduino_LSM9DS1"
  "Harvard_TinyMLx"
)

run_cli() {
  echo "+ $CLI $*"
  "$CLI" "$@"
}

have_core() {
  "$CLI" core list | grep -q '^arduino:mbed_nano\b'
}

have_library() {
  local lib="$1"
  "$CLI" lib list | grep -Fq "$lib"
}

echo "Using Arduino CLI: $CLI"
run_cli version

echo
echo "==> Updating package indexes"
run_cli core update-index
run_cli lib update-index

echo
echo "==> Installing board core: $CORE"
if have_core; then
  echo "Core already installed: $CORE"
else
  run_cli core install "$CORE"
fi

echo
echo "==> Installing libraries"
for lib in "${LIBRARIES[@]}"; do
  if have_library "$lib"; then
    echo "Library already installed: $lib"
  else
    run_cli lib install "$lib"
  fi
done

echo
echo "==> Installed cores"
run_cli core list

echo
echo "==> Installed libraries"
run_cli lib list

echo
echo "Bootstrap complete."
echo "Common FQBN for Nano 33 BLE / BLE Sense: arduino:mbed_nano:nano33ble"
echo "List detected boards with: $CLI board list"
echo "List all matching board names with: $CLI board listall | grep -i 'Nano 33 BLE'"
