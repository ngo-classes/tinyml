$ErrorActionPreference = 'Stop'

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = Split-Path -Parent $ScriptDir

# Prefer an explicit ARDUINO_CLI path, then PATH, then the repo-local tools folder.
if ($env:ARDUINO_CLI) {
    $Cli = $env:ARDUINO_CLI
} elseif (Get-Command arduino-cli -ErrorAction SilentlyContinue) {
    $Cli = (Get-Command arduino-cli).Source
} elseif (Test-Path (Join-Path $RepoRoot 'tools/arduino-cli/arduino-cli.exe')) {
    $Cli = Join-Path $RepoRoot 'tools/arduino-cli/arduino-cli.exe'
} elseif (Test-Path (Join-Path $RepoRoot 'tools/arduino-cli/arduino-cli')) {
    $Cli = Join-Path $RepoRoot 'tools/arduino-cli/arduino-cli'
} else {
    Write-Error "Could not find arduino-cli. Place arduino-cli.exe in tools/arduino-cli/, add it to PATH, or set `$env:ARDUINO_CLI."
}

$env:ARDUINO_CONFIG_FILE = Join-Path $RepoRoot 'arduino-cli.yaml'
$env:ARDUINO_DIRECTORIES_DATA = Join-Path $RepoRoot '.arduino-data'
$env:ARDUINO_DIRECTORIES_DOWNLOADS = Join-Path $RepoRoot '.arduino-downloads'
$env:ARDUINO_DIRECTORIES_USER = Join-Path $RepoRoot '.arduino-user'
$env:ARDUINO_BUILD_CACHE_PATH = Join-Path $RepoRoot '.arduino-build-cache'

@(
    $env:ARDUINO_DIRECTORIES_DATA,
    $env:ARDUINO_DIRECTORIES_DOWNLOADS,
    $env:ARDUINO_DIRECTORIES_USER,
    $env:ARDUINO_BUILD_CACHE_PATH
) | ForEach-Object {
    if (-not (Test-Path $_)) {
        New-Item -ItemType Directory -Force -Path $_ | Out-Null
    }
}

$Core = 'arduino:mbed_nano'
$Libraries = @(
    'ArduinoBLE',
    'Arduino_LSM9DS1',
    'Harvard_TinyMLx'
)

function Invoke-ArduinoCli {
    param([Parameter(ValueFromRemainingArguments = $true)][string[]]$Args)
    Write-Host "+ $Cli $($Args -join ' ')"
    & $Cli @Args
}

function Test-CoreInstalled {
    $out = & $Cli core list
    return ($out | Select-String -Pattern '^arduino:mbed_nano\b' -Quiet)
}

function Test-LibraryInstalled {
    param([string]$Name)
    $out = & $Cli lib list
    return ($out | Select-String -SimpleMatch $Name -Quiet)
}

Write-Host "Using Arduino CLI: $Cli"
Invoke-ArduinoCli version

Write-Host "`n==> Updating package indexes"
Invoke-ArduinoCli core update-index
Invoke-ArduinoCli lib update-index

Write-Host "`n==> Installing board core: $Core"
if (Test-CoreInstalled) {
    Write-Host "Core already installed: $Core"
} else {
    Invoke-ArduinoCli core install $Core
}

Write-Host "`n==> Installing libraries"
foreach ($lib in $Libraries) {
    if (Test-LibraryInstalled -Name $lib) {
        Write-Host "Library already installed: $lib"
    } else {
        Invoke-ArduinoCli lib install $lib
    }
}

Write-Host "`n==> Installed cores"
Invoke-ArduinoCli core list

Write-Host "`n==> Installed libraries"
Invoke-ArduinoCli lib list

Write-Host "`nBootstrap complete."
Write-Host "Common FQBN for Nano 33 BLE / BLE Sense: arduino:mbed_nano:nano33ble"
Write-Host "List detected boards with: $Cli board list"
Write-Host "List all matching board names with: $Cli board listall | findstr /I \"Nano 33 BLE\""
