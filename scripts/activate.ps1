$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = Split-Path -Parent $ScriptDir
$ToolsDir = Join-Path $RepoRoot 'tools/arduino-cli'

$null = New-Item -ItemType Directory -Force -Path (Join-Path $RepoRoot '.arduino-data')
$null = New-Item -ItemType Directory -Force -Path (Join-Path $RepoRoot '.arduino-downloads')
$null = New-Item -ItemType Directory -Force -Path (Join-Path $RepoRoot '.arduino-user')
$null = New-Item -ItemType Directory -Force -Path (Join-Path $RepoRoot '.arduino-build-cache')

$env:ARDUINO_CONFIG_FILE = Join-Path $RepoRoot 'arduino-cli.yaml'
$env:ARDUINO_DIRECTORIES_DATA = Join-Path $RepoRoot '.arduino-data'
$env:ARDUINO_DIRECTORIES_DOWNLOADS = Join-Path $RepoRoot '.arduino-downloads'
$env:ARDUINO_DIRECTORIES_USER = Join-Path $RepoRoot '.arduino-user'
$env:ARDUINO_BUILD_CACHE_PATH = Join-Path $RepoRoot '.arduino-build-cache'

if (Test-Path (Join-Path $ToolsDir 'arduino-cli.exe')) {
    $env:PATH = "$ToolsDir;$env:PATH"
} elseif (Test-Path (Join-Path $ToolsDir 'arduino-cli')) {
    $env:PATH = "$ToolsDir;$env:PATH"
}

Write-Host "Activated repo-local Arduino CLI settings."
Write-Host "REPO_ROOT=$RepoRoot"
Write-Host "ARDUINO_CONFIG_FILE=$env:ARDUINO_CONFIG_FILE"
