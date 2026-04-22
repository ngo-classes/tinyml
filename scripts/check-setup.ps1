if (-not (Get-Command arduino-cli -ErrorAction SilentlyContinue)) {
    Write-Host 'arduino-cli not found in PATH.'
    Write-Host 'Run: .\scripts\activate.ps1'
    exit 1
}

Write-Host '=== Arduino CLI version ==='
arduino-cli version
Write-Host ''
Write-Host '=== Active config ==='
arduino-cli config dump --verbose
