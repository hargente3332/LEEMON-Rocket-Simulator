# ============================================================================
# LEEMON Distribution - Viper Example (PowerShell)
# ============================================================================
# Usage: powershell -ExecutionPolicy Bypass -File run_viper.ps1

$ScriptPath = Split-Path -Parent $MyInvocation.MyCommand.Path
$DistRoot = Split-Path -Parent (Split-Path -Parent $ScriptPath)

Write-Host ""
Write-Host "============================================================================"
Write-Host "LEEMON V2 - Viper Rocket Simulation"
Write-Host "============================================================================"
Write-Host ""
Write-Host "Distribution: $DistRoot"
Write-Host ""

# Check Python
Write-Host "Checking Python..."
$PythonPath = (Get-Command python -ErrorAction SilentlyContinue).Source

if (-not $PythonPath) {
    Write-Host "[ERROR] Python not found in PATH"
    Write-Host "Please install Python 3.7+ from https://www.python.org/downloads/"
    Read-Host "Press Enter to continue"
    exit 1
}

$PythonVersion = python --version 2>&1
Write-Host "[OK] $PythonVersion"
Write-Host ""

# Run simulation
Write-Host "Running Viper example from: $ScriptPath"
Write-Host ""

# Run simulation
Write-Host "Running Viper example from: $ScriptPath"
Write-Host ""

$SimScript = Join-Path $ScriptPath "viper_run.py"

Push-Location $DistRoot
try {
    python $SimScript
    $ExitCode = $LASTEXITCODE
}
finally {
    Pop-Location
}

Write-Host ""
if ($ExitCode -eq 0) {
    Write-Host "[OK] Simulation completed successfully!"
    Write-Host "Results saved to: examples/viper/results/"
} else {
    Write-Host "[ERROR] Simulation failed with exit code: $ExitCode"
}

Read-Host "Press Enter to exit"
