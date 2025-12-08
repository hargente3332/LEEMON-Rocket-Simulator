@echo off
REM ============================================================================
REM LEEMON Distribution - Run Viper Example
REM ============================================================================
REM This script runs the Viper rocket simulation example
REM No compilation needed - uses pre-compiled binaries!
REM ============================================================================

echo.
echo ============================================================================
echo LEEMON V2 - Viper Rocket Simulation
echo ============================================================================
echo.
echo Location: %~dp0
echo.

REM Check if Python is installed
python --version >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Python not found in PATH
    echo Please install Python 3.7 or higher
    echo https://www.python.org/downloads/
    pause
    exit /b 1
)

echo [OK] Python found
echo.

REM Run the simulation
echo Running Viper example...
echo.

python "%~dp0viper_run.py"

if errorlevel 1 (
    echo.
    echo [ERROR] Simulation failed!
    pause
    exit /b 1
)

echo.
echo [OK] Simulation completed successfully!
echo Results saved to: examples\viper\results\
pause
