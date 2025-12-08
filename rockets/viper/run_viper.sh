#!/bin/bash
# ============================================================================
# LEEMON Distribution - Run Viper Example (Linux/Mac)
# ============================================================================

echo ""
echo "============================================================================"
echo "LEEMON V2 - Viper Rocket Simulation"
echo "============================================================================"
echo ""

# Check if Python is installed
if ! command -v python3 &> /dev/null; then
    echo "[ERROR] Python 3 not found"
    echo "Please install Python 3.7 or higher"
    exit 1
fi

echo "[OK] Python found: $(python3 --version)"
echo ""

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "Running Viper example (compiled code)..."
echo ""

python3 "$SCRIPT_DIR/run_viper_compiled.py"

if [ $? -eq 0 ]; then
    echo ""
    echo "[OK] Simulation completed successfully!"
    echo "Results saved to: examples/viper/results/"
else
    echo ""
    echo "[ERROR] Simulation failed!"
    exit 1
fi
