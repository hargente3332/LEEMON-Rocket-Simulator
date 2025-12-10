#!/usr/bin/env python3
"""
LEEMON V2 - Portable Distribution
Rocket Flight Simulator
"""

import sys
from pathlib import Path

__LEEMON_ROOT__ = Path(__file__).parent.resolve()

__PYTHON_FOLDER__ = __LEEMON_ROOT__ / "python"
if __PYTHON_FOLDER__.exists():
    sys.path.insert(0, str(__PYTHON_FOLDER__))
else:
    raise ImportError(f"Python folder not found at {__PYTHON_FOLDER__}")

try:
    from leemonSim import LEEMONSimulator, randomizeParam
    from analysisTools import FlightAnalyzer, VariabilityAnalyzer
except ImportError as e:
    raise ImportError(f"Failed to load LEEMON modules: {e}")

__version__ = "2.0.0"

__all__ = ['LEEMONSimulator', 'FlightAnalyzer', 'VariabilityAnalyzer', 'randomizeParam']

if __name__ == "__main__":
    print(f"LEEMON V2 Portable - Version {__version__}")
    print(f"Root: {__LEEMON_ROOT__}")
    print("\nUsage: python rockets/myRocket/mySimulation.py")
