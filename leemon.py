#!/usr/bin/env python3
"""
LEEMON V2 - Portable Distribution
Rocket Flight Simulator

This wrapper provides easy access to LEEMON functionality
without exposing the source code.
"""

import sys
from pathlib import Path

# Get the directory where this script is located
__LEEMON_ROOT__ = Path(__file__).parent.resolve()

# Add python folder to path (contains compiled .pyc files)
__PYTHON_FOLDER__ = __LEEMON_ROOT__ / "python"
if __PYTHON_FOLDER__.exists():
    sys.path.insert(0, str(__PYTHON_FOLDER__))
else:
    raise ImportError(
        f"LEEMON installation error: python/ folder not found at {__PYTHON_FOLDER__}\n"
        f"Please ensure the LEEMON-Portable folder structure is intact."
    )

try:
    # Import from compiled bytecode
    from leemonSim import LEEMONSimulator, randomizeParam
    from analysisTools import FlightAnalyzer, VariabilityAnalyzer
except ImportError as e:
    raise ImportError(
        f"Failed to load LEEMON modules: {e}\n"
        f"Python folder: {__PYTHON_FOLDER__}\n"
        f"Available files: {list(__PYTHON_FOLDER__.glob('*.pyc'))}"
    )

__version__ = "2.0.0"
__author__ = "Hugo Argente - LEEM UPM"

# Export main classes
__all__ = [
    'LEEMONSimulator',
    'FlightAnalyzer', 
    'VariabilityAnalyzer',
    'randomizeParam'
]

def get_version():
    """Get LEEMON version"""
    return __version__

def get_root_path():
    """Get LEEMON installation root path"""
    return __LEEMON_ROOT__

if __name__ == "__main__":
    print("="*70)
    print(f"LEEMON V2 - Portable Distribution v{__version__}")
    print("="*70)
    print()
    print(f"Installation root: {__LEEMON_ROOT__}")
    print(f"Python modules: {__PYTHON_FOLDER__}")
    print()
    print("Quick Start:")
    print("  1. Navigate to rockets/myRocket/")
    print("  2. Run: python mySimulation.py")
    print()
    print("For more examples, see rockets/myRocket/ folder")
    print("="*70)
