#!/usr/bin/env python3
"""
LEEMON V2 - Commercial Distribution
Rocket Flight Simulator Interface

Usage:
    from leemon import Simulator, Analyzer, getDefaultConfig
"""

import sys
from pathlib import Path

# Add compiled modules to path
lib_path = Path(__file__).parent / "lib"
sys.path.insert(0, str(lib_path))

# Import from compiled bytecode
from leemonSim import (
    LEEMONSimulator as Simulator,
    getDefaultConfig,
    randomizeParam
)

from analysisTools import (
    FlightAnalyzer as Analyzer,
    VariabilityAnalyzer
)

__version__ = "2.0.0"
__author__ = "Hugo Argente"

# Aliases for easier use
SimulationEngine = Simulator
DataAnalyzer = Analyzer
MonteCarloAnalyzer = VariabilityAnalyzer


