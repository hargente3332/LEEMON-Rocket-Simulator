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
from leemonSim import ( # type: ignore
    LEEMONSimulator as Simulator,
    getDefaultConfig,
    randomizeParam
)

from analysisTools import ( # type: ignore
    FlightAnalyzer as Analyzer,
    VariabilityAnalyzer
)

__version__ = "2.0.0"
__author__ = "Hugo Argente"

# Aliases for easier use
SimulationEngine = Simulator
DataAnalyzer = Analyzer
MonteCarloAnalyzer = VariabilityAnalyzer

def quick_start():
    """
    Quick start guide for new users
    """
    print("="*70)
    print("LEEMON V2 - Quick Start Guide")
    print("="*70)
    print()
    print("1. Create a simulator:")
    print("   >>> from leemon import Simulator, getDefaultConfig")
    print("   >>> sim = Simulator()")
    print()
    print("2. Get default configuration:")
    print("   >>> config = getDefaultConfig()")
    print()
    print("3. Run a simulation:")
    print("   >>> sim.quickRun(config, outputFile='results/flight.csv')")
    print()
    print("4. Analyze results:")
    print("   >>> from leemon import Analyzer")
    print("   >>> analyzer = Analyzer('results/flight.csv')")
    print("   >>> analyzer.printSummary()")
    print("   >>> analyzer.plotFlightProfile()")
    print()
    print("="*70)

if __name__ == "__main__":
    quick_start()
