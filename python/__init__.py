"""
LEEMON V2 - Python Package for Rocket Simulation

This package provides Python interfaces to pre-compiled LEEMON simulation engine.

Main Components:
- LEEMONSimulator: Main simulator class with compiled binary support
- LEEMONGui: Tkinter-based GUI for rocket design and simulation
- FlightAnalyzer: Post-processing and analysis tools

Usage:
    from py import LEEMONSimulator
    
    sim = LEEMONSimulator(use_compiled=True)
    config = sim.loadConfigFromFile("config.txt")
    results = sim.runSimulation(config)

Note:
    This package includes pre-compiled binaries (leemon.exe, libleemon.dll)
    in the same directory. No compilation is needed!

For details, see README_COMPILED.md
"""

__version__ = "2.0"
__author__ = "LEEMON Project"

from .leemonSim import LEEMONSimulator
from .analysisTools import FlightAnalyzer, VariabilityAnalyzer

__all__ = [
    'LEEMONSimulator',
    'FlightAnalyzer',
    'VariabilityAnalyzer',
]
