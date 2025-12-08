#!/usr/bin/env python3
"""
VIPER Rocket Simulation - Pre-compiled Distribution
Ready to run without compilation needed!
"""

import sys
import os
sys.path.append("python")

from leemonSim import LEEMONSimulator, randomizeParam
from leemonSim import FlightAnalyzer, VariabilityAnalyzer
import os
from multiprocessing import freeze_support

def main():
    """
    Main function for VIPER simulation
    All simulation code must be inside this function for Windows multiprocessing
    """
    
    # ============================================================================
    # INITIALIZE SIMULATOR (no compilation needed - uses pre-compiled leemon.exe)
    # ============================================================================
    ROOT_DIR = os.getcwd()
    sim = LEEMONSimulator(ROOT_DIR)

    # ============================================================================
    # LOAD CONFIGURATION
    # ============================================================================
    config = sim.loadConfigFromFile("rockets/viper/viper.txt")
               
    # Override specific parameters if needed
    config["windSpeed"] = 0.0               
    config["windAngle"] = 0.0 
    config["railLength"] = 6.0              # Launch rail length [m]
    config["railAngle"] = 84.0              # Launch rail angle [degrees)
    config["initialPosDown"] = 0.0          # Initial Down position [m]

    # ============================================================================
    # RUN SINGLE SIMULATION
    # ============================================================================
    print("\n" + "="*70)
    print("VIPER ROCKET - SINGLE FLIGHT SIMULATION")
    print("="*70)
    
    sim.quickRun(config, outputFile="rockets/viper/results/viperData.csv", compileFirst=False)

    # ============================================================================
    # RUN VARIABILITY ANALYSIS (Monte Carlo)
    # ============================================================================
    print("\n" + "="*70)
    print("VIPER ROCKET - MONTE CARLO VARIABILITY ANALYSIS")
    print("="*70)
    
    # Define parameters to vary
    param_dict = {
        "massEmpty": [9.0, 10.0, 11.0, 15.0],
        "railAngle": randomizeParam(84.0, 1.0, 2)
    }
     
    # Run parallel analysis (uses all CPU cores)
    sim.variabilityAnalysis(
        baseConfig=config,
        paramDict=param_dict,
        compileFirst=False,  # USE PRE-COMPILED EXECUTABLE
        verbose=True,
        n_jobs=None  # Use all available cores
    )

    # ============================================================================
    # ANALYZE RESULTS
    # ============================================================================
    print("\n" + "="*70)
    print("ANALYZING RESULTS")
    print("="*70)
    
    # Load single flight data
    try:
        analyzer = FlightAnalyzer('rockets/viper/results/viperData.csv')
        analyzer.plot("altitude", "qInf")
        print("✓ Single flight plot generated")
    except Exception as e:
        print(f"Warning: Could not plot single flight: {e}")

    # Load variability analysis data
    try:
        var_analyzer = VariabilityAnalyzer('rockets/viper/results')
        var_analyzer.loadAllSimulations('viperData_*.csv')
        var_analyzer.printComparisonTable()
        var_analyzer.plotComparison("massEmpty", "apogee", color_param="railAngle", 
                                   legend_position="outside")
        print("✓ Variability analysis plots generated")
    except Exception as e:
        print(f"Warning: Could not generate analysis plots: {e}")

    print("\n" + "="*70)
    print("SIMULATION COMPLETE!")
    print("Results saved to: rockets/viper/results/")
    print("="*70)


# ============================================================================
# CRITICAL: Required for Windows multiprocessing
# ============================================================================
if __name__ == '__main__':
    freeze_support()
    main()
