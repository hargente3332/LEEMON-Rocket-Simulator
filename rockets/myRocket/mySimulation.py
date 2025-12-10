#!/usr/bin/env python3
"""
My Rocket Simulation - Example Template
Modify this file to create your own rocket simulation!
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
    Main function for your rocket simulation
    """
    
    # ============================================================================
    # INITIALIZE SIMULATOR
    # ============================================================================
    ROOT_DIR = os.getcwd()
    sim = LEEMONSimulator(ROOT_DIR)

   # ============================================================================
    # LOAD CONFIGURATION
    # ============================================================================
    config = sim.loadConfigFromFile("rockets/myRocket/example.txt")
               
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
    print("MY ROCKET ROCKET - SINGLE FLIGHT SIMULATION")
    print("="*70)
    
    sim.quickRun(config, outputFile=config["outputFile"], compileFirst=False)

    # ============================================================================
    # RUN VARIABILITY ANALYSIS (Monte Carlo)
    # ============================================================================
    print("\n" + "="*70)
    print("MY ROCKET ROCKET - MONTE CARLO VARIABILITY ANALYSIS")
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
        analyzer = FlightAnalyzer('rockets/myRocket/results/myRocketData.csv')
        analyzer.plot("altitude", "qInf")
        print("✓ Single flight plot generated")
    except Exception as e:
        print(f"Warning: Could not plot single flight: {e}")

    # Load variability analysis data
    try:
        var_analyzer = VariabilityAnalyzer('rockets/myRocket/results')
        var_analyzer.loadAllSimulations('myRocketData_*.csv')
        var_analyzer.printComparisonTable()
        var_analyzer.plotComparison("massEmpty", "apogee", color_param="railAngle", 
                                   legend_position="outside")
        print("✓ Variability analysis plots generated")
    except Exception as e:
        print(f"Warning: Could not generate analysis plots: {e}")




# ============================================================================
# CRITICAL: Required for Windows multiprocessing
# ============================================================================
if __name__ == '__main__':
    freeze_support()
    main()
