#!/usr/bin/env python3
"""
Full Configuration Simulation with Parallel Processing Support
Define ALL parameters here for your rocket simulation
"""

import sys
from pathlib import Path
from multiprocessing import freeze_support

# Add LEEMON root to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from leemon import LEEMONSimulator, FlightAnalyzer, VariabilityAnalyzer, randomizeParam

def main():
    """
    Main function - REQUIRED for Windows multiprocessing
    All simulation code must be inside this function or protected by if __name__ == '__main__'
    """
    # ============================================================================
    # CREATE SIMULATOR
    # ============================================================================

    sim = LEEMONSimulator()

    # ============================================================================
    # DEFINE ALL PARAMETERS
    # ============================================================================

    config = {}

    config = sim.loadConfigFromFile("rockets/viper/viper.txt")
               
    # Wind 
    config["windSpeed"] = 0.0               
    config["windAngle"] = 0.0 

    # ============================================================================
    # LAUNCH CONDITIONS
    # ============================================================================
    config["railLength"] = 6.0              # Launch rail length [m]
    config["railAngle"] = 84.0              # Launch rail angle [degrees] (90° = vertical)

    # Initial position in NED frame [m]
    config["initialPosDown"] = 0.0          # Initial Down position [m] (0 = ground level)

    # ============================================================================
    # VARIABILITY ANALYSIS WITH PARALLEL PROCESSING
    # ============================================================================
    
    print("\n" + "="*70)
    print("VIPER ROCKET - MONTE CARLO ANALYSIS")
    print("="*70)
    sim.quickRun(config, outputFile=config["outputFile"])
    # Define parameters to vary
    param_dict = {
        "massEmpty": [9.0,10.0,11.0,15.0],
        "railAngle": randomizeParam(84.0, 1.0, 2)
    }
     
    # Run the analysis
    sim.variabilityAnalysis(
        baseConfig=config,
        paramDict=param_dict,
        compileFirst=True,
        verbose=True,
        n_jobs=None  # Use all available cores
    )
    
    n_jobs = 1
    # Cargar datos de un solo vuelo
    analyzer = FlightAnalyzer('rockets/viper/results/viperData.csv')
    analyzer.plot("altitude","qInf")

    # Crear analizador de variabilidad
    var_analyzer = VariabilityAnalyzer('rockets/viper/results')

    # Cargar todas las simulaciones que coincidan con el patrón
    var_analyzer.loadAllSimulations('viperData_*.csv')
    var_analyzer.printComparisonTable()

    var_analyzer.plotComparison("massEmpty", "apogee", color_param= "railAngle",legend_position = "outside")











# ============================================================================
# CRITICAL: Required for Windows multiprocessing
# ============================================================================
if __name__ == '__main__':
    freeze_support()  # Required for Windows when creating executables
    main()