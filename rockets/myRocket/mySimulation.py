#!/usr/bin/env python3
"""
Full Configuration Simulation with Parallel Processing Support
Define ALL parameters here for your rocket simulation
"""
import os
from leemonSim import LEEMONSimulator, randomizeParam # type: ignore
from leemonSim import FlightAnalyzer, VariabilityAnalyzer # type: ignore
from multiprocessing import freeze_support

def main():
    """
    Main function - REQUIRED for Windows multiprocessing
    All simulation code must be inside this function or protected by if __name__ == '__main__'
    """
    # ============================================================================
    # CREATE SIMULATOR
    # ============================================================================
    ROOT_DIR = os.getcwd() # This is your Project Root

    sim = LEEMONSimulator(ROOT_DIR)

    # ============================================================================
    # DEFINE ALL PARAMETERS
    # ============================================================================

    config = {}

    config = sim.loadConfigFromFile("rockets/myRocket/example.txt")
               
    # Wind 
    config["windSpeed"] = 5.0               
    config["windAngle"] = 20.0 

    # ============================================================================
    # LAUNCH CONDITIONS
    # ============================================================================
    config["railLength"] = 6.0              # Launch rail length [m]
    config["railAngle"] = 84.0              # Launch rail angle [degrees] (90° = vertical)

    # Initial position in NED frame [m]
    config["initialPosDown"] = 0.0          # Initial Down position [m] (0 = ground level)

    # ============================================================================
    # QUICK SIMULATION
    # ============================================================================
 
    sim.quickRun(config, outputFile=config["outputFile"],compileFirst=False)

    # ============================================================================
    # VARIABILITY ANALYSIS WITH PARALLEL PROCESSING
    # ============================================================================
    
    print("\n" + "="*70)
    print("MY ROCKET - MONTE CARLO ANALYSIS")
    print("="*70)
    
    # Define parameters to vary
    param_dict = {
        "massEmpty": randomizeParam(10.0, 0.50, 2),
        "railAngle": randomizeParam(84.0, 1.0, 5)
    }
     
    # Run the analysis
    sim.variabilityAnalysis(
        baseConfig=config,
        paramDict=param_dict,
        compileFirst=False,
        verbose=True,
        n_jobs=None  # Use all available cores
    )
    
    # Cargar datos de un solo vuelo
    analyzer = FlightAnalyzer('rockets/myRocket/results/myRocketData.csv')
    analyzer.plot("altitude","qInf")

 # Crear analizador de variabilidad
    var_analyzer = VariabilityAnalyzer('rockets/myRocket/results')

 # Cargar todas las simulaciones que coincidan con el patrón
    var_analyzer.loadAllSimulations('myRocketData_*.csv')
    var_analyzer.printComparisonTable()

    var_analyzer.plotComparison("massEmpty", "apogee", color_param= "railAngle",legend_position = "outside")





# ============================================================================
# CRITICAL: Required for Windows multiprocessing
# ============================================================================
if __name__ == '__main__':
    freeze_support()  # Required for Windows when creating executables
    main()