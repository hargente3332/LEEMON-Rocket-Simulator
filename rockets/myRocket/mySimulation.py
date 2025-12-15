#!/usr/bin/env python3
"""
LEEMON V2 - Rocket Simulation Script
Works on any computer without modification!
"""

import sys
from pathlib import Path
from multiprocessing import freeze_support

# Add LEEMON root to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from leemon import LEEMONSimulator, FlightAnalyzer, randomizeParam, VariabilityAnalyzer


def main():
    """Main simulation function"""
    
    sim = LEEMONSimulator()
    
    # =========================================================================
    # LOAD CONFIGURATION
    # =========================================================================
    config = sim.loadConfigFromFile("rockets/myRocket/example.txt")
    
    # Override parameters
    config["windSpeed"] = 5.0
    config["windAngle"] = 20.0
    config["railLength"] = 6.0
    config["railAngle"] = 84.0
    
    # =========================================================================
    # RUN SIMULATION
    # =========================================================================
    print("\n" + "="*70)
    print("RUNNING SIMULATION")
    print("="*70)
    
    sim.quickRun(config, outputFile=config.get("outputFile", "rockets/myRocket/results/flightData.csv"))
    
    # =========================================================================
    # ANALYZE RESULTS
    # =========================================================================
    output_file = config.get("outputFile", "rockets/myRocket/results/flightData.csv")
    analyzer = FlightAnalyzer(output_file)
    analyzer.printSummary()
    
    # =========================================================================
    # VARIABILITY ANALYSIS
    # =========================================================================
    print("\n" + "="*70)
    print("VARIABILITY ANALYSIS")
    print("="*70)
    
    param_dict = {
        "massEmpty": randomizeParam(10.0, 0.50, 2),
        "railAngle": randomizeParam(84.0, 1.0, 3)
    }
    
    sim.variabilityAnalysis(
        baseConfig=config,
        paramDict=param_dict,
        verbose=True,
        n_jobs=None
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

    

if __name__ == '__main__':
    freeze_support()
    main()
