#!/usr/bin/env python3
"""
LEEMON V2 - Viper Rocket Simulation Example
Using pre-compiled binaries (no compilation needed!)

This is an EXAMPLE script. You can:
- Modify parameters in the config dictionary
- Change rocket configuration (viper.txt)
- Analyze results
- Copy this to create new simulations
"""

import sys
from pathlib import Path

# Add python modules to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "python"))

from leemonSim import LEEMONSimulator, randomizeParam
from leemonSim import FlightAnalyzer, VariabilityAnalyzer


def main():
    """Main simulation function"""
    
    # Get distribution root
    DIST_ROOT = Path(__file__).parent.parent.parent.resolve()
    
    print("\n" + "="*70)
    print("LEEMON V2 - Viper Rocket Simulation")
    print("="*70)
    print(f"Distribution root: {DIST_ROOT}\n")
    
    # =========================================================================
    # CREATE SIMULATOR (uses pre-compiled binaries)
    # =========================================================================
    
    sim = LEEMONSimulator(str(DIST_ROOT), use_compiled=True)
    
    print(f"Executable: {sim.executable}")
    print(f"Status: {'Ready' if sim.executable.exists() else 'NOT FOUND'}\n")
    
    # =========================================================================
    # LOAD ROCKET CONFIGURATION
    # =========================================================================
    
    config_file = "rockets/viper/viper.txt"
    print(f"Loading configuration: {config_file}")
    
    config = sim.loadConfigFromFile(config_file)
    
    # =========================================================================
    # MODIFY PARAMETERS (EDIT THESE)
    # =========================================================================
    
    # Environment conditions
    config["windSpeed"] = 0.0               # Wind speed [m/s]
    config["windAngle"] = 0.0               # Wind angle [degrees]
    
    # Launch conditions
    config["railLength"] = 6.0              # Launch rail length [m]
    config["railAngle"] = 84.0              # Launch rail angle [degrees] (90=vertical)
    config["initialPosDown"] = 0.0          # Initial altitude [m]
    
    # Output

    
    
    # =========================================================================
    # RUN SIMULATION
    # =========================================================================
    
    print("Running simulation...")
    print("-" * 70)
    
    sim.quickRun(config, outputFile=config["outputFile"])
    

    # =========================================================================
    # OPTIONAL: VARIABILITY ANALYSIS (Uncomment to run)
    # =========================================================================
    
    param_dict = {
         "massEmpty": [9.0, 10.0, 11.0],
         "railAngle": randomizeParam(84.0, 1.0, 2)
     }
    sim.variabilityAnalysis(
         baseConfig=config,
         paramDict=param_dict,
         compileFirst=False,
         verbose=True,
         n_jobs=None
     )
    
    # =========================================================================
    # ANALYZE RESULTS
    # =========================================================================

 
    var_analyzer = VariabilityAnalyzer('rockets/viper/results')

 
    var_analyzer.loadAllSimulations('viperData_*.csv')


    var_analyzer.plotComparison("massEmpty", "apogee", color_param= "railAngle",legend_position = "outside")
    


if __name__ == "__main__":
    main()