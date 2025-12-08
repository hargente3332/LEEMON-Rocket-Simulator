#!/usr/bin/env python3
"""
LEEMON V2 - MyRocket Example Simulation
Using pre-compiled binaries

This is another EXAMPLE. You can:
- Copy viper/ to create your own rocket
- Edit configuration file (example.txt)
- Modify parameters below
- Run your simulations
"""

import sys
from pathlib import Path

# Add python modules to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "python"))

from leemonSim import LEEMONSimulator, randomizeParam
from leemonSim import FlightAnalyzer


def main():
    """Main simulation function"""
    
    # Get distribution root
    DIST_ROOT = Path(__file__).parent.parent.parent.resolve()
    
    print("\n" + "="*70)
    print("LEEMON V2 - MyRocket Example")
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
    
    config_file = "rockets/myRocket/example.txt"
    print(f"Loading configuration: {config_file}")
    
    config = sim.loadConfigFromFile(config_file)
    
    # =========================================================================
    # MODIFY PARAMETERS (EDIT THESE)
    # =========================================================================
    
    # Environment conditions
    config["windSpeed"] = 5.0               # Wind speed [m/s]
    config["windAngle"] = 20.0              # Wind angle [degrees]
    
    # Launch conditions
    config["railLength"] = 6.0              # Launch rail length [m]
    config["railAngle"] = 82.0              # Launch rail angle [degrees]
    config["initialPosDown"] = 0.0          # Initial altitude [m]
    
    # Output
    config["outputFile"] = "rockets/myRocket/results/myRocketData.csv"
    
    print(f"Wind: {config['windSpeed']} m/s @ {config['windAngle']}°")
    print(f"Launch: {config['railLength']}m rail @ {config['railAngle']}°")
    print(f"Output: {config['outputFile']}\n")
    
    # =========================================================================
    # RUN SIMULATION
    # =========================================================================
    
    print("Running simulation...")
    print("-" * 70)
    
    sim.quickRun(config, outputFile=config["outputFile"])
    
    print("-" * 70)
    print("\n[OK] Simulation completed!")
    print(f"Results saved to: {config['outputFile']}\n")
    
    # =========================================================================
    # ANALYZE RESULTS
    # =========================================================================
    
    print("="*70)
    print("Flight Data Summary")
    print("="*70 + "\n")
    
    try:
        analyzer = FlightAnalyzer(config["outputFile"])
        print(f"Simulation time: {analyzer.df['time'].max():.2f} seconds")
        print(f"Max altitude: {analyzer.df['altitude'].max():.2f} meters")
        print(f"Max velocity: {analyzer.df['velocity'].max():.2f} m/s")
        print(f"Max acceleration: {analyzer.df['acceleration'].max():.2f} m/s²\n")
        
    except Exception as e:
        print(f"Could not load results: {e}\n")
    
    print("="*70)
    print("Done!")
    print("="*70 + "\n")


if __name__ == "__main__":
    main() 

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
 
    sim.quickRun(config, outputFile=config["outputFile"])

    # ============================================================================
    # VARIABILITY ANALYSIS WITH PARALLEL PROCESSING
    # ============================================================================
    
    print("\n" + "="*70)
    print("MY ROCKET - MONTE CARLO ANALYSIS")
    print("="*70)
    
    # Define parameters to vary
    param_dict = {
        "emptyMass": randomizeParam(10.0, 0.50, 2),
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
    





# ============================================================================
# CRITICAL: Required for Windows multiprocessing
# ============================================================================
if __name__ == '__main__':
    freeze_support()  # Required for Windows when creating executables
    main()