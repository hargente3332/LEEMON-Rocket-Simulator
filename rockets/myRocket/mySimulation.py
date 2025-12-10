#!/usr/bin/env python3
"""
LEEMON V2 - Rocket Simulation Script
Works on any computer without modification!
"""

import sys
from pathlib import Path
from multiprocessing import freeze_support

# Auto-detect LEEMON root and add python folder to path
def setup_leemon():
    """Setup LEEMON paths automatically"""
    script_dir = Path(__file__).parent.resolve()
    
    # Navigate up to find LEEMON root (contains bin/ and python/ folders)
    current = script_dir
    for _ in range(5):
        if (current / "bin" / "leemon.exe").exists() and (current / "python").exists():
            python_dir = current / "python"
            if str(python_dir) not in sys.path:
                sys.path.insert(0, str(python_dir))
            return current
        current = current.parent
    
    raise RuntimeError(
        "Could not find LEEMON installation.\n"
        "Ensure this script is inside the LEEMON-Portable/rockets/ folder."
    )

LEEMON_ROOT = setup_leemon()

from leemonSim import LEEMONSimulator, randomizeParam
from analysisTools import FlightAnalyzer


def main():
    """Main simulation function"""
    
    sim = LEEMONSimulator(project_root=LEEMON_ROOT)
    
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
    output_file = LEEMON_ROOT / config.get("outputFile", "rockets/myRocket/results/flightData.csv")
    if output_file.exists():
        analyzer = FlightAnalyzer(str(output_file))
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


if __name__ == '__main__':
    freeze_support()
    main()
