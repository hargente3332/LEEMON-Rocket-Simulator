#!/usr/bin/env python3
"""
Test all controller types: PID, Gain Scheduling, and MPC
Works on any computer without modification!
"""

import sys
from pathlib import Path
from multiprocessing import freeze_support

def setup_leemon():
    """Setup LEEMON paths automatically"""
    script_dir = Path(__file__).parent.resolve()
    current = script_dir
    for _ in range(5):
        if (current / "bin" / "leemon.exe").exists() and (current / "python").exists():
            python_dir = current / "python"
            if str(python_dir) not in sys.path:
                sys.path.insert(0, str(python_dir))
            return current
        current = current.parent
    raise RuntimeError("Could not find LEEMON installation.")

LEEMON_ROOT = setup_leemon()

from leemonSim import LEEMONSimulator
from analysisTools import FlightAnalyzer


def main():
    """Test all controller types"""
    sim = LEEMONSimulator(project_root=LEEMON_ROOT)
    
    print("\n" + "="*70)
    print("CONTROLLER COMPARISON TEST")
    print("="*70)
    
    tests = [
        {"name": "Baseline", "config_file": "rockets/myRocket/example.txt"},
        {"name": "Fixed Airbrakes", "config_file": "rockets/myRocket/example_airbrakes_fixed.txt"},
        {"name": "PID Controller", "config_file": "rockets/myRocket/example_pid.txt"},
        {"name": "Gain Scheduling", "config_file": "rockets/myRocket/example_gain_scheduling.txt"},
        {"name": "MPC", "config_file": "rockets/myRocket/example_mpc.txt"},
    ]
    
    results = {}
    
    for test in tests:
        print(f"\n{'='*70}")
        print(f"Running: {test['name']}")
        print(f"{'='*70}")
        
        config = sim.loadConfigFromFile(test['config_file'])
        
        try:
            success = sim.quickRun(config, outputFile=config.get("outputFile"))
            
            if success and config.get("outputFile"):
                output_file = LEEMON_ROOT / config.get("outputFile")
                if output_file.exists():
                    analyzer = FlightAnalyzer(str(output_file))
                    summary = analyzer.getSummary()
                    if summary:
                        results[test['name']] = summary
                        print(f"✓ Apogee: {summary['apogee']:.2f} m")
        except Exception as e:
            print(f"✗ Error: {e}")
    
    print(f"\n{'='*70}")
    print("SUMMARY")
    print(f"{'='*70}")
    print(f"{'Controller':<25} {'Apogee (m)':<15}")
    print("-"*40)
    for name, data in results.items():
        print(f"{name:<25} {data['apogee']:<15.2f}")


if __name__ == '__main__':
    freeze_support()
    main()
