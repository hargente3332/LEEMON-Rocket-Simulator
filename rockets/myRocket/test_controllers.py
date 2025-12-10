#!/usr/bin/env python3
"""
Test all controller types: PID, Gain Scheduling, and MPC
Compares performance of different control strategies
"""

from leemonSim import LEEMONSimulator # type: ignore
from analysisTools import FlightAnalyzer # type: ignore
from multiprocessing import freeze_support
import os

def main():
    """
    Test all three controller types
    """
    sim = LEEMONSimulator()
    
    print("\n" + "="*70)
    print("CONTROLLER COMPARISON TEST")
    print("="*70)
    
    # Test configurations
    tests = [
        {
            "name": "Baseline (No Controller)",
            "config_file": "rockets/myRocket/example.txt",
            "description": "No airbrake control"
        },
        {
            "name": "Fixed Airbrakes 30%",
            "config_file": "rockets/myRocket/example_airbrakes_fixed.txt",
            "description": "Fixed 30% airbrake extension"
        },
        {
            "name": "PID 1000m Target",
            "config_file": "rockets/myRocket/example_pid_1000m.txt",
            "description": "PID targeting exactly 1000m apogee"
        },
        {
            "name": "PID Velocity Control",
            "config_file": "rockets/myRocket/example_pid.txt",
            "description": "PID for vertical velocity control"
        },
        {
            "name": "Gain Scheduling",
            "config_file": "rockets/myRocket/example_gain_scheduling.txt",
            "description": "PID with altitude-dependent gains for 1000m"
        },
        {
            "name": "MPC",
            "config_file": "rockets/myRocket/example_mpc.txt",
            "description": "Model Predictive Control (optimized)"
        }
    ]
    
    results = {}
    
    for test in tests:
        print(f"\n{'='*70}")
        print(f"Running: {test['name']}")
        print(f"Description: {test['description']}")
        print(f"Config: {test['config_file']}")
        print(f"{'='*70}")
        
        # Load configuration
        config = sim.loadConfigFromFile(test['config_file'])
        
        # Run simulation
        try:
            success = sim.quickRun(config, outputFile=config.get("outputFile", None))
            
            # Read results from output file
            if success and config.get("outputFile"):
                output_file = config.get("outputFile")
                if os.path.exists(output_file):
                    # Use analyzer to get key metrics
                    analyzer = FlightAnalyzer(output_file)
                    summary = analyzer.getSummary()
                    
                    if summary:
                        results[test['name']] = {
                            'apogee': summary['apogee'],
                            'max_velocity': summary['max_velocity'],
                            'flight_time': summary['flight_time'],
                            'config_file': test['config_file']
                        }
                        
                        print(f"✓ Success!")
                        print(f"  Apogee: {summary['apogee']:.2f} m")
                        print(f"  Max Velocity: {summary['max_velocity']:.2f} m/s")
                        print(f"  Flight Time: {summary['flight_time']:.2f} s")
                    else:
                        print(f"✗ Could not get summary")
                        results[test['name']] = None
                else:
                    print(f"✗ Output file not found: {output_file}")
                    results[test['name']] = None
            else:
                print(f"✗ Simulation failed")
                results[test['name']] = None
                
        except Exception as e:
            print(f"✗ Error: {str(e)}")
            import traceback
            traceback.print_exc()
            results[test['name']] = None
    
    # Summary
    print(f"\n{'='*70}")
    print("SUMMARY - CONTROLLER COMPARISON (TARGET: 1000m APOGEE)")
    print(f"{'='*70}")
    print(f"{'Controller':<30} {'Apogee (m)':<15} {'Error (m)':<15}")
    print(f"{'-'*70}")
    
    target_apogee = 1000.0
    for name, data in results.items():
        if data:
            error = abs(data['apogee'] - target_apogee)
            print(f"{name:<30} {data['apogee']:<15.2f} {error:<15.2f}")
        else:
            print(f"{name:<30} {'FAILED':<15} {'-':<15}")
    
    print(f"{'='*70}\n")

if __name__ == '__main__':
    freeze_support()
    main()
