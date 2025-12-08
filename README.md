# LEEMON Rocket Simulator 🚀🍋

<p align="center">
  <img src="docs/leemon_banner.jpg" alt="LEEMON Logo" width="2000"/>
</p>

<p align="center">
  <strong>Professional Rocket Flight Simulator</strong><br>
  6-DOF Trajectory Analysis | Monte Carlo Simulations | Parallel Processing
</p>

---

## 🎯 What is LEEMON?

**LEEMON** is a professional rocket flight simulator that allows you to design, simulate, and analyze rocket trajectories with high accuracy. No programming knowledge required - just configure your rocket parameters and launch!

### Key Features

- ✅ **Easy to Use**: Configure rockets with simple text files
- ⚡ **Fast Simulations**: Optimized C++ engine with parallel processing
- 📊 **Professional Results**: Detailed flight data and automatic plots
- 🎯 **High Accuracy**: 6-DOF physics with atmosphere and wind models
- 🔧 **Flexible**: Custom motors, aerodynamics, and parachute systems
- 📈 **Monte Carlo Analysis**: Run hundreds of simulations to study variability

---

## 🚀 Quick Start (5 Minutes)

### Step 1: Run the Example

```bash
# Navigate to your LEEMON folder
cd LEEMON_Rocket_Simulator

# Run the example simulation
python rockets/myRocket/mySimulation.py
```

**That's it!** You'll see:
- ✓ Simulation progress
- ✓ Flight summary (apogee, velocity, range)
- 📊 Automatic plots (altitude, velocity, trajectory)
- 📁 Results saved in `results/` folder

### Step 2: Create Your Own Rocket

```bash
# Copy the example to start your project
cp -r rockets/myRocket rockets/my_new_rocket

# Edit your rocket configuration
# Open: rockets/my_new_rocket/example.txt
```

### Step 3: Customize and Launch

Edit `rockets/my_new_rocket/mySimulation.py`:

```python
# Change this line to point to your configuration
config = sim.loadConfigFromFile("rockets/my_new_rocket/example.txt")

# Adjust launch conditions
config["railAngle"] = 85.0
config["windSpeed"] = 10.0
```

Run your simulation:
```bash
python rockets/my_new_rocket/mySimulation.py
```

---

## 📁 Project Structure

```
LEEMON_Rocket_Simulator/
├── 📄 leemon.py                      # Main interface (import this)
├── 📄 README.md
├── 📄 README.txt
├── 📄 LICENSE.txt
│
├── 📁 bin/                           # Simulation engine (don't modify)
│   └── leemon.exe
│
├── 📁 lib/                           # Core libraries (don't modify)
│   ├── leemonSim.pyc
│   └── analysisTools.pyc
│
├── 📁 rockets/                       # YOUR PROJECTS GO HERE
│   └── myRocket/                     # Example project
│       ├── mySimulation.py           # Simulation script
│       ├── example.txt               # Rocket configuration
│       ├── data/                     # Aerodynamic data (Cd curves)
│       │   ├── CDoff.csv
│       │   └── CDon.csv
│       ├── results/                  # Simulation outputs (CSV files)
│       └── plots/                    # Generated plots (PNG files)
│
├── 📁 temp/                          # Temporary files
└── 📁 docs/                          # Documentation
    └── leemon_banner.jpg
```

---

## 🎨 Creating Your Rocket Project

### Method 1: Copy the Example

```bash
# Copy the template
cp -r rockets/myRocket rockets/falcon9

# Your new project is ready!
cd rockets/falcon9
```

### Method 2: Create from Scratch

```bash
# Create project folders
mkdir -p rockets/my_rocket/{data,results,plots}

# Copy configuration template
cp rockets/myRocket/example.txt rockets/my_rocket/
cp rockets/myRocket/mySimulation.py rockets/my_rocket/
```

---

## ⚙️ Configuration Guide

Edit `example.txt` in your rocket folder to define your rocket:

### 🚀 Basic Rocket Parameters

```txt
# Body
diameter = 0.13          # Body diameter [m]
length = 1.41            # Body length [m]
massEmpty = 10.0         # Empty mass without motor [kg]

# Nose cone
noseType = ogive         # Options: ogive, conical, parabolic
noseLength = 0.33        # [m]

# Fins
numFins = 4              # Number of fins
rootChord = 0.2          # Root chord [m]
tipChord = 0.07          # Tip chord [m]
span = 0.2               # Fin span [m]
finPosition = 1.53       # Distance from nose tip [m]
```

### 🔥 Motor Configuration

**Option 1: Total Impulse (Recommended for beginners)**
```txt
totalImpulse = 3000.0    # Total impulse [N·s]
burnTime = 3.0           # Burn duration [s]
motorFuelMass = 2.02     # Propellant mass [kg]
```

The simulator will calculate constant thrust automatically.

**Option 2: Custom Thrust Curve**
```txt
thrustFile = rockets/myRocket/data/Thrust.csv
```

Create a CSV file:
```csv
time,thrust
0.0,0
0.1,1500
1.0,1200
2.0,800
3.0,0
```

### 🪂 Parachute System (Optional)

```txt
# Drogue parachute (deploys at apogee)
useDrogue = true
drogueDiameter = 0.5     # [m]
drogueCd = 0.75          # Drag coefficient

# Main parachute (deploys at altitude)
useMain = true
mainDiameter = 1.5       # [m]
mainCd = 1.0
mainDeployAltitude = 300.0  # [m]
```

### 🌬️ Environment & Launch

In your `mySimulation.py`:

```python
# Wind conditions
config["windSpeed"] = 5.0       # [m/s]
config["windAngle"] = 45.0      # [degrees] (0° = North)

# Launch rail
config["railLength"] = 6.0      # [m]
config["railAngle"] = 84.0      # [degrees] (90° = vertical)
```

### 📊 Aerodynamics

Place your drag coefficient files in `rockets/your_rocket/data/`:

**CDoff.csv** (motor off):
```csv
mach,cd
0.0,0.45
0.5,0.46
0.8,0.52
1.0,0.75
1.5,0.62
2.0,0.55
```

**CDon.csv** (motor on):
```csv
mach,cd
0.0,0.48
0.5,0.49
...
```

---

## 🎯 Running Simulations

### Single Flight

```python
# In mySimulation.py
sim.quickRun(config, outputFile=config["outputFile"])
```

### Monte Carlo Analysis

Test multiple parameters with uncertainties:

```python
from leemon import randomizeParam

# Define parameters to vary
param_dict = {
    "massEmpty": randomizeParam(10.0, 0.5, 10),    # mean, std_dev, N samples
    "railAngle": randomizeParam(84.0, 1.0, 10),
    "windSpeed": [0, 5, 10, 15, 20]                # Or discrete values
}

# Run parallel simulations
sim.variabilityAnalysis(
    baseConfig=config,
    paramDict=param_dict,
    n_jobs=None  # Uses all CPU cores
)
```

This will run **N×M×...** simulations (10×10×5 = 500 in this example).

---

## 📊 Analyzing Results

### View Results

All simulations save CSV files in `rockets/your_rocket/results/`:

```
myRocketData.csv
myRocketData_massEmpty_9.5_railAngle_83.0.csv
myRocketData_massEmpty_9.5_railAngle_84.0.csv
...
```

### Automatic Analysis

Add to your simulation script:

```python
from leemon import Analyzer, VariabilityAnalyzer

# Single flight
analyzer = Analyzer('rockets/myRocket/results/myRocketData.csv')
analyzer.printSummary()
analyzer.plotFlightProfile()
analyzer.plotTrajectory(view='3d')

# Compare multiple flights
var_analyzer = VariabilityAnalyzer('rockets/myRocket/results')
var_analyzer.loadAllSimulations('myRocketData_*.csv')
var_analyzer.printComparisonTable()
var_analyzer.plotComparison('massEmpty', 'apogee', color_param='railAngle')
```

### Available Plots

```python
# Time histories
analyzer.plot('time', 'altitude')
analyzer.plot('time', 'mach')
analyzer.plot('altitude', 'qInf')  # Dynamic pressure vs altitude

# Trajectories
analyzer.plotTrajectory(view='3d')      # 3D view
analyzer.plotTrajectory(view='xy')      # Top view (ground track)
analyzer.plotTrajectory(view='altitude') # Side view

# Complete profile
analyzer.plotFlightProfile()  # Altitude, velocity, Mach, acceleration

# Attitude
analyzer.plotAttitude()  # Roll, pitch, yaw angles
```

---

## 📈 Example Workflow

### Scenario: Design a High-Power Rocket

**Goal**: Reach 3000m apogee with maximum stability

**Step 1: Create Project**
```bash
mkdir -p rockets/hpr3000/{data,results,plots}
cp rockets/myRocket/example.txt rockets/hpr3000/
cp rockets/myRocket/mySimulation.py rockets/hpr3000/
```

**Step 2: Configure Rocket** (`rockets/hpr3000/example.txt`)
```txt
diameter = 0.15
massEmpty = 12.0
totalImpulse = 5000.0
burnTime = 3.5
motorFuelMass = 3.0
```

**Step 3: Run Initial Simulation**
```bash
python rockets/hpr3000/mySimulation.py
```

**Step 4: Optimize Launch Angle**

Edit `mySimulation.py`:
```python
param_dict = {
    "railAngle": [80, 82, 84, 86, 88, 90]
}

sim.variabilityAnalysis(config, param_dict)
```

**Step 5: Analyze Results**
```python
var_analyzer.plotComparison('railAngle', 'apogee')
```

**Step 6: Check Stability in Wind**
```python
param_dict = {
    "railAngle": [84],  # Optimal angle from step 4
    "windSpeed": [0, 5, 10, 15, 20],
    "windAngle": [0, 45, 90, 135, 180]
}

sim.variabilityAnalysis(config, param_dict, n_jobs=8)
```

---

## 🎓 Tips & Best Practices

### 1. Organize Your Projects

```
rockets/
├── test_flights/        # Quick tests
├── competition_2024/    # Competition rockets
├── research/            # Experimental designs
└── archive/             # Old projects
```

### 2. Version Your Configurations

```txt
# example_v1.txt
massEmpty = 10.0

# example_v2.txt (lighter version)
massEmpty = 9.0
```

### 3. Save Important Results

```bash
# Rename important runs
cp results/myRocketData.csv results/myRocketData_baseline_2024-01-15.csv
```

### 4. Document Your Simulations

Add comments in `mySimulation.py`:
```python
# 2024-01-15: Baseline configuration
# - Target apogee: 3000m
# - Competition rules: max diameter 0.15m
# - Wind limit: 10 m/s
```

### 5. Use Realistic Parameters

- **Center of Gravity**: Should be forward of Center of Pressure for stability
- **Rail Length**: Minimum 3-5m for safe launch
- **Launch Angle**: 84-86° typical (not vertical to account for wind)
- **Parachute Sizing**: Main parachute for <5 m/s landing speed

---

## 🆘 Troubleshooting

### Problem: Simulation doesn't run

**Check:**
1. Is `bin/leemon.exe` present?
2. Are all paths in `example.txt` correct?
3. Run with verbose: `sim.quickRun(config, verbose=True)`

### Problem: Unrealistic results

**Common causes:**
- Center of gravity behind center of pressure (unstable)
- Motor impulse too low (doesn't clear rail)
- Drag coefficient files missing or incorrect format
- Excessive wind speed for rocket mass

### Problem: Plots don't appear

```python
# Add at end of script
import matplotlib.pyplot as plt
plt.show()  # Force display
```

### Problem: CSV file not found

Check the output path:
```python
# Use absolute path or check working directory
config["outputFile"] = "rockets/myRocket/results/myRocketData.csv"
```

---

## 📚 Output Data Format

All simulations save CSV files with these columns:

| Column | Description | Units |
|--------|-------------|-------|
| `time` | Simulation time | s |
| `north`, `east`, `down` | Position (NED frame) | m |
| `altitude` | Altitude above ground | m |
| `vNorth`, `vEast`, `vDown` | Velocity (NED) | m/s |
| `vxBody`, `vyBody`, `vzBody` | Velocity (body frame) | m/s |
| `roll`, `pitch`, `yaw` | Euler angles | rad |
| `mach` | Mach number | - |
| `qInf` | Dynamic pressure | Pa |
| `alpha` | Angle of attack | rad |
| `mass` | Current mass | kg |
| `Fx`, `Fy`, `Fz` | Forces (body frame) | N |
| `drogueDeployed` | Drogue status | 0/1 |
| `mainDeployed` | Main parachute status | 0/1 |

Use these for custom analysis in Excel, MATLAB, or Python.

---

## 📞 Support

- 📧 **Email**: hugo.argente25@gmail.com
- 📖 **Documentation**: Check `docs/` folder
- 🐛 **Issues**: Report problems or request features

---

## 📄 License

This software is licensed for commercial use.

**Restrictions:**
- ❌ No redistribution
- ❌ No reverse engineering
- ❌ No source code modifications

**Allowed:**
- ✅ Create unlimited rocket simulations
- ✅ Use for commercial projects
- ✅ Publish results and papers

See `LICENSE.txt` for full terms.

---

## 🎉 Examples Gallery

### Example 1: Basic Flight
- **Apogee**: 3547 m
- **Max Velocity**: 312 m/s
- **Range**: 145 m

### Example 2: High Wind (15 m/s)
- **Apogee**: 3421 m
- **Max Velocity**: 298 m/s
- **Range**: 872 m (wind drift)

### Example 3: Dual Deployment
- **Apogee**: 4012 m
- **Drogue Deploy**: 4012 m
- **Main Deploy**: 300 m
- **Landing Speed**: 4.2 m/s

---

<p align="center">
  <strong>Ready to launch? 🚀</strong><br>
  Start with: <code>python rockets/myRocket/mySimulation.py</code>
</p>

<p align="center">
  <sub>LEEMON Rocket Simulator from LEEM UPM</sub>
</p>

<p align="center">
  © 2024 LEEM UPM. All rights reserved.
</p>