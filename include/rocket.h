/**
 * @file rocket.h
 * @brief Rocket component definitions and aerodynamic models
 * 
 * This header file defines the mass, propulsive, and aerodynamic properties
 * of rocket components including nose cone, fin set, motor, parachute, and the 
 * complete rocket assembly.
 */

#ifndef ROCKET_H
#define ROCKET_H

#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>
#include <sstream>
#include <Eigen/Dense>
#include "controller.h"

/**
 * @class NoseCone
 * @brief Nose cone geometry and aerodynamic properties
 * 
 * Defines the geometry and computes aerodynamic coefficients for different
 * nose cone types (conical, ogive, parabolic). The nose cone contributes
 * to the total normal force coefficient and affects the center of pressure.
 */
class NoseCone{
    public:

    /**
     * @enum ojiveType
     * @brief Available nose cone geometries
     */
    enum ojiveType {
        conical = 1,   ///< Conical nose cone
        parabolic = 3, ///< Parabolic nose cone
        ogive = 2      ///< Ogive (tangent or secant) nose cone
    };

    /**
     * @brief Constructor for NoseCone class
     * 
     * Creates a nose cone with specified geometry parameters.
     * 
     * @param noseConeType Type of nose cone geometry
     * @param length Nose cone length [m]
     * @param diameter Nose cone base diameter [m]
     */
    NoseCone(ojiveType &noseConeType, double &length, double &diameter){
        this->noseConeType = noseConeType; 
        this->length = length;
        this->diameter = diameter;
    }
    
    /**
     * @brief Default constructor
     */
    NoseCone();

    double cna;      ///< Normal force coefficient derivative [1/rad]
    double xcp;      ///< Center of pressure location from nose tip [m]
    double length;   ///< Nose cone length [m]
    double diameter; ///< Nose cone base diameter [m]

    /**
     * @brief Computes aerodynamic coefficients for the nose cone
     * 
     * Calculates Cna and xcp based on nose cone type and compressibility factor.
     * 
     * @param beta Compressibility correction factor (Prandtl-Glauert)
     */
    void computeAeroCoefficients(double &beta);

    private:
    ojiveType noseConeType; ///< Nose cone geometry type
};

/**
 * @class FinSet
 * @brief Fin set geometry and aerodynamic properties
 * 
 * Defines the geometry of a fin set (typically 3 or 4 fins) and computes
 * aerodynamic coefficients including body interference effects. Uses different
 * methods for subsonic, transonic, and supersonic flight regimes.
 */
class FinSet{
    public:

    /**
     * @brief Constructor for FinSet class
     * 
     * Creates a fin set with specified geometric parameters.
     * 
     * @param N Number of fins (typically 3 or 4)
     * @param rootChord Root chord length [m]
     * @param tipChord Tip chord length [m]
     * @param span Fin span (semi-span from body to tip) [m]
     * @param sweepLength Sweep length (leading edge offset) [m]
     * @param rocketRadius Rocket body radius at fin location [m]
     */
    FinSet(int &N, double &rootChord, double &tipChord, double &span, 
           double &sweepLength, double &rocketRadius){
        this->N = N;
        this->rootChord = rootChord;
        this->tipChord = tipChord;
        this->span = span;
        this->sweepLength = sweepLength;
        this->rocketRadius = rocketRadius;
    }

    /**
     * @brief Default constructor
     */
    FinSet();

    double cna; ///< Normal force coefficient derivative [1/rad]
    double xcp; ///< Center of pressure location from nose tip [m]

    /**
     * @brief Computes geometric parameters of the fin set
     * 
     * Calculates derived geometric properties such as fin area, aspect ratio,
     * mean aerodynamic chord location, and body interference factors.
     */
    void computeGeometry();

    /**
     * @brief Computes aerodynamic coefficients for the fin set
     * 
     * Calculates Cna and xcp based on Mach number, compressibility factor,
     * and fin geometry. Uses different formulations for subsonic, transonic,
     * and supersonic regimes.
     * 
     * @param mach Mach number
     * @param beta Compressibility correction factor
     * @param position Fin set position from nose tip [m]
     */
    void computeAeroCoefficients(double &mach, double &beta, double &position);

    private:
    int N;                  ///< Number of fins
    double rootChord;       ///< Root chord length [m]
    double tipChord;        ///< Tip chord length [m]
    double span;            ///< Fin span [m]
    double sweepLength;     ///< Sweep length [m]
    double rocketRadius;    ///< Rocket body radius [m]

    // Derived geometric parameters
    double Yr;              ///< Sum of root and tip chords [m]
    double Afc;             ///< Fin planform area [m²]
    double AR;              ///< Aspect ratio
    double gamma;           ///< Mid-chord sweep angle [rad]
    double Yma;             ///< Span-wise location of mean aerodynamic chord [m]

    // Body interference parameters
    double tau;             ///< Fin-body interference factor
    double lift_interference; ///< Lift interference factor
    double lambda_r;        ///< Taper ratio (tip/root)
};

/**
 * @class Motor
 * @brief Rocket motor properties and thrust model
 * 
 * Defines motor mass properties (empty mass, propellant mass, inertia tensors)
 * and thrust characteristics. Supports two thrust modes:
 * 1. Thrust curve from CSV file (time vs thrust)
 * 2. Constant thrust from total impulse and burn time
 * 
 * The motor automatically selects the appropriate mode:
 * - If thrustFile is provided and exists, uses CSV thrust curve
 * - Otherwise, uses constant thrust calculated from totalImpulse and burnTime
 */
class Motor{
    public:
    
    /**
     * @brief Constructor for Motor class with CSV thrust curve
     * 
     * Creates a motor with specified mass properties and thrust curve from file.
     * If the thrust file cannot be loaded, the motor will fall back to constant
     * thrust mode using totalImpulse if available.
     * 
     * @param thrustFile Filename containing thrust curve data (.csv format)
     * @param emptyMass Empty motor mass (casing only) [kg]
     * @param fuelMass Propellant mass [kg]
     * @param xcgEmpty Center of gravity location when empty [m from nose]
     * @param xcgFull Center of gravity location when full [m from nose]
     * @param emptyInertia Inertia tensor when empty [kg·m²]
     * @param fullInertia Inertia tensor when full [kg·m²]
     * @param burnTime Total burn duration [s]
     * @param totalImpulse Total impulse (optional, for constant thrust mode) [N·s]
     */
    Motor(std::string &thrustFile, double &emptyMass, double &fuelMass, 
          double &xcgEmpty, double &xcgFull, Eigen::Matrix3d &emptyInertia, 
          Eigen::Matrix3d &fullInertia, double &burnTime, double totalImpulse = 0.0){
        this->thrustFile = thrustFile;
        this->emptyMass = emptyMass;
        this->fuelMass = fuelMass;
        this->xcgEmpty = xcgEmpty;
        this->xcgFull = xcgFull;
        this->emptyInertia = emptyInertia;
        this->fullInertia = fullInertia;
        this->burnTime = burnTime;
        this->totalImpulse = totalImpulse;
        this->useConstantThrust = false;
        this->constantThrustValue = 0.0;
        
        loadThrustData();
    }
    
    /**
     * @brief Default constructor
     */
    Motor();

    std::string thrustFile;         ///< Thrust curve filename (.csv)
    double emptyMass;               ///< Empty motor mass [kg]
    double fuelMass;                ///< Propellant mass [kg]
    double xcgEmpty;                ///< CG location when empty [m]
    double xcgFull;                 ///< CG location when full [m]
    Eigen::Matrix3d emptyInertia;   ///< Inertia tensor when empty [kg·m²]
    Eigen::Matrix3d fullInertia;    ///< Inertia tensor when full [kg·m²]
    double burnTime;                ///< Total burn duration [s]
    double totalImpulse;            ///< Total impulse for constant thrust mode [N·s]
    bool useConstantThrust;         ///< Flag indicating constant thrust mode is active
    double constantThrustValue;     ///< Constant thrust value [N] (calculated from totalImpulse/burnTime)

    /**
     * @brief Returns thrust at a given time
     * 
     * Retrieves motor thrust based on the active thrust mode:
     * - If using CSV data: interpolates from loaded thrust curve
     * - If using constant thrust: returns constantThrustValue during burn time
     * Returns zero before ignition and after burnout.
     * 
     * @param time Current simulation time [s]
     * @return double Thrust force [N]
     */
    double getThrust(double &time);

    private:
    std::vector<double> thrustTimes;   ///< Time points from thrust curve [s]
    std::vector<double> thrustValues;  ///< Thrust values from curve [N]
    
    /**
     * @brief Loads thrust curve data from CSV file
     * 
     * Attempts to read time and thrust columns from CSV file.
     * Expected format: time,thrust (with or without header)
     * 
     * If the file cannot be loaded and totalImpulse > 0, automatically
     * switches to constant thrust mode and calculates constantThrustValue
     * as totalImpulse / burnTime.
     * 
     * Prints appropriate messages indicating which thrust mode is active.
     */
    void loadThrustData();
};

/**
 * @class Airbrakes
 * @brief Air braking system for controlled descent
 * 
 * Defines air brake properties including maximum area, drag coefficient,
 * and extension control. Air brakes increase the rocket's effective drag
 * coefficient by deploying panels that increase the frontal area.
 * The additional drag is calculated as: cd_airbrakes * extension * Amax / Aref
 * where Aref is the rocket's cross-sectional area.
 * 
 * Can be controlled manually (fixed extension) or via a controller (PID, gain scheduling).
 */
class Airbrakes {
    public:
    
    /**
     * @brief Constructor for Airbrakes class
     * 
     * @param maxArea Maximum air brake area when fully deployed [m²]
     * @param cd Drag coefficient of the air brakes
     */
    Airbrakes(double maxArea, double cd){
        this->maxArea = maxArea;
        this->cd = cd;
        this->extension = 0.0;
        this->controller = nullptr;
        this->useController = false;
    }
    
    /**
     * @brief Default constructor
     */
    Airbrakes();

    double maxArea;     ///< Maximum air brake area [m²]
    double cd;          ///< Air brake drag coefficient
    double extension;   ///< Extension fraction (0 = retracted, 1 = fully deployed)
    
    Controller* controller;  ///< Optional controller for active control
    bool useController;      ///< Flag indicating if controller is active
    
    /**
     * @brief Sets the air brake extension manually
     * 
     * @param ext Extension fraction between 0 and 1
     */
    void setExtension(double ext){
        extension = std::max(0.0, std::min(1.0, ext));
    }
    
    /**
     * @brief Gets current air brake extension
     * 
     * @return double Current extension fraction (0-1)
     */
    double getExtension() const {
        return extension;
    }
    
    /**
     * @brief Adds a controller to the air brakes
     * 
     * @param ctrl Pointer to controller object
     */
    void addController(Controller* ctrl){
        this->controller = ctrl;
        this->useController = (ctrl != nullptr && ctrl->enabled);
    }
    
    /**
     * @brief Updates air brake extension using controller
     * 
     * @param dt Time step [s]
     * @param stateMap Map of current state variables
     */
    void updateWithController(double dt, const std::map<ControlVariable, double>& stateMap){
        if (useController && controller != nullptr) {
            // Get current value of controlled variable
            auto it = stateMap.find(controller->controlledVar);
            if (it != stateMap.end()) {
                double currentValue = it->second;
                double newExtension = controller->compute(currentValue, dt, stateMap);
                setExtension(newExtension);
            }
        }
    }
    
    /**
     * @brief Computes additional drag coefficient contribution
     * 
     * Calculates the extra Cd added by the air brakes based on their
     * extension and area relative to rocket reference area.
     * 
     * @param Aref Rocket reference area (cross-sectional area) [m²]
     * @return double Additional drag coefficient
     */
    double getAdditionalCd(double Aref){
        if (Aref <= 0.0) return 0.0;
        return cd * extension * maxArea / Aref;
    }
};

/**
 * @class Parachute
 * @brief Parachute recovery system
 * 
 * Defines parachute properties including drag coefficient, surface area,
 * deployment conditions, and deployment dynamics. Supports dual deployment
 * configurations with drogue and main parachutes.
 */
class Parachute {
    public:
    
    /**
     * @enum DeploymentTrigger
     * @brief Parachute deployment trigger type
     */
    enum DeploymentTrigger {
        APOGEE = 0,      ///< Deploy at apogee detection
        ALTITUDE = 1     ///< Deploy at specific altitude
    };
    
    /**
     * @brief Constructor for Parachute class
     * 
     * @param diameter Parachute diameter [m]
     * @param cd Drag coefficient (typically 0.75-1.5 for round parachutes)
     * @param deploymentDelay Time delay after trigger condition [s]
     * @param deploymentTime Time it takes to fully deploy [s]
     * @param trigger Deployment trigger type (APOGEE or ALTITUDE)
     * @param deploymentAltitude Altitude for deployment (only for ALTITUDE trigger) [m]
     */
    Parachute(double diameter, double cd, double deploymentDelay = 0.0, 
              double deploymentTime = 0.5, DeploymentTrigger trigger = APOGEE,
              double deploymentAltitude = 0.0){
        this->diameter = diameter;
        this->cd = cd;
        this->deploymentDelay = deploymentDelay;
        this->deploymentTime = deploymentTime;
        this->trigger = trigger;
        this->deploymentAltitude = deploymentAltitude;
        this->area = M_PI * pow(diameter/2.0, 2.0);
        this->deployed = false;
        this->deploymentFraction = 0.0;
        this->deploymentStartTime = -1.0;
        this->triggerActivated = false;
    }
    
    /**
     * @brief Default constructor
     */
    Parachute();

    // Parachute properties
    double diameter;           ///< Parachute diameter [m]
    double cd;                 ///< Drag coefficient
    double area;               ///< Parachute surface area [m²]
    double deploymentDelay;    ///< Delay after trigger [s]
    double deploymentTime;     ///< Time to fully deploy [s]
    DeploymentTrigger trigger; ///< Deployment trigger type
    double deploymentAltitude; ///< Deployment altitude for ALTITUDE trigger [m]
    
    // Deployment state
    bool deployed;             ///< Whether parachute is fully deployed
    bool triggerActivated;     ///< Whether trigger condition has been met
    double deploymentFraction; ///< Deployment fraction (0 = stowed, 1 = fully deployed)
    double deploymentStartTime;///< Time when deployment started [s]
    
    /**
     * @brief Triggers parachute deployment
     * 
     * @param currentTime Current simulation time [s]
     */
    void triggerDeployment(double currentTime){
        if (!triggerActivated) {
            triggerActivated = true;
            deploymentStartTime = currentTime + deploymentDelay;
        }
    }
    
    /**
     * @brief Checks altitude-based deployment condition
     * 
     * @param currentAltitude Current rocket altitude [m]
     * @param currentTime Current simulation time [s]
     * @param descendingOnly Only trigger when descending (velocity < 0)
     * @param apogeeReached Whether apogee has been reached
     */
    void checkAltitudeTrigger(double currentAltitude, double currentTime, bool descendingOnly = true, bool apogeeReached = false){
        if (trigger == ALTITUDE && !triggerActivated) {
            bool altitudeCondition = (currentAltitude <= deploymentAltitude);
            bool velocityCondition = !descendingOnly || (apogeeReached);
            
            if (altitudeCondition && velocityCondition) {
                triggerDeployment(currentTime);
            }
        }
    }
    
    /**
     * @brief Updates parachute deployment state
     * 
     * @param currentTime Current simulation time [s]
     */
    void updateDeployment(double currentTime){
        if (!triggerActivated || deploymentStartTime < 0) return;
        
        if (currentTime < deploymentStartTime) {
            deploymentFraction = 0.0;
            deployed = false;
        }
        else if (currentTime < deploymentStartTime + deploymentTime) {
            // Linear deployment model
            deploymentFraction = (currentTime - deploymentStartTime) / deploymentTime;
            deployed = false;
        }
        else {
            deploymentFraction = 1.0;
            deployed = true;
        }
    }
    
    /**
     * @brief Computes parachute drag force
     * 
     * @param velocity Velocity magnitude [m/s]
     * @param rho Air density [kg/m³]
     * @return double Drag force magnitude [N]
     */
    double getDragForce(double velocity, double rho){
        if (deploymentFraction <= 0.0) return 0.0;
        
        // Effective area increases with deployment fraction
        double effectiveArea = area * deploymentFraction;
        
        // Drag force: F = 0.5 * rho * v² * Cd * A
        return 0.5 * rho * velocity * velocity * cd * effectiveArea;
    }
    
    /**
     * @brief Resets parachute to initial state
     */
    void reset(){
        deployed = false;
        triggerActivated = false;
        deploymentFraction = 0.0;
        deploymentStartTime = -1.0;
    }
};

/**
 * @class Rocket
 * @brief Complete rocket assembly with all components
 * 
 * Combines all rocket components (nose cone, body, fins, motor, parachutes) and manages:
 * - Total aerodynamic coefficients (combining all components)
 * - Mass properties (mass, CG, inertia) that vary with time as propellant burns
 * - Aerodynamic forces and moments in body frame
 * - Drag coefficient from CSV files (power on/off)
 * - Dual deployment parachute system (drogue and main)
 */
class Rocket{
    public:

    /**
     * @brief Constructor for Rocket class
     * 
     * Creates a rocket with specified geometry and mass properties.
     * Components (nose cone, fins, motor, parachutes) must be added separately.
     * 
     * @param diameter Maximum rocket diameter [m]
     * @param length Rocket body length (excluding nose cone) [m]
     * @param massEmpty Mass of empty rocket body (without motor) [kg]
     * @param inertiaEmpty Inertia tensor of empty body [kg·m²]
     * @param power_off_drag Filename for drag coefficient vs Mach (motor off)
     * @param power_on_drag Filename for drag coefficient vs Mach (motor on)
     * @param xcgEmpty CG location of empty body [m from nose]
     */
    Rocket(double &diameter, double &length, double &massEmpty, 
           Eigen::Matrix3d &inertiaEmpty, std::string &power_off_drag, 
           std::string &power_on_drag, double &xcgEmpty){
        this->diameter = diameter;
        this->length = length;
        this->massEmpty = massEmpty;
        this->inertiaEmpty = inertiaEmpty;
        this->power_off_drag = power_off_drag;
        this->power_on_drag = power_on_drag;
        this->xcgEmpty = xcgEmpty;
        this->hasDrogue = false;
        this->hasMain = false;
        this->hasAirbrakes = false;
        loadDragData();
    }

    /**
     * @brief Default constructor
     */
    Rocket();
    
    FinSet finSet;           ///< Fin set component
    Motor motor;             ///< Motor component
    NoseCone noseCone;       ///< Nose cone component
    Parachute drogueChute;   ///< Drogue parachute (deploys at apogee)
    Parachute mainChute;     ///< Main parachute (deploys at altitude)
    Airbrakes airbrakes;     ///< Air brakes system

    bool hasDrogue;          ///< Whether rocket has drogue parachute
    bool hasMain;            ///< Whether rocket has main parachute
    bool hasAirbrakes;       ///< Whether rocket has air brakes

    double cna;              ///< Total normal force coefficient derivative [1/rad]
    double xcp;              ///< Total center of pressure location [m from nose]
    double cma;              ///< Aerodynamic moment coefficient derivative [1/rad]
    double cd;               ///< Drag coefficient

    Eigen::Matrix3d inertia; ///< Current total inertia tensor [kg·m²]
    double mass;             ///< Current total mass [kg]
    double xcg;              ///< Current CG location [m from nose]

    /**
     * @brief Adds a nose cone to the rocket
     * 
     * @param noseCone NoseCone object to attach
     */
    void addNoseCone(NoseCone &noseCone){
        this->noseCone = noseCone;
    }

    /**
     * @brief Adds a fin set to the rocket
     * 
     * @param finSet FinSet object to attach
     * @param finPosition Axial position of fin leading edge root [m from nose]
     */
    void addFinSet(FinSet &finSet, double &finPosition){
        this->finPosition = finPosition;
        this->finSet = finSet;
    }

    /**
     * @brief Adds a motor to the rocket
     * 
     * @param motor Motor object to install
     */
    void addMotor(Motor &motor){
        this->motor = motor;
    }
    
    /**
     * @brief Adds a drogue parachute to the rocket
     * 
     * The drogue parachute deploys at apogee and provides initial deceleration.
     * 
     * @param parachute Parachute object configured as drogue
     */
    void addDrogueParachute(Parachute &parachute){
        this->drogueChute = parachute;
        this->hasDrogue = true;
    }
    
    /**
     * @brief Adds a main parachute to the rocket
     * 
     * The main parachute typically deploys at a lower altitude for final descent.
     * 
     * @param parachute Parachute object configured as main
     */
    void addMainParachute(Parachute &parachute){
        this->mainChute = parachute;
        this->hasMain = true;
    }
    
    /**
     * @brief Adds air brakes to the rocket
     * 
     * Air brakes provide active control of descent rate by increasing drag.
     * Extension can be controlled during flight via the airbrakes member.
     * 
     * @param airbrakes Airbrakes object configured with max area and Cd
     */
    void addAirbrakes(Airbrakes &airbrakes){
        this->airbrakes = airbrakes;
        this->hasAirbrakes = true;
    }

    /**
     * @brief Computes total aerodynamic coefficients
     * 
     * Combines nose cone, body, and fin contributions to calculate total
     * Cna, xcp, and cma. Applies compressibility corrections based on Mach number.
     * 
     * @param mach Mach number
     * @param alpha Angle of attack [rad]
     */
    void computeAeroCoefficients(double &mach, double &alpha);

    /**
     * @brief Gets drag coefficient from lookup table
     * 
     * Retrieves Cd based on Mach number and motor state (on/off) from CSV files.
     * Uses linear interpolation between data points.
     * 
     * @param mach Mach number
     * @param time Current simulation time [s]
     */
    void getCd(double &mach, double &time);

    /**
     * @brief Computes aerodynamic angles from velocity vector
     * 
     * Calculates angle of attack, sideslip angle, and dynamic pressure from
     * the aerodynamic velocity vector in body frame.
     * 
     * @param aeroSpeed Aerodynamic velocity vector in body frame [m/s]
     * @param rho Air density [kg/m³]
     * @return Eigen::Vector3d [alpha, sideslip, qInf]
     */
    Eigen::Vector3d computeVelocity(Eigen::Vector3d &aeroSpeed, double &rho);

    /**
     * @brief Computes aerodynamic forces in body frame
     * 
     * Calculates drag, lift, and side forces based on aerodynamic coefficients
     * and flow conditions.
     * 
     * @param qInf Dynamic pressure [Pa]
     * @param alpha Angle of attack [rad]
     * @param sideSlipAngle Sideslip angle [rad]
     * @return Eigen::Vector3d Force vector [Fx, Fy, Fz] in body frame [N]
     */
    Eigen::Vector3d computeAeroForces(double &qInf, double &alpha, double &sideSlipAngle);

    /**
     * @brief Computes aerodynamic moments in body frame
     * 
     * Calculates restoring moments about CG due to aerodynamic forces.
     * 
     * @param qInf Dynamic pressure [Pa]
     * @param alpha Angle of attack [rad]
     * @param sideSlipAngle Sideslip angle [rad]
     * @return Eigen::Vector3d Moment vector [Mx, My, Mz] in body frame [N·m]
     */
    Eigen::Vector3d computeAeroMoments(double &qInf, double &alpha, double &sideSlipAngle);

    /**
     * @brief Updates mass properties based on propellant consumption
     * 
     * Computes current mass, CG location, and inertia tensor as propellant burns.
     * Uses parallel axis theorem to combine component inertias about the
     * instantaneous center of gravity.
     * 
     * @param time Current simulation time [s]
     */
    void computeMassProperties(double &time);

    private:
    double diameter;                ///< Rocket diameter [m]
    double length;                  ///< Body length [m]
    double massEmpty;               ///< Empty body mass [kg]
    Eigen::Matrix3d inertiaEmpty;   ///< Empty body inertia [kg·m²]
    std::string power_off_drag;     ///< Drag file (motor off)
    std::string power_on_drag;      ///< Drag file (motor on)
    double xcgEmpty;                ///< Empty body CG location [m]
    double finPosition;             ///< Fin axial position [m from nose]
    
    // Drag curve data vectors
    std::vector<double> dragMachOff;    ///< Mach numbers (motor off)
    std::vector<double> dragCdOff;      ///< Cd values (motor off)
    std::vector<double> dragMachOn;     ///< Mach numbers (motor on)
    std::vector<double> dragCdOn;       ///< Cd values (motor on)
    
    /**
     * @brief Loads drag curve data from CSV files
     * 
     * Reads Mach and Cd columns from both power-on and power-off CSV files.
     * Expected format: mach,cd (with or without header)
     */
    void loadDragData();
};

#endif // ROCKET_H