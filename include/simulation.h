/**
 * @file simulation.h
 * @brief Flight simulation and atmospheric environment definitions
 */

#ifndef SIMULATION_H
#define SIMULATION_H

#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "rocket.h"

/**
 * @class Environment
 * @brief Atmospheric environment model
 */
class Environment{
    public:
    enum environmentTypes {
        ISA = 1,
        custom = 2
    };

    environmentTypes envType;
    double pInf;
    double aInf;
    double rhoInf;
    double tInf;
    double windSpeed[3];
    double g = 9.81;

    Environment(environmentTypes envType = ISA){
        this->envType = envType;
    }

    void getEnvironmentData(double &h);
    void setWind(double &module, double &orientation);
};

/**
 * @struct FlightState
 * @brief Complete state of the rocket at a single time instant
 * 
 * Contains all kinematic, dynamic, and physical properties of the rocket
 * during flight. This includes:
 * - Position and velocity (in both inertial NED and body frames)
 * - Orientation (quaternion, Euler angles, DCM)
 * - Angular velocity and acceleration
 * - Mass properties (mass, inertia tensor, CG location)
 * - Forces and moments acting on the rocket
 * - Aerodynamic flow parameters (Mach, angle of attack, dynamic pressure)
 * - Parachute deployment status
 * 
 * The state uses the following coordinate systems:
 * - NED (North-East-Down): Inertial reference frame fixed to Earth
 * - Body frame: Moving frame fixed to rocket, X-axis along longitudinal axis
 */
struct FlightState {
    double time;                          ///< Current simulation time [s]
    Eigen::Vector3d position;             ///< Position vector in NED frame [m]
    Eigen::Vector3d velocityBody;         ///< Velocity in body frame [m/s]
    Eigen::Vector3d velocityInertial;     ///< Velocity in NED frame [m/s]
    Eigen::Vector4d quaternion;           ///< Rotation quaternion (NED to body): [q0, q1, q2, q3]
    Eigen::Vector3d eulerAngles;          ///< Euler angles: [roll, pitch, yaw] [rad]
    Eigen::Matrix3d DCM;                  ///< Direction Cosine Matrix (NED to body transformation)
    Eigen::Vector3d angularVelocity;      ///< Angular velocity in body frame [rad/s]
    Eigen::Vector3d angularAcceleration;  ///< Angular acceleration in body frame [rad/s²]
    Eigen::Vector3d accelerationBody;     ///< Linear acceleration in body frame [m/s²]
    double mass;                          ///< Total mass (including remaining propellant) [kg]
    Eigen::Matrix3d inertia;              ///< Inertia tensor about CG in body frame [kg·m²]
    double xcg;                           ///< Center of gravity location from nose tip [m]
    Eigen::Vector3d forces;               ///< Total force vector in body frame [N]
    Eigen::Vector3d moments;              ///< Total moment vector in body frame [N·m]
    double mach;                          ///< Mach number (V/a)
    double qInf;                          ///< Dynamic pressure: q = 0.5·ρ·V² [Pa]
    double alpha;                         ///< Angle of attack [rad]
    double sideSlipAngle;                 ///< Sideslip angle [rad]
    double altitude;                      ///< Altitude above ground (positive up) [m]
    bool drogueDeployed;                  ///< Drogue parachute deployment status
    bool mainDeployed;                    ///< Main parachute deployment status
};

/**
 * @class Flight
 * @brief Flight simulation manager for 6-DOF rocket dynamics
 */
class Flight{
    public:
    Flight(Environment &env, Rocket &rocket){
        this->env = env; 
        this->rocket = rocket;
        this->isInitialized = false;
        this->stopAtApogee = false;
        this->apogeeDetected = false;
        this->maxAltitudeReached = 0.0;
    }

    Environment env;
    Rocket rocket;
    FlightState currentState;
    std::vector<FlightState> data;
    double railLength;
    double railAngle;
    
    // Apogee and parachute control
    bool stopAtApogee;        ///< If true, simulation stops at apogee
    bool apogeeDetected;      ///< Flag indicating apogee has been reached
    double maxAltitudeReached;///< Maximum altitude reached so far [m]

    void setRail(const double railLength, const double railAngle){
        this->railLength = railLength;
        this->railAngle = railAngle*M_PI/180.0;
    }
    
    /**
     * @brief Sets whether simulation should stop at apogee
     * 
     * @param stop If true, simulation terminates when apogee is detected
     */
    void setStopAtApogee(bool stop){
        this->stopAtApogee = stop;
    }

    void setInitialConditions(const Eigen::Vector3d& position,
                             const Eigen::Vector3d& velocityBody,
                             const Eigen::Vector3d& eulerAngles,
                             const Eigen::Vector3d& angularVelocity);

    void simulate(double tFinal, double dt);
    void step(double dt);
    
    double getApogee() const;
    double getMaxVelocity() const;
    double getMaxMach() const;
    
    void saveToCSV(const std::string& filename) const;
    Eigen::Vector3d computeGravity(double mass);
    void getDCM(Eigen::Quaterniond quaternion);
    
    /**
     * @brief Checks if apogee has been reached
     * 
     * Detects apogee by comparing current altitude with previous maximum.
     * If altitude starts decreasing, apogee is detected and drogue parachute
     * deployment is triggered.
     */
    void checkApogee(){
        if (currentState.altitude > maxAltitudeReached) {
            maxAltitudeReached = currentState.altitude;
        }
        else if (currentState.altitude < maxAltitudeReached - 1.0 && !apogeeDetected) {
            // Apogee detected (altitude decreased by more than 1m)
            apogeeDetected = true;
            
            // Trigger drogue parachute if available
            if (rocket.hasDrogue) {
                rocket.drogueChute.triggerDeployment(currentState.time);
            }
            
            if (stopAtApogee) {
            }
        }
    }

    private:
    bool isInitialized;

    struct StateDerivative {
        Eigen::Vector3d positionDot;
        Eigen::Vector3d velocityDot;
        Eigen::Vector4d quaternionDot;
        Eigen::Vector3d angularVelocityDot;
    };

    Eigen::Matrix3d quaternionToDCM(const Eigen::Vector4d& q);
    Eigen::Vector3d quaternionToEuler(const Eigen::Vector4d& q);
    Eigen::Vector4d eulerToQuaternion(const Eigen::Vector3d& euler);
    Eigen::Vector4d normalizeQuaternion(const Eigen::Vector4d& q);
    
    StateDerivative computeDerivatives(double t, 
                                      const Eigen::Vector3d& pos,
                                      const Eigen::Vector3d& vel,
                                      const Eigen::Vector4d& quat,
                                      const Eigen::Vector3d& omega);
    
    std::pair<Eigen::Vector3d, Eigen::Vector3d> computeForcesAndMoments(const FlightState& state);
};

#endif // SIMULATION_H