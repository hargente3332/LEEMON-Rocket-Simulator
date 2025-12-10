/**
 * @file controller.h
 * @brief Control system definitions for active rocket components
 * 
 * This header file defines various control algorithms for active control
 * of rocket components such as air brakes. Includes PID controllers,
 * gain scheduling, and reference tracking capabilities.
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <cmath>
#include <vector>
#include <string>
#include <map>
#include <fstream>
#include <sstream>
#include <iostream>

/**
 * @enum ControlVariable
 * @brief Variables that can be controlled by the control system
 */
enum ControlVariable {
    VERTICAL_VELOCITY,     ///< Vertical velocity in NED frame [m/s]
    ALTITUDE,              ///< Altitude above ground [m]
    ALTITUDE_RATE,         ///< Rate of altitude change [m/s]
    ROLL_RATE,             ///< Roll angular velocity [rad/s]
    PITCH_RATE,            ///< Pitch angular velocity [rad/s]
    YAW_RATE,              ///< Yaw angular velocity [rad/s]
    ANGLE_OF_ATTACK,       ///< Angle of attack [rad]
    VELOCITY_MAGNITUDE,    ///< Total velocity magnitude [m/s]
    MACH_NUMBER,           ///< Mach number
    DYNAMIC_PRESSURE       ///< Dynamic pressure [Pa]
};

/**
 * @enum ReferenceType
 * @brief Type of reference signal for the controller
 */
enum ReferenceType {
    STATIC_REFERENCE,      ///< Constant reference value
    BREAKPOINT_REFERENCE   ///< Reference varies with breakpoints (e.g., altitude-based)
};

/**
 * @class ReferenceSignal
 * @brief Defines the desired reference value for the controller
 * 
 * Can be either a static constant value or vary based on breakpoints
 * (e.g., desired velocity as a function of altitude).
 */
class ReferenceSignal {
public:
    ReferenceType type;
    double staticValue;                    ///< Static reference value
    std::vector<double> breakpointKeys;    ///< Breakpoint independent variable (e.g., altitude)
    std::vector<double> breakpointValues;  ///< Breakpoint dependent variable (e.g., desired velocity)
    
    /**
     * @brief Constructor for static reference
     */
    ReferenceSignal(double value) {
        type = STATIC_REFERENCE;
        staticValue = value;
    }
    
    /**
     * @brief Constructor for breakpoint reference
     */
    ReferenceSignal(const std::vector<double>& keys, const std::vector<double>& values) {
        type = BREAKPOINT_REFERENCE;
        breakpointKeys = keys;
        breakpointValues = values;
        staticValue = 0.0;
    }
    
    /**
     * @brief Default constructor
     */
    ReferenceSignal() {
        type = STATIC_REFERENCE;
        staticValue = 0.0;
    }
    
    /**
     * @brief Get reference value (with interpolation for breakpoints)
     * 
     * @param breakpointParam Current value of breakpoint parameter
     * @return double Reference value
     */
    double getValue(double breakpointParam = 0.0) const {
        if (type == STATIC_REFERENCE) {
            return staticValue;
        }
        
        // Interpolate breakpoints
        if (breakpointKeys.empty()) return 0.0;
        
        if (breakpointParam <= breakpointKeys[0]) {
            return breakpointValues[0];
        }
        
        if (breakpointParam >= breakpointKeys.back()) {
            return breakpointValues.back();
        }
        
        for (size_t i = 0; i < breakpointKeys.size() - 1; i++) {
            if (breakpointParam >= breakpointKeys[i] && breakpointParam <= breakpointKeys[i+1]) {
                double t = (breakpointParam - breakpointKeys[i]) / 
                          (breakpointKeys[i+1] - breakpointKeys[i]);
                return breakpointValues[i] + t * (breakpointValues[i+1] - breakpointValues[i]);
            }
        }
        
        return 0.0;
    }
};

/**
 * @class Controller
 * @brief Base class for control algorithms
 * 
 * Abstract base class that defines the interface for all controllers.
 * Derived classes implement specific control algorithms (PID, gain scheduling, etc.).
 */
class Controller {
public:
    bool enabled;                    ///< Controller enable flag
    ControlVariable controlledVar;   ///< Variable being controlled
    ReferenceSignal reference;       ///< Desired reference value
    ControlVariable breakpointVar;   ///< Variable used for breakpoints (if applicable)
    
    double outputMin;                ///< Minimum output (saturation)
    double outputMax;                ///< Maximum output (saturation)
    
    /**
     * @brief Default constructor
     */
    Controller() {
        enabled = false;
        controlledVar = VERTICAL_VELOCITY;
        breakpointVar = ALTITUDE;
        outputMin = 0.0;
        outputMax = 1.0;
    }
    
    /**
     * @brief Virtual destructor
     */
    virtual ~Controller() {}
    
    /**
     * @brief Compute control output
     * 
     * @param currentValue Current value of controlled variable
     * @param dt Time step [s]
     * @param stateMap Map of all state variables for breakpoint evaluation
     * @return double Control output (saturated between outputMin and outputMax)
     */
    virtual double compute(double currentValue, double dt, 
                          const std::map<ControlVariable, double>& stateMap) = 0;
    
    /**
     * @brief Reset controller internal state
     */
    virtual void reset() = 0;
    
    /**
     * @brief Saturate output to limits
     */
    double saturate(double value) const {
        if (value < outputMin) return outputMin;
        if (value > outputMax) return outputMax;
        return value;
    }
};

/**
 * @class PIDController
 * @brief PID (Proportional-Integral-Derivative) controller
 * 
 * Standard PID controller with anti-windup and derivative filtering.
 * Control law: u = Kp*e + Ki*∫e + Kd*de/dt
 */
class PIDController : public Controller {
public:
    double Kp;                       ///< Proportional gain
    double Ki;                       ///< Integral gain
    double Kd;                       ///< Derivative gain
    
    double integralError;            ///< Accumulated integral error
    double previousError;            ///< Previous error for derivative
    bool firstRun;                   ///< First execution flag
    
    double integralLimit;            ///< Anti-windup limit for integral term
    
    /**
     * @brief Constructor
     */
    PIDController(double kp, double ki, double kd) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
        integralError = 0.0;
        previousError = 0.0;
        firstRun = true;
        integralLimit = 1.0;
        outputMin = 0.0;
        outputMax = 1.0;
    }
    
    /**
     * @brief Default constructor
     */
    PIDController() {
        Kp = 0.0;
        Ki = 0.0;
        Kd = 0.0;
        integralError = 0.0;
        previousError = 0.0;
        firstRun = true;
        integralLimit = 1.0;
    }
    
    /**
     * @brief Compute PID control output
     */
    double compute(double currentValue, double dt, 
                  const std::map<ControlVariable, double>& stateMap) override {
        if (!enabled) return 0.0;
        
        // Get reference value
        double refValue = reference.staticValue;
        if (reference.type == BREAKPOINT_REFERENCE) {
            auto it = stateMap.find(breakpointVar);
            if (it != stateMap.end()) {
                refValue = reference.getValue(it->second);
            }
        }
        
        // Calculate error
        double error = refValue - currentValue;
        
        // Proportional term
        double P = Kp * error;
        
        // Integral term with anti-windup
        integralError += error * dt;
        if (integralError > integralLimit) integralError = integralLimit;
        if (integralError < -integralLimit) integralError = -integralLimit;
        double I = Ki * integralError;
        
        // Derivative term
        double D = 0.0;
        if (!firstRun && dt > 0.0) {
            double errorRate = (error - previousError) / dt;
            D = Kd * errorRate;
        }
        
        previousError = error;
        firstRun = false;
        
        // Compute and saturate output
        double output = P + I + D;
        return saturate(output);
    }
    
    /**
     * @brief Reset controller state
     */
    void reset() override {
        integralError = 0.0;
        previousError = 0.0;
        firstRun = true;
    }
};

/**
 * @class GainSchedulingController
 * @brief PID controller with gain scheduling
 * 
 * PID controller where gains (Kp, Ki, Kd) vary based on a scheduling
 * variable (e.g., altitude, Mach number, dynamic pressure).
 */
class GainSchedulingController : public Controller {
public:
    ControlVariable schedulingVar;           ///< Variable used for gain scheduling
    std::vector<double> schedulingPoints;    ///< Scheduling parameter values
    std::vector<double> KpSchedule;          ///< Kp values at each scheduling point
    std::vector<double> KiSchedule;          ///< Ki values at each scheduling point
    std::vector<double> KdSchedule;          ///< Kd values at each scheduling point
    
    double integralError;                    ///< Accumulated integral error
    double previousError;                    ///< Previous error for derivative
    bool firstRun;                           ///< First execution flag
    double integralLimit;                    ///< Anti-windup limit
    
    /**
     * @brief Constructor
     */
    GainSchedulingController(ControlVariable schedVar,
                            const std::vector<double>& schedPoints,
                            const std::vector<double>& kpSched,
                            const std::vector<double>& kiSched,
                            const std::vector<double>& kdSched) {
        schedulingVar = schedVar;
        schedulingPoints = schedPoints;
        KpSchedule = kpSched;
        KiSchedule = kiSched;
        KdSchedule = kdSched;
        integralError = 0.0;
        previousError = 0.0;
        firstRun = true;
        integralLimit = 1.0;
        outputMin = 0.0;
        outputMax = 1.0;
    }
    
    /**
     * @brief Default constructor
     */
    GainSchedulingController() {
        schedulingVar = ALTITUDE;
        integralError = 0.0;
        previousError = 0.0;
        firstRun = true;
        integralLimit = 1.0;
    }
    
    /**
     * @brief Interpolate gains based on scheduling parameter
     */
    void getGains(double schedValue, double& kp, double& ki, double& kd) const {
        if (schedulingPoints.empty()) {
            kp = ki = kd = 0.0;
            return;
        }
        
        if (schedValue <= schedulingPoints[0]) {
            kp = KpSchedule[0];
            ki = KiSchedule[0];
            kd = KdSchedule[0];
            return;
        }
        
        if (schedValue >= schedulingPoints.back()) {
            kp = KpSchedule.back();
            ki = KiSchedule.back();
            kd = KdSchedule.back();
            return;
        }
        
        for (size_t i = 0; i < schedulingPoints.size() - 1; i++) {
            if (schedValue >= schedulingPoints[i] && schedValue <= schedulingPoints[i+1]) {
                double t = (schedValue - schedulingPoints[i]) / 
                          (schedulingPoints[i+1] - schedulingPoints[i]);
                kp = KpSchedule[i] + t * (KpSchedule[i+1] - KpSchedule[i]);
                ki = KiSchedule[i] + t * (KiSchedule[i+1] - KiSchedule[i]);
                kd = KdSchedule[i] + t * (KdSchedule[i+1] - KdSchedule[i]);
                return;
            }
        }
        
        kp = ki = kd = 0.0;
    }
    
    /**
     * @brief Compute gain-scheduled PID control output
     */
    double compute(double currentValue, double dt, 
                  const std::map<ControlVariable, double>& stateMap) override {
        if (!enabled) return 0.0;
        
        // Get scheduling parameter value
        auto schedIt = stateMap.find(schedulingVar);
        if (schedIt == stateMap.end()) return 0.0;
        double schedValue = schedIt->second;
        
        // Interpolate gains
        double Kp, Ki, Kd;
        getGains(schedValue, Kp, Ki, Kd);
        
        // Get reference value
        double refValue = reference.staticValue;
        if (reference.type == BREAKPOINT_REFERENCE) {
            auto it = stateMap.find(breakpointVar);
            if (it != stateMap.end()) {
                refValue = reference.getValue(it->second);
            }
        }
        
        // Calculate error
        double error = refValue - currentValue;
        
        // PID computation
        double P = Kp * error;
        
        integralError += error * dt;
        if (integralError > integralLimit) integralError = integralLimit;
        if (integralError < -integralLimit) integralError = -integralLimit;
        double I = Ki * integralError;
        
        double D = 0.0;
        if (!firstRun && dt > 0.0) {
            double errorRate = (error - previousError) / dt;
            D = Kd * errorRate;
        }
        
        previousError = error;
        firstRun = false;
        
        double output = P + I + D;
        return saturate(output);
    }
    
    /**
     * @brief Reset controller state
     */
    void reset() override {
        integralError = 0.0;
        previousError = 0.0;
        firstRun = true;
    }
};

/**
 * @class MPCController
 * @brief Model Predictive Controller with linear model
 * 
 * MPC controller that predicts future system behavior over a horizon
 * and optimizes control action. Uses a simplified linear model:
 * x(k+1) = A*x(k) + B*u(k)
 * 
 * Cost function: J = Σ[Q*(x_ref - x)² + R*u²] over prediction horizon
 */
class MPCController : public Controller {
public:
    // Model parameters (linear approximation)
    double A;                        ///< State transition parameter (x_next = A*x + B*u)
    double B;                        ///< Control input parameter
    
    // MPC parameters
    int predictionHorizon;           ///< Number of steps to predict ahead
    double Q;                        ///< State error weight in cost function
    double R;                        ///< Control effort weight in cost function
    
    // Optimization constraints
    double controlRateLimit;         ///< Maximum control rate of change per step
    double previousControl;          ///< Previous control output
    bool firstRun;                   ///< First execution flag
    
    // Model adaptation
    double previousState;            ///< Previous state value for model identification
    double modelAdaptationRate;      ///< Learning rate for online model adaptation (0 = no adaptation)
    
    /**
     * @brief Constructor
     */
    MPCController(double a, double b, int horizon, double q, double r) {
        A = a;
        B = b;
        predictionHorizon = horizon;
        Q = q;
        R = r;
        controlRateLimit = 0.1;  // Default: max 10% change per step
        previousControl = 0.0;
        previousState = 0.0;
        firstRun = true;
        modelAdaptationRate = 0.0;  // Disabled by default
        outputMin = 0.0;
        outputMax = 1.0;
    }
    
    /**
     * @brief Default constructor
     */
    MPCController() {
        A = 0.95;
        B = 0.05;
        predictionHorizon = 10;
        Q = 1.0;
        R = 0.1;
        controlRateLimit = 0.1;
        previousControl = 0.0;
        previousState = 0.0;
        firstRun = true;
        modelAdaptationRate = 0.0;
        outputMin = 0.0;
        outputMax = 1.0;
    }
    
    /**
     * @brief Predict future state trajectory for given control sequence
     */
    std::vector<double> predictTrajectory(double currentState, 
                                         const std::vector<double>& controlSequence) const {
        std::vector<double> trajectory(predictionHorizon + 1);
        trajectory[0] = currentState;
        
        for (int k = 0; k < predictionHorizon; k++) {
            double u = (k < controlSequence.size()) ? controlSequence[k] : 0.0;
            trajectory[k+1] = A * trajectory[k] + B * u;
        }
        
        return trajectory;
    }
    
    /**
     * @brief Evaluate cost function for a control sequence
     */
    double evaluateCost(double currentState, double referenceValue,
                       const std::vector<double>& controlSequence) const {
        std::vector<double> trajectory = predictTrajectory(currentState, controlSequence);
        double cost = 0.0;
        
        // State tracking cost
        for (int k = 1; k <= predictionHorizon; k++) {
            double stateError = referenceValue - trajectory[k];
            cost += Q * stateError * stateError;
        }
        
        // Control effort cost
        for (int k = 0; k < predictionHorizon; k++) {
            double u = (k < controlSequence.size()) ? controlSequence[k] : 0.0;
            cost += R * u * u;
        }
        
        // Control rate penalty
        double prevU = previousControl;
        for (int k = 0; k < predictionHorizon; k++) {
            double u = (k < controlSequence.size()) ? controlSequence[k] : 0.0;
            double deltaU = u - prevU;
            cost += 0.5 * R * deltaU * deltaU;  // Penalize control rate changes
            prevU = u;
        }
        
        return cost;
    }
    
    /**
     * @brief Optimize control using gradient descent
     */
    double optimizeControl(double currentState, double referenceValue) {
        // Initialize control sequence with previous control value
        std::vector<double> controlSequence(predictionHorizon, previousControl);
        
        // Simple gradient descent optimization
        const int maxIterations = 20;
        const double learningRate = 0.01;
        const double epsilon = 1e-6;
        
        for (int iter = 0; iter < maxIterations; iter++) {
            double currentCost = evaluateCost(currentState, referenceValue, controlSequence);
            
            // Compute gradient and update control sequence
            for (int k = 0; k < predictionHorizon; k++) {
                // Finite difference gradient
                std::vector<double> perturbedSequence = controlSequence;
                perturbedSequence[k] += epsilon;
                double perturbedCost = evaluateCost(currentState, referenceValue, perturbedSequence);
                double gradient = (perturbedCost - currentCost) / epsilon;
                
                // Gradient descent update
                controlSequence[k] -= learningRate * gradient;
                
                // Apply constraints
                controlSequence[k] = saturate(controlSequence[k]);
                
                // Rate limit
                double maxChange = previousControl + controlRateLimit;
                double minChange = previousControl - controlRateLimit;
                if (k == 0) {
                    if (controlSequence[k] > maxChange) controlSequence[k] = maxChange;
                    if (controlSequence[k] < minChange) controlSequence[k] = minChange;
                }
            }
        }
        
        // Return first control action (receding horizon principle)
        double optimalControl = controlSequence[0];
        
        // Apply rate limiting to first control action
        double maxControl = previousControl + controlRateLimit;
        double minControl = previousControl - controlRateLimit;
        if (optimalControl > maxControl) optimalControl = maxControl;
        if (optimalControl < minControl) optimalControl = minControl;
        
        return saturate(optimalControl);
    }
    
    /**
     * @brief Online model adaptation (optional)
     */
    void adaptModel(double currentState, double dt) {
        if (firstRun || modelAdaptationRate <= 0.0 || dt <= 0.0) {
            return;
        }
        
        // Predict what state should have been based on previous control
        double predictedState = A * previousState + B * previousControl;
        double predictionError = currentState - predictedState;
        
        // Update model parameters using prediction error
        // Gradient descent on squared prediction error
        if (std::abs(previousState) > 1e-6) {
            A += modelAdaptationRate * predictionError * previousState;
        }
        if (std::abs(previousControl) > 1e-6) {
            B += modelAdaptationRate * predictionError * previousControl;
        }
        
        // Keep A stable (bounded)
        if (A > 1.0) A = 1.0;
        if (A < 0.0) A = 0.0;
    }
    
    /**
     * @brief Compute MPC control output
     */
    double compute(double currentValue, double dt, 
                  const std::map<ControlVariable, double>& stateMap) override {
        if (!enabled) return 0.0;
        
        // Adapt model based on previous prediction (if enabled)
        adaptModel(currentValue, dt);
        
        // Get reference value
        double refValue = reference.staticValue;
        if (reference.type == BREAKPOINT_REFERENCE) {
            auto it = stateMap.find(breakpointVar);
            if (it != stateMap.end()) {
                refValue = reference.getValue(it->second);
            }
        }
        
        // Optimize control action
        double optimalControl = optimizeControl(currentValue, refValue);
        
        // Store for next iteration
        previousControl = optimalControl;
        previousState = currentValue;
        firstRun = false;
        
        return optimalControl;
    }
    
    /**
     * @brief Reset controller state
     */
    void reset() override {
        previousControl = 0.0;
        previousState = 0.0;
        firstRun = true;
    }
};

/**
 * @brief Parse control variable from string
 */
ControlVariable parseControlVariable(const std::string& str);

/**
 * @brief Load controller configuration from file
 */
Controller* loadControllerFromFile(const std::string& filename);

#endif // CONTROLLER_H
