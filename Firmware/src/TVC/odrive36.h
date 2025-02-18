/**
 * @file odrive36.h
 * @author Riley Horrix (rh1122@ic.ac.uk)
 * @brief Odrive 3.6 driver interface.
 * @version 0.1
 * @date 2025-02-17
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

#include <Arduino.h>

#include "tvc/odriveEnums.h"

/**
 * @brief Driver component for the ODrive v3.6 board.
 * 
 */
class Odrive36 {
public:
    /**
     * @brief Construct a new Odrive36 object in homing mode.
     * 
     * @param maxTurns The upper bound number of turns the actuator is able to
     * turn from the endstop.
     */
    Odrive36(float maxTurns);

    /**
     * @brief Enum for selecting the motor axis.
     */
    enum class MotorAxis : int {
        MOTOR_AXIS_ZERO = 0,
        MOTOR_AXIS_ONE = 1
    };

    /**
     * @brief The method of position control.
     */
    enum class ControlType {
        TRAP_TRAJ
    };

    /**
     * @brief Request a position in turns from the endstop position.
     * 
     * @param axisZero Turns for motor axis zero.
     * @param axisOne Turns for motor axis one.
     */
    void commandAxisTurns(float axisZero, float axisOne);

    /**
     * @brief Arm the ODrive controller for an axis.
     * 
     * @param motor The motor axis.
     * @return bool Whether the arming was successful or not.
     */
    bool armAxis(MotorAxis motor);

    /**
     * @brief Set the axis to idle state.
     * 
     * @param motor The motor axis.
     */
    void idleAxis(MotorAxis motor);

    void update();

    void start() {
        executing = true;
    }

    /**
     * @brief Lock the axis to it's current position.
     * 
     * @param motor The motor axis.
     */
    void lockAxis(MotorAxis motor);

    void disarmAxis(void);

    /**
     * @brief Structure to hold error states from the ODrive.
     * 
     */
    struct ODriveError {
        struct ODriveAxisError {
            ODriveAxisError(int main = 0, int controller = 0, int motor = 0, int encoder = 0);
            int main;
            int controller;
            int motor;
            int encoder;
        };

        ODriveError(int main = 0);

        /**
         * @brief ODrive main error.
         */
        int main;

        ODriveAxisError axis0;
        ODriveAxisError axis1;

        std::string toString(void);
    };

    /**
     * @brief Check if the ODrive axis has any errors.
     * 
     * If it does, it will set the errors in the ODrive Error Object.
     * 
     * It will also set the main error code too.
     */
    bool checkErrorsAxis(MotorAxis axis);

    /**
     * @brief Get the ODrive Error object.
     * 
     * @return const ODriveError& Reference to the error state.
     */
    inline const ODriveError& getError(void) {
        return error;
    }

private:
    /// @brief The current error status of the ODrive (if hasError).
    ODriveError error;

    /// @brief Any of the error fields are set
    bool hasAnyError; 
    
    /// @brief Number of turns from 0 until the maximal extension of the actuator.
    int maxTurns = 0;

    /// @brief The Serial connection to the ODrive.
    Stream& serial;

    /// @brief Current method of control.
    ControlType controlType;

    // Temp (should use state)
    bool armed = false;
    bool configured = false;

    /// @brief Odrive system commands.
    enum class SysCommand {
        REBOOT,
        SAVE_CONF,
        ERASE_CONF,
        CLEAR_ERR
    };

    bool executing = false;

    /**
     * @brief Run an axis state.
     * 
     * @param axis The axis to run the state on.
     * @param requestedState The requested axis state.
     * @param waitForIdle Whether the function should wait for the axis state to return to idle.
     * @param timeout Time to wait for idle state to finish (ms).
     * @return bool Whether or not the state successfully ran.
     */
    bool runState(MotorAxis axis, AxisState requestedState, bool waitForIdle = true, float timeout = 5000);

    void configureAxis(MotorAxis axis);

    void writeConfig(const std::string& config, const float value);
    void writeConfig(const std::string& config, const int value);
    void writeConfig(const std::string& config, const bool value);

    int readConfigInt(const std::string& config);
    float readConfigFloat(const std::string& config);

    String readString();
    
    float readFloat();

    int readInt();

    void command(SysCommand command);
};