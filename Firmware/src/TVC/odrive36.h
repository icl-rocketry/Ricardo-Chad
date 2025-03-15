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

#include "TVC/odriveEnums.h"

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
    Odrive36(float maxTurns, float commandOffset);

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
     * @brief Request a position in turns from the software endstop position.
     * 
     * @param axisZero Turns for motor axis zero.
     * @param axisOne Turns for motor axis one.
     */
    void commandAxisTurns(float axisZero, float axisOne);

    /**
     * @brief Request a position in command signal.
     * 
     * @param axisZero Command signal [0, 1] for axis 0.
     * @param axisOne Command signal [0, 1] for axis 1.
     */
    void commandAxisControl(float axisZero, float axisOne);

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
    bool idleAxis(MotorAxis motor);

    /**
     * Request position information from the Odrive.
     * 
     * @param motor The motor axis.
     * 
     * @param requested Returns the requested position (turns).
     * @param position Returns the actual actuator position (turns).
     * @param velocity Returns the actuator velocity (turns / s).
     */
    void getPosition(MotorAxis motor, float& requested, float& position, float& velocity);

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

    /**
     * @brief Read and print a configuration from the odrive.
     * 
     * @param config The config to read.
     */
    void poll(std::string config);

    void getMotorCurrent(float& zero, float& one);
    void getVoltage(float& voltage);

    void waitForOdrive();

    bool isAlive();

    /**
     * Re-configure a certain axis.
     * 
     * This should NOT need to be done since configurations are saved
     * on ODrive in persistent memory.
     * 
     * @param axis The axis to configure.
     */
    void configureAxis(MotorAxis axis);
private:
    /// @brief The current error status of the ODrive (if hasError).
    ODriveError error;

    /// @brief Number of turns from 0 until the maximal extension of the actuator.
    const float maxTurns;

    /// @brief Software min endstop
    const float minEndstop;

    /// @brief The Serial connection to the ODrive.
    Stream& serial;

    /// @brief Odrive system commands.
    enum class SysCommand {
        REBOOT,
        SAVE_CONF,
        ERASE_CONF,
        CLEAR_ERR
    };

    /// @brief Last requested command for axis 0
    float axis0Requested = minEndstop;

    /// @brief Last requested command for axis 1
    float axis1Requested = minEndstop;

    /**
     * @brief Write a floating point config to the Odrive.
     * 
     * @param config String config to write, UART connection means odrv0.<>
     *               does not need to be specified, and the string should
     *               start after this, i.e. axis0.motor.config...
     * @param value The value to set the configuration to.
     */
    void writeConfig(const std::string& config, const float value);

    /**
     * @brief Write an integer config to the Odrive.
     * 
     * @param config String config to write, UART connection means odrv0.<>
     *               does not need to be specified, and the string should
     *               start after this, i.e. axis0.motor.config...
     * @param value The value to set the configuration to.
     */
    void writeConfig(const std::string& config, const int value);
    
    /**
     * @brief Write a boolean config to the Odrive.
     * 
     * @param config String config to write, UART connection means odrv0.<>
     *               does not need to be specified, and the string should
     *               start after this, i.e. axis0.motor.config...
     * @param value The value to set the configuration to.
     */
    void writeConfig(const std::string& config, const bool value);

    /// @brief Read an integer configuration from the Odrive.
    int readConfigInt(const std::string& config);

    /// @brief Read a floating point configuration from the Odrive.
    float readConfigFloat(const std::string& config);

    /// @brief Read a boolean configuration from the Odrive.
    bool readConfigBool(const std::string& config);

    /// @brief Read a string from the serial port.
    String readString();
    
    /// @brief Read a float from the serial port.
    float readFloat();

    /// @brief Read an integer from the serial port.
    int readInt();

    /**
     * @brief Send a system command to the odrive.
     * 
     * @param command The command to send.
     */
    void command(SysCommand command);

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
};