#pragma once

#include <cinttypes>
#include <ArduinoJson.h>
#include <librrc/Helpers/jsonconfighelper.h>

#include "ODriveEnums.h"

/**
 * CHAD GPIO LAYOUT
 * 
 * 10    GND
 * 9     8
 * 7     3v3
 * 6(TX) 5(RX) 
 */

class ODriveController {
public:
    /**
     * @brief How the board is connected to the ODrive.
     */
    enum Connection {
        UART,
        CAN
    };

    enum Axis {
        ZERO = 0,
        ONE = 1
    };

    /**
     * @brief Privately construct a new ODriveController object using a serial Stream.
     */
    ODriveController(Stream& serial, float turnRange);

    /** @brief Factory method for constructing the ODrive controller from a config setup. */    
    static ODriveController fromConfig(JsonObjectConst config);

    /**
     * @brief Arms the controller.
     * 
     * @return true Success
     * @return false Failure
     */
    void arm(Axis axis);

    /**
     * @brief Query the status of the controller.
     * 
     * @return true All good!
     * @return false Bad
     */
    bool available();

    /**
     * @brief Queries the ODrive for some debug information and prints.
     */
    void printDebug();

    int error();
    int error(Axis axis);

    void fullErrors(Axis, int&, int&, int&, int&);

    /**
     * @brief Send a position command to the ODrive controller.
     * 
     * Commands should be in the range [0, 1], 0 representing the 
     * minimal position, and 1 the maximal.
     * 
     * @param axis0 Position command for the motor on the x-axis (Motor 0).
     * @param axis1 Position command for the motor on the y-axis (Motor 1).
     */
    void position(float axis0, float axis1);

    enum class SysCommand {
        REBOOT,
        SAVE_CONF,
        ERASE_CONF,
        CLEAR_ERR
    };

    /**
     * @brief Blocking call to restart the connected ODrive.
     * 
     * The call is blocking as it awaits a valid response from the ODrive.
     * 
     * @param type The type of restart call.
     */
    void command(SysCommand type);

    void writeConfig(const std::string& conf, const float command);
    void writeConfig(const std::string& conf, const int command);
    void writeConfig(const std::string& conf, const bool command);

    void calibrateAxis(Axis axis);

    void requestFeedback(Axis axis, float& position, float& velocity);
    void readConfig(const std::string& config);
    float readConfigFloat(const std::string& config);
    int readConfigInt(const std::string& config);

    /**
     * @brief Query the status of the ODrive, returns wether or not the ODrive sent a 
     * valid heartbeat on the last construction / restart.
     * 
     * Use status() to query the board directly.
     * 
     * @return true 
     * @return false 
     */
    explicit operator bool();
    void setVelocity(int motor_number, float velocity);
private:

    String readString();

    // Commands
    void setPosition(int motor_number, float position);
    void setPosition(int motor_number, float position, float velocity_feedforward);
    void setPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward);
    void setVelocity(int motor_number, float velocity, float current_feedforward);
    void setCurrent(int motor_number, float current);
    void trapezoidalMove(int motor_number, float position);

    // Getters
    float getVelocity(int motor_number);
    float getPosition(int motor_number);

    // General params
    float readFloat();
    int32_t readInt();

    // State helper
    bool run_state(int axis, int requested_state, bool wait_for_idle = true, float timeout = 10.0f);

    //! @brief The status from the ODrive from the last status check
    bool operational = false;

    //! @brief How the controller is connected to this board.
    Connection connection;
    
    //! @brief Can address of ODrive
    int canTxAddr = -1;

    //! @brief Can address of this Board
    int canRxAddr = -1;

    //! @brief UART transmit pin
    int uartTxPin = -1;

    //! @brief UART receive pin
    int uartRxPin = -1;

    Stream& serial;

    //! @brief Maximal number of turns of the motor between minimal and maximal position.
    float turnRange = 0;

    // //! @brief Current number of turns from minimum.
    // float currentTurns = 0;
};