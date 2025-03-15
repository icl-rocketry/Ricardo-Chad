/**
 * @file odrive36.cpp
 * @author Riley Horrix (rh1122@ic.ac.uk)
 * @brief Odrive Driver implementations.
 * @version 0.1
 * @date 2025-02-17
 *
 * @copyright Copyright (c) 2025
 *
 */
#include <HardwareSerial.h>

#include <libriccore/riccorelogging.h>

#include "TVC/odrive36.h"
#include "TVC/odriveEnums.h"
#include "odrive36.h"

#include "Config/pinmap_config.h"

#define log(x) RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(x)

/// @brief Board UART transmit pin.
const int TX_PIN = PinMap::oDriveTx;

/// @brief Board UART receive pin.
const int RX_PIN = PinMap::oDriveRx;

/// @brief UART baud rate.
const int UART_BAUD = 115200;

Odrive36::Odrive36(float maxTurns, float minEndstop): 
        maxTurns(maxTurns),
        minEndstop(minEndstop),
        serial(Serial1) {
    Serial1.begin(UART_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
}

void Odrive36::commandAxisTurns(float axisZero, float axisOne) {
    axisZero = constrain(axisZero, minEndstop, minEndstop + maxTurns);
    axisOne = constrain(axisOne, minEndstop, minEndstop + maxTurns);

    axis0Requested = axisZero;
    axis1Requested = axisOne;

    serial.printf("t %d %.4f\n", static_cast<int>(MotorAxis::MOTOR_AXIS_ZERO), axisZero);
    serial.printf("t %d %.4f\n", static_cast<int>(MotorAxis::MOTOR_AXIS_ONE), axisOne);
}

void Odrive36::commandAxisControl(float axisZero, float axisOne) {
    axisZero = constrain(axisZero * maxTurns + minEndstop, minEndstop, minEndstop + maxTurns);
    axisOne = constrain(axisOne * maxTurns + minEndstop, minEndstop, minEndstop + maxTurns);

    axis0Requested = axisZero;
    axis1Requested = axisOne;

    serial.printf("t %d %.4f\n", static_cast<int>(MotorAxis::MOTOR_AXIS_ZERO), axisZero);
    serial.printf("t %d %.4f\n", static_cast<int>(MotorAxis::MOTOR_AXIS_ONE), axisOne);
}

bool Odrive36::armAxis(MotorAxis motor) {
    // Wait for the odrive
    log("[odrive36]: Arming!");
    waitForOdrive();

    // Clear errors
    command(SysCommand::CLEAR_ERR);
    const std::string axis = motor == MotorAxis::MOTOR_AXIS_ZERO ? "axis0" : "axis1";

    // If already armed then skip
    if (!readConfigBool(axis + ".motor.is_armed")) {
        log("[odrive36] Starting Motor and Encoder Calibration.");
        runState(motor, AxisState::AXIS_STATE_FULL_CALIBRATION_SEQUENCE);
    
        if (checkErrorsAxis(motor)) {
            log(error.toString());
            return false;
        }
    } else {
        log("[odrive36] Already armed, skipping arming.");
    }

    // If already homed then skip
    if (!readConfigBool(axis + ".is_homed")) {
        log("[odrive36] Starting Homing Calibration.");
        runState(motor, AxisState::AXIS_STATE_HOMING, true, 100000);
        
        if (checkErrorsAxis(motor)) {
            log(error.toString());
            return false;
        }
    } else {
        log("[odrive36] Already homed, skipping homing.");
    }

    log("[odrive36] Calibration Complete.");
    return true;
}

bool Odrive36::idleAxis(MotorAxis motor) {
    return runState(motor, AxisState::AXIS_STATE_IDLE);
}

bool Odrive36::runState(MotorAxis axis, AxisState requestedState, bool waitForIdle, float timeout) {
    const int delay_ms = 10;
    int timeoutCtr = static_cast<int>(timeout / delay_ms);
    serial.printf("w axis%d.requested_state %d\n", static_cast<int>(axis), static_cast<int>(requestedState));
    if (waitForIdle) {
        do {
            delay(delay_ms);
            serial.printf("r axis%d.current_state\n", static_cast<int>(axis));
        } while (readInt() != static_cast<int>(AxisState::AXIS_STATE_IDLE) && --timeoutCtr > 0);
    }

    return timeoutCtr > 0;
}

void Odrive36::getMotorCurrent(float & zero, float & one) {
    zero = readConfigFloat("axis0.motor.current_control.Iq_measured");
    one = readConfigFloat("axis1.motor.current_control.Iq_measured");
}

void Odrive36::getVoltage(float & voltage) {
    voltage = readConfigFloat("vbus_voltage");
}

bool Odrive36::isAlive() {
    return readConfigFloat("vbus_voltage") != 0.0;
}

void Odrive36::waitForOdrive() {
    while (!isAlive()) {
        static int i = 0;
        if (i % 50 == 0) {
            log("[odrive36] ODrive not found");
        }
        i ++;
    }
    log("[odrive36] Connected");
}

// TODO : This is hilariously slow with the string additions
// (doesnt really matter because it is only done once but I still hate it)
void Odrive36::configureAxis(MotorAxis motor) {
    const std::string axis = motor == MotorAxis::MOTOR_AXIS_ZERO ? "axis0" : "axis1";
    const int endstopGpio = motor == MotorAxis::MOTOR_AXIS_ZERO ? 3 : 4;
    log("[odrive]: Arming " + axis);

    // Brake resistor configs
    writeConfig("config.enable_brake_resistor", true);
    writeConfig("config.brake_resistance", 5.0f);

    // Set controller mode
    writeConfig(axis + ".controller.config.input_mode", static_cast<int>(InputMode::INPUT_MODE_TRAP_TRAJ));

    // Endstop configs
    writeConfig("config.gpio" + std::to_string(endstopGpio) + "_mode", static_cast<int>(GpioMode::GPIO_MODE_DIGITAL));
    writeConfig(axis + ".min_endstop.config.gpio_num", endstopGpio);
    writeConfig(axis + ".min_endstop.config.is_active_high ", false);
    writeConfig(axis + ".min_endstop.config.offset", 0.0f);
    writeConfig(axis + ".min_endstop.config.enabled", true);
    writeConfig(axis + ".max_endstop.config.enabled", false);
    writeConfig("config.gpio" + std::to_string(endstopGpio) + "_mode", static_cast<int>(GpioMode::GPIO_MODE_DIGITAL_PULL_UP));

    // Motor configs
    writeConfig(axis + ".motor.config.current_lim", 20);
    writeConfig(axis + ".motor.config.current_lim_margin", 2);
    writeConfig(axis + ".motor.config.pole_pairs", 11);
    writeConfig(axis + ".motor.config.torque_constant", 0.01333871f);

    // Encoder setup
    writeConfig(axis + ".encoder.config.cpr", 8192);

    // Controller setup
    writeConfig(axis + ".trap_traj.config.vel_limit", 90);
    writeConfig(axis + ".trap_traj.config.accel_limit", 500);
    writeConfig(axis + ".trap_traj.config.decel_limit", 500);
    writeConfig(axis + ".controller.config.vel_limit", 100); // 10% more than the trap_traj setting 
    writeConfig(axis + ".controller.config.vel_gain", 0.03f);
    writeConfig(axis + ".controller.config.pos_gain", 5);

    // Save the configuration & wait for it to reboot
    command(SysCommand::SAVE_CONF);
    waitForOdrive();
}

void Odrive36::writeConfig(const std::string& config, const float value) {
    serial.printf("w %s %.4f\n", config.c_str(), value);
}

void Odrive36::writeConfig(const std::string& config, const int value) {
    serial.printf("w %s %d\n", config.c_str(), value);
}

void Odrive36::writeConfig(const std::string& config, const bool value) {
    serial.printf("w %s %d\n", config.c_str(), value ? 1 : 0);
}

String Odrive36::readString() {
    String str = "";
    static const unsigned long timeout = 1000;
    unsigned long timeout_start = millis();
    for (;;) {
        while (!serial.available()) {
            if (millis() - timeout_start >= timeout) {
                return str;
            }
        }
        char c = serial.read();
        if (c == '\n')
            break;
        str += c;
    }
    return str;
}

float Odrive36::readFloat() {
    String str = readString();
    return str.length() == 0 ? 0 : str.toFloat();
}

int Odrive36::readInt() {
    String str = readString();
    return str.length() == 0 ? 0 : static_cast<int>(str.toInt());
}

float Odrive36::readConfigFloat(const std::string& config) {
    serial.printf("r %s\n", config.c_str());
    return readFloat();
}

bool Odrive36::readConfigBool(const std::string& config) {
    return readConfigInt(config) != 0;
}

int Odrive36::readConfigInt(const std::string& config) {
    serial.printf("r %s\n", config.c_str());
    return readInt();
}

void Odrive36::command(SysCommand command) {
    switch (command) {
        case SysCommand::REBOOT:
            serial.printf("sr\n");
            break;
        case SysCommand::ERASE_CONF:
            serial.printf("se\n");
            break;
        case SysCommand::SAVE_CONF:
            serial.printf("ss\n");
            break;
        case SysCommand::CLEAR_ERR:
            serial.printf("sc\n");
            break;
    }
}

Odrive36::ODriveError::ODriveAxisError::ODriveAxisError(int main, int controller, int motor, int encoder): 
        main(main), controller(controller), motor(motor), encoder(encoder) {}

Odrive36::ODriveError::ODriveError(int main):
        main(main) {}

std::string Odrive36::ODriveError::toString(void) {
    std::stringstream result;

    result 
        << "ODriveError {main : " << main << ", "
        << "axis0 : { "
        << "main : " << axis0.main << ", "
        << "controller : " << axis0.controller << ", "
        << "motor : " << axis0.motor << ", "
        << "encoder : " << axis0.encoder << " }, "
        << "axis1 : { "
        << "main : " << axis1.main << ", "
        << "controller : " << axis1.controller << ", "
        << "motor : " << axis1.motor << ", "
        << "encoder : " << axis1.encoder << " } }";

    return result.str();
}

bool Odrive36::checkErrorsAxis(MotorAxis axis) {
    std::string axisStr = axis == MotorAxis::MOTOR_AXIS_ZERO ? "axis0" : "axis1";

    int main            = readConfigInt("error");
    int axisMain         = readConfigInt(axisStr + ".error");
    int axisController  = readConfigInt(axisStr + ".controller.error");
    int axisMotor       = readConfigInt(axisStr + ".motor.error");
    int axisEncoder     = readConfigInt(axisStr + ".encoder.error");

    error.main = main;
    ODriveError::ODriveAxisError axisErr(axisMain, axisController, axisMotor, axisEncoder);

    if (axis == MotorAxis::MOTOR_AXIS_ZERO) {
        error.axis0 = axisErr;
    } else {
        error.axis1 = axisErr;
    }

    return main || axisMain || axisController || axisMotor || axisEncoder;
}

// void Odrive36::idleAxis(MotorAxis motor) {
//     runState(MotorAxis::MOTOR_AXIS_ZERO, AxisState::AXIS_STATE_IDLE);
//     runState(MotorAxis::MOTOR_AXIS_ONE, AxisState::AXIS_STATE_IDLE);
// }


void Odrive36::getPosition(MotorAxis motor, float& requested, float& position, float& velocity) {
    const int motorId = motor == MotorAxis::MOTOR_AXIS_ZERO ? 0 : 1;
    serial.printf("f %d\n", motorId);
    position = readFloat();
    velocity = readFloat();
    requested = motor == MotorAxis::MOTOR_AXIS_ZERO ? axis0Requested : axis1Requested;
}

void Odrive36::poll(std::string config) {
    static uint64_t prev = millis();
    uint64_t time = millis();

    // Only send commands at <= 10Hz
    if (time - prev < 100) {
        return;
    }
    prev = time;

    log("[odrive36]: Read config " + config + " : " + std::to_string(readConfigInt(config)));
}

