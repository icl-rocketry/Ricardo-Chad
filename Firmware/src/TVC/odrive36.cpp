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

#include "tvc/odrive36.h"
#include "tvc/odriveEnums.h"
#include "odrive36.h"

#define log(x) RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(x)

/// @brief Board UART transmit pin.
const int TX_PIN = 6;

/// @brief Board UART receive pin.
const int RX_PIN = 5;

/// @brief UART baud rate.
const int UART_BAUD = 115200;

Odrive36::Odrive36(float maxTurns): 
        maxTurns(maxTurns),
        serial(Serial1),
        controlType(ControlType::TRAP_TRAJ) {
    Serial1.begin(UART_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
    delay(10);  // Wait for serial connection
    configureAxis(MotorAxis::MOTOR_AXIS_ZERO);
    // configureAxis(MotorAxis::MOTOR_AXIS_ONE);
}

void Odrive36::commandAxisTurns(float axisZero, float axisOne) {
    if (!armed) {
        log("[odrive36] Odrive motor not armed!");
        return;
    }
    axisZero = constrain(axisZero, 0, maxTurns);
    axisOne = constrain(axisOne, 0, maxTurns);

    switch (controlType) {
        case ControlType::TRAP_TRAJ:
            serial.printf("t %d %.4f\n", static_cast<int>(MotorAxis::MOTOR_AXIS_ZERO), axisZero);
            // serial.printf("t %d %.4f\n", static_cast<int>(MotorAxis::MOTOR_AXIS_ONE), axisOne);
            break;
        
        default:
            break;
    }
}

bool Odrive36::armAxis(MotorAxis motor) {
    if (!configured) {
        log("[odrive36] Tried to arm unconfigured odrive!");
        return false;
    }
    log("[odrive36] Starting Motor Calibration.");
    runState(motor, AxisState::AXIS_STATE_MOTOR_CALIBRATION);
    log("[odrive36] Starting Encoder Offset Calibration.");
    runState(motor, AxisState::AXIS_STATE_ENCODER_OFFSET_CALIBRATION);
    log("[odrive36] Starting Homing Calibration.");
    delay(100);
    runState(motor, AxisState::AXIS_STATE_HOMING, true, 10000);
    log("[odrive36] Calibration Complete.");
    armed = true;

    return true; // TODO
}

bool Odrive36::runState(MotorAxis axis, AxisState requestedState, bool waitForIdle, float timeout) {
    int timeoutCtr = static_cast<int>(timeout / 50.0f);
    serial.printf("w axis%d.requested_state %d\n", static_cast<int>(axis), static_cast<int>(requestedState));
    if (waitForIdle) {
        do {
            delay(50);
            serial.printf("r axis%d.current_state\n", static_cast<int>(axis));
        } while (readInt() != static_cast<int>(AxisState::AXIS_STATE_IDLE) && --timeoutCtr > 0);
    }

    return timeoutCtr > 0;
}

// TODO : This is hilariously slow with the string additions
// (doesnt really matter because it is only done once but i still hate it)
void Odrive36::configureAxis(MotorAxis motor) {
    const std::string axis = motor == MotorAxis::MOTOR_AXIS_ZERO ? "axis0" : "axis1";

    // Endstop Configs
    writeConfig("config.gpio4_mode", static_cast<int>(GpioMode::GPIO_MODE_DIGITAL));
    writeConfig(axis + ".min_endstop.config.debounce_ms", 50);
    writeConfig(axis + ".min_endstop.config.gpio_num", 4);
    writeConfig(axis + ".min_endstop.config.is_active_high ", false);
    writeConfig(axis + ".min_endstop.config.offset", -0.25f);
    writeConfig(axis + ".min_endstop.config.enabled", true);
    writeConfig(axis + ".max_endstop.config.enabled", false);
    writeConfig("config.gpio4_mode", static_cast<int>(GpioMode::GPIO_MODE_DIGITAL_PULL_UP));
    writeConfig(axis + ".controller.config.homing_speed", 0.25f);

    // Motor Configs
    writeConfig(axis + ".motor.config.current_lim", 20);
    writeConfig(axis + ".motor.config.current_lim_margin", 2);
    writeConfig(axis + ".motor.config.pole_pairs", 11);
    writeConfig(axis + ".motor.config.torque_constant", 0.01333871f);

    // Encoder Setup
    writeConfig(axis + ".encoder.config.calib_scan_distance", 20);
    writeConfig(axis + ".encoder.config.cpr", 8192);

    // Controller Setup
    writeConfig(axis + ".trap_traj.config.vel_limit", 100);
    writeConfig(axis + ".trap_traj.config.accel_limit", 1000);
    writeConfig(axis + ".trap_traj.config.decel_limit", 1000);
    writeConfig(axis + ".controller.config.vel_limit", 110); // 10% more than the trap_traj setting 
    writeConfig(axis + ".controller.config.vel_ramp_rate", 1);
    writeConfig(axis + ".controller.config.vel_gain", 0.02f);
    writeConfig(axis + ".controller.config.pos_gain", 5);
    configured = true;
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

void Odrive36::update() {
    if (!executing) {
        return;
    }
    static uint64_t prev = millis();
    uint64_t time = millis();

    // Only send commands at <= 60Hz
    if (time - prev < 20) {
        return;
    }

    prev = time;

    static float i = 0;
    i += 0.2;

    float command = (sin(i) + 1.0f) / 2.0f;

    commandAxisTurns(command * 10.0f, 0);
}

void Odrive36::lockAxis(MotorAxis motor) {
    executing = false;
}
