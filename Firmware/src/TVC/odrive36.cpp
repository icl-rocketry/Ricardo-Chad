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

#include "Odrive36.h"

#define log(x) RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(x)

template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 3); return obj; }
template<>        inline Print& operator <<(Print &obj, std::string arg) { obj.print(arg.c_str()); return obj; }
template<>        inline Print& operator <<(Print &obj, bool arg) { obj.print(arg ? 1 : 0); return obj; }

/// @brief Board UART transmit pin.
const int TX_PIN = 6

/// @brief Board UART receive pin.
const int RX_PIN = 5

/// @brief UART baud rate.
const int UART_BAUD = 115200;

Odrive36::Odrive36(float maxTurns): 
        maxTurns(maxTurn),
        serial(Serial1),
        controlType(ControlType.TRAP_TRAJ) {
    serial.begin(UART_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
    delay(10);  // Wait for serial connection

}

void Odrive36::commandAxisTurns(float axisZero, float axisOne) {
    if (!armed) {
        log("[odrive36] Odrive motor not armed!");
        return;
    }
    axisZero = constrain(axisZero, 0, maxTurn);
    axisOne = constrain(axisOne, 0, maxTurn);

    switch (controlType) {
        case ControlType.TRAP_TRAJ:
            serial.printf("t %d %.4f\n", MotorAxis.MOTOR_AXIS_ZERO, axisZero);
            serial.printf("t %d %.4f\n", MotorAxis.MOTOR_AXIS_ONE, axisOne);
            break;
        
        default:
            break;
    }
}

bool Odrive36::armAxis(MotorAxis motor) {
    if (!configured) {
        log("[odrive36] Tried to arm unconfigured odrive!")
        return;
    }
    log("[odrive36] Starting Motor Calibration.");
    run_state(motor, AxisState.AXIS_STATE_MOTOR_CALIBRATION);
    log("[odrive36] Starting Encoder Offset Calibration.");
    run_state(motor, AxisState.AXIS_STATE_ENCODER_OFFSET_CALIBRATION);
    log("[odrive36] Starting Homing Calibration.");
    delay(100);
    run_state(motor, AxisState.AXIS_STATE_HOMING, true, 10000);
    log("[odrive36] Calibration Complete.");
    armed = true;
}

bool Odrive36::runState(MotorAxis axis, AxisState requestedState, bool waitForIdle, float timeout) {
    int timeoutCtr = std::static_cast<int>(timeout / 50.0f);
    serial.printf("w axis%d.requested_state %d\n", axis, requestedState);
    if (waitForIdle) {
        do {
            delay(50);
            serial << "r axis" << axis << ".current_state\n";
        } while (readInt() != AXIS_STATE_IDLE && --timeoutCtr > 0);
    }

    return timeout_ctr > 0;
}

// TODO : This is hilariously slow with the string additions
// (doesnt really matter because it is only done once but i still hate it)
void Odrive36::configureAxis(MotorAxis axis) {
    const std::string axis = actuator == MotorAxis::MOTOR_AXIS_ZERO ? "axis0" : "axis1";

    // Endstop Configs
    controller.writeConfig("config.gpio4_mode", GPIO_MODE_DIGITAL);
    controller.writeConfig(axis + ".min_endstop.config.debounce_ms", 50);
    controller.writeConfig(axis + ".min_endstop.config.gpio_num", 4);
    controller.writeConfig(axis + ".min_endstop.config.is_active_high ", false);
    controller.writeConfig(axis + ".min_endstop.config.offset", -0.25);
    controller.writeConfig(axis + ".min_endstop.config.enabled", true);
    controller.writeConfig(axis + ".max_endstop.config.enabled", false);
    controller.writeConfig("config.gpio4_mode", GPIO_MODE_DIGITAL_PULL_UP);
    controller.writeConfig(axis + ".controller.config.homing_speed", 0.25f);

    // Motor Configs
    controller.writeConfig(axis + ".motor.config.current_lim", 20);
    controller.writeConfig(axis + ".motor.config.current_lim_margin", 2);
    controller.writeConfig(axis + ".motor.config.pole_pairs", 11);
    controller.writeConfig(axis + ".motor.config.torque_constant", 0.01333871f);

    // Encoder Setup
    controller.writeConfig(axis + ".encoder.config.calib_scan_distance", 20);
    controller.writeConfig(axis + ".encoder.config.cpr", 8192);

    // Controller Setup
    controller.writeConfig(axis + ".trap_traj.config.vel_limit", 100);
    controller.writeConfig(axis + ".trap_traj.config.accel_limit", 1000);
    controller.writeConfig(axis + ".trap_traj.config.decel_limit", 1000);
    controller.writeConfig(axis + ".controller.config.vel_limit", 110); // 10% more than the trap_traj setting 
    controller.writeConfig(axis + ".controller.config.vel_ramp_rate", 1);
    controller.writeConfig(axis + ".controller.config.vel_gain", 0.02f);
    controller.writeConfig(axis + ".controller.config.pos_gain", 5);
    configured = true;
}

void Odrive36::writeConfig(const std::string& config, const float value) {
    serial.printf("w %s %.4f\n", config.c_str(), value);
}

void Odrive36::writeConfig(const std::string& config, const int value) {
    serial.printf("w %s %f\n", config.c_str(), value);
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

float Odrive36::readConfig<float>(const std::string& config) {
    serial.printf("r %s\n", config.c_str());
    return readFloat();
}

int Odrive36::readConfig<int>(const std::string& config) {
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
