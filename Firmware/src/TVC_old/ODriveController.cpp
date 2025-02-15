#include "ODriveController.h"

#include <ArduinoJson.h>
#include <librrc/Helpers/jsonconfighelper.h>
#include <libriccore/riccorelogging.h>

//! @brief Baud rate of UART connection for the ODrive
const unsigned int UART_BAUD = 115200;

const int X_AXIS = 0;
const int Y_AXIS = 1;

// template<RicCoreLoggingConfig::LOGGERS Conf>
// using log_impl = RicCoreLogging::log<Conf>;
// using log = log_impl<RicCoreLoggingConfig::LOGGERS::SYS>;

#define log(x) RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(x)

// Odrive UART RX = 1, TX = 2

// Print with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 3); return obj; }
template<>        inline Print& operator <<(Print &obj, std::string arg) { obj.print(arg.c_str()); return obj; }
template<>        inline Print& operator <<(Print &obj, bool arg) { obj.print(arg ? 1 : 0); return obj; }

ODriveController::ODriveController(Stream& serial, float turnRange): serial(serial), turnRange(turnRange) {}

bool ODriveController::available() {
    float voltage = readConfigFloat("vbus_voltage");
    int it = 0;

    while (voltage == 0.0f && it++ < 5) {
        delay(5);
        voltage = readConfigFloat("vbus_voltage");
    }

    return it < 5;
}

void ODriveController::printDebug() {
    const int error = this->error();
    const std::string err = "Error code : " + std::to_string(error) + "\n";
    // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(err);
}

int ODriveController::error() {
    return readConfigInt("error");
}

int ODriveController::error(Axis axis) {
    return readConfigInt(std::string(axis == Axis::ZERO ? "axis0" : "axis1") + ".error");
}

void ODriveController::fullErrors(Axis axis, int& main, int& axisErr, int& motor, int& controller) {
    std::string axisStr = axis == Axis::ZERO ? "axis0" : "axis1";
    main = readConfigInt("error");
    axisErr = readConfigInt(axisStr + ".error");
    controller = readConfigInt(axisStr + ".controller.error");
    motor = readConfigInt(axisStr + ".motor.error");
}

void ODriveController::position(float axis0, float axis1) {
    // Constrain Axis Values
    axis0 = axis0 > 1.0f ? 1.0f : (axis0 < 0.0f ? 0.0f : axis0);
    axis1 = axis1 > 1.0f ? 1.0f : (axis1 < 0.0f ? 0.0f : axis1);

    // serial << "p 0 " << 100 << "\n";
    // const std::string turns = "turn req : " + std::to_string(axis0Turns) + " ax0 : " + std::to_string(axis0) + "\n";
    // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(turns);
    setPosition(0, axis0 * turnRange);
    // setPosition(1, axis1Turns);
}

ODriveController::operator bool() {
    return available();
};

void ODriveController::calibrateAxis(Axis axis_) {
    int axis = axis_ == ZERO ? 0 : 1;

    log("\nStarting Calibration\n");
    run_state(axis, AXIS_STATE_MOTOR_CALIBRATION);
    log("\nMotor Calibration Complete\n");
    run_state(axis, AXIS_STATE_ENCODER_OFFSET_CALIBRATION);
    log("\nEncoder Calibration Complete\n");
    // log("\nStarting Homing in 3s\n");
    // delay(3000);
    // run_state(axis, AXIS_STATE_HOMING, true, 100.0f);
    log("\nHoming Calibration Complete\n");

    float turns = 0;
    float vel = 0;
    requestFeedback(axis_, turns, vel);

    std::string logging = "\nCalibration : t " + std::to_string(turns) + "\n";
    log(logging);
}

void ODriveController::arm(Axis axis_) {
    int axis = axis_ == Axis::ZERO ? 0 : 1;
    run_state(axis, AXIS_STATE_CLOSED_LOOP_CONTROL);
    writeConfig(axis + ".controller.config.input_mode", INPUT_MODE_TRAP_TRAJ);
}

void ODriveController::requestFeedback(Axis axis, float &position, float &velocity) {
    serial << "f " << (axis == ZERO ? 0 : 1) << "\n";
    position = readFloat();
    velocity = readFloat();
}

/**
MIT License

Copyright (c) 2017 Oskar Weigl

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


void ODriveController::setPosition(int motor_number, float position) {
    setPosition(motor_number, position, 0.0f, 0.0f);
}

void ODriveController::setPosition(int motor_number, float position, float velocity_feedforward) {
    setPosition(motor_number, position, velocity_feedforward, 0.0f);
}

void ODriveController::setPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward) {
    assert(motor_number == 0 || motor_number == 1);
    log("setting position : " + std::to_string(position));
    serial.printf("t %d %.4f\n", motor_number, position);
}

void ODriveController::setVelocity(int motor_number, float velocity) {
    setVelocity(motor_number, velocity, 0.0f);
}

void ODriveController::setVelocity(int motor_number, float velocity, float current_feedforward) {
    serial << "v " << motor_number  << " " << velocity << " " << current_feedforward << "\n";
}

void ODriveController::setCurrent(int motor_number, float current) {
    serial << "c " << motor_number << " " << current << "\n";
}

void ODriveController::trapezoidalMove(int motor_number, float position) {
    serial << "t " << motor_number << " " << position << "\n";
}

float ODriveController::readFloat() {
    String str = readString();
    return str.length() == 0 ? 0 : str.toFloat();
}

float ODriveController::getVelocity(int motor_number) {
	serial<< "r axis" << motor_number << ".encoder.vel_estimate\n";
	return ODriveController::readFloat();
}

float ODriveController::getPosition(int motor_number) {
    serial << "r axis" << motor_number << ".encoder.pos_estimate\n";
    return ODriveController::readFloat();
}

int32_t ODriveController::readInt() {
    String str = readString();
    return str.length() == 0 ? 0 : str.toInt();
}

bool ODriveController::run_state(int axis, int requested_state, bool wait_for_idle, float timeout) {
    int timeout_ctr = (int)(timeout * 10.0f);
    serial << "w axis" << axis << ".requested_state " << requested_state << '\n';
    if (wait_for_idle) {
        do {
            delay(100);
            serial << "r axis" << axis << ".current_state\n";
        } while (readInt() != AXIS_STATE_IDLE && --timeout_ctr > 0);
    }

    return timeout_ctr > 0;
}

void ODriveController::writeConfig(const std::string& config, const float value) {
    serial << "w " << config << " " << value << "\n";
}

void ODriveController::writeConfig(const std::string& config, const int value) {
    serial << "w " << config << " " << value << "\n";
}

void ODriveController::writeConfig(const std::string& config, const bool value) {
    serial << "w " << config << " " << value << "\n";
}

void ODriveController::readConfig(const std::string& config) {
    serial << "r " << config << "\n";
}

float ODriveController::readConfigFloat(const std::string& config) {
    serial << "r " << config << "\n";
    return readFloat();
}

int ODriveController::readConfigInt(const std::string& config) {
    serial << "r " << config << "\n";
    return readInt();
}

void ODriveController::command(SysCommand command) {
    switch (command) {
        case SysCommand::REBOOT:
            serial << "sr\n";
            break;
        case SysCommand::ERASE_CONF:
            serial << "se\n";
            break;
        case SysCommand::SAVE_CONF:
            serial << "ss\n";
            break;
        case SysCommand::CLEAR_ERR:
            serial << "sc\n";
            break;
    }
}


String ODriveController::readString() {
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
