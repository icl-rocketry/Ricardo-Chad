#include "ODriveController.h"

#include <ArduinoJson.h>
#include <librrc/Helpers/jsonconfighelper.h>
#include <libriccore/riccorelogging.h>

//! @brief Baud rate of UART connection for the ODrive
const unsigned int UART_BAUD = 115200;

const int X_AXIS = 0;
const int Y_AXIS = 1;

// Odrive UART RX = 1, TX = 2

// Print with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 5); return obj; }
template<>        inline Print& operator <<(Print &obj, std::string arg) { obj.print(arg.c_str()); return obj; }
template<>        inline Print& operator <<(Print &obj, bool arg) { obj.print(arg ? 1 : 0); return obj; }

ODriveController::ODriveController(Stream& serial, float turnRange, float currentTurns): serial(serial), turnRange(turnRange), currentTurns(currentTurns) {}


// ODriveController ODriveController::fromConfig(JsonObjectConst config) {
//     std::string connectionType = LIBRRC::JsonConfigHelper::getIfContains<std::string>(config, "connection_type");

//     ODriveController controller;

//     if (connectionType == "CAN") {
//         controller.connection = Connection::CAN;

//         // CAN controller not supported for now since there is no current easy
//         // way to receive CAN messages from the ODrive since it uses a 
//         // different format.
//         // This is TODO
//         throw "[ODriveController] CAN controller not supported";

//     } else if (connectionType == "UART") {
//         controller.connection = Connection::UART;

//         int uartTxPin = LIBRRC::JsonConfigHelper::getIfContains<int>(config, "uart_tx_pin");
//         int uartRxPin = LIBRRC::JsonConfigHelper::getIfContains<int>(config, "uart_rx_pin");

//         controller.uartTxPin = uartTxPin;
//         controller.uartRxPin = uartRxPin;


//     } else {
//         throw "[ODriveController] Invalid connection config type";
//     }

//     float turnRange = LIBRRC::JsonConfigHelper::getIfContains<float>(config, "turn_range");
//     controller.turnRange = turnRange;

//     float currentTurns = LIBRRC::JsonConfigHelper::getIfContains<float>(config, "turn_initial");
//     controller.currentTurns = std::min(currentTurns, turnRange);

//     return controller;
// }

bool ODriveController::status(int timeout) {
    float voltage = 0.0f;
    int it = 0;
    do {
        serial << "r vbus_voltage\n";
        voltage = readFloat();
        delay(timeout);
    } while (voltage == 0.0f && it++ < 20);
    return it < 20;
}

void ODriveController::printDebug() {
    readConfig("error");
    const int error = readInt();
    const std::string err = "Error code : " + std::to_string(error) + "\n";
    // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(err);
}

void ODriveController::position(float axis0, float axis1) {
    // Constrain Axis Values
    // axis0 = axis0 > 1.0f ? 1.0f : (axis0 < 0.0f ? 0.0f : axis0);
    axis1 = axis1 > 1.0f ? 1.0f : (axis1 < 0.0f ? 0.0f : axis1);

    const float axis0Turns = axis0 * turnRange - currentTurns;
    // const float axis1Turns = axis1 * turnRange - currentTurns;

    // serial << "p 0 " << 100 << "\n";
    const std::string turns = "turn req : " + std::to_string(axis0Turns) + " ax0 : " + std::to_string(axis0) + "\n";
    // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(turns);
    setPosition(0, axis0Turns);
    // setPosition(1, axis1Turns);
}

ODriveController::operator bool() {
    return status();
};

void ODriveController::calibrateAxis(Axis axis) {
    run_state(axis == ZERO ? 0 : 1, AXIS_STATE_FULL_CALIBRATION_SEQUENCE);
    run_state(axis == ZERO ? 0 : 1, AXIS_STATE_ENCODER_OFFSET_CALIBRATION);
    // run_state(axis == ZERO ? 0 : 1, AXIS_STATE_HOMING);
}

void ODriveController::arm() {
    run_state(0, AXIS_STATE_CLOSED_LOOP_CONTROL);
    // run_state(1, AXIS_STATE_CLOSED_LOOP_CONTROL);
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
    serial << "p " << motor_number  << " " << position << " " << velocity_feedforward << " " << current_feedforward << "\n";
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
    return readString().toFloat();
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
    return readString().toInt();
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
