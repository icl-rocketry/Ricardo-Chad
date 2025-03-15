/**
 * @file tvc.cpp
 * @author Riley Horrix (rh1122@ic.ac.uk)
 * @brief Implementation of TVC interface
 * @version 0.1
 * @date 2025-02-17
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "TVC/tvc.h"

#include "TVC/odrive36.h"
#include <librnp/default_packets/simplecommandpacket.h>

#include <libriccore/riccorelogging.h>
#include "tvc.h"

#define log(x) RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(x)

// Current states
#define LOCK        0
#define PROGRAM_1   1
#define PROGRAM_2   2
#define PROGRAM_3   3
#define REQ_TELEM   8

TVC::TVC(RnpNetworkManager &networkManager):
        NRCRemoteActuatorBase(networkManager),
        networkManager(networkManager) {}

int TVC::lock(void) {
    float x;
    float x2;
    float x3;
    float y;
    float y2;
    float y3;
    odrv.getPosition(Odrive36::MotorAxis::MOTOR_AXIS_ZERO, x, x2, x3);
    odrv.getPosition(Odrive36::MotorAxis::MOTOR_AXIS_ONE, y, y2, y3);
    odrv.commandAxisTurns(x, y);
    return 0;
}

void TVC::arm_base(int32_t arg) {
    if (arg == 0) {
        log("[tvc] Skipping ODrive configure.");
    } else {
        log("[tvc] Configuring ODrive.");
        odrv.configureAxis(Odrive36::MotorAxis::MOTOR_AXIS_ZERO);
        odrv.configureAxis(Odrive36::MotorAxis::MOTOR_AXIS_ONE);
    }

    log("[tvc] Arming ODrive.");

    if (!odrv.armAxis(Odrive36::MotorAxis::MOTOR_AXIS_ZERO)) {
        log("[tvc] Failure arming axis zero.");
        return;
    }

    if (!odrv.armAxis(Odrive36::MotorAxis::MOTOR_AXIS_ONE)) {
        log("[tvc] Failure arming axis one.");
        return;
    }

    this->_state.deleteFlag(LIBRRC::COMPONENT_STATUS_FLAGS::DISARMED);
    this->_state.newFlag(LIBRRC::COMPONENT_STATUS_FLAGS::NOMINAL);
}

void TVC::disarm_base() {
    log("[tvc] Disarming ODrive.");

    bool success =
        odrv.idleAxis(Odrive36::MotorAxis::MOTOR_AXIS_ONE) ||
        odrv.idleAxis(Odrive36::MotorAxis::MOTOR_AXIS_ZERO);

    if (!success) {
        log("[tvc] Failed to disarm!");
    } else {
        this->_state.deleteFlag(LIBRRC::COMPONENT_STATUS_FLAGS::NOMINAL);
        this->_state.newFlag(LIBRRC::COMPONENT_STATUS_FLAGS::DISARMED);
    }
}

void circle_program(uint64_t time_ms, float& x, float& y) {
    // static const float a = 0.6;
    // at22 = a * (time / 1000 - 2) + 1 = a * time - b
    static const float a = 0.0006;
    static const float b = -0.2;
    const float at22 = a * time_ms + b;

    // Normalise to [0, 1] * amplitude.
    x = time_ms < 2000 ? 0.5 :
        time_ms < 12000 ? 0.5 * sin(PI * at22 * at22) + 0.5 :
            0.5;

    y = time_ms < 1000 ? 0.5 :
        time_ms < 2000 ? 0.0005 * time_ms :
        time_ms < 12000 ? -0.5 * cos(PI * at22 * at22) + 0.5 :
            0.5;
}

void square_program(uint64_t time_ms, float& x, float& y) {
    // {a = time_s} x = mod2(floor(0.8a^2 -6a - 80))
    // {a = time_s - 5} y = mod2(floor(0.8a^2 -6a - 80))
    float time_s = time_ms / 1000.0;
    if (time_s < 5.0) {
        x = static_cast<float>(static_cast<uint64_t>(floor(0.8f * time_s * time_s - 6.0f * time_s - 80.0f)) % 2);
        y = 0.5;
    } else if (time_s < 10.0) {
        x = 0.5;
        time_s -= 5.0; // Shift program left
        y = static_cast<float>(static_cast<uint64_t>(floor(0.8f * time_s * time_s - 6.0f * time_s - 80.0f)) % 2);
    } else {
        x = 0.5;
        y = 0.5;
    }
}

void TVC::update() {
    // If not executing a program then return
    if (currentProgram != PROGRAM_1 && currentProgram != PROGRAM_2 && currentProgram != PROGRAM_3) {
        return;
    }

    float x;
    float y;
    uint64_t time_ms = millis();
    if (currentProgram == PROGRAM_1) {
        circle_program(time_ms - time_execute, x, y);
    }
    if (currentProgram == PROGRAM_2) {
        square_program(time_ms - time_execute, x, y);
    }
    if (currentProgram == PROGRAM_3) {
        x = 0.5;
        y = 0.5;
    }

    odrv.commandAxisControl(x, y);
}


void TVC::execute_base(int32_t arg) {
    if (arg != REQ_TELEM) {
        currentProgram = arg;
    }

    switch (arg) {
        case LOCK:
            lock();
            break;
        case PROGRAM_1:
        case PROGRAM_2:
        case PROGRAM_3:
            time_execute = millis();
            break;
        case REQ_TELEM:
            requestTelem();
            break;
    }
}

void TVC::requestTelem() {
    static uint64_t lastTelem = millis();
    uint64_t currentTime = millis();

    if ((currentTime - lastTelem) < 20) {
        networkManager.sendPacket(telemPacket);
        return;
    }

    lastTelem = currentTime;

    odrv.getPosition(Odrive36::MotorAxis::MOTOR_AXIS_ZERO, telemPacket.axis0Requested, telemPacket.axis0Turns, telemPacket.axis0Velocity);
    odrv.getPosition(Odrive36::MotorAxis::MOTOR_AXIS_ONE, telemPacket.axis1Requested, telemPacket.axis1Turns, telemPacket.axis1Velocity);

    telemPacket.time = millis();
    telemPacket.state = currentProgram;

    odrv.getMotorCurrent(telemPacket.axis0Current, telemPacket.axis1Current);
    odrv.getVoltage(telemPacket.vbusVoltage);

    networkManager.sendPacket(telemPacket);
}
