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

#include "TVC/states/tvcTypes.h"
#include "TVC/states/default.h"

#include <librnp/default_packets/simplecommandpacket.h>

#include <libriccore/riccorelogging.h>

#define log(x) RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(x)

TVC::TVC(RnpNetworkManager &networkManager):
        NRCRemoteActuatorBase(networkManager),
        networkManager(networkManager) {
    // Initialise state machine to default
    stateMachine.initalize(std::make_unique<TVCDefault>(tvcStatus));

    // Set both internal states to show none.
    tvcArmedStatus.newFlag(ARMED_STATUS_FLAGS::NONE);
    tvcExecutingStatus.newFlag(EXECUTING_STATUS_FLAGS::NONE);
}

int TVC::requestControl(float xAxis, float yAxis) {
    odrive.commandAxisTurns(xAxis, yAxis);
    return 0;
}

int TVC::arm(void) {
    bool ax0 = odrive.armAxis(Odrive36::MotorAxis::MOTOR_AXIS_ZERO);
    bool ax1 = odrive.armAxis(Odrive36::MotorAxis::MOTOR_AXIS_ONE);
    bool res = ax0 && ax1;
    if (res) {
        odrive.commandAxisTurns(maxTurns / 2.0, maxTurns / 2.0);
    }
    return res ? 1 : 0;
}

int TVC::lock(void) {
    odrive.lockAxis(Odrive36::MotorAxis::MOTOR_AXIS_ZERO);
    odrive.lockAxis(Odrive36::MotorAxis::MOTOR_AXIS_ONE);
    return 0;
}

int TVC::idle(void) {
    odrive.idleAxis(Odrive36::MotorAxis::MOTOR_AXIS_ZERO);
    odrive.idleAxis(Odrive36::MotorAxis::MOTOR_AXIS_ONE);
    return 0;
}

void TVC::disarm(void) {
    odrive.disarmAxis();
}

void TVC::arm_base(int32_t arg) {
    if(arm()) {
        this->_state.deleteFlag(LIBRRC::COMPONENT_STATUS_FLAGS::DISARMED);
        this->_state.newFlag(LIBRRC::COMPONENT_STATUS_FLAGS::NOMINAL);
        log("[tvc]: arm good.");
    } else {
        log("[tvc]: FAILED TO ARM.");
    }
}

void TVC::disarm_base() {
    disarm();
}

#define LOCK 0x0
#define EXECUTE 0x1

void program(uint64_t time_ms, float& x, float& y) {
    const float amplitude = 1.0;

    // Normalise to [0, 1] * amplitude. 
    x = sin(time_ms / 1000.0) + 1.0 * (amplitude / 2.0);

    // Initially dont move y.
    if (time_ms / 1000.0 < (3.14159266 / 2.0)) { 
        y = 0;
    }
    y = cos(time_ms / 1000.0) + 1.0 * (amplitude / 2.0);
}

void TVC::update() {
    if (running) {
        float x;
        float y;
        uint64_t time_ms = millis();
        program(time_ms - time_execute, x, y);
        odrive.commandAxisTurns(x, y);
    }
}


void TVC::execute_base(int32_t arg) {
    switch (arg) {
        case LOCK:
            lock();
            break;
        case EXECUTE:
            running = true;
            time_execute = millis();
            break;
    }
}
