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

#define log(x) RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(x)

TVC::TVC(RnpNetworkManager &networkManager):
        NRCRemoteActuatorBase(networkManager),
        networkManager(networkManager) {}

int TVC::requestControl(float xAxis, float yAxis) {
    odrv.commandAxisTurns(xAxis, yAxis);
    return 0;
}

int TVC::arm(void) {
    bool ax0 = odrv.armAxis(Odrive36::MotorAxis::MOTOR_AXIS_ZERO);
    bool ax1 = odrv.armAxis(Odrive36::MotorAxis::MOTOR_AXIS_ONE);
    bool res = ax0 && ax1;
    if (res) {
        odrv.commandAxisTurns(10.0, 10.0);
    }
    return res ? 1 : 0;
}

int TVC::lock(void) {
    odrv.lockAxis(Odrive36::MotorAxis::MOTOR_AXIS_ZERO);
    odrv.lockAxis(Odrive36::MotorAxis::MOTOR_AXIS_ONE);
    return 0;
}

int TVC::idle(void) {
    odrv.idleAxis(Odrive36::MotorAxis::MOTOR_AXIS_ZERO);
    odrv.idleAxis(Odrive36::MotorAxis::MOTOR_AXIS_ONE);
    return 0;
}

void TVC::disarm(void) {
    odrv.disarmAxis();
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

void circle_program(uint64_t time_ms, float& x, float& y) {
    static const float a = 0.6;
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
}

void TVC::update() {
    if (running) {
        float x;
        float y;
        uint64_t time_ms = millis();
        circle_program(time_ms - time_execute, x, y);
        odrv.commandAxisTurns(x * maxTurns, y * maxTurns);
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
