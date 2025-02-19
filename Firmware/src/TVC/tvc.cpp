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

#include "tvc/tvc.h"
#include "tvc/odrive36.h"

#include "tvc/states/tvcTypes.h"
#include "tvc/states/default.h"

#include <librnp/default_packets/simplecommandpacket.h>

#include <libriccore/riccorelogging.h>

#define log(x) RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(x)

TVC::TVC(RnpNetworkManager &networkManager):
        NRCRemoteActuatorBase(networkManager),
        networkManager(networkManager),
        stateMachine(std::make_unique<Default>(odrv)) {}

int TVC::requestControl(float xAxis, float yAxis) {
    odrv.commandAxisTurns(xAxis, yAxis);
    return 0;
}

int TVC::arm(void) {
    bool ax0 = odrv.armAxis(Odrive36::MotorAxis::MOTOR_AXIS_ZERO);
    // odrv.armAxis(Odrive36::MotorAxis::MOTOR_AXIS_ONE);
    return ax0 ? 1 : 0;
}

int TVC::lock(void) {
    odrv.lockAxis(Odrive36::MotorAxis::MOTOR_AXIS_ZERO);
    // odrv.lockAxis(Odrive36::MotorAxis::MOTOR_AXIS_ONE);
    return 0;
}

int TVC::idle(void)
{
    odrv.idleAxis(Odrive36::MotorAxis::MOTOR_AXIS_ZERO);
    // odrv.idleAxis(Odrive36::MotorAxis::MOTOR_AXIS_ONE);
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

void TVC::execute_base(int32_t arg) {
    switch (arg) {
        case LOCK:
            lock();
            break;
        case EXECUTE:
            odrv.start();
            break;
    }
}
