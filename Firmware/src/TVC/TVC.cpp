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
#include "tvc.h"

TVC::TVC(RnpNetworkManager &networkManager):
        NRCRemoteActuatorBase(networkManager),
        networkManager(networkManager) {}

int TVC::requestControl(float xAxis, float yAxis) {
    odrv.commandAxisTurns(xAxis, yAxis);
    return 0;
}

int TVC::arm(void) {
    odrv.armAxis(Odrive36::MotorAxis::MOTOR_AXIS_ZERO);
    odrv.armAxis(Odrive36::MotorAxis::MOTOR_AXIS_ONE);
    return 0;
}

int TVC::lock(void) {
    odrv.lockAxis(Odrive36::MotorAxis::MOTOR_AXIS_ZERO);
    odrv.lockAxis(Odrive36::MotorAxis::MOTOR_AXIS_ONE);
    return 0;
}

int TVC::idle(void)
{
    odrv.idleAxis(Odrive36::MotorAxis::MOTOR_AXIS_ZERO);
    odrv.idleAxis(Odrive36::MotorAxis::MOTOR_AXIS_ONE);
    return 0;
}

void TVC::disarm(void) {
    odrv.disarmAxis();
}

void TVC::arm_base(int32_t arg) {
    arm();
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
