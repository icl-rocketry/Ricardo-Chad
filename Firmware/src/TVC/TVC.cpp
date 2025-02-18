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
