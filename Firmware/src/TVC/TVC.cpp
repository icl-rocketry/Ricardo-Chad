#include "TVC.h"
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
TVC::TVC(RnpNetworkManager &networkManager):
        networkManager(networkManager) {}



int TVC::requestControl(float xAxis, float yAxis) {
    odrive.commandAxisTurns(xAxis, yAxis);
    return 0;
}

int TVC::arm(void) {
    odrive.armAxis(MotorAxis::MOTOR_AXIS_ZERO);
    odrive.armAxis(MotorAxis::MOTOR_AXIS_ONE);
    return 0;
}
