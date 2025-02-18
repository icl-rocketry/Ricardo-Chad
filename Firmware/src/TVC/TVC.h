/**
 * @file tvc.h
 * @author Riley Horrix (rh1122@ic.ac.uk)
 * @brief TVC Control Interface
 * @version 0.1
 * @date 2025-02-15
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <librrc/Remote/nrcremoteservo.h>

#include <librnp/rnp_networkmanager.h>

#include "tvc/odrive36.h"
#include "tvc/tvcTelemPacket.h"

class TVC : public NRCRemoteActuatorBase<TVC> {
public:
    /**
     * @brief Construct a new TVC object.
     * 
     * @param networkManager A reference to the network manager.
     */
    TVC(RnpNetworkManager& networkManager);

    /**
     * @brief Request a force output from the TVC.
     * 
     * If one of the motors has an error, then the no commands will be sent to either motor.
     * 
     * @param x Force through x axis (N).
     * @param y Force through y axis (N).
     * @param z Force through z axis (N).
     * 
     * @return int An integer error code, 0 for success.
     */
    int requestForce(float x, float y, float z);

    /**
     * @brief Arm the TVC.
     * 
     * This will arm both TVC axes.
     * 
     * @return int An integer error code, 0 for success.
     */
    int arm(void);

    /**
     * @brief Lock the TVC in its current position.
     * 
     * @return int An integer error code, 0 for success.
     */
    int lock(void);

    /**
     * @brief Set the TVC into un-powered, idle mode.
     * 
     * @return int An integer error code, 0 for success.
     */
    int idle(void);

    /**
     * @brief Disarm the TVC.
     */
    void disarm(void);

    void update(void) {
        odrv.update();
    }

    /**
     * @brief Request a control signal [0, 1] for each motor.
     * 
     * This is used for testing the motors easily.
     * 
     * @param yAxis Control signal for the y-axis.
     * @param zAxis Control signal for the z-axis.
     * 
     * @return int An integer error code, 0 for success.
     */
    int requestControl(float xAxis, float yAxis);

    /**
     * @brief Override of arm implementation for remote actuator.
     * 
     * @param arg 
     */
    void arm_base(int32_t arg);

    /**
     * @brief Override of disarm implementation for remote actuator.
     */
    void disarm_base();

    /**
     * @brief Executes a command for the TVC.
     * 
     * Commands : 
     *     0x0 - Locked State.
     *     0x1 - Execute Program.
     * 
     * @param arg The command.
     */
    void execute_base(int32_t arg);

private:
    // state_t y_axis; // In body axis coordinates
    // state_t z_axis; // In body axis coordinates

    /**
     * @brief Network manager reference.
     */
    RnpNetworkManager& networkManager;

    /**
     * @brief Telemetry packet instance;
     */
    TVCTelemPacket telemPacket;

    Odrive36 odrv = Odrive36(10.0f);
};