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

#include "TVC/odrive36.h"
#include "TVC/tvcTelemPacket.h"

class TVC : public NRCRemoteActuatorBase<TVC> {
public:
    /**
     * @brief Construct a new TVC object.
     * 
     * @param networkManager A reference to the network manager.
     */
    TVC(RnpNetworkManager& networkManager);

    /// @brief Main update loop.
    void update();

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
    /**
     * @brief Network manager reference.
     */
    RnpNetworkManager& networkManager;
    
    /// @brief The current running program.
    uint32_t currentProgram = 0x0;
    
    /**
     * @brief Telemetry packet instance;
     */
    TVCTelemPacket telemPacket;
    
    /// @brief The time that the current execute command started running.
    uint64_t time_execute = 0;
    
    /// @brief Software end stop for the odrive.
    const float softwareMinEndstop = 1.0;
    
    /// @brief Software maximal turns from the endstop for the odrive.
    const float softwareMaxTurns = 10.0;
    
    /// @brief The odrive.
    Odrive36 odrv = Odrive36(softwareMaxTurns, softwareMinEndstop);
    
    /// @brief Request a telemetry packet from the TVC.
    void requestTelem();

    /**
     * @brief Lock the TVC in its current position.
     * 
     * @return int An integer error code, 0 for success.
     */
    int lock(void);
};