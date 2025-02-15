#pragma once

#include <librrc/Remote/nrcremoteservo.h>

#include <librnp/rnp_networkmanager.h>

#include "TVCTelemPacket.h"

class TVC : public NRCRemoteActuatorBase<TVC> {
public:
    /**
     * @brief Construct a new TVC object.
     * 
     * @param networkmanager A reference to the network manager.
     */
    TVC(RnpNetworkManager& networkmanager);

    /**
     * @brief Request a force output from the TVC.
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
     * @brief Lock the TVC in it's current position.
     * 
     * @return int An integer error code, 0 for success.
     */
    int lock(void);

    /**
     * @brief Set the TVC into idle mode.
     * 
     * @return int An integer error code, 0 for success.
     */
    int idle(void);
private:
    // state_t y_axis; // In body axis coordinates
    // state_t z_axis; // In body axis coordinates

    /**
     * @brief Network manager reference.
     * 
     */
    RnpNetworkManager& networkManager;

    /**
     * @brief Telemetry packet instance;
     * 
     */
    TVCTelemPacket telemPacket;
};