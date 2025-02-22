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

#include "TVC/states/tvcTypes.h"

#define FLAG(n) (0x1 << n)

/// @brief Possible status flags available in armed state.
enum class ARMED_STATUS_FLAGS: uint32_t {
    NONE =                  FLAG(0),
    CONFIGURED =            FLAG(1),
    CLOSED_LOOP_CONTROL =   FLAG(2)
};

using TVC_ARMED_STATUS = SystemStatus<ARMED_STATUS_FLAGS>;

/**
 * @brief Possible status flags available in executing state.
 * 
 * The TVC will have state::NONE when the armed status is not in 
 * state::CLOSED_LOOP_CONTROL.
 */
enum class EXECUTING_STATUS_FLAGS: uint32_t {
    NONE =      FLAG(0),
    IDLE =      FLAG(1),
    LOCKED =    FLAG(2),
    PROGRAM_0 = FLAG(3),
    PROGRAM_1 = FLAG(4),
    PROGRAM_2 = FLAG(5)
};

using TVC_EXECUTING_STATUS = SystemStatus<EXECUTING_STATUS_FLAGS>;


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

    void update(void);

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

    static constexpr float maxTurns = 20.0f;

private:
    /// @brief The underlying TVC driver.
    Odrive36 odrive = Odrive36(maxTurns);

    /// @brief Network manager reference.
    RnpNetworkManager& networkManager;

    /// @brief Telemetry packet instance.
    TVCTelemPacket telemPacket;

    /// @brief Current status of the TVC
    TVCStatus tvcStatus;

    /// @brief Current armed status of the TVC.
    TVC_ARMED_STATUS tvcArmedStatus;

    /// @brief Current executing status of the TVC.
    TVC_EXECUTING_STATUS tvcExecutingStatus;

    /// @brief Unique pointer to the current state machine.
    TVCStateMachine stateMachine;
    bool running = false;

    bool isConfigured = false;
    

    uint64_t time_execute = 0;
};