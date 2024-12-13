#pragma once

#include <librrc/Remote/nrcremoteservo.h>
#include <librnp/rnp_networkmanager.h>

#include "ODriveController.h"

class TVCController : public NRCRemoteActuatorBase<TVCController>{
public:

    enum class ExecutionProgram {
        LOCKED,
        PROGRAM_ONE,
        PROGRAM_TWO,
        PROGRAM_THREE,
    };

    /**
     * @brief Construct a new TVCController object
     * 
     * @param networkmanager To connect to the NRCRemoteActuatorBase
     */
    TVCController(RnpNetworkManager& networkmanager, Stream& serial);

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
     *     0x0 - Locked State
     *     0x1 - Program 1
     *     0x2 - Program 2
     *     0x3 - Program 3
     * 
     * @param arg The command
     */
    void execute_base(int32_t arg);

    /**
     * @brief Run a single iteration of the logic loop.
     * 
     */
    void update();

private:
    /**
     * @brief Arm a certain actuator.
     * 
     * This does all of the arming & calibration for the actuator.
     * 
     * WARNING - This command will cause the actuators to find the endstop.
     * Ensure that you are ready for the actuator to move before calling this
     * command.
     * 
     * @param actuator The actuator to control.
     * @return true Success in arming.
     * @return false Failure in arming.
     */
    bool arm_actuator(int actuator);

    /**
     * @brief Send configuration commands to the ODrive.
     * 
     * @param actuator Actuator to configure.
     */
    void configure_actuator(int actuator);

    /**
     * @brief Single iteration of the locked state.
     */
    void locked();

    /**
     * @brief Single iteration of program one.
     */
    void programOne();

    /**
     * @brief Backing TVC controller.
     */
    ODriveController controller;

    /**
     * @brief Currently running program.
     */
    ExecutionProgram currentProgram;
};