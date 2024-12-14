#include <libriccore/riccorelogging.h>
#include <librrc/Remote/nrcremoteservo.h>
#include <librnp/rnp_networkmanager.h>

#include "ODriveController.h"

#include "TVCController.h"

// template<RicCoreLoggingConfig::LOGGERS Conf>
// using log_impl = RicCoreLogging::log<Conf>;
// using log = log_impl<RicCoreLoggingConfig::LOGGERS::SYS>;

#define log(x) RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(x)


TVCController::TVCController(RnpNetworkManager& networkManager, Stream& serial): 
    NRCRemoteActuatorBase(networkManager),
    controller(serial, 5) {

    // Wait for signal from the ODrive
    while(!controller.available()) {
        log("\nTVC NOT FOUND\n");
    }

    log("\nTVC FOUND!\n");

    // Initialise TVC
    log("Initialising ODrive\n");
    controller.command(ODriveController::SysCommand::CLEAR_ERR);

    // Setup Brake Resistor
    // controller.writeConfig("config.enable_brake_resistor", true);
    // controller.writeConfig("config.brake_resistance", 5);

    // Battery Configs
    controller.writeConfig("config.dc_max_negative_current", -10);
    
    // Endstop Configs
    // controller.writeConfig("config.gpio5_mode", GPIO_MODE_DIGITAL);
    // controller.writeConfig("config.gpio4_mode", GPIO_MODE_DIGITAL);

    // Configure individual axis
    configure_actuator(0);
    configure_actuator(1);
}

void TVCController::configure_actuator(int actuator) {
    const std::string axis = actuator == 0 ? "axis0" : "axis1";

    controller.writeConfig(axis + ".min_endstop.config.enabled", false);
    controller.writeConfig(axis + ".max_endstop.config.enabled", false);

    // Motor Configs
    controller.writeConfig(axis + ".motor.config.current_lim", 10);
    controller.writeConfig(axis + ".motor.config.pole_pairs", 7);
    controller.writeConfig(axis + ".motor.config.torque_constant", 0.05907142857f);

    // Encoder Setup
    controller.writeConfig(axis + ".encoder.config.calib_scan_distance", 20);
    controller.writeConfig(axis + ".encoder.config.cpr", 8192);

    // Controller Setup
    controller.writeConfig(axis + ".controller.config.vel_limit", 100);
    controller.writeConfig(axis + ".controller.config.homing_speed", -2);
    controller.writeConfig(axis + ".controller.config.vel_ramp_rate", 0.5f);

    // Trapezium Trajectory Setup
    controller.writeConfig(axis + ".trap_traj.config.vel_limit", 2);
    controller.writeConfig(axis + ".trap_traj.config.accel_limit", 2);
    controller.writeConfig(axis + ".trap_traj.config.decel_limit", 2);
}

void TVCController::locked() {
    controller.position(0.5, 0.5);
}

void TVCController::programOne() {
    static uint64_t prev = millis();

    const uint64_t time = millis();

    // log("Time : " + std::to_string(time) + "\n");

    // Only send commands at <= 60Hz
    if (time - prev < 10) {
        return;
    }
    prev = time;
    // log("Sending Position at time : "+std::to_string(time)+"\n");
    // log("Error value : " + std::to_string(controller.error()) + " " + std::to_string(controller.error(ODriveController::Axis::ZERO)) + " " + std::to_string(controller.error(ODriveController::Axis::ONE)) + "\n");

    const float axis0Command = (sin(time / 300.0) / 4.0) + 0.5;
    const float axis1Command = (cos(time / 300.0) / 4.0) + 0.5;


    controller.position(axis0Command, axis1Command);
}

void TVCController::arm_base(int32_t /* arg */) {
    // check for any other errors, only arm if we are error free
    if (this->_state.getStatus() != static_cast<uint16_t>(LIBRRC::COMPONENT_STATUS_FLAGS::DISARMED)){
        return;
    }

    bool success = false;

    success |= arm_actuator(0);
    // success |= arm_actuator(1);

    if (!success) {
        return;
    }
    
    this->_state.deleteFlag(LIBRRC::COMPONENT_STATUS_FLAGS::DISARMED);
    this->_state.newFlag(LIBRRC::COMPONENT_STATUS_FLAGS::NOMINAL);
};

void TVCController::disarm_base() {
    this->_state.deleteFlag(LIBRRC::COMPONENT_STATUS_FLAGS::NOMINAL);
    this->_state.newFlag(LIBRRC::COMPONENT_STATUS_FLAGS::DISARMED);
}

void TVCController::execute_base(int32_t arg) {
    switch (arg) {
        case 0:
            currentProgram = ExecutionProgram::LOCKED;
            break;
        case 1:
            currentProgram = ExecutionProgram::PROGRAM_ONE;
            break;
        case 2:
            currentProgram = ExecutionProgram::PROGRAM_TWO;
            break;
        case 3:
            currentProgram = ExecutionProgram::PROGRAM_THREE;
            break;
        default:
            log("\nTVC : Invalid command : " + std::to_string(arg) + "\n");
            break;
    }
}

void TVCController::update() {
    switch (currentProgram) {
        case ExecutionProgram::LOCKED:
            locked();
            break;
        case ExecutionProgram::PROGRAM_ONE:
            programOne();
            break;
        case ExecutionProgram::PROGRAM_TWO:
            break;
        case ExecutionProgram::PROGRAM_THREE:
            break;
    }
};

bool TVCController::arm_actuator(int actuator) {
    const std::string axis = actuator == 0 ? "axis0" : "axis1";

    controller.calibrateAxis(actuator == 0 ? ODriveController::Axis::ZERO : ODriveController::Axis::ONE);

    controller.arm(actuator == 0 ? ODriveController::Axis::ZERO : ODriveController::Axis::ONE);

    // Ensure there are no errors
    int err = controller.readConfigInt(axis + ".error");

    if (err != 0) {
        const std::string errMsg = "TVC Fatal : error when arming axis " + std::to_string(actuator) + " with err code " + std::to_string(err) + ".\n"; 
        log(errMsg);
        return false;
    }

    return true;
}
