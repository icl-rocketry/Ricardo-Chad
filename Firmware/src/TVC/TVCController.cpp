#include <libriccore/riccorelogging.h>
#include <librrc/Remote/nrcremoteservo.h>
#include <librnp/rnp_networkmanager.h>

#include "ODriveController.h"

#include "TVCController.h"
#include "TVCTelemPacket.h"

// template<RicCoreLoggingConfig::LOGGERS Conf>
// using log_impl = RicCoreLogging::log<Conf>;
// using log = log_impl<RicCoreLoggingConfig::LOGGERS::SYS>;

#define log(x) RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(x)


TVCController::TVCController(RnpNetworkManager& networkManager, Stream& serial): 
    NRCRemoteActuatorBase(networkManager),
    controller(serial, 5),
    networkManager(networkManager) {

    // Wait for signal from the ODrive
    while(!controller.available()) {
        log("\nTVC NOT FOUND\n");
    }

    log("\nTVC FOUND!\n");

    checkVoltage();

    // controller.command(ODriveController::SysCommand::ERASE_CONF);

    while(!controller.available()) {
        log("\nTVC NOT FOUND\n");
    }

    // Initialise TVC
    log("Initialising ODrive\n");
    controller.command(ODriveController::SysCommand::CLEAR_ERR);

    // Setup Brake Resistor
    controller.writeConfig("config.enable_brake_resistor", true);
    controller.writeConfig("config.brake_resistance", 5);

    // Battery Configs
    controller.writeConfig("config.dc_max_negative_current", -0.1f);

    // Configure individual axis
    configure_actuator(0);
    // configure_actuator(1);

    controller.command(ODriveController::SysCommand::SAVE_CONF);

    while(!controller.available()) {
        log("\nTVC NOT FOUND\n");
    }
    log("\nTVC FOUND! - SAVED\n");

    controller.command(ODriveController::SysCommand::REBOOT);

    while(!controller.available()) {
        log("\nTVC NOT FOUND\n");
    }
    log("\nTVC FOUND! - REBOOT\n");
}

void TVCController::extendedCommandHandler_impl(const NRCPacket::NRC_COMMAND_ID commandID, packetptr_t packetptr) {
    if (static_cast<uint8_t>(commandID) == 8) {
        sendTelem();
    } else {
        NRCRemoteActuatorBase::extendedCommandHandler_impl(commandID, std::move(packetptr));
    }
}

void TVCController::configure_actuator(int actuator) {
    const std::string axis = actuator == 0 ? "axis0" : "axis1";

    // "axis0.min_endstop.config.debounce_ms": 50,
    // "axis0.min_endstop.config.enabled": true,
    // "axis0.min_endstop.config.gpio_num": 4,
    // "axis0.min_endstop.config.is_active_high": false,
    // "axis0.min_endstop.config.offset": -0.25,

    // Endstop Configs
    controller.writeConfig("config.gpio4_mode", GPIO_MODE_DIGITAL);
    controller.writeConfig(axis + ".min_endstop.config.gpio_num", 4);
    controller.writeConfig(axis + ".min_endstop.config.is_active_high ", false);
    controller.writeConfig(axis + ".min_endstop.config.offset", 2);
    controller.writeConfig(axis + ".min_endstop.config.enabled", true);
    controller.writeConfig(axis + ".max_endstop.config.enabled", false);
    controller.writeConfig("config.gpio4_mode", GPIO_MODE_DIGITAL_PULL_UP);
    controller.writeConfig(axis + ".controller.config.homing_speed", 0.25f);

    // Motor Configs
    controller.writeConfig(axis + ".motor.config.current_lim", 20);
    controller.writeConfig(axis + ".motor.config.current_lim_margin", 2);
    controller.writeConfig(axis + ".motor.config.pole_pairs", 11);
    controller.writeConfig(axis + ".motor.config.torque_constant", 0.01333871f);

    // Encoder Setup
    controller.writeConfig(axis + ".encoder.config.calib_scan_distance", 20);
    controller.writeConfig(axis + ".encoder.config.cpr", 8192);

    // Controller Setup
    controller.writeConfig(axis + ".controller.config.vel_limit", 10);
    controller.writeConfig(axis + ".controller.config.vel_ramp_rate", 1);
    controller.writeConfig(axis + ".controller.config.vel_gain", 0.016f);
    controller.writeConfig(axis + ".controller.config.pos_gain", 2);
    

    // Trapezium Trajectory Setup
    controller.writeConfig(axis + ".trap_traj.config.vel_limit", 1);
    controller.writeConfig(axis + ".trap_traj.config.accel_limit", 2);
    controller.writeConfig(axis + ".trap_traj.config.decel_limit", 2);
}

void TVCController::locked() {
    controller.position(0.5, 0.5);
}

void TVCController::programOne() {
    static uint64_t prev = millis();

    // Only send commands at <= 60Hz
    if (time - prev < 10) {
        return;
    }

    prev = time;
    // log("Sending Position at time : "+std::to_string(time)+"\n");
    int mainErr;
    int axisErr;
    int motorErr;
    int controllerErr;

    controller.fullErrors(ODriveController::Axis::ZERO, mainErr, axisErr, motorErr, controllerErr);

    log("Error values : \n\tmain       - " + std::to_string(mainErr) + 
        "\n\taxis       - " + std::to_string(axisErr) + 
        "\n\tmotor      - " + std::to_string(motorErr) + 
        "\n\tcontroller - " + std::to_string(controllerErr) + "\n");

    const float axis0Command = -1 * ((sin(time / 300.0) / 4.0) + 0.5);
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

    log("\nArming done, set to 0.5\n");
    delay(1000);
    controller.position(0.5, 0.5);
    
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
    time = millis();
    static uint64_t prev = time;

    // Query voltage at 10Hz
    if (time - prev > 100) {
        prev = time;
        checkVoltage();
    }

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
}

void TVCController::sendTelem() {
    telemPacket.odriveVoltage = odriveVoltage;

    controller.requestFeedback(ODriveController::Axis::ZERO, telemPacket.axis0Turns, telemPacket.axis0Velocity);
    controller.requestFeedback(ODriveController::Axis::ONE, telemPacket.axis1Turns, telemPacket.axis1Velocity);

    networkManager.sendPacket(telemPacket);
}

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

void TVCController::checkVoltage() {
    odriveVoltage = controller.readConfigFloat("vbus_voltage");
    if (odriveVoltage < 9.0) {
        while (true) {
            log("\nODRIVE LOW VOLTAGE\n");
        }
    }
}