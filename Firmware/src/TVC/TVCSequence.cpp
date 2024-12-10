#include "TVCSequence.h"

#include <libriccore/riccorelogging.h>
#include "ODriveEnums.h"

#define log(x) RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(x)

TVCSequence::TVCSequence(Stream& serial): controller(serial, 100, 50) {
    // Wait until TVC connected
    while(!controller.status()) {
        delay(100);
    }

    // Initialise TVC
    log("Initialising ODrive");
    // controller.command(ODriveController::SysCommand::ERASE_CONF);

    // Wait for initialisation
    delay(5000);

    // Clear Errors
    controller.command(ODriveController::SysCommand::CLEAR_ERR);

    log("Setting up ODrive");

    // Setup Brake Resistor
    // controller.writeConfig("config.enable_brake_resistor", true);
    // controller.writeConfig("config.brake_resistance", 5);

    // Battery Configs
    controller.writeConfig("config.dc_max_negative_current", -10);

    // controller.writeConfig("config.gpio5_mode", GPIO_MODE_DIGITAL);
    // controller.writeConfig("config.gpio4_mode", GPIO_MODE_DIGITAL);
    controller.writeConfig("min_endstop.config.enabled", false);
    controller.writeConfig("max_endstop.config.enabled", false);

    // Motor Configs
    controller.writeConfig("axis0.motor.config.current_lim", 10);
    controller.writeConfig("axis0.motor.config.pole_pairs", 7);
    controller.writeConfig("axis0.motor.config.torque_constant", 0.05907142857f);

    // Encoder Setup
    controller.writeConfig("axis0.encoder.config.calib_scan_distance", 20);
    controller.writeConfig("axis0.encoder.config.cpr", 8192);

    // Controller Setup
    controller.writeConfig("axis0.controller.config.vel_limit", 100);
    controller.writeConfig("axis0.controller.config.homing_speed", -2);
    controller.writeConfig("axis0.controller.config.vel_ramp_rate", 0.5f);

    // Trapezium Trajectory Setup
    controller.writeConfig("axis0.trap_traj.config.vel_limit", 2);
    controller.writeConfig("axis0.trap_traj.config.accel_limit", 2);
    controller.writeConfig("axis0.trap_traj.config.decel_limit", 2);

    // Endstop Setup
    // controller.writeConfig("min_endstop.config.gpio_num", 5);
    // controller.writeConfig("min_endstop.config.is_active_high", true);

    log("Finished configuration");
}

TVCSequence::~TVCSequence() {}

void TVCSequence::calibrateAxes() {
    controller.calibrateAxis(ODriveController::Axis::ZERO);
    // controller.calibrateAxis(ODriveController::Axis::ZERO);
}


void TVCSequence::startProgram(Program program) {
    currentProgram = program;
}

void TVCSequence::programOne() {
    // controller.command(ODriveController::SysCommand::CLEAR_ERR);
    const uint64_t time = (uint64_t) micros();

    // Command is between 0.25 and 0.75
    const float axis0Command = (sin(time) / 4) + 0.25;
    const float axis1Command = (cos(time) / 4) + 0.25;

    controller.position(axis0Command, axis1Command);
}

void TVCSequence::update() {
    switch (currentProgram) {
        case PROGRAM_ONE:
            programOne();
            break;
        default:
            break;
    }
}
