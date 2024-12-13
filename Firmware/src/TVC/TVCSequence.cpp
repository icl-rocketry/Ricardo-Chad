#include "TVCSequence.h"

#include <libriccore/riccorelogging.h>
#include "ODriveEnums.h"

// #define log(x) RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(x)
// template<RicCoreLoggingConfig::LOGGERS Conf>
// using log_tem = RicCoreLogging::log<Conf>;

// #define log(x) log_tem<RicCoreLoggingConfig::LOGGERS::SYS>(x);

TVCSequence::TVCSequence(Stream& serial): controller(serial, 10) {
    // Wait until TVC connected
    while(!controller.available()) {
        delay(100);
    }

    // Initialise TVC
    // log("Initialising ODrive\n");
    // controller.command(ODriveController::SysCommand::ERASE_CONF);

    // Wait for initialisation
    delay(5000);

    // Clear Errors
    controller.command(ODriveController::SysCommand::CLEAR_ERR);

    // log("Setting up ODrive\n");

    // Setup Brake Resistor
    // controller.writeConfig("config.enable_brake_resistor", true);
    // controller.writeConfig("config.brake_resistance", 5);

    // Battery Configs
    controller.writeConfig("config.dc_max_negative_current", -10);

    // controller.writeConfig("config.gpio5_mode", GPIO_MODE_DIGITAL);
    // controller.writeConfig("config.gpio4_mode", GPIO_MODE_DIGITAL);
    controller.writeConfig("axis0.min_endstop.config.enabled", false);
    controller.writeConfig("axis0.max_endstop.config.enabled", false);

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

    // log("Finished configuration\n");
}

TVCSequence::~TVCSequence() {}


void TVCSequence::arm() {
    // controller.arm();
}


void TVCSequence::startProgram(Program program) {
    currentProgram = program;
}

void TVCSequence::programOne() {
    // controller.command(ODriveController::SysCommand::CLEAR_ERR);
    const uint64_t time = millis();
    static int it = 0;
    
    static float time_t = 0;
    time_t += PI / 10;

    // Command is between 0.25 and 0.75
    const float axis0Command = (sin(time_t) / 2) + 0.5;
    const float axis1Command = (cos(time) / 2) + 0.5;

    // if (it++ % 100 == 0) {
    //     float position;
    //     float vel;
    //     controller.requestFeedback(ODriveController::Axis::ZERO, position, vel);

    //     const std::string message = "\n\np " + std::to_string(position) + " v " + std::to_string(vel) + "\n";
    //     log(message);
    //     const std::string command = "command : " + std::to_string(axis0Command) + "\n";
    //     log(command);
    //     const std::string status = "Status of controller : " + std::string(controller.status() ? "t" : "f") + "\n\n";
    //     log(status);
    //     controller.printDebug();
    // }

    // controller.position(axis0Command, axis1Command);
    if (it++ % 2 == 0) {
        controller.position(axis0Command, 0);
    }
}

void TVCSequence::update() {
    programOne();

    // switch (currentProgram) {
    //     case PROGRAM_ONE:
    //         programOne();
    //         break;
    //     default:
    //         break;
    // }
}
