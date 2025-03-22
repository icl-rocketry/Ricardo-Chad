/**
 * @file armed.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-02-17
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

#include "TVC/states/tvcTypes.h"

#include "TVC/odrive36.h"

#include <libriccore/riccorelogging.h>

#define log(x) RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(x)

class Armed : public TVCState {
public:
    /**
      * @brief Armed state constructor.
      * 
      */
    Armed(TVCStatus& tvcStatus, Odrive36& odrive, bool shouldError): 
            State(TVC_FLAGS::STATE_DEFAULT, tvcStatus),
            tvcStatus(tvcStatus), 
            odrive(odrive), 
            shouldError(shouldError) {}

    /**
      * @brief Perform any initialization required for the state
      * 
      */
    void initialize() override {
        TVCState::initialise(tvcStatus);
    }

    /**
      * @brief Function called every update cycle, use to implement periodic actions such as checking sensors. If nullptr is returned, the statemachine will loop the state,
      * otherwise pass a new state ptr to transition to a new state.
      * 
      * The armed update loop will check the odrive for errors and transition into 
      * an error state if there is an error.
      * 
      * If the should error flag is false, then this loop will just clear errors.
      * 
      * @return std::unique_ptr<State> 
      */
    std::unique_ptr<TVCState> update() override {
        static uint64_t time_last_log = 0;
        
        if (odrive.checkErrorsAxis(Odrive36::MotorAxis::MOTOR_AXIS_ZERO) || odrive.checkErrorsAxis(Odrive36::MotorAxis::MOTOR_AXIS_ONE)) {
            uint64_t now_time = millis();
            if (now_time - time_last_log > logIntervalMs) {
                log(odrive.getError().toString());
                time_last_log = now_time;
            }

            if (shouldError) {

            } else {
                odrive.command(Odrive36::SysCommand::CLEAR_ERR);
            }
        }
    }

    /**
      * @brief Exit state actions, cleanup any files opened, save data that kinda thing.
      * 
      */
    void exit() override;

private:
    TVCStatus& tvcStatus;
    Odrive36& odrive;
    bool shouldError;

    /// @brief Send error messages every 200ms at 5Hz.
    static constexpr logIntervalMs = 200;
};