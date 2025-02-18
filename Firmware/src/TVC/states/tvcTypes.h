/**
 * @file tvcTypes.h
 * @author your name (you@domain.com)
 * @brief TVC typings and flags for states.
 * @version 0.1
 * @date 2025-02-17
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

#include <libriccore/systemstatus/systemstatus.h>
#include <libriccore/fsm/state.h>
#include <libriccore/fsm/statemachine.h>

#define FLAG(n) (0x1 << n)

/**
 * @brief State and error flags for the main TVC Controller.
 */
enum class TVC_FLAGS : uint32_t {
    // State flags
    STATE_IDLE =        FLAG(0),
    STATE_ARMED =       FLAG(1),
    STATE_EXECUTING =   FLAG(2),
    STATE_ERROR =       FLAG(3),
};

using TVCStatus         = SystemStatus<TVC_FLAGS>;
using TVCState          = State<TVC_FLAGS>;
using TVCStateMachine   = StateMachine<TVC_FLAGS>;

/**
 * @brief State an error flags for a single TVC motor.
 */
enum class TVC_MOTOR_FLAGS : uint32_t {
    // State flags
    STATE_IDLE =        FLAG(0),
    STATE_ARMED =       FLAG(1),
    STATE_EXECUTING =   FLAG(2),
    STATE_ERROR =       FLAG(3),
    STATE_LOCKED =      FLAG(4)
};

using TVCMotorStatus         = SystemStatus<TVC_MOTOR_FLAGS>;
using TVCMotorState          = State<TVC_MOTOR_FLAGS>;
using TVCMotorStateMachine   = StateMachine<TVC_MOTOR_FLAGS>;