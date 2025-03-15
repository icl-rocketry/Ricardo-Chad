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
    STATE_PROGRAM_1 =   FLAG(2),
    STATE_PROGRAM_2 =   FLAG(3),
    STATE_PROGRAM_3 =   FLAG(4),
    STATE_LOCKED =      FLAG(4),
};

using TVCStatus         = SystemStatus<TVC_FLAGS>;
using TVCState          = State<TVC_FLAGS>;
using TVCStateMachine   = StateMachine<TVC_FLAGS>;