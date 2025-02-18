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

#include "../tvcTypes.h"

class Idle : public TVCMotorState {
public:
    /**
     * @brief Idle state constructor. All states require the systemstatus object to be passed in, as well as any other system level objects required. For example, if
     * we want to control the available commands, we need to pass in the command handler from the riccoresystem.
     * 
     */
    Idle(TVCMotorStatus& DefaultInitParams);

    /**
     * @brief Perform any initialization required for the state
     * 
     */
    void initialize() override;

    /**
     * @brief Function called every update cycle, use to implement periodic actions such as checking sensors. If nullptr is returned, the statemachine will loop the state,
     * otherwise pass a new state ptr to transition to a new state.
     * 
     * @return std::unique_ptr<State> 
     */
    std::unique_ptr<TVCMotorState> update() override;

    /**
     * @brief Exit state actions, cleanup any files opened, save data that kinda thing.
     * 
     */
    void exit() override;
};