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

#include "tvc/odrive36.h"

#include "tvc/states/tvcTypes.h"

class Default : public TVCMotorState {
public:
    /**
     * @brief Default state constructor. 
     */
    Default(Odrive36& odrv);

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

private: 
    /// @brief Reference to the motor driver.
    Odrive36& odrive;
};