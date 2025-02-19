/**
 * @file armed.h
 * @author Riley Horrix (rh1122@ic.ac.uk)
 * @brief 
 * @version 0.1
 * @date 2025-02-17
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

#include "../tvcTypes.h"

class Executing : public TVCState {
public:
    /**
      * @brief Executing state constructor.
      * 
      */
    Executing();

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
    std::unique_ptr<TVCState> update() override;

    /**
      * @brief Exit state actions, cleanup any files opened, save data that kinda thing.
      * 
      */
    void exit() override;
};