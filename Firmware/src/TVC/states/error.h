/**
 * @file error.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-02-17
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

#include "TVC/state/tvcTypes.h"

#include "TVC/odrive36.h"

class Error : public TVCState {
public:
    /**
      * @brief Error state constructor.
      * 
      */
    Error(TVCStatus& tvcStatus, Odrive36& odrive): State(TVC_FLAGS::STATE_DEFAULT, tvcStatus), tvcStatus(tvcStatus), odrive(odrive) {}

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
      * 
      * @return std::unique_ptr<State> 
      */
    std::unique_ptr<TVCState> update() override {

    }

    /**
      * @brief Exit state actions, cleanup any files opened, save data that kinda thing.
      * 
      */
    void exit() override;

private:
    TVCStatus& tvcStatus;
    Odrive36& odrive;
};