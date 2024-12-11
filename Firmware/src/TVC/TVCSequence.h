#pragma once

#include "ODriveController.h"

class TVCSequence {
public:
    enum Program {
        NONE = 0,
        PROGRAM_ONE = 1,
        PROGRAM_TWO = 2,
        PROGRAM_THREE = 3,
    };

    /**
     * @brief Construct a new TVCSequence object.
     */
    TVCSequence(Stream& serial);

    /**
     * @brief Destroy the TVCSequence object.
     */
    ~TVCSequence();

    /**
     * @brief Start a program.
     * 
     * @param program Program to start.
     */
    void startProgram(Program program);

    void calibrateAxes();

    void arm();

    void update();

private:
    void programOne();

    Program currentProgram = Program::NONE;

    ODriveController controller;
};