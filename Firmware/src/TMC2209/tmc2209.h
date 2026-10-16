#pragma once

#include <librrc/Remote/nrcremoteactuatorbase.h>
#include <librrc/Remote/nrcremoteservo.h>
#include <librrc/Remote/nrcremoteptap.h>
#include <Config/services_config.h>

#include <librnp/rnp_networkmanager.h>
#include <librnp/rnp_packet.h>
#include <libriccore/fsm/statemachine.h>
#include <libriccore/riccorelogging.h>
// #include <SoftwareSerial.h>

#include "tmcTelemPacket.h"

class System;

class TMC2209 : public NRCRemoteActuatorBase<TMC2209> {
public:
    TMC2209(System& system, RnpNetworkManager &networkmanager, uint8_t stepperStep, uint8_t stepperDir):
        NRCRemoteActuatorBase(networkmanager),
        m_system(system),
        m_networkmanager(networkmanager),
        m_stepperStepPin(stepperStep),
        m_stepperDirPin(stepperDir) {}

    /**
     * @brief The setup function will setup all of the pins and do any serial
     * communication to the stepper driver.
     */
    void setup();

    /**
     * @brief The update function is only concerned with stepping the motor
     * if it is in any mode other than hold.
     */
    void update();

    void extendedCommandHandler_impl(const NRCPacket::NRC_COMMAND_ID commandID, packetptr_t packetptr);

    void execute_base(int32_t arg);

    struct Log {
        uint32_t state;
        bool armed;
    };

    const Log& getLog();

private:
    void setRetract();
    void setExtend();

    System& m_system;
    RnpNetworkManager& m_networkmanager;

    static constexpr uint64_t HalfStepDelayMs = 12;

    static constexpr uint32_t STEPPER_COMMAND_TYPE = 51;

    static constexpr uint8_t ExtendDirection = 0;
    static constexpr uint8_t RetractDirection = 1;

    enum class StepperCommandPayload: uint16_t {
        HOLD = 0,
        EXTEND = 1,
        RETRACT = 2
    };

    using StepperCommandPacket = BasicDataPacket<StepperCommandPayload, 0, STEPPER_COMMAND_TYPE>;

    StepperCommandPayload m_stepperCommand = StepperCommandPayload::HOLD;

    bool m_armed = false;

    uint64_t m_lastStepTimeMs = 0;

    Log m_log { 0 };

    TMCTelemPacket m_tmcTelem;

    // SoftwareSerial tmcSerial;
    const uint8_t m_stepperStepPin;
    const uint8_t m_stepperDirPin;
};