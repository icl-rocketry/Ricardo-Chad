#pragma once

#include "librrc/nrcremoteactuatorbase.h"
#include "librrc/nrcremoteservo.h"

#include <librnp/rnp_networkmanager.h>
#include <librnp/rnp_packet.h>

#include "LuT.h"
#include "LinearInterp.h"

#include <SiC43x.h>


class NRCThanos : public NRCRemoteActuatorBase<NRCThanos>
{

    public:

        NRCThanos(RnpNetworkManager &networkmanager,
                    uint8_t fuelServoGPIO,
                    uint8_t fuelServoChannel,
                    uint8_t oxServoGPIO,
                    uint8_t oxServoChannel,
                    uint8_t overrideGPIO,
                    uint8_t address,
                    SiC43x& Buck
                    ):
            NRCRemoteActuatorBase(networkmanager),
            _networkmanager(networkmanager),      
            _fuelServoGPIO(fuelServoGPIO),
            _fuelServoChannel(fuelServoChannel),
            _oxServoGPIO(oxServoGPIO),
            _oxServoChannel(oxServoChannel),
            _overrideGPIO(overrideGPIO),
            _address(address),
            fuelServo(fuelServoGPIO,fuelServoChannel,networkmanager,0,0,180,0,175),
            oxServo(oxServoGPIO,oxServoChannel,networkmanager,0,0,180,10,160),
            _Buck(Buck),
            // PcAngleLuT({4,14},{61.52,129.40})
            PcAngleLuT({4.2,12.0},{92.9,136.88})
            {};

        void setup();
        void update();
        void updateChamberP(float chamberP);
        bool getPollingStatus() { return _polling; };

        float getFuelAngle() { return fuel_angl_high_res; };
        float getOxAngle() { return ox_angl_high_res; };

        float getI() {return I_angle;};
        float getP() {return P_angle;};
        uint8_t getStatus(){return static_cast<uint8_t>(currentEngineState);};

    protected:
        RnpNetworkManager &_networkmanager;
        const uint8_t _fuelServoGPIO;
        const uint8_t _fuelServoChannel;
        const uint8_t _oxServoGPIO;
        const uint8_t _oxServoChannel;
        const uint8_t _overrideGPIO;

        const uint8_t _address;

        NRCRemoteServo fuelServo;
        NRCRemoteServo oxServo;

        SiC43x& _Buck;

        LuT<float,LinearInterp<float>> PcAngleLuT;

        friend class NRCRemoteActuatorBase;
        friend class NRCRemoteBase;

        void execute_impl(packetptr_t packetptr);
        // void arm_impl(packetptr_t packetptr);
        // void disarm_impl(packetptr_t packetptr);
        void override_impl(packetptr_t packetptr);
        void extendedCommandHandler_impl(const NRCPacket::NRC_COMMAND_ID commandID, packetptr_t packetptr);


        enum class EngineState : uint8_t
        {
            Default = 1<<0,
            Ignition = 1<<1,
            ShutDown = 1<<2,
            Controlled = 1<<3,
            Debug = 1<<5
        };

        EngineState currentEngineState = EngineState::Default;

        uint64_t ignitionTime;

        float _chamberP;
        float _thrust;

        bool timeFrameCheck(int64_t start_time, int64_t end_time = -1);
        bool nominalEngineOp();
        bool pValUpdated();

        void firePyro(uint32_t duration);
        void openOxFill();
        void closeOxFill();

        float nextOxAngle();
        float nextFuelAngle();
        float PcSetpoint();
        float oxAngleFF(float Pc);

        void resetVars(){
            last_demand_Pc = 0;
            m_I_err = 0;
        };

        // Ignition sequence timings from moment ignition command received
        const uint64_t pyroFires = 0;

        const uint64_t preAngleTime = 500;
        const uint64_t endOfIgnitionSeq = 1500;

        const uint16_t fuelServoPreAngle = 95;
        const uint16_t oxServoPreAngle = 70;


        uint64_t lastTimeChamberPUpdate;

        const uint32_t m_cutoffTime = 15500;
        const uint32_t m_oxDelay = 100;
        const uint32_t m_oxFillCloseTime = 14000;
       
        bool _polling = false;

        //vars related to ox fill open method
        bool oxFillClosed = false;
        uint8_t closeOxFillCalls = 0;

        const uint8_t m_oxFillService = 10;
        const uint8_t m_oxFillNode = 103;

        //vars related to the ignition command
        static constexpr uint8_t m_ingitionService = 10;
        static constexpr uint8_t m_ignitionNode = 107;
        uint8_t _ignitionCalls = 0;
        static constexpr uint8_t _ignitionCommandMaxCalls = 2;
        static constexpr uint8_t _ignitionCommandSendDelta = 50;
        uint32_t _prevFiring = 0;

        static constexpr uint32_t pressureUpdateTimeLim = 1000;

        //
        uint32_t m_nominalEntry;

        //
        float m_oxPercent = 0;
        float m_fuelPercent = 0;
        uint16_t m_oxThrottleRange = 0;
        uint16_t m_fuelThrottleRange = 0;

        float m_fuelExtra = -0.45;

        //vectors to define throttle profile from ignition
        std::vector<float> m_targetPc = {13.2,13.2,8.3,5.8,5.8,8.3,13.2,13.2};
        std::vector<uint32_t> m_testTime = {1500,6900,7500,7800,10500,10800,11700,17000};

        //controller params
        static constexpr uint16_t m_maxControlledOx = 155;
        static constexpr float K_p = 6.0;
        static constexpr float K_i = 6.0;
        float last_demand_Pc = 0;
        float m_I_err = 0;
        uint64_t m_prev_int_t = 0;
        float m_I_max = 5; 
        static constexpr uint16_t m_maxPc = 23;

        float I_angle;
        float P_angle;

        float fuel_angl_high_res;
        float ox_angl_high_res;

        float prev_err = 0;

        float lastindex = 0;
        uint32_t m_pcIndPrev = 0;
        
};