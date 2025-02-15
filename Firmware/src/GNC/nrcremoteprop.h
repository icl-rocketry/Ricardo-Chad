#pragma once

#include <librrc/nrcremoteactuatorbase.h>

#include <librnp/rnp_networkmanager.h>
#include <librnp/rnp_packet.h>
#include <librrc/packets/servocalibrationpacket.h>

#include <math.h>
#include <Preferences.h>

typedef uint16_t counts_t;

class NRCRemoteProp : public NRCRemoteActuatorBase<NRCRemoteProp>
{

public:    
    NRCRemoteProp(uint8_t gpio,
                    uint8_t channel,
                    RnpNetworkManager &networkmanager,
                    uint16_t no_speed = 0,
                    uint16_t max_speed = 100,
                    counts_t min_counts = counts(1130),
                    counts_t max_counts = counts(2000)
                    ): 
        NRCRemoteActuatorBase(networkmanager),
        _gpio(gpio),
        _channel(channel),
        _no_speed(no_speed),
        _max_speed(max_speed),
        _min_counts((uint16_t)min_counts),
        _max_counts((uint16_t)max_counts)
        {};

    void setup();

    /**
     * @brief Temporary implementation of reset. Drives servo to min angle position. This needs to be updated
     * to be configurable.
     * 
     */
    void turnOff();

    void goto_Speed(uint16_t speed);
    
protected:

    friend class NRCRemoteActuatorBase;
    friend class NRCRemoteBase;

    const uint8_t _gpio;
    const uint8_t _channel;
    uint16_t _default_speed;
    const uint16_t _no_speed;
    const uint16_t _max_speed;

    uint16_t _angl_lim_min;
    uint16_t _angl_lim_max;

    const uint16_t _min_counts;
    const uint16_t _max_counts;

    
    void execute_impl(packetptr_t packetptr);

    uint16_t speedtocounts(uint16_t speed);

    static constexpr int timer_width = 14;
    static constexpr int freq = 50;
    
    static constexpr int counts(int usec){
       return (int)(float(usec) / (float(1000000/freq)/float(std::pow(2,timer_width))));
    }
};