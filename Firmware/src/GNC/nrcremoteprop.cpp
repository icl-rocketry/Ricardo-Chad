#include <librrc/Remote/nrcremoteservo.h>
#include <librrc/Helpers/rangemap.h>

#include "esp32-hal-ledc.h"

#include <Arduino.h>
#include <Preferences.h>

#include "nrcremoteprop.h"

void NRCRemoteProp::setup(){

    ledcAttach(_gpio, freq, timer_width);  // replaces both ledcSetup + ledcAttachPin
    turnOff();
    
}


void NRCRemoteProp::execute_impl(packetptr_t packetptr)
{

    SimpleCommandPacket execute_command(*packetptr);

    goto_Speed(execute_command.arg);
}



void NRCRemoteProp::goto_Speed(uint16_t speed)
{
    if (speed > _max_speed)
        _value = _max_speed;
    else if (speed < _no_speed)
        _value = _no_speed;
    else
        _value = speed;

    ledcWrite(_gpio, speedtocounts((uint16_t)_value)); // use _gpio not _channel
}

uint16_t NRCRemoteProp::speedtocounts(uint16_t speed)
{
    return LIBRRC::rangemap<uint16_t>(speed,_no_speed,_max_speed,_min_counts,_max_counts); 
}

void NRCRemoteProp::turnOff()
{
    goto_Speed(_no_speed);
}
