// #include <librrc/nrcremoteservo.h>
// #include <librrc/Helpers/rangemap.h>

// #include "esp32-hal-ledc.h"

// #include <Arduino.h>
// #include <Preferences.h>

// #include "nrcremoteprop.h"

// void NRCRemoteProp::setup(){

//     ledcSetup(_channel,freq,timer_width);
//     ledcAttachPin(_gpio,_channel);

//     turnOff(); // send servo to default position
    
// }


// void NRCRemoteProp::execute_impl(packetptr_t packetptr)
// {

//     SimpleCommandPacket execute_command(*packetptr);

//     goto_Speed(execute_command.arg);
// }



// void NRCRemoteProp::goto_Speed(uint16_t speed)
// {
//     /*Check if speed value is outside of speed limits. Would also have added checking for the min_speed and max_speed but 
//     rangemap function already has checking for that so there's no point. */

//     if (speed > _max_speed)
//     {
//         _value = _max_speed;
//     }
//     else if (speed < _no_speed)
//     {
//         _value = _no_speed;
//     }
//     else
//     {
//         _value = speed; // update new speed of motor
//     }

//     ledcWrite(_channel, speedtocounts((uint16_t)_value));
// }

// uint16_t NRCRemoteProp::speedtocounts(uint16_t speed)
// {
//     return LIBRRC::rangemap<uint16_t>(speed,_no_speed,_max_speed,_min_counts,_max_counts); 
// }

// void NRCRemoteProp::turnOff()
// {
//     goto_Speed(_no_speed);
// }