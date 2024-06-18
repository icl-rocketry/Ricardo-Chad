#include "GNC.h"
#include <math.h>
#include <Arduino.h>

#include <libriccore/commands/commandhandler.h>
#include <libriccore/riccorelogging.h>

#include "Config/services_config.h"

/**
* @brief makes, checks, and runs states for GNC chad
*/

void GNC::setup()
{
    externalServo.setup();
    internalServo.setup();    
}


void GNC::update() 
{

    switch (currentGNCState)
    {

        case GNCState::Idle:
        {
            //make code so nothing runs ie disarmed 
            //set propTop power = 0 
            //set propBot = 0

            //for testing jun 15
            //set angle to 0 
            //wait 1 second 
            //go to arm
            sleep(1000);
            externalServo.goto_Angle(0);
            sleep(1000);
            externalServo.goto_Angle(90);

            currentGNCState = GNCState::Arm;
            break;
 

        }
        case GNCState::Arm:
        {
           // check time since last packet is less than TIME if true, abort, if not continue 

           // for testing jun 15
           //go to angle 90 
           //wait 1 second
           //go to idle
           externalServo.goto_Angle(0);
           sleep(1000);
           currentGNCState = GNCState::Idle;
           break;

      
        }
        case GNCState::Abort: 
        {
            //cut power 
        }
       
    }
}