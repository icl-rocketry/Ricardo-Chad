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
    topProp.setup();
    bottomProp.setup();    
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

            topProp.goto_Speed(0);
            delay(1000);
            topProp.goto_Speed(1);
            delay(1000);
            topProp.goto_Speed(0);

            currentGNCState = GNCState::Armed;
 
            break;

        }
        case GNCState::Armed:
        {
           // check time since last packet is less than TIME if true, abort, if not continue 

           // for testing jun 15
           //go to angle 90 
           //wait 1 second
           //go to idle
            bottomProp.goto_Speed(0);
            delay(1000);
            bottomProp.goto_Speed(1);
            delay(1000);
            bottomProp.goto_Speed(0);

            currentGNCState = GNCState::Idle;
            break;

      
        }
        case GNCState::Abort: 
        {
            //cut power 
        }
       
    }
}