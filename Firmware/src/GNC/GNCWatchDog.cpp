#include "GNCWatchDog.h"
#include "Arduino.h"

GNCWatchDog::GNCWatchDog(RnpNetworkManager& networkmanager,NRCRemoteProp& motor1,NRCRemoteProp& motor2):
    m_networkmanager(networkmanager),
    PIDcheck(1, GeneralConfig::pickleAddr, static_cast<uint8_t>(Services::ID::PicklePID), static_cast<uint8_t>(Services::ID::PicklePID), m_networkmanager, [](const std::string& msg){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(msg);}),
    PicklePoller(uint16_t(300), &PIDcheck),
    Motor1(motor1),
    Motor2(motor2)
    {};


void GNCWatchDog::setup(){

    PicklePoller.setup(),
    oldTime = 0;

}

void GNCWatchDog::update(){
    currentTime = millis();
    timeElapsed = currentTime - oldTime;

    if(timeElapsed > timeInterval){
   
         if (PicklePoller.update()){
            Serial.println("pickletpoller");
            Serial.println( PicklePoller.update());
             Motor1.turnOff(); 
             Motor2.turnOff();
         }
        
        //Serial.println( PicklePoller.update());
        

        // if (requestTime > 15.0){ // if no update for 15 seconds kill motors

        //     Motor1.turnOff(); 
        //     Motor2.turnOff(); 
        // }
        oldTime = currentTime; 
    }
}

