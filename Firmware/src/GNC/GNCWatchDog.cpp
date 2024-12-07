#include "GNCWatchDog.h"
void GNCWatchDog::pollPickle()
{
    SimpleCommandPacket test_command_1(3, 0);
    test_command_1.header.source_service = 10;
    test_command_1.header.source = 2;
    test_command_1.header.destination_service = 11;
    test_command_1.header.destination = 104;
    test_command_1.header.uid = 0;
    networkmanager.sendPacket(test_command_1);

}

void GNCWatchDog::killMotors()
{
    //kill both the motors by setting them to 0
    motor1.turnOff(); 
    motor2.turnOff(); 
}

void GNCWatchDog::update()
{
    //poll the pickle to get time since last test wtf does this return cuz idk 😭
    //
    try {SensorPoller.update} catch (const std::exception &e)
    {
        killMotors();
    }
}


