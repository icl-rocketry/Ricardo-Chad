#include <librnp/default_packets/simplecommandpacket.h>

class GNCWatchDog 
{
public:
    
    GNCWatchDog(uint32_t timeout, 
                RnpNetworkManager &networkmanager, 
                NRCRemoteProp &motor1, 
                NRCRemoteProp &motor2) : 
                timeout(timeout),
                lastUpdate(0),
                networkmanager(networkmanager),
                motor1(motor1),
                motor2(motor2)
    {
    }
    
    void update();

 /*   void update()
    {
        if (millis() - last_update > timeout)
        {
            // kill motors
        };
    };
*/
private:
    uint32_t timeout; 
    uint32_t lastUpdate;
    RnpNetworkManager &networkmanager;
    NRCRemoteProp &motor1;
    NRCRemoteProp &motor2;

    void pollPickle();
    void killMotors();

};
