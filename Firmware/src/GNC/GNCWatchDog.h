// #include <librnp/default_packets/simplecommandpacket.h>

// class GNCWatchDog 
// {
// public:
//     GNCWatchDog(uint32_t timeout, 
//                 RnpNetworkManager &networkmanager, 
//                 NRCRemoteProp &motor1, 
//                 NRCRemoteProp &motor2) : 
//                 networkmanager(networkmanager),
//                 motor1(motor1),
//                 motor2(motor2)
//     {
//     }

//     void update()
//     {
//         if (millis() - last_update > timeout)
//         {
//             // If the motors are not moving
//         };
//     };

// private:
//     uint32_t timeout;
//     uint32_t last_update;
//     RnpNetworkManager &networkmanager;
//     NRCRemoteProp &motor1;
//     NRCRemoteProp &motor2;

//     void pollPickle();

// };
