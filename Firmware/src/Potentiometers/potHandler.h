#include <librrc/Remote/nrcremotebase.h>

class potHandler : NRCRemoteBase<potHandler>
{
public:
    potHandler(std::string name, RnpNetworkManager &networkmanager) : NRCRemoteBase<potHandler>(name, networkmanager)
    {
    }

private:
    void extendedCommandHandler_impl(const NRCPacket::NRC_COMMAND_ID commandID, packetptr_t packetptr);

    void updatePotValues();
    int getPotValue(int potIndex);
    int mapPotValue(int rawValue, int minVal, int maxVal);

    // Potentiometer values
    int m_pot0Value;
    int m_pot1Value;
    // ... other potentiometer values
};
