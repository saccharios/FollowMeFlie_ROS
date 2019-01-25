#include "PacketHandler.h"
#include "text_logger.h"
#include "math/types.h"
#include "crazyflie/protocol.h"

void PacketHandler::RegisterPacketToSend(CRTPPacket packet)
{
    emit RawPacketReadyToSend(packet);
}

void PacketHandler::ReceiveRawPacket(CRTPPacket packet)
{
    ProcessPacket(packet);
}
void PacketHandler::ProcessPacket(CRTPPacket packet)
{
    // Dispatch incoming packet according to port and channel
    switch(packet.GetPort() )
    {
        case Console::id:
        {
            if(packet.GetData().size() > 0)
            {
                textLogger << "Console text: ";
                for(auto const & element : packet.GetData())
                {
                    textLogger << static_cast<char>(element);
                }
                textLogger << "\n";
            }
            break;
        }

        case Logger::id:
        {
            emit NewLoggerPacket(packet);
            break;
        }

        case Commander::id:
//        std::cout << "Receiving from Commander not implemented\n";
            break;
        case CommanderGeneric::id:
//        std::cout << "Receiving from CommanderGeneric not implemented\n";
            break;
        case Debug::id:
//        std::cout << "Receiving from Debug not implemented\n";
            break;
        case Link::id:
//        std::cout << "Receiving from Link not implemented\n";
            break;
        case Parameter::id:
            emit NewParameterPacket(packet);
            break;
        default:
//        std::cout << "Receiving random bullshit: " << packet << std::endl;
            break;
    }
}
