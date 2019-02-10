#include "PacketHandler.h"
#include "text_logger.h"
#include "math/types.h"
#include "crazyflie/protocol.h"
#include "raw_packet.h"

#include "ros/ros.h"
#include "follow_me_flie_ros/RawPacket.h"
void PacketHandler::RegisterPacketToSend(CRTPPacket packet)
{
    RawPacket rawPacket(packet);
    emit RawPacketReadyToSend(rawPacket);

    follow_me_flie_ros::RawPacket rawPacketMsg = ConvertRawPacketToMsgPacket(rawPacket);
    _publisherSendPackets.publish(rawPacketMsg);
}

void PacketHandler::ReceiveRawPacket(RawPacket rawPacket)
{

    _ackReceived = rawPacket._data.at(0) & 0x01;
    CRTPPacket packet(rawPacket);
    ProcessPacket(packet);
}
void PacketHandler::ProcessPacket(CRTPPacket packet)
{
    // Dispatch incoming packet according to port and channel
    switch(packet.GetPort() )
    {
        case Console::id:
        {
//            std::cout << packet;
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
//            std::cout << packet;
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
//            std::cout << packet;
            emit NewParameterPacket(packet);
            break;
        default:
//        std::cout << "Receiving random bullshit: " << packet << std::endl;
            break;
    }
}

bool PacketHandler::AckReceived() const
{
    return _ackReceived;
}

bool PacketHandler::IsUsbConnectionOk() const
{
    return _isUsbConnectionOk;
}

void PacketHandler::USBConnectionOK(bool ok)
{
    _isUsbConnectionOk = ok;
}

void PacketHandler::ReceiveRawPacketMsg(follow_me_flie_ros::RawPacketConstPtr const & packet){

    RawPacket rawPacket = ConvertMsgPacketToRawPacket(packet);
    ReceiveRawPacket(rawPacket);
}

void PacketHandler::USBConnStatus(follow_me_flie_ros::USBConnStatusConstPtr const & usbConnStatus)
{
    USBConnectionOK(usbConnStatus->USBConnStatus);
}