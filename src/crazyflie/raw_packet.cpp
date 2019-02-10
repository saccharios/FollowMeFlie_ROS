//
// Created by stefan on 03.02.19.
//

#include "raw_packet.h"

std::ostream & operator << (std::ostream& stream, RawPacket const & packet)
{
    for (int i = 0; i < packet._length; ++i)
    {
        stream << static_cast<int>(packet._data.at(i)) << " ";
    }
    stream << "\n";
    return stream;
}


RawPacket ConvertMsgPacketToRawPacket(const follow_me_flie_ros::RawPacketConstPtr &packet)  {
    RawPacket rawPacket;
    rawPacket._length = packet->length;
    for(int i = 0; i < rawPacket._length; ++i)
    {
        rawPacket._data.at(i) = packet->data.at(i);
    }
    return rawPacket;
}


follow_me_flie_ros::RawPacket ConvertRawPacketToMsgPacket(const RawPacket & rawPacket)  {
    follow_me_flie_ros::RawPacket packet;
    packet.length = rawPacket._length;
    for(int i = 0; i < rawPacket._length; ++i)
    {
        packet.data.at(i) = rawPacket._data.at(i);
    }
    return packet;
}