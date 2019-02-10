#pragma once
#include </usr/include/eigen3/Eigen/Core>
#include <memory>
#include "crtp_packet.h"
#include "follow_me_flie_ros/RawPacket.h"

class RawPacket
{
public:
    static constexpr int maxBufferLength = 64;
    std::array<uint8_t, maxBufferLength> _data;
    uint8_t _length;

    RawPacket(CRTPPacket packet)
    {
        _data.at(0) = 1;
        copy(packet.SendableData(), 1);
        _length = packet.GetSendableDataLength();
    }

    RawPacket(uint8_t data[maxBufferLength] , uint8_t length)
    {
        copy(data,0);
        _length = length;
    }
    RawPacket() :
        _data(),
        _length(0)
    {}

    void copy(uint8_t const * const src, int offset)
    {
        for (int i = 0; i < maxBufferLength-offset; ++i)
        {
            _data.at(i+offset) = src[i];
        }
    }

};

RawPacket ConvertMsgPacketToRawPacket(const follow_me_flie_ros::RawPacketConstPtr &packet);
follow_me_flie_ros::RawPacket ConvertRawPacketToMsgPacket(const RawPacket & rawPacket);


std::ostream & operator << (std::ostream& stream, RawPacket const & packet);