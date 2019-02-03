#pragma once
#include </usr/include/eigen3/Eigen/Core>
#include <memory>
#include "crtp_packet.h"

class RawPacket
{
public:
    static constexpr int maxBufferLength = 64;
    std::array<uint8_t, maxBufferLength> _data;
    int _length;

    RawPacket(CRTPPacket packet)
    {
        _data.at(0) = 1;
        copy(packet.SendableData(), 1);
        _length = packet.GetSendableDataLength();
    }

    RawPacket(uint8_t data[maxBufferLength] , int length)
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


std::ostream & operator << (std::ostream& stream, RawPacket const & packet);