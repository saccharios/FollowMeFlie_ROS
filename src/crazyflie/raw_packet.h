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
        copy(packet.SendableData());
        _length = packet.GetSendableDataLength();
    }

    RawPacket(uint8_t data[maxBufferLength] , int length)
    {
        copy(data);
        _length = length;
    }
    RawPacket() :
        _data(),
        _length(0)
    {}

    void copy(uint8_t const * const src)
    {
        for (int i = 0; i < maxBufferLength-1; ++i)
        {
            _data.at(i+1) = src[i];
        }
    }

};


std::ostream & operator << (std::ostream& stream, RawPacket const & packet);