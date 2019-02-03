#include "crtp_packet.h"
#include <vector>
#include <iostream>
#include "math/types.h"
#include "text_logger.h"
#include "raw_packet.h"

template<>
float ExtractData<float>(Data const & data, int offset)
{
    constexpr unsigned int typeLength = sizeof(float);
    IntFloat bits;
    bits.int_value = 0;
    if(data.size() >= offset + typeLength)
    {
        for(unsigned int i = 0; i < typeLength; ++i)
        {
            bits.int_value |= (data.at(offset + i) << 8*i);
        }
    }
    else
    {
        textLogger << "Packet is not large enough\n";
    }
    return bits.float_value;
}

CRTPPacket::CRTPPacket(RawPacket rawPacket):
        CRTPPacket(rawPacket._data, rawPacket._length) {}

CRTPPacket::CRTPPacket(std::array<uint8_t, 64> buffer, int totalLength)
{
    // Exctract port and channel information from buffer[1]
    // TODO SF Add Port and channel checking
    auto port = static_cast<uint8_t>((buffer[1] & 0xf0) >> 4);
    auto channel = static_cast<uint8_t>(buffer[1] & 0b00000011);

    // Actual data starts at buffer[2]
    Data data;
//    textLogger << "totalLength = " << totalLength << "\n";
    for(int i = 2; i < totalLength+1; ++i)
    {
        data.push_back(buffer[i]);
    }
    CRTPPacket packet(port, channel, std::move(data));
}

CRTPPacket:: CRTPPacket(uint8_t port, uint8_t channel, Data && data) :
        _port (port),
        _channel(channel),
        _data (data)
{}


Data const & CRTPPacket::GetData() const
{
    return _data;
}

uint8_t * CRTPPacket::SendableData() const
{
    uint8_t* sendable = new uint8_t[GetSendableDataLength()]();

    // Header byte
    sendable[0] = (static_cast<int>(_port) << 4) | 0b00001100 | (static_cast<int>(_channel) & 0x03);

    // Payload
    for(std::size_t i = 0; i < _data.size(); ++i)
    {
        sendable[i+1] = _data[i];
    }
    // Finishing byte
    //sendable[_dataLength + 1] = 0x27;

    return sendable;
}

int CRTPPacket::GetSendableDataLength() const
{
    return _data.size() + 1;//2;
}

uint8_t CRTPPacket::GetPort() const
{
    return _port;
}

uint8_t CRTPPacket::GetChannel() const
{
    return _channel;
}

void CRTPPacket::Print() const
{
    textLogger << "Port = " << static_cast<int>(_port)
              << " Channel = "  << static_cast<int>(_channel)
              << " Size = " << _data.size()
              << "\n";
    PrintData(_data);
}


std::ostream & operator << (std::ostream& stream, CRTPPacket const & packet)
{
    stream    << "Port = " << static_cast<int>(packet.GetPort())
              << " Channel = "  << static_cast<int>(packet.GetChannel())
              << " Size = " << packet.GetData().size()
              << "\n";
    stream << packet.GetData() << "\n";;
    return stream;
}




