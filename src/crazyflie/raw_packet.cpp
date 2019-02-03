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