#include "Stream.hpp"

using namespace stream;

void Stream<StreamType::PacketStreamType>::CleanUsedPacket()
{
    auto packet = m_registered_packet_list.begin();

    while (packet !=m_registered_packet_list.end())
    {
        if ((*packet)->GetUsedFlag())
        {
            (*packet)->SetUpdateFlag(false);
        }
        packet++;
    }
}

void Stream<StreamType::BytesStreamType>::CleanUsedPacket()
{
    auto packet = m_registered_packet_list.begin();

    while (packet !=m_registered_packet_list.end())
    {
        if ((*packet)->GetUsedFlag())
        {
            (*packet)->SetUpdateFlag(false);
        }
        packet++;
    }
}