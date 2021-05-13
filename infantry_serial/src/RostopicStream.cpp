#include "RostopicStream.hpp"

using namespace stream;

static RostopicInputStream *input_stream_ptr = nullptr;


RostopicOutputStream::RostopicOutputStream(ros::NodeHandle *_node_handel_ptr)
    : m_node_handel_ptr(_node_handel_ptr), Stream<StreamType::PacketStreamType>()
{
}

void RostopicOutputStream::Init()
{
    auto registered_packet_iterator = m_registered_packet_list.cbegin();
    RostopicPacketBehaviorProto *packet_ptr;

    while (registered_packet_iterator != m_registered_packet_list.cend())
    {
        packet_ptr = dynamic_cast<RostopicPacketBehaviorProto *>(*registered_packet_iterator);


        packet_ptr->PublisherInit(m_node_handel_ptr);
        registered_packet_iterator++;
    }
}

void RostopicOutputStream::Send()
{
    auto registered_packet_iterator = m_registered_packet_list.cbegin();
    RostopicPacketBehaviorProto *packet_ptr;

    while (registered_packet_iterator != m_registered_packet_list.cend())
    {
        packet_ptr = dynamic_cast<RostopicPacketBehaviorProto *>(*registered_packet_iterator);

        if (packet_ptr->GetUpdateFlag())
        {
            packet_ptr->SetUpdateFlag(false);
            packet_ptr->ToStream(this);
            packet_ptr->Publish();
        }
        registered_packet_iterator++;
    }
}

RostopicInputStream::RostopicInputStream(ros::NodeHandle *_node_handle_ptr) : m_node_handel_ptr(_node_handle_ptr), Stream<StreamType::PacketStreamType>()
{
}

void RostopicInputStream::Init()
{
    auto packet = m_registered_packet_list.begin();
    RostopicPacketBehaviorProto* packet_ptr;

    while (packet!=m_registered_packet_list.end())
    {
        packet_ptr = dynamic_cast<RostopicPacketBehaviorProto*>(*packet);

        packet_ptr->SubscriberInit(m_node_handel_ptr);
        packet++;
    }
}

RostopicPacketBehaviorProto *RostopicInputStream::GetRegisteredPacket(uint16_t id)
{
    auto it = std::find_if(m_registered_packet_list.begin(), m_registered_packet_list.end(),
                           [id](PacketBehaviorProto *packet) { return id == packet->GetID(); });

    if (it != m_registered_packet_list.end())
    {
        return dynamic_cast<RostopicPacketBehaviorProto *>(*it);
    }

    return nullptr;
}
