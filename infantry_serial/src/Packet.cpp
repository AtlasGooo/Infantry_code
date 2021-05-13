#include "Stream.hpp"

using namespace stream;

PacketBehaviorProto& PacketBehaviorProto::operator<<(PacketBehaviorProto& giver)
{
    auto giver_model = giver.GetModel();

    if (giver_model->m_unit_list.size()!=m_model->m_unit_list.size())
    {
        std::cerr<<"Unit list size not equal!"<<std::endl;
        return (*this);
    }
    
    auto giver_model_iterator   = giver_model->m_unit_list.begin();
    auto giver_model_end        = giver_model->m_unit_list.end();
    auto my_model_iterator      = m_model->m_unit_list.begin();

    while (giver_model_iterator!=giver_model_end)
    {
        (*my_model_iterator)->FromStream(*giver_model_iterator);

        giver_model_iterator++;
        my_model_iterator++;
    }   
}

PacketBehaviorProto& PacketBehaviorProto::operator>>(PacketBehaviorProto& receiver)
{
    auto receiver_model = receiver.GetModel();

    if (receiver_model->m_unit_list.size()!=m_model->m_unit_list.size())
    {
        std::cerr<<"Unit list size not equal!"<<std::endl;
        return (*this);
    }
    
    auto receiver_model_iterator = receiver_model->m_unit_list.begin();
    auto receiver_model_end = receiver_model->m_unit_list.end();
    auto my_model_iterator = m_model->m_unit_list.begin();

    while (receiver_model_iterator!=receiver_model_end)
    {
        (*my_model_iterator)->ToStream(*receiver_model_iterator);

        receiver_model_iterator++;
        my_model_iterator++;
    }   
}

void PacketBehaviorProto::ToStream(PacketBehaviorProto* receiver_packet)
{
    auto receiver_unit_list_iterator = receiver_packet->m_model->m_unit_list.begin();
    auto my_unit_list_iterator = m_model->m_unit_list.begin();
    int my_unit_list_size = m_model->m_unit_list.size();

    if (receiver_packet->m_model->m_unit_list.size() == my_unit_list_size)
    {
        for (int i = 0; i < my_unit_list_size; i++)
        {
            (*receiver_unit_list_iterator)->FromStream(*my_unit_list_iterator);
            my_unit_list_iterator++;
            receiver_unit_list_iterator++;
        }
    }
}

void PacketBehaviorProto::ToStream(Stream<StreamType::BytesStreamType>* stream)
{
    auto my_unit_list_iterator = m_model->m_unit_list.begin();
    auto end = m_model->m_unit_list.end();

    while (my_unit_list_iterator!= end)
    {
        (*my_unit_list_iterator)->ToStream(&stream->m_buffer);
        my_unit_list_iterator++;
    }
}

void PacketBehaviorProto::FromStream(PacketBehaviorProto* giver_packet)
{
    auto giver_unit_list_iterator = giver_packet->m_model->m_unit_list.begin();
    auto my_unit_list_iterator = m_model->m_unit_list.begin();
    int my_unit_list_size = m_model->m_unit_list.size();

    if (giver_packet->m_model->m_unit_list.size() == my_unit_list_size)
    {
        for (int i = 0; i < my_unit_list_size; i++)
        {
            (*my_unit_list_iterator)->FromStream(*giver_unit_list_iterator);
            my_unit_list_iterator++;
            giver_unit_list_iterator++;
        }
    }
}

void PacketBehaviorProto::FromStream(Stream<StreamType::BytesStreamType>* stream)
{
    auto my_unit_list_iterator = m_model->m_unit_list.begin();
    auto end = m_model->m_unit_list.end();

    while (my_unit_list_iterator != end)
    {
        (*my_unit_list_iterator)->FromStream(&stream->m_buffer);
        my_unit_list_iterator++;
    }
}
