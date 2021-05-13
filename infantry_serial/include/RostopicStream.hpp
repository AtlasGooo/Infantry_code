#ifndef ROSTOPIC_STREAM_HPP
#define ROSTOPIC_STREAM_HPP

#include <ros/ros.h>
#include "Stream.hpp"
#include "RostopicPacket.hpp"

namespace ros
{
    class NodeHandle;
}

namespace stream
{
    class RostopicOutputStream : public Stream<StreamType::PacketStreamType>
    {
    private:
        ros::NodeHandle *m_node_handel_ptr;

    public:
        RostopicOutputStream(ros::NodeHandle *_node_handle_ptr);
        ~RostopicOutputStream() { ; }

        void Init();
        void Send() override;
    };

    class RostopicInputStream : public Stream<StreamType::PacketStreamType>
    {
    private:
        ros::NodeHandle *m_node_handel_ptr;
        std::vector<ros::Subscriber> m_subscriber_list;

    public:
        RostopicInputStream(ros::NodeHandle *_node_handle_ptr);
        ~RostopicInputStream() { ; }

        void Init();

        RostopicPacketBehaviorProto *GetRegisteredPacket(uint16_t id);
    };
} // namespace stream

#endif
