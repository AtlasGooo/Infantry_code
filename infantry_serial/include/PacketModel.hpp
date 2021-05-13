#ifndef PACKET_MODEL_HPP
#define PACKET_MODEL_HPP

#include "Packet.hpp"

namespace stream
{
    class GimbalAngleSetPacket : public PacketModelProto
    {
    public:
        GimbalAngleSetPacket();
        ~GimbalAngleSetPacket() { ; }

        DataUnit<uint8_t> yaw_mode;
        DataUnit<uint8_t> pitch_mode;
        DataUnit<float> yaw_angle;
        DataUnit<float> pitch_angle;
    };

    class GimbalAngleFdbPacket : public PacketModelProto
    {
    public:
        GimbalAngleFdbPacket();
        ~GimbalAngleFdbPacket() { ; }

        DataUnit<float> yaw_angle;
        DataUnit<float> pitch_angle;
    };

    class EnemyColorPacket : public PacketModelProto
    {
    public:
        EnemyColorPacket();
        ~EnemyColorPacket() { ; }

        DataUnit<uint8_t> is_red;
    };
} // namespace stream

#endif