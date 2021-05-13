#include "RefereePacket.hpp"
#include "RefereeStream.hpp"

using namespace stream;

// void RefereeGameStatusPacket::FromStream(Stream<StreamType::BytesStreamType> *stream)
// {
//     uint8_t temp = 0;

//     (stream->m_buffer) >> temp >> stage_remain_time;

//     game_type.data = temp & 0x0F;
//     game_progress.data = temp >> 4;
// }

// void RefereeICRABuffDebuffZoneStatusPacket::FromStream(Stream<StreamType::BytesStreamType> *stream)
// {
//     uint8_t temp[3] = {0};
//     uint32_t temp32 = 0;

//     (stream->m_buffer) >> temp[0] >> temp[1] >> temp[2];

//     temp32 = (temp[0] << 16) + (temp[1] << 8) + temp[2];

//     F1_zone_status.data = temp32 & 1;
//     F2_zone_status.data = (temp32 >> 4) & 1;
//     F3_zone_status.data = (temp32 >> 8) & 1;
//     F4_zone_status.data = (temp32 >> 12) & 1;
//     F5_zone_status.data = (temp32 >> 16) & 1;
//     F6_zone_status.data = (temp32 >> 20) & 1;

//     F1_zone_buff_debuff_status.data = temp32 & (7);
//     F2_zone_buff_debuff_status.data = (temp32 >> 4) & 7;
//     F3_zone_buff_debuff_status.data = (temp32 >> 8) & 7;
//     F4_zone_buff_debuff_status.data = (temp32 >> 12) & 7;
//     F5_zone_buff_debuff_status.data = (temp32 >> 16) & 7;
//     F6_zone_buff_debuff_status.data = (temp32 >> 20) & 7;
// }

// void RefereeEventDataPacket::FromStream(Stream<StreamType::BytesStreamType> *stream)
// {
//     uint32_t temp = 0;

//     (stream->m_buffer) >> temp;

//     parking_apron_status.data = temp & 3;
//     energy_authorities_status.data = (temp & 12) >> 2;
//     base_virtual_shield_status.data = (temp & 16) >> 3;
// }

// void RefereeGameRobotStatusPacket::FromStream(Stream<StreamType::BytesStreamType> *stream)
// {
//     uint8_t output = 0;

//     (stream->m_buffer) >> robot_id >> robot_level >> remain_HP >> max_HP >>
//         shooter_id1_17mm_cooling_rate >> shooter_id1_17mm_cooling_limit >>
//         shooter_id1_17mm_speed_limit >> shooter_id2_17mm_cooling_rate >>
//         shooter_id2_17mm_cooling_limit >> shooter_id2_17mm_speed_limit >>
//         shooter_id1_42mm_cooling_rate >> shooter_id1_42mm_cooling_limit >>
//         shooter_id1_42mm_speed_limit >> max_chassis_power >> output;

//     mains_power_gimbal_output.data = output & (1);
//     mains_power_chassis_output.data = (output >> 1) & 1;
//     mains_power_shooter_output.data = (output >> 2) & 1;
// }

// void RefereeRobotHurtPacket::FromStream(Stream<StreamType::BytesStreamType> *stream)
// {
//     uint8_t temp = 0;

//     (stream->m_buffer) >> temp;

//     armor_id.data = temp & 0x0F;
//     hurt_type.data = temp >> 4;
// }

// void RefereeBetweenRobotPacket::FromStream(Stream<StreamType::BytesStreamType> *stream)
// {
//     data.data_length = GetDataLength()-6;
//     PacketBehaviorProto::FromStream(stream);
// }

// void RefereeBetweenRobotPacket::ToStream(Stream<StreamType::BytesStreamType> *stream)
// {
//     data.data_length = data.data.size();
//     SetDataLength(data.data_length+6);
//     RefereePacketBehavior::ToStream(stream);
// }
