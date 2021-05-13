#include "PacketModel.hpp"
#include "PacketInfo.hpp"

using namespace stream;

GimbalAngleSetPacket::GimbalAngleSetPacket():PacketModelProto(GIMBAL_ANGLE_SET_PACKET_ID,GIMBAL_ANGLE_SET_PACKET_SIZE)
{
	m_unit_list.push_back(&yaw_mode);
	m_unit_list.push_back(&pitch_mode);
	m_unit_list.push_back(&yaw_angle);
	m_unit_list.push_back(&pitch_angle);
}
GimbalAngleFdbPacket::GimbalAngleFdbPacket():PacketModelProto(GIMBAL_ANGLE_FDB_PACKET_ID,GIMBAL_ANGLE_FDB_PACKET_SIZE)
{
	m_unit_list.push_back(&yaw_angle);
	m_unit_list.push_back(&pitch_angle);
}  

EnemyColorPacket::EnemyColorPacket():PacketModelProto(ENEMY_COLOR_PACKET_ID,ENEMY_COLOR_PACKET_SIZE)
{
	m_unit_list.push_back(&is_red);
}  