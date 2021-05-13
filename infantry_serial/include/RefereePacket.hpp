#ifndef REFEREE_PACKET_HPP
#define REFEREE_PACKET_HPP

#include "PacketBehavior.hpp"
#include "PacketModel.hpp"

namespace stream
{
	// template<class model>
	// class RefereeTemplatePacket : public model, public RefereePacketBehavior
	// {
	// public:
	// 	RefereeTemplatePacket():model(), RefereePacketBehavior(this) { ; }
	// };

	// class RefereeGameStatusPacket : public GameStatusPacket, public RefereePacketBehavior
	// {
	// public:
	// 	RefereeGameStatusPacket() :GameStatusPacket(), RefereePacketBehavior(this) { ; }

	// 	void FromStream(Stream<StreamType::BytesStreamType>*) override;
	// };

	// class RefereeGameResultPacket : public GameResultPacket, public RefereePacketBehavior
	// {
	// public:
	// 	RefereeGameResultPacket() :GameResultPacket(), RefereePacketBehavior(this) { ; }
	// };

	// class RefereeGameRobotHPPacket : public GameRobotHPPacket, public RefereePacketBehavior
	// {
	// public:
	// 	RefereeGameRobotHPPacket() :GameRobotHPPacket(), RefereePacketBehavior(this) { ; }
	// };

	// class RefereeDartStatusPacket : public DartStatusPacket, public RefereePacketBehavior
	// {
	// public:
	// 	RefereeDartStatusPacket() :DartStatusPacket(), RefereePacketBehavior(this) { ; }
	// };

	// class RefereeICRABuffDebuffZoneStatusPacket : public ICRABuffDebuffZoneStatusPacket,
	// 	public RefereePacketBehavior
	// {
	// public:
	// 	RefereeICRABuffDebuffZoneStatusPacket() :ICRABuffDebuffZoneStatusPacket(),
	// 		RefereePacketBehavior(this) {
	// 		;
	// 	}

	// 	void FromStream(Stream<StreamType::BytesStreamType>*) override;
	// };

	// class RefereeEventDataPacket : public EventDataPacket, public RefereePacketBehavior
	// {
	// public:
	// 	RefereeEventDataPacket() :EventDataPacket(), RefereePacketBehavior(this) { ; }

	// 	void FromStream(Stream<StreamType::BytesStreamType>*) override;
	// };

	// class RefereeSupplyProjectileActionPacket : public SupplyProjectileActionPacket,
	// 	public RefereePacketBehavior
	// {
	// public:
	// 	RefereeSupplyProjectileActionPacket() :SupplyProjectileActionPacket(),
	// 		RefereePacketBehavior(this) {
	// 		;
	// 	}
	// };

	// class RefereeRefereePacket : public RefereePacket, public RefereePacketBehavior
	// {
	// public:
	// 	RefereeRefereePacket() :RefereePacket(), RefereePacketBehavior(this) { ; }
	// };

	// class RefereeGameRobotStatusPacket : public GameRobotStatusPacket,
	// 	public RefereePacketBehavior
	// {
	// public:
	// 	RefereeGameRobotStatusPacket() :GameRobotStatusPacket(),
	// 		RefereePacketBehavior(this) {
	// 		;
	// 	}

	// 	void FromStream(Stream<StreamType::BytesStreamType>*) override;
	// };

	// class RefereePowerHeatPacket : public PowerHeatPacket, public RefereePacketBehavior
	// {
	// public:
	// 	RefereePowerHeatPacket() :PowerHeatPacket(), RefereePacketBehavior(this) { ; }
	// };

	// class RefereeGameRobotPosPacket : public GameRobotPosPacket, public RefereePacketBehavior
	// {
	// public:
	// 	RefereeGameRobotPosPacket() :GameRobotPosPacket(), RefereePacketBehavior(this) { ; }
	// };

	// class RefereeBufferPacket : public BufferPacket, public RefereePacketBehavior
	// {
	// public:
	// 	RefereeBufferPacket() :BufferPacket(), RefereePacketBehavior(this) { ; }
	// };

	// class RefereeAerialRobotEnergyPacket : public AerialRobotEnergyPacket, public RefereePacketBehavior
	// {
	// public:
	// 	RefereeAerialRobotEnergyPacket() :AerialRobotEnergyPacket(), RefereePacketBehavior(this) { ; }
	// };

	// class RefereeRobotHurtPacket : public RobotHurtPacket, public RefereePacketBehavior
	// {
	// public:
	// 	RefereeRobotHurtPacket() :RobotHurtPacket(), RefereePacketBehavior(this) { ; }

	// 	void FromStream(Stream<StreamType::BytesStreamType>*) override;
	// };

	// class RefereeShootDataPacket : public ShootDataPacket, public RefereePacketBehavior
	// {
	// public:
	// 	RefereeShootDataPacket() :ShootDataPacket(), RefereePacketBehavior(this) { ; }
	// };

	// class RefereeBulletRemainingPacket : public BulletRemainingPacket,
	// 	public RefereePacketBehavior
	// {
	// public:
	// 	RefereeBulletRemainingPacket() :BulletRemainingPacket(),
	// 		RefereePacketBehavior(this) {
	// 		;
	// 	}
	// };

	// class RefereeDartClientPacket : public DartClientPacket, public RefereePacketBehavior
	// {
	// public:
	// 	RefereeDartClientPacket() :DartClientPacket(), RefereePacketBehavior(this) { ; }
	// };

	// class RefereeBetweenRobotPacket : public BetweenRobotPacket, public RefereePacketBehavior
	// {
	// public:
	// 	RefereeBetweenRobotPacket() :BetweenRobotPacket(), RefereePacketBehavior(this) { ; }

	// 	void FromStream(Stream<StreamType::BytesStreamType>*) override;
	// 	void ToStream(Stream<StreamType::BytesStreamType>*) override;
	// };

}

#endif
