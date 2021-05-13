#ifndef SERIALPACKET_HPP
#define SERIALPACKET_HPP

#include "PacketBehavior.hpp"
#include "PacketModel.hpp"

namespace stream
{

	template<class model>
	class SerialTemplatePacket : public model, public SerialPacketBehavior
	{
	public:
		SerialTemplatePacket():model(), SerialPacketBehavior(this) { ; }
	};

}

#endif // ! SERIALPACKET_HPP