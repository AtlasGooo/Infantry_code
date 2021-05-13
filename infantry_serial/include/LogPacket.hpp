#ifndef  LOGPACKET_HPP
#define LOGPACKET_HPP

#include "PacketBehavior.hpp"
#include "PacketModel.hpp"

namespace stream
{
	template <class Model>
	class LogTemplatePacket : public Model, public LogPacketBehavior
	{
	public:
		LogTemplatePacket(std::string name): Model(),LogPacketBehavior(this,name){;}
		~LogTemplatePacket(){}
	};


}

#endif // ! LOGPACKET_HPP
