#include "LogStream.hpp"

using namespace stream;

void LogOutputStream::Send()
{
	auto iterator = m_registered_packet_list.begin();

	while (iterator!= m_registered_packet_list.end())
	{
		if((*iterator)->GetUpdateFlag())
		{
			dynamic_cast<LogPacketBehavior*>(*iterator)->ToStream(this);
			(*iterator)->SetUpdateFlag(false);
		}
		iterator++;
	}
}
