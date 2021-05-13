#ifndef  LOG_STREAM_HPP
#define LOG_STREAM_HPP

#include "Stream.hpp"
#include "LogPacket.hpp"
#include <fstream>

namespace stream
{
	class LogOutputStream : public Stream<StreamType::PacketStreamType>
	{
	public:
		LogOutputStream() { ; }
		~LogOutputStream() { ; }

		void Send() override;

		std::ofstream output_file_stream;
	};
}

#endif // ! LOG_STREAM_HPP
