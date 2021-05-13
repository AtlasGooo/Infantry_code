#ifndef REFEREE_STREAM_HPP
#define REFEREE_STREAM_HPP

#include "serial.h"
#include "Stream.hpp"

namespace stream
{
	class RefereeOutputStream : public Stream<StreamType::BytesStreamType>
	{
	public:
		RefereeOutputStream() :Stream<StreamType::BytesStreamType>() { ; }
		~RefereeOutputStream() { ; }

		void Send() override;
		void Serialize() override;

		void SetSerial(serial::Serial* _serial) { p_serial = _serial; }

	private:
		serial::Serial* p_serial;
	};

	class RefereeInputStream : public Stream<StreamType::BytesStreamType>
	{
	public:
		RefereeInputStream() :Stream<StreamType::BytesStreamType>() { ; }
		~RefereeInputStream() { ; }

		void Receive() override;
		void Deserialize() override;

		void SetSerial(serial::Serial* _serial) { p_serial = _serial; }

	private:
		serial::Serial* p_serial;
		uint8_t m_bytes_buffer[5120];
	};
}

#endif