#ifndef SERIAL_STREAM_HPP
#define SERIAL_STREAM_HPP

#include "serial.h"
#include "Stream.hpp"

namespace stream
{
	class SerialOutputStream : public Stream<StreamType::BytesStreamType>
	{
	public:
		SerialOutputStream() :Stream<StreamType::BytesStreamType>(){;}
		~SerialOutputStream() { ; }

		void Send() override;
		void Serialize() override;

		void SetSerial(serial::Serial* _serial) { p_serial = _serial; }

		serial::Serial* p_serial;
	};
	
	class SerialInputStream : public Stream<StreamType::BytesStreamType>
	{
	public:
		SerialInputStream():Stream<StreamType::BytesStreamType>(){;}

		~SerialInputStream() { ; }
		
		void Receive() override;
		void Deserialize() override;

		void SetSerial(serial::Serial* _serial) { p_serial = _serial; }

	private:
		serial::Serial* p_serial;
		uint8_t m_bytes_buffer[5120];
	};
}
#endif